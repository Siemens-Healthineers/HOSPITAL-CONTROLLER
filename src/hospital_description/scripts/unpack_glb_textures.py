#!/usr/bin/env python3
# Copyright (c) 2026 Siemens Healthineers AG
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Unpack embedded textures from .glb files into RViz-friendly .gltf files.

RViz's mesh loader (assimp + Ogre) has two limitations this tool works around:

1. It cannot read textures embedded inside a binary .glb (empty texture path ->
   yellow/black "missing texture" checker).
2. It only renders the *base color* texture. Normal / metallic-roughness /
   occlusion / emissive maps are not used, and when a material's ONLY image is a
   normal map, RViz still ends up with a broken texture unit -> the same
   yellow/black checker instead of falling back to the solid base color.

So this tool rewrites each ``<name>.glb`` as a glTF "separate" set that contains
*only* base-color textures; every other texture reference is stripped and the
now-orphaned images are pruned:

    <name>.gltf        JSON scene, references the .bin and any base-color .png
    <name>.bin         the original binary buffer (geometry)
    <name>_texN.png    each base-color image, now an external file

Materials that had no base-color texture become solid-color (baseColorFactor)
materials, which RViz renders correctly.

Run it whenever the source .glb files change (e.g. after re-exporting from
Blender):

    python3 scripts/unpack_glb_textures.py meshes/visual

It only reads .glb and writes new files next to them; the .glb originals are
left untouched. Delete them afterwards: the repository keeps only the generated
.gltf, .bin and .png files, and ``*.glb`` is git-ignored.
"""
from __future__ import annotations

import json
from pathlib import Path
import struct
import sys

_MIME_EXT = {'image/png': '.png', 'image/jpeg': '.jpg'}


def _read_glb(path: Path) -> tuple[dict, bytes]:
    data = path.read_bytes()
    magic, _version, _length = struct.unpack_from('<III', data, 0)
    if magic != 0x46546C67:  # "glTF"
        raise ValueError(f'{path} is not a binary glTF (.glb)')
    offset = 12
    gltf_json: dict | None = None
    bin_chunk = b''
    while offset < len(data):
        chunk_len, chunk_type = struct.unpack_from('<II', data, offset)
        offset += 8
        chunk = data[offset:offset + chunk_len]
        offset += chunk_len
        if chunk_type == 0x4E4F534A:  # "JSON"
            gltf_json = json.loads(chunk)
        elif chunk_type == 0x004E4942:  # "BIN\0"
            bin_chunk = chunk
    if gltf_json is None:
        raise ValueError(f'{path}: no JSON chunk')
    return gltf_json, bin_chunk


def _strip_non_basecolor(gltf: dict) -> None:
    """
    Remove every texture reference RViz can't use, leaving baseColorTexture.

    Then prune the textures/images arrays down to those still referenced and
    remap all indices so the result is a valid, minimal glTF.
    """
    materials = gltf.get('materials', [])
    for mat in materials:
        mat.pop('normalTexture', None)
        mat.pop('occlusionTexture', None)
        mat.pop('emissiveTexture', None)
        pbr = mat.get('pbrMetallicRoughness')
        if pbr:
            pbr.pop('metallicRoughnessTexture', None)

    textures = gltf.get('textures', [])
    images = gltf.get('images', [])

    # Which textures are still referenced (only baseColorTexture survives)?
    kept_tex_ids = sorted(
        {
            mat['pbrMetallicRoughness']['baseColorTexture']['index']
            for mat in materials
            if mat.get('pbrMetallicRoughness', {}).get('baseColorTexture')
        }
    )
    tex_remap = {old: new for new, old in enumerate(kept_tex_ids)}
    new_textures = [textures[i] for i in kept_tex_ids]

    # Drop the optional sampler reference so we can safely remove the samplers
    # array (a dangling "sampler" index makes the glTF invalid and RViz silently
    # drops the whole mesh). RViz uses the default sampler anyway.
    for tex in new_textures:
        tex.pop('sampler', None)

    # Repoint material baseColorTexture indices.
    for mat in materials:
        bct = mat.get('pbrMetallicRoughness', {}).get('baseColorTexture')
        if bct is not None:
            bct['index'] = tex_remap[bct['index']]

    # Prune images down to those referenced by the kept textures.
    kept_img_ids = sorted(
        {t['source'] for t in new_textures if 'source' in t}
    )
    img_remap = {old: new for new, old in enumerate(kept_img_ids)}
    new_images = [images[i] for i in kept_img_ids]
    for tex in new_textures:
        if 'source' in tex:
            tex['source'] = img_remap[tex['source']]

    if new_textures:
        gltf['textures'] = new_textures
    else:
        gltf.pop('textures', None)
    if new_images:
        gltf['images'] = new_images
    else:
        gltf.pop('images', None)
    gltf.pop('samplers', None)  # samplers only matter with textures; safe to drop


def unpack(glb_path: Path) -> bool:
    """Convert one .glb. Returns True if it kept any base-color texture."""
    gltf, blob = _read_glb(glb_path)
    stem = glb_path.stem

    # Remove stale textures from a previous run before regenerating.
    for old in glb_path.parent.glob(f'{stem}_tex*.png'):
        old.unlink()

    _strip_non_basecolor(gltf)

    buffer_views = gltf.get('bufferViews', [])
    images = gltf.get('images', [])

    had_texture = False
    for idx, image in enumerate(images):
        if image.get('uri') is not None or 'bufferView' not in image:
            continue  # already external
        had_texture = True
        bv = buffer_views[image['bufferView']]
        start = bv.get('byteOffset', 0)
        end = start + bv['byteLength']
        ext = _MIME_EXT.get(image.get('mimeType', ''), '.png')
        tex_name = f'{stem}_tex{idx}{ext}'
        (glb_path.parent / tex_name).write_bytes(blob[start:end])
        image['uri'] = tex_name
        image.pop('bufferView', None)
        image.pop('mimeType', None)

    # Externalize the single binary buffer so the .gltf is self-describing.
    if gltf.get('buffers'):
        bin_name = f'{stem}.bin'
        (glb_path.parent / bin_name).write_bytes(blob)
        gltf['buffers'][0]['uri'] = bin_name
        gltf['buffers'][0]['byteLength'] = len(blob)

    (glb_path.parent / f'{stem}.gltf').write_text(json.dumps(gltf))
    return had_texture


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print(__doc__)
        return 2
    folder = Path(argv[1])
    glbs = sorted(folder.glob('*.glb'))
    if not glbs:
        print(f'No .glb files in {folder}')
        return 1
    textured = 0
    for glb in glbs:
        if unpack(glb):
            textured += 1
            print(f'  base-color: {glb.name} -> {glb.stem}.gltf')
        else:
            print(f'  solid:      {glb.name} -> {glb.stem}.gltf')
    print(f'Converted {len(glbs)} meshes ({textured} kept base-color textures).')
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv))
