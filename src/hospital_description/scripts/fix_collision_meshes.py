#!/usr/bin/env python3
# Copyright (c) 2026 Siemens Healthineers AG
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Repair over-merged collision STL meshes.

Several collision meshes were exported from Blender with the *entire* scene
merged in, so every one of them contains a copy of all the doors, the bed and
the whole CT scanner. In RViz (Collision Enabled) this shows up as many "ghost"
copies of each part frozen at the default joint pose.

This script rebuilds each broken collision mesh so it only contains the
geometry that actually belongs to its own object. It does that by:

  1. reading the object's *visual* glTF world-space bounding box (the visual
     meshes are correct, one object each),
  2. splitting the merged collision mesh into connected components,
  3. keeping only the components whose bounding box lies inside the visual
     bounding box (plus a small margin).

Run with --apply to overwrite the meshes (originals are backed up to
meshes/collision_backup/). Without --apply it only reports what it would do.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MESH_DIR = os.path.normpath(os.path.join(HERE, '..', 'meshes'))
COLLISION_DIR = os.path.join(MESH_DIR, 'collision')
VISUAL_DIR = os.path.join(MESH_DIR, 'visual')
BACKUP_DIR = os.path.join(MESH_DIR, 'collision_backup')

# Meshes whose collision export contains the whole scene and must be rebuilt.
BROKEN = [
    'hospital_bed',
    'ct_base',
    'ct_gantry',
    'ct_cantilever_support',
    'ct_carriage_base',
    'ct_tabletop',
    'ct_shroud_01',
    'ct_shroud_02',
    'ct_shroud_03',
    'ct_shroud_04',
    'ct_shroud_05',
]

MARGIN = 0.10  # metres of slack around the visual bbox


# --------------------------- STL (binary) I/O ---------------------------
def load_stl(path):
    with open(path, 'rb') as f:
        f.read(80)
        n = struct.unpack('<I', f.read(4))[0]
        raw = np.frombuffer(f.read(n * 50), dtype=np.uint8).reshape(n, 50)
    normals = raw[:, 0:12].copy().view('<f4').reshape(n, 3)
    verts = raw[:, 12:48].copy().view('<f4').reshape(n, 3, 3)
    return normals, verts


def save_stl(path, normals, verts):
    n = len(verts)
    with open(path, 'wb') as f:
        f.write(b'\x00' * 80)
        f.write(struct.pack('<I', n))
        buf = np.zeros((n, 50), dtype=np.uint8)
        buf[:, 0:12] = normals.astype('<f4').view(np.uint8).reshape(n, 12)
        buf[:, 12:48] = verts.astype('<f4').view(np.uint8).reshape(n, 36)
        f.write(buf.tobytes())


# --------------------------- glTF bbox ---------------------------
def _node_matrix(node):
    if 'matrix' in node:
        return np.array(node['matrix'], dtype=float).reshape(4, 4).T
    T = np.eye(4)
    R = np.eye(4)
    S = np.eye(4)
    if 'translation' in node:
        T[:3, 3] = node['translation']
    if 'scale' in node:
        S[:3, :3] = np.diag(node['scale'])
    if 'rotation' in node:
        x, y, z, w = node['rotation']
        R[:3, :3] = np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ]
        )
    return T @ R @ S


def gltf_world_bbox(path):
    g = json.load(open(path))
    nodes = g.get('nodes', [])
    mins, maxs = [], []

    def walk(idx, parent):
        node = nodes[idx]
        M = parent @ _node_matrix(node)
        if 'mesh' in node:
            for prim in g['meshes'][node['mesh']]['primitives']:
                acc = g['accessors'][prim['attributes']['POSITION']]
                lo = np.array(acc['min'])
                hi = np.array(acc['max'])
                corners = np.array(
                    [
                        [x, y, z, 1]
                        for x in (lo[0], hi[0])
                        for y in (lo[1], hi[1])
                        for z in (lo[2], hi[2])
                    ]
                )
                w = (M @ corners.T).T[:, :3]
                mins.append(w.min(0))
                maxs.append(w.max(0))
        for c in node.get('children', []):
            walk(c, M)

    scene = g['scenes'][g.get('scene', 0)]
    for r in scene['nodes']:
        walk(r, np.eye(4))
    return np.min(mins, 0), np.max(maxs, 0)


# --------------------------- connected components ---------------------------
def connected_components(verts):
    """Return an array of component ids (one per triangle) using shared verts."""
    n = len(verts)
    quant = np.round(verts.reshape(-1, 3) * 1e4).astype(np.int64)
    _, vid = np.unique(quant, axis=0, return_inverse=True)
    vid = vid.reshape(n, 3)

    parent = np.arange(vid.max() + 1)

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    for tri in vid:
        union(tri[0], tri[1])
        union(tri[1], tri[2])

    tri_root = np.array([find(tri[0]) for tri in vid])
    return tri_root


def part_bbox(name):
    """Visual world bbox of a part, expanded by MARGIN, plus its volume."""
    lo, hi = gltf_world_bbox(os.path.join(VISUAL_DIR, f'{name}.gltf'))
    lo = lo - MARGIN
    hi = hi + MARGIN
    vol = float(np.prod(hi - lo))
    return lo, hi, vol


def repair(name, boxes, apply):
    coll = os.path.join(COLLISION_DIR, f'{name}.stl')
    # Always rebuild from the pristine original if we have one backed up, so the
    # script is safe to re-run.
    backup = os.path.join(BACKUP_DIR, f'{name}.stl')
    source = backup if os.path.exists(backup) else coll

    lo, hi, _ = boxes[name]
    normals, verts = load_stl(source)
    roots = connected_components(verts)

    keep = np.zeros(len(verts), dtype=bool)
    for root in np.unique(roots):
        mask = roots == root
        cv = verts[mask].reshape(-1, 3)
        cmin, cmax = cv.min(0), cv.max(0)
        if not (np.all(cmin >= lo) and np.all(cmax <= hi)):
            continue
        # This component fits inside our box. Only claim it if no other part
        # has a *tighter* (smaller) box that also fully contains it -- that
        # sibling is the real owner (e.g. the tabletop nested in the cantilever).
        _, _, my_vol = boxes[name]
        owned = True
        for other, (olo, ohi, ovol) in boxes.items():
            if other == name:
                continue
            if ovol < my_vol and np.all(cmin >= olo) and np.all(cmax <= ohi):
                owned = False
                break
        if owned:
            keep[mask] = True

    kept = int(keep.sum())
    kv = verts[keep].reshape(-1, 3)
    bbox = '(empty)'
    if kept:
        d = kv.max(0) - kv.min(0)
        bbox = 'size(%.2f x %.2f x %.2f) origin[%.2f,%.2f,%.2f]' % (
            d[0], d[1], d[2], *kv.min(0)
        )
    print(f'{name:28s} {len(verts):7d} -> {kept:6d} tris  {bbox}')

    if apply:
        if kept == 0:
            print(f'  !! refusing to write empty mesh for {name}')
            return
        os.makedirs(BACKUP_DIR, exist_ok=True)
        if not os.path.exists(backup):
            shutil.copy2(coll, backup)
        save_stl(coll, normals[keep], verts[keep])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--apply', action='store_true', help='overwrite meshes (with backup)')
    args = ap.parse_args()
    boxes = {name: part_bbox(name) for name in BROKEN}
    print(f"{'mesh':28s} {'tris':>7s}    {'kept':>6s}")
    for name in BROKEN:
        repair(name, boxes, args.apply)
    if not args.apply:
        print('\nDry run only. Re-run with --apply to write the repaired meshes.')


if __name__ == '__main__':
    sys.exit(main())
