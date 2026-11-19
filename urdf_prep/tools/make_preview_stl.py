#!/usr/bin/env python3
"""Assemble the robot's visual meshes (all joints at zero) into one binary STL: media/karr.stl.

GitHub shows .stl files in a repo with an interactive 3D viewer, and the README links to it. Chassis, steering modules
and wheels keep their full detail; only very dense meshes (Livox, ZED cameras: 20k-76k triangles each for parts a few
cm across) are simplified by vertex clustering.

    python3 urdf_prep/tools/make_preview_stl.py                  # -> media/karr.stl
    python3 urdf_prep/tools/make_preview_stl.py --max-part 2000 --out /tmp/karr.stl

Needs only numpy. Re-run after the robot model changes.
"""
import argparse
import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def origin_tf(el):
    T = np.eye(4)
    o = el.find('origin') if el is not None else None
    if o is not None:
        T[:3, :3] = rpy_matrix(*map(float, o.get('rpy', '0 0 0').split()))
        T[:3, 3] = list(map(float, o.get('xyz', '0 0 0').split()))
    return T


def load_stl(path):
    d = path.read_bytes()
    n = struct.unpack('<I', d[80:84])[0]
    tri = np.frombuffer(d[84:84 + n * 50], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    return tri['v'].reshape(-1, 3, 3).astype(np.float64)


def cluster(tris, cell):
    """Vertex-clustering simplification: snap vertices to a grid, merge, drop collapsed triangles."""
    v = tris.reshape(-1, 3)
    keys = np.floor(v / cell).astype(np.int64)
    _, inv = np.unique(keys, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    sums = np.zeros((inv.max() + 1, 3))
    np.add.at(sums, inv, v)
    rep = sums / np.bincount(inv)[:, None]
    f = inv.reshape(-1, 3)
    f = f[(f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])]
    if len(f):
        _, first = np.unique(np.sort(f, axis=1), axis=0, return_index=True)
        f = f[np.sort(first)]
    return rep[f]


def assemble(urdf, mesh_dir):
    root = ET.parse(urdf).getroot()
    world = {}
    children = {j.find('child').get('link') for j in root.findall('joint')}
    world[next(l.get('name') for l in root.findall('link') if l.get('name') not in children)] = np.eye(4)
    joints = root.findall('joint')
    while len(world) < len(root.findall('link')):
        for j in joints:
            p, c = j.find('parent').get('link'), j.find('child').get('link')
            if p in world and c not in world:
                world[c] = world[p] @ origin_tf(j)
    parts = []
    for link in root.findall('link'):
        vis = link.find('visual')
        mesh = vis.find('geometry/mesh') if vis is not None else None
        if mesh is None:
            continue
        T = world[link.get('name')] @ origin_tf(vis)
        t = load_stl(mesh_dir / mesh.get('filename').split('/')[-1])
        parts.append(t @ T[:3, :3].T + T[:3, 3])
    return parts


def write_binary(tris, path):
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(tris), dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    rec['n'], rec['v'] = n, tris.reshape(-1, 9)
    with open(path, 'wb') as f:
        f.write(b'K.A.R.R. (Keystone Autonomous Resource Rover)'.ljust(80, b' '))
        f.write(struct.pack('<I', len(tris)))
        f.write(rec.tobytes())


def simplify(t, max_tris):
    if len(t) <= max_tris:
        return t
    size = float(np.max(t.reshape(-1, 3).max(0) - t.reshape(-1, 3).min(0)))
    lo, hi = size * 1e-4, size * 0.2      # bisect on grid cell size
    for _ in range(30):
        cell = math.sqrt(lo * hi)
        lo, hi = (cell, hi) if len(cluster(t, cell)) > max_tris else (lo, cell)
    return cluster(t, hi)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--urdf', type=Path, default=REPO / 'karr_description/urdf/karr.urdf')
    ap.add_argument('--max-part', type=int, default=4000, help='simplify any part above this many triangles')
    ap.add_argument('--out', type=Path, default=REPO / 'media/karr.stl')
    a = ap.parse_args()
    parts = assemble(a.urdf, a.urdf.parent.parent / 'meshes')
    tris = np.concatenate([simplify(t, a.max_part) for t in parts])
    a.out.parent.mkdir(parents=True, exist_ok=True)
    write_binary(tris, a.out)
    print(f'{sum(len(t) for t in parts)} -> {len(tris)} triangles, {a.out.stat().st_size / 1e6:.1f} MB -> {a.out}')


if __name__ == '__main__':
    main()
