#!/usr/bin/env python3
"""Assemble the robot's visual meshes (all joints at zero) into one small ASCII STL and embed it in README.md.

GitHub renders a ```stl code block as an interactive 3D viewer, so this puts a rotatable K.A.R.R. on the repo page.
Meshes are simplified by vertex clustering to keep the README small.

    python3 urdf_prep/tools/make_preview_stl.py                 # default: ~3000 triangles, updates README.md
    python3 urdf_prep/tools/make_preview_stl.py --tris 4000 --out docs/karr_preview.stl

Needs only numpy. Re-run after the robot model changes.
"""
import argparse
import math
import re
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
BEGIN, END = '<!-- preview-stl:begin -->', '<!-- preview-stl:end -->'


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
    return np.concatenate(parts)


def to_ascii(tris):
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    g = lambda a, d=4: ' '.join(f'{x:.{d}g}' for x in a)  # noqa: E731
    out = ['solid karr']
    for t, nn in zip(tris, n):
        out.append(f'facet normal {g(np.round(nn, 2) + 0.0, 3)}\nouter loop\nvertex {g(t[0])}\nvertex {g(t[1])}\nvertex {g(t[2])}\n'
                   'endloop\nendfacet')
    out.append('endsolid karr')
    return '\n'.join(out) + '\n'


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--urdf', type=Path, default=REPO / 'karr_description/urdf/karr.urdf')
    ap.add_argument('--tris', type=int, default=3000, help='target triangle count')
    ap.add_argument('--out', type=Path, help='write a .stl file instead of updating README.md')
    a = ap.parse_args()

    tris = assemble(a.urdf, a.urdf.parent.parent / 'meshes')
    tris *= 1000.0  # mm: whole numbers keep the text short
    lo, hi = 1.0, 200.0  # bisect on grid cell size (mm)
    for _ in range(25):
        cell = math.sqrt(lo * hi)
        lo, hi = (cell, hi) if len(cluster(tris, cell)) > a.tris else (lo, cell)
    small = cluster(tris, hi)
    text = to_ascii(np.round(small, 0))
    print(f'{len(tris)} -> {len(small)} triangles, cell {hi:.1f} mm, {len(text) / 1e3:.0f} kB')

    if a.out:
        a.out.write_text(text)
        return
    readme = REPO / 'README.md'
    s = readme.read_text(encoding='utf-8')
    block = f'{BEGIN}\n```stl\n{text}```\n{END}'
    if BEGIN not in s:
        raise SystemExit(f'README.md has no {BEGIN} ... {END} markers')
    readme.write_text(re.sub(re.escape(BEGIN) + '.*?' + re.escape(END), lambda _: block, s, flags=re.S),
                      encoding='utf-8')
    print('updated README.md')


if __name__ == '__main__':
    main()
