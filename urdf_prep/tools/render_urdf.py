"""Render URDF meshes (front + side + top views) to a PNG using forward kinematics at zero joint angles."""
import struct, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from urdf_parser_py.urdf import URDF

urdf_path, mesh_dir, out = sys.argv[1:4]
robot = URDF.from_xml_string(open(urdf_path,'rb').read())


def rpy_to_mat(rpy):
    r, p, y = rpy
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def tf(origin):
    T = np.eye(4)
    if origin is not None:
        T[:3, :3] = rpy_to_mat(origin.rpy or [0, 0, 0])
        T[:3, 3] = origin.xyz or [0, 0, 0]
    return T


world = {robot.get_root(): np.eye(4)}
changed = True
while changed:
    changed = False
    for j in robot.joints:
        if j.parent in world and j.child not in world:
            world[j.child] = world[j.parent] @ tf(j.origin)
            changed = True


def load_stl(path):
    d = open(path, 'rb').read()
    n = struct.unpack('<I', d[80:84])[0]
    arr = np.frombuffer(d[84:84 + n * 50], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    return arr['v'].reshape(-1, 3)


fig, axes = plt.subplots(1, 3, figsize=(18, 6))
views = [('Side (X-Z)', 0, 2), ('Front (Y-Z)', 1, 2), ('Top (X-Y)', 0, 1)]
cmap = plt.get_cmap('tab20')
for i, link in enumerate(robot.links):
    if not link.visual or link.name not in world:
        continue
    fn = link.visual.geometry.filename.split('/')[-1]
    v = load_stl(f'{mesh_dir}/{fn}')
    v = v[:: max(1, len(v) // 4000)]
    T = world[link.name] @ tf(link.visual.origin)
    w = (T[:3, :3] @ v.T).T + T[:3, 3]
    for ax, (title, a, b) in zip(axes, views):
        ax.scatter(w[:, a], w[:, b], s=0.3, color=cmap(i % 20), label=link.name if ax is axes[0] else None)
for ax, (title, a, b) in zip(axes, views):
    ax.set_title(title); ax.set_aspect('equal'); ax.grid(True, lw=0.3)
    if b == 2:
        ax.axhline(0, color='k', lw=1)
axes[0].legend(markerscale=20, fontsize=7, loc='upper left')
plt.tight_layout()
plt.savefig(out, dpi=80)
print('root:', robot.get_root())

