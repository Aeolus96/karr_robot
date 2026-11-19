"""Print world position and axis directions of selected frames (zero joint angles).  Usage: frame_report.py karr.urdf [frame ...]"""
import sys
import numpy as np
from urdf_parser_py.urdf import URDF

robot = URDF.from_xml_string(open(sys.argv[1], 'rb').read())


def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


T = {robot.get_root(): np.eye(4)}
while len(T) < len(robot.links):
    for j in robot.joints:
        if j.parent in T and j.child not in T:
            M = np.eye(4)
            M[:3, :3] = rpy(*(j.origin.rpy or [0, 0, 0]))
            M[:3, 3] = j.origin.xyz or [0, 0, 0]
            T[j.child] = T[j.parent] @ M

names = sys.argv[2:] or sorted(T)
fmt = lambda v: '(' + ' '.join(f'{x:+.3f}' for x in v) + ')'
for n in names:
    M = T[n]
    print(f'{n:24s} pos {fmt(M[:3, 3])}  +X->{fmt(M[:3, 0])}  +Z->{fmt(M[:3, 2])}')
