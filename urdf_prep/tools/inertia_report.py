"""Per-link mass, inertia sanity (positive definite + triangle inequality, which Gazebo requires) and total mass."""
import sys
import numpy as np
from urdf_parser_py.urdf import URDF

robot = URDF.from_xml_string(open(sys.argv[1], 'rb').read())
total = 0.0
for l in robot.links:
    if l.inertial is None:
        continue
    i = l.inertial.inertia
    I = np.array([[i.ixx, i.ixy, i.ixz], [i.ixy, i.iyy, i.iyz], [i.ixz, i.iyz, i.izz]])
    ev = np.linalg.eigvalsh(I)
    ok = ev.min() > 0 and all(ev[a] + ev[b] >= ev[c] * (1 - 1e-6) for a, b, c in ((0, 1, 2), (0, 2, 1), (1, 2, 0)))
    total += l.inertial.mass
    print(f'{l.name:24s} {l.inertial.mass:8.3f} kg  principal {ev.round(6)}  {"ok" if ok else "INVALID"}')
print(f'TOTAL {total:.2f} kg')
