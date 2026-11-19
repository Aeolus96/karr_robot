#!/usr/bin/env python3
"""Convert the SolidWorks (sw_urdf_exporter, ROS 1) export of K.A.R.R. into the ROS 2 karr_description package.

Re-run after every SolidWorks re-export. Pure Python stdlib, so it runs on Windows, macOS, Linux or in the container:
    python3 urdf_prep/tools/convert_sw_export.py                       # defaults below
    python3 urdf_prep/tools/convert_sw_export.py <export_dir> <out_pkg_dir>
Defaults: <export_dir> = urdf_prep/karr_description_ros1, <out_pkg_dir> = karr_description (repo root).
Full check (container):  ./dev.sh verify   (Windows: dev.ps1 verify)

Generates urdf/karr.urdf, config/footprint.yaml and meshes/. Package files, launch and rviz files are left alone.
All hand-tuned values live in the CONFIG section below: edit them here, never in the generated URDF.
"""
import math
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

PKG = 'karr_description'
ROBOT = 'karr'
HALF_PI = '1.5708'

# ===================================== CONFIG =========================================================
# ---- Materials (assigned by link name; SolidWorks export colours are ignored) ----
MATERIALS = {
    'body_blue':   '0 0.3686 0.7216 0.5',   # blue #005EB8 / RGB 0,94,184, 50 % translucent
    'aluminum':    '0.78 0.80 0.82 1',
    'rubber':      '0.10 0.10 0.10 1',
    'gps_white':   '0.95 0.95 0.95 1',
    'lidar_grey':  '0.20 0.20 0.22 1',
    'camera_grey': '0.25 0.25 0.25 1',      # same as Stereolabs' own ZED model colour
}


def material_for(link):
    if link == 'base_link':
        return 'body_blue'
    if link.endswith('_steering_link'):
        return 'aluminum'
    if link.endswith('_wheel_link'):
        return 'rubber'
    if link.endswith('_gps_link'):
        return 'gps_white'
    if link.startswith('livox'):
        return 'lidar_grey'
    if link.endswith('_zed_camera_link'):
        return 'camera_grey'
    return None


# ---- Masses (kg). Placeholders until parts can be weighed (robot under construction); total ~100 kg. ----
# Inertia tensors are the SolidWorks ones (correct shape for the geometry) scaled to these masses.
MASS_STEERING = 4.0      # each steering module
MASS_WHEEL = 4.0         # each wheel (Segway 350 W hub motor)
MASS_GPS = 0.3           # each GNSS antenna
MASS_LIVOX = 0.265       # Livox Mid-360 datasheet
MASS_ZED = 0.1           # each ZED X One S (approx.)
# ZED inertia from a solid box of the camera body (the export's camera inertials are unreliable: one came out 0).
ZED_BODY_SIZE = (0.038, 0.025, 0.025)       # m, x y z
ZED_BODY_CENTER = '-0.0052 0 0.0125'        # m, body centre in the camera link frame
MASS_TOTAL_TARGET = 100.0
# base_link: SolidWorks inertial computed for a 100 kg body (first export); scaled to whatever mass is left.
BASE_INERTIAL_100KG = dict(xyz='0.011368 2.2931E-06 0.43149',
                           ixx=13.7, ixy=-0.00020798, ixz=-1.0087, iyy=19.462, iyz=-0.00014075, izz=23.711)

# ---- Joint limits ----
STEERING_AXIS = '0 0 1'          # REP-103: positive steering angle turns the wheel left (CCW seen from above)
STEERING_EFFORT = 17.0           # Nm, steering motor peak (6 Nm continuous)
STEERING_LIMIT = 165.0           # deg each way from straight ahead: 330 deg total, a hard stop so the wiring can't wind up
WHEEL_EFFORT = 30.0              # Nm, estimate for the 350 W hub motor at 46 V (>=15 Nm proven on 20 deg ramp @146 kg)
WHEEL_RADIUS = 0.1225            # m, from the wheel mesh
WHEEL_WIDTH = 0.0762             # m
MAX_SPEED = 6 * 0.44704          # m/s, 6 mph software/safety limit
WHEEL_VELOCITY = MAX_SPEED / WHEEL_RADIUS   # rad/s (~21.9)

# ---- Footprint / collision envelope (measured from the exported footprint_link mesh, Nov 2026) ----
FOOTPRINT_X = 0.9906             # m, length
FOOTPRINT_Y = 0.7874             # m, width
FOOTPRINT_CORNER_R = 0.0843      # m, rounded corner radius
FOOTPRINT_TOP = 1.0668           # m, top of envelope (above GPS antennas)
BODY_BOX_BOTTOM = 0.1323         # m, chassis underside: clears 3 in obstacles between the wheels
FOOTPRINT_ARC_TOL = 0.01         # m, max deviation of the Nav2 polygon from the rounded corners

# ---- Sensors ----
# Livox Mid-360(S): point-cloud origin O is 47.0 mm above the mounting face; IMU at (11.0, 23.29, -44.12) mm from O
# (Livox Mid-360 user manual). livox_ros_driver2 stamps cloud + IMU with frame_id "livox_frame" by default.
LIVOX_ORIGIN_Z = 0.047
LIVOX_IMU_XYZ = '0.011 0.02329 -0.04412'
# ZED X One S: exported link origin = centre of the 4 bottom mounting holes, X = optical axis, Z up, which is the
# wrapper's <camera_name>_camera_link. zed-ros2-wrapper (camera_model:=zedxones) publishes the rest below it.
ZED_RENAME = {'left_zedxones': 'left_zed_camera',
              'center_zedxones': 'center_zed_camera',
              'right_zedxones': 'right_zed_camera'}
# ======================================================================================================


def el(tag, parent=None, **attrib):
    attrib = {k: str(v) for k, v in attrib.items()}
    return ET.Element(tag, attrib) if parent is None else ET.SubElement(parent, tag, attrib)


def fixed_joint(name, parent, child, xyz='0 0 0', rpy='0 0 0'):
    j = el('joint', name=name, type='fixed')
    el('origin', j, xyz=xyz, rpy=rpy)
    el('parent', j, link=parent)
    el('child', j, link=child)
    return j


def g(v):
    return f'{v:.6g}'


def set_inertial(link, mass, xyz=None, tensor=None):
    """Set mass; scale the existing tensor to the new mass unless an explicit (tensor, ref_mass) is given."""
    inertial = link.find('inertial')
    m_el, i_el, o_el = inertial.find('mass'), inertial.find('inertia'), inertial.find('origin')
    if tensor is None:
        exported = float(m_el.get('value'))
        if exported <= 0:
            sys.exit(f'ERROR: {link.get("name")} has zero mass in the export; give it an explicit inertia in CONFIG')
        scale = mass / exported
        tensor = {k: float(i_el.get(k)) * scale for k in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz')}
    if xyz is not None:
        o_el.set('xyz', xyz)
    m_el.set('value', g(mass))
    for k, v in tensor.items():
        i_el.set(k, g(v))


def footprint_polygon():
    hx, hy, r = FOOTPRINT_X / 2, FOOTPRINT_Y / 2, FOOTPRINT_CORNER_R
    seg = max(1, math.ceil((math.pi / 2) / (2 * math.acos(1 - FOOTPRINT_ARC_TOL / r))))
    poly = []
    for cx, cy, start in ((hx - r, hy - r, 0), (-hx + r, hy - r, 90), (-hx + r, -hy + r, 180), (hx - r, -hy + r, 270)):
        for k in range(seg + 1):
            a = math.radians(start + 90 * k / seg)
            poly.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return poly


def main(export_dir, out_dir):
    export_dir, out_dir = Path(export_dir), Path(out_dir)
    src_urdf = next((export_dir / 'urdf').glob('*.urdf'))
    old_pkg = export_dir.name
    (export_dir / 'COLCON_IGNORE').touch()   # raw ROS 1 export must never be built by colcon
    root = ET.parse(src_urdf).getroot()
    root.set('name', ROBOT)

    # --- ZED: exported camera links are the mount points -> rename to the wrapper's <name>_camera_link -------
    for old, new in ZED_RENAME.items():
        for l in root.findall('link'):
            if l.get('name') == f'{old}_link':
                l.set('name', f'{new}_link')
        for j in root.findall('joint'):
            if j.get('name') == old:
                j.set('name', f'{new}_joint')
            if j.find('child').get('link') == f'{old}_link':
                j.find('child').set('link', f'{new}_link')

    # --- drop an exported footprint_link (envelope is defined in CONFIG instead) -----------------------------
    for l in root.findall('link'):
        if l.get('name') == 'footprint_link':
            root.remove(l)
    for j in root.findall('joint'):
        if j.find('child').get('link') == 'footprint_link':
            root.remove(j)

    links = {l.get('name'): l for l in root.findall('link')}
    joints = {j.get('name'): j for j in root.findall('joint')}

    # --- mesh paths -------------------------------------------------------------------------------------------
    for mesh in root.iter('mesh'):
        mesh.set('filename', mesh.get('filename').replace(f'package://{old_pkg}/', f'package://{PKG}/'))

    # --- joints -----------------------------------------------------------------------------------------------
    for name, j in joints.items():
        if j.get('type') == 'fixed':                       # drop meaningless <axis xyz="0 0 0"/>
            for a in j.findall('axis'):
                j.remove(a)
        elif name.endswith('_steering'):
            j.find('axis').set('xyz', STEERING_AXIS)
            j.find('limit').set('effort', g(STEERING_EFFORT))
            j.find('limit').set('lower', g(round(-math.radians(STEERING_LIMIT), 4)))
            j.find('limit').set('upper', g(round(math.radians(STEERING_LIMIT), 4)))
        elif name.endswith('_wheel'):
            # exported wheel frames have the axle along Z: rotate so the axle is +Y (positive = drive forward)
            j.find('origin').set('rpy', f'-{HALF_PI} 0 0')
            j.find('limit').set('effort', g(WHEEL_EFFORT))
            j.find('limit').set('velocity', g(round(WHEEL_VELOCITY, 2)))

    # --- masses / inertias ------------------------------------------------------------------------------------
    for name, l in links.items():
        if name.endswith('_steering_link'):
            set_inertial(l, MASS_STEERING)
        elif name.endswith('_wheel_link'):
            set_inertial(l, MASS_WHEEL)
        elif name.endswith('_gps_link'):
            # mirrored part: exported COM carries a +0.3048 m Y offset; the mesh is centred on the frame
            x, _, z = l.find('inertial/origin').get('xyz').split()
            set_inertial(l, MASS_GPS, xyz=f'{x} 0 {z}')
        elif name.startswith('livox'):
            set_inertial(l, MASS_LIVOX)
        elif name.endswith('_zed_camera_link'):
            sx, sy, sz = ZED_BODY_SIZE
            k = MASS_ZED / 12
            set_inertial(l, MASS_ZED, xyz=ZED_BODY_CENTER,
                         tensor=dict(ixx=k * (sy**2 + sz**2), ixy=0, ixz=0, iyy=k * (sx**2 + sz**2), iyz=0,
                                     izz=k * (sx**2 + sy**2)))
    others = sum(float(l.find('inertial/mass').get('value')) for n, l in links.items() if n != 'base_link')
    base_mass = MASS_TOTAL_TARGET - others
    ref = BASE_INERTIAL_100KG
    set_inertial(links['base_link'], base_mass, xyz=ref['xyz'],
                 tensor={k: ref[k] * base_mass / 100.0 for k in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz')})

    # --- collision: one body box (footprint envelope, raised); wheel cylinders; nothing else ---------------
    for name, l in links.items():
        for c in l.findall('collision'):
            l.remove(c)
        if name == 'base_link':
            c = el('collision', l)
            el('origin', c, xyz=f'0 0 {g((BODY_BOX_BOTTOM + FOOTPRINT_TOP) / 2)}', rpy='0 0 0')
            el('box', el('geometry', c), size=f'{g(FOOTPRINT_X)} {g(FOOTPRINT_Y)} {g(FOOTPRINT_TOP - BODY_BOX_BOTTOM)}')
        elif name.endswith('_wheel_link'):
            c = el('collision', l)
            el('origin', c, xyz='0 0 0', rpy='0 0 0')     # wheel frame: axle along local Z
            el('cylinder', el('geometry', c), radius=g(WHEEL_RADIUS), length=g(WHEEL_WIDTH))

    # --- materials --------------------------------------------------------------------------------------------
    for name, l in links.items():
        vis = l.find('visual')
        if vis is None:
            continue
        for m in vis.findall('material'):
            vis.remove(m)
        mname = material_for(name)
        if mname:
            el('material', vis, name=mname)
        else:
            print(f'WARNING: no material rule for {name}, left uncoloured')

    # --- rebuild document: materials, base_footprint, exported tree, sensor frames -------------------------
    body = list(root)
    for child in body:
        root.remove(child)
    for mname, rgba in MATERIALS.items():
        el('color', el('material', root, name=mname), rgba=rgba)

    # REP-105 root. base_link origin is already on the ground (wheel centres at z = wheel radius).
    el('link', root, name='base_footprint')
    root.append(fixed_joint('base_footprint_joint', 'base_footprint', 'base_link'))
    for child in body:
        root.append(child)

    if 'livox_base_link' in links:
        el('link', root, name='livox_frame')
        root.append(fixed_joint('livox_frame_joint', 'livox_base_link', 'livox_frame', xyz=f'0 0 {LIVOX_ORIGIN_Z}'))
        el('link', root, name='livox_imu_link')
        root.append(fixed_joint('livox_imu_joint', 'livox_frame', 'livox_imu_link', xyz=LIVOX_IMU_XYZ))

    # --- write URDF -------------------------------------------------------------------------------------------
    ET.indent(root, space='  ')
    header = (
        '<?xml version="1.0" encoding="utf-8"?>\n'
        '<!-- K.A.R.R. robot description. GENERATED by urdf_prep/tools/convert_sw_export.py from the SolidWorks\n'
        f'     sw_urdf_exporter output ({old_pkg}). Do not edit by hand: change the script and re-run it. -->\n'
    )
    (out_dir / 'urdf').mkdir(parents=True, exist_ok=True)
    out = out_dir / 'urdf' / f'{ROBOT}.urdf'
    out.write_text(header + ET.tostring(root, encoding='unicode') + '\n', encoding='utf-8')

    # --- meshes -----------------------------------------------------------------------------------------------
    mesh_out = out_dir / 'meshes'
    mesh_out.mkdir(exist_ok=True)
    for stl in mesh_out.glob('*.STL'):
        stl.unlink()
    for stl in (export_dir / 'meshes').glob('*.STL'):
        if stl.stem != 'footprint_link':
            shutil.copy2(stl, mesh_out / stl.name)

    # --- Nav2 footprint ---------------------------------------------------------------------------------------
    pts = ', '.join(f'[{x:.3f}, {y:.3f}]' for x, y in footprint_polygon())
    fp_yaml = out_dir / 'config' / 'footprint.yaml'
    fp_yaml.parent.mkdir(exist_ok=True)
    fp_yaml.write_text(
        '# GENERATED by urdf_prep/tools/convert_sw_export.py: rounded-rectangle robot outline in base_footprint.\n'
        '# Use in Nav2 local_costmap / global_costmap params:  footprint: "<value below>"\n'
        f'footprint: "[{pts}]"\n', encoding='utf-8')

    total = sum(float(l.find('inertial/mass').get('value')) for l in links.values())
    print(f'wrote {out}  ({len(root.findall("link"))} links, {len(root.findall("joint"))} joints, '
          f'{total:.1f} kg, base_link {base_mass:.1f} kg)')


if __name__ == '__main__':
    repo = Path(__file__).resolve().parents[2]
    if len(sys.argv) == 1:
        main(repo / 'urdf_prep' / 'karr_description_ros1', repo / PKG)
    elif len(sys.argv) == 3:
        main(sys.argv[1], sys.argv[2])
    else:
        sys.exit(__doc__)
