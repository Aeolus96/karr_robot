"""Render K.A.R.R. animations for the README with Blender (Cycles), straight from the URDF and full-quality meshes.

    blender -b -P media/render_karr.py -- turntable             # 360 deg studio turntable -> media/frames/turntable/
    blender -b -P media/render_karr.py -- mission               # island mission loop      -> media/frames/mission/
    blender -b -P media/render_karr.py -- mission --still 40,120 # test frames (PNG) instead of the whole sequence
    blender    -P media/render_karr.py -- mission --no-render   # open the scene in the Blender UI to look around
    bash media/make_gifs.sh                                     # frames -> media/*.gif

Options: --res 960x540  --fps 24  --samples 32  --still N[,N...]  --frames A-B  --no-render  --cpu
GPU: Metal on macOS, CUDA on Linux/WSL (OptiX is tried last; it fails in WSL). KARR_DEVICE=CUDA|OPTIX|... forces one.
Everything is procedural: the robot comes from karr_description, its steering angles and wheel speeds from
karr_control's kinematics. Set pieces live in karr_world.py, the mission story and shots in karr_mission.py.
Look and camera rules: docs/design/README.md. Both animations loop perfectly.
"""
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import bpy
import numpy as np
from mathutils import Euler, Matrix, Vector

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'karr_control'))
from karr_control.kinematics import Module, module_commands  # noqa: E402,F401  (used by karr_mission)

URDF = REPO / 'karr_description' / 'urdf' / 'karr.urdf'
MESHES = REPO / 'karr_description' / 'meshes'
MODULES = [Module('fl', 0.3556, 0.2286), Module('fr', 0.3556, -0.2286),
           Module('bl', -0.3556, 0.2286), Module('br', -0.3556, -0.2286)]
WHEEL_R = 0.1225


# ===================================== arguments =====================================================================
def parse_args():
    argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
    a = {'scene': 'turntable', 'res': (960, 540), 'fps': 24, 'samples': 32, 'still': None, 'render': True,
         'gpu': True, 'frames': None}
    it = iter(argv)
    for x in it:
        if x in ('turntable', 'mission'):
            a['scene'] = x
        elif x == '--res':
            a['res'] = tuple(int(v) for v in next(it).split('x'))
        elif x == '--fps':
            a['fps'] = int(next(it))
        elif x == '--samples':
            a['samples'] = int(next(it))
        elif x == '--still':
            a['still'] = [int(v) for v in next(it).split(',')]
        elif x == '--frames':
            a['frames'] = tuple(int(v) for v in next(it).split('-'))
        elif x == '--no-render':
            a['render'] = False
        elif x == '--cpu':
            a['gpu'] = False
    return a


# ===================================== helpers =======================================================================
def rpy_matrix(r, p, y):
    return Euler((r, p, y), 'XYZ').to_matrix().to_4x4()


def origin_matrix(el):
    o = el.find('origin') if el is not None else None
    if o is None:
        return Matrix.Identity(4)
    xyz = [float(v) for v in o.get('xyz', '0 0 0').split()]
    rpy = [float(v) for v in o.get('rpy', '0 0 0').split()]
    return Matrix.Translation(xyz) @ rpy_matrix(*rpy)


def material(name, color, rough=0.5, metal=0.0, coat=0.0, alpha=1.0, emission=None):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes.get('Principled BSDF')
    b.inputs['Base Color'].default_value = (*color, 1.0)
    b.inputs['Roughness'].default_value = rough
    b.inputs['Metallic'].default_value = metal
    if 'Coat Weight' in b.inputs:
        b.inputs['Coat Weight'].default_value = coat
    b.inputs['Alpha'].default_value = alpha
    if emission:
        b.inputs['Emission Color'].default_value = (*emission[0], 1.0)
        b.inputs['Emission Strength'].default_value = emission[1]
    return m


def srgb(hexstr):
    c = [int(hexstr[i:i + 2], 16) / 255 for i in (0, 2, 4)]
    return tuple(x / 12.92 if x <= 0.04045 else ((x + 0.055) / 1.055) ** 2.4 for x in c)


def link_obj(obj):
    bpy.context.scene.collection.objects.link(obj)
    return obj


def empty(name, parent=None):
    e = link_obj(bpy.data.objects.new(name, None))
    e.empty_display_size = 0.05
    e.parent = parent
    return e


def smooth(obj, angle=0.6):
    me = obj.data
    me.shade_smooth()
    if hasattr(me, 'set_sharp_from_angle'):
        me.set_sharp_from_angle(angle=angle)


def mesh_from(name, verts, faces, mat=None):
    me = bpy.data.meshes.new(name)
    me.from_pydata(verts, [], faces)
    me.update()
    obj = link_obj(bpy.data.objects.new(name, me))
    if mat:
        obj.data.materials.append(mat)
    return obj


def look_at(cam, target):
    d = Vector(target) - cam.location
    cam.rotation_mode = 'QUATERNION'
    cam.rotation_quaternion = d.to_track_quat('-Z', 'Y')


def key(obj, *paths, frame):
    for p in paths:
        obj.keyframe_insert(p, frame=frame)


# ===================================== scene setup ===================================================================
def reset_scene(a):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.context.preferences.edit.keyframe_new_interpolation_type = 'LINEAR'
    sc = bpy.context.scene
    sc.render.engine = 'CYCLES'
    sc.render.resolution_x, sc.render.resolution_y = a['res']
    sc.render.resolution_percentage = 100
    sc.render.fps = a['fps']
    sc.cycles.samples = a['samples']
    sc.cycles.use_denoising = True
    sc.cycles.max_bounces = 4
    sc.render.use_motion_blur = True
    sc.render.motion_blur_shutter = 0.5
    sc.render.image_settings.file_format = 'PNG'
    for look in ('AgX - Medium High Contrast', 'Medium High Contrast'):
        try:
            sc.view_settings.look = look
            break
        except TypeError:
            pass
    if a['gpu']:
        prefs = bpy.context.preferences.addons['cycles'].preferences
        import os
        order = ('METAL', 'CUDA', 'HIP', 'ONEAPI', 'OPTIX')
        if os.environ.get('KARR_DEVICE'):
            order = (os.environ['KARR_DEVICE'],)
        for dev_type in order:
            try:
                prefs.compute_device_type = dev_type
                prefs.get_devices()
                if any(d.type != 'CPU' for d in prefs.devices):
                    for d in prefs.devices:
                        d.use = True
                    sc.cycles.device = 'GPU'
                    break
            except TypeError:
                continue
    return sc


# ===================================== robot =========================================================================
ROBOT_MATS = {}   # original export-style materials; karr_world.restyle_robot() swaps in the design-system look


def robot_materials():
    ROBOT_MATS.update({
        'body': material('karr_body', srgb('005EB8'), rough=0.28, metal=0.2, coat=0.6),
        'alu': material('karr_aluminum', (0.72, 0.74, 0.77), rough=0.3, metal=1.0),
        'rubber': material('karr_rubber', (0.025, 0.025, 0.028), rough=0.75),
        'white': material('karr_white', (0.85, 0.85, 0.85), rough=0.35, coat=0.3),
        'sensor': material('karr_sensor', (0.04, 0.045, 0.05), rough=0.25, coat=0.5),
    })


def mat_for(link):
    if link == 'base_link':
        return ROBOT_MATS['body']
    if link.endswith('_steering_link'):
        return ROBOT_MATS['alu']
    if link.endswith('_wheel_link'):
        return ROBOT_MATS['rubber']
    if link.endswith('_gps_link'):
        return ROBOT_MATS['white']
    return ROBOT_MATS['sensor']


def build_robot():
    """Robot as empties mirroring the URDF tree. Returns (root, {joint_name: rotating empty})."""
    robot_materials()
    root_xml = ET.parse(URDF).getroot()
    root = empty('karr')
    root.rotation_mode = 'XYZ'
    links = {}
    joints = {}
    child_of = {j.find('child').get('link'): j for j in root_xml.findall('joint')}
    root_link = next(l.get('name') for l in root_xml.findall('link') if l.get('name') not in child_of)
    links[root_link] = empty(root_link, root)

    pending = list(root_xml.findall('joint'))
    while pending:
        for j in list(pending):
            parent = j.find('parent').get('link')
            if parent not in links:
                continue
            pending.remove(j)
            child = j.find('child').get('link')
            org = empty(f'{j.get("name")}_origin', links[parent])
            org.matrix_basis = origin_matrix(j)
            if j.get('type') in ('revolute', 'continuous'):
                # rotating empty: Euler about the joint axis (all K.A.R.R. joints turn about local +Z)
                rot = empty(f'{j.get("name")}_rot', org)
                rot.rotation_mode = 'XYZ'
                joints[j.get('name')] = rot
                links[child] = empty(child, rot)
            else:
                links[child] = empty(child, org)

    for l in root_xml.findall('link'):
        mesh = l.find('visual/geometry/mesh')
        if mesh is None:
            continue
        path = MESHES / mesh.get('filename').split('/')[-1]
        bpy.ops.wm.stl_import(filepath=str(path))
        obj = bpy.context.selected_objects[0]
        obj.name = f'{l.get("name")}_mesh'
        obj.parent = links[l.get('name')]
        obj.matrix_basis = origin_matrix(l.find('visual'))
        obj.data.materials.clear()
        obj.data.materials.append(mat_for(l.get('name')))
        smooth(obj, 0.5 if l.get('name').endswith('_wheel_link') else 0.7)
    return root, joints


def set_joints(joints, steer, wheel, frame):
    for m in ('fl', 'fr', 'bl', 'br'):
        joints[f'{m}_steering'].rotation_euler = (0, 0, steer[m])
        joints[f'{m}_wheel'].rotation_euler = (0, 0, wheel[m])
        key(joints[f'{m}_steering'], 'rotation_euler', frame=frame)
        key(joints[f'{m}_wheel'], 'rotation_euler', frame=frame)


def set_pose(root, x, y, z, roll, pitch, yaw, frame):
    root.location = (x, y, z)
    root.rotation_euler = (roll, pitch, yaw)
    key(root, 'location', 'rotation_euler', frame=frame)


# ===================================== periodic splines ==============================================================
def catmull_rom_closed(points, n):
    """Dense samples of a closed centripetal-ish (uniform) Catmull-Rom spline through points."""
    P = np.asarray(points, float)
    k = len(P)
    out = []
    per = n // k
    for i in range(k):
        p0, p1, p2, p3 = P[(i - 1) % k], P[i], P[(i + 1) % k], P[(i + 2) % k]
        t = np.linspace(0, 1, per, endpoint=False)[:, None]
        out.append(0.5 * ((2 * p1) + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t ** 2
                          + (-p0 + 3 * p1 - 3 * p2 + p3) * t ** 3))
    return np.concatenate(out)


def circ_smooth(x, sigma):
    """Circular Gaussian smoothing of a periodic sequence (sigma in samples)."""
    if sigma <= 0:
        return x
    n = len(x)
    k = np.fft.fftfreq(n)
    return np.real(np.fft.ifft(np.fft.fft(x) * np.exp(-2 * (math.pi * k * sigma) ** 2)))


def smoothstep(x):
    x = np.clip(x, 0, 1)
    return x * x * x * (x * (6 * x - 15) + 10)


# ===================================== TURNTABLE =====================================================================
def scene_turntable(a):
    sc = reset_scene(a)
    root, joints = build_robot()
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import karr_world
    karr_world.restyle_robot()
    karr_world.plates(bpy.data.objects['base_link'])

    world = bpy.data.worlds.new('studio')
    sc.world = world
    world.use_nodes = True
    world.node_tree.nodes['Background'].inputs['Color'].default_value = (0.012, 0.014, 0.018, 1)  # = bg
    world.node_tree.nodes['Background'].inputs['Strength'].default_value = 1.0

    # floor that fades into the background colour: an endless studio backdrop
    bg = (0.012, 0.014, 0.018)
    fm = material('floor', (0.03, 0.033, 0.04), rough=0.3)
    nt = fm.node_tree
    out = nt.nodes['Material Output']
    bsdf = nt.nodes['Principled BSDF']
    tc = nt.nodes.new('ShaderNodeTexCoord')
    ln = nt.nodes.new('ShaderNodeVectorMath')
    ln.operation = 'LENGTH'
    mr = nt.nodes.new('ShaderNodeMapRange')
    mr.inputs['From Min'].default_value, mr.inputs['From Max'].default_value = 1.8, 6.0
    em = nt.nodes.new('ShaderNodeEmission')
    em.inputs['Color'].default_value = (*bg, 1)
    mix = nt.nodes.new('ShaderNodeMixShader')
    nt.links.new(tc.outputs['Object'], ln.inputs[0])
    nt.links.new(ln.outputs['Value'], mr.inputs['Value'])
    nt.links.new(mr.outputs['Result'], mix.inputs['Fac'])
    nt.links.new(bsdf.outputs['BSDF'], mix.inputs[1])
    nt.links.new(em.outputs['Emission'], mix.inputs[2])
    nt.links.new(mix.outputs['Shader'], out.inputs['Surface'])
    mesh_from('floor', [(-30, -30, 0), (30, -30, 0), (30, 30, 0), (-30, 30, 0)], [(0, 1, 2, 3)], fm)

    def area(name, loc, energy, size, color=(1, 1, 1)):
        ld = bpy.data.lights.new(name, 'AREA')
        ld.energy, ld.size, ld.color = energy, size, color
        lo = link_obj(bpy.data.objects.new(name, ld))
        lo.location = loc
        look_at(lo, (0, 0, 0.5))
        return lo
    area('key', (2.0, -2.2, 3.4), 650, 2.0, (1.0, 0.96, 0.9))
    area('fill', (-3.0, -1.0, 1.6), 250, 3.0, (0.85, 0.9, 1.0))
    area('rim', (-1.5, 3.0, 2.6), 900, 1.5, (0.75, 0.85, 1.0))
    area('top', (0, 0, 4.0), 250, 4.0)

    cam = link_obj(bpy.data.objects.new('camera', bpy.data.cameras.new('camera')))
    sc.camera = cam
    cam.data.lens = 50
    cam.data.dof.use_dof = True
    cam.data.dof.aperture_fstop = 5.6

    n = a['fps'] * 6                      # 6 s loop
    sc.frame_start, sc.frame_end = 0, n - 1
    target = Vector((0, 0, 0.5))
    for f in range(-1, n + 1):
        u = f / n
        az = -math.pi / 2 + 2 * math.pi * u + 0.6     # start front-left 3/4
        r, h = 3.4, 1.15 + 0.25 * math.sin(2 * math.pi * u)
        cam.location = (r * math.cos(az), r * math.sin(az), h)
        look_at(cam, target)
        cam.data.dof.focus_distance = (cam.location - target).length
        key(cam, 'location', 'rotation_quaternion', frame=f)
        cam.data.dof.keyframe_insert('focus_distance', frame=f)
        s = 0.35 * math.sin(4 * math.pi * u)            # front wheels sway twice per loop
        set_joints(joints, {'fl': s, 'fr': s, 'bl': 0.0, 'br': 0.0}, {m: 0.0 for m in 'fl fr bl br'.split()}, f)
        set_pose(root, 0, 0, 0, 0, 0, 0, f)
    return sc


# ===================================== main ==========================================================================
def main():
    a = parse_args()
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    if a['scene'] == 'mission':
        import karr_mission
        sc = karr_mission.scene_mission(a)
    else:
        sc = scene_turntable(a)
    if a['frames']:
        sc.frame_start, sc.frame_end = a['frames']
    out = REPO / 'media' / 'frames' / a['scene']
    out.mkdir(parents=True, exist_ok=True)
    if not a['render']:
        return
    if a['still'] is not None:
        for f in a['still']:
            sc.frame_set(f)
            sc.render.filepath = str(REPO / 'media' / 'frames' / f'{a["scene"]}_still_{f:04d}.png')
            bpy.ops.render.render(write_still=True)
    else:
        sc.render.filepath = str(out / 'f_')
        bpy.ops.render.render(animation=True)


if __name__ == '__main__':
    main()
