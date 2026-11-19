"""Render K.A.R.R. animations for the README with Blender (Cycles), straight from the URDF and full-quality meshes.

    blender -b -P media/render_karr.py -- turntable            # 360 deg studio turntable  -> media/frames/turntable/
    blender -b -P media/render_karr.py -- action               # looping action sequence  -> media/frames/action/
    blender -b -P media/render_karr.py -- action --still 40,120 # test frames (PNG) instead of the whole sequence
    blender    -P media/render_karr.py -- action --no-render   # open the scene in the Blender UI to look around
    bash media/make_gifs.sh                                    # frames -> media/*.gif

Options: --res 640x360  --fps 20  --samples 24  --still N[,N...]  --no-render  --cpu
Everything is procedural: the robot comes from karr_description, the action path is a closed spline, and steering
angles / wheel speeds come from karr_control's kinematics, so the wheels do what the real robot would do (mostly;
the jump is artistic licence). Both animations loop perfectly.
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
from karr_control.kinematics import Module, module_commands  # noqa: E402

URDF = REPO / 'karr_description' / 'urdf' / 'karr.urdf'
MESHES = REPO / 'karr_description' / 'meshes'
MODULES = [Module('fl', 0.3556, 0.2286), Module('fr', 0.3556, -0.2286),
           Module('bl', -0.3556, 0.2286), Module('br', -0.3556, -0.2286)]
WHEEL_R = 0.1225


# ===================================== arguments =====================================================================
def parse_args():
    argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
    a = {'scene': 'turntable', 'res': (640, 360), 'fps': 20, 'samples': 24, 'still': None, 'render': True,
         'gpu': True}
    it = iter(argv)
    for x in it:
        if x in ('turntable', 'action'):
            a['scene'] = x
        elif x == '--res':
            a['res'] = tuple(int(v) for v in next(it).split('x'))
        elif x == '--fps':
            a['fps'] = int(next(it))
        elif x == '--samples':
            a['samples'] = int(next(it))
        elif x == '--still':
            a['still'] = [int(v) for v in next(it).split(',')]
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
        for dev_type in ('METAL', 'OPTIX', 'CUDA', 'HIP', 'ONEAPI'):
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
ROBOT_MATS = {}


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


def periodic_keys(u, keys):
    """Smooth periodic interpolation of keyframes [(u_k, value), ...] (u in [0,1)) at u (array)."""
    us = np.array([k[0] for k in keys])
    vs = np.array([k[1] for k in keys], float)
    out = np.empty_like(u, dtype=float)
    n = len(us)
    for idx, x in enumerate(u):
        x = x % 1.0
        i = np.searchsorted(us, x, side='right') - 1
        i0, i1 = i % n, (i + 1) % n
        u0, u1 = us[i0], us[i1] + (1.0 if i1 <= i0 else 0.0)
        xx = x + (1.0 if x < u0 else 0.0)
        t = (xx - u0) / (u1 - u0)
        im, i2 = (i0 - 1) % n, (i1 + 1) % n
        p0, p1, p2, p3 = vs[im], vs[i0], vs[i1], vs[i2]
        out[idx] = 0.5 * ((2 * p1) + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t ** 2
                          + (-p0 + 3 * p1 - 3 * p2 + p3) * t ** 3)
    return out


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


# ===================================== ACTION ========================================================================
# Cartoon warehouse loading dock. The robot bursts out of dock door A, jumps off the dock, slaloms the cones, drifts
# round the barrels, stops for a hero swing turn, drives up the dock ramp and back in through door B. Inside the
# building (hidden) it turns round and comes out of door A again: a perfect loop.
# x = east (away from the building), y = north. The dock face is at x = 0, the dock edge at x = DOCK_EDGE.
DOCK_H = 1.2            # m, standard 48 in dock height (= warehouse floor)
DOCK_EDGE = 4.0         # m, dock platform depth
DOOR_A_Y, DOOR_B_Y = 3.0, -5.0
KICKER = dict(x0=3.0, x1=DOCK_EDGE, h=0.2)       # little lip on the dock edge for extra air
RAMP_FOOT = 12.0        # dock ramp from (RAMP_FOOT, DOOR_B_Y, 0) up to (DOCK_EDGE, DOOR_B_Y, DOCK_H)
TRACK = [(-3, DOOR_A_Y), (0, DOOR_A_Y), (2, DOOR_A_Y), (4, DOOR_A_Y), (10, DOOR_A_Y), (14, DOOR_A_Y),
         (17, 4.1), (20, 1.9), (23, 4.1), (26, 3.2), (30, 1.0), (31, -3.0), (27.5, -5.3), (23, -5),
         (17, -5), (12, -5), (4, -5), (1, -5), (-3, -5), (-6.5, -3.5), (-7, 0), (-6, 2.4)]
CONES = [(17, 3.0), (20, 3.0), (23, 3.0)]                 # 0.4 m (16 in) traffic cones
BARRELS = [(26.6, -1.2), (27.4, -0.4), (27.6, -1.6)]       # 0.9 m (3 ft) drums
HERO_X = 17.5           # hero stop on the westbound run, facing the ramp
V_MAX, A_LAT, A_DEC, A_ACC = 8.0, 12.0, 8.0, 5.0
G_JUMP = 6.0            # floaty cartoon gravity
SLOWMO = 0.45           # story time runs at 45 % while airborne
HERO_HOLD = 3.4         # s: settle, swing turn, nod
FIXED_CAM = ((14.0, -1.0, 3.2), (0.0, -1.0, 3.3))          # watches both doors while the robot is inside


def track_tables():
    P = catmull_rom_closed(TRACK, 44000)
    d = np.diff(np.vstack([P, P[:1]]), axis=0)
    seg = np.hypot(d[:, 0], d[:, 1])
    s = np.concatenate([[0], np.cumsum(seg)[:-1]])
    S = seg.sum()
    head = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
    total = 2 * math.pi * round((head[-1] - head[0]) / (2 * math.pi))
    dh = np.diff(np.concatenate([head, [head[0] + total]]))
    kappa = circ_smooth(dh / np.maximum(seg, 1e-9), 60)
    return P, s, S, head, kappa


def build_story():
    """Robot state over one loop, sampled in story time (400 Hz)."""
    P, s, S, head, kappa = track_tables()

    def s_at(x_target, y_target):
        return s[int(np.argmin(np.hypot(P[:, 0] - x_target, P[:, 1] - y_target)))]
    s_kick0, s_edge = s_at(KICKER['x0'], DOOR_A_Y), s_at(DOCK_EDGE, DOOR_A_Y)
    s_ramp0, s_ramp1 = s_at(RAMP_FOOT, DOOR_B_Y), s_at(DOCK_EDGE, DOOR_B_Y)
    s_hero = s_at(HERO_X, DOOR_B_Y)

    theta = math.atan2(KICKER['h'], KICKER['x1'] - KICKER['x0'])
    vz = V_MAX * math.sin(theta)
    z0 = DOCK_H + KICKER['h']
    t_air = (vz + math.sqrt(vz * vz + 2 * G_JUMP * z0)) / G_JUMP
    s_land = s_edge + V_MAX * math.cos(theta) * t_air
    ramp_pitch = math.atan2(DOCK_H, RAMP_FOOT - DOCK_EDGE)

    v = np.minimum(V_MAX, np.sqrt(A_LAT / np.maximum(np.abs(kappa), 1e-6)))
    v = circ_smooth(v, 200)
    v[(s > s_edge - 8) & (s < s_land + 2)] = V_MAX
    v = np.minimum.reduce([v, np.sqrt(2 * A_DEC * ((s_hero - s) % S)), np.sqrt(2 * A_ACC * ((s - s_hero) % S))])

    ds = np.diff(np.concatenate([s, [S]]))
    tau_s = np.concatenate([[0], np.cumsum(ds / np.maximum(v, 0.03))[:-1]])
    i_hero = int(np.argmin(np.abs(s - s_hero)))
    tau_s[i_hero:] += HERO_HOLD
    T_story = tau_s[-1] + ds[-1] / max(v[-1], 0.03)

    dt = 1 / 400
    tau = np.arange(0, T_story, dt)
    n = len(tau)
    st = dict(x=np.zeros(n), y=np.zeros(n), z=np.zeros(n), floor=np.zeros(n), roll=np.zeros(n),
              pitch=np.zeros(n), yaw=np.zeros(n), path_yaw=np.zeros(n), warp=np.ones(n), inside=np.zeros(n),
              steer={m.name: np.zeros(n) for m in MODULES}, wspeed={m.name: np.zeros(n) for m in MODULES})
    tau_hero0 = tau_s[i_hero] - HERO_HOLD
    for i, t in enumerate(tau):
        if tau_hero0 <= t < tau_hero0 + HERO_HOLD:
            j, vv, h = i_hero, 0.0, t - tau_hero0
        else:
            j = min(np.searchsorted(tau_s, t, side='right') - 1, len(s) - 1)
            vv, h = v[j], None
        x, y = P[j]
        ps, k, sj = head[j], kappa[j], s[j]
        st['path_yaw'][i] = ps
        beta = float(np.clip(0.0045 * vv * vv * k, -0.42, 0.42))    # fast corner = drift
        yaw, roll = ps + beta, -np.clip(vv * vv * k * 0.006, -0.09, 0.09)
        a_long = (v[min(j + 40, len(v) - 1)] - v[max(j - 40, 0)]) / max(80 * (S / len(s)) / max(vv, 0.5), 1e-3)
        pitch = float(np.clip(-0.010 * a_long, -0.06, 0.06))
        cmds = module_commands(vv, vv * k, MODULES, 'front', WHEEL_R)
        steer = {n_: c[0] - (beta * 1.6 if n_ in ('fl', 'fr') else 0.0) for n_, c in cmds.items()}
        wsp = {n_: c[1] for n_, c in cmds.items()}

        # --- height: dock / kicker / air / yard / ramp
        on_dock = sj >= s_ramp1 or sj < s_edge
        floor = DOCK_H if on_dock else 0.0
        z = floor
        st['inside'][i] = float(x < 0.0)
        if s_kick0 <= sj < s_edge:
            q = (sj - s_kick0) / (s_edge - s_kick0)
            z = DOCK_H + KICKER['h'] * q
            pitch = -theta * smoothstep(q / 0.4)
        elif s_edge <= sj < s_land:
            ta = (sj - s_edge) / (V_MAX * math.cos(theta))
            q = ta / t_air
            z = z0 + vz * ta - 0.5 * G_JUMP * ta * ta
            floor = 0.0
            pitch = -theta + (theta + 0.15) * q
            yaw += 2 * math.pi * smoothstep(q)                  # 360 in the air, obviously
            roll = 0.15 * math.sin(math.pi * q)
            wsp = {n_: V_MAX / WHEEL_R for n_ in wsp}
            steer = {n_: 0.25 * math.sin(2 * math.pi * q) if n_ in ('fl', 'fr') else 0.0 for n_ in steer}
            st['warp'][i] = SLOWMO
        elif s_land <= sj < s_land + 6:
            ta = (sj - s_land) / V_MAX
            z = -0.09 * math.exp(-6 * ta) * math.cos(14 * ta)  # landing squash + rebound
            pitch = 0.14 * math.exp(-5 * ta) * math.cos(11 * ta)
        elif s_ramp0 <= sj < s_ramp1:
            q = (sj - s_ramp0) / (s_ramp1 - s_ramp0)
            z = floor = DOCK_H * q
            pitch = -ramp_pitch * smoothstep(q / 0.12) * (1 - smoothstep((q - 0.88) / 0.12))

        # --- hero: stop, swing turn about the rear axle centre, nod at the camera
        if h is not None:
            swing0, swing1 = 0.45, 2.75
            q = float(smoothstep((h - swing0) / (swing1 - swing0)))
            ang = 2 * math.pi * q
            c = np.array([x, y]) - np.array([math.cos(ps), math.sin(ps)]) * 0.3556
            x, y = c + np.array([math.cos(ps + ang), math.sin(ps + ang)]) * 0.3556
            yaw = ps + ang
            blend = float(smoothstep(h / 0.4) * (1 - smoothstep((h - swing1) / 0.4)))
            sw = module_commands(0.0, 1.0, MODULES, 'front', WHEEL_R)
            steer = {n_: sw[n_][0] * blend for n_ in sw}
            dq = 2 * math.pi / (swing1 - swing0) * 30 * q * q * (1 - q) * (1 - q) if swing0 < h < swing1 else 0.0
            wsp = {n_: sw[n_][1] * dq for n_ in sw}
            pitch = 0.07 * math.sin(math.pi * min(h / 0.5, 1.0)) \
                + 0.09 * math.sin(math.pi * float(np.clip((h - 2.95) / 0.45, 0, 1))) ** 2
        st['x'][i], st['y'][i], st['z'][i], st['floor'][i] = x, y, z, floor
        st['roll'][i], st['pitch'][i], st['yaw'][i] = roll, pitch, yaw
        for n_ in steer:
            st['steer'][n_][i], st['wspeed'][n_][i] = steer[n_], wsp[n_]

    st['warp'] = np.clip(circ_smooth(st['warp'], 0.12 / dt), SLOWMO, 1.0)
    for n_ in st['steer']:
        st['steer'][n_] = circ_smooth(st['steer'][n_], 0.06 / dt)
    st['wangle'] = {n_: np.cumsum(st['wspeed'][n_]) * dt for n_ in st['wspeed']}
    st['yaw'] = np.unwrap(st['yaw'])
    st['path_yaw'] = np.unwrap(st['path_yaw'])
    st['tau'], st['dt'], st['T_story'] = tau, dt, T_story
    return st


# ---------------- cartoon set ----------------
def toon(name, hexcolor, rough=0.8, **kw):
    return material(name, srgb(hexcolor), rough=rough, **kw)


def box(name, center, size, mat, rot_z=0.0):
    bpy.ops.mesh.primitive_cube_add(size=1, location=center, rotation=(0, 0, rot_z))
    o = bpy.context.object
    o.name = name
    o.scale = size
    o.data.materials.append(mat)
    return o


def cyl(name, center, r, h, mat, verts=32):
    bpy.ops.mesh.primitive_cylinder_add(vertices=verts, radius=r, depth=h, location=center)
    o = bpy.context.object
    o.name = name
    o.data.materials.append(mat)
    smooth(o, 1.0)
    return o


def build_set(sc):
    # sky: simple vertical gradient, flat and friendly
    world = bpy.data.worlds.new('sky')
    sc.world = world
    world.use_nodes = True
    nt = world.node_tree
    tc = nt.nodes.new('ShaderNodeTexCoord')
    sep = nt.nodes.new('ShaderNodeSeparateXYZ')
    cr = nt.nodes.new('ShaderNodeValToRGB')
    cr.color_ramp.elements[0].position, cr.color_ramp.elements[0].color = 0.0, (*srgb('dcefff'), 1)
    cr.color_ramp.elements[1].position, cr.color_ramp.elements[1].color = 0.45, (*srgb('5fb2f0'), 1)
    nt.links.new(tc.outputs['Generated'], sep.inputs[0])
    nt.links.new(sep.outputs['Z'], cr.inputs['Fac'])
    nt.links.new(cr.outputs['Color'], nt.nodes['Background'].inputs['Color'])
    nt.nodes['Background'].inputs['Strength'].default_value = 0.9
    sun = bpy.data.lights.new('sun', 'SUN')
    sun.energy, sun.angle = 3.2, math.radians(4)
    so = link_obj(bpy.data.objects.new('sun', sun))
    so.rotation_euler = Euler((math.radians(48), 0, math.radians(68)), 'XYZ')   # from the south-east: lights the dock
    sc.view_settings.exposure = -0.6

    m = {k_: toon(k_, c) for k_, c in {
        'asphalt': '4a5160', 'dock': 'a8acb3', 'wall': 'f2c14e', 'wall_low': 'd9883b', 'trim': 'c4553b',
        'roof': '8c3a2e', 'door': '9aa3ad', 'dark': '15161a', 'bumper': '222222', 'yellow': 'ffd21f',
        'grass': '7cc95a', 'hill': '5fae4a', 'trunk': '7a4e2d', 'leaf': '3f9e3c', 'fence': 'e8e8e8',
        'cone': 'ff6a13', 'white': 'f7f7f7', 'barrel': 'ff6a13', 'crate': 'c79a5b', 'pallet': 'a6773f',
        'ramp': '8f99a6', 'sign': '1f4e9c', 'line': 'ffe066'}.items()}

    # ground: yard asphalt, grass beyond
    mesh_from('grass', [(-120, -120, -0.01), (160, -120, -0.01), (160, 120, -0.01), (-120, 120, -0.01)],
              [(0, 1, 2, 3)], m['grass'])
    box('yard', (22, -1, -0.005), (44, 30, 0.01), m['asphalt'])
    for (cx, cy, lx, ly) in [(9, 6.5, 10, 0.15), (9, -8.5, 10, 0.15), (16, -1, 0.15, 13), (36, -1, 0.15, 13)]:
        box('line', (cx, cy, 0.002), (lx, ly, 0.004), m['line'])

    # warehouse: floor at dock height, walls, roof, two open roll-up doors
    W0, W1, H = -18.0, 0.0, 7.5
    Y0, Y1 = -14.0, 12.0
    box('wh_floor', ((W0 + W1) / 2, (Y0 + Y1) / 2, DOCK_H / 2), (W1 - W0, Y1 - Y0, DOCK_H), m['dock'])
    box('dock', (DOCK_EDGE / 2, -1.0, DOCK_H / 2), (DOCK_EDGE, 16.0, DOCK_H), m['dock'])
    box('dock_stripe', (DOCK_EDGE - 0.1, -1.0, DOCK_H + 0.003), (0.2, 16.0, 0.006), m['yellow'])
    door_w, door_h = 3.4, 3.6
    # front wall split into pieces around the door openings
    z_mid = DOCK_H + door_h
    pieces = [(Y0, DOOR_B_Y - door_w / 2), (DOOR_B_Y + door_w / 2, DOOR_A_Y - door_w / 2), (DOOR_A_Y + door_w / 2, Y1)]
    for a_, b_ in pieces:
        box('wall_front', (W1 - 0.15, (a_ + b_) / 2, DOCK_H + door_h / 2), (0.3, b_ - a_, door_h), m['wall'])
    box('wall_top', (W1 - 0.15, (Y0 + Y1) / 2, (z_mid + H) / 2), (0.3, Y1 - Y0, H - z_mid), m['wall'])
    box('wall_low', (W1 - 0.1, (Y0 + Y1) / 2, DOCK_H / 2), (0.2, Y1 - Y0, DOCK_H + 0.02), m['wall_low'])
    box('wall_n', ((W0 + W1) / 2, Y1, (DOCK_H + H) / 2), (W1 - W0, 0.3, H - DOCK_H), m['wall'])
    box('wall_s', ((W0 + W1) / 2, Y0, (DOCK_H + H) / 2), (W1 - W0, 0.3, H - DOCK_H), m['wall'])
    box('wall_back', (W0, (Y0 + Y1) / 2, (DOCK_H + H) / 2), (0.3, Y1 - Y0, H - DOCK_H), m['wall'])
    box('roof', ((W0 + W1) / 2 + 0.3, (Y0 + Y1) / 2, H + 0.2), (W1 - W0 + 1.2, Y1 - Y0 + 1.2, 0.4), m['roof'])
    box('trim', (W1 + 0.05, (Y0 + Y1) / 2, H - 0.25), (0.12, Y1 - Y0, 0.25), m['trim'])
    box('interior', ((W0 + W1) / 2, (Y0 + Y1) / 2, H / 2 + 1), (W1 - W0 - 1, Y1 - Y0 - 1, 0.1), m['dark'])
    for dy in (DOOR_A_Y, DOOR_B_Y):
        box('door_roll', (W1 + 0.05, dy, z_mid + 0.25), (0.35, door_w + 0.2, 0.5), m['door'])
        for side in (-1, 1):
            box('bumper', (W1 + 0.0 + DOCK_EDGE + 0.06, dy + side * (door_w / 2 + 0.4), DOCK_H - 0.35),
                (0.12, 0.25, 0.45), m['bumper'])
        box('door_light', (W1 + 0.2, dy + door_w / 2 + 0.5, z_mid + 0.3), (0.2, 0.3, 0.3), m['yellow'])
    # big sign
    bpy.ops.object.text_add(location=(W1 + 0.32, -1.0, z_mid + 1.1), rotation=(math.pi / 2, 0, math.pi / 2))
    txt = bpy.context.object
    txt.data.body = 'K.A.R.R. HQ'
    txt.data.align_x, txt.data.align_y = 'CENTER', 'CENTER'
    txt.data.size = 1.25
    txt.data.extrude = 0.05
    txt.data.materials.append(m['sign'])
    box('sign_board', (W1 + 0.2, -1.0, z_mid + 1.1), (0.08, 9.6, 1.8), m['white'])
    # dark interior blocks the view through the doors
    box('inside_dark', (W1 - 3.5, (Y0 + Y1) / 2, DOCK_H + 3), (0.2, Y1 - Y0 - 0.6, 6), m['dark'])
    # crates and pallets on the dock
    for (cx, cy, sz) in [(1.2, 7.5, 0.9), (1.3, 6.6, 0.7), (1.0, 7.1, 0.6), (2.0, -9.0, 0.9), (1.1, -8.6, 0.8)]:
        box('crate', (cx, cy, DOCK_H + sz / 2 + (0.6 if sz == 0.6 else 0)), (sz, sz, sz), m['crate'])
    box('pallet', (2.2, -1.0, DOCK_H + 0.07), (1.2, 1.0, 0.14), m['pallet'])

    # dock ramp (metal) up to door B
    rw = 1.8
    mesh_from('dock_ramp', [(RAMP_FOOT, DOOR_B_Y - rw / 2, 0), (DOCK_EDGE, DOOR_B_Y - rw / 2, DOCK_H),
                            (DOCK_EDGE, DOOR_B_Y + rw / 2, DOCK_H), (RAMP_FOOT, DOOR_B_Y + rw / 2, 0),
                            (DOCK_EDGE, DOOR_B_Y - rw / 2, 0), (DOCK_EDGE, DOOR_B_Y + rw / 2, 0)],
              [(0, 3, 2, 1), (0, 1, 4), (3, 5, 2), (1, 2, 5, 4)], m['ramp'])
    for side in (-1, 1):   # handrails
        yy = DOOR_B_Y + side * (rw / 2 + 0.03)
        for xx in np.linspace(RAMP_FOOT - 0.3, DOCK_EDGE + 0.3, 4):
            zb = DOCK_H * (RAMP_FOOT - xx) / (RAMP_FOOT - DOCK_EDGE)
            cyl('post', (xx, yy, zb + 0.45), 0.03, 0.9, m['yellow'], 12)
    # kicker lip at the dock edge in front of door A
    kw = 1.6
    mesh_from('kicker', [(KICKER['x0'], DOOR_A_Y - kw / 2, DOCK_H), (KICKER['x1'], DOOR_A_Y - kw / 2, DOCK_H),
                         (KICKER['x1'], DOOR_A_Y + kw / 2, DOCK_H), (KICKER['x0'], DOOR_A_Y + kw / 2, DOCK_H),
                         (KICKER['x1'], DOOR_A_Y - kw / 2, DOCK_H + KICKER['h']),
                         (KICKER['x1'], DOOR_A_Y + kw / 2, DOCK_H + KICKER['h'])],
              [(0, 4, 5, 3), (0, 1, 4), (3, 5, 2), (1, 2, 5, 4)], m['yellow'])

    # cones (0.4 m) and barrels (0.9 m)
    for (cx, cy) in CONES:
        bpy.ops.mesh.primitive_cone_add(vertices=24, radius1=0.12, radius2=0.02, depth=0.38, location=(cx, cy, 0.21))
        c = bpy.context.object
        c.data.materials.append(m['cone'])
        smooth(c, 1.0)
        cyl('cone_band', (cx, cy, 0.24), 0.078, 0.06, m['white'], 24)
        box('cone_base', (cx, cy, 0.012), (0.28, 0.28, 0.024), m['cone'])
    for (bx, by) in BARRELS:
        for z0, hh, mm in [(0, 0.22, 'barrel'), (0.22, 0.1, 'white'), (0.32, 0.22, 'barrel'), (0.54, 0.1, 'white'),
                           (0.64, 0.26, 'barrel')]:
            cyl('barrel', (bx, by, z0 + hh / 2), 0.29, hh, m[mm], 32)

    # scenery: fence, cartoon trees, hills
    for xx in np.arange(2, 45, 2.5):
        for yy in (-16.5, 14.5):
            cyl('fence_post', (xx, yy, 0.6), 0.05, 1.2, m['fence'], 8)
    for yy in (-16.5, 14.5):
        box('fence_rail', (23.5, yy, 1.0), (43, 0.05, 0.08), m['fence'])
        box('fence_rail', (23.5, yy, 0.55), (43, 0.05, 0.08), m['fence'])
    rng = np.random.default_rng(7)
    for _ in range(38):
        side = rng.choice([-1, 1])
        tx_, ty_ = rng.uniform(-10, 70), side * rng.uniform(19, 40)
        if rng.random() < 0.3:
            tx_, ty_ = rng.uniform(46, 70), rng.uniform(-20, 20)
        s_ = rng.uniform(0.8, 1.5)
        cyl('trunk', (tx_, ty_, 0.9 * s_), 0.18 * s_, 1.8 * s_, m['trunk'], 10)
        bpy.ops.mesh.primitive_ico_sphere_add(subdivisions=2, radius=1.3 * s_, location=(tx_, ty_, 2.6 * s_))
        leaf = bpy.context.object
        leaf.data.materials.append(m['leaf'])
    for (hx, hy, r_, sz) in [(90, -40, 40, 0.25), (110, 30, 55, 0.22), (60, 75, 45, 0.25), (40, -80, 50, 0.2),
                             (-30, 80, 50, 0.2), (130, -5, 50, 0.2)]:
        bpy.ops.mesh.primitive_uv_sphere_add(segments=48, ring_count=24, radius=r_, location=(hx, hy, 0))
        hill = bpy.context.object
        hill.scale = (1, 1, sz)
        hill.data.materials.append(m['hill'])
        smooth(hill, 1.0)


def scene_action(a):
    sc = reset_scene(a)
    sc.render.motion_blur_shutter = 0.35
    root, joints = build_robot()
    st = build_story()
    build_set(sc)

    # ---------------- frame time <-> story time (slow-mo while airborne) ----------------
    t_frame = np.concatenate([[0], np.cumsum(st['dt'] / st['warp'])[:-1]])
    T = t_frame[-1] + st['dt'] / st['warp'][-1]
    n = int(round(T * a['fps']))
    print(f'story {st["T_story"]:.1f} s -> loop {T:.1f} s, {n} frames @ {a["fps"]} fps')
    sc.frame_start, sc.frame_end = 0, n - 1

    def sample(arr, t):
        loops = math.floor(t / T)
        tt = t - loops * T
        i = min(np.searchsorted(t_frame, tt, side='right') - 1, len(arr) - 1)
        return arr[i], loops

    def frac(i):
        """frame-time fraction of story sample i"""
        return t_frame[i] / T

    # ---------------- camera ----------------
    cam = link_obj(bpy.data.objects.new('camera', bpy.data.cameras.new('camera')))
    sc.camera = cam
    cam.data.clip_end = 500
    frames = np.arange(n)
    tf = frames / a['fps']
    xs = np.array([sample(st['x'], t)[0] for t in tf])
    ys = np.array([sample(st['y'], t)[0] for t in tf])
    zs = np.array([sample(st['z'], t)[0] for t in tf])
    fl = np.array([sample(st['floor'], t)[0] for t in tf])
    py = np.array([sample(st['path_yaw'], t)[0] for t in tf])
    lin = py[0] + 2 * math.pi * round((py[-1] - py[0]) / (2 * math.pi)) * frames / n
    py_s = circ_smooth(py - lin, 0.7 * a['fps']) + lin
    tx = circ_smooth(xs, 0.15 * a['fps'])
    ty = circ_smooth(ys, 0.15 * a['fps'])
    tz = 0.45 + circ_smooth(0.6 * zs + 0.4 * fl, 0.1 * a['fps'])
    base_z = circ_smooth(fl, 0.4 * a['fps'])
    u = frames / n

    def first(cond):
        return int(np.argmax(cond))
    X, Y = st['x'], st['y']
    i_out = first((X > 0) & (np.abs(Y - DOOR_A_Y) < 1))                     # leaves door A
    i_edge = first((X > DOCK_EDGE) & (np.abs(Y - DOOR_A_Y) < 1))
    i_land = first((X > DOCK_EDGE + 7) & (np.abs(Y - DOOR_A_Y) < 2))
    i_slalom = first((X > CONES[1][0]) & (Y > 0))
    i_uturn = first((X > 29) & (Y < 0))
    i_stop = first((np.abs(X - HERO_X) < 0.5) & (Y < -3) & (np.hypot(np.gradient(X), np.gradient(Y)) < 1e-7))
    i_ramp = first((X < RAMP_FOOT) & (Y < -3) & (np.arange(len(X)) > i_stop))
    i_in = first((X < 0) & (Y < -3) & (np.arange(len(X)) > i_stop))          # enters door B
    u_out, u_edge, u_land, u_sl, u_ut = map(frac, (i_out, i_edge, i_land, i_slalom, i_uturn))
    u_stop, u_ramp, u_in = map(frac, (i_stop, i_ramp, i_in))
    print('shot frames: ' + ', '.join(f'{k_} {v_ * n:.0f}' for k_, v_ in dict(
        out=u_out, edge=u_edge, land=u_land, slalom=u_sl, uturn=u_ut, stop=u_stop, ramp=u_ramp, inside=u_in).items()))

    # (u, azimuth relative to travel direction [deg; 0 = ahead looking back, 90 = robot's left], distance,
    #  height above the robot's floor, lens, weight of the fixed door camera)
    shots = [
        (u_out - 0.03, 0, 5, 1.0, 28, 1.0),                 # door camera: bursts out...
        (u_edge, 20, 5.5, 0.6, 26, 0.6),                    # ...flies off the dock toward us
        ((u_edge + u_land) / 2, 80, 4.6, -0.2, 22, 0.0),    # swing round under it in slow-mo
        (u_land, 130, 5.0, 0.6, 26, 0.0),
        (u_sl, 95, 3.8, 0.6, 30, 0.0),                      # side tracking through the cones
        (u_ut, 125, 4.8, 1.3, 28, 0.0),                     # outside of the drift
        (u_stop - 0.04, 15, 5.5, 0.6, 32, 0.0),             # brakes toward the camera
        (u_stop + 0.01, 22, 3.5, 0.55, 40, 0.0),            # hero close-up
        (u_stop + 0.09, 65, 3.3, 0.5, 40, 0.0),             # orbit during the swing turn
        (u_ramp, 160, 4.5, 0.9, 30, 0.0),                   # chase up the ramp
        (u_in, 175, 7.0, 1.6, 30, 0.6),
        (u_in + 0.04, 0, 5, 1.0, 28, 1.0),                  # door camera while it's inside
    ]
    shots = sorted(((u0 % 1.0, *rest) for u0, *rest in shots), key=lambda s_: s_[0])
    az = np.radians(periodic_keys(u, [(s_[0], s_[1]) for s_ in shots]))
    dist = periodic_keys(u, [(s_[0], s_[2]) for s_ in shots])
    hgt = periodic_keys(u, [(s_[0], s_[3]) for s_ in shots])
    lens = periodic_keys(u, [(s_[0], s_[4]) for s_ in shots])
    wfix = np.clip(periodic_keys(u, [(s_[0], s_[5]) for s_ in shots]), 0, 1)
    fpos, ftgt = np.array(FIXED_CAM[0]), np.array(FIXED_CAM[1])

    for f in range(-1, n + 1):
        i = f % n
        ang = py_s[i] + az[i]
        rel = np.array([tx[i] + dist[i] * math.cos(ang), ty[i] + dist[i] * math.sin(ang),
                        max(base_z[i] + hgt[i], 0.25)])
        tgt = np.array([tx[i], ty[i], tz[i]])
        w = wfix[i]
        cam.location = Vector((1 - w) * rel + w * fpos)
        look_at(cam, Vector((1 - w) * tgt + w * ftgt))
        cam.data.lens = lens[i]
        key(cam, 'location', 'rotation_quaternion', frame=f)
        cam.data.keyframe_insert('lens', frame=f)

        t = f / a['fps']
        x, loops = sample(st['x'], t)
        yaw0 = 2 * math.pi * round((st['yaw'][-1] - st['yaw'][0]) / (2 * math.pi))
        set_pose(root, x, sample(st['y'], t)[0], sample(st['z'], t)[0], sample(st['roll'], t)[0],
                 sample(st['pitch'], t)[0], sample(st['yaw'], t)[0] + loops * yaw0, f)
        steer = {m_.name: sample(st['steer'][m_.name], t)[0] for m_ in MODULES}
        wheel = {}
        for m_ in MODULES:
            wa = st['wangle'][m_.name]
            wheel[m_.name] = sample(wa, t)[0] + loops * (wa[-1] + st['wspeed'][m_.name][-1] * st['dt'])
        set_joints(joints, steer, wheel, f)
    return sc


# ===================================== main ==========================================================================
def main():
    a = parse_args()
    sc = scene_turntable(a) if a['scene'] == 'turntable' else scene_action(a)
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


main()
