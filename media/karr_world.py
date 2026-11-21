"""Set pieces for the K.A.R.R. renders: rounded, matte, slightly textured (see docs/design/README.md).

Colours come from the project's Field Manual tokens (docs/visual-style/tokens.json) so renders, docs and slides
share one palette; only render-specific colours (sky, haze, grass, props) are defined here.

Every builder takes real-world sizes in metres and returns the Blender object (or a group empty).
"""
import math

import bpy
from mathutils import Euler

import render_karr as rk

# Field Manual tokens (light theme): slate, orange, ink, slate-tint. Render-only: warm_white, concrete, sky.
PAL = dict(slate='5a6675', orange='e8590c', ink='1b2430', slate_tint='eceff3', warm_white='f2efe6',
           concrete='c7bfb1', concrete2='a9adb3', sky_low='f6e3c8', sky_high='9cc6e6')


# ===================================== materials =====================================================================
def mat(name, hexc, rough=0.7, metal=0.0, emit=0.0, alpha=1.0):
    if name in bpy.data.materials:
        return bpy.data.materials[name]
    m = rk.material(name, rk.srgb(hexc), rough=rough, metal=metal, alpha=alpha)
    if emit:
        b = m.node_tree.nodes['Principled BSDF']
        b.inputs['Emission Color'].default_value = (*rk.srgb(hexc), 1)
        b.inputs['Emission Strength'].default_value = emit
    return m


def textured(name, hex_a, hex_b, scale=40.0, bump=0.4, rough=0.9, ribs=None):
    """Matte surface with colour variation and bumpy grain (asphalt, concrete, ramp). ribs: (axis, freq)."""
    if name in bpy.data.materials:
        return bpy.data.materials[name]
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    b = nt.nodes['Principled BSDF']
    tc = nt.nodes.new('ShaderNodeTexCoord')
    n1 = nt.nodes.new('ShaderNodeTexNoise')
    n1.inputs['Scale'].default_value = scale * 0.08
    n1.inputs['Detail'].default_value = 6
    cr = nt.nodes.new('ShaderNodeValToRGB')
    cr.color_ramp.elements[0].color = (*rk.srgb(hex_a), 1)
    cr.color_ramp.elements[1].color = (*rk.srgb(hex_b), 1)
    nt.links.new(tc.outputs['Object'], n1.inputs['Vector'])
    nt.links.new(n1.outputs['Fac'], cr.inputs['Fac'])
    nt.links.new(cr.outputs['Color'], b.inputs['Base Color'])
    vor = nt.nodes.new('ShaderNodeTexVoronoi')
    vor.inputs['Scale'].default_value = scale
    nt.links.new(tc.outputs['Object'], vor.inputs['Vector'])
    bm = nt.nodes.new('ShaderNodeBump')
    bm.inputs['Strength'].default_value = bump
    bm.inputs['Distance'].default_value = 0.02
    height = vor.outputs['Distance']
    if ribs:
        wave = nt.nodes.new('ShaderNodeTexWave')
        wave.bands_direction = ribs[0]
        wave.inputs['Scale'].default_value = ribs[1]
        add = nt.nodes.new('ShaderNodeMath')
        add.operation = 'MULTIPLY_ADD'
        add.inputs[1].default_value = 0.2
        nt.links.new(tc.outputs['Object'], wave.inputs['Vector'])
        nt.links.new(vor.outputs['Distance'], add.inputs[0])
        nt.links.new(wave.outputs['Fac'], add.inputs[2])
        height = add.outputs['Value']
        bm.inputs['Strength'].default_value = bump * 1.5
    nt.links.new(height, bm.inputs['Height'])
    nt.links.new(bm.outputs['Normal'], b.inputs['Normal'])
    b.inputs['Roughness'].default_value = rough
    return m


# ===================================== primitives ====================================================================
def rbox(name, center, size, m, radius=0.06, rot_z=0.0, seg=4, parent=None):
    """Box with rounded edges (bevel modifier on real-size geometry)."""
    sx, sy, sz = (v / 2 for v in size)
    v = [(-sx, -sy, -sz), (sx, -sy, -sz), (sx, sy, -sz), (-sx, sy, -sz),
         (-sx, -sy, sz), (sx, -sy, sz), (sx, sy, sz), (-sx, sy, sz)]
    f = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    o = rk.mesh_from(name, v, f, m)
    o.location, o.rotation_euler = center, (0, 0, rot_z)
    if radius > 0:
        bev = o.modifiers.new('round', 'BEVEL')
        bev.width = min(radius, min(size) / 2 * 0.95)
        bev.segments = seg
        bev.limit_method = 'ANGLE'
        o.data.shade_smooth()
        o.modifiers.new('wn', 'WEIGHTED_NORMAL').keep_sharp = True
    if parent:
        o.parent = parent
    return o


def rcyl(name, center, r, h, m, verts=32, bevel=0.03, rot=(0, 0, 0), parent=None):
    bpy.ops.mesh.primitive_cylinder_add(vertices=verts, radius=r, depth=h, location=center, rotation=rot)
    o = bpy.context.object
    o.name = name
    o.data.materials.append(m)
    if bevel:
        b = o.modifiers.new('round', 'BEVEL')
        b.width, b.segments, b.limit_method = bevel, 3, 'ANGLE'
    o.data.shade_smooth()
    o.modifiers.new('wn', 'WEIGHTED_NORMAL').keep_sharp = True
    if parent:
        o.parent = parent
    return o


def blob(name, center, r, m, sub=3, squash=1.0, parent=None):
    bpy.ops.mesh.primitive_ico_sphere_add(subdivisions=sub, radius=r, location=center)
    o = bpy.context.object
    o.name = name
    o.scale.z = squash
    o.data.materials.append(m)
    o.data.shade_smooth()
    if parent:
        o.parent = parent
    return o


def wedge(name, x_foot, x_top, y, h, w, m):
    o = rk.mesh_from(name, [(x_foot, y - w / 2, 0), (x_top, y - w / 2, h), (x_top, y + w / 2, h),
                            (x_foot, y + w / 2, 0), (x_top, y - w / 2, 0), (x_top, y + w / 2, 0)],
                     [(0, 3, 2, 1), (0, 1, 4), (3, 5, 2), (1, 2, 5, 4), (0, 4, 5, 3)], m)
    b = o.modifiers.new('round', 'BEVEL')
    b.width, b.segments, b.limit_method = 0.04, 3, 'ANGLE'
    return o


def text(body, loc, rot, size, m, parent=None, extrude=0.002):
    bpy.ops.object.text_add(location=loc, rotation=rot)
    t = bpy.context.object
    t.data.body = body
    t.data.align_x = t.data.align_y = 'CENTER'
    t.data.size, t.data.extrude = size, extrude
    t.data.materials.append(m)
    if parent:
        t.parent = parent
    return t


# ===================================== robot dressing ================================================================
ROBOT_LOOK = dict(body=(PAL['slate'], 0.55), steer=(PAL['orange'], 0.55), wheel=('2a2f38', 0.85),
                  gps=('e9e6df', 0.6), sensor=('17181b', 0.4))
DECK_Z = 0.615          # top of the robot's rear deck (base_link frame)


def restyle_robot():
    """Design-system materials on the URDF robot: matte, soft plastic, no metal or clear coat."""
    keymap = {'karr_body': 'body', 'karr_aluminum': 'steer', 'karr_rubber': 'wheel', 'karr_white': 'gps',
              'karr_sensor': 'sensor'}
    for mname, k in keymap.items():
        b = bpy.data.materials[mname].node_tree.nodes['Principled BSDF']
        hexc, rough = ROBOT_LOOK[k]
        b.inputs['Base Color'].default_value = (*rk.srgb(hexc), 1)
        b.inputs['Roughness'].default_value = rough
        b.inputs['Metallic'].default_value = 0.0
        if 'Coat Weight' in b.inputs:
            b.inputs['Coat Weight'].default_value = 0.0
        if k == 'body' and 'Subsurface Weight' in b.inputs:
            b.inputs['Subsurface Weight'].default_value = 0.08
            b.inputs['Subsurface Radius'].default_value = (0.02, 0.02, 0.02)
        if 'Sheen Weight' in b.inputs and k in ('body', 'steer'):
            b.inputs['Sheen Weight'].default_value = 0.3


def plates(base_link):
    """Off-white KARR licence plates with an orange stripe, front and rear."""
    plate, ink = mat('plate', PAL['warm_white'], 0.6), mat('plate_ink', PAL['ink'], 0.6)
    stripe = mat('plate_stripe', PAL['orange'], 0.6)
    for side, x, rot in (('front', 0.473, math.pi / 2), ('rear', -0.473, -math.pi / 2)):
        s = 1 if x > 0 else -1
        rbox(f'plate_{side}', (x, 0, 0.43), (0.012, 0.21, 0.085), plate, 0.01, parent=base_link)
        t = text('KARR', (x + 0.007 * s, 0, 0.427), (math.pi / 2, 0, rot), 0.062, ink, base_link, 0.0015)
        t.data.space_character = 1.15
        rbox(f'plate_stripe_{side}', (x + 0.0065 * s, 0, 0.392), (0.002, 0.2, 0.008), stripe, 0.0, parent=base_link)


def cardboard_box(name, size=(0.36, 0.32, 0.26), tint='c99d63', sticker=None):
    """Taped cardboard box, origin at the bottom centre. sticker = (text, background hex, text hex): a label on both
    long sides and on the top, so it reads from any camera (EXPRESS on the drone parcel, HEAVY on the forklift box)."""
    g = rk.empty(name)
    g.rotation_mode = 'XYZ'
    sx, sy, sz = size
    rbox(f'{name}_body', (0, 0, sz / 2), size, textured(f'cardboard_{tint}', tint, 'b88d55', 25, 0.15, 0.85), 0.012,
         parent=g)
    rbox(f'{name}_tape', (0, 0, sz + 0.001), (sx + 0.004, 0.05, 0.004), mat('tape', 'e8d9b5', 0.4), 0.0, parent=g)
    if sticker:
        label, bg, fg = sticker
        mbg, mfg = mat(f'sticker_{bg}', bg, 0.55), mat(f'sticker_ink_{fg}', fg, 0.55)
        h = sz * 0.34
        for side in (-1, 1):            # long sides (faces +-y)
            y = side * (sy / 2 + 0.002)
            rbox(f'{name}_sticker', (0, y, sz * 0.5), (sx * 0.78, 0.003, h), mbg, 0.0, parent=g)
            text(label, (0, y + side * 0.002, sz * 0.5), (math.pi / 2, 0, 0 if side < 0 else math.pi),
                 h * 0.5, mfg, g, 0.0)
        for side in (-1, 1):            # ends (faces +-x), smaller text so it fits
            x = side * (sx / 2 + 0.002)
            rbox(f'{name}_sticker_end', (x, 0, sz * 0.5), (0.003, sy * 0.85, h), mbg, 0.0, parent=g)
            text(label, (x + side * 0.002, 0, sz * 0.5), (math.pi / 2, 0, math.pi / 2 if side > 0 else -math.pi / 2),
                 h * 0.5 * min(1.0, sy / sx), mfg, g, 0.0)
        rbox(f'{name}_sticker_top', (-sx * 0.12, -sy * 0.18, sz + 0.004), (sx * 0.55, sy * 0.36, 0.003), mbg, 0.0,
             parent=g)
        text(label, (-sx * 0.12, -sy * 0.18, sz + 0.0065), (0, 0, 0), sy * 0.13, mfg, g, 0.0)
    return g


# ===================================== vehicles ======================================================================
def truck(name, cab_hex, box_hex='eceae4', open_rear=False):
    """Box truck, origin at the middle of the rear bumper on the ground, facing +x. Length ~7.6 m, bed at 1.2 m."""
    g = rk.empty(name)
    g.rotation_mode = 'XYZ'
    mb, mc = mat(f'{name}_box', box_hex, 0.6), mat(f'{name}_cab', cab_hex, 0.55)
    dark, tyre = mat('dark', '2b2d33', 0.7), mat('tyre', '2a2c30', 0.85)
    L = 5.2
    rbox(f'{name}_floor', (L / 2, 0, 1.1), (L, 2.4, 0.2), dark, 0.03, parent=g)
    if open_rear:   # shell open at the back so a forklift can drive in; dark inside
        rbox(f'{name}_roof', (L / 2, 0, 3.55), (L, 2.4, 0.1), mb, 0.05, parent=g)
        for side in (-1, 1):
            rbox(f'{name}_side', (L / 2, side * 1.17, 2.35), (L, 0.06, 2.5), mb, 0.03, parent=g)
        rbox(f'{name}_front', (L - 0.03, 0, 2.35), (0.06, 2.4, 2.5), mb, 0.03, parent=g)
        rbox(f'{name}_inside', (L - 0.2, 0, 2.3), (0.05, 2.25, 2.3), mat('void', '0d0e10', 1.0), 0.0, parent=g)
        for side in (-1, 1):   # doors swung open against the dock seals
            rbox(f'{name}_door', (-0.04, side * 1.2, 2.35), (0.06, 0.04, 2.4), mb, 0.02, parent=g)
    else:
        rbox(f'{name}_shell', (L / 2, 0, 2.35), (L, 2.4, 2.5), mb, 0.16, parent=g)
        rbox(f'{name}_doors', (-0.02, 0, 2.35), (0.06, 2.2, 2.3), mat(f'{name}_door', 'd7d4cc', 0.6), 0.04, parent=g)
    rbox(f'{name}_cab', (L + 1.05, 0, 1.45), (1.9, 2.3, 2.0), mc, 0.35, parent=g)
    rbox(f'{name}_win', (L + 1.85, 0, 1.95), (0.35, 2.0, 0.7), mat('glass_t', '9fd3f0', 0.15), 0.1, parent=g)
    rbox(f'{name}_chassis', (L / 2 + 0.6, 0, 0.6), (L + 1.6, 1.8, 0.35), dark, 0.08, parent=g)
    rbox(f'{name}_bumper', (0.05, 0, 0.75), (0.15, 2.3, 0.18), mat('bumper_y', 'ffc21a', 0.6), 0.04, parent=g)
    wheels = []
    for wx in (1.1, 2.3, L + 1.1):
        for wy in (-1.0, 1.0):
            w = rk.empty(f'{name}_wheel_axis', g)
            w.location, w.rotation_mode = (wx, wy, 0.5), 'XYZ'
            rcyl(f'{name}_tyre', (0, 0, 0), 0.5, 0.35, tyre, 32, 0.08, (math.pi / 2, 0, 0), parent=w)
            rcyl(f'{name}_hub', (0, 0.18 * wy, 0), 0.22, 0.04, mat('hub', 'c8c8c8', 0.5), 24, 0.01,
                 (math.pi / 2, 0, 0), parent=w)
            wheels.append(w)
    return g, wheels


def forklift(name='forklift', body_hex='ffc21a'):
    """Counterbalance forklift, origin at its centre on the ground, facing +x. Returns (group, carriage, beacon_mat).
    Animate carriage.location.z for the fork height (0 = forks at 0.12 m)."""
    g = rk.empty(name)
    g.rotation_mode = 'XYZ'
    yb, dark, tyre = mat(f'{name}_body', body_hex, 0.55), mat('dark', '2b2d33', 0.7), mat('tyre', '2a2c30', 0.85)
    rbox(f'{name}_chassis', (0, 0, 0.55), (1.6, 1.0, 0.6), yb, 0.18, parent=g)
    rbox(f'{name}_cw', (-0.75, 0, 0.75), (0.45, 1.0, 0.85), mat('fork_cw', '3b3d42', 0.7), 0.16, parent=g)
    rbox(f'{name}_seat', (-0.15, 0, 1.0), (0.45, 0.5, 0.35), dark, 0.1, parent=g)
    for px, py in ((0.45, 0.45), (0.45, -0.45), (-0.55, 0.45), (-0.55, -0.45)):
        rbox(f'{name}_post', (px, py, 1.5), (0.06, 0.06, 1.0), dark, 0.02, parent=g)
    rbox(f'{name}_roof', (-0.05, 0, 2.02), (1.15, 1.0, 0.07), dark, 0.03, parent=g)
    bm = mat(f'{name}_beacon', 'ffa31a', 0.4, emit=2.0)
    rcyl(f'{name}_beacon', (-0.4, 0, 2.12), 0.06, 0.1, bm, 16, 0.01, parent=g)
    rbox(f'{name}_mast', (0.86, 0, 1.15), (0.1, 0.8, 2.0), dark, 0.03, parent=g)
    car = rk.empty(f'{name}_carriage', g)
    car.location = (0, 0, 0)
    rbox(f'{name}_plate', (0.95, 0, 0.45), (0.06, 0.8, 0.6), dark, 0.02, parent=car)
    for fy in (-0.18, 0.18):
        rbox(f'{name}_fork', (1.42, fy, 0.12), (0.95, 0.1, 0.04), dark, 0.012, parent=car)
    for wx in (-0.5, 0.55):
        for wy in (-0.5, 0.5):
            rcyl(f'{name}_wheel', (wx, wy, 0.25), 0.25, 0.2, tyre, 24, 0.05, (math.pi / 2, 0, 0), parent=g)
    return g, car, bm


def drone(name='drone'):
    """Delivery quadcopter, origin at the body centre. Returns (group, winch hook empty)."""
    g = rk.empty(name)
    g.rotation_mode = 'XYZ'
    mb, ma, md = mat('drone_body', 'eceae4', 0.5), mat('drone_acc', PAL['orange'], 0.55), mat('dark', '2b2d33', 0.7)
    rbox('d_body', (0, 0, 0), (0.6, 0.6, 0.17), mb, 0.07, parent=g)
    rbox('d_belly', (0, 0, -0.11), (0.32, 0.32, 0.08), md, 0.03, parent=g)
    blur = mat('prop_blur', 'd0d0d0', 0.4, alpha=0.25)
    for k in range(4):
        a = math.pi / 4 + k * math.pi / 2
        x, y = 0.52 * math.cos(a), 0.52 * math.sin(a)
        rbox('d_arm', (x / 2, y / 2, 0.02), (0.55, 0.06, 0.04), mb, 0.02, a, parent=g)
        rcyl('d_motor', (x, y, 0.06), 0.06, 0.08, ma, 24, 0.015, parent=g)
        rcyl('d_prop', (x, y, 0.11), 0.22, 0.006, blur, 48, 0.0, parent=g)
    hook = rk.empty('drone_hook', g)
    return g, hook


# ===================================== buildings and ground ==========================================================
def textures():
    return dict(
        asphalt=textured('asphalt', '5a5652', '4c4844', 120, 0.8, 0.95),
        road=textured('road', '565b66', '4b505a', 120, 0.7, 0.95),
        concrete=textured('concrete', 'c7bfb1', 'b5ad9f', 60, 0.35, 0.9),
        ramp=textured('ramp_steel', '8a94a0', '7b8590', 90, 0.6, 0.75),
        siding=textured('siding', 'dfe3e6', 'd5dade', 30, 0.25, 0.8, ribs=('Y', 6.0)),
        rim=textured('island_rim', 'b08560', 'a07650', 18, 0.35, 0.9),
    )


def warehouse(T, x_front=-8.0, x_back=-26.0, y0=-15.0, y1=15.0, h=7.5, dock_h=1.2, robot_doors=(-6.0, -1.5),
              truck_doors=(5.0, 10.0), platform_depth=4.0):
    """Warehouse with an open loading platform along the front (x = x_front .. x_front + platform_depth)."""
    trim, dark, door = mat('trim', '5b6672', 0.7), mat('void', '0d0e10', 1.0), mat('rolldoor', 'eef0f2', 0.6)
    plinth = mat('plinth', 'b8ad99', 0.8)
    yellow = mat('hazard', 'ffc21a', 0.6)
    W = x_front - x_back
    rbox('wh_floor', ((x_front + x_back) / 2, (y0 + y1) / 2, dock_h / 2), (W, y1 - y0, dock_h), plinth, 0.08)
    rbox('wh_walls', ((x_front + x_back) / 2, (y0 + y1) / 2, dock_h + (h - dock_h) / 2), (W, y1 - y0, h - dock_h),
         T['siding'], 0.3)
    rbox('wh_parapet', ((x_front + x_back) / 2, (y0 + y1) / 2, h + 0.15), (W + 0.3, y1 - y0 + 0.3, 0.3), trim, 0.12)
    rbox('wh_roof', ((x_front + x_back) / 2, (y0 + y1) / 2, h + 0.02), (W - 0.4, y1 - y0 - 0.4, 0.1),
         mat('roof', '8f979f', 0.8), 0.05)
    rbox('wh_windows', (x_front + 0.02, (y0 + y1) / 2, h - 1.0), (0.1, y1 - y0 - 2.5, 0.55),
         mat('glass', 'f2d39b', 0.2, emit=0.3), 0.05)
    # loading platform
    xp = x_front + platform_depth
    rbox('platform', (x_front + platform_depth / 2, (y0 + y1) / 2 - 1.0, dock_h / 2),
         (platform_depth + 0.1, y1 - y0 - 4.0, dock_h), T['concrete'], 0.06)
    rbox('platform_edge', (xp - 0.08, (y0 + y1) / 2 - 1.0, dock_h + 0.003), (0.16, y1 - y0 - 4.0, 0.006), yellow, 0.0)
    for dy in robot_doors:
        rbox('rdoor_frame', (x_front + 0.03, dy, dock_h + 1.6), (0.14, 2.9, 3.3), trim, 0.06)
        rbox('rdoor_void', (x_front + 0.06, dy, dock_h + 1.5), (0.14, 2.6, 3.0), dark, 0.02)
        rbox('rdoor_roll', (x_front + 0.13, dy, dock_h + 3.0), (0.3, 2.8, 0.35), door, 0.1)
    for dy in truck_doors:
        rbox('tdoor_frame', (x_front + 0.03, dy, dock_h + 1.8), (0.14, 3.2, 3.7), trim, 0.06)
        rbox('tdoor_void', (x_front + 0.06, dy, dock_h + 1.7), (0.14, 2.9, 3.4), dark, 0.02)
        rbox('tdoor_roll', (x_front + 0.13, dy, dock_h + 3.45), (0.3, 3.1, 0.4), door, 0.1)
        rbox('canopy', (x_front + 0.8, dy, dock_h + 4.0), (1.6, 4.0, 0.15), trim, 0.06)
        for side in (-1, 1):   # dock bumpers on the platform face
            rbox('bumper', (xp + 0.07, dy + side * 1.3, dock_h - 0.3), (0.14, 0.3, 0.45), dark, 0.05)
    lamps = []
    for dy in list(truck_doors) + list(robot_doors):
        lm = mat(f'lamp_{dy}', 'ffb020', 0.4, emit=0.0)
        rbox('dock_lamp', (x_front + 0.2, dy + 1.8, dock_h + 3.6), (0.2, 0.25, 0.2), lm, 0.05)
        lamps.append(lm)
    return lamps


def pallet_stack(x, y, z, n=2):
    wood, card = mat('pallet', 'b9874f', 0.85), textured('cardboard', 'c99d63', 'b88d55', 25, 0.15, 0.85)
    rbox('pallet', (x, y, z + 0.07), (1.2, 1.0, 0.14), wood, 0.02)
    for i in range(n):
        for j in range(2):
            rbox('crate', (x - 0.28 + 0.56 * j, y, z + 0.14 + 0.22 + i * 0.44), (0.52, 0.9, 0.42), card, 0.02)


def island(T, radius=36.0, lot=(-4, 23, -15, 15), ring_r=31.0, ring_w=4.6):
    grass = mat('grass', 'a3cc6c', 0.85)
    rcyl('island', (0, 0, -2.0), radius, 3.8, T['rim'], 160, 1.2)
    rcyl('island_top', (0, 0, -0.06), radius + 0.2, 0.12, grass, 160, 0.06)
    x0, x1, y0, y1 = lot
    rbox('lot', ((x0 + x1) / 2, (y0 + y1) / 2, -0.01), (x1 - x0, y1 - y0, 0.04), T['asphalt'], 0.4)
    bpy.ops.mesh.primitive_torus_add(major_radius=ring_r, minor_radius=ring_w / 2, major_segments=192,
                                     minor_segments=8, location=(0, 0, -0.03))
    r = bpy.context.object
    r.name = 'ring_road'
    r.scale.z = 0.02
    r.data.materials.append(T['road'])
    dash = mat('dash', 'f2efe6', 0.7)
    for k in range(48):
        a = 2 * math.pi * k / 48
        rbox('ring_dash', (ring_r * math.cos(a), ring_r * math.sin(a), 0.005), (1.4, 0.12, 0.008), dash, 0.0,
             a + math.pi / 2)
    # access road from the ring into the lot
    rbox('access', (24.0 + 2.5, 0.0, -0.012), (6.5, 6.0, 0.04), T['asphalt'], 0.2)


def lot_markings(lines):
    line = mat('line', 'ffd84a', 0.7)
    for (cx, cy, lx, ly) in lines:
        rbox('line', (cx, cy, 0.012), (lx, ly, 0.008), line, 0.0)


def drone_pad(x, y):
    rcyl('pad_ring', (x, y, 0.012), 1.6, 0.012, mat('pad_or', PAL['orange'], 0.6), 64, 0.0)
    rcyl('pad', (x, y, 0.018), 1.35, 0.012, mat('pad_w', PAL['warm_white'], 0.6), 64, 0.0)
    rcyl('pad_dot', (x, y, 0.024), 0.25, 0.012, mat('pad_or', PAL['orange'], 0.6), 48, 0.0)


def cone(x, y):
    o, w = mat('cone', 'ff6a13', 0.6), mat('cone_w', 'f7f5f0', 0.6)
    bpy.ops.mesh.primitive_cone_add(vertices=32, radius1=0.12, radius2=0.025, depth=0.38, location=(x, y, 0.21))
    c = bpy.context.object
    c.data.materials.append(o)
    c.data.shade_smooth()
    rcyl('cone_band', (x, y, 0.24), 0.079, 0.06, w, 32, 0.0)
    rbox('cone_base', (x, y, 0.015), (0.3, 0.3, 0.03), o, 0.012)


def barrel(x, y):
    o, w = mat('cone', 'ff6a13', 0.6), mat('cone_w', 'f7f5f0', 0.6)
    for z0, hh, mm in [(0, 0.24, o), (0.24, 0.1, w), (0.34, 0.22, o), (0.56, 0.1, w), (0.66, 0.24, o)]:
        rcyl('barrel', (x, y, z0 + hh / 2), 0.29, hh, mm, 40, 0.03)


def tree(x, y, s, z=0.0):
    rcyl('trunk', (x, y, z + 0.8 * s), 0.16 * s, 1.6 * s, mat('trunk', '8a5a36', 0.85), 12, 0.04)
    blob('leaf', (x, y, z + 2.4 * s), 1.2 * s, mat('leaf', '5f9e45', 0.8), 3)
    blob('leaf', (x + 0.5 * s, y + 0.3 * s, z + 2.0 * s), 0.8 * s, mat('leaf2', '7bb456', 0.8), 3)


def cloud(x, y, z, s, rng):
    m = mat('cloud', 'ffffff', 0.9)
    for k in range(4):
        blob('cloud', (x + k * 2.2 * s + rng.uniform(-1, 1) * s, y + rng.uniform(-1, 1) * s,
                       z + rng.uniform(-0.5, 0.8) * s), rng.uniform(2.0, 3.2) * s, m, 3, 0.7)


def mini_island(x, y, z, r, rng):
    T = textures()  # cached by name
    rcyl('mini', (x, y, z - 1.2), r, 2.4, T['rim'], 64, 0.6)
    rcyl('mini_top', (x, y, z), r + 0.1, 0.1, mat('grass', 'a3cc6c', 0.85), 64, 0.05)
    for _ in range(max(1, int(r / 2))):
        a, d = rng.uniform(0, 2 * math.pi), rng.uniform(0, r * 0.6)
        tree(x + d * math.cos(a), y + d * math.sin(a), rng.uniform(0.8, 1.3), z)


# ===================================== sky, light, grade =============================================================
def sky(sc, low=PAL['sky_low'], high=PAL['sky_high'], strength=0.9):
    world = bpy.data.worlds.new('sky')
    sc.world = world
    world.use_nodes = True
    nt = world.node_tree
    tc = nt.nodes.new('ShaderNodeTexCoord')
    sep = nt.nodes.new('ShaderNodeSeparateXYZ')
    cr = nt.nodes.new('ShaderNodeValToRGB')
    cr.color_ramp.elements[0].position, cr.color_ramp.elements[0].color = 0.47, (*rk.srgb(low), 1)
    cr.color_ramp.elements[1].position, cr.color_ramp.elements[1].color = 0.6, (*rk.srgb(high), 1)
    nt.links.new(tc.outputs['Generated'], sep.inputs[0])
    nt.links.new(sep.outputs['Z'], cr.inputs['Fac'])
    nt.links.new(cr.outputs['Color'], nt.nodes['Background'].inputs['Color'])
    nt.nodes['Background'].inputs['Strength'].default_value = strength


def sun(elev, az, energy, color='ffffff', angle=6.0):
    s = bpy.data.lights.new('sun', 'SUN')
    s.energy, s.angle, s.color = energy, math.radians(angle), rk.srgb(color)
    o = rk.link_obj(bpy.data.objects.new('sun', s))
    o.rotation_euler = Euler((math.radians(90 - elev), 0, math.radians(az + 90)), 'XYZ')
    return o


def soft_look(sc, haze_hex='e9dccb', mist=(25, 160)):
    """AgX medium-low contrast, distance haze from the mist pass, light bloom (Blender 5 compositor)."""
    for look in ('AgX - Medium Low Contrast', 'Medium Low Contrast'):
        try:
            sc.view_settings.look = look
            break
        except TypeError:
            pass
    sc.view_layers[0].use_pass_mist = True
    sc.world.mist_settings.start, sc.world.mist_settings.depth = mist
    try:
        tree = bpy.data.node_groups.new('soft', 'CompositorNodeTree')
        sc.compositing_node_group = tree
        tree.interface.new_socket('Image', in_out='OUTPUT', socket_type='NodeSocketColor')
        rl = tree.nodes.new('CompositorNodeRLayers')
        out = tree.nodes.new('NodeGroupOutput')
        mix = tree.nodes.new('ShaderNodeMix')
        mix.data_type = 'RGBA'
        mix.inputs['B'].default_value = (*rk.srgb(haze_hex), 1)
        tree.links.new(rl.outputs['Mist'], mix.inputs['Factor'])
        tree.links.new(rl.outputs['Image'], mix.inputs['A'])
        glare = tree.nodes.new('CompositorNodeGlare')
        tree.links.new(mix.outputs['Result'], glare.inputs['Image'])
        for k_, v_ in (('Type', 'Bloom'), ('Strength', 0.25), ('Size', 0.6), ('Threshold', 0.8)):
            if k_ in glare.inputs:
                try:
                    glare.inputs[k_].default_value = v_
                except TypeError:
                    pass
        tree.links.new(glare.outputs['Image'], out.inputs['Image'])
    except Exception as e:  # noqa: BLE001  (compositor API differs between Blender versions)
        print('soft_look: compositor skipped:', e)
