"""K.A.R.R. mission loop: a small island world with a warehouse, a busy loading dock and a drone pad.

Story (one seamless loop, ~19 s):
  1. A drone lowers box A (small, orange EXPRESS sticker) onto K.A.R.R. waiting on the pad.
  2. K.A.R.R. weaves through cones, climbs the dock ramp and drives into the dark warehouse (door A).
  3. Inside (hidden) it swaps boxes and comes out of door B with box B (larger, dark HEAVY sticker: that is
     why a forklift takes it).
  4. At bay 1 the forklift, already waiting half out of the docked truck, lifts box B off the deck and reverses in.
  5. K.A.R.R. hops off the dock between the two trucks (slow motion), drifts past the barrels and heads back to the
     pad, where the next drone is coming in.
Meanwhile trucks circle the ring road, a second forklift unloads bay 2 and the dock lamps blink.

Edit (agreed with the user, Nov 2026): four continuous shots joined by three hard cuts, camera moves instead of
cuts wherever possible (see the shot section in scene_mission). No hero swing turn: it was cut for length.

x = east (yard), y = north, z = up. The warehouse front wall is at X_FRONT, the platform edge at X_EDGE.
Motion is computed in "story time" (world time); the edit runs in frame time, with slow motion while airborne.
Camera shots are hard cuts, keyed on sub-frames so motion blur never smears across a cut.
"""
import math

import bpy
import numpy as np
from mathutils import Quaternion, Vector

import karr_world as W
import render_karr as rk

# ---------------------------------------------------------------- layout
DOCK_H = 1.2
X_FRONT, X_EDGE = -8.0, -4.0
DOOR_IN_Y, DOOR_OUT_Y = -6.0, -1.5
BAY1_Y, BAY2_Y = 5.0, 10.0
RAMP_Y, RAMP_FOOT = -11.0, 4.0
JUMP_Y = 7.5
PAD = (16.0, 3.0)
HAND = (-6.9, BAY1_Y + 0.10)           # robot base when the forklift takes box B (box B sits 0.10 m behind base)
VIEW = (14.0, 11.8)                     # path point after the barrels
BARREL_PASS = (13.0, 11.6)              # 'past the barrels': the camera goes aerial from here
LOT = (-4, 23, -15, 15)
RING_R, RING_LANE = 31.0, 32.1


def slalom_points():
    a, b = np.array([13.8, 1.6]), np.array([9.2, -8.6])
    d = (b - a) / np.linalg.norm(b - a)
    n = np.array([-d[1], d[0]])
    pts, cones = [], []
    for k, f in enumerate((0.22, 0.5, 0.78)):
        c = a + (b - a) * f
        cones.append(tuple(c))
        pts.append(tuple(c + n * (0.85 if k % 2 == 0 else -0.85)))
    return pts, cones


SLALOM, CONES = slalom_points()
TRACK = ([PAD, (14.6, 2.6)] + SLALOM +
         [(8.2, -10.4), (6.2, -11.0), (RAMP_FOOT, RAMP_Y), (0.0, RAMP_Y), (X_EDGE, RAMP_Y), (-5.6, -10.6),
          (-6.4, -8.6), (-6.9, -6.8), (-7.6, -6.05), (-9.5, DOOR_IN_Y), (-11.0, -5.3), (-11.5, -3.75),
          (-11.0, -2.2), (-9.5, DOOR_OUT_Y), (-7.6, -1.45), (-6.95, -0.6), (-6.9, 1.5), HAND, (-6.85, 6.6),
          (-6.1, 7.4), (X_EDGE, JUMP_Y), (0.0, JUMP_Y), (4.5, 7.6), (8.5, 8.6), (11.0, 11.0), VIEW, (17.5, 10.6),
          (19.6, 7.6), (19.4, 4.4), (18.0, 3.2)])
BARRELS = [(11.9, 9.5), (12.5, 10.1), (13.1, 9.6)]

# ---------------------------------------------------------------- motion
V_LOT, V_PLATFORM, V_RAMP, V_INSIDE = 9.0, 5.5, 6.5, 4.0   # inside is hidden: slow = time for the camera move
A_LAT, A_DEC, A_ACC = 12.0, 8.0, 6.0
HOLDS = {'pad': 1.8, 'hand': 1.8}       # in path order
G_JUMP, HOP_VZ, SLOWMO = 6.0, 1.2, 0.5
CZ_LOW, CZ_HIGH = 0.475, 0.62            # forklift carriage lift: fork tops level with the deck, then lifted


def track_tables():
    P = rk.catmull_rom_closed(TRACK, 60000)
    d = np.diff(np.vstack([P, P[:1]]), axis=0)
    seg = np.hypot(d[:, 0], d[:, 1])
    s = np.concatenate([[0], np.cumsum(seg)[:-1]])
    head = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
    total = 2 * math.pi * round((head[-1] - head[0]) / (2 * math.pi))
    dh = np.diff(np.concatenate([head, [head[0] + total]]))
    kappa = rk.circ_smooth(dh / np.maximum(seg, 1e-9), 80)
    return P, s, seg.sum(), head, kappa


def build_story():
    P, s, S, head, kappa = track_tables()
    near = lambda x, y: int(np.argmin(np.hypot(P[:, 0] - x, P[:, 1] - y)))  # noqa: E731
    i = {k: near(*xy) for k, xy in dict(foot=(RAMP_FOOT, RAMP_Y), top=(X_EDGE, RAMP_Y), door_in=(X_FRONT, DOOR_IN_Y),
                                        door_out=(X_FRONT, DOOR_OUT_Y), hand=HAND, edge=(X_EDGE, JUMP_Y),
                                        barrel=BARREL_PASS).items()}
    i['pad'] = 0
    sk = {k: s[v] for k, v in i.items()}

    v = np.minimum(V_LOT, np.sqrt(A_LAT / np.maximum(np.abs(kappa), 1e-6)))
    zone = np.full_like(s, V_LOT)
    zone[(s >= sk['foot'] - 1.5) & (s < sk['top'])] = V_RAMP
    zone[(s >= sk['top']) & (s < sk['door_in'])] = V_PLATFORM
    zone[(s >= sk['door_in']) & (s < sk['door_out'])] = V_INSIDE
    zone[(s >= sk['door_out']) & (s < sk['hand'])] = V_PLATFORM
    v = rk.circ_smooth(np.minimum(v, zone), 150)
    for name in HOLDS:
        v = np.minimum.reduce([v, np.sqrt(2 * A_DEC * ((sk[name] - s) % S)), np.sqrt(2 * A_ACC * ((s - sk[name]) % S))])
    # the hop off the dock: constant speed while airborne
    v_edge = max(float(v[i['edge']]), 2.0)
    t_air = (HOP_VZ + math.sqrt(HOP_VZ ** 2 + 2 * G_JUMP * DOCK_H)) / G_JUMP
    sk['land'] = sk['edge'] + v_edge * t_air
    air = (s >= sk['edge']) & (s < sk['land'])
    v[air] = v_edge

    ds = np.diff(np.concatenate([s, [S]]))
    tau_s = np.concatenate([[0], np.cumsum(ds / np.maximum(v, 0.03))[:-1]])
    hold_at = {}
    for name in HOLDS:                            # in path order
        j = i[name]
        hold_at[name] = (j, tau_s[j])
        tau_s[j + 1:] += HOLDS[name]
    T = tau_s[-1] + ds[-1] / max(v[-1], 0.03)
    # recompute arrival times (holds shift later points)
    arrive = {}
    for name, (j, _) in hold_at.items():
        arrive[name] = tau_s[j] if name != 'pad' else 0.0

    dt = 1 / 480
    tau = np.arange(0, T, dt)
    n = len(tau)
    st = {k: np.zeros(n) for k in ('x', 'y', 'z', 'roll', 'pitch', 'yaw', 'path_yaw', 's')}
    st['warp'] = np.ones(n)
    st['steer'] = {m.name: np.zeros(n) for m in rk.MODULES}
    st['wspeed'] = {m.name: np.zeros(n) for m in rk.MODULES}
    hold_ranges = {name: (arrive[name], arrive[name] + HOLDS[name]) for name in arrive}
    theta_ramp = math.atan2(DOCK_H, RAMP_FOOT - X_EDGE)

    for k, t in enumerate(tau):
        h, hold = None, None
        for name, (a0, a1) in hold_ranges.items():
            if a0 <= t < a1:
                h, hold = t - a0, name
        if hold:
            j, vv = i[hold], 0.0
        else:
            j = min(np.searchsorted(tau_s, t, side='right') - 1, len(s) - 1)
            vv = v[j]
        x, y = P[j]
        ps, kp, sj = head[j], kappa[j], s[j]
        st['path_yaw'][k], st['s'][k] = ps, sj
        on_lot = not (sk['foot'] - 1 <= sj < sk['land'] + 1)
        beta = float(np.clip(0.004 * vv * vv * kp, -0.35, 0.35)) if on_lot else 0.0     # fast corner = drift
        yaw, roll = ps + beta, -float(np.clip(vv * vv * kp * 0.005, -0.08, 0.08))
        jv = min(j + 60, len(v) - 1)
        a_long = (v[jv] - v[max(j - 60, 0)]) / max(120 * (S / len(s)) / max(vv, 0.5), 1e-3)
        pitch = float(np.clip(-0.010 * a_long, -0.05, 0.05))
        cmds = rk.module_commands(vv, vv * kp, rk.MODULES, 'front', rk.WHEEL_R)
        steer = {m: c[0] - (beta * 1.5 if m in ('fl', 'fr') else 0.0) for m, c in cmds.items()}
        wsp = {m: c[1] for m, c in cmds.items()}

        z = 0.0
        if sk['top'] <= sj < sk['edge']:
            z = DOCK_H
        elif sk['foot'] <= sj < sk['top']:
            q = (sj - sk['foot']) / (sk['top'] - sk['foot'])
            z = DOCK_H * q
            pitch = -theta_ramp * float(rk.smoothstep(q / 0.1) * (1 - rk.smoothstep((q - 0.9) / 0.1)))
        elif sk['edge'] <= sj < sk['land']:
            ta = (sj - sk['edge']) / v_edge
            q = ta / t_air
            z = DOCK_H + HOP_VZ * ta - 0.5 * G_JUMP * ta * ta
            pitch = -0.12 + 0.30 * q                               # little wheelie, nose down for landing
            roll = 0.08 * math.sin(2 * math.pi * q)
            wsp = {m: v_edge / rk.WHEEL_R for m in wsp}
            st['warp'][k] = SLOWMO
        elif sk['land'] <= sj < sk['land'] + 5:
            ta = (sj - sk['land']) / max(vv, 1.0)
            z = -0.07 * math.exp(-6 * ta) * math.cos(14 * ta)
            pitch = 0.12 * math.exp(-5 * ta) * math.cos(11 * ta)

        if hold:
            steer = {m: 0.0 for m in steer}
            wsp = {m: 0.0 for m in wsp}
            pitch = 0.05 * math.sin(math.pi * min(h / 0.35, 1.0))  # settles after braking
        st['x'][k], st['y'][k], st['z'][k] = x, y, z
        st['roll'][k], st['pitch'][k], st['yaw'][k] = roll, pitch, yaw
        for m in steer:
            st['steer'][m][k], st['wspeed'][m][k] = steer[m], wsp[m]

    st['warp'] = np.clip(rk.circ_smooth(st['warp'], 0.1 / dt), SLOWMO, 1.0)
    for m in st['steer']:
        st['steer'][m] = rk.circ_smooth(st['steer'][m], 0.06 / dt)
    st['wangle'] = {m: np.cumsum(st['wspeed'][m]) * dt for m in st['wspeed']}
    st['yaw'], st['path_yaw'] = np.unwrap(st['yaw']), np.unwrap(st['path_yaw'])
    st.update(tau=tau, dt=dt, T=T, sk=sk, holds=hold_ranges)
    # story times of path events (first time the robot passes each point after the pad hold)
    ev = {}
    for name in ('foot', 'top', 'door_in', 'door_out', 'edge', 'land', 'barrel'):
        ev[name] = float(tau[int(np.argmax((st['s'] >= sk[name]) & (tau > 0.5)))])
    ev.update(pad0=0.0, pad1=hold_ranges['pad'][1], hand0=hold_ranges['hand'][0], hand1=hold_ranges['hand'][1])
    st['ev'] = ev
    return st


# ---------------------------------------------------------------- world animation (story time, period T)
def ease(x):
    return float(rk.smoothstep(x))


def drone_state(tau, T):
    """(drone position, box A position or None if box is not on the drone)."""
    hover = Vector((PAD[0] + 0.15, PAD[1], 3.6))
    f_in, f_out = Vector((44.0, 24.0, 24.0)), Vector((-6.0, 46.0, 28.0))
    t = tau % T
    bob = Vector((0, 0, 0.04 * math.sin(2 * math.pi * t / 1.3)))
    if t >= T - 3.5:
        q = ease((t - (T - 3.5)) / 3.5)
        p = f_in.lerp(hover, q) + bob * q
        return p, p + Vector((0, 0, -0.45))
    if t < 1.3:
        q = ease((t - 0.15) / 1.15)
        box_z = (hover.z - 0.45) * (1 - q) + 0.615 * q
        return hover + bob, Vector((hover.x, hover.y, box_z))
    if t < 1.6:
        return hover + bob, None
    if t < 4.6:
        q = ease((t - 1.6) / 3.0)
        return hover.lerp(f_out, q), None
    return Vector((0, 0, 300)), None


def forklift1_state(tau, ev):
    """Bay 1 forklift: (x of centre, carriage lift). It waits half out of the truck (forks over the platform) so it
    is in the scene before it is needed, takes box B, backs deep into the truck, then rolls out to wait again."""
    t0 = ev['hand0']
    x_wait, x_out, x_in = -2.6, -5.1, 0.3
    if tau < t0 - 0.6 or tau >= t0 + 6.5:
        return x_wait, CZ_LOW
    if tau < t0 + 0.2:
        return x_wait + (x_out - x_wait) * ease((tau - (t0 - 0.6)) / 0.8), CZ_LOW
    if tau < t0 + 0.8:
        return x_out, CZ_LOW + (CZ_HIGH - CZ_LOW) * ease((tau - (t0 + 0.25)) / 0.45)
    if tau < t0 + 2.0:
        return x_out + (x_in - x_out) * ease((tau - (t0 + 0.8)) / 1.2), CZ_HIGH
    if tau < t0 + 5.0:
        return x_in, CZ_HIGH
    q = ease((tau - (t0 + 5.0)) / 1.5)
    return x_in + (x_wait - x_in) * q, CZ_HIGH + (CZ_LOW - CZ_HIGH) * q


def forklift2_state(tau, T):
    """Bay 2 forklift: shuttles crates from the truck into the warehouse twice per loop. (x, loaded)."""
    ph = (tau % (T / 2)) / (T / 2)
    x_truck, x_wh = 0.6, -10.8
    if ph < 0.15:
        return x_truck, True
    if ph < 0.5:
        return x_truck + (x_wh - x_truck) * ease((ph - 0.15) / 0.35), True
    if ph < 0.6:
        return x_wh, False
    return x_wh + (x_truck - x_wh) * ease((ph - 0.6) / 0.4), False


# ---------------------------------------------------------------- scene
def build_world(sc):
    T_ = W.textures()
    rng = np.random.default_rng(11)
    W.island(T_, lot=LOT, ring_r=RING_R)
    lamps = W.warehouse(T_, x_front=X_FRONT, robot_doors=(DOOR_IN_Y, DOOR_OUT_Y), truck_doors=(BAY1_Y, BAY2_Y))
    W.wedge('ramp', RAMP_FOOT, X_EDGE, RAMP_Y, DOCK_H, 2.0, T_['ramp'])
    yel = W.mat('hazard', 'ffc21a', 0.6)
    for side in (-1, 1):
        for xx in np.linspace(RAMP_FOOT - 0.3, X_EDGE + 0.3, 4):
            zb = DOCK_H * (RAMP_FOOT - xx) / (RAMP_FOOT - X_EDGE)
            W.rcyl('rail_post', (xx, RAMP_Y + side * 1.05, zb + 0.45), 0.035, 0.9, yel, 16, 0.015)
    W.lot_markings([(9.0, -13.5, 22, 0.14), (1.0, 12.5, 0.14, 4.0), (1.0, 2.5, 0.14, 4.0), (-1.5, 7.5, 5.0, 0.14)])
    W.drone_pad(*PAD)
    for c in CONES:
        W.cone(*c)
    for b in BARRELS:
        W.barrel(*b)
    W.pallet_stack(-7.2, -11.8, DOCK_H, 2)
    W.pallet_stack(-7.2, 12.6, DOCK_H, 1)
    # docked trucks (rear against the platform edge)
    # docked trucks: no orange here (it blended with the barrels); one orange truck circles the ring road instead
    t1, _ = W.truck('truck_bay1', '2f3a48', open_rear=True)
    t1.location = (X_EDGE + 0.05, BAY1_Y, 0)
    t2, _ = W.truck('truck_bay2', '3d73c2', open_rear=True)
    t2.location = (X_EDGE + 0.05, BAY2_Y, 0)
    # ring-road traffic
    ring = [W.truck(f'ring_truck{k}', c)[0:2] for k, c in enumerate((W.PAL['orange'], 'f2efe6', '7bb456'))]
    # trees, mini islands, clouds
    for _ in range(46):
        a = rng.uniform(0, 2 * math.pi)
        r = rng.uniform(26.0, 28.4) if rng.random() < 0.5 else rng.uniform(33.8, 35.2)
        x, y = r * math.cos(a), r * math.sin(a)
        if X_FRONT - 19 < x < 25 and -16 < y < 16:
            continue
        W.tree(x, y, rng.uniform(0.8, 1.35))
    for (x, y, sz) in [(-14, -21, 1.0), (-20, 20, 1.2), (12, -21, 0.9), (16, 20, 1.1), (-24, -4, 1.3), (5, -21, 1.0),
                       (2, 21, 1.2)]:
        W.tree(x, y, sz)
    for (x, y, z, r) in [(70, 30, -8, 7), (-60, 55, -5, 5), (40, -75, -10, 8), (-80, -35, -6, 6), (95, -20, -12, 9)]:
        W.mini_island(x, y, z, r, rng)
    for _ in range(14):
        a, d = rng.uniform(0, 2 * math.pi), rng.uniform(55, 120)
        W.cloud(d * math.cos(a), d * math.sin(a), rng.uniform(-14, 30), rng.uniform(0.8, 1.6), rng)
    W.sky(sc)
    W.sun(32, -125, 3.4, 'ffd9ad', 7)
    sc.view_settings.exposure = -0.35
    W.soft_look(sc, mist=(45, 240))
    return dict(lamps=lamps, ring=ring)


def scene_mission(a):
    sc = rk.reset_scene(a)
    sc.render.motion_blur_shutter = 0.5
    root, joints = rk.build_robot()
    W.restyle_robot()
    base = bpy.data.objects['base_link']
    W.plates(base)
    st = build_story()
    T, ev = st['T'], st['ev']
    world = build_world(sc)
    # box A: small express parcel from the drone; box B: big heavy crate-box, which is why the forklift takes it
    boxA = W.cardboard_box('boxA', (0.34, 0.30, 0.24), sticker=('EXPRESS', W.PAL['orange'], W.PAL['warm_white']))
    boxB = W.cardboard_box('boxB', (0.50, 0.42, 0.40), tint='9c7444',
                           sticker=('HEAVY  40 kg', W.PAL['ink'], W.PAL['warm_white']))
    fk1, car1, bm1 = W.forklift('forklift1')
    fk1.rotation_euler = (0, 0, math.pi)
    fk2, car2, bm2 = W.forklift('forklift2', 'ffb000')
    fk2.rotation_euler = (0, 0, math.pi)
    crate2 = W.cardboard_box('crate2', (0.5, 0.5, 0.45), 'b98b56')
    crate2.parent = car2
    crate2.location = (1.4, 0, 0.16)
    dr, _ = W.drone()

    # ---------------- story time <-> frame time ----------------
    t_frame = np.concatenate([[0], np.cumsum(st['dt'] / st['warp'])[:-1]])
    TF = t_frame[-1] + st['dt'] / st['warp'][-1]
    n = int(round(TF * a['fps']))
    sc.frame_start, sc.frame_end = 0, n - 1

    def tau_of(tf):
        loops = math.floor(tf / TF)
        i = min(np.searchsorted(t_frame, tf - loops * TF, side='right') - 1, len(t_frame) - 1)
        return i, loops

    def tf_of(tau):
        return float(t_frame[min(int(tau / st['dt']), len(t_frame) - 1)])

    E = {k: tf_of(v) for k, v in ev.items()}
    print(f'loop: story {T:.1f} s, film {TF:.1f} s, {n} frames @ {a["fps"]} fps')
    print('events (film s): ' + ', '.join(f'{k} {v:.2f}' for k, v in sorted(E.items(), key=lambda kv: kv[1])))

    def robot_at(tf):
        i, loops = tau_of(tf)
        yaw0 = 2 * math.pi * round((st['yaw'][-1] - st['yaw'][0]) / (2 * math.pi))
        pyaw0 = 2 * math.pi * round((st['path_yaw'][-1] - st['path_yaw'][0]) / (2 * math.pi))
        return dict(i=i, loops=loops, tau=st['tau'][i] + loops * T, x=st['x'][i], y=st['y'][i], z=st['z'][i],
                    roll=st['roll'][i], pitch=st['pitch'][i], yaw=st['yaw'][i] + loops * yaw0,
                    pyaw=st['path_yaw'][i] + loops * pyaw0)

    def local(r, dx, dy, dz, yaw=None):
        y_ = r['pyaw'] if yaw is None else yaw
        c, s_ = math.cos(y_), math.sin(y_)
        return Vector((r['x'] + c * dx - s_ * dy, r['y'] + s_ * dx + c * dy, r['z'] + dz))

    def smooth_robot(tf, win=0.25):
        ps = [robot_at(tf + d) for d in np.linspace(-win, win, 7)]
        return Vector((np.mean([p['x'] for p in ps]), np.mean([p['y'] for p in ps]),
                       np.mean([p['z'] for p in ps]) + 0.45))

    # ---------------- shots ----------------
    # Four continuous shots, three hard cuts (after the Beni reference: low cameras, foreground, moves that carry the
    # eye from one place to the next). A pose is (pos, target, lens mm, f-stop, roll deg); moves blend two poses.
    def P(pos, tgt, lens, fstop=4.0, roll=0.0):
        return (Vector(pos), Vector(tgt), lens, fstop, roll)

    def blend(p0, p1, q, lift=0.0):
        """Ease between two poses. lift raises the camera in the middle of the move (an arc instead of a line)."""
        pos = p0[0].lerp(p1[0], q) + Vector((0, 0, lift * math.sin(math.pi * q)))
        return (pos, p0[1].lerp(p1[1], q), *(x + (y - x) * q for x, y in zip(p0[2:], p1[2:])))

    def seg(tf, t0, t1):
        return ease((tf - t0) / max(t1 - t0, 1e-3))

    over_pos = Vector((PAD[0] + 2.4, PAD[1] - 2.8, 11.5))
    over_tgt = Vector((PAD[0] + 0.2, PAD[1], 0.4))
    OVER = P(over_pos, over_tgt, 38, 5.6)                     # the loop starts and ends on this exact frame

    # Shot A: overhead while the drone lowers box A, swoop down beside the pad, K.A.R.R. pulls away past the lens.
    cam_low = (PAD[0] - 2.6, PAD[1] + 1.0, 0.2)
    ta0, ta1 = 1.25, E['pad1'] + 0.2

    def shot_a(tf):
        low = P(cam_low, smooth_robot(tf, 0.12), 24, 4.0, -3)
        if tf < ta0:
            return OVER
        if tf < ta1:
            return blend(OVER, low, seg(tf, ta0, ta1))
        return low

    # Shot B: low follow from behind through the slalom, slight tilt (closer than v1).
    def shot_b(tf):
        r = robot_at(tf)
        return P(local(r, -2.2, 0.55, 0.42), local(r, 3.0, 0, 0.42), 26, 5.6, 6)

    # Shot C: locked-off wide of the ramp climb and the robot entering door A; while it is hidden inside the camera
    # flies to the platform facing door B, picks the robot up as it comes out and glides to the forklift handoff.
    WIDE = P((10.5, -18.5, 1.7), (-3.0, -8.0, 1.4), 30, 5.6)
    DOOR = P((-4.9, -4.3, DOCK_H + 0.55), (X_FRONT, DOOR_OUT_Y, DOCK_H + 0.8), 26)
    HANDV = P((-7.55, 2.3, DOCK_H + 1.15), (-6.75, BAY1_Y, DOCK_H + 0.75), 34, 2.8)
    tw0, tw1 = E['door_in'] + 0.1, E['door_out'] - 0.2
    tg0, tg1 = E['door_out'] + 0.5, E['hand0'] - 0.1

    def shot_c(tf):
        if tf < tw0:
            return WIDE
        if tf < tw1:
            return blend(WIDE, DOOR, seg(tf, tw0, tw1), lift=2.5)
        door_track = (DOOR[0], DOOR[1].lerp(smooth_robot(tf, 0.2), seg(tf, tw1, tw1 + 0.6)), *DOOR[2:])
        if tf < tg0:
            return door_track
        if tf < tg1:
            return blend(door_track, HANDV, seg(tf, tg0, tg1))
        return HANDV

    # Shot D: head-on between the trucks for the hop (slow motion), swing alongside after the landing, rise to an
    # aerial chase once past the barrels, and settle on the opening overhead as K.A.R.R. reaches the pad.
    td0 = E['land'] + 0.3
    td1 = td0 + 0.9
    td2 = E['barrel'] - 0.3
    td3 = td2 + 1.2

    def shot_d(tf):
        r = robot_at(tf)
        jump = P((5.0, JUMP_Y + 0.2, 0.32), smooth_robot(tf, 0.15), 30, 5.6)
        side = P(local(r, 1.0, -3.0, 0.55), local(r, 0.6, 0, 0.45), 28, 4.0)
        aerial = P(local(r, -5.0, -2.0, 8.5), local(r, 4.0, 0, 0.0), 30, 5.6)
        if tf < td0:
            return jump
        if tf < td1:
            return blend(jump, side, seg(tf, td0, td1))
        if tf < td2:
            return side
        if tf < td3:
            return blend(side, aerial, seg(tf, td2, td3))
        return blend(aerial, OVER, seg(tf, td3, TF))

    shots = [(0.0, shot_a), (E['pad1'] + 1.5, shot_b), (E['foot'] - 0.6, shot_c), (E['hand0'] + 1.6, shot_d)]
    print('shots (film s): ' + ', '.join(f'{f.__name__}@{t0:.2f}' for t0, f in shots))

    def shot_at(tf):
        tf = tf % TF
        cur = shots[0][1]
        for t0, f in shots:
            if tf >= t0:
                cur = f
        return cur

    def ring_pos(tau, k):
        """Ring-road truck k: one lap per loop, three trucks 120 deg apart, counter-clockwise in the outer lane."""
        ang = 2 * math.pi * ((tau / T) + k / 3.0)
        return ang, Vector((RING_LANE * math.cos(ang), RING_LANE * math.sin(ang), 0))

    # ---------------- bake ----------------
    cam = rk.link_obj(bpy.data.objects.new('camera', bpy.data.cameras.new('camera')))
    sc.camera = cam
    cam.data.clip_end = 600
    cam.data.dof.use_dof = True
    lamps = world['lamps']
    for f in range(-1, n + 1):
        shot = shot_at(f / a['fps'])
        for dsub in (-0.25, 0.0, 0.25):
            fr = f + dsub
            tf = fr / a['fps']
            # camera (always the shot of the whole frame f, so the blur never crosses a cut)
            pos, tgt, lens, fstop, roll = shot(tf)
            cam.location = pos
            rk.look_at(cam, tgt)
            if roll:      # dutch tilt about the view axis
                cam.rotation_quaternion = cam.rotation_quaternion @ Quaternion((0, 0, 1), math.radians(roll))
            cam.data.lens = lens
            cam.data.dof.aperture_fstop = fstop
            cam.data.dof.focus_distance = (Vector(tgt) - Vector(pos)).length
            rk.key(cam, 'location', 'rotation_quaternion', frame=fr)
            cam.data.keyframe_insert('lens', frame=fr)
            cam.data.dof.keyframe_insert('aperture_fstop', frame=fr)
            cam.data.dof.keyframe_insert('focus_distance', frame=fr)

            r = robot_at(tf)
            tau = r['tau']
            ts = tau % T
            root.location = (r['x'], r['y'], r['z'])
            root.rotation_euler = (r['roll'], r['pitch'], r['yaw'])
            rk.key(root, 'location', 'rotation_euler', frame=fr)
            for m in rk.MODULES:
                wa = st['wangle'][m.name]
                joints[f'{m.name}_steering'].rotation_euler = (0, 0, st['steer'][m.name][r['i']])
                joints[f'{m.name}_wheel'].rotation_euler = (0, 0, wa[r['i']] + r['loops'] * wa[-1])
                rk.key(joints[f'{m.name}_steering'], 'rotation_euler', frame=fr)
                rk.key(joints[f'{m.name}_wheel'], 'rotation_euler', frame=fr)
            deck_a = local(r, -0.15, 0, W.DECK_Z, r['yaw'])
            deck_b = local(r, -0.10, 0, W.DECK_Z, r['yaw'])

            # drone + box A
            dpos, dbox = drone_state(ts, T)
            dr.location = dpos
            dr.rotation_euler = (0, 0, 0.3 * math.sin(2 * math.pi * ts / T))
            rk.key(dr, 'location', 'rotation_euler', frame=fr)
            far = Vector((0, 0, -300))
            if dbox is not None:
                boxA.location, boxA.rotation_euler = dbox, (0, 0, math.pi)
            elif 1.3 <= ts < ev['door_in'] + 0.5:
                boxA.location, boxA.rotation_euler = deck_a, (r['roll'], r['pitch'], r['yaw'])
            else:
                boxA.location = far
            rk.key(boxA, 'location', 'rotation_euler', frame=fr)

            # forklift 1 + box B
            fx, cz = forklift1_state(ts, ev)
            fk1.location = (fx, BAY1_Y, DOCK_H)
            car1.location = (0, 0, cz)
            rk.key(fk1, 'location', frame=fr)
            rk.key(car1, 'location', frame=fr)
            lift = ev['hand0'] + 0.25
            if ev['door_out'] - 0.6 <= ts < lift:
                boxB.location, boxB.rotation_euler = deck_b, (r['roll'], r['pitch'], r['yaw'])
            elif lift <= ts < ev['hand0'] + 2.0:          # on the forks until it is deep inside truck 1
                hx, hy = HAND
                boxB.location = Vector((fx - (-5.1) + hx, hy - 0.10, DOCK_H + W.DECK_Z + (cz - CZ_LOW)))
                boxB.rotation_euler = (0, 0, math.pi / 2)
            else:
                boxB.location = far
            rk.key(boxB, 'location', 'rotation_euler', frame=fr)

            # forklift 2 shuttle, ring traffic, lamps
            x2, loaded = forklift2_state(ts, T)
            fk2.location = (x2, BAY2_Y, DOCK_H)
            crate2.scale = (1, 1, 1) if loaded else (0.001, 0.001, 0.001)
            rk.key(fk2, 'location', frame=fr)
            rk.key(crate2, 'scale', frame=fr)
            for k, (g, wheels) in enumerate(world['ring']):
                ang, p = ring_pos(tau, k)
                g.location = p
                g.rotation_euler = (0, 0, ang + math.pi / 2)
                rk.key(g, 'location', 'rotation_euler', frame=fr)
                spin = RING_LANE * 2 * math.pi * (tau / T) / 0.5
                for w in wheels:
                    w.rotation_euler = (0, spin, 0)
                    rk.key(w, 'rotation_euler', frame=fr)
            blink = 3.0 if math.sin(2 * math.pi * ts / (T / round(T / 1.2))) > 0.3 else 0.0
            for lm in lamps[:2]:
                lm.node_tree.nodes['Principled BSDF'].inputs['Emission Strength'].default_value = blink
                lm.node_tree.nodes['Principled BSDF'].inputs['Emission Strength'].keyframe_insert(
                    'default_value', frame=fr)
            for bm in (bm1, bm2):
                bm.node_tree.nodes['Principled BSDF'].inputs['Emission Strength'].default_value = \
                    4.0 if math.sin(2 * math.pi * ts * 1.5) > 0 else 0.5
                bm.node_tree.nodes['Principled BSDF'].inputs['Emission Strength'].keyframe_insert(
                    'default_value', frame=fr)
    return sc
