# Pip v4 hype video, step 2: Blender (EEVEE) shots from the STL scenes written by export_scenes.py.
#   blender -b --factory-startup --python rnd/video/render_shots.py -- MANIFEST OUT_DIR SHOT [first last] [pct]
# Writes OUT_DIR/SHOT/####.png (1920x1080 at pct=100). Shots: reveal, arm, kx, roll, hand, lift, glimpse, hero.
# Same scene recipe as rnd/render_robot.py (STL import at 1/1000, Principled materials by colour name), plus trailer
# lighting (dark world, coloured rims, bloom, reflective floor, DOF) and a per-frame python-driven animation.
import bpy, sys, math, json
from pathlib import Path
from mathutils import Vector, Euler, Matrix

argv = sys.argv[sys.argv.index("--") + 1:]
MAN, OUT, SHOT = json.loads(Path(argv[0]).read_text()), Path(argv[1]), argv[2]
F0 = int(argv[3]) if len(argv) > 3 and argv[3] else None
F1 = int(argv[4]) if len(argv) > 4 and argv[4] else None
PCT = int(argv[5]) if len(argv) > 5 else 100

bpy.ops.wm.read_factory_settings(use_empty=True)
scn = bpy.context.scene


# ---------------------------------------------------------------- materials
def mat(n, c, r=0.4, metal=0.0, emit=0.0, coat=0.0):
    m = bpy.data.materials.new(n); m.use_nodes = True; b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (*c, 1); b.inputs["Roughness"].default_value = r; b.inputs["Metallic"].default_value = metal
    if coat:
        b.inputs["Coat Weight"].default_value = coat; b.inputs["Coat Roughness"].default_value = 0.12
    if emit:
        b.inputs["Emission Color"].default_value = (*c, 1); b.inputs["Emission Strength"].default_value = emit
    return m


PAL = {"black": ((0.02, 0.021, 0.024), 0.30, 0.6, 0.0, 0.3), "white": ((0.72, 0.73, 0.75), 0.30, 0.0, 0.0, 0.5),
       "dark": ((0.035, 0.037, 0.042), 0.36, 0.2, 0.0, 0.2), "orange": ((1.0, 0.40, 0.05), 0.32, 0.0, 0.3, 0.3),
       "steel": ((0.62, 0.63, 0.66), 0.22, 1.0, 0.0, 0.0)}


def material_for(color, per_object=False, name="m"):
    c, r, metal, emit, coat = PAL.get(color, PAL["white"])
    if not per_object:
        key = "shared_" + color
        return bpy.data.materials.get(key) or mat(key, c, r, metal, emit, coat)
    return mat(name, c, r, metal, emit, coat)


def set_emit(m, color, strength):
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Emission Color"].default_value = (*color, 1); b.inputs["Emission Strength"].default_value = strength


# ---------------------------------------------------------------- import
def load(scene, per_object=False):
    objs = []
    for p in MAN[scene]:
        bpy.ops.wm.stl_import(filepath=p["file"])
        o = bpy.context.selected_objects[0]; o.name = p["name"]
        o.scale = (0.001, 0.001, 0.001)
        m = material_for(p["color"], per_object, "m_" + p["name"])
        if p["name"].startswith("eye") or p["name"].endswith("eye_l") or p["name"].endswith("eye_r"):
            m = bpy.data.materials.get("eyes") or mat("eyes", (1.0, 0.45, 0.08), 0.3, emit=12.0)
        o.data.materials.append(m)
        for f in o.data.polygons: f.use_smooth = True
        try: o.data.use_auto_smooth = True; o.data.auto_smooth_angle = math.radians(35)
        except AttributeError: pass
        o["lift"] = bool(p.get("lift")); o["color"] = p["color"]
        objs.append(o)
    bpy.context.view_layer.update()
    return objs


def wbbox(objs):
    lo = Vector((1e9,) * 3); hi = Vector((-1e9,) * 3)
    for o in objs:
        for c in o.bound_box:
            w = o.matrix_world @ Vector(c)
            lo = Vector(map(min, lo, w)); hi = Vector(map(max, hi, w))
    return lo, hi


# ---------------------------------------------------------------- stage
def area(loc, tgt, size, e, col=(1, 1, 1), name="a"):
    l = bpy.data.lights.new(name, "AREA"); l.size = size; l.energy = e; l.color = col
    o = bpy.data.objects.new(name, l); bpy.context.collection.objects.link(o); o.location = loc
    o.rotation_euler = (Vector(tgt) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
    return o


def spot(loc, tgt, e, col, size_deg=40, blend=0.6, name="s"):
    l = bpy.data.lights.new(name, "SPOT"); l.energy = e; l.color = col; l.spot_size = math.radians(size_deg); l.spot_blend = blend
    l.shadow_soft_size = 0.15
    o = bpy.data.objects.new(name, l); bpy.context.collection.objects.link(o); o.location = loc
    o.rotation_euler = (Vector(tgt) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
    return o


def world(col=(0.006, 0.008, 0.014), strength=1.0):
    scn.world = bpy.data.worlds.new("w"); scn.world.use_nodes = True
    bg = scn.world.node_tree.nodes["Background"]; bg.inputs["Color"].default_value = (*col, 1); bg.inputs["Strength"].default_value = strength


def floor(z=0.0, size=40, col=(0.012, 0.013, 0.016), rough=0.18):
    bpy.ops.mesh.primitive_plane_add(size=size, location=(0, 0, z)); fl = bpy.context.object
    fl.data.materials.append(mat("floor", col, rough, 0.0)); return fl


def camera(lens=50):
    cam = bpy.data.cameras.new("c"); cam.lens = lens; cam.clip_start = 0.01; cam.clip_end = 100
    co = bpy.data.objects.new("c", cam); bpy.context.collection.objects.link(co); scn.camera = co
    cam.dof.use_dof = True; cam.dof.aperture_fstop = 2.8
    return co


def aim(co, loc, tgt, focus=None, roll=0.0):
    co.location = Vector(loc)
    q = (Vector(tgt) - Vector(loc)).to_track_quat("-Z", "Y")
    co.rotation_euler = (q @ Euler((0, 0, roll)).to_quaternion()).to_euler() if roll else q.to_euler()
    co.data.dof.focus_distance = ((Vector(focus) if focus else Vector(tgt)) - Vector(loc)).length


def engine():
    scn.render.engine = "BLENDER_EEVEE"
    e = scn.eevee
    e.taa_render_samples = int(__import__("os").environ.get("HYPE_SAMPLES", "12"))
    e.use_bloom = True; e.bloom_threshold = 1.1; e.bloom_intensity = 0.025; e.bloom_radius = 5.0; e.bloom_knee = 0.5
    env = __import__("os").environ
    e.use_ssr = env.get("HYPE_SSR", "1") == "1"; e.use_ssr_refraction = False; e.ssr_quality = 0.5; e.ssr_thickness = 0.1
    e.use_gtao = True; e.gtao_distance = 0.15
    e.use_soft_shadows = env.get("HYPE_SOFT", "1") == "1"; e.shadow_cube_size = env.get("HYPE_CUBE", "1024"); e.shadow_cascade_size = "2048"
    e.use_gtao = env.get("HYPE_AO", "1") == "1"
    scn.render.resolution_x, scn.render.resolution_y, scn.render.resolution_percentage = 1920, 1080, PCT
    scn.view_settings.view_transform = "AgX"
    try: scn.view_settings.look = "AgX - Punchy"
    except TypeError: pass
    scn.render.image_settings.file_format = "PNG"; scn.render.image_settings.color_depth = "8"
    scn.render.film_transparent = False


# ---------------------------------------------------------------- easing
def clamp(x, a=0.0, b=1.0): return max(a, min(b, x))
def smooth(t): t = clamp(t); return t * t * (3 - 2 * t)
def ease_out_cubic(t): t = clamp(t); return 1 - (1 - t) ** 3
def ease_in_out(t): t = clamp(t); return 4 * t ** 3 if t < 0.5 else 1 - (-2 * t + 2) ** 3 / 2


def ease_out_back(t, s=2.2):
    t = clamp(t); c3 = s + 1
    return 1 + c3 * (t - 1) ** 3 + s * (t - 1) ** 2


def lerp(a, b, t): return a + (b - a) * t
def vlerp(a, b, t): return Vector(a).lerp(Vector(b), t)


def orbit(tgt, dist, az_deg, el_deg):
    a, e = math.radians(az_deg), math.radians(el_deg)
    return Vector(tgt) + dist * Vector((math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)))


# ---------------------------------------------------------------- shots
def module_shot(scene, n_frames, az0, az1, el0, el1, d0, d1, hold=24, stagger=5, dur=16, spread=1.0, spin=25,
                twist=None, lens=50, key_az=None, roll=0.0):
    """Exploded parts snap into place one by one (largest first), camera orbiting; returns the per-frame updater."""
    objs = load(scene, per_object=True)
    lo, hi = wbbox(objs); c = (lo + hi) / 2; size = (hi - lo).length
    for o in objs:                                     # recentre the module on the origin
        o.location -= c
    bpy.context.view_layer.update()
    world((0.004, 0.005, 0.009))
    co = camera(lens)
    k = size * 1.6
    area((k * 0.9, -k * 1.1, k * 1.2), (0, 0, 0), size * 1.2, 260 * size ** 2 * 1.6, (1.0, 0.93, 0.86), "key")
    area((-k * 1.2, k * 0.4, k * 0.5), (0, 0, 0), size * 0.8, 380 * size ** 2 * 1.6, (1.0, 0.45, 0.12), "rim_o")
    area((k * 0.2, k * 1.3, -k * 0.3), (0, 0, 0), size * 0.8, 320 * size ** 2 * 1.6, (0.25, 0.5, 1.0), "rim_b")
    area((0, 0, k * 1.6), (0, 0, 0), size, 90 * size ** 2 * 1.6, (0.8, 0.85, 1.0), "top")
    info = []
    vols = []
    for o in objs:
        l2, h2 = wbbox([o]); vols.append(((h2 - l2).x * (h2 - l2).y * (h2 - l2).z, o, (l2 + h2) / 2))
    vols.sort(key=lambda v: -v[0])
    for i, (_, o, pc) in enumerate(vols):
        d = pc.copy()
        if d.length < size * 0.08:
            d = Vector((0.15, -0.2, 1.0))
        d.normalize()
        dist = size * spread * (0.55 + 0.45 * (i % 3) / 2)
        info.append(dict(o=o, home=o.location.copy(), dir=d, dist=dist, start=hold + i * stagger, ax=d.orthogonal().normalized(),
                         rot0=o.rotation_euler.copy(), m=o.data.materials[0]))
    last = hold + (len(info) - 1) * stagger + dur

    def update(f):
        t = f / (n_frames - 1)
        for it in info:
            u = (f - it["start"]) / dur
            pre = clamp(-u * 0.02, 0, 0.3) if u < 0 else 0         # tiny outward drift while waiting
            e = ease_out_back(u) if u > 0 else 0.0
            off = it["dist"] * (1 - e) + it["dist"] * pre
            o = it["o"]
            o.location = it["home"] + it["dir"] * off
            ang = math.radians(spin) * (1 - e)
            o.rotation_euler = (Matrix.Rotation(ang, 3, it["ax"]) @ it["rot0"].to_matrix()).to_euler()
            # landing flash
            fl = max(0.0, 1 - abs(f - (it["start"] + dur * 0.55)) / 5.0)
            base_emit = 0.3 if o["color"] == "orange" else 0.0
            set_emit(it["m"], (1.0, 0.55, 0.2) if o["color"] != "white" else (1.0, 0.85, 0.7), base_emit + 3.5 * fl)
        if twist:                                           # (name, deg, frame0, frame1): the KX collar locks
            nm, deg, a, b = twist
            o = bpy.data.objects[nm]
            u = ease_out_back((f - a) / (b - a), 1.4)
            o.rotation_euler = Euler((0, 0, math.radians(deg) * (1 - u)))
        az = lerp(az0, az1, ease_in_out(t)); el = lerp(el0, el1, t); d = lerp(d0, d1, ease_out_cubic(t)) * size
        aim(co, orbit((0, 0, 0), d, az, el), (0, 0, 0), roll=math.radians(roll))
    return update, last


def join_static(objs):
    """Join parts by (material, lift flag): 234 objects -> ~12, which cuts EEVEE's per-frame sync time."""
    groups = {}
    for o in objs:
        groups.setdefault((o.data.materials[0].name, bool(o["lift"])), []).append(o)
    out = []
    for (mname, lift), g in groups.items():
        bpy.ops.object.select_all(action="DESELECT")
        for o in g: o.select_set(True)
        bpy.context.view_layer.objects.active = g[0]
        if len(g) > 1: bpy.ops.object.join()
        j = bpy.context.view_layer.objects.active; j["lift"] = lift; out.append(j)
    return out


HEAD = Vector((0.1, 0, 1.6))


def robot_stage(scene, dark=False):
    global HEAD
    objs = load(scene)
    HEAD = obj_center(["head_shell", "visor"])
    objs = join_static(objs)
    world((0.003, 0.004, 0.007) if dark else (0.006, 0.008, 0.014))
    floor()
    return objs


def obj_center(names):
    obs = [bpy.data.objects[n] for n in names if n in bpy.data.objects]
    lo, hi = wbbox(obs); return (lo + hi) / 2


def shot_reveal(n=120):
    robot_stage("robot", dark=True)
    head = HEAD
    co = camera(65)
    rim1 = area((-1.4, 1.6, 2.2), head, 1.0, 0, (1.0, 0.42, 0.08), "rimO")
    rim2 = area((-1.2, -1.8, 1.8), head, 1.0, 0, (0.3, 0.5, 1.0), "rimB")
    top = spot((0.6, -0.2, 3.2), (0.05, 0, 0.6), 0, (1.0, 0.95, 0.9), 35, 0.8, "top")
    eyes = bpy.data.materials.get("eyes")
    chest = bpy.data.materials.get("shared_orange")

    def update(f):
        t = f / (n - 1)
        rim1.data.energy = 700 * smooth(t / 0.35); rim2.data.energy = 450 * smooth((t - 0.15) / 0.35)
        top.data.energy = 1100 * smooth((t - 0.55) / 0.3)
        # eyes: off, a two-blink power-up at ~55 %, then full
        on = 0.0
        if t > 0.5: on = 1.0 if (t > 0.62 or 0.52 < t < 0.55) else 0.15
        set_emit(eyes, (1.0, 0.45, 0.08), 40.0 * on)
        set_emit(chest, (1.0, 0.40, 0.05), 0.15 + 0.9 * smooth((t - 0.6) / 0.3))
        d = lerp(2.9, 1.05, ease_in_out(t))
        tgt = vlerp((0.1, 0, 1.05), head, smooth(t * 1.2))
        aim(co, orbit(tgt, d, -18 + 10 * t, lerp(-4, 4, t)), tgt)
    return update


def shot_lift(n=240):
    objs = robot_stage("robot")
    lifts = [o for o in objs if o["lift"]]
    home = {o.name: o.location.copy() for o in lifts}
    co = camera(35)
    area((2.4, -1.8, 2.6), (0.1, 0, 0.7), 2.0, 380, (1.0, 0.92, 0.85), "key")
    area((-1.6, 1.5, 1.6), (0.1, 0, 0.8), 1.5, 600, (1.0, 0.42, 0.1), "rimO")
    area((-1.4, -1.7, 1.2), (0.1, 0, 0.8), 1.5, 450, (0.3, 0.5, 1.0), "rimB")

    def update(f):
        t = f / (n - 1)
        u = ease_in_out(clamp((t - 0.12) / 0.7))
        dz = lerp(-0.30, 0.0, u)
        for o in lifts:
            o.location = home[o.name] + Vector((0, 0, dz))
        tgt = Vector((0.1, 0, lerp(0.45, 0.85, u)))
        aim(co, orbit(tgt, lerp(2.3, 2.0, t), lerp(-70, -25, ease_in_out(t)), lerp(-8, 2, t)), tgt)
    return update


def shot_hero(n=300, scene="robot_hero"):
    robot_stage(scene)
    co = camera(40)
    area((2.6, -1.6, 2.8), (0.1, 0, 0.8), 2.0, 360, (1.0, 0.93, 0.86), "key")
    area((-1.8, 1.4, 1.8), (0.1, 0, 0.9), 1.2, 800, (1.0, 0.42, 0.1), "rimO")
    area((-1.5, -1.6, 1.4), (0.1, 0, 0.9), 1.2, 560, (0.3, 0.5, 1.0), "rimB")
    spot((0.3, 0.0, 3.4), (0.1, 0, 0.0), 1200, (1.0, 0.96, 0.9), 30, 0.7, "top")
    set_emit(bpy.data.materials["eyes"], (1.0, 0.45, 0.08), 40.0)

    def update(f):
        t = f / (n - 1)
        tgt = Vector((0.1, 0, lerp(0.75, 0.95, t)))
        aim(co, orbit(tgt, lerp(3.4, 2.5, ease_out_cubic(t)), lerp(-120, -40, ease_in_out(t)), lerp(-6, 6, t)), tgt)
    return update


GLIMPSE = [(-30, 0.95, 1.2, 8), (35, 1.3, 1.0, -5), (-100, 0.6, 1.4, 12), (150, 1.1, 1.6, 3), (-60, 0.25, 1.1, 20), (10, 0.8, 2.2, -2)]


def shot_glimpse(per=12):
    robot_stage("robot")
    co = camera(55)
    area((2.6, -1.6, 2.8), (0.1, 0, 0.8), 2.0, 360, (1.0, 0.93, 0.86), "key")
    area((-1.8, 1.4, 1.8), (0.1, 0, 0.9), 1.2, 800, (1.0, 0.42, 0.1), "rimO")
    area((-1.5, -1.6, 1.4), (0.1, 0, 0.9), 1.2, 560, (0.3, 0.5, 1.0), "rimB")
    set_emit(bpy.data.materials["eyes"], (1.0, 0.45, 0.08), 40.0)

    def update(f):
        g = GLIMPSE[(f // per) % len(GLIMPSE)]; u = (f % per) / per
        az, z, d, el = g
        tgt = Vector((0.12, 0, z))
        aim(co, orbit(tgt, d * lerp(1.0, 0.9, u), az + 6 * u, el), tgt)
    return update


# ---------------------------------------------------------------- main
engine()
N = {"reveal": 120, "arm": 180, "kx": 120, "roll": 120, "hand": 120, "lift": 240, "glimpse": 72, "hero": 300}[SHOT]
if SHOT == "reveal": upd = shot_reveal(N)
elif SHOT == "lift": upd = shot_lift(N)
elif SHOT == "hero": upd = shot_hero(N)
elif SHOT == "glimpse": upd = shot_glimpse()
elif SHOT == "arm": upd, _ = module_shot("arm", N, -70, 10, 14, 6, 3.0, 1.75, hold=18, stagger=3, dur=16, spread=0.35, lens=45, roll=-90)
elif SHOT == "kx": upd, _ = module_shot("kx", N, -40, 40, 28, 14, 3.8, 2.6, hold=14, stagger=5, dur=14, spread=1.0, twist=("collar", 35, 92, 108))
elif SHOT == "roll": upd, _ = module_shot("roll", N, 30, 110, 20, 10, 3.9, 2.8, hold=10, stagger=3, dur=14, spread=0.9)
elif SHOT == "hand": upd, _ = module_shot("hand", N, -150, -60, 15, 5, 3.9, 2.8, hold=12, stagger=5, dur=14, spread=0.9)
(OUT / SHOT).mkdir(parents=True, exist_ok=True)
STEP = int(__import__("os").environ.get("HYPE_STEP", "1"))
for f in range(F0 or 0, (F1 + 1) if F1 is not None else N, STEP):
    fp = OUT / SHOT / f"{f:04d}.png"
    if fp.exists() and __import__("os").environ.get("HYPE_RESUME", "1") == "1":
        continue                                   # resume an interrupted render
    upd(f)
    scn.render.filepath = str(fp)
    bpy.ops.render.render(write_still=True)
