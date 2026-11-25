import bpy, bmesh, math, sys
from mathutils import Vector, Matrix
argv = sys.argv[sys.argv.index("--")+1:] if "--" in sys.argv else []
# usage: blender -b --factory-startup --python concept_mobile_manipulator.py -- out.png 128
OUT = argv[0] if argv else "concept_mobile_manipulator.png"
SAMPLES = int(argv[1]) if len(argv) > 1 else 96

bpy.ops.wm.read_factory_settings(use_empty=True)
scn = bpy.context.scene

def mat(name, color, rough=0.45, metal=0.0, emit=None, strength=4.0, coat=0.0):
    m = bpy.data.materials.new(name); m.use_nodes = True
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (*color, 1)
    b.inputs["Roughness"].default_value = rough
    b.inputs["Metallic"].default_value = metal
    if coat: b.inputs["Coat Weight"].default_value = coat
    if emit:
        b.inputs["Emission Color"].default_value = (*emit, 1)
        b.inputs["Emission Strength"].default_value = strength
    return m

WHITE = mat("white", (0.82, 0.83, 0.84), 0.38, coat=0.3)
DARK  = mat("dark", (0.025, 0.027, 0.03), 0.42)
ALU   = mat("alu", (0.55, 0.57, 0.6), 0.32, 1.0)
GLASS = mat("glass", (0.005, 0.006, 0.008), 0.06, coat=1.0)
ORANGE= mat("orange", (0.95, 0.38, 0.06), 0.4)
CYAN  = mat("cyan", (0.1, 0.8, 1.0), 0.3, emit=(0.15, 0.85, 1.0), strength=6)
AMBER = mat("amber", (1.0, 0.5, 0.1), 0.3, emit=(1.0, 0.45, 0.08), strength=5)
WOOD  = mat("wood", (0.42, 0.27, 0.15), 0.55)
FLOOR = mat("floor", (0.55, 0.56, 0.58), 0.6)
TOYS = [mat("t1", (0.85, 0.12, 0.1), 0.35), mat("t2", (0.1, 0.55, 0.2), 0.35),
        mat("t3", (0.95, 0.75, 0.1), 0.35), mat("t4", (0.12, 0.35, 0.85), 0.35)]
BIN = mat("bin", (0.85, 0.86, 0.88), 0.5)

def finish(o, m, bevel=0.0, seg=5, smooth=True):
    o.data.materials.append(m)
    if bevel:
        bv = o.modifiers.new("bev", "BEVEL"); bv.width = bevel; bv.segments = seg
        bv.limit_method = "ANGLE"; bv.harden_normals = False
    if smooth:
        for p in o.data.polygons: p.use_smooth = True
        o.data.use_auto_smooth = True; o.data.auto_smooth_angle = math.radians(35)
    return o

def rbox(name, size, loc, m, bevel=0.02, rot=(0,0,0)):
    me = bpy.data.meshes.new(name); bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    for v in bm.verts: v.co = Vector((v.co.x*size[0], v.co.y*size[1], v.co.z*size[2]))
    bm.to_mesh(me); bm.free()
    o = bpy.data.objects.new(name, me); bpy.context.collection.objects.link(o)
    o.location = loc; o.rotation_euler = rot
    return finish(o, m, bevel)

def cyl(name, r, depth, loc, m, rot=(0,0,0), bevel=0.004, verts=64, r2=None):
    if r2 is None:
        bpy.ops.mesh.primitive_cylinder_add(vertices=verts, radius=r, depth=depth, location=loc, rotation=rot)
    else:
        bpy.ops.mesh.primitive_cone_add(vertices=verts, radius1=r, radius2=r2, depth=depth, location=loc, rotation=rot)
    o = bpy.context.object; o.name = name
    return finish(o, m, bevel)

def torus(name, R, r, loc, m, rot=(0,0,0)):
    bpy.ops.mesh.primitive_torus_add(major_radius=R, minor_radius=r, location=loc, rotation=rot, major_segments=96, minor_segments=16)
    o = bpy.context.object; o.name = name; return finish(o, m)

def along(name, a, b, r1, r2, m, bevel=0.012):
    a, b = Vector(a), Vector(b); d = b - a
    q = Vector((0,0,1)).rotation_difference(d.normalized())
    o = cyl(name, r1, d.length, (a+b)/2, m, bevel=bevel, r2=r2)
    o.rotation_mode = "QUATERNION"; o.rotation_quaternion = q
    return o

def joint(name, c, axis, r=0.042, w=0.07, ring=CYAN):
    q = Vector((0,0,1)).rotation_difference(Vector(axis).normalized())
    o = cyl(name, r, w, c, DARK, bevel=0.006); o.rotation_mode="QUATERNION"; o.rotation_quaternion = q
    for s in (-1, 1):
        t = torus(name+"_ring%d"%s, r*0.78, 0.0032, Vector(c) + Vector(axis).normalized()*s*w/2, ring)
        t.rotation_mode="QUATERNION"; t.rotation_quaternion = q
    return o

# ---------------- base (holonomic, 3 omni wheels) ----------------
cyl("base", 0.25, 0.12, (0,0,0.105), WHITE, bevel=0.03, verts=96)
cyl("base_band", 0.253, 0.03, (0,0,0.075), DARK, bevel=0.008, verts=96)
torus("base_led", 0.251, 0.0035, (0,0,0.135), CYAN)
cyl("collar", 0.17, 0.13, (-0.05,0,0.225), WHITE, bevel=0.02, verts=96, r2=0.075)
for i in range(3):
    a = math.radians(90 + 120*i)
    c = (0.175*math.cos(a), 0.175*math.sin(a), 0.048)
    w = cyl("wheel%d"%i, 0.048, 0.034, c, DARK, rot=(math.pi/2, 0, a+math.pi/2), bevel=0.006)
cyl("base_top", 0.17, 0.02, (-0.02,0,0.17), DARK, bevel=0.006)

# ---------------- lift column ----------------
rbox("column", (0.085, 0.095, 0.99), (-0.06, 0, 0.165+0.495), ALU, 0.012)
rbox("column_back", (0.03, 0.10, 0.95), (-0.11, 0, 0.165+0.48), WHITE, 0.012)
rbox("lift_slot", (0.006, 0.03, 0.8), (-0.0165, 0, 0.66), DARK, 0.002)
rbox("column_cap", (0.1, 0.11, 0.03), (-0.07, 0, 1.165), DARK, 0.012)

# ---------------- torso carriage ----------------
TZ = 0.88
rbox("torso", (0.19, 0.30, 0.17), (0.0, 0, TZ), WHITE, 0.045)
rbox("torso_band", (0.006, 0.30, 0.022), (0.094, 0, TZ-0.035), DARK, 0.004)
rbox("chest_led", (0.003, 0.12, 0.004), (0.0975, 0, TZ-0.035), AMBER, 0.0015)

# ---------------- arms ----------------
def arm(side, upper_dir, fore_dir, grip_dir, holding=None):
    s = side
    S = Vector((0.0, s*0.205, TZ+0.03))
    joint("shoulder%d"%s, S + Vector((0, s*0.02, 0)), (0,1,0), r=0.05, w=0.06, ring=ORANGE)
    joint("shoulder_roll%d"%s, S + Vector((0, s*0.07, 0)), (1,0,0), r=0.043, w=0.075)
    u = Vector(upper_dir).normalized(); f = Vector(fore_dir).normalized(); g = Vector(grip_dir).normalized()
    A = S + Vector((0, s*0.07, 0))
    E = A + 0.26*u; W = E + 0.24*f
    along("upper%d"%s, A + 0.04*u, E - 0.035*u, 0.036, 0.029, WHITE)
    along("fore%d"%s, E + 0.035*f, W - 0.03*f, 0.03, 0.024, WHITE)
    ax = u.cross(f); ax = ax if ax.length > 1e-3 else Vector((0,1,0))
    joint("elbow%d"%s, E, ax, r=0.037, w=0.065)
    joint("wrist%d"%s, W, f.cross(ax), r=0.027, w=0.05)
    # gripper
    H = W + 0.05*g
    q = Vector((0,0,1)).rotation_difference(g)
    palm = rbox("palm%d"%s, (0.07, 0.04, 0.045), H, DARK, 0.01); palm.rotation_mode="QUATERNION"; palm.rotation_quaternion=q
    side_v = (q @ Vector((1,0,0)))
    for k in (-1, 1):
        fc = H + 0.055*g + k*0.022*side_v
        fo = rbox("finger%d%d"%(s,k), (0.012, 0.026, 0.07), fc, WHITE, 0.005); fo.rotation_mode="QUATERNION"; fo.rotation_quaternion=q
    cam = rbox("wristcam%d"%s, (0.022, 0.03, 0.018), W + 0.03*g + 0.035*(q @ Vector((0,1,0))), DARK, 0.004)
    cam.rotation_mode="QUATERNION"; cam.rotation_quaternion=q
    if holding:
        c = H + 0.07*g
        rbox("held", (0.045,0.045,0.045), c, holding, 0.006)
    return H + 0.07*g

held_at = arm(-1, (0.55,-0.12,-0.83), (0.96,0,-0.28), (0.62,0,-0.78), holding=TOYS[0])
arm(1, (0.35, 0.3, -0.89), (0.92, 0.25, 0.3), (0.95, 0.2, 0.2))

# ---------------- head ----------------
HZ = TZ + 0.205
cyl("neck", 0.032, 0.07, (0.0, 0, TZ+0.11), DARK, bevel=0.005)
rbox("head", (0.17, 0.25, 0.135), (0.01, 0, HZ), WHITE, 0.05)
rbox("visor", (0.02, 0.215, 0.095), (0.092, 0, HZ-0.002), GLASS, 0.03)
for s in (-1, 1):
    rbox("eye%d"%s, (0.006, 0.042, 0.022), (0.1025, s*0.052, HZ+0.004), CYAN, 0.009)
    cyl("tilt%d"%s, 0.03, 0.02, (0.01, s*0.128, HZ), DARK, rot=(math.pi/2,0,0), bevel=0.004)
rbox("depthcam", (0.02, 0.09, 0.018), (0.075, 0, HZ+0.08), DARK, 0.006)
for dy in (-0.032, 0.0, 0.032):
    cyl("lens%.2f"%dy, 0.0055, 0.004, (0.086, dy, HZ+0.08), GLASS, rot=(0, math.pi/2, 0), bevel=0.001)

# ---------------- props: side table + toy bin ----------------
tx, ty = held_at.x + 0.02, held_at.y - 0.02
rbox("tabletop", (0.42, 0.48, 0.03), (tx, ty, 0.30), WOOD, 0.006)
for dx in (-0.18, 0.18):
    for dy in (-0.21, 0.21):
        cyl("leg%.2f%.2f"%(dx,dy), 0.014, 0.285, (tx+dx, ty+dy, 0.1425), WOOD, bevel=0.002)
binz = 0.315
bpy.ops.mesh.primitive_cube_add(size=1, location=(tx, ty, binz+0.065))
b = bpy.context.object; b.scale = (0.26, 0.30, 0.13); bpy.ops.object.transform_apply(scale=True)
sol = b.modifiers.new("shell", "SOLIDIFY"); sol.thickness = 0.008
bm = bmesh.new(); bm.from_mesh(b.data)
top = [f for f in bm.faces if f.normal.z > 0.9]; bmesh.ops.delete(bm, geom=top, context="FACES"); bm.to_mesh(b.data); bm.free()
finish(b, BIN, 0.004)
for i, (dx, dy) in enumerate([(-0.06, -0.08), (0.05, 0.07), (0.07, -0.06)]):
    rbox("binToy%d"%i, (0.045,0.045,0.045), (tx+dx, ty+dy, binz+0.03), TOYS[i+1], 0.006, rot=(0,0,0.4*i))
for i, (x, y) in enumerate([(0.25, 0.62), (0.5, 0.78), (0.05, 0.55)]):
    rbox("floorToy%d"%i, (0.05,0.05,0.05), (x, y, 0.025), TOYS[i % 4], 0.007, rot=(0,0,0.7*i))
bpy.ops.mesh.primitive_uv_sphere_add(radius=0.045, location=(0.38, 0.5, 0.045)); finish(bpy.context.object, TOYS[3])

# ---------------- studio ----------------
bpy.ops.mesh.primitive_plane_add(size=1, location=(0,0,0))
fl = bpy.context.object
bm = bmesh.new()
bmesh.ops.create_grid(bm, x_segments=1, y_segments=40, size=1.0)
bm.free()
# cyclorama: floor that curves up into a back wall
me = bpy.data.meshes.new("cyc"); bm = bmesh.new()
W2 = 25.0; pts = []
for i in range(60):
    t = i/59
    if t < 0.6: x, z = 8.0 - t/0.6*9.5, 0.0
    else:
        a = (t-0.6)/0.4*math.pi/2; x, z = -1.5 - 1.0*math.sin(a), 1.0 - 1.0*math.cos(a)
    pts.append((x, z))
pts.append((-2.5, 4.0))
vs = [[bm.verts.new((x, y, z)) for (x, z) in pts] for y in (-W2, W2)]
for i in range(len(pts)-1):
    bm.faces.new((vs[0][i], vs[0][i+1], vs[1][i+1], vs[1][i]))
bm.to_mesh(me); bm.free()
cyc = bpy.data.objects.new("cyc", me); bpy.context.collection.objects.link(cyc); finish(cyc, FLOOR)
bpy.data.objects.remove(fl)

def area(name, loc, target, size, energy, color=(1,1,1)):
    l = bpy.data.lights.new(name, "AREA"); l.size = size; l.energy = energy; l.color = color
    o = bpy.data.objects.new(name, l); bpy.context.collection.objects.link(o); o.location = loc
    d = Vector(target) - Vector(loc); o.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()
area("key", (1.9, 2.0, 2.5), (0,0,0.7), 2.0, 260, (1.0, 0.96, 0.92))
area("fill", (2.4, -1.6, 1.2), (0,0,0.6), 2.5, 70, (0.9, 0.95, 1.0))
area("rim", (-1.4, -1.0, 2.2), (0,0,1.0), 1.2, 260)
scn.world = bpy.data.worlds.new("w"); scn.world.use_nodes = True
scn.world.node_tree.nodes["Background"].inputs["Color"].default_value = (0.22, 0.23, 0.25, 1)
scn.world.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.18

cam = bpy.data.cameras.new("cam"); cam.lens = 42
co = bpy.data.objects.new("cam", cam); bpy.context.collection.objects.link(co)
co.location = (2.35, 1.45, 1.05)
d = Vector((0.18, -0.1, 0.62)) - co.location; co.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()
scn.camera = co

scn.render.engine = "CYCLES"; scn.cycles.device = "CPU"; scn.cycles.samples = SAMPLES
scn.cycles.use_denoising = False
scn.cycles.sample_clamp_indirect = 3.0
scn.cycles.max_bounces = 6
scn.render.resolution_x, scn.render.resolution_y = 1600, 1200
scn.view_settings.view_transform = "AgX"; scn.view_settings.look = "AgX - Medium High Contrast"
scn.render.filepath = OUT
bpy.ops.render.render(write_still=True)
