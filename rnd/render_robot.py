# Blender: render the whole-robot assembly.  blender -b --factory-startup --python rnd/render_robot.py -- out.png [samples] [list] [cam az deg]
import bpy, sys, math
from pathlib import Path
from mathutils import Vector
argv = sys.argv[sys.argv.index("--") + 1:]
OUT = argv[0]; SAMPLES = int(argv[1]) if len(argv) > 1 else 64
LIST = argv[2] if len(argv) > 2 else "asm_robot.txt"
AZ = math.radians(float(argv[3])) if len(argv) > 3 else math.radians(-38)
d = Path(__file__).resolve().parent / "out"
bpy.ops.wm.read_factory_settings(use_empty=True)
scn = bpy.context.scene


def mat(n, c, r=0.4, metal=0.0, emit=0.0):
    m = bpy.data.materials.new(n); m.use_nodes = True; b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (*c, 1); b.inputs["Roughness"].default_value = r; b.inputs["Metallic"].default_value = metal
    if emit:
        b.inputs["Emission Color"].default_value = (*c, 1); b.inputs["Emission Strength"].default_value = emit
    return m


M = {"black": mat("k", (0.02, 0.021, 0.024), 0.32, 0.6), "white": mat("w", (0.80, 0.81, 0.83), 0.28), "dark": mat("d", (0.035, 0.037, 0.042), 0.38),
     "orange": mat("o", (1.0, 0.42, 0.06), 0.35, emit=0.6), "steel": mat("s", (0.55, 0.56, 0.58), 0.3, 0.8)}
for line in (d / LIST).read_text().splitlines():
    n, c = line.split()
    bpy.ops.wm.stl_import(filepath=str(d / f"{n}.stl"))
    o = bpy.context.selected_objects[0]; o.scale = (0.001, 0.001, 0.001); o.data.materials.append(M[c])
    for f in o.data.polygons: f.use_smooth = True
    try: o.data.use_auto_smooth = True; o.data.auto_smooth_angle = math.radians(35)
    except AttributeError: pass
# floor
bpy.ops.mesh.primitive_plane_add(size=12); fl = bpy.context.object
fl.data.materials.append(mat("floor", (0.42, 0.43, 0.45), 0.7))


def area(loc, tgt, size, e):
    l = bpy.data.lights.new("a", "AREA"); l.size = size; l.energy = e
    o = bpy.data.objects.new("a", l); bpy.context.collection.objects.link(o); o.location = loc
    o.rotation_euler = (Vector(tgt) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()


tgt = Vector((0.10, 0, 0.64))
area((2.2, -1.6, 2.6), tgt, 2.5, 380); area((-1.8, -1.2, 1.6), tgt, 2.5, 110); area((0.2, 2.2, 2.0), tgt, 2.0, 140)
scn.world = bpy.data.worlds.new("w"); scn.world.use_nodes = True
bg = scn.world.node_tree.nodes["Background"]; bg.inputs["Color"].default_value = (0.86, 0.88, 0.91, 1); bg.inputs["Strength"].default_value = 0.25
cam = bpy.data.cameras.new("c"); cam.lens = 50; co = bpy.data.objects.new("c", cam); bpy.context.collection.objects.link(co)
dist = 2.35
ELEV = float(argv[4]) if len(argv) > 4 else 0.55
co.location = tgt + Vector((dist * math.cos(AZ), dist * math.sin(AZ), ELEV))
co.rotation_euler = (tgt - co.location).to_track_quat("-Z", "Y").to_euler()
scn.camera = co; scn.render.engine = "CYCLES"; scn.cycles.samples = SAMPLES; scn.cycles.use_denoising = False
scn.render.resolution_x, scn.render.resolution_y = 1100, 1400
scn.view_settings.view_transform = "AgX"; scn.render.filepath = OUT
bpy.ops.render.render(write_still=True)
