# Blender: render the exploded cartridge assembly.  blender -b --factory-startup --python rnd/render_parts.py -- out.png
import bpy, sys, math
from pathlib import Path
from mathutils import Vector
argv = sys.argv[sys.argv.index("--") + 1:]
OUT = argv[0]; SAMPLES = int(argv[1]) if len(argv) > 1 else 64
LIST = argv[2] if len(argv) > 2 else "asm_colors.txt"
TGT = tuple(float(v) for v in argv[3].split(",")) if len(argv) > 3 else (0.055, 0, 0.065)
OFF = tuple(float(v) for v in argv[4].split(",")) if len(argv) > 4 else (0.30, -0.36, 0.17)
d = Path(__file__).resolve().parent / "out"
bpy.ops.wm.read_factory_settings(use_empty=True)
scn = bpy.context.scene
def mat(n, c, r=0.4, metal=0.0):
    m = bpy.data.materials.new(n); m.use_nodes = True; b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (*c, 1); b.inputs["Roughness"].default_value = r; b.inputs["Metallic"].default_value = metal
    return m
M = {"white": mat("w", (0.62, 0.63, 0.66), 0.45), "dark": mat("d", (0.03, 0.03, 0.035), 0.4),
     "orange": mat("o", (0.95, 0.4, 0.07), 0.38), "steel": mat("s", (0.8, 0.8, 0.82), 0.15, 1.0)}
for line in (d / LIST).read_text().splitlines():
    n, c = line.split()
    bpy.ops.wm.stl_import(filepath=str(d / f"{n}.stl"))
    o = bpy.context.selected_objects[0]; o.scale = (0.001, 0.001, 0.001); o.data.materials.append(M[c])
    for f in o.data.polygons: f.use_smooth = True
    try: o.data.use_auto_smooth = True; o.data.auto_smooth_angle = math.radians(35)
    except AttributeError: pass
def area(loc, tgt, size, e):
    l = bpy.data.lights.new("a", "AREA"); l.size = size; l.energy = e
    o = bpy.data.objects.new("a", l); bpy.context.collection.objects.link(o); o.location = loc
    o.rotation_euler = (Vector(tgt) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
area((0.35, -0.3, 0.4), (0.05, 0, 0.06), 0.5, 22); area((-0.3, -0.1, 0.25), (0.05, 0, 0.06), 0.6, 9); area((0.1, 0.4, 0.1), (0.05, 0, 0.06), 0.5, 7)
scn.world = bpy.data.worlds.new("w"); scn.world.use_nodes = True
bg = scn.world.node_tree.nodes["Background"]; bg.inputs["Color"].default_value = (0.9, 0.91, 0.93, 1); bg.inputs["Strength"].default_value = 0.35
cam = bpy.data.cameras.new("c"); cam.lens = 60; co = bpy.data.objects.new("c", cam); bpy.context.collection.objects.link(co)
tgt = Vector(TGT); co.location = tgt + Vector(OFF); co.rotation_euler = (tgt - co.location).to_track_quat("-Z", "Y").to_euler()
scn.camera = co; scn.render.engine = "CYCLES"; scn.cycles.samples = SAMPLES; scn.cycles.use_denoising = False
scn.render.resolution_x, scn.render.resolution_y = 1400, 1000
scn.view_settings.view_transform = "AgX"; scn.render.filepath = OUT
bpy.ops.render.render(write_still=True)
