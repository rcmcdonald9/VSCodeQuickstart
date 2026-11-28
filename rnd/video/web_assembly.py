"""Web build guide: the whole-robot CAD export as per-step binary meshes + a steps.json for an interactive three.js page.

    python rnd/video/web_assembly.py OUT_DIR [--src rnd/out] [--list asm_cmp_latestc.txt] [--steps rnd/video/web_steps.json]

Input: the robot export of rnd/parts/robot.py (export(..., name="cmp_latestc") with covers): one STL per part
(mm, z up, x forward, left arm at +y) and a list `<name> <colour>`. The step definitions (titles, the plain-language
text, the doc each line came from, which parts each step adds and the direction they come in from) live in
rnd/video/web_steps.json; this script resolves them against the CAD:

* every part must land in exactly one step (first matching pattern wins; unmatched parts are an error);
* "from" is the direction the parts start out in, relative to where the CAD puts them: a vector, "radial" or
  "-radial" (out from / in toward the robot's z axis through the group's centre). With "frame": "pitch" | "elbow" |
  "wrist" the vector is in that arm frame (robot.py: mirror y for the right arm, then the joint rotations of the
  exported pose, all about y), so cartridge moves run along the real joint axes;
  "{s}" in a pattern means both arms; the right arm's copy has y mirrored;
* each step is a list of sub-steps ("subs": a one- or two-sentence caption, its fasteners, a duration and its moves,
  with "t" local to the sub-step); the page plays one sub-step per Next. A part is hidden until its first move starts,
  so nested moves (motor into its can in one sub-step, the cartridge into the arm in a later one) read as a sequence;
* short moves are stretched to about one part-length (MIN_TRAVEL, MAX_STRETCH) along the same direction, and every
  sub-step gets a framing box (the moving parts at their start and end poses) for the page's camera;
* "ghost" lists earlier parts the page draws see-through during the step (the torso over the shoulder modules);
* parts listed under "split" are cut by a plane into two pieces (the printed torso halves are one solid in the CAD).

Meshes are not decimated: the CAD's facets are kept, vertices are split at creases sharper than CREASE so curved
faces shade smoothly and edges stay crisp, positions are quantised to int16 inside each part's bounding box
(≤ 0.02 mm), normals to int8, indices uint16. One file per step: models/sNN.txt (base64 of the binary layout). The page (web_assembly_page.html,
copied to OUT_DIR/index.html) decodes them; three.js 0.147.0 from jsDelivr.
"""
from __future__ import annotations

import fnmatch
import base64
import json
import math
import re
import sys
from pathlib import Path

import numpy as np
import trimesh
from trimesh import graph as tgraph
from trimesh import intersections as tint

ROOT = Path(__file__).resolve().parents[2]
MIN_TRAVEL, MAX_STRETCH = 60.0, 250.0   # mm: every move travels at least ~one part-length (≤ 250) and ≥ 60 mm
CREASE = math.radians(32)          # the smoothing angle (rnd/render_robot.py uses 35°)
PAGE = Path(__file__).with_name("web_assembly_page.html")


def _args(argv):
    out = Path(argv[0])
    opt = {"--src": ROOT / "rnd" / "out", "--list": "asm_cmp_latestc.txt", "--steps": ROOT / "rnd" / "video" / "web_steps.json"}
    for k in list(opt):
        if k in argv:
            opt[k] = argv[argv.index(k) + 1]
    return out, Path(opt["--src"]), opt["--list"], Path(opt["--steps"])


def load_parts(src: Path, lst: str):
    prefix = lst.rsplit(".", 1)[0] + "_"
    parts = []
    for line in (src / lst).read_text().splitlines():
        if not line.strip():
            continue
        n, c = line.split()
        m = trimesh.load(src / f"{n}.stl", force="mesh")
        parts.append(dict(name=n[len(prefix):] if n.startswith(prefix) else n, colour=c, mesh=m))
    return parts


def split_parts(parts, splits):
    out = []
    for p in parts:
        sp = next((s for s in splits if s["part"] == p["name"]), None)
        if sp is None:
            out.append(p)
            continue
        nrm = np.array(sp["normal"], float)
        org = np.array(sp.get("origin", [0, 0, 0]), float)
        a = tint.slice_mesh_plane(p["mesh"], nrm, org)
        b = tint.slice_mesh_plane(p["mesh"], -nrm, org)
        out.append(dict(name=sp["into"][0], colour=p["colour"], mesh=a))
        out.append(dict(name=sp["into"][1], colour=p["colour"], mesh=b))
    return out


def unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def rot_y(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


FRAMES = {"pitch": ("pitch",), "elbow": ("pitch", "elbow"), "wrist": ("pitch", "elbow", "wrist")}


def to_world(v, side, frame, poses):
    """A direction given in an arm frame (robot.py: mirror(y) for the right arm, then the pitch / elbow / wrist
    rotations, all about y while roll and yaw are 0 as in the export) -> world."""
    v = np.array(v, float) * [1, -1.0 if side == "r" else 1.0, 1]
    if frame:
        v = rot_y(sum(poses[side or "l"][k] for k in FRAMES[frame])) @ v
    return v


def resolve(steps_def, parts, poses):
    names = [p["name"] for p in parts]
    by_name = {p["name"]: p for p in parts}
    step_of = {}
    for si, st in enumerate(steps_def):
        pats = []
        for pat in st["parts"]:
            pats += [pat.replace("{s}", s) for s in ("l", "r")] if "{s}" in pat else [pat]
        for n in names:
            if n not in step_of and any(fnmatch.fnmatchcase(n, pat) for pat in pats):
                step_of[n] = si
    missing = [n for n in names if n not in step_of]
    if missing:
        raise SystemExit(f"parts in no step: {missing}")
    steps = []
    for si, st in enumerate(steps_def):
        mine = [n for n in names if step_of[n] == si]
        moved = set()
        moves = []
        subs = []
        for ki, sub in enumerate(st["subs"]):
            focus = []                                            # the parts this sub-step moves (camera framing)
            for mv in sub["moves"]:
                sides = ("l", "r") if any("{s}" in p for p in mv["parts"]) else (None,)
                for side in sides:
                    pats = [p.replace("{s}", side) if side else p for p in mv["parts"]]
                    sel = [n for n in mine if any(fnmatch.fnmatchcase(n, p) for p in pats)]
                    if not sel:
                        raise SystemExit(f"step {si + 1}.{ki + 1}: move {mv['parts']} ({side}) matches nothing in the step")
                    t0, t1 = mv.get("t", (0.0, 1.0))
                    if side == "r":
                        t0, t1 = t0 + mv.get("rdelay", 0.0), t1 + mv.get("rdelay", 0.0)
                    groups = {}
                    if "per" in mv:                               # one vector per regex group (e.g. per wheel module)
                        for n in sel:
                            k = re.match(mv["per"], n)
                            groups.setdefault(k.group(1) if k else n, []).append(n)
                    else:
                        groups[None] = sel
                    frame = mv.get("frame")
                    for gi, g in enumerate(groups.values()):
                        f = mv["from"]
                        bb = np.vstack([by_name[n]["mesh"].bounds for n in g])
                        lo, hi = bb.min(axis=0), bb.max(axis=0)
                        if f in ("radial", "-radial"):
                            c = (lo + hi) / 2
                            v = unit([c[0], c[1], 0.0]) * (1 if f == "radial" else -1)
                        else:
                            v = unit(to_world(f, side, frame, poses))
                        dist = mv["dist"]
                        if dist > 0:                              # travel at least about one part-length (capped), so it reads on a phone
                            corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
                            ext = float(np.ptp(corners @ v))
                            dist = max(dist, min(ext, MAX_STRETCH), MIN_TRAVEL)
                        dt = mv.get("stagger", 0.0) * gi
                        moves.append(dict(p=[names.index(n) for n in g], v=[round(float(x), 2) for x in v * dist], s=ki,
                                          t=[round(t0 + dt, 3), round(min(1.0, t1 + dt), 3)]))
                        moved.update(g)
                        focus += g
            subs.append(dict({k: v for k, v in sub.items() if k != "moves"}, dur=sub.get("dur", 2.0), focus=focus))
        still = [n for n in mine if n not in moved]
        if still:
            raise SystemExit(f"step {si + 1} ({st['title']}): parts with no move: {still}")
        # framing box per sub-step: the moving parts (left arm only when both arms move) at their start and end poses
        for ki, sub in enumerate(subs):
            foc = sub.pop("focus")
            if any(n.startswith("l_") for n in foc):
                foc = [n for n in foc if not n.startswith("r_")]
            boxes = []
            for n in foc:
                i = names.index(n)
                lo, hi = by_name[n]["mesh"].bounds
                for first in (ki, ki + 1):                        # offset at the start (moves from this sub on) and the end
                    off = np.zeros(3)
                    for m in moves:
                        if i in m["p"] and m["s"] >= first:
                            off += m["v"]
                    boxes += [lo + off, hi + off]
            if not boxes:                                         # the finished robot: the whole thing
                boxes = [b for p in parts for b in p["mesh"].bounds]
            bx = np.array(boxes)
            sub["box"] = [[round(float(x), 1) for x in bx.min(axis=0)], [round(float(x), 1) for x in bx.max(axis=0)]]
        ghost = [names.index(n) for n in names if step_of[n] < si and any(fnmatch.fnmatchcase(n, p) for p in st.get("ghost", []))]
        steps.append(dict(parts=[names.index(n) for n in mine], moves=moves, ghost=ghost, subs=subs))
    return step_of, steps


def crease_split(mesh):
    """trimesh.graph.smooth_shade with a different flat-face rule: faces join a smoothing patch across edges flatter
    than CREASE, but a coplanar facet of 3 or more triangles (a real flat CAD face, not one quad of a tessellated
    cylinder) stays its own patch, so flat faces next to a fillet don't pick up shading streaks."""
    if len(mesh.face_adjacency) == 0:
        return mesh.copy()
    adj = mesh.face_adjacency[mesh.face_adjacency_angles < CREASE]
    facets = [f for f in mesh.facets if len(f) >= 3]
    nodes = None
    if facets:
        mask = np.ones(len(mesh.faces), dtype=bool)
        mask[np.hstack(facets)] = False
        adj = adj[mask[adj].all(axis=1)]
        nodes = np.unique(adj)
    comps = tgraph.connected_components(adj, min_len=2, nodes=nodes) + list(facets)
    seen = np.unique(np.hstack(comps)) if comps else np.zeros(0, int)
    rest = np.setdiff1d(np.arange(len(mesh.faces)), seen)
    comps += list(rest.reshape((-1, 1)))
    return mesh.submesh(comps, only_watertight=False, append=True)


def encode(mesh):
    """Crease-split, quantised: (int16 positions, int8 normals, uint16 indices, bbox min, bbox max)."""
    m = crease_split(mesh)
    v = np.asarray(m.vertices, float)
    f = np.asarray(m.faces, np.int64)
    n = np.asarray(m.vertex_normals, float)
    if len(v) > 65535:
        raise SystemExit("part over 65535 vertices: split it or use uint32")
    lo, hi = v.min(axis=0), v.max(axis=0)
    span = np.where(hi - lo > 1e-9, hi - lo, 1.0)
    q = np.round((v - lo) / span * 65535.0) - 32768.0
    nq = np.clip(np.round(n * 127.0), -127, 127)
    return q.astype("<i2"), nq.astype("i1"), f.astype("<u2"), lo, hi


def pad4(b: bytes) -> bytes:
    return b + b"\0" * (-len(b) % 4)


def main(argv):
    out, src, lst, steps_path = _args(argv)
    sd = json.loads(steps_path.read_text())
    parts = split_parts(load_parts(src, lst), sd.get("split", []))
    step_of, steps = resolve(sd["steps"], parts, sd["poses"])
    (out / "models").mkdir(parents=True, exist_ok=True)
    for old in list((out / "models").glob("s*.bin")) + list((out / "models").glob("s*.txt")):
        old.unlink()
    manifest = []
    blobs = {si: bytearray() for si in range(len(steps))}
    tris = 0
    for i, p in enumerate(parts):
        si = step_of[p["name"]]
        q, nq, f, lo, hi = encode(p["mesh"])
        buf = blobs[si]
        o = len(buf)
        buf += pad4(q.tobytes()) + pad4(nq.tobytes()) + pad4(f.tobytes())
        tris += len(f)
        manifest.append(dict(n=p["name"], c=p["colour"], s=si, o=o, nv=int(len(q)), ni=int(f.size),
                             lo=[round(float(x), 3) for x in lo], hi=[round(float(x), 3) for x in hi]))
    files = []
    for si, b in blobs.items():
        if not b:                                         # a step that adds no parts (the finished robot)
            files.append(dict(path=None, bytes=0))
            continue
        fn = f"models/s{si + 1:02d}.txt"                  # base64 text: artifacts don't serve raw .bin
        txt = base64.b64encode(bytes(b)).decode("ascii")
        (out / fn).write_text(txt)
        files.append(dict(path=fn, bytes=len(txt)))
    lo = np.min([p["mesh"].bounds[0] for p in parts], axis=0)
    hi = np.max([p["mesh"].bounds[1] for p in parts], axis=0)
    doc = dict(
        meta=dict(sd.get("meta", {}), source=lst, parts=len(parts), triangles=int(tris),
                  bounds=[[round(float(x), 1) for x in lo], [round(float(x), 1) for x in hi]]),
        colours=sd["colours"], files=files, parts=manifest,
        steps=[dict({k: v for k, v in st.items() if k not in ("parts", "subs")}, **r) for st, r in zip(sd["steps"], steps)])
    (out / "steps.json").write_text(json.dumps(doc, separators=(",", ":")))
    (out / "index.html").write_text(PAGE.read_text())                  # page content only: the host adds the skeleton
    print(f"{len(parts)} parts, {tris} triangles, {len(steps)} steps -> {out}")
    for si, b in blobs.items():
        print(f"  s{si + 1:02d} {len(b) / 1e6:6.2f} MB  {sd['steps'][si]['title']}")


if __name__ == "__main__":
    main(sys.argv[1:])
