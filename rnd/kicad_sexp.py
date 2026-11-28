"""Minimal KiCad S-expression reader / writer and symbol-library helpers for rnd/kicad_board.py.

KiCad 7 files are S-expressions; this parses them into nested lists (quoted atoms as `Q`, a str subclass, so a dump
round-trips the quoting) and writes them back. `Lib` loads a .kicad_sym, flattens derived (`extends`) symbols the
way eeschema embeds them in a schematic, and gives each unit's pins (number, name, type, position, angle) and bbox.
"""
from __future__ import annotations

import copy
import re
from pathlib import Path

_TOK = re.compile(r'\s*(?:(\()|(\))|"((?:[^"\\]|\\.)*)"|([^\s()"]+))', re.S)


class Q(str):
    """A quoted string atom."""


def parse(text):
    stack, cur, pos, n = [], [], 0, len(text)
    while pos < n:
        m = _TOK.match(text, pos)
        if not m:
            if text[pos:].strip() == "":
                break
            raise ValueError(f"bad s-expression at {pos}: {text[pos:pos + 40]!r}")
        pos = m.end()
        if m.group(1):
            stack.append(cur)
            cur = []
        elif m.group(2):
            done, cur = cur, stack.pop()
            cur.append(done)
        elif m.group(3) is not None:
            cur.append(Q(m.group(3).replace('\\"', '"').replace("\\\\", "\\")))
        else:
            cur.append(m.group(4))
    return cur[0]


def q(s):
    """Quote a python string for a KiCad file."""
    return '"' + str(s).replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n") + '"'


def dump(node, ind=0):
    """Pretty-print: a list without sub-lists on one line, otherwise one element per line."""
    if not isinstance(node, list):
        return q(node) if isinstance(node, Q) else str(node)
    if all(not isinstance(x, list) for x in node):
        return "(" + " ".join(dump(x) for x in node) + ")"
    k = next(i for i, x in enumerate(node) if isinstance(x, list))
    s = "(" + " ".join(dump(x) for x in node[:k])
    for x in node[k:]:
        s += "\n" + "  " * (ind + 1) + dump(x, ind + 1)
    return s + "\n" + "  " * ind + ")"


def find(node, key):
    for x in node:
        if isinstance(x, list) and x and x[0] == key:
            return x
    return None


def find_all(node, key):
    return [x for x in node if isinstance(x, list) and x and x[0] == key]


def _num(x):
    return float(x)


class Lib:
    """One .kicad_sym library: flattened symbols, pins per unit, bounding boxes."""

    def __init__(self, path: Path, nick: str):
        self.path, self.nick = Path(path), nick
        root = parse(self.path.read_text())
        self.raw = {s[1]: s for s in find_all(root, "symbol")}

    def flat(self, name):
        """The symbol as eeschema embeds it in a schematic's lib_symbols: derived symbols flattened, named nick:name."""
        s = copy.deepcopy(self.raw[name])
        ext = find(s, "extends")
        if ext:
            parent = self.flat_local(ext[1])
            props = [x for x in s if isinstance(x, list) and x[0] == "property"]
            body = [x for x in parent if not (isinstance(x, list) and x[0] == "property")]
            out = [body[0], Q(name)]
            out += [x for x in body[2:] if not (isinstance(x, list) and x[0] == "symbol")]
            out += props
            for sub in find_all(parent, "symbol"):
                sub = copy.deepcopy(sub)
                sub[1] = Q(name + sub[1][len(ext[1]):])
                out.append(sub)
            s = out
        s[1] = Q(f"{self.nick}:{name}")
        return s

    def flat_local(self, name):
        s = self.flat(name)
        s[1] = Q(name)
        return s

    @staticmethod
    def units(sym):
        """{unit: {"pins": [...], "bbox": (x0, y0, x1, y1)}} in library coordinates (y up), style 1 only."""
        name = sym[1].split(":")[-1]
        subs = find_all(sym, "symbol")
        per = {}
        for sub in subs:
            m = re.match(re.escape(name) + r"_(\d+)_(\d+)$", sub[1])
            if not m:
                raise ValueError(f"unexpected sub-symbol {sub[1]} in {name}")
            u, st = int(m.group(1)), int(m.group(2))
            if st not in (0, 1):
                continue
            per.setdefault(u, []).append(sub)
        n_units = max([u for u in per if u > 0] or [1])
        out = {}
        for u in range(1, n_units + 1):
            pins, xs, ys = [], [], []
            for sub in per.get(0, []) + per.get(u, []):
                for g in sub[2:]:
                    if not isinstance(g, list):
                        continue
                    if g[0] == "pin":
                        at = find(g, "at")
                        ln = find(g, "length")
                        x, y, a = _num(at[1]), _num(at[2]), int(float(at[3])) if len(at) > 3 else 0
                        L = _num(ln[1]) if ln else 2.54
                        pins.append(dict(number=str(find(g, "number")[1]), name=str(find(g, "name")[1]), type=g[1],
                                         x=x, y=y, angle=a % 360, length=L, hidden="hide" in g))
                        xs += [x]
                        ys += [y]
                    elif g[0] in ("rectangle",):
                        for k in ("start", "end"):
                            p = find(g, k)
                            xs.append(_num(p[1]))
                            ys.append(_num(p[2]))
                    elif g[0] in ("polyline", "bezier"):
                        for p in find_all(find(g, "pts") or [], "xy"):
                            xs.append(_num(p[1]))
                            ys.append(_num(p[2]))
                    elif g[0] == "circle":
                        c, r = find(g, "center"), _num(find(g, "radius")[1])
                        xs += [_num(c[1]) - r, _num(c[1]) + r]
                        ys += [_num(c[2]) - r, _num(c[2]) + r]
                    elif g[0] == "arc":
                        for k in ("start", "mid", "end"):
                            p = find(g, k)
                            xs.append(_num(p[1]))
                            ys.append(_num(p[2]))
            if not xs:
                xs, ys = [0.0], [0.0]
            out[u] = dict(pins=pins, bbox=(min(xs), min(ys), max(xs), max(ys)))
        return out
