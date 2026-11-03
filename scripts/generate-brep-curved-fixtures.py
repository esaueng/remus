#!/usr/bin/env python3
"""Write a synthetic analytic tube without running a CAD library."""

from math import tau
from pathlib import Path


def tube(version):
    curves2, curves3, surfaces, shapes = [], [], [], []

    def shape(kind, payload, children=()):
        shapes.append((kind, payload, children))
        return len(shapes) - 1

    def curve(table, text):
        table.append(text)
        return len(table)

    def edge(c3, first, last, reps, vertices):
        cached = "\n0 0 0 0" if version == 2 else ""
        text = f"1e-7 1 1 0\n1 {c3} 0 {first} {last}\n"
        text += "\n".join(rep + cached for rep in reps) + "\n0"
        return shape("Ed", text, vertices)

    def use(index, forward=True):
        return index, forward

    plane0 = curve(surfaces, "1 0 0 0 0 0 1 1 0 0 0 1 0")
    plane1 = curve(surfaces, "1 0 0 12 0 0 1 1 0 0 0 1 0")
    walls, bottoms, tops = [], [], []
    for radius in (5, 2):
        cylinder = curve(surfaces, f"2 0 0 0 0 0 1 1 0 0 0 1 0 {radius}")
        vertices = [shape("Ve", f"1e-7\n{radius} 0 {z}\n0 0") for z in (0, 12)]
        rims = []
        for z, vertex, plane in zip((0, 12), vertices, (plane0, plane1)):
            circle = curve(curves3, f"2 0 0 {z} 0 0 1 1 0 0 0 1 0 {radius}")
            cap = curve(curves2, f"2 0 0 1 0 0 1 {radius}")
            rim = curve(curves2, f"1 0 {z} 1 0")
            rims.append(edge(circle, 0, tau, [f"2 {cap} {plane} 0 0 {tau}",
                                           f"2 {rim} {cylinder} 0 0 {tau}"],
                             [use(vertex), use(vertex, False)]))
        line = curve(curves3, f"1 {radius} 0 0 0 0 1")
        left = curve(curves2, "1 0 0 0 1")
        right = curve(curves2, f"1 {tau} 0 0 1")
        # Space-separated continuity complements the glued token in real data.
        seam = edge(line, 0, 12, [f"3 {right} {left} CN {cylinder} 0 0 12"],
                    [use(vertices[0]), use(vertices[1], False)])
        wire = shape("Wi", "", [use(seam, False), use(rims[1], False),
                                  use(seam), use(rims[0])])
        walls.append(shape("Fa", f"0 1e-7 {cylinder} 0", [use(wire)]))
        bottoms.append(shape("Wi", "", [use(rims[0], radius == 5)]))
        tops.append(shape("Wi", "", [use(rims[1], radius == 5)]))
    cap0 = shape("Fa", f"0 1e-7 {plane0} 0", [use(bottoms[1]), use(bottoms[0])])
    cap1 = shape("Fa", f"0 1e-7 {plane1} 0", [use(tops[1]), use(tops[0])])
    shell = shape("Sh", "", [use(walls[0]), use(walls[1], False),
                                use(cap0, False), use(cap1)])
    shape("So", "", [use(shell)])
    text = f"CASCADE Topology V{version}, (c) Matra-Datavision\nLocations 0\n"
    for name, table in (("Curve2ds", curves2), ("Curves", curves3)):
        text += f"{name} {len(table)}\n" + "\n".join(table) + "\n"
    text += "Polygon3D 0\nPolygonOnTriangulations 0\n"
    text += f"Surfaces {len(surfaces)}\n" + "\n".join(surfaces) + "\nTriangulations 0\n"
    text += f"TShapes {len(shapes)}\n"
    for kind, payload, children in shapes:
        refs = " ".join(f"{'+' if forward else '-'}{len(shapes) - index} 0"
                        for index, forward in children)
        text += f"{kind}\n{payload}\n0101000\n{refs} *\n"
    return text + "+1 0\n"


if __name__ == "__main__":
    target = Path(__file__).resolve().parents[1] / "crates/io/tests/data/brep"
    for version in (1, 2, 3):
        (target / f"tube-v{version}.brep").write_text(tube(version))
