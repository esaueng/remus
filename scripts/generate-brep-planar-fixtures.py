#!/usr/bin/env python3
"""Write synthetic, straight-edged V1 solids from explicit oriented polygons.

No CAD library is used. Dimensions and closed-form expectations are recorded in
crates/io/tests/data/brep/README.md. Re-running this script is deterministic.
"""

from math import sqrt
from pathlib import Path


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def unit(a):
    length = sqrt(sum(x * x for x in a))
    return tuple(x / length for x in a)


def numbers(a):
    return " ".join(format(x, ".17g") for x in a)


def ring(points, z):
    return [(x, y, z) for x, y in points]


def walls(points, low, high):
    return [[[(*a, low), (*b, low), (*b, high), (*a, high)]]
            for a, b in zip(points, points[1:] + points[:1])]


def prism(outer, holes, high=8, floor=0):
    bottom = [list(reversed(ring(outer, 0)))]
    if floor == 0:
        bottom.extend(list(reversed(ring(hole, 0))) for hole in holes)
    top = [ring(outer, high), *(ring(hole, high) for hole in holes)]
    faces = [bottom, top, *walls(outer, 0, high)]
    for hole in holes:
        faces.extend(walls(hole, floor, high))
        if floor:
            faces.append([list(reversed(ring(hole, floor)))])
    return faces


def write(name, faces):
    vertices = dict.fromkeys(p for loops in faces for loop in loops for p in loop)
    vertices = {p: i for i, p in enumerate(vertices)}
    edges = {}
    for loops in faces:
        for loop in loops:
            for a, b in zip(loop, loop[1:] + loop[:1]):
                key = tuple(sorted((vertices[a], vertices[b])))
                edges.setdefault(key, len(edges))
    points = list(vertices)
    curves = []
    pcurves = []
    surfaces = []
    edge_pcurves = [[] for _ in edges]
    for a, b in edges:
        curves.append("1 " + numbers((*points[a], *unit(sub(points[b], points[a])))))
    for surface, loops in enumerate(faces, start=1):
        p, q, r = loops[0][:3]
        x = unit(sub(q, p))
        normal = unit(cross(sub(q, p), sub(r, q)))
        y = cross(normal, x)
        surfaces.append("1 " + numbers((*p, *normal, *x, *y)))
        for loop in loops:
            for a, b in zip(loop, loop[1:] + loop[:1]):
                key = tuple(sorted((vertices[a], vertices[b])))
                a, b = (points[index] for index in key)
                delta = sub(a, p)
                direction = unit(sub(b, a))
                uv = tuple(sum(v * axis for v, axis in zip(delta, basis)) for basis in (x, y))
                duv = tuple(sum(v * axis for v, axis in zip(direction, basis)) for basis in (x, y))
                pcurves.append("1 " + numbers((*uv, *duv)))
                length = sqrt(sum(v * v for v in sub(b, a)))
                edge_pcurves[edges[key]].append(f"2 {len(pcurves)} {surface} 0 0 {length:.17g}")
    # All vertices precede edges, all edges precede wires, etc. The persisted
    # references count backwards from the final table size, as V1 requires.
    shapes = []

    def shape(kind, data, children):
        index = len(shapes)
        shapes.append((kind, data, children))
        return index

    vertex_ids = [shape("Ve", f"1e-07\n{numbers(p)}\n0 0", []) for p in points]
    edge_ids = []
    for (a, b), edge in edges.items():
        length = sqrt(sum(v * v for v in sub(points[b], points[a])))
        data = f"1e-07 1 1 0\n1 {edge + 1} 0 0 {length:.17g}\n"
        data += "\n".join(edge_pcurves[edge]) + "\n0"
        edge_ids.append(shape("Ed", data, [(vertex_ids[a], True), (vertex_ids[b], False)]))
    face_ids = []
    for surface, loops in enumerate(faces, start=1):
        wire_ids = []
        for loop in loops:
            children = []
            for a, b in zip(loop, loop[1:] + loop[:1]):
                key = tuple(sorted((vertices[a], vertices[b])))
                children.append((edge_ids[edges[key]], vertices[a] == key[0]))
            # Wire records are sets of connected oriented uses, not ordered
            # polygon arrays. Deliberately shuffle them to exercise traversal.
            children = children[::2] + children[1::2]
            wire_ids.append(shape("Wi", "", children))
        # The perimeter is deliberately last on a face with holes.
        children = [(wire, True) for wire in wire_ids[1:] + wire_ids[:1]]
        face_ids.append(shape("Fa", f"0 1e-07 {surface} 0", children))
    shell = shape("Sh", "", [(face, True) for face in face_ids])
    shape("So", "", [(shell, True)])
    total = len(shapes)
    output = ["CASCADE Topology V1, (c) Matra-Datavision", "Locations 0",
              f"Curve2ds {len(pcurves)}", *pcurves, f"Curves {len(curves)}", *curves,
              "Polygon3D 0", "PolygonOnTriangulations 0", f"Surfaces {len(surfaces)}",
              *surfaces, "Triangulations 0", f"TShapes {total}"]
    for kind, data, children in shapes:
        output.extend([kind, data, "0101000",
                       " ".join(f"{'+' if forward else '-'}{total - child} 0"
                                for child, forward in children) + " *"])
    output.append("+1 0")
    destination = Path(__file__).resolve().parents[1] / "crates/io/tests/data/brep" / f"{name}.brep"
    destination.write_text("\n".join(output) + "\n")
    return destination


def touching_control(destination):
    """Invalid solid: a hole touches its perimeter, with rotated plane charts.

    Zero tolerances and omitted pcurves isolate source-vertex classification.
    The geometry itself stays axis-aligned, so the chart must not create a gap.
    """
    lines = []
    edge = False
    surfaces = 0
    for line in destination.read_text().splitlines():
        if line == "Ed":
            edge = True
        elif edge and line.strip() == "0":
            edge = False
        elif edge and line.strip().startswith("2 "):
            continue
        if surfaces:
            values = list(map(float, line.split()[1:]))
            p, n, x, y = [values[i:i + 3] for i in range(0, 12, 3)]
            xx = [0.8 * a + 0.6 * b for a, b in zip(x, y)]
            yy = [-0.6 * a + 0.8 * b for a, b in zip(x, y)]
            line = "1 " + numbers(p + n + xx + yy)
            surfaces -= 1
        elif line.startswith("Surfaces "):
            surfaces = int(line.split()[1])
        lines.append(line.replace("1e-07", "0"))
    destination.write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    rectangle = [(0, 0), (10, 0), (10, 12), (0, 12)]
    hole = [(2, 2), (2, 5), (4, 5), (4, 2)]
    hole2 = [(6, 7), (6, 10), (8, 10), (8, 7)]
    write("two-holes-v1", prism(rectangle, [hole, hole2]))
    def placed(p):
        x, y, z = p
        return (0.6 * x - 0.8 * y + 37,
                0.48 * x + 0.36 * y - 0.8 * z - 19,
                0.64 * x + 0.48 * y + 0.6 * z + 11)
    write("oblique-two-holes-v1", [[[placed(p) for p in loop] for loop in loops]
                                  for loops in prism(rectangle, [hole, hole2])])
    write("blind-pocket-v1", prism(rectangle, [[(2, 3), (2, 9), (8, 9), (8, 3)]], floor=5))
    concave = [(0, 0), (10, 0), (10, 7), (6, 7), (6, 12), (0, 12)]
    write("concave-hole-v1", prism(concave, [hole]))
    touching_control(write("touching-hole-zero-v1", prism(rectangle,
        [[(0, 2), (0, 5), (2, 5), (2, 2)]])))
