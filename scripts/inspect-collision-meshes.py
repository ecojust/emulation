#!/usr/bin/env python3
"""Read-only STL topology and collision-fixture audit; Python standard library only.

This is an independent development oracle, not the application's collision engine.
No vertices, faces, source files or normals are repaired or rewritten.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct


ROOT = Path(__file__).resolve().parents[1]
MODEL_IDS = ("irb1100-4-0.475", "irb1100-4-0.58")
BASIS = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
RAY_SEEDS = ((1, .371, .127), (.219, 1, .537), (.163, .285, 1),
             (-.617, .421, 1), (.881, -.313, .577))


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def length(a):
    return math.sqrt(dot(a, a))


def read_stl(path):
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"Truncated binary STL: {path}")
    count, = struct.unpack_from("<I", data, 80)
    if len(data) != 84 + count * 50:
        raise ValueError(f"Unexpected binary STL length: {path}")
    triangles, normals = [], []
    for i in range(count):
        values = struct.unpack_from("<12f", data, 84 + 50 * i)
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"Non-finite STL coordinate: {path}, face {i}")
        normals.append(values[:3])
        triangles.append((values[3:6], values[6:9], values[9:12]))
    return triangles, normals, hashlib.sha256(data).hexdigest()


def topology(triangles, normals, weld_mm):
    vertices, edges, faces, volumes, parents = {}, {}, [], [], []
    zero_area = 0
    normal_opposes = 0

    def find(index):
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    for triangle, normal in zip(triangles, normals):
        a, b, c = triangle
        face_normal = cross(sub(b, a), sub(c, a))
        area2 = length(face_normal)
        zero_area += area2 == 0
        normal_opposes += dot(face_normal, normal) < -1e-10
        # Match the documented topology threshold. Original faces remain intact.
        if area2 <= 1e-12:
            continue
        vertex_ids = []
        for vertex in triangle:
            key = tuple(round(value / weld_mm) for value in vertex) if weld_mm else vertex
            vertex_ids.append(vertices.setdefault(key, len(vertices)))
        if len(set(vertex_ids)) < 3:
            continue
        index = len(faces)
        faces.append(vertex_ids)
        parents.append(index)
        volumes.append(dot(a, cross(b, c)) / 6)
        for x, y in zip(vertex_ids, vertex_ids[1:] + vertex_ids[:1]):
            key = (min(x, y), max(x, y))
            incidences = edges.setdefault(key, [])
            if incidences:
                parents[find(index)] = find(incidences[0][0])
            incidences.append((index, 1 if x < y else -1))
    components = {}
    for index, volume in enumerate(volumes):
        component = components.setdefault(find(index), {
            "triangles": 0, "signedVolumeMm3": 0,
            "boundaryEdges": 0, "nonManifoldEdges": 0, "inconsistentEdges": 0,
        })
        component["triangles"] += 1
        component["signedVolumeMm3"] += volume
    for incidences in edges.values():
        component = components[find(incidences[0][0])]
        component["boundaryEdges"] += len(incidences) == 1
        component["nonManifoldEdges"] += len(incidences) > 2
        component["inconsistentEdges"] += len(incidences) == 2 and incidences[0][1] == incidences[1][1]
    details = sorted(components.values(), key=lambda item: -item["triangles"])
    counts = {name: sum(item[name] for item in details)
              for name in ("boundaryEdges", "nonManifoldEdges", "inconsistentEdges")}
    return {
        "triangles": len(triangles), "keptTopologyTriangles": len(faces),
        "zeroAreaFaces": zero_area, "storedNormalOpposesWinding": normal_opposes,
        "weldToleranceMm": weld_mm, **counts, "components": len(details),
        "closedOrientedEdgeManifold": not any(counts.values()) and len(faces) == len(triangles),
        "componentDetails": details,
    }


def triangle_box_intersects(triangle, center, size):
    vertices = [sub(vertex, center) for vertex in triangle]
    half = [value / 2 for value in size]
    for axis in range(3):
        coordinates = [vertex[axis] for vertex in vertices]
        if min(coordinates) > half[axis] + 1e-9 or max(coordinates) < -half[axis] - 1e-9:
            return False
    edges = [sub(vertices[1], vertices[0]), sub(vertices[2], vertices[1]), sub(vertices[0], vertices[2])]
    axes = [cross(edges[0], edges[1])] + [cross(edge, basis) for edge in edges for basis in BASIS]
    for axis in axes:
        projections = [dot(vertex, axis) for vertex in vertices]
        radius = sum(abs(value) * extent for value, extent in zip(axis, half))
        epsilon = length(axis) * 1e-9
        if min(projections) > radius + epsilon or max(projections) < -radius - epsilon:
            return False
    return True


def ray_crossings(triangles, point, direction):
    hits = []
    for a, b, c in triangles:
        edge1, edge2 = sub(b, a), sub(c, a)
        h = cross(direction, edge2)
        determinant = dot(edge1, h)
        if abs(determinant) <= 1e-10:
            continue
        t = sub(point, a)
        u = dot(t, h) / determinant
        if u < -1e-10 or u > 1 + 1e-10:
            continue
        q = cross(t, edge1)
        v = dot(direction, q) / determinant
        distance = dot(edge2, q) / determinant
        if v >= -1e-10 and u + v <= 1 + 1e-10 and distance > 1e-7:
            hits.append(distance)
    hits.sort()
    distinct = []
    for distance in hits:
        if not distinct or distance - distinct[-1] > 1e-6:
            distinct.append(distance)
    return len(distinct)


def fixture(triangles, name, center, size, expected_surface, expected_inside):
    directions = [tuple(value / length(seed) for value in seed) for seed in RAY_SEEDS]
    directions += [tuple(-value for value in direction) for direction in directions]
    hits = sum(triangle_box_intersects(triangle, center, size) for triangle in triangles)
    crossings = [ray_crossings(triangles, center, direction) for direction in directions]
    if hits != expected_surface or not all(count % 2 == int(expected_inside) for count in crossings):
        raise AssertionError(f"Fixture changed: {name}, surface={hits}, ray crossings={crossings}")
    return {"name": name, "centerMm": center, "sizeMm": size,
            "surfaceTriangleHits": hits, "rayCrossings": crossings,
            "observedRayParityInside": expected_inside}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--weld-mm", type=float, default=0,
                        help="Diagnostic vertex-key quantization only; never rewrites STL (default: exact floats).")
    args = parser.parse_args()
    if not math.isfinite(args.weld_mm) or not 0 <= args.weld_mm <= .01:
        parser.error("--weld-mm must be finite and between 0 and 0.01 mm")
    results, fixtures = [], []
    for model_id in MODEL_IDS:
        for part_name in ("base_link", *(f"link_{index}" for index in range(1, 7))):
            path = ROOT / "public/models" / model_id / f"{part_name}.stl"
            triangles, normals, digest = read_stl(path)
            report = {"modelId": model_id, "part": path.stem,
                      "path": path.relative_to(ROOT).as_posix(), "sha256": digest,
                      **topology(triangles, normals, args.weld_mm)}
            results.append(report)
            part_fixtures = []
            if path.stem == "base_link":
                part_fixtures = [
                    fixture(triangles, "obb_empty_corner", [75, 75, 80], [4, 4, 4], 0, False),
                    fixture(triangles, "surface_intersection", [0, 70, 100], [4, 4, 4], 3, True),
                    fixture(triangles, "box_contained_in_solid", [0, 0, 50], [2, 2, 2], 0, True),
                ]
            elif path.stem == "link_1":
                if report["closedOrientedEdgeManifold"]:
                    raise AssertionError("Known non-manifold input changed; review its uncertain fixture")
                part_fixtures = [fixture(triangles, "non_manifold_no_surface_hit", [0, 85, 180], [4, 4, 4], 0, False)]
                part_fixtures[0]["expectedSingleLinkResult"] = "uncertain"
            elif path.stem == "link_5":
                center = ([301.75, -16.708984375, 537.6882934570312] if model_id.endswith("0.475")
                          else [351.75, -16.708984375, 592.6882934570312])
                part_fixtures = [fixture(triangles, "empty_internal_cavity", center, [.1, .1, .1], 0, False)]
            for item in part_fixtures:
                fixtures.append({"modelId": model_id, "part": path.stem, "sha256": digest, **item})
    print(json.dumps({"units": "mm", "totalTriangles": sum(item["triangles"] for item in results),
                      "meshes": results, "fixtures": fixtures, "fixtureChecksPassed": True}, indent=2))


if __name__ == "__main__":
    main()
