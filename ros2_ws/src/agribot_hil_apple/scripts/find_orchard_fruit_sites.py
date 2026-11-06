"""Run with Blender: find actual woody branch undersides, not floating fruit sites."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import bpy
import numpy as np
from mathutils import Vector
from mathutils.bvhtree import BVHTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(sys.argv[sys.argv.index('--')+1:])
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    bpy.ops.wm.collada_import(filepath=str(args.source))
    vertices, polygons = [], []
    for obj in bpy.context.scene.objects:
        if obj.type != 'MESH':
            continue
        start = len(vertices)
        vertices.extend(obj.matrix_world @ vertex.co for vertex in obj.data.vertices)
        obj.data.calc_loop_triangles()
        polygons.extend([start+index for index in triangle.vertices] for triangle in obj.data.loop_triangles)
    points = np.asarray(vertices)
    surface = BVHTree.FromPolygons(vertices, polygons)
    pairs = np.array([(polygon[i], polygon[(i+1) % len(polygon)])
                      for polygon in polygons for i in range(len(polygon))])
    duplicates = cKDTree(points).query_pairs(.0001, output_type='ndarray')
    pairs = np.concatenate([pairs, duplicates])
    graph = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])), shape=(len(points), len(points)))
    count, labels = connected_components(graph, directed=False)
    centers = []
    for label in range(count):
        cluster = points[labels == label]
        if len(cluster) >= 30 and cluster[:, 2].min() < .3 and cluster[:, 2].max() > .6:
            basal = cluster[cluster[:, 2] <= cluster[:, 2].min()+.15, :2]
            centers.append(basal.mean(axis=0))
    centers.sort(key=lambda xy: (-xy[1], xy[0]))
    groups, occupied = [], []
    for center in centers:
        candidates = []
        for x in np.arange(center[0]-1.2, center[0]+1.21, .1):
            for y in np.arange(center[1]-1.3, center[1]+1.31, .1):
                hit, normal, _, _ = surface.ray_cast(Vector((x, y, .45)), Vector((0, 0, 1)), 1.1)
                if hit is not None and normal.z < -.05 and .6 <= hit.z <= 1.3:
                    candidates.append([float(hit.x), float(hit.y), float(hit.z)])
        # Favor low, outside branches on the north aisle-facing side.
        candidates.sort(key=lambda p: (-(p[1]-center[1]) + .3*p[2], abs(p[0]-center[0])))
        selected = []
        for candidate in candidates:
            if all(np.linalg.norm(np.array(candidate)-np.array(old)) > .13 for old in occupied):
                selected.append(candidate)
                occupied.append(candidate)
            if len(selected) == 3:
                break
        groups.append({'tree_center': center.tolist(), 'anchors': selected,
                       'candidate_count': len(candidates)})
    pickup = []
    for x in np.arange(27.85, 28.251, .02):
        for y in np.arange(31.25, 31.651, .02):
            hit, normal, _, _ = surface.ray_cast(Vector((x, y, .45)), Vector((0, 0, 1)), .6)
            if hit is not None and normal.z < -.05 and .6 <= hit.z <= .9:
                pickup.append([float(hit.x), float(hit.y), float(hit.z)])
    pickup.sort(key=lambda p: (-p[1], p[2], abs(p[0]-28.)))
    selected = []
    for candidate in pickup:
        if all(np.linalg.norm(np.array(candidate)-np.array(old)) > .105 for old in selected):
            selected.append(candidate)
        if len(selected) == 6:
            break
    outer_branch = []
    for x in np.arange(29.8, 30.701, .02):
        for y in np.arange(31.2, 31.901, .02):
            hit, normal, _, _ = surface.ray_cast(Vector((x, y, .45)), Vector((0, 0, 1)), .6)
            if hit is not None and normal.z < -.05 and .6 <= hit.z <= .95:
                outer_branch.append([float(hit.x), float(hit.y), float(hit.z)])
    outer_branch.sort(key=lambda p: (-p[1], p[2], abs(p[0]-30.24)))
    secondary = []
    for candidate in outer_branch:
        if all(np.linalg.norm(np.array(candidate)-np.array(old)) > .105 for old in secondary):
            secondary.append(candidate)
        if len(secondary) == 3:
            break
    report = {'source': str(args.source), 'source_sha256': hashlib.sha256(args.source.read_bytes()).hexdigest(),
              'method': 'Connected woody components and upward BVH ray hits on woody undersides',
              'trees': groups, 'pickup_candidates': selected, 'secondary_pickup_candidates': secondary}
    triangles = np.asarray(polygons)
    triangle_points = points[triangles]
    lower, upper = np.array([27.0, 30.65, -.2]), np.array([32.5, 33.25, 1.7])
    selected = triangles[np.all(triangle_points.max(axis=1) >= lower, axis=1) &
                         np.all(triangle_points.min(axis=1) <= upper, axis=1)]
    indices, compact = np.unique(selected, return_inverse=True)
    report['manipulation_environment_mesh'] = {
        'vertices_in_world': np.round(points[indices], 7).tolist(),
        'triangles': compact.reshape(-1, 3).tolist()}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print('FRUIT_SITES', len(groups), 'trees', sum(len(g['anchors']) for g in groups), 'anchors')
    print('NEAR_SECOND_ROW', [g for g in groups if abs(g['tree_center'][0]-25)<2 and abs(g['tree_center'][1]-30)<2])


if __name__ == '__main__':
    main()
