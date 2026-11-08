"""Check the actual STL screw bores against shared assembly transforms; no dynamics."""
import itertools
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

from pillbot.sim.assembly import (PAN_VISUALS, UPPER_VISUALS, FOREARM_VISUALS,
                                  SHOULDER_HEIGHT_MM, UPPER_ARM_MM)

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'outputs/assembly_review_v2'


def holes(name, end):
    mesh = trimesh.load_mesh(ROOT / f'hardware/stl/arm/{name}.stl')
    face_z = mesh.bounds[end, 2]
    z = face_z + (0.1 if end == 0 else -0.1)
    section = mesh.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
    centers = []
    for loop in section.discrete:
        points = loop[:-1, :2]
        if len(points) < 12 or np.linalg.norm(loop[0] - loop[-1]) > .001:
            continue
        A = np.column_stack((2 * points, np.ones(len(points))))
        x, y, k = np.linalg.lstsq(A, (points**2).sum(1), rcond=None)[0]
        radius = np.sqrt(max(0, k + x*x + y*y))
        error = np.max(abs(np.linalg.norm(points - [x, y], axis=1) - radius))
        bolt_circle = name != 'drive_plate' or abs(np.linalg.norm([x + .838527, y - .129585]) - 30) < .1
        if abs(radius - 2) < .03 and error < .03 and bolt_circle:
            centers.append([x, y, face_z])
    assert len(centers) == 3, (name, centers)
    return np.array(centers)


def main():
    placement = {}
    for parts, offset in [(PAN_VISUALS, 0), (UPPER_VISUALS, SHOULDER_HEIGHT_MM),
                          (FOREARM_VISUALS, SHOULDER_HEIGHT_MM + UPPER_ARM_MM)]:
        for name, xyz, rpy in parts:
            placement[name] = (np.array(xyz) + [0, 0, offset], Rotation.from_euler('xyz', rpy))
    pairs = [('drive_plate', 1, 'shoulder_1', 1), ('shoulder_2', 1, 'link_1', 0),
             ('link_1', 1, 'elbow_1', 0), ('elbow_2', 1, 'link_2', 0)]
    fig, axes = plt.subplots(1, 4, figsize=(15, 4), layout='constrained')
    report = []
    for ax, (a, ae, b, be) in zip(axes, pairs):
        pa = placement[a][1].apply(holes(a, ae)) + placement[a][0]
        pb = placement[b][1].apply(holes(b, be)) + placement[b][0]
        pb = min((pb[list(p)] for p in itertools.permutations(range(3))), key=lambda p: np.linalg.norm(pa-p))
        errors = np.linalg.norm(pa - pb, axis=1)
        assert errors.max() < .1, (a, b, errors)
        report.append(dict(parts=[a,b], hole_centers_a_mm=pa.tolist(), hole_centers_b_mm=pb.tolist(), errors_mm=errors.tolist()))
        ax.scatter(pa[:,0], pa[:,1], s=180, facecolors='none', edgecolors='#0072B2', label=a)
        ax.scatter(pb[:,0], pb[:,1], s=70, marker='+', color='#D55E00', label=b)
        ax.set(title=f'Max mismatch: {errors.max():.4f} mm', xlabel='X (mm)', ylabel='Y (mm)', aspect='equal')
        ax.legend(fontsize=8); ax.grid(alpha=.2)
    fig.suptitle('Three-screw mounting interfaces: independently fitted STL bores after assembly')
    OUT.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT / 'hole_alignment.png', dpi=150)
    (OUT / 'hole_alignment.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps([{'parts': r['parts'], 'max_error_mm': max(r['errors_mm'])} for r in report]))


if __name__ == '__main__':
    main()
