"""Static URDF inspection only: no controller, physics stepping, or hardware access."""
from pathlib import Path
import json
import math
import struct
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'outputs' / 'assembly_review_v2'


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    path = ROOT / 'hardware/urdf/pillbot.urdf'
    tree = ET.parse(path)
    meshes = []
    for mesh in tree.findall('.//mesh'):
        source = (path.parent / mesh.attrib['filename']).resolve()
        meshes.append({'path': str(source), 'exists': source.is_file()})
    stls = []
    for source in sorted((ROOT / 'hardware/stl').rglob('*.stl')):
        raw = source.read_bytes()
        n = struct.unpack_from('<I', raw, 80)[0] if len(raw) >= 84 else 0
        valid = len(raw) == 84 + 50 * n and n > 0
        record = {'path': str(source.relative_to(ROOT)), 'triangles': n, 'binary_structure_valid': valid}
        if valid:
            faces = np.frombuffer(raw, dtype=np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')]), offset=84)
            vertices = faces['vertices'].reshape(-1, 3)
            record.update(finite=bool(np.isfinite(vertices).all()), bounds_mm=[vertices.min(0).tolist(), vertices.max(0).tolist()])
        stls.append(record)
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    for name, angle in {'shoulder_lift': 0, 'elbow_flex': 0, 'gripper': 45}.items():
        data.qpos[model.jnt_qposadr[model.joint(name).id]] = math.radians(angle)
    mujoco.mj_forward(model, data)
    model.vis.global_.offwidth = 900
    model.vis.global_.offheight = 900
    canvas = Image.new('RGB', (2700, 990), '#17202a')
    with mujoco.Renderer(model, height=900, width=900) as renderer:
        for i, azimuth in enumerate((90, 120, 120)):
            if i == 2:
                for name, angle in {'shoulder_lift': 35, 'elbow_flex': 100}.items():
                    data.qpos[model.jnt_qposadr[model.joint(name).id]] = math.radians(angle)
                mujoco.mj_forward(model, data)
            camera = mujoco.MjvCamera()
            camera.lookat[:] = [0.10 if i == 2 else 0, 0, 0.38]
            camera.distance = 1.05
            camera.azimuth = azimuth
            camera.elevation = 0 if i == 0 else -15
            renderer.update_scene(data, camera=camera)
            canvas.paste(Image.fromarray(renderer.render()), (i * 900, 70))
    draw = ImageDraw.Draw(canvas)
    draw.text((25, 15), 'REVISED ASSEMBLY | Side / oblique / bent pose | Approval pending', fill='white', font_size=28)
    draw.text((25, 50), 'Real shoulder brackets; matched three-hole interfaces; elbow halves on separate links. Gripper choice / drivetrain seating still unverified. No motion test executed.', fill='#ffcc66', font_size=19)
    canvas.save(OUT / 'assembly.png')
    report = {'urdf_loads': True, 'mesh_references': meshes, 'stl_checks': stls,
              'collision_elements': len(tree.findall('.//collision')),
              'closed_loop_tests_executed': 0,
              'limitations': ['Mesh structure checks do not establish mechanical correctness or watertightness.',
                             'Summary requests two Robonine parallel grippers; URDF uses SO-101 hinged gripper.',
                             'URDF has no collision elements; inertia and drivetrain seating require measurement.',
                             'Fixture and bottle are not included in the arm URDF.']}
    (OUT / 'audit.json').write_text(json.dumps(report, indent=2))
    print(json.dumps({'image': str(OUT / 'assembly.png'), 'urdf_loads': True, 'stls': len(stls), 'missing_meshes': sum(not m['exists'] for m in meshes), 'collision_elements': report['collision_elements']}))


if __name__ == '__main__':
    main()
