# Assembly correction — 2026-11-08

User rejected the first preview: the upper-link and elbow three-screw patterns must align, and the shoulder must use the actual bracket assembly shown in the supplied reference image.

## Changes

- Replaced the mesh-mode cylindrical shoulder placeholder with `drive_plate`, `shoulder_1` and `shoulder_2` meshes. Inverted the exported fixed shoulder bracket and matched its three mounting bores to the pan drive plate.
- Seated `link_1` on the moving shoulder bracket. Seated `elbow_1` on the upper link's top face: the old source placement embedded it 40.5 mm into that link.
- Moved `elbow_2` to the forearm body; centered its rounded pivot profile on the elbow pivot and seated `link_2` on its three-hole flange. Both elbow halves previously moved with the upper arm.
- Kept the base shell stationary during pan motion, matching the URDF body structure.
- Centralized visual transforms in `src/pillbot/sim/assembly.py`, used by both MJCF and URDF. No STL vertices were altered.
- Updated nominal geometry: shoulder height above the riser 118.408 mm; shoulder-to-elbow 233.207 mm; elbow-to-wrist 185.513 mm. Earlier 150 mm lengths described mesh extents, not complete pivot spacing. These reconstructed dimensions need physical confirmation.
- Export URDF explicitly as UTF-8 to avoid Windows default encoding producing invalid XML.

## Static evidence

`scripts/check_assembly_interfaces.py` independently sections the original STLs, fits their 4 mm screw bores, transforms the centers and compares all three holes and mating-face heights. Maximum discrepancy at each tested interface is below 0.001 mm (numerical precision, not manufactured-part accuracy):

1. Drive plate ↔ fixed shoulder bracket.
2. Moving shoulder bracket ↔ upper link.
3. Upper link ↔ fixed elbow housing.
4. Moving elbow housing ↔ forearm link.

Across 50 static joint configurations, ten arm meshes have matching URDF/MJCF world poses to numerical precision. Existing grasp-frame parity check: maximum 0.040 mm. All 26 STL structural checks pass; the regenerated URDF parses and loads. None of these checks steps dynamics or executes a closed-loop motion.

Images and numeric reports: `outputs/assembly_review_v2/assembly.png`, `hole_alignment.png`, `hole_alignment.json`, `mesh_parity.json`, `audit.json`. The original rejected preview remains in `outputs/assembly_review/`.

## Pending before motion testing

This is a corrected **external mesh assembly**, not a fully verified physical robot. The image retains the existing SO-101 end effector pending the user's gripper selection. Internal shoulder gearing, coupler, bearings, axial seating, wrist adapter, mass/inertia, collision geometry, bottle fixture, and real joint calibration are not verified. The riser remains configured but is not drawn in the arm-only URDF. Shaft centres inferred from rounded mesh profiles are less certain than the directly matched screw patterns.

Old reach-policy results and workspace/reach claims must be revalidated for these changed dimensions; do not use the existing checkpoint as evidence of validation. No policy training or closed-loop test was run. User approval of the revised assembly is still required before the first iteration, and every later closed-loop test must be recorded locally.

## Reproduce static checks (PowerShell, repository root)

```powershell
$env:PYTHONPATH='src'
.venv\Scripts\python.exe -m pillbot.sim.urdf --check
.venv\Scripts\python.exe scripts/check_assembly_interfaces.py
.venv\Scripts\python.exe scripts/check_mesh_parity.py
.venv\Scripts\python.exe scripts/assembly_review.py
```

Local review environment: Python 3.14.2, MuJoCo 4.0.0, NumPy, PyYAML, Pillow, trimesh, SciPy, matplotlib and networkx. This does not establish LeRobot compatibility or readiness for hardware execution.
