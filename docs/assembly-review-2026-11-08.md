# Assembly approval checkpoint — 2026-11-08

Goal: continue sim-to-real development and validate closed-loop pill-bottle gripping, recording every test locally. User requires an assembly image and explicit approval before the first iteration.

## Evidence collected

- Read supplied `C:/Users/mikey/Downloads/pill_bottle_opener_summary.md`.
- Read Research-Career-Vault AI-START.md, Home.md and AGENTS.md. No matching MakeCu/pill-bottle project or task was found by content search. Requested canonical project path/context package; vault records have NOT been updated.
- Local commit `4adb345502412b882f3e530f30ffa9cf3cc99c6d` matches GitHub origin HEAD verified with `git ls-remote origin HEAD`.
- Loaded the existing URDF successfully in MuJoCo 4.0.0. All referenced mesh files exist. All 26 local binary STLs passed record-size and finite-coordinate checks. These checks do not prove watertightness, fit, joint alignment, or physical fidelity.
- Saved static two-view assembly image to `outputs/assembly_review/assembly.png` and machine-readable audit to `outputs/assembly_review/audit.json`. Rendering uses forward geometry only, with no dynamics stepping or controller execution.

## Unresolved model differences

- Supplied summary specifies Robonine parallel grippers at the end effector and base. Repository instead uses an SO-101 hinged end effector and a simplified rack clamp in the separate simulation scene.
- URDF has zero collision elements, simplified tower, placeholder inertias and estimated joint positions. It omits the bottle, fixture and riser geometry. It is not a complete validated physical assembly.
- Summary and current configuration differ in link dimensions, riser height, gear ratios and gripper mass. Measurements and the intended hardware revision must resolve these differences.
- Real joint mapping and camera/tactile calibration remain unverified. No sim-to-real success is claimed.

## Decision and next action

Paused before the first closed-loop iteration, as requested. No gripping tests or hardware commands were executed, so no test videos exist yet.

Next: obtain the correct vault context and confirm the intended gripper assembly; reconcile CAD, URDF and collision geometry; show the revised assembly and obtain explicit approval. After approval, ensure every closed-loop run writes a unique local video (including failed runs), synchronized observations/actions and outcome metadata before running it. Simulation evidence must remain separate from real-hardware validation.

Reproduce the static inspection on this computer in PowerShell:

```powershell
$env:PYTHONPATH='D:\MakeCu2026\.sim-deps'
python scripts/assembly_review.py
```

Rendering dependencies were installed in ignored `.sim-deps`. The default Python is 3.11; the main pipeline requires Python 3.12 or newer and has not been executed.
