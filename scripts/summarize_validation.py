"""Verify recorded artifacts and index every trial, including unsuccessful iterations."""
import json
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1] / 'outputs/closed_loop'


def main():
    records = []
    for path in sorted(ROOT.glob('*/result.json')):
        result = json.loads(path.read_text(encoding='utf-8'))
        if result.get('status') == 'running':
            continue
        video = path.parent / 'demo.mp4'
        cap = cv2.VideoCapture(str(video))
        count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        fps = cap.get(cv2.CAP_PROP_FPS)
        first, _ = cap.read()
        cap.set(cv2.CAP_PROP_POS_FRAMES, max(0, count-1))
        last, _ = cap.read()
        cap.release()
        verified = bool(first and last and count == result['frames'])
        assert verified, f'Invalid video: {video}'
        sources = result.get('source_sha256', {})
        records.append({'run':path.parent.name, 'mode':result.get('mode',result['label']), 'seed':result['seed'],
                        'success':result['success'], 'video_verified':verified, 'frames':count,
                        'duration_s':count/fps, 'controller_hash':sources.get('src/pillbot/sim/validate_grip.py'),
                        'grasp_hash':sources.get('src/pillbot/control/grasp.py'), 'result':result})
    grips = [r for r in records if r['mode']=='grip']
    latest = grips[-1]
    cohort = [r for r in grips if (r['controller_hash'],r['grasp_hash']) == (latest['controller_hash'],latest['grasp_hash'])]
    successes = sum(r['success'] for r in cohort)
    lines = ['# Local closed-loop validation videos', '',
             'Simulation only. Every executed trial, including failures, is retained below.', '',
             f'Latest bottle-grip controller cohort: **{successes}/{len(cohort)} passed**; seeds {[r["seed"] for r in cohort]}.',
             f'All {len(records)} completed videos passed first/last-frame and frame-count checks.', '',
             '## Latest controller cohort', '',
             '| Seed | Pass | Minimum hold height (mm) | Contact fraction | Maximum drift (mm) | Video |',
             '|---|---|---|---|---|---|']
    for r in cohort:
        v=r['result']
        lines.append(f'| {r["seed"]} | {r["success"]} | {v.get("min_hold_height_mm",float("nan")):.2f} | {v.get("contact_fraction",0):.2f} | {v.get("max_relative_drift_mm",float("nan")):.3f} | [MP4]({r["run"]}/demo.mp4) |')
    twists = [r for r in records if r['mode']=='fixed_push_twist']
    if twists:
        lines += ['', '## Bottle fixed to ground: push and twist', '',
                  '| Run | Pass | Cap rotation (deg) | Bottle displacement (mm) | Video |', '|---|---|---|---|---|']
        for r in twists:
            v = r['result']
            lines.append(f'| {r["run"]} | {r["success"]} | {v.get("actual_cap_twist_deg",0):.2f} | {v.get("bottle_displacement_mm",0):.6f} | [MP4]({r["run"]}/demo.mp4) |')
    lines += ['', '## Complete trial history', '', '| Run | Result | Duration (s) | Video |', '|---|---|---|---|']
    for r in records:
        lines.append(f'| {r["run"]} | {"PASS" if r["success"] else "FAIL"} | {r["duration_s"]:.1f} | [MP4]({r["run"]}/demo.mp4) |')
    lines += ['', 'Scope: one bottle profile, known bench target, limited randomized placement/mass/friction. Fixed-body twist is an instrumented simulation stroke; full cap removal is not demonstrated. No real hardware, vision validation or new policy training is claimed.']
    (ROOT / 'validation-summary.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    summary = {'total_videos':len(records), 'latest_cohort_runs':[r['run'] for r in cohort],
               'latest_passes':successes, 'latest_trials':len(cohort), 'records':records}
    (ROOT / 'validation-summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in summary.items() if k!='records'}))


if __name__ == '__main__':
    main()
