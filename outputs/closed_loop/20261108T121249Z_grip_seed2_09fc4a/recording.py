"""Per-run MP4 and telemetry. Recording starts before motion and survives failures."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import uuid

import imageio_ffmpeg
import mujoco
import numpy as np
from PIL import Image, ImageDraw

from pillbot.utils.config import ROOT


def json_default(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


class RunRecorder:
    def __init__(self, world, label, seed, root=None, fps=15):
        self.world, self.fps = world, fps
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        self.path = Path(root or ROOT / 'outputs/closed_loop') / f'{stamp}_{label}_seed{seed}_{uuid.uuid4().hex[:6]}'
        self.path.mkdir(parents=True, exist_ok=False)
        self.phase, self.frames, self.next_frame = 'initial', 0, 0.
        self.log = (self.path / 'telemetry.jsonl').open('w', encoding='utf-8')
        self.writer = imageio_ffmpeg.write_frames(str(self.path / 'demo.mp4'), (1280, 560), fps=fps,
                                                 codec='libx264', pix_fmt_out='yuv420p',
                                                 output_params=['-crf', '20', '-movflags', '+faststart'])
        self.writer.send(None)
        self.metadata = {'label': label, 'seed': seed, 'simulation_only': True, 'fps': fps,
                         'sim_config': world.sim, 'robot_config': {'gripper': world.robot_cfg['gripper'],
                         'safety': world.robot_cfg['safety'], 'tactile_threshold': world.robot_cfg['tactile']['contact_threshold']},
                         'model_sha256': hashlib.sha256((ROOT / 'hardware/urdf/pillbot.urdf').read_bytes()).hexdigest(),
                         'status': 'running', 'controller_inputs': 'joint encoders, tactile; known bench target',
                         'ground_truth': 'bottle pose/contact forces used only for scoring and telemetry'}
        self.metadata['configured_motor_ids'] = world.robot_cfg['follower'].get('motor_ids', {})
        self.metadata['joint_angle_convention'] = 'sim degrees; real = sign * sim + offset_deg using joint_map'
        self.metadata['source_sha256'] = {}
        for relative in ['src/pillbot/sim/validate_grip.py', 'src/pillbot/control/grasp.py',
                         'src/pillbot/sim/world.py', 'src/pillbot/sim/scene.py', 'src/pillbot/sim/assembly.py',
                         'src/pillbot/sim/recording.py', 'src/pillbot/sim/backends.py']:
            data = (ROOT / relative).read_bytes()
            self.metadata['source_sha256'][relative] = hashlib.sha256(data).hexdigest()
            (self.path / Path(relative).name).write_bytes(data)
        mujoco.mj_saveLastXML(str(self.path / 'model.xml'), world.model)
        self.metadata['model_xml_sha256'] = hashlib.sha256((self.path / 'model.xml').read_bytes()).hexdigest()
        self.metadata['randomized_cap'] = {'thread_torque_Nm':world.thread_torque, 'unlock_mm':world.unlock_mm}
        self.save_manifest()
        self(world)  # prove video/render path works before advancing physics
        world.on_step = self

    def save_manifest(self):
        (self.path / 'result.json').write_text(json.dumps(self.metadata, indent=2, default=json_default), encoding='utf-8')

    def __call__(self, world):
        if world.time + 1e-9 < self.next_frame:
            return
        pressure = float(world.tactile_kpa().sum())
        contacts = {'fixed_pad': 0., 'moving_pad': 0.}
        bottle_contacts = {}
        for index in range(world.data.ncon):
            c = world.data.contact[index]
            if not any(int(g) in world.bottle_geoms for g in (c.geom1, c.geom2)):
                continue
            force = np.zeros(6)
            mujoco.mj_contactForce(world.model, world.data, index, force)
            pair_name = '/'.join(world.model.geom(int(g)).name or f'geom{g}' for g in (c.geom1, c.geom2))
            bottle_contacts[pair_name] = bottle_contacts.get(pair_name, 0.) + abs(float(force[0]))
            for g in (c.geom1, c.geom2):
                name = world.model.geom(int(g)).name
                if name == 'fixed_pad': contacts['fixed_pad'] += abs(float(force[0]))
                if name.startswith('taxel_'): contacts['moving_pad'] += abs(float(force[0]))
        row = {'time_s': world.time, 'phase': self.phase, 'joints_deg': world.read(),
               'targets': world.targets(), 'torques_Nm': world.joint_torques(), 'tactile_sum_kpa': pressure,
               'bottle_xyz_m': world.bottle_pos, 'bottle_tilt_deg': world.bottle_upright(),
               'grasp_xyz_m': world.grasp_pos(), 'pad_contact_N': contacts, 'bottle_contact_pairs_N': bottle_contacts}
        self.log.write(json.dumps(row, default=json_default) + '\n')
        self.log.flush()
        canvas = Image.new('RGB', (1280, 560), '#15202c')
        for i in range(2):
            cam = mujoco.MjvCamera()
            cam.lookat[:] = [.12, .02, .26] if i == 0 else world.grasp_pos()
            cam.distance = .85 if i == 0 else .27
            cam.azimuth, cam.elevation = (120, -20) if i == 0 else (145, -15)
            canvas.paste(Image.fromarray(world.render(cam, 640, 480)), (640*i, 40))
        draw = ImageDraw.Draw(canvas)
        draw.text((12, 10), f'SIMULATION | {self.phase} | {world.time:.2f} s | seed {self.metadata["seed"]}', fill='white', font_size=20)
        draw.text((12, 529), f'Jaw {world.read()["gripper"]:.1f}/100 | Tactile {pressure:.0f} | Bottle z {world.bottle_pos[2]*1000:.1f} mm | Tilt {world.bottle_upright():.1f} deg', fill='white', font_size=17)
        self.writer.send(np.asarray(canvas))
        if self.frames == 0 or self.phase != getattr(self, '_last_phase', None):
            canvas.save(self.path / f'{self.frames:05d}_{self.phase}.png')
        self._last_phase = self.phase
        self.frames += 1
        self.next_frame += 1 / self.fps

    def finish(self, result):
        self.world.on_step = None
        self.next_frame = self.world.time
        self(self.world)
        try:
            self.writer.close()
        finally:
            self.log.close()
        self.metadata.update(result)
        self.metadata.update(frames=self.frames, simulated_seconds=self.world.time,
                             video=str(self.path / 'demo.mp4'), status='complete')
        import cv2
        capture = cv2.VideoCapture(str(self.path / 'demo.mp4'))
        count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        first_ok, _ = capture.read()
        capture.set(cv2.CAP_PROP_POS_FRAMES, max(0, count - 1))
        last_ok, _ = capture.read()
        capture.release()
        self.metadata['video_verified'] = bool(first_ok and last_ok and count == self.frames)
        if not self.metadata['video_verified']:
            self.metadata['status'] = 'recording_failed'
        self.save_manifest()
        if not self.metadata['video_verified']:
            raise RuntimeError(f'Video verification failed: {self.path}')
        return self.path
