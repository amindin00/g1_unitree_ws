"""Check motion against the actual G1 geometry, limits, and recording format."""

import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from g1_motion_demo.motion import PoseFilter, RobotModel, retarget_landmarks, wave_pose
from g1_motion_demo.recording import Playback, Recorder


@pytest.fixture
def model():
    return RobotModel(Path(__file__).parents[2] / 'g1_unitree_description/urdf/g1_29dof.urdf')


def test_wave_limits_continuity_and_raised_hand(model):
    assert len(model.names) == 29
    for seconds in np.linspace(0, 14, 1000):
        pose = wave_pose(seconds)
        assert all(model.clamp(pose)[name] == value for name, value in pose.items())
    assert wave_pose(0) == wave_pose(7)
    raised = model.transform('right_rubber_hand', wave_pose(2.0))[:3, 3]
    lowered = model.transform('right_rubber_hand', wave_pose(0.0))[:3, 3]
    assert raised[2] > lowered[2] + 0.4


def test_filter_limits_velocity_and_tracking_loss(model):
    filter_ = PoseFilter(model, smoothing=0.05, max_speed=1.0)
    before = dict(filter_.pose)
    after = filter_.update({name: 999.0 for name in model.names}, 0.02)
    assert max(abs(after[n] - before[n]) for n in model.names) <= 0.02000001
    for _ in range(500):
        after = filter_.update({}, 0.02)
    assert after == pytest.approx(model.clamp(wave_pose(0)), abs=1e-6)
    with pytest.raises(ValueError, match='Non-finite'):
        filter_.update({'left_elbow_joint': float('nan')}, 0.02)


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('pitch,elbow', [(-1.2, -0.4), (-0.6, 0.5), (0.0, 1.3)])
def test_arm_solver_matches_reachable_directions(model, side, pitch, elbow):
    pose = {f'{side}_shoulder_pitch_joint': pitch,
            f'{side}_shoulder_roll_joint': 0.5 if side == 'left' else -0.5,
            f'{side}_elbow_joint': elbow}
    target = model.arm_directions(side, pose)
    result = model.solve_arm(side, target[:3], target[3:], {})
    assert np.linalg.norm(model.arm_directions(side, result) - target) < 0.05
    assert all(model.clamp(result)[name] == value for name, value in result.items())


def test_landmark_retargeting_uses_anatomical_sides_and_handles_occlusion(model):
    # Camera coordinates: image right = body left, image down = body down.
    points = [SimpleNamespace(x=0., y=0., z=0., presence=1., visibility=1.)
              for _ in range(33)]
    for index, xyz in {11: (.2, -.5, 0), 12: (-.2, -.5, 0),
                       23: (.15, 0, 0), 24: (-.15, 0, 0),
                       13: (.5, -.5, 0), 15: (.8, -.5, 0),
                       14: (-.2, -.2, 0), 16: (-.2, .1, 0)}.items():
        points[index].x, points[index].y, points[index].z = xyz
    pose = retarget_landmarks(model, points, {})
    assert model.arm_directions('left', pose)[1] > 0.9
    assert model.arm_directions('right', pose)[2] < -0.9
    assert abs(pose['waist_roll_joint']) < 1e-6
    points[13].visibility = 0.1
    pose = retarget_landmarks(model, points, pose)
    assert not any(name.startswith('left_') for name in pose)
    assert 'right_elbow_joint' in pose
    points[23].visibility = 0.1
    assert retarget_landmarks(model, points, pose) == {}


def test_recording_round_trip_and_interpolation(model, tmp_path):
    path = tmp_path / 'wave.jsonl'
    first, last = model.clamp(wave_pose(0)), model.clamp(wave_pose(2))
    recorder = Recorder(path, model.names)
    recorder.append(0.1, first)
    recorder.append(2.1, last)
    recorder.close()
    playback = Playback(path, model.names)
    assert playback.sample(0) == first
    assert playback.sample(1) == pytest.approx({n: (first[n] + last[n]) / 2 for n in model.names})
    assert playback.sample(10, loop=False) == last
    assert playback.sample(2, loop=True) == first
    with pytest.raises(FileExistsError):
        Recorder(path, model.names)


@pytest.mark.parametrize('bad_frame', [
    {'t': 0, 'positions': [float('nan')]},
    {'t': -1, 'positions': [0]},
    {'t': 0, 'positions': []},
])
def test_reject_invalid_recordings(tmp_path, bad_frame):
    path = tmp_path / 'invalid.jsonl'
    header = {'format': 'g1_motion_demo', 'version': 1, 'joint_names': ['joint']}
    path.write_text(json.dumps(header) + '\n' + json.dumps(bad_frame) + '\n')
    with pytest.raises(ValueError, match='invalid'):
        Playback(path, ['joint'])
