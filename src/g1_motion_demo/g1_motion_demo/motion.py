"""URDF-aware motion generation, filtering, and arm retargeting."""

import math
import xml.etree.ElementTree as ET

import numpy as np


def unit(vector):
    """Normalize a nonzero finite vector."""
    vector = np.asarray(vector, dtype=float)
    length = np.linalg.norm(vector)
    if not np.isfinite(length) or length < 1e-6:
        raise ValueError('Degenerate pose vector')
    return vector / length


def rotation(axis, angle):
    """Return an axis-angle rotation matrix."""
    x, y, z = unit(axis)
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * skew @ skew


class RobotModel:
    """Read joint limits and actual arm geometry from this robot's URDF."""

    def __init__(self, path):
        self.joints = {}
        self.parents = {}
        self.limits = {}
        for element in ET.parse(path).getroot().findall('joint'):
            name = element.attrib['name']
            origin = element.find('origin')
            xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
            r, p, y = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')
            transform = np.eye(4)
            transform[:3, :3] = (
                rotation([0, 0, 1], y) @ rotation([0, 1, 0], p)
                @ rotation([1, 0, 0], r)
            )
            transform[:3, 3] = xyz
            axis_element = element.find('axis')
            axis = np.fromstring(axis_element.get('xyz'), sep=' ') if (
                axis_element is not None) else np.array([1., 0., 0.])
            parent = element.find('parent').get('link')
            child = element.find('child').get('link')
            self.joints[name] = (parent, transform, axis)
            self.parents[child] = name
            if element.get('type') == 'revolute':
                limit = element.find('limit')
                self.limits[name] = (
                    float(limit.get('lower')), float(limit.get('upper')),
                    float(limit.get('velocity')),
                )
        self.names = list(self.limits)
        self._chains = {}

    def clamp(self, pose):
        """Return a complete finite pose, bounded by the URDF limits."""
        result = {}
        for name, (lower, upper, _) in self.limits.items():
            value = float(pose.get(name, 0.0))
            if not math.isfinite(value):
                raise ValueError(f'Non-finite angle for {name}')
            result[name] = max(lower, min(upper, value))
        return result

    def transform(self, link, pose):
        """Compute a link transform in the pelvis frame."""
        if link not in self._chains:
            chain = []
            current = link
            while current in self.parents:
                name = self.parents[current]
                chain.append(name)
                current = self.joints[name][0]
            self._chains[link] = chain[::-1]
        transform = np.eye(4)
        for name in self._chains[link]:
            _, origin, axis = self.joints[name]
            transform = transform @ origin
            angle = pose.get(name, 0.0)
            if angle:
                turn = np.eye(4)
                turn[:3, :3] = rotation(axis, angle)
                transform = transform @ turn
        return transform

    def arm_directions(self, side, pose):
        """Return upper-arm and forearm directions in the pelvis frame."""
        shoulder, elbow, wrist = [
            self.transform(f'{side}_{part}_link', pose)[:3, 3]
            for part in ['shoulder_roll', 'elbow', 'wrist_roll']
        ]
        return np.concatenate((unit(elbow - shoulder), unit(wrist - elbow)))

    def solve_arm(self, side, upper, forearm, previous):
        """Fit four arm joints to directions using damped least squares."""
        names = [f'{side}_{part}_joint' for part in [
            'shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow']]
        target = np.concatenate((unit(upper), unit(forearm)))
        angles = np.array([previous.get(name, 0.0) for name in names])
        lower = np.array([self.limits[name][0] for name in names])
        upper_limits = np.array([self.limits[name][1] for name in names])

        def evaluate(values):
            return self.arm_directions(side, dict(zip(names, values)))

        for _ in range(12):
            current = evaluate(angles)
            error = target - current
            if np.linalg.norm(error) < 0.015:
                break
            jacobian = np.column_stack([
                (evaluate(angles + np.eye(4)[index] * 1e-4) - current) / 1e-4
                for index in range(4)
            ])
            delta = np.linalg.solve(
                jacobian.T @ jacobian + 0.03 * np.eye(4), jacobian.T @ error)
            angles = np.clip(angles + np.clip(delta, -0.25, 0.25), lower, upper_limits)
        return dict(zip(names, angles))


def wave_pose(seconds):
    """Generate a seven-second wave with smooth entry and exit."""
    phase = seconds % 7.0
    envelope = min(1.0, max(0.0, phase - 0.5), max(0.0, 6.5 - phase))
    envelope = envelope * envelope * (3.0 - 2.0 * envelope)
    return {
        'left_shoulder_roll_joint': 0.12,
        'right_shoulder_roll_joint': -0.12 - 0.50 * envelope,
        'right_shoulder_pitch_joint': -1.35 * envelope,
        'right_elbow_joint': 1.35 - 1.70 * envelope,
        'left_elbow_joint': 1.35,
        'right_wrist_roll_joint': 0.4 * envelope * math.sin(2 * math.pi * 1.3 * phase),
        'right_shoulder_yaw_joint': 0.22 * envelope * math.sin(2 * math.pi * 0.8 * phase),
    }


def _dance_keyframes():
    """Build a Michael Jackson-inspired routine using named, editable poses."""
    neutral = wave_pose(0.0)
    ready = {
        **neutral,
        'left_shoulder_roll_joint': 0.20,
        'right_shoulder_roll_joint': -0.20,
        'left_elbow_joint': 1.15,
        'right_elbow_joint': 1.15,
        'left_hip_roll_joint': 0.06,
        'right_hip_roll_joint': -0.06,
        'left_hip_pitch_joint': -0.10,
        'right_hip_pitch_joint': -0.10,
        'left_knee_joint': 0.20,
        'right_knee_joint': 0.20,
        'left_ankle_pitch_joint': -0.10,
        'right_ankle_pitch_joint': -0.10,
        'left_ankle_roll_joint': -0.06,
        'right_ankle_roll_joint': 0.06,
    }
    hat_tip = {
        **ready,
        'right_shoulder_pitch_joint': -1.50,
        'right_shoulder_roll_joint': -0.30,
        'right_elbow_joint': -0.40,
        'right_wrist_roll_joint': -0.25,
        'left_shoulder_pitch_joint': 0.15,
        'left_shoulder_yaw_joint': -0.30,
        'left_elbow_joint': 0.65,
        'waist_pitch_joint': 0.10,
        'waist_yaw_joint': -0.12,
    }
    point_right = {
        **ready,
        'right_shoulder_pitch_joint': -2.10,
        'right_shoulder_roll_joint': -0.65,
        'right_shoulder_yaw_joint': -0.15,
        'right_elbow_joint': 1.30,
        'left_shoulder_pitch_joint': 0.15,
        'left_elbow_joint': 0.65,
        'waist_yaw_joint': -0.22,
        'waist_roll_joint': 0.12,
    }
    shoulder_hit = {
        **ready,
        'left_shoulder_pitch_joint': -0.35,
        'left_shoulder_roll_joint': 0.95,
        'left_shoulder_yaw_joint': -0.35,
        'left_elbow_joint': 0.25,
        'right_shoulder_pitch_joint': -0.25,
        'right_shoulder_roll_joint': -0.55,
        'right_elbow_joint': 0.10,
        'waist_yaw_joint': 0.28,
        'waist_roll_joint': -0.12,
        'waist_pitch_joint': -0.08,
    }
    slide_left = {
        **hat_tip,
        # The straight leg slides back while the bent leg points its toe down.
        # This suggests a moonwalk in place; it does not solve floor contact.
        'left_hip_pitch_joint': 0.25,
        'left_knee_joint': 0.12,
        'left_ankle_pitch_joint': -0.37,
        'right_hip_pitch_joint': -0.25,
        'right_knee_joint': 0.45,
        'right_ankle_pitch_joint': 0.30,
        'waist_yaw_joint': 0.06,
        'waist_roll_joint': -0.06,
    }
    slide_right = {
        **slide_left,
        'left_hip_pitch_joint': -0.25,
        'left_knee_joint': 0.45,
        'left_ankle_pitch_joint': 0.30,
        'right_hip_pitch_joint': 0.25,
        'right_knee_joint': 0.12,
        'right_ankle_pitch_joint': -0.37,
        'waist_yaw_joint': -0.06,
        'waist_roll_joint': 0.06,
    }
    lean = {
        **ready,
        'waist_pitch_joint': 0.38,
        'left_shoulder_pitch_joint': 0.20,
        'right_shoulder_pitch_joint': 0.20,
        'left_elbow_joint': 1.35,
        'right_elbow_joint': 1.35,
    }

    def mirror(pose):
        reflected = {}
        for name, angle in pose.items():
            if name.startswith('left_'):
                name = 'right_' + name[5:]
            elif name.startswith('right_'):
                name = 'left_' + name[6:]
            if '_roll_' in name or '_yaw_' in name:
                angle = -angle
            reflected[name] = angle
        return reflected

    point_left, opposite_hit = mirror(point_right), mirror(shoulder_hit)
    frames = (
        (0.0, neutral), (1.0, ready),
        (2.2, hat_tip), (3.0, hat_tip),
        (4.4, point_right), (5.2, point_right),
        (6.4, hat_tip), (7.8, point_left), (8.6, point_left),
        (9.8, shoulder_hit), (10.6, opposite_hit),
        (11.4, shoulder_hit), (12.2, opposite_hit), (13.0, ready),
        (14.0, slide_left), (15.2, slide_right),
        (16.4, slide_left), (17.6, slide_right),
        (18.8, slide_left), (20.0, slide_right), (21.0, ready),
        (22.4, lean), (23.2, lean),
        (24.6, point_right), (25.4, point_right),
        (26.8, hat_tip), (27.6, hat_tip), (28.8, ready), (30.0, neutral),
    )
    # Fill omitted joints with neutral, so transitions release previous gestures.
    names = dict.fromkeys(name for _, pose in frames for name in pose)
    baseline = {name: neutral.get(name, 0.0) for name in names}
    return tuple((seconds, {**baseline, **pose}) for seconds, pose in frames)


_DANCE_KEYFRAMES = _dance_keyframes()


def dance_pose(seconds):
    """Loop a 30-second MJ-inspired RViz routine with eased pose transitions.

    Hat tips, diagonal points, shoulder hits, heel-toe slides, and a waist lean.
    The pelvis is fixed: there is no root travel, balance, or foot-contact solve.
    """
    phase = seconds % _DANCE_KEYFRAMES[-1][0]
    for (start, before), (end, after) in zip(_DANCE_KEYFRAMES, _DANCE_KEYFRAMES[1:]):
        if phase < end:
            alpha = (phase - start) / (end - start)
            alpha = alpha * alpha * (3.0 - 2.0 * alpha)
            return {name: value + alpha * (after[name] - value)
                    for name, value in before.items()}


class PoseFilter:
    """Apply exponential smoothing and a bounded joint speed."""

    def __init__(self, model, smoothing=0.10, max_speed=2.5):
        self.model = model
        self.smoothing = smoothing
        self.max_speed = max_speed
        self.neutral = wave_pose(0.0)
        self.pose = model.clamp(self.neutral)

    def update(self, target, dt):
        dt = max(0.0, min(float(dt), 0.1))
        alpha = 1.0 if self.smoothing == 0 else -math.expm1(-dt / self.smoothing)
        for name, value in self.model.clamp({**self.neutral, **target}).items():
            speed = min(self.max_speed, self.model.limits[name][2])
            delta = (value - self.pose[name]) * alpha
            self.pose[name] += max(-speed * dt, min(speed * dt, delta))
        return dict(self.pose)


def retarget_landmarks(model, landmarks, previous, confidence=0.6):
    """Map MediaPipe world landmarks to arms in a body-relative frame.

    Anatomical left maps to robot left. Occluded limbs relax to neutral.
    Legs remain at zero; this is kinematic visualization, not balance control.
    """
    def point(index):
        landmark = landmarks[index]
        if min(landmark.visibility, landmark.presence) < confidence:
            raise ValueError('Landmark is occluded')
        return np.array([landmark.x, landmark.y, landmark.z])

    pose = {}
    try:
        left, right = point(11), point(12)
        up = unit((left + right - point(23) - point(24)) / 2)
        lateral = unit(left - right)
        forward = unit(np.cross(lateral, up))
        lateral = unit(np.cross(up, forward))
        basis = np.column_stack((forward, lateral, up))
    except (ValueError, IndexError):
        return pose
    for side, indices in [('left', (11, 13, 15)), ('right', (12, 14, 16))]:
        try:
            shoulder, elbow, wrist = [point(index) for index in indices]
            pose.update(model.solve_arm(
                side, basis.T @ (elbow - shoulder), basis.T @ (wrist - elbow), previous))
        except ValueError:
            continue
    pose['waist_roll_joint'] = float(np.clip(-math.atan2(up[0], -up[1]), -0.3, 0.3))
    return pose
