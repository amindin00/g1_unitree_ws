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
