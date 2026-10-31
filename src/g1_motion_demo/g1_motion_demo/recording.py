"""Stream joint poses to JSON Lines and play them back."""

import bisect
import json
import math
from pathlib import Path


class Recorder:
    """Record angles only, never camera images; refuse to overwrite a file."""

    def __init__(self, path, names):
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        self.names = names
        self.stream = path.open('x', buffering=1)
        self.stream.write(json.dumps({'format': 'g1_motion_demo', 'version': 1,
                                      'joint_names': names}) + '\n')

    def append(self, seconds, pose):
        self.stream.write(json.dumps({'t': seconds,
                                      'positions': [pose[n] for n in self.names]},
                                     allow_nan=False) + '\n')

    def close(self):
        self.stream.close()


class Playback:
    """Validate a recording and interpolate its samples."""

    def __init__(self, path, expected_names):
        with Path(path).expanduser().open() as stream:
            header = json.loads(next(stream, '{}'))
            if header.get('format') != 'g1_motion_demo' or header.get('version') != 1:
                raise ValueError('Unsupported recording format')
            self.names = header.get('joint_names', [])
            if (len(self.names) != len(expected_names)
                    or set(self.names) != set(expected_names)):
                raise ValueError('Recording joint names do not match this G1 model')
            frames = [json.loads(line) for line in stream if line.strip()]
        if not frames:
            raise ValueError('Recording contains no poses')
        self.times, self.positions = [], []
        for frame in frames:
            timestamp = float(frame['t'])
            positions = list(map(float, frame['positions']))
            if (not math.isfinite(timestamp) or timestamp < 0
                    or len(positions) != len(self.names)
                    or not all(math.isfinite(value) for value in positions)
                    or (self.times and timestamp <= self.times[-1])):
                raise ValueError('Recording contains invalid angles or timestamps')
            self.times.append(timestamp)
            self.positions.append(positions)
        start = self.times[0]
        self.times = [value - start for value in self.times]
        self.duration = self.times[-1]

    def sample(self, seconds, loop=True):
        if loop and self.duration > 0:
            seconds %= self.duration
        seconds = max(0.0, min(seconds, self.duration))
        index = max(0, bisect.bisect_right(self.times, seconds) - 1)
        following = min(index + 1, len(self.times) - 1)
        span = self.times[following] - self.times[index]
        alpha = (seconds - self.times[index]) / span if span else 0.0
        return dict(zip(self.names, [
            a + alpha * (b - a)
            for a, b in zip(self.positions[index], self.positions[following])
        ]))
