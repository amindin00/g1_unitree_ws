"""Publish a wave, a webcam pose, or a recording as ROS joint states."""

import math
import signal
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .motion import PoseFilter, RobotModel, wave_pose
from .recording import Playback, Recorder


class MotionDemo(Node):
    """One publisher for all 29 joints, with optional recording."""

    def __init__(self):
        super().__init__('motion_demo')
        self.camera = self.recorder = None
        try:
            self.configure()
        except Exception:
            self.close()
            self.destroy_node()
            raise

    def configure(self):
        defaults = {
            'urdf_path': '', 'mode': 'wave', 'rate': 30.0, 'speed': 1.0,
            'loop': True, 'smoothing': 0.10, 'max_joint_speed': 2.5,
            'record_path': '', 'replay_path': '', 'model_path': '',
            'camera': 0, 'video_path': '', 'preview': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        settings = {name: self.get_parameter(name).value for name in defaults}
        self.mode = settings['mode']
        if self.mode not in ('wave', 'webcam', 'replay'):
            raise ValueError('mode must be wave, webcam, or replay')
        for name in ('rate', 'speed', 'max_joint_speed'):
            if not math.isfinite(settings[name]) or settings[name] <= 0:
                raise ValueError(f'{name} must be a positive finite number')
        if not math.isfinite(settings['smoothing']) or settings['smoothing'] < 0:
            raise ValueError('smoothing must be nonnegative and finite')
        if settings['rate'] > 120:
            raise ValueError('rate must not exceed 120 Hz')
        self.model = RobotModel(settings['urdf_path'])
        self.filter = PoseFilter(self.model, settings['smoothing'], settings['max_joint_speed'])
        self.playback = None
        if self.mode == 'replay':
            if not settings['replay_path']:
                raise ValueError('Set replay_path to a recording file')
            self.playback = Playback(settings['replay_path'], self.model.names)
        if self.mode == 'webcam':
            from .webcam import Webcam
            self.camera = Webcam(self.model, settings['model_path'], settings['camera'],
                                 settings['video_path'], settings['preview'])
        if settings['record_path']:
            self.recorder = Recorder(settings['record_path'], self.model.names)
        self.speed, self.loop = settings['speed'], settings['loop']
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.started = self.previous_time = time.monotonic()
        self.timer = self.create_timer(1.0 / settings['rate'], self.tick)
        self.get_logger().info(
            f'{self.mode} mode: publishing {len(self.model.names)} joints for RViz. '
            + (f'Recording to {settings["record_path"]}.' if self.recorder else ''))

    def tick(self):
        if self.camera:
            try:
                target = self.camera.read()
            except StopIteration as error:
                self.get_logger().info(str(error))
                self.timer.cancel()
                rclpy.try_shutdown()
                return
        now = time.monotonic()
        elapsed = now - self.started
        if self.mode == 'wave':
            target = wave_pose(elapsed * self.speed)
        elif self.playback:
            target = self.playback.sample(elapsed * self.speed, self.loop)
        pose = self.filter.update(target, now - self.previous_time)
        self.previous_time = now
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = self.model.names
        message.position = [pose[name] for name in message.name]
        self.publisher.publish(message)
        if self.recorder:
            self.recorder.append(elapsed, pose)

    def close(self):
        if self.camera:
            self.camera.close()
        if self.recorder:
            self.recorder.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    failed = False
    try:
        node = MotionDemo()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        print(f'G1 demo: {error}', file=sys.stderr)
        failed = True
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node:
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()
    if failed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
