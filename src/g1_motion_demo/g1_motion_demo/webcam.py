"""Optional local MediaPipe camera input."""

from pathlib import Path
import time

from .motion import retarget_landmarks


class Webcam:
    """Read a webcam or video file and estimate one person's arm pose."""

    def __init__(self, model, model_path, camera=0, video='', preview=True):
        try:
            import cv2
            import mediapipe as mp
        except ImportError as error:
            raise RuntimeError(
                'Webcam dependencies are missing. Run scripts/setup_webcam.sh '
                'and set python_executable to .venv/bin/python3; see README.md.') from error
        if not Path(model_path).expanduser().is_file():
            raise ValueError('Pose model not found. Set model_path to pose_landmarker_lite.task.')
        self.cv2, self.mp = cv2, mp
        self.model, self.preview = model, preview
        self.capture = cv2.VideoCapture(str(Path(video).expanduser()) if video else camera)
        if not self.capture.isOpened():
            self.capture.release()
            raise RuntimeError(
                f'Cannot open {video or f"camera {camera}"}. '
                'Connect a camera, choose camera:=1, or use mode:=wave.')
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        options = mp.tasks.vision.PoseLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=str(Path(model_path).expanduser())),
            running_mode=mp.tasks.vision.RunningMode.VIDEO,
            num_poses=1,
            min_pose_detection_confidence=0.6,
            min_pose_presence_confidence=0.6,
            min_tracking_confidence=0.6,
        )
        try:
            self.detector = mp.tasks.vision.PoseLandmarker.create_from_options(options)
        except Exception:
            self.capture.release()
            raise
        self.previous = {}
        self.last_timestamp = -1
        self.started = time.monotonic()

    def read(self):
        """Return an arm target, or end the session when the preview is closed."""
        ok, frame = self.capture.read()
        if not ok:
            raise StopIteration('Camera stopped or video finished')
        rgb = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2RGB)
        timestamp = max(self.last_timestamp + 1, int((time.monotonic() - self.started) * 1000))
        self.last_timestamp = timestamp
        result = self.detector.detect_for_video(
            self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=rgb), timestamp)
        pose = {}
        if result.pose_world_landmarks:
            pose = retarget_landmarks(self.model, result.pose_world_landmarks[0], self.previous)
        self.previous = pose
        if self.preview:
            if result.pose_landmarks:
                points = result.pose_landmarks[0]
                height, width = frame.shape[:2]
                for a, b in [(11, 12), (11, 13), (13, 15), (12, 14), (14, 16),
                             (11, 23), (12, 24), (23, 24)]:
                    if min(points[a].visibility, points[b].visibility) < 0.6:
                        continue
                    endpoints = [(int(points[i].x * width), int(points[i].y * height))
                                 for i in (a, b)]
                    self.cv2.line(frame, *endpoints, (100, 230, 100), 3)
            # Flip only the display. Anatomical labels used for inference stay intact.
            frame = self.cv2.flip(frame, 1)
            status = 'Tracking arms' if any('elbow' in name for name in pose) else 'Step back: show arms and hips'
            self.cv2.putText(frame, status, (15, 30), self.cv2.FONT_HERSHEY_SIMPLEX,
                             0.65, (100, 230, 100), 2)
            self.cv2.putText(frame, 'Q / Esc: stop', (15, 58), self.cv2.FONT_HERSHEY_SIMPLEX,
                             0.55, (240, 240, 240), 1)
            self.cv2.imshow('G1 webcam mirror', frame)
            key = self.cv2.waitKey(1) & 0xff
            if key in (ord('q'), 27) or self.cv2.getWindowProperty(
                    'G1 webcam mirror', self.cv2.WND_PROP_VISIBLE) < 1:
                raise StopIteration('Preview closed')
        return pose

    def close(self):
        self.capture.release()
        self.detector.close()
        if self.preview:
            self.cv2.destroyAllWindows()
