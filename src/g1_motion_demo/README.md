# G1 motion playground

Make the 29-DOF G1 wave, dance without a camera, copy your arms through a webcam,
and record/replay a routine in RViz. This is kinematic animation: the pelvis stays
fixed, and there is no balance, contact, or collision controller. It publishes
visualization joint states under `/g1_demo`; it does not send motor commands.

## Start with a wave

From this workspace:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to g1_motion_demo
source install/setup.bash
ros2 launch g1_motion_demo demo.launch.py
```

The right hand waves in a seven-second loop. Close RViz or press Ctrl+C to
stop. This launch starts its own robot-state publisher and exactly one joint
publisher; the original slider/Gazebo launch is not needed. Run one demo at a
time. The original robot description is unchanged.

The launch clears inherited Snap GTK paths for its GUI processes when present,
so it can also start from a Snap-installed editor without a libc mismatch.

For a machine without a desktop, add `rviz:=false`. Inspect the animation with
`ros2 topic echo /g1_demo/joint_states`.

## Predefined dance (no camera)

After building and sourcing the workspace as above:

```bash
ros2 launch g1_motion_demo demo.launch.py mode:=dance
```

The 30-second Michael Jackson-inspired routine loops through hat-tip gestures,
diagonal arm points, shoulder hits, alternating heel-toe slides, and a finishing
waist lean. Brief holds and eased transitions give each gesture a distinct pose.
It needs no camera, MediaPipe, video, music file, or recording.
Add `speed:=0.5` for half speed or `speed:=1.25` for a faster routine. The usual
smoothing and joint-speed limits still apply. Close RViz or press Ctrl+C to stop.
The `loop` argument applies only to replay mode.

The pelvis stays fixed, so the moonwalk-style section is in place. Foot contact
is not enforced, and feet can lift or slide relative to the grid. The hands are
rigid and there is no hat prop. Customize the poses and timing in
`_dance_keyframes()` in `g1_motion_demo/motion.py`; `dance_pose()` interpolates
them. This is visualization, not physical robot control.

## Mirror your arms

Install the optional dependencies and Google's Pose Landmarker Lite model into
the workspace. This uses a virtual environment with access to system ROS Python
packages; it does not install Python packages globally.

```bash
bash scripts/setup_webcam.sh
ros2 launch g1_motion_demo demo.launch.py mode:=webcam camera:=48 \
  python_executable:="$PWD/.venv/bin/python3" \
  model_path:="$PWD/models/pose_landmarker_lite.task"
```

Face the camera and step back until both shoulders, elbows, wrists, and hips
are visible. Start with slow arm raises and elbow bends. Your anatomical left
arm controls the robot's left arm. The camera preview is flipped like a mirror;
the RViz camera starts facing the robot. Press Q or Esc in the camera window to
stop, or press Ctrl+C in the terminal.

- The verified Astra Pro HD Camera is `/dev/video48` (`camera:=48`), delivering
  640 × 480 color frames at 30 fps. Live arm mirroring was verified with this camera.
- Use `camera:=N` for `/dev/videoN`. Device numbers can change after reconnecting;
  check `v4l2-ctl --list-devices` and `/dev/v4l/by-id/` from a host terminal.
- Use `video_path:=/absolute/path/clip.mp4` to process a local video instead.
  Video is read one frame per inference tick, rather than synchronized to its
  original frame rate.
- Use `preview:=false` to suppress the camera window.
- If an arm is occluded, it relaxes smoothly; if the torso is lost, both arms
  relax. Legs stay still. A small waist lean follows your torso.

Tracking runs locally. Only joint angles are recorded. Monocular depth is
approximate, so side-on poses, crossed arms, and fast motions can look imperfect.
Wrists and fingers are not retargeted. The arm solver uses the actual URDF
geometry and fits upper-arm and forearm directions within joint limits.

MediaPipe reference:
[Pose Landmarker for Python](https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker/python).

## Record a routine and replay it

Add `record_path` to wave, dance, or webcam mode. Stop with Ctrl+C to finish:

```bash
ros2 launch g1_motion_demo demo.launch.py \
  record_path:="$PWD/recordings/my_wave.jsonl"
```

Then replay the recording:

```bash
ros2 launch g1_motion_demo demo.launch.py mode:=replay \
  replay_path:="$PWD/recordings/my_wave.jsonl"
```

A verified starter recording is available in this workspace at
`recordings/example_wave.jsonl`.

Recordings contain a versioned JSON Lines header and timestamped joint angles
in radians. Existing files are never overwritten: choose a new filename for
each take. Frames are line-buffered, so completed frames survive interruption.

Replay loops by default. Add `loop:=false` to hold the final pose, or `speed:=0.5`
for half speed (`speed` also controls wave and dance). Smoothing and the speed limit
also apply during replay; rapid movements may be softened. Loops interpolate
between recorded frames but do not add a bespoke transition between the last
and first pose; use similar start/end poses for a seamless routine.

## Tuning

| Argument | Default | Meaning |
| --- | --- | --- |
| `rate` | `30.0` | Requested update rate in Hz; camera inference can lower it |
| `smoothing` | `0.10` | Smoothing time constant in seconds; zero disables it |
| `max_joint_speed` | `2.5` | Visualization speed cap in rad/s, also bounded by URDF |
| `speed` | `1.0` | Wave/dance/replay timeline multiplier |

All 29 joint positions are published. Angles are clamped to URDF limits. Pauses
do not trigger large catch-up jumps. ROS topics are `/g1_demo/joint_states`,
`/g1_demo/robot_description`, `/g1_demo/tf`, and `/g1_demo/tf_static`.

## Checks

```bash
PYTHONPATH=src/g1_motion_demo python3 -m pytest -q src/g1_motion_demo/test
```

Tests cover the real URDF's wave geometry, joint limits, smoothing, arm
retargeting, lost tracking, and recording interpolation/validation.
