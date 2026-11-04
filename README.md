# Unitree G1 motion playground

A ROS 2 Jazzy workspace for animating the Unitree G1 29-DOF model in RViz.
The demo can wave, perform a predefined dance without a camera, mirror your
arms using an Astra Pro camera, and record or replay joint motion.

This is kinematic visualization. The pelvis stays fixed, the legs stay still
in webcam mode, and there is no balance or contact controller. The demo publishes
joint states for RViz; it does not send commands to a physical robot.

## Quick start on this machine

The workspace has already been built, and the webcam dependencies and pose model
are installed. Open a terminal and run:

```bash
cd /home/amadeus/g1_unitree_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch g1_motion_demo demo.launch.py mode:=webcam camera:=48 \
  python_executable:="$PWD/.venv/bin/python3" \
  model_path:="$PWD/models/pose_landmarker_lite.task"
```

This opens the camera preview and RViz. Face the camera and step back until
your shoulders, elbows, wrists, and hips are visible. Start with slow arm raises
and elbow bends. The preview should display **Tracking arms**.

Your anatomical left arm controls the G1's left arm. The camera preview is
flipped like a mirror, and the RViz camera initially faces the robot. A small
waist lean follows your torso; wrists and fingers are not tracked.

Press **Q** or **Esc** in the camera preview, close RViz, or press **Ctrl+C** in
the launching terminal to stop. Run one demo at a time and stop it before
switching modes. The demo launch includes its own robot-state publisher and
joint publisher; the original slider or Gazebo launch is not needed.

## Verified setup

Validated on **1 November 2026**:

| Component | Tested configuration |
| --- | --- |
| OS | Ubuntu 24.04.5 LTS |
| ROS | ROS 2 Jazzy, installed at `/opt/ros/jazzy` |
| Python | 3.12.3 |
| Robot | `g1_unitree_description/urdf/g1_29dof.urdf`, 29 movable joints |
| Camera | Astra Pro HD Camera, color stream |
| Camera device | `/dev/video48`, selected with `camera:=48` |
| Capture | 640 × 480, reported 30 fps; frames successfully read |
| Pose estimator | MediaPipe 0.10.35, Pose Landmarker Lite |
| OpenCV | `opencv-contrib-python` 4.11.0.86 in `.venv` |
| NumPy | 1.26.4 |
| Viewer | RViz 2 |

Live webcam mirroring was verified: one joint-state publisher delivered 210
messages in seven seconds, with changes in both arms' shoulder and elbow angles.
All 13 automated motion tests passed. Wave motion, ROS transforms,
recording/replay, and the rendered RViz model were also checked.

The Astra color stream is sufficient for this demo; its depth stream is not used.
Camera device numbers can change after reconnecting or rebooting.

## First-time setup or rebuilding

These instructions assume ROS 2 Jazzy is already installed. You need `colcon`,
the ROS package dependencies, and Python's `venv` and `pip` tools. Webcam setup
requires internet access to download Python dependencies and the pose model.
Use a terminal with desktop access for RViz and the camera preview.

From the workspace root:

```bash
cd /home/amadeus/g1_unitree_ws
source /opt/ros/jazzy/setup.bash

# On a new machine with rosdep initialized, install declared ROS dependencies.
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y

colcon build --base-paths src --symlink-install --packages-up-to g1_motion_demo
source install/setup.bash

# Only needed for webcam/video mirroring.
bash scripts/setup_webcam.sh
```

If Python reports that `ensurepip` or `venv` is unavailable, install the Ubuntu
`python3-venv` and `python3-pip` packages before running the setup script again.
On another machine, replace the workspace path with your checkout location.

The webcam setup script creates `.venv` with `--system-site-packages`, allowing
it to import the system ROS Python packages. It installs the requirements in
[`requirements-webcam.txt`](src/g1_motion_demo/requirements-webcam.txt) and
downloads the model to `models/pose_landmarker_lite.task`. Python packages are
installed into the workspace environment rather than globally. The tested
versions above describe this installation; the requirements file uses version
ranges, so a later setup may resolve different compatible versions.

Always pass `python_executable` when using webcam mode. Activating a virtual
environment alone does not change the installed ROS executable's interpreter.
Wave, dance, and replay modes do not require MediaPipe or a camera.

## Finding the Astra camera

The device tested successfully was `/dev/video48`. The Astra also exposed
`/dev/video49`, but that node did not provide a usable video capture in testing.
The many lower-numbered Intel IPU6 capture nodes belong to the laptop's camera
pipeline; a USB webcam is not necessarily camera 0 or camera 1.

Inspect devices from a normal host terminal:

```bash
v4l2-ctl --list-devices
ls -l /dev/v4l/by-id/
```

`v4l2-ctl` is provided by the Ubuntu `v4l-utils` package. On this machine, the
Astra's persistent device link is:

```text
/dev/v4l/by-id/usb-Astra_Pro_HD_Camera_Astra_Pro_HD_Camera-video-index0
```

To resolve its current number and launch without hard-coding 48, run the
following after sourcing ROS and the workspace from the quick start:

```bash
camera_device="$(readlink -f /dev/v4l/by-id/usb-Astra_Pro_HD_Camera_Astra_Pro_HD_Camera-video-index0)"
if [[ "$camera_device" =~ ^/dev/video([0-9]+)$ && -c "$camera_device" ]]; then
  camera_index="${BASH_REMATCH[1]}"
  ros2 launch g1_motion_demo demo.launch.py mode:=webcam camera:="$camera_index" \
    python_executable:="$PWD/.venv/bin/python3" \
    model_path:="$PWD/models/pose_landmarker_lite.task"
else
  echo "Astra camera not found. Reconnect it and check /dev/v4l/by-id/."
fi
```

Some sandboxes hide `/dev/video*` even when the host detects the camera. That
caused the initial camera check to miss this device. Check from the host terminal
before concluding that no camera is connected.

## Demo modes

All commands below assume the workspace root is the current directory and that
both `/opt/ros/jazzy/setup.bash` and `install/setup.bash` have been sourced.

### Wave

```bash
ros2 launch g1_motion_demo demo.launch.py
```

The right hand waves in a smooth seven-second loop. Add `speed:=0.5` to slow the
animation to half speed.

### Predefined dance (no camera)

```bash
ros2 launch g1_motion_demo demo.launch.py mode:=dance
```

The 30-second Michael Jackson-inspired routine loops automatically: hat-tip
gestures, diagonal arm points, shoulder hits, alternating heel-toe slides, and a
finishing waist lean. Poses include brief holds and eased transitions. No camera,
video, MediaPipe, music file, or recording is needed.
Add `speed:=0.5` for half speed, or `speed:=1.25` for a faster routine. The
existing smoothing and joint-speed limits apply. Close RViz or press Ctrl+C to
stop; `loop` only controls replay mode.

This is animation with a fixed pelvis: the moonwalk-style section stays in place,
and the feet can lift or slide relative to the grid. It does not provide balance
or foot-contact control for a physical G1. The hand gestures use the existing
rigid hands; no hat or finger animation is added. The predefined poses and their
timing are in `_dance_keyframes()`, sampled by `dance_pose()`, in
[`motion.py`](src/g1_motion_demo/g1_motion_demo/motion.py).

### Webcam mirror

Use the Astra command in the quick start, or select another `/dev/videoN`
capture device with `camera:=N`. If an arm becomes occluded, it relaxes smoothly.
If the torso cannot be tracked, both arms relax. Tracking uses local inference;
fast motion, crossed arms, and side-on poses can produce imperfect estimates.

### Video input

```bash
ros2 launch g1_motion_demo demo.launch.py mode:=webcam \
  video_path:=/absolute/path/to/clip.mp4 \
  python_executable:="$PWD/.venv/bin/python3" \
  model_path:="$PWD/models/pose_landmarker_lite.task"
```

`video_path` takes precedence over `camera`. Video is read one frame per inference
tick, rather than synchronized to the file's original frame rate. The launch
stops when the video ends.

For execution without windows, add `rviz:=false`; for webcam/video mode also add
`preview:=false`. Turning off the preview does not turn off camera capture.

## Recording and replay

Add `record_path` to wave, dance, or webcam mode. To record a webcam routine:

```bash
ros2 launch g1_motion_demo demo.launch.py mode:=webcam camera:=48 \
  python_executable:="$PWD/.venv/bin/python3" \
  model_path:="$PWD/models/pose_landmarker_lite.task" \
  record_path:="$PWD/recordings/my_motion.jsonl"
```

Stop the demo, then replay the saved motion:

```bash
ros2 launch g1_motion_demo demo.launch.py mode:=replay \
  replay_path:="$PWD/recordings/my_motion.jsonl"
```

A verified wave recording is already present in this workspace:

```bash
ros2 launch g1_motion_demo demo.launch.py mode:=replay \
  replay_path:="$PWD/recordings/example_wave.jsonl"
```

Recordings contain a versioned JSON Lines header with joint names, followed by
timestamped joint angles in radians. Camera images are not recorded. Completed
frames are line-buffered, and an existing recording file is never overwritten;
choose a new filename for each take.

Replay loops by default. Add `loop:=false` to hold the final pose or `speed:=0.5`
for half-speed playback. Smoothing and joint-speed limits still apply, so rapid
motion can be softened. Use similar first and last poses for a seamless loop.

The `recordings/` directory is ignored by Git. The example recording is a local
artifact and will not automatically be included in a fresh checkout.

## How motion reaches RViz

The webcam pipeline estimates body landmarks, converts arm directions into a
body-relative frame, and fits the G1's shoulder and elbow angles using the actual
URDF geometry. The motion filter smooths changes and enforces position and speed
limits before publishing all 29 joint positions.

`robot_state_publisher` converts those positions into transforms, and RViz
renders the existing meshes. The original description files are unchanged.

| Topic | Purpose |
| --- | --- |
| `/g1_demo/joint_states` | All 29 joint positions |
| `/g1_demo/robot_description` | Robot model used by RViz |
| `/g1_demo/tf` | Moving link transforms |
| `/g1_demo/tf_static` | Fixed link transforms |

The demo uses the `/g1_demo` namespace and remaps its TF topics. RViz uses
`pelvis` as its fixed frame.

## Launch arguments

| Argument | Default | Meaning |
| --- | --- | --- |
| `mode` | `wave` | `wave`, `dance`, `webcam`, or `replay` |
| `camera` | `0` | Camera index; use `48` for the Astra as tested |
| `python_executable` | Launch interpreter | Use `$PWD/.venv/bin/python3` for webcam mode |
| `model_path` | Empty | Path to the MediaPipe `.task` model; required in webcam mode |
| `video_path` | Empty | Video file to use instead of a camera |
| `rviz` | `true` | Open RViz |
| `preview` | `true` | Show the webcam/video preview |
| `rate` | `30.0` | Requested update rate in Hz, up to 120; inference can lower it |
| `smoothing` | `0.10` | Smoothing time constant in seconds; zero disables smoothing |
| `max_joint_speed` | `2.5` | Joint speed cap in rad/s, also bounded by the URDF |
| `speed` | `1.0` | Positive wave/dance/replay timeline multiplier; does not change webcam speed |
| `loop` | `true` | Loop replay; false holds the final pose |
| `record_path` | Empty | New output JSONL file; parent directories are created |
| `replay_path` | Empty | Input JSONL recording; required for replay |

List the launch arguments with:

```bash
ros2 launch g1_motion_demo demo.launch.py --show-args
```

## Verification

Run the package tests:

```bash
colcon test --base-paths src --packages-select g1_motion_demo --event-handlers console_direct+
colcon test-result --test-result-base build/g1_motion_demo
```

Expected result for this version: **13 tests, 0 errors, 0 failures**. Tests cover
wave geometry, joint limits, smoothing, arm retargeting, lost tracking, and
recording validation/interpolation.

With one demo running, inspect its live output in a second sourced terminal:

```bash
ros2 topic hz /g1_demo/joint_states
ros2 topic info /g1_demo/joint_states --verbose
ros2 topic echo /g1_demo/joint_states --once
```

There should be exactly one publisher on `/g1_demo/joint_states`. The default
target is 30 Hz; the achieved webcam rate depends on camera capture and inference.

## Troubleshooting

| Symptom | What to check |
| --- | --- |
| `Package 'g1_motion_demo' not found` | Build the workspace and source `install/setup.bash` in the current terminal. |
| Camera cannot open | Recheck its device number or persistent link. Close other applications using the camera and verify host access to `/dev/videoN`. |
| `No module named mediapipe` | Run `bash scripts/setup_webcam.sh` and pass `python_executable:="$PWD/.venv/bin/python3"`. |
| Pose model not found | Run the setup script and pass the full `model_path` from the quick start. |
| Preview asks to show arms and hips | Step back, improve lighting, and keep both shoulders, elbows, wrists, and hips visible. |
| Robot alternates between unrelated poses | Stop duplicate demo launches; check that the joint-state topic has one publisher. |
| Recording already exists | Choose a new `record_path`; overwriting is intentionally refused. |
| No desktop/display available | Use `rviz:=false` and, for webcam/video, `preview:=false`. |

Launching from a Snap-installed editor initially caused RViz to fail with a
`GLIBC_PRIVATE` / `libpthread.so.0` error. The demo launch now clears inherited
Snap `GTK_PATH` and `GIO_MODULE_DIR` values for its GUI processes. It does not
change the parent shell's environment.

The existing URDF can produce a KDL warning about inertia on the root `pelvis`
link. This did not prevent robot-state publishing or RViz visualization in the
verified setup.

## Workspace layout

```text
g1_unitree_ws/
├── README.md
├── scripts/setup_webcam.sh
├── src/
│   ├── g1_unitree_description/    # Original URDF, meshes, RViz/Gazebo resources
│   └── g1_motion_demo/
│       ├── g1_motion_demo/        # Motion, webcam input, ROS node, recording
│       ├── launch/demo.launch.py
│       ├── rviz/demo.rviz
│       ├── test/test_motion.py
│       └── requirements-webcam.txt
├── .venv/                       # Generated webcam Python environment
├── models/                      # Downloaded pose model
├── recordings/                  # Local motion recordings
├── build/                       # Generated by colcon
├── install/                     # Generated ROS workspace overlay
└── log/                         # Colcon build/test logs
```

Package-level usage is also documented in
[`src/g1_motion_demo/README.md`](src/g1_motion_demo/README.md). The camera estimator
uses the [MediaPipe Pose Landmarker Python API](https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker/python).
