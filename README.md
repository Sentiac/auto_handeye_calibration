# Automatic Hand-Eye Calibration for ROS 2

Robot-independent, automatic hand-eye calibration for six-axis arms using ROS 2, MoveIt and a ChArUco target.

The operator places the arm at any safe pose from which the complete board is visible. The node then generates target-relative viewpoints, asks MoveIt to plan and collision-check every motion, captures timestamp-matched image/robot observations, evaluates calibration seeds, refines the result and selects additional informative views.

The repository contains no robot driver, robot model, controller or custom IK solver. If your six-DoF arm already plans and executes poses through MoveIt, it can be integrated through YAML parameters.

> **Status: experimental.** The eye-in-hand workflow has been exercised end-to-end in Gazebo on one six-axis arm. Eye-to-hand support and the example Doosan configuration compile but have not yet received an end-to-end simulation or hardware validation. Automatic physical motion must be commissioned gradually.

## Features

- Eye-in-hand (`camera relative to flange`) and eye-to-hand (`camera relative to base`) frame models.
- No fixed starting joint pose; the only starting requirement is full board visibility.
- ChArUco corner observations retained as raw 2-D/3-D correspondences.
- TF lookup at the image timestamp—never a silent latest-TF substitution.
- Generic target-relative bootstrap view generation.
- All five OpenCV hand-eye initializers for eye-in-hand calibration.
- Joint nonlinear refinement through ROS-Industrial `industrial_calibration`.
- Information-ranked follow-up views using predicted corner Jacobians.
- MoveIt IK, joint-limit, collision and complete trajectory validation before execution.
- Conservative motion scaling and hardware automatic-execution lockout by default.
- Reproducible session directories containing images, poses, corners, candidate decisions and results.

## Supported software

- Ubuntu 24.04
- ROS 2 Jazzy
- MoveIt 2
- OpenCV 4.6 or newer with the ArUco module
- ROS-Industrial `industrial_calibration` 1.2.0

The versions used for the reference test are recorded in [DEPENDENCIES.md](DEPENDENCIES.md).

## Build

Create a clean workspace and clone this repository as one package:

```bash
mkdir -p ~/active_handeye_ws/src
cd ~/active_handeye_ws/src
git clone https://github.com/Sentiac/auto_handeye_calibration.git auto_handeye_calibration
vcs import . < auto_handeye_calibration/dependencies.repos

cd ~/active_handeye_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to auto_handeye_calibration
source install/setup.bash
```

## Robot requirements

Bring up the robot before starting calibration. The running system must provide:

- a MoveIt planning group for the arm;
- a working trajectory execution controller;
- current `/joint_states`;
- a connected TF tree containing the configured base, flange and optical frames;
- an RGB `sensor_msgs/Image` topic and matching `CameraInfo`;
- collision geometry for the robot and relevant environment.

The calibration package does not change the robot's kinematics plugin. KDL, IKFast, TRAC-IK, analytical solvers and vendor solvers are all acceptable if MoveIt can plan the requested flange poses reliably.

## Configure an arm

Copy [config/common.yaml](config/common.yaml) and set at least:

```yaml
auto_handeye_calibration:
  ros__parameters:
    calibration.mode: eye_in_hand  # or eye_to_hand
    frames.base: base_link
    frames.flange: tool0
    frames.camera_optical: camera_color_optical_frame
    topics.image: /camera/color/image_raw
    topics.camera_info: /camera/color/camera_info
    topics.joint_states: /joint_states
    moveit.planning_group: manipulator
    target.dictionary: DICT_5X5_250
    target.squares_x: 5
    target.squares_y: 7
    target.square_length_m: 0.038268
    target.marker_length_m: 0.023960
```

Lengths are metres. `frames.camera_optical` must be the optical frame associated with the configured image—not the camera housing frame. Board square counts and dimensions must match the printed target.

## Obtain a calibration target

We recommend [CalibrX](https://calibrx.io/) for obtaining calibration-target
printouts. This repository intentionally does not distribute target images or
a target generator.

Create or order a ChArUco target, then enter its dictionary, square count,
measured square length and measured marker length in the calibration YAML.
Print downloadable targets at **100% / actual size** with "fit to page"
disabled. Measure several printed squares and markers with calipers; the
measured dimensions, not nominal printer settings, must be used by the
calibration node.

At startup, all ChArUco corners are required by default. Later views may be partially visible according to `capture.minimum_corner_fraction`.

## Run

First start the robot's normal driver, controllers and MoveIt `move_group`. Then run:

```bash
ros2 launch auto_handeye_calibration auto_handeye.launch.py \
  config:=/absolute/path/to/my_robot.yaml
```

Physical automatic execution is disabled by default. First inspect the camera,
TF tree and planning scene. To allow the node to execute MoveIt trajectories,
set `safety.automatic_execution_enabled: true` in the robot YAML while keeping
`safety.operator_arm_required: true`, then arm the session explicitly:

```bash
ros2 service call /auto_handeye_calibration/start std_srvs/srv/Trigger
```

Controls:

```bash
ros2 service call /auto_handeye_calibration/pause std_srvs/srv/Trigger
ros2 service call /auto_handeye_calibration/abort std_srvs/srv/Trigger
ros2 topic echo /auto_handeye_calibration/status
```

An abort prevents further trajectories. It intentionally does not improvise an unplanned retreat.

## Output

Each run creates `session_<timestamp>` below `session.output_directory`:

```text
session_<timestamp>/
├── config_snapshot.yaml
├── dependency_versions.json
├── samples/
│   ├── sample_000.png
│   └── sample_000.yaml
├── candidates.jsonl
├── final_calibration.yaml
└── report.json
```

The final YAML states the parent and child frames, transform convention, units and quaternion order. Inspect a report with:

```bash
ros2 run auto_handeye_calibration analyze_session.py /path/to/session_<timestamp>
```

## Safety

This software commands robot motion and is not a safety-rated system. Commission it in stages: candidate preview, reduced-speed single moves, operator-confirmed bootstrap, then automatic operation only after the planning scene and emergency-stop path have been verified. See [docs/SAFETY.md](docs/SAFETY.md).

## Validation performed

The reference Gazebo eye-in-hand run collected 10 accepted views, evaluated the five OpenCV seeds, used information gain for the final views and converged to `0.065 px` training reprojection RMS. Against the simulated camera mount, the recovered transform differed by approximately `0.9 mm` and `0.04°`.

Unit tests cover transform direction, optical look-at geometry, information gain and zero-noise nonlinear recovery:

```bash
colcon test --packages-select auto_handeye_calibration
colcon test-result --test-result-base build/auto_handeye_calibration --verbose
```

## Known limitations

- Only ROS 2 Jazzy is currently tested.
- Eye-to-hand has not yet been validated end-to-end.
- The Doosan YAML is an unvalidated integration example, not a claim of support.
- The current report contains training residuals; it does not yet automate held-out physical validation.
- Dynamic obstacles are only considered if the user's MoveIt planning scene contains them.
- Camera intrinsic calibration is assumed to be correct and is not estimated here.

## Contributing and license

See [CONTRIBUTING.md](CONTRIBUTING.md). This project is provided under the [MIT License](LICENSE).
