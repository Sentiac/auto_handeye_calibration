# Contributing

Issues and pull requests are welcome. Before submitting a change:

1. Describe the robot, camera arrangement, ROS distribution and MoveIt version.
2. Keep robot-specific frame names and limits in YAML; do not branch the algorithm by robot model.
3. Add tests for transform-direction, projection or selection changes.
4. Build and test with ROS 2 Jazzy:

   ```bash
   colcon build --packages-up-to auto_handeye_calibration
   colcon test --packages-select auto_handeye_calibration
   colcon test-result --verbose
   ```

Never weaken collision, visibility or timestamp checks merely to make a robot pass. Do not include images, bags or calibration outputs containing sensitive facility information without permission.
