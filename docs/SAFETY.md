# Motion safety

The calibration node never sends Cartesian commands directly to a controller. Every candidate is submitted to MoveIt as a flange pose. MoveIt performs IK, joint-limit, self-collision, environment-collision, and full trajectory planning before execution.

Hardware configurations default to `automatic_execution_enabled: false` and
`operator_arm_required: true`. The start service does not override the execution
interlock. After commissioning the robot at reduced controller limits, set
`automatic_execution_enabled: true` in that robot's YAML and arm each session
with:

```bash
ros2 service call /auto_handeye_calibration/start std_srvs/srv/Trigger
```

The Gazebo overlay is the only supplied configuration that enables unattended execution. Aborting prevents all subsequent motion but does not command an unplanned retreat:

```bash
ros2 service call /auto_handeye_calibration/abort std_srvs/srv/Trigger
```
