# Dependency baseline

- ROS 2 Jazzy
- MoveIt 2 2.12.4
- OpenCV 4.6.0
- Ceres Solver 2.2.0
- `industrial_calibration` 1.2.0
- `boost_plugin_loader` 0.2.1
- `ros_industrial_cmake_boilerplate` 0.5.4

Pinned source dependencies are recorded in the repository-root
`dependencies.repos` file. Import them into the workspace with:

```bash
vcs import src < src/auto_handeye_calibration/dependencies.repos
```

The remaining dependencies are resolved through `rosdep` from `package.xml`.
