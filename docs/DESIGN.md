# Generic active hand-eye design

The package has no robot model or starting joint pose in its algorithm. Robot differences are parameters: MoveIt group, base/flange/camera frames, topics, and safety policy. The first valid observation supplies the target location. Candidate views are generated relative to that target and converted to flange poses using the nominal TF mount only for planning.

`T_A_B` always maps coordinates from B to A. For eye-in-hand, ROS-Industrial jointly estimates `T_E_C` and `T_B_T`. For eye-to-hand, the same optimizer is configured with a stationary camera mount and moving target mount and estimates `T_B_C` and `T_E_T`.

Images are paired with `base -> flange` using TF at the image timestamp. Latest-TF lookup is deliberately not used. Raw images, target corner pixels, target coordinates, robot poses, quality metrics, candidate decisions, and results are stored in a timestamped session.

MoveIt owns IK, collision checking, joint constraints, planning and execution. A candidate rejected by MoveIt is recorded and skipped; collision checking is never bypassed.
