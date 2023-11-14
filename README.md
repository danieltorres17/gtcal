# gtcal
Camera intrinsics/extrinsics calibration using factor graphs

### TODO: Camera pose in target frame estimator front end.
- [X] Using Ceres solver.
- [ ] Modify Ceres solver front end to do optimization on manifold.
- [X] Using gtsam.

### TODO: Batch solver that optimizes for all poses and intrinsics jointly.
- [X] Using gtsam's ISAM2 jointly solve for poses and calibration.
- [ ] Re-do/clean up how the cameras are passed into and modified in the `BatchSolver::State` struct.
- [ ] Create a stereo factor to allow for multi-camera rig setup.
- [ ] Using Ceres solver?

### TODO: Visualization.
- [X] Setup basic visualization with PCL (it crashes so either I need to debug this or move to something else).
- [ ] Create visualization using Pangolin?
- [ ] Update visualization to see pose estimates and even camera projections live for each iteration.

### TODO: Misc.
- [ ] Re-do the gtcal::Camera interface in `gtcal/camera.h`.
- [ ] Upload the Dockerfile and setup scripts.

Note: pose variables are named as follows `pose_target_cam` meaning the pose of the camera relative to the target or the pose of the camera in the target frame. All point sets are also labeled in their corresponding reference frame. Ex: `pts3d_target` denotes the target points in the target frame.

The goal is to set up a stereo pair (and maybe even more cameras with different types of models) and jointly solve for the extrinsics and intrinsics. Currently the individual components are there: a front-end pose solver, the batch solver backend, wrapper class for gtsam camera models. To enable a fully functioning pipeline, there are still many steps needed to join the front and backend as well as keep track of the state of the system and a relative camera factor but it's doable. The code quality can be improved as well, this was a project I used to mainly get familiar with gtsam a bit more but I took the liberty to explore more of C++'s modern features like `std::variant`. I'm currenty happy with how it is but I might come back to it.

The unit tests are where you'll find how the individual components work (I left the visualizations commented out since PCL keeps crashing). I left a ton of comments but let me know if you have any questions and I'll do my best to answer them.

![gtcal](https://user-images.githubusercontent.com/29615268/232159611-ffacb76d-c550-44f7-97e6-e5e48a616739.png)
|:--:|
|Camera poses sampled around the target.|

![gtcal_default_cam_poses](https://github.com/danieltorres17/gtcal/assets/29615268/a27a1fbf-2e74-4824-ab35-cd7ee709727c)
|:--:|
|Default camera poses used for testing. The test set includes 25 poses in total where the roll and pitch angles are varied at each of the locations. Other functions exist to randomly sample poses around the target.|
