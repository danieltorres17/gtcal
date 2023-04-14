# gtcal
Multi-camera intrinsics/extrinsics calibration using factor graphs

Currently, this is able to solve for a camera pose in the target frame using either Ceres solver or gtsam. 
I still need to add noise to both the camera measurements and initial poses to truly validate this but at least 
there's a baseline. There's a lot of cleanup to do but little by little I'll get it there.

The goal is to set up a stereo pair (and maybe even more cameras with different types of models) and jointly solve for the extrinsics and intrinsics.

![gtcal](https://user-images.githubusercontent.com/29615268/232159611-ffacb76d-c550-44f7-97e6-e5e48a616739.png)
