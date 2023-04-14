# gtcal
Multi-camera intrinsics/extrinsics calibration using factor graphs

Currently, this is able to solve for a camera pose in the target frame using either Ceres solver or gtsam. 
I still need to add noise to both the camera measurements and initial poses to truly validate this but at least 
there's a baseline. There's a lot of cleanup to do but little by little I'll get it there.
