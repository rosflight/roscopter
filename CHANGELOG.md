# Changelog

## [2.0.0-beta.1](https://github.com/rosflight/roscopter/compare/v2.0.0-beta...v2.0.0-beta.1) (2025-12-15)


### Bug Fixes

* update controller to use correct state definition ([3db4365](https://github.com/rosflight/roscopter/commit/3db4365ce46f4c200c3bda562c33b24fb3bb0954))

## [2.0.0-beta](https://github.com/rosflight/roscopter/compare/v1.0.0...v2.0.0-beta) (2025-12-12)


### ⚠ BREAKING CHANGES

* update to ROS2

### Features

* add release-please workflow ([e76970f](https://github.com/rosflight/roscopter/commit/e76970f793bd0ece86cf0ca2569d1dbb806616ba))
* Added a sensor monitor that warns the user if an sensor has not ([38c0b82](https://github.com/rosflight/roscopter/commit/38c0b82f096dbe978cf5ab5c49d521f87bdcd367))
* Added a sensor monitor that warns the user if an sensor has not ([1c9f4f6](https://github.com/rosflight/roscopter/commit/1c9f4f6e7efbfb329e9603454118df4340cfd1e1))
* Added dynamic air density calculation. ([ddf48d8](https://github.com/rosflight/roscopter/commit/ddf48d8c88f90134d6bd63978ffb4d3f11a60994))
* added Ian's estimator to launch scripts and added quaternion to the estimated state output ([8f5c0c8](https://github.com/rosflight/roscopter/commit/8f5c0c8bc3721c316290309237a2f20e70f990a3))
* added sub to status to turn off integrators when rc has control ([39aa443](https://github.com/rosflight/roscopter/commit/39aa443fbf0b4b025b6b73394d889af014c986a8))
* cleaned up trajectory follower ([5051306](https://github.com/rosflight/roscopter/commit/5051306acbe596154491205ede950c41ad263d60))
* standardized the cascading loops to be in different frames ([73f27e9](https://github.com/rosflight/roscopter/commit/73f27e954e55e7a795de1d10f7115012b020b6cb))
* switched to a trajectory-based trajectory follower using differential flatness principles ([5919e62](https://github.com/rosflight/roscopter/commit/5919e622275ffd0c5b1faeff7587b8efb4b573fb))
* switched traj follower to use mode=RPYTHROTTLE, and tuned in sim ([b0b41b3](https://github.com/rosflight/roscopter/commit/b0b41b39b2f00165aee80e2611f7ba28917f7fef))
* traj follower with ch 14 implementation of path manager ([d8aaff5](https://github.com/rosflight/roscopter/commit/d8aaff5ea4b2be0d27795360dba5e7c796b6d483))
* update to ROS2 ([e76970f](https://github.com/rosflight/roscopter/commit/e76970f793bd0ece86cf0ca2569d1dbb806616ba))
* updated roscopter launch files to mirror rosplane ([8bb3bde](https://github.com/rosflight/roscopter/commit/8bb3bde47ab57d18f1bcab533bab6d5b37bc2660))
* updates to roscopter signal generator ([5643640](https://github.com/rosflight/roscopter/commit/5643640b45a599ca0cfcd767a1bd8f907e704eb7))


### Bug Fixes

* add roscopter_gcs to roscopter_sim launch file ([#48](https://github.com/rosflight/roscopter/issues/48)) ([b62d47c](https://github.com/rosflight/roscopter/commit/b62d47c815c430fbe026e7eff4505534ec34ad83))
* changed trajectory follower wrap while loop to a single jump ([4ed5d62](https://github.com/rosflight/roscopter/commit/4ed5d62f329abce2a49659268f7df7d6fd2c9c8f))
* dependencies for Jazzy ([4268e46](https://github.com/rosflight/roscopter/commit/4268e462b2b9b97d7d6eee0a72efb1463b1c4e01))
* dependencies for Jazzy ([5a43d42](https://github.com/rosflight/roscopter/commit/5a43d42ffb402eda8304d4affc0502347c174a49))
* fixed initializion bug after clearing waypoints and wrapping bug ([e606f21](https://github.com/rosflight/roscopter/commit/e606f211ba78216eb35201ccb85366b7557ae702))
* fixed path manager previous waypoint initialization ([6fc9d3d](https://github.com/rosflight/roscopter/commit/6fc9d3d6f8a170fd5b829efec7fc6e2f3f271e5d))
* Fixed the declination added in the wrong direction. ([eb4ab89](https://github.com/rosflight/roscopter/commit/eb4ab8995b46487b5d0608e1ed2843b516a0273d))
* Fixed the declination added in the wrong direction. ([0dbabb2](https://github.com/rosflight/roscopter/commit/0dbabb2c5369b7e313767af9f02df3c0487c16d9))
* Fixed tilt-compensated mag uncert and EKF function refrences. ([f9b99d0](https://github.com/rosflight/roscopter/commit/f9b99d014f83d9a5208403cb72a14c04b5cbcb75))
* Fixed tilt-compensated mag uncert and EKF function refrences. ([68366c1](https://github.com/rosflight/roscopter/commit/68366c127e7840e7fcc7a4c657d821d09860e853))
* removed lpf on static pressure that causes time delay in altitude estimate. ([dff4e4d](https://github.com/rosflight/roscopter/commit/dff4e4d33f0886f6a7bc81397cd97a06d4538ce2))
* removed time delay causing LPF on the static pressure. ([fe67526](https://github.com/rosflight/roscopter/commit/fe67526b3aaa319a5406a208d7c9c62aa1e49fa4))
* trajectory follower saturate down commanded accel ([94bec2f](https://github.com/rosflight/roscopter/commit/94bec2f509e9357ec6431abcaca3231d22bfb7ed))
* trajectory follower saturate down commanded accel ([d89073e](https://github.com/rosflight/roscopter/commit/d89073eb5f95b6f2dd4e536171018e362ec751c0))
