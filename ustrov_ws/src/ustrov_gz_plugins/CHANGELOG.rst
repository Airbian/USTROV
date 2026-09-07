^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ustrov_gz_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.3 (2024-11-28)
------------------
* fix sign error and add much needed comments
* Contributors: NBauschmann

1.0.2 (2024-10-23)
------------------
* disabled hardcoded deadband for thrusters. Future us should probably use a parameter for this.
* Contributors: Thies Lennart Alff

1.0.1 (2024-09-24)
------------------
* added sh environment hook. dsv does not seem to work for deb install
* Contributors: Thies Lennart Alff

1.0.0 (2024-09-24)
------------------
* migrated deprecated code
* adapted plugin paths to follow ROSCON22 best practices
* migrated ignition to gz in package xml
* removed nonexistent dependency
* migration to jazzy/harmonic
* use range sensor pose for distance measurements
* avoid race condition
  by updating the world pose component in every update step instead of
  only once.
* removed overly verbose logging
* corrected fov/opening angle issue by dividing by 2.0
* removed compiler warnings
* fixed zero pose bug
  WorldPose component does not seem to be updated for static models.
  We now use a convenience utility function to get the pose that relies on
  the Pose component that seems to get updated.
* added changes by niklastkl
* make the simulation behave more nicely
* added pre-commit hooks
* revert to 917ea7e9a5d94f0c8b048ce1853b11bd373440ec to undo PRs
* cleanup odometry plugin
* revert to upstream
* created bluerof urdf as macro, modified buoyancy plugin as parameters are not loaded properly
* cleanup commented out sections
* added kinematic control gz plugin
* revert to upstream status
* commit before upstream pull request
* fixed wrong thruster mapping, manipulator compensation + added accelerations, restructured velocity controller auv
* added accel estimation and PID control for BlueROV, modified odometry plugin for ground truth
* fix gazebo issue with velocity tracking by canceling out offset
* kinematic control pipeline works, gazebo not accurate in setting link velocities
* added files for bluerov control, viz and bringup, bridge clock topic
* created bluerof urdf as macro, modified buoyancy plugin as parameters are not loaded properly
* bugfix torque direction in buoyancy plugin, published odometry velocity in base link frame
* initial commit
* Contributors: Niklas T, Thies Lennart Alff, niklastkl
