^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ustrov_sim
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.5 (2024-11-30)
------------------
* decrease probability that distance measurements get lost
* more logical comment about range sensor
* move vertical camera position to the back of bluerov
* Contributors: NBauschmann

1.0.4 (2024-11-27)
------------------
* fix ign command to gz for compatibility with Gazebo Harmonic
* Contributors: NBauschmann

1.0.3 (2024-10-30)
------------------
* change t200 linear thrust coefficient to zero to avoid accidental deadzone for very small thrusts
* add fix for wayland problems with Gazebo GUI by adding environment variable to launchdescription
* Contributors: NBauschmann

1.0.2 (2024-10-23)
------------------
* use dumb mixer for the bluerov
* Contributors: Thies Lennart Alff

1.0.1 (2024-10-12)
------------------
* added mixer to spawn launch files
* Contributors: NBauschmann

1.0.0 (2024-09-24)
------------------
* removed nonexistent dependency
* migration to jazzy/harmonic
* ditched yapf
* fixed formatting issues
* use range sensor pose for distance measurements
* migrated to ustrov_control_msgs
* tidy up launch files
* fix topic name for camera_info to be the same as image
* add use_vertical_camera param and actually use it for hippo, make camera_macro dumber
* add acoustic modem link to bluerov and hippocampus model
* fixed fixed joint lumping in camera_macro
* fixed non-adaptable base link name for vertical_cameraOC
* fixed non-adaptable base link name for range sensor
* fixed wrong apriltag frame and set camera resolution
* switched to EventsExecutor to reduce CPU load
* added camera projection matrix required for apriltag algorithms
* changed fake vision topic name
  this way it should match other naming conventions (onboard localization
  for example)
* enable fake vision by default
* fill range sensor params
* updated tag position for range tags
* partial cherry-pick edca0b5ddbdfac3b0abdd582e4b615c777bea42d
* move bluerov xacro to macro
* added changes by niklastkl
* make the simulation behave more nicely
* restructured, more configurable
* renamed PassLaunchArguments to something more descriptive
* added pre-commit hooks
* minor cleanup
* make cameras configurable during launch
* cleanup
* clenaup
* make usage of range sensor togglable
* added complete launch file for bluerov
* cleanup
* make the vehicle name configurable via cli
* changed default of fake_state_estimation to true
  true has been the de-facto default anyway
* revert to 917ea7e9a5d94f0c8b048ce1853b11bd373440ec to undo PRs
* cleanup odometry plugin
* revert to upstream
* bug fix in buoyancy torque calculatieon, rviz integration for xacro files, map_body broadcaster
* added files for bluerov control, viz and bringup, bridge clock topic
* created bluerof urdf as macro, modified buoyancy plugin as parameters are not loaded properly
* revert to upstream status
* commit before upstream pull request
* adaption bluerov viz
* deleted setpoint timeouts on velocity level, parameter and launch file adaptions for hardware experiments
* fixed wrong thruster mapping, manipulator compensation + added accelerations, restructured velocity controller auv
* added accel estimation and PID control for BlueROV, modified odometry plugin for ground truth
* added physical mapping for BlueROV thruster
* adapted changes in namespace and clock after merge
* commit before rebase, changed added buoyancy for manipulator compensation
* fix gazebo issue with velocity tracking by canceling out offset
* kinematic control pipeline works, gazebo not accurate in setting link velocities
* added bluerov velocity controller, preparation for uvms integration
* QoS settings and minor fixes
* added missing xacro file for rviz
* bug fix in buoyancy torque calculatieon, rviz integration for xacro files, map_body broadcaster
* added files for bluerov control, viz and bringup, bridge clock topic
* created bluerof urdf as macro, modified buoyancy plugin as parameters are not loaded properly
* do not overwrite header manually
* use tag poses file for apriltag grid generation
* added lens element for camera. this does NOT get populated automatically by image width and horizontal FOV
* improved educated guess for camera position
* switched to quaternions for tag poses and updated default for tag size to match real world
* start gazebo headless and unpaused
* added tag poses as config file
* reworked sim launch setup. Using sim time for the simulation
* set reasonable camera topic names and frame_ids
* initial commit
* Contributors: NBauschmann, Niklas T, Thies Lennart Alff, niklastkl
