^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ustrov_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.2 (2024-10-12)
------------------
* fixed error in heading/roll to quaternion conversion
* updated formmating
* Contributors: Thies Lennart Alff

1.0.1 (2024-09-11)
------------------
* lemniscate demo work with qualisys or onboard camera
* moved rosdep to ros-infrastructure
* added readme
* another test commit
* Revert "another test commit"
  This reverts commit ca0355cd995f5cc574d46c50648c34d75b42d716.
* another test commit
* Revert "testcommit"
  This reverts commit 9f26f2363b9e103f2f21faf4f7df714c488bfd9e.
* testcommit
* ros-distro specific rosdep file
* added dvl to rosdep
* added packages to rosdep
* Contributors: Thies Lennart Alff

1.0.0 (2024-07-15)
------------------
* added all our prebuilt msg packages
* updated rosdeps to include gantry_msgs
* added rosdep.yaml for hippocampus custom rosdeps
* added formatting settings
* improved launch file setup
* added missing yaml-cpp dependency
* minor build-command update
* fixed wrong package name for config file
* added missing executable
* added px4 bridge to launch files
* demo launchfiles updated
* launch file cleanup
* added convenience function for config file dir
* removed unused dependency on hippo_msgs
* ditched yapf
* install even if not in path
* added convenience scripts for building/cleaning the workspace
* migrated to ustrov_control_msgs
* added tag markers publisher
* added launchfile for event camera bagfile
* set camera position based on CAD files
* fixed typo in launch file
* launch file cleanup
* added convenience launch file
* add robot mesh publisher
* added ground truth tag publisher
* moved calibration launchfile to visual_localization
* added image_decoding options for mjpeg_cam setups
* updated calibration launch file
* removed old/unused launch files
* added pose encoding/decoding
* fixed wrong front camera transformation
* added missing vision prefix
* fixed missing transform
  transformation was not added to vector. fixed it now.
* macro in param utils for integer range
* added convenience macro to add float param with range
* changed default vertical camera position
  the vertical camera is now located at the rear side
  x=-0.15 is only and educated guess
* added tfs for front camera
* corrected vertical camera offset to match simulation
* added tf publisher for bluerov
* added conversion function eigen_ref->vector3
* added assign param function for strings
* Niklas' changes squashed
* renamed PassLaunchArguments to something more descriptive
* added pre-commit hooks
* start camera bridge only if camera is enabled
* fixed formatting errors after latest merge
* snapshot for paper
* removed unused source file
* best effort subscriptions for tf_publisher_vehicle for better compatibility.
  Can subscribe to all publishers' QoS settings this way
* added function to rertrieve the angle of a rotation expressed as quaternion.
  Can be useful in case just an angle is required. Otherwise a angle-axis represantion can be constructed from a quaternion directly by using eigen
* added normalization just to be super safe. for normalized headings actually not required
* added licenses and applied formatting to all source files
* inlined template specializations and added conversion for actuator-setpoint message
* added param convenience macros
* assume unit vector for heading
* simplified control launch files
* path planning package added
* adapted mixer matrix and thruster mapping to physical model
* add conversion utility functions between eigen and ros for eigen reference objects
* adapted to upstream
* Merge remote-tracking branch 'upstream/main' into merge_from_upstream
* refactored vision frame_id code
* educated guess for default camera position and changed tf names
* Merge remote-tracking branch 'upstream/main'
* added convenience code for launch files
* added tf between camera and vision base link
* added convenience functions for camera names
* publish tf for the vision pose
* added tf publishing code based on odometry
* added nav_msgs dependency
* simplified cmakelists
* removed accidentally added param override
* tf-publisher setup completed
* Added convertion util functions for Eigen Reference type
* added control target msg type
* fixed wrong tf dependency
* added logging for boolean params
* added compiler optimizations
* log message for int parameter assignment
* added log text for paramter assignment
* removed redundant code
* general update
* simplified node/component setup
* corrected place for camera frame_id -> tf id
* added frame parameter for apriltag detection
* minor changes to api
* removed px4 dependency from ustrov_common
* made tf_publisher composable
* moved definitions to cpp
* addd tf node
* updated launchfiles
* fixed typo
* changed topic remappings for camera
* launch arg for distortion coeffs added
* added camera launch file
* created launchfile for apriltag detection
* added calibration launch file
* fixed build error due to changed library
* simple mesh publisher
* simplified cmakelists
* added fake vision
* convenience library for ros <-> eigen conversion
* simplifications
* quaternion utility added
* added cmake definition for clang-tidy
* added compiler flags
* initial commit
* Contributors: NBauschmann, Niklas T, Thies Lennart Alff, niklastkl
