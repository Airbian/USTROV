set(lib_name kinematic_control)
add_library(${lib_name} SHARED
  kinematic_control.cpp
)
target_link_libraries(${lib_name}
  gz-plugin::core
  gz-sim::core
  gz-utils::core
)
ament_target_dependencies(${lib_name}
  gz-plugin
  gz-sim
  gz-utils
)

install(TARGETS ${lib_name}
  DESTINATION lib/${PROJECT_NAME}
)
