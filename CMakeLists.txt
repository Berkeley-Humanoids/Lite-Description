cmake_minimum_required(VERSION 3.16)
project(lite_description)

find_package(ament_cmake REQUIRED)

# Lite variants live under robots/<robot>/ (franka_description layout: package at
# the repo root, per-robot assets in a robots/ subdir). Each ships generated,
# installable artifacts (xacro / urdf / mjcf / meshes). The cad/ subdir holds
# generation INPUTS (Onshape config, joint_properties.json, physics.json,
# ros2_control.json, scad) and is intentionally NOT installed.
# Keep this list in sync with the converter's robot list.
set(ROBOTS
  lite
  lite_dummy
)

# Install only the installable subdirs that exist for each robot (defensive:
# a model-only robot may have no ros2_control xacro, etc.).
foreach(robot ${ROBOTS})
  foreach(sub xacro urdf mjcf meshes)
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/robots/${robot}/${sub})
      install(
        DIRECTORY robots/${robot}/${sub}
        DESTINATION share/${PROJECT_NAME}/robots/${robot}
      )
    endif()
  endforeach()
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Per-package LICENSE rather than per-file headers.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
