cmake_minimum_required(VERSION 3.5)

# Following directive defines the project name.
project(einride_mini_truck_gazebo)


# Following directives find required packages and load their configuration.
# The 'set' directive defines a variable (e.g. 'GZ_PLUGIN_VER').
# Such variables can be used lateron in the CMakeLists.txt file.
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(einride_mini_truck_description REQUIRED)

# Gazebo is consumed through the ROS 2 Jazzy `gz_*_vendor` packages rather than a
# standalone Gazebo install. Each vendor package puts its own prefix on
# CMAKE_PREFIX_PATH and exposes an unversioned `gz-<lib>` config, so the version
# numbers stay out of this file and follow whatever the ROS distro ships.
# NOTE: only gz_cmake_vendor is pulled in here, not an unversioned `gz-cmake`.
# gz_cmake_vendor's generic unversioned shim tries to alias library targets that
# the build-system-only gz-cmake package does not have, so find_package(gz-cmake)
# fails. Nothing below needs gz-cmake's macros anyway.
find_package(gz_cmake_vendor REQUIRED)

find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin_VERSION_MAJOR})

find_package(gz_common_vendor REQUIRED)
find_package(gz-common REQUIRED COMPONENTS profiler)
set(GZ_COMMON_VER ${gz-common_VERSION_MAJOR})

find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
set(GZ_SIM_VER ${gz-sim_VERSION_MAJOR})

message(STATUS "Compiling against ROS 2 $ENV{ROS_DISTRO} vendored Gazebo (gz-sim ${gz-sim_VERSION})")


# Following 'add_library' directive defines a library target named 'BasicSystem'.
# The 'SHARED' keyword indicates that a shared library should be compiled, and
# is followed by the list of source files for the target.
add_library(BasicSystem
  SHARED
  src/BasicSystem.cc
)

# Following 'target_include_directories' directive specifies 'include' as the
# include directory to use when compiling the 'BasicSystem' target.
# The 'PRIVATE' keyword specifies that this directive will populate the
# INCLUDE_DIRECTORIES property for the 'BasicSystem' target.
target_include_directories(
  BasicSystem PRIVATE include
)

# Following 'target_link_libraries' directive specifies to use the 
# gz-sim library when linking the 'BasicSystem' target.
# The 'PRIVATE' keyword stipulates that the gz-sim library will not
# automatically be included if the 'BasicSystem' target were to
# be linked to anoter target. 
# ${GZ_SIM_VER} is substituted by the value that is was set to above.
target_link_libraries(BasicSystem PRIVATE
  gz-sim::gz-sim)


# Following directives similarly specify another target: 'FullSystem'.
add_library(FullSystem
  SHARED
  src/FullSystem.cc
)

target_include_directories(
  FullSystem PRIVATE include
)

target_link_libraries(FullSystem PRIVATE
  gz-sim::gz-sim)



# Following 'install' directive ensures that the compiled libraries
# of the 'BasicSystem' and 'FullSystem' targets will be copied
# to the subfolder 'lib/einride_mini_truck_gazebo' of the install directory.
install(
  TARGETS BasicSystem FullSystem
  DESTINATION lib/${PROJECT_NAME}
)


# Following 'install' directive ensures that the 'worlds' subfolder
# will be copied to the 'share/einride_mini_truck_gazebo/worlds'
# subfolder of the installation directory.
install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)


# The eight warehouse docks are generated from one template rather than
# checked in eight times over. dock_template/model.sdf.in holds every
# dimension once - see DOCK_PLAN.md, which is the spec it is built to - and the
# docks differ only in their two identity textures and their name. Copying the
# file eight times would mean eight places to miss when a number changes, and
# the numbers in it are load-bearing for both the LiDAR and the tag.
#
# Generated models land in the worlds/ install tree on purpose: the environment
# hook already exports share/${PROJECT_NAME}/worlds on GZ_SIM_RESOURCE_PATH, so
# model://warehouse_dock_a resolves with no change to the hooks.
#
# Each entry is "<letter> <tag id>". The letter is what a human reads off the
# dock's own letter decal and the id is what apriltag_ros reports; keeping
# them paired here is what stops the two markings drifting apart.
set(DOCK_ASSIGNMENTS "A 0" "B 1" "C 2" "D 3" "E 4" "F 5" "G 6" "H 7")

foreach(assignment IN LISTS DOCK_ASSIGNMENTS)
  separate_arguments(parts UNIX_COMMAND "${assignment}")
  list(GET parts 0 DOCK_LETTER)
  list(GET parts 1 DOCK_TAG_ID)
  string(TOLOWER "${DOCK_LETTER}" dock_letter_lower)

  set(DOCK_MODEL "warehouse_dock_${dock_letter_lower}")
  set(DOCK_LETTER_TEXTURE "letter_${dock_letter_lower}.png")
  # apriltag-imgs names its files with a 5-digit zero-padded id
  string(REGEX REPLACE "^(.)$" "0000\\1" padded_id "${DOCK_TAG_ID}")
  set(DOCK_TAG_TEXTURE "tag36h11_${padded_id}.png")

  set(dock_out "${CMAKE_CURRENT_BINARY_DIR}/docks/${DOCK_MODEL}")
  configure_file(dock_template/model.sdf.in    "${dock_out}/model.sdf"    @ONLY)
  configure_file(dock_template/model.config.in "${dock_out}/model.config" @ONLY)

  # Only the textures this dock actually references, so each model stays
  # self-contained the way Gazebo expects.
  file(COPY
    "dock_template/textures/${DOCK_TAG_TEXTURE}"
    "dock_template/textures/${DOCK_LETTER_TEXTURE}"
    DESTINATION "${dock_out}/materials/textures"
  )

  install(
    DIRECTORY "${dock_out}"
    DESTINATION share/${PROJECT_NAME}/worlds
  )
  message(STATUS "Dock ${DOCK_LETTER}: ${DOCK_MODEL}, tag36h11 id ${DOCK_TAG_ID}")
endforeach()


# The arena props are third-party Gazebo Fuel assets, vendored so that a fresh
# clone plus `colcon build` plus launch works with no ~/.gz/fuel cache and no
# network. Upstream meshes and textures are byte-for-byte (props/MANIFEST.sha256);
# the model.sdf and model.config are ours and generated, for the reasons spelled
# out in prop_template/model.sdf.in.
#
# Same shape as the dock loop above, and for the same reason: the scale and offsets
# are load-bearing for the LiDAR, so they live in exactly one place
# (props/props.cmake) rather than being copy-pasted per prop.
#
# Generated models land in the worlds/ install tree, like the docks, because the
# environment hook already exports share/${PROJECT_NAME}/worlds on
# GZ_SIM_RESOURCE_PATH - so model://construction_barrel resolves with no change to
# hooks/.
include(props/props.cmake)

foreach(assignment IN LISTS PROP_ASSIGNMENTS)
  separate_arguments(parts UNIX_COMMAND "${assignment}")
  list(GET parts 0 PROP_DIR)
  list(GET parts 1 PROP_SCALE)
  list(GET parts 2 PROP_Z_OFFSET)
  list(GET parts 3 PROP_Y_OFFSET)

  set(prop_src "${CMAKE_CURRENT_SOURCE_DIR}/props/${PROP_DIR}")

  # Deliberately a hard failure, not a download. Fetching at build time would make
  # the build non-reproducible and require network in CI; the whole point of
  # vendoring is that the bytes are already in git.
  if(NOT EXISTS "${prop_src}/mesh.txt")
    message(FATAL_ERROR
      "props/${PROP_DIR} is not vendored. Run tools/fetch_props.sh once, with a "
      "network, and commit the result. This package does not download at build time.")
  endif()
  file(READ "${prop_src}/mesh.txt" PROP_MESH)
  string(STRIP "${PROP_MESH}" PROP_MESH)
  if(NOT EXISTS "${prop_src}/${PROP_MESH}")
    message(FATAL_ERROR "props/${PROP_DIR}/${PROP_MESH} is missing")
  endif()

  set(prop_out "${CMAKE_CURRENT_BINARY_DIR}/props/${PROP_DIR}")
  configure_file(prop_template/model.sdf.in    "${prop_out}/model.sdf"    @ONLY)
  configure_file(prop_template/model.config.in "${prop_out}/model.config" @ONLY)

  install(FILES "${prop_out}/model.sdf" "${prop_out}/model.config"
          DESTINATION share/${PROJECT_NAME}/worlds/${PROP_DIR})
  # Installed straight from the source tree rather than copied via the build dir:
  # the dock loop's file(COPY) runs at CONFIGURE time, so an edited asset does not
  # reach the install tree until cmake re-runs. Not a wart worth propagating.
  install(DIRECTORY "props/${PROP_DIR}/meshes"
          DESTINATION share/${PROJECT_NAME}/worlds/${PROP_DIR})
  if(EXISTS "${prop_src}/materials")
    install(DIRECTORY "props/${PROP_DIR}/materials"
            DESTINATION share/${PROJECT_NAME}/worlds/${PROP_DIR})
  endif()
  # Licence and provenance travel with the assets, not only with the repo.
  install(FILES props/README.md props/LICENSE.CC0 props/MANIFEST.sha256
          DESTINATION share/${PROJECT_NAME}/worlds/${PROP_DIR})

  message(STATUS "Prop ${PROP_DIR}: scale ${PROP_SCALE}, mesh ${PROP_MESH}")
endforeach()


# The simulation-side stand-in for the LD19 driver: gz's gpu_lidar gets the scan
# geometry right but not the message semantics. Python rather than another gz
# system plugin because it acts on the ROS side of the bridge, after gz is done.
#
# oak_calibration_tf is the same idea for the camera: model.sdf deliberately does
# not declare the frames inside the OAK-D Lite, because only the device knows
# where its own sensors are. On hardware the driver publishes them from the
# camera's EEPROM; this replays the same arithmetic over a dump of that EEPROM so
# simulation's TF tree is the same tree.
ament_python_install_package(${PROJECT_NAME})
install(
  PROGRAMS
    scripts/ld19_scan_model
    scripts/oak_calibration_tf
  DESTINATION lib/${PROJECT_NAME}
)


# Following directives are used when testing.
if(BUILD_TESTING)
  # Explicitly, not via ament_lint_auto: with flake8 and pep257 now declared as
  # test dependencies, ament_lint_auto_find_test_dependencies() registers them
  # under its own test names and collides with the calls below. It was also
  # running zero checks before they were declared, so nothing is lost by
  # dropping it - the same trap einride_mini_truck_bringup/CMakeLists.txt
  # describes.
  #
  # Same linter set and the same pep257 ignore list as the other two packages
  # that ship Python - see einride_mini_truck_bringup/CMakeLists.txt for why the
  # arguments are spelled out rather than left to ament_lint_auto.
  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8()

  find_package(ament_cmake_pep257 REQUIRED)
  ament_pep257(--add-ignore D213 D406 D407 D413)

  # Asserts the transform against numbers measured on the real device, so it
  # needs no Gazebo and no lidar on the USB bus.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_ld19_scan_model test/test_ld19_scan_model.py)

  # Asserts the simulated camera frames against a /tf_static snapshot recorded
  # from the real OAK-D Lite, so it needs neither Gazebo nor a camera on the USB
  # bus. This is the only sim-versus-hardware check any camera frame has.
  ament_add_pytest_test(test_oak_calibration_tf test/test_oak_calibration_tf.py)

  # model.sdf has to state where the mono sensors sit, which duplicates numbers
  # the calibration dump already holds. This recomputes them rather than trusting
  # the copy, so the two cannot drift apart silently.
  ament_add_pytest_test(test_mono_pair_model test/test_mono_pair_model.py)
endif()


# Following hooks are used to ensure that the correct environment variables
# will be set by exectuting 'source install/setup.bash' after compilation.
# When using this template for your project, change the filenames of the
# files in the 'hooks' folder, to correspond to your project name.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")


# Following directive configures ament based on the previous directives, and should
# typically be the last directive in the CMakeLists.txt file.
ament_package()
