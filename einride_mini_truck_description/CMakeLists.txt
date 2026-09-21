cmake_minimum_required(VERSION 3.5)

project(einride_mini_truck_description)

find_package(ament_cmake REQUIRED)
# Before sdformat_urdf, which exports a link dependency on sdformat14::sdformat14
# and does not find it itself - without this, configuring fails on a target that
# "was not found" rather than on a missing package.
find_package(sdformat_vendor REQUIRED)
find_package(sdformat14 REQUIRED)
find_package(sdformat_urdf REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(urdfdom REQUIRED)

# model.urdf is generated from model.sdf, never written by hand and never
# checked in - the same rule the generated dock and prop models follow. The SDF
# stays the one description; the URDF exists because Foxglove's 3D panel reads
# only URDF, and tools/sdf_to_urdf.cpp explains the rest.
add_executable(sdf_to_urdf tools/sdf_to_urdf.cpp)
target_link_libraries(sdf_to_urdf
  sdformat_urdf::sdformat_urdf
  urdfdom::urdfdom_model
  ${TINYXML2_LIBRARIES}
)
target_include_directories(sdf_to_urdf PRIVATE ${TINYXML2_INCLUDE_DIRS})

set(model_sdf "${CMAKE_CURRENT_SOURCE_DIR}/models/einride_mini_truck/model.sdf")
set(model_urdf "${CMAKE_CURRENT_BINARY_DIR}/model.urdf")

add_custom_command(
  OUTPUT "${model_urdf}"
  COMMAND sdf_to_urdf "${model_sdf}" "${model_urdf}"
  DEPENDS sdf_to_urdf "${model_sdf}"
  COMMENT "Converting model.sdf to model.urdf"
  VERBATIM
)
add_custom_target(model_urdf ALL DEPENDS "${model_urdf}")

install(
  DIRECTORY
    models/
  DESTINATION share/${PROJECT_NAME}/models
)

# Installed beside model.sdf rather than in a directory of its own, because
# consumers find it the same way they find the SDF and the meshes it points at.
install(
  FILES "${model_urdf}"
  DESTINATION share/${PROJECT_NAME}/models/einride_mini_truck
)

# The OAK-D Lite's factory EEPROM calibration, dumped off the device with
# camera.i_calibration_dump. It lives here rather than in the bringup or gazebo
# package because it describes the robot's geometry, the same as model.sdf does -
# it is the part of that geometry only the device can state. On hardware the
# driver reads it straight off the camera; simulation reads this copy, so the
# filename carries the mxid to make it obvious that it belongs to ONE device.
# Swap the camera and this file has to be re-dumped.
install(
  DIRECTORY
    calibration/
  DESTINATION share/${PROJECT_NAME}/calibration
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_urdf_export test/test_urdf_export.py
    ENV MODEL_SDF=${model_sdf} MODEL_URDF=${model_urdf}
  )
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
