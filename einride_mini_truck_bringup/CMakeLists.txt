cmake_minimum_required(VERSION 3.5)

project(einride_mini_truck_bringup)

find_package(ament_cmake REQUIRED)
find_package(einride_mini_truck_description REQUIRED)
find_package(einride_mini_truck_gazebo REQUIRED)

# Install project launch files
install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install project configuration files
install(
  DIRECTORY
    config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  # The linters are invoked explicitly rather than through ament_lint_auto,
  # which only ever calls them with their defaults. pep257 needs arguments: the
  # shipped "ament" convention is pydocstyle's whole rule set minus a short
  # ignore list, which pulls in D213 (blank first docstring line) and three
  # numpy-style section checks this project does not follow. Same list as
  # einride_mini_truck_hardware/test/test_pep257.py, which explains each code.
  #
  # Until this was added the launch files and the conformance test were linted
  # by nothing at all: ament_lint_auto_find_test_dependencies() had no lint
  # packages to find, so it silently ran zero checks.
  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8()

  find_package(ament_cmake_pep257 REQUIRED)
  ament_pep257(--add-ignore D213 D406 D407 D413)

  # Launches simulation.launch.py and hardware.launch.py in turn and compares
  # the topic, type and QoS sets they produce. With two separate launch files
  # this is the only thing stopping them drifting apart, so it is worth the
  # minute it costs: Gazebo alone needs ~35 s to have every topic up.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_interface_conformance
    test/test_interface_conformance.py
    TIMEOUT 400
  )
endif()

ament_package()
