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

# The systemd service that runs hardware.launch.py on the robot, and the scripts
# that install it there over ssh. Shipped into the install space so the service
# can be reinstalled or removed from a machine that has the workspace but not the
# source repo - the scripts find their templates relative to themselves, so an
# installed copy resolves them the same way a source one does.
#
# Only the parts that need no sources work from there, which is most of what
# matters on a robot: provision.sh --uninstall, and --no-build against a prepared
# artifacts directory. A cross-build needs the whole workspace src/ as its
# context, and deploy.sh needs a workspace to build in, so both of those want the
# source checkout.
#
# PROGRAMS, not FILES: both are executable, and install(FILES) drops the mode.
install(
  PROGRAMS
    systemd/run-hardware.sh
    systemd/run-foxglove.sh
  DESTINATION share/${PROJECT_NAME}/systemd
)
install(
  FILES
    systemd/einride-mini-truck.service.in
    systemd/einride-mini-truck-foxglove.service.in
    systemd/einride-mini-truck.env.in
  DESTINATION share/${PROJECT_NAME}/systemd
)
install(
  PROGRAMS
    tools/provision.sh
    tools/deploy.sh
  DESTINATION share/${PROJECT_NAME}/tools
)
# Sourced by the two scripts above, not run, so FILES rather than PROGRAMS.
install(
  FILES
    tools/_common.sh
  DESTINATION share/${PROJECT_NAME}/tools
)
install(
  DIRECTORY
    udev/
  DESTINATION share/${PROJECT_NAME}/udev
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

  # Asserts /scan against a real LD19: bin count, sweep direction, range limits,
  # NaN-not-inf, and above all that any message arrives at all. Skips itself when
  # no lidar is attached, so it is safe to run anywhere; on the robot it is the
  # only thing that catches the driver going active and publishing nothing.
  ament_add_pytest_test(test_lidar_scan_contract
    test/test_lidar_scan_contract.py
    TIMEOUT 200
  )
endif()

ament_package()
