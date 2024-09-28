cmake_minimum_required(VERSION 3.8)
project(walking_neural_controller_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

SET (GOROBOTS "../../../../../../..")
SET (CONTROLLER "${GOROBOTS}/controllers")
SET (DB_CONTROLLER_SIM "${GOROBOTS}/controllers/db_alpha/walking_neural_control/sim")
SET (UTILS "${GOROBOTS}/utils")
message( ${GOROBOTS} )
message( ${CONTROLLER} )
message( ${UTILS} )
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
        "${GOROBOTS}"
        "${CONTROLLER}"
        "${DB_CONTROLLER_SIM}"
        "${UTILS}"
        )

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


add_executable(db_beta_controller_sim 
    ${DB_CONTROLLER_SIM}/main_dbalpha_controller
    ${DB_CONTROLLER_SIM}/dbalpha_controller
    ${DB_CONTROLLER_SIM}/simRos2Class
    ${DB_CONTROLLER_SIM}/dbModularController
    ${DB_CONTROLLER_SIM}/alphaMotorDefinition
    ${DB_CONTROLLER_SIM}/joystick
    ${DB_CONTROLLER_SIM}/utils/db_pcpg
    ${DB_CONTROLLER_SIM}/utils/db_psn
    ${DB_CONTROLLER_SIM}/utils/dln
    ${DB_CONTROLLER_SIM}/utils/grabcpg
    ${DB_CONTROLLER_SIM}/utils/AMC
    ${UTILS}/ann-framework/ann
    ${UTILS}/ann-framework/neuron
    ${UTILS}/ann-framework/synapse
    ${UTILS}/ann-library/vrn
    ${UTILS}/ann-library/pmn
    ${UTILS}/ann-library/so2cpg
    ${UTILS}/ann-library/adaptiveso2cpgsynplas
    ${UTILS}/ann-library/extendedso2cpg
    ${UTILS}/ann-library/pcpg
    ${UTILS}/vaam-II-library/AdaptiveMotorController
    ${UTILS}/vaam-II-library/Matrix
    ${UTILS}/delayline
    ${UTILS}/interpolator2d
)
ament_target_dependencies(db_beta_controller_sim
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    tf2_geometry_msgs
)

# Install
install(TARGETS
  db_beta_controller_sim
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
