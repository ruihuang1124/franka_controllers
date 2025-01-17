cmake_minimum_required(VERSION 3.0.2)
project(franka_controllers)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  controller_interface
  dynamic_reconfigure
  eigen_conversions
  franka_hw
  franka_gripper
  geometry_msgs
  hardware_interface
  tf
  tf_conversions
  message_generation
  pluginlib
  realtime_tools
  trajectory_msgs
  nav_msgs
)

# add_subdirectory(frankx)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.7.0 REQUIRED)

add_message_files(FILES
  JointTorqueComparison.msg
)

add_service_files(
  FILES 
  movetoHome.srv
  switchController.srv
  createTrajectory.srv
        ExecuteDualPose.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

generate_dynamic_reconfigure_options(
  cfg/compliance_param.cfg
  cfg/desired_mass_param.cfg
  cfg/dual_arm_compliance_param.cfg
  cfg/teleop_gripper_param.cfg
  cfg/teleop_param.cfg
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS 
    controller_interface
    dynamic_reconfigure
    eigen_conversions
    franka_hw
    franka_gripper
    geometry_msgs
    hardware_interface
    tf
    tf_conversions
    message_runtime
    pluginlib
    realtime_tools
    roscpp
  DEPENDS Franka
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
        include
        /home/${USER}/curi/curi_ros/curi_frankx_interface/frankx/include
        /home/${USER}/tools/frankx/affx/include
        /home/${USER}/tools/frankx/ruckig/include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)
## Your package locations should be listed before other locations
#include_directories(
#include
#  ${catkin_INCLUDE_DIRS}
#)

## Declare a C++ library
# add_library(franka_controllers_src
#   include/${PROJECT_NAME}/whole_body_controller.h
#   include/${PROJECT_NAME}/pseudo_inversion.h
#   src/whole_body_controller.cpp
# )

add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/whole_body_controller.h
  include/${PROJECT_NAME}/cartesian_impedance_controller.h
  include/${PROJECT_NAME}/joint_impedance_controller.h
  include/${PROJECT_NAME}/pseudo_inversion.h
  include/${PROJECT_NAME}/dual_arm_cartesian_impedance_controller.h
  src/whole_body_controller.cpp
  src/joint_impedance_controller.cpp
  src/cartesian_impedance_controller.cpp
  src/dual_arm_cartesian_impedance_controller.cpp
        include/franka_controllers/dual_arm_joint_position_controller.h src/dual_arm_joint_position_controller.cpp src/libfranka_test.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
#add_executable(use_whole_body_controller src/use_whole_body_controller.cpp)
add_executable(pub_test src/pub_test.cpp)
add_executable(libfranka_test src/libfranka_test.cpp)
add_executable(pub_target src/pub_target.cpp)
add_executable(pub_trajectory_test src/pub_trajectory_test.cpp)
add_executable(execute_multi_client_server src/execute_multi_client_server.cpp)
add_executable(execute_dual_gripper_server src/execute_dual_gripper_server.cpp)
add_executable(execute_dual_arm_pose src/execute_dual_arm_pose_server.cpp)
add_executable(execute_dual_arm_joint_server src/execute_dual_arm_joint_server.cpp)
add_executable(pub_joint_ptp_trajectory src/pub_joint_ptp_trajectory.cpp)
add_executable(execute_cartesian_trajectory_action_client src/execute_cartesian_trajectory_action_client.cpp)
add_executable(execute_dual_arm_pose_new src/execute_dual_arm_pose_server_new.cpp)
add_executable(execute_dual_arm_pose_torso_joint_position src/execute_dual_arm_pose_torso_joint_position_server.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}_gencfg
)

#add_dependencies(use_whole_body_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(pub_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(libfranka_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(pub_target ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(pub_trajectory_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(execute_multi_client_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(execute_dual_gripper_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(execute_dual_arm_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(execute_dual_arm_joint_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(pub_joint_ptp_trajectory ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(execute_cartesian_trajectory_action_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(execute_dual_arm_pose_new ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(execute_dual_arm_pose_torso_joint_position ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
        frankx
        Eigen3::Eigen
)

# target_link_libraries(mytarget matlogger2::matlogger2)

#target_link_libraries(use_whole_body_controller
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#)
target_link_libraries(pub_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_link_libraries(libfranka_test
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

target_link_libraries(pub_target
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

target_link_libraries(pub_trajectory_test
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

target_link_libraries(execute_multi_client_server
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

target_link_libraries(execute_dual_gripper_server
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

target_link_libraries(execute_dual_arm_pose
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

target_link_libraries(execute_dual_arm_joint_server
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

target_link_libraries(pub_joint_ptp_trajectory
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

target_link_libraries(execute_cartesian_trajectory_action_client
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

target_link_libraries(execute_dual_arm_pose_new
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

target_link_libraries(execute_dual_arm_pose_torso_joint_position
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(${PROJECT_NAME} PUBLIC
  include
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES ${PROJECT_NAME}_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


catkin_install_python(
  PROGRAMS scripts/interactive_marker.py scripts/move_to_start.py scripts/dual_arm_interactive_marker.py scripts/curi_dual_arm_keyboard_control.py scripts/franka_move_action_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_franka_controllers.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

# Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
)
if(CLANG_TOOLS)
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  file(GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  )
  add_format_target(franka_controllers_src FILES ${SOURCES} ${HEADERS})
  add_tidy_target(franka_controllers_src
    FILES ${SOURCES}
    DEPENDS franka_controllers_src
  )
endif()

include(${CMAKE_CURRENT_LIST_DIR}/../cmake/PepTools.cmake OPTIONAL
  RESULT_VARIABLE PEP_TOOLS
)
if(PEP_TOOLS)
  file(GLOB_RECURSE PYSOURCES ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py)
  add_pyformat_target(franka_control FILES ${PYSOURCES})
endif()
