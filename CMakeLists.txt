cmake_minimum_required(VERSION 3.0.2)
project(motion_primitive_planner)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_BUILD_TYPE Debug)

add_definitions(-DHAVE_CSTDDEF -D_USE_MATH_DEFINES)

#ECBS
#BOOST14
find_package(Boost 1.58 REQUIRED COMPONENTS program_options)
#EIGEN
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
#PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

#OCTOMAP
find_package(octomap_ros REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})
add_definitions(-DOCTOMAP_NODEBUGOUT)

#EDT
find_package(dynamicEDT3D REQUIRED)
include_directories(${DYNAMICEDT3D_INCLUDE_DIRS})
link_libraries(${DYNAMICEDT3D_LIBRARIES})

#Decomp
find_package(decomp_util REQUIRED)
include_directories(${DECOMP_UTIL_INCLUDE_DIRS})
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  base_local_planner
  costmap_2d
  gazebo_ros
  geometry_msgs
  nav_core
  nav_msgs
  std_msgs
  pluginlib
  tf2
  tf2_ros
  visualization_msgs
  interactive_markers
  octomap_ros
  octomap_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(ompl REQUIRED)


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
#   std_msgs  # Or other packages containing msgs
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
  LIBRARIES motion_primitive_planner
  CATKIN_DEPENDS gazebo_ros roscpp roslib message_runtime
  DEPENDS OMPL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  "/home/weijian/123/include/iris"
  "/home/weijian/123/include/iris/iris"
  "/usr/include/python3.8/"
)

find_library(IRIS_LIB iris HINTS  "/home/weijian/123/lib/iris")
find_library(IRIS_GEOMETRY_LIB iris_geometry HINTS "/home/weijian/123/lib/iris")
find_library(IRIS_CVXGEN_LDP_CPP_LIB NAMES iris_cvxgen_ldp_cpp HINTS "/home/weijian/123/lib/iris")
find_library(IRIS_CVXGEN_LDP_LIB iris_cvxgen_ldp HINTS "/home/weijian/123/lib/iris")
find_library(MOSEK_LIB NAMES mosek64 HINTS "/home/weijian/mosek/7/tools/platform/linux64x86/bin")

# Declare a C++ library
add_library(motion_primitive_planner
  src/formation_planning/liom_local_planner_ros.cpp
  src/formation_planning/lightweight_nlp_problem.cpp
  src/math/aabox2d.cpp
  src/math/box2d.cpp
  src/math/line_segment2d.cpp
  src/math/math_utils.cpp
  src/math/polygon2d.cpp
  src/math/vec2d.cpp
  src/visualization/color.cpp
  src/visualization/plot.cpp
  src/formation_planning/liom_local_planner.cpp
  src/formation_planning/coarse_path_planner.cpp
  src/formation_planning/environment.cpp
  src/yaml.cpp
)
target_include_directories(motion_primitive_planner PUBLIC "/home/weijian/123/lib/iris")
target_include_directories(motion_primitive_planner PUBLIC "/home/weijian/mosek/7/tools/platform/linux64x86/bin")
target_link_libraries(motion_primitive_planner ${catkin_LIBRARIES} ${OMPL_LIBRARIES} ipopt adolc ${IRIS_LIB} ${IRIS_GEOMETRY_LIB} ${MOSEK_LIB} ${IRIS_CVXGEN_LDP_CPP_LIB})

add_executable(liom_test_node src/formation_planning/liom_test_node.cpp)
target_link_libraries(liom_test_node ${catkin_LIBRARIES} motion_primitive_planner)
target_link_libraries(liom_test_node /usr/local/lib/libyaml-cpp.so.0.8)

# add_executable(coupled_traj_planner
#   src/formation_generation/coupled_traj_planner.cpp
# )
# target_link_libraries(coupled_traj_planner
#   ${catkin_LIBRARIES}
#   ${OCTOMAP_INCLUDE_DIRS}
#   ${SIPP_LINK_LIBS}
#   ${PYTHON_LIBRARIES}
#   m
#   ipopt
# )

add_executable(compute_optimal_traj src/sarah_simulation/compute_optimal_traj.cpp)
target_link_libraries(compute_optimal_traj ${catkin_LIBRARIES} motion_primitive_planner)
target_link_libraries(compute_optimal_traj /usr/local/lib/libyaml-cpp.so.0.8)
target_link_libraries(compute_optimal_traj
  ${OCTOMAP_INCLUDE_DIRS}
  ${SIPP_LINK_LIBS}
  ${PYTHON_LIBRARIES}
  m
  ipopt
)

add_executable(traj_tracking_sim src/sarah_simulation/traj_tracking.cpp)
target_link_libraries(traj_tracking_sim ${catkin_LIBRARIES} motion_primitive_planner)
target_link_libraries(traj_tracking_sim /usr/local/lib/libyaml-cpp.so.0.8)
target_link_libraries(traj_tracking_sim
  ${OCTOMAP_INCLUDE_DIRS}
  ${SIPP_LINK_LIBS}
  ${PYTHON_LIBRARIES}
  m
  ipopt
)

add_executable(visualize src/sarah_simulation/visualize.cpp)
target_link_libraries(visualize ${catkin_LIBRARIES} motion_primitive_planner)
target_link_libraries(visualize /usr/local/lib/libyaml-cpp.so.0.8)
target_link_libraries(visualize
  ${OCTOMAP_INCLUDE_DIRS}
  ${SIPP_LINK_LIBS}
  ${PYTHON_LIBRARIES}
  m
  ipopt
)

add_executable(visualize_real src/sarah_simulation/visualize_real.cpp)
target_link_libraries(visualize_real ${catkin_LIBRARIES} motion_primitive_planner)
target_link_libraries(visualize_real /usr/local/lib/libyaml-cpp.so.0.8)
target_link_libraries(visualize_real
  ${OCTOMAP_INCLUDE_DIRS}
  ${SIPP_LINK_LIBS}
  ${PYTHON_LIBRARIES}
  m
  ipopt
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
# catkin_add_gtest(${PROJECT_NAME}-test test/motion_primitive_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)