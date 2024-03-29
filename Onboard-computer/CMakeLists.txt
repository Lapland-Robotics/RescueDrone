cmake_minimum_required(VERSION 3.0.2)
project(sar_drone)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
             roscpp
             rospy
             dji_sdk
             std_msgs
             message_generation
             roslaunch
)

find_package(DJIOSDK REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

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
add_message_files(
  FILES
  coordinates.msg
  rel_coordinates.msg
  directions.msg
  status.msg
  send_mobile.msg
)

## Generate services in the 'svg' folder
add_service_files(
  FILES
  routeplanner.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs 
)

catkin_install_python(PROGRAMS scripts/human_detection.py scripts/routeplanner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


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
 LIBRARIES sar_drone
 CATKIN_DEPENDS message_runtime rospy
)

#######################
## Check Launch file ##
#######################

roslaunch_add_file_check(launch)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
set(pkg_parrent_class ParrentDroneClass)
set(pkg_position_class PositionClass)

add_library(${pkg_parrent_class}
  src/class_files/ParrentDroneClass.cpp
)

target_link_libraries(${pkg_parrent_class} ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})

add_library(${pkg_position_class}
  src/class_files/Position.cpp
)

target_link_libraries(${pkg_position_class} ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES} ${pkg_parrent_class})

add_library(ControlleDrone
  src/class_files/ControlleDrone.cpp
)

target_link_libraries(ControlleDrone ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES} ${pkg_parrent_class} ${pkg_parrent_class} ${pkg_position_class})

add_library(SendReceveDataMSDK
  src/class_files/SendReceveDataMSDK.cpp
)

target_link_libraries(SendReceveDataMSDK ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES} ${pkg_parrent_class})

add_library(mappAlgClass
  src/class_files/mapAlg.cpp
)

target_link_libraries(mappAlgClass ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES} ${pkg_parrent_class} ${pkg_position_class})


set(pkg_libs
    ControlleDrone
    SendReceveDataMSDK
    mappAlgClass
)

add_executable(communication_with_drone src/DroneCon.cpp)

target_link_libraries(communication_with_drone ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES} ${pkg_libs})

add_dependencies(communication_with_drone dji_sdk_generate_messages_cpp)


add_executable(communication_with_MSDK src/MsdkCon.cpp)

target_link_libraries(communication_with_MSDK ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES} ${pkg_libs})

add_dependencies(communication_with_MSDK dji_sdk_generate_messages_cpp)

add_executable(mapping_alg src/mapping_alg.cpp)

target_link_libraries(mapping_alg ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES} ${pkg_libs})

add_dependencies(mapping_alg dji_sdk_generate_messages_cpp)

add_executable(readable_log src/readable_log.cpp)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sar_drone.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
