cmake_minimum_required(VERSION 2.8.3)
project(behavior_coordinator)

### Use version 2011 of C++ (c++11). By default ROS uses c++98
add_definitions(-std=c++11)
add_definitions(-g)

# Directories definition
set(BEHAVIOR_COORDINATOR_SOURCE_DIR
  src/source
)

set(BEHAVIOR_COORDINATOR_INCLUDE_DIR
  src/include
)

set(BEHAVIOR_COORDINATOR_TEST_DIR
  src/test
)

# Files declaration
set(BEHAVIOR_COORDINATOR_SOURCE_FILES
  ${BEHAVIOR_COORDINATOR_SOURCE_DIR}/behavior_coordinator.cpp
  ${BEHAVIOR_COORDINATOR_SOURCE_DIR}/behavior_coordinator_main.cpp
  ${BEHAVIOR_COORDINATOR_SOURCE_DIR}/behavior.cpp
  ${BEHAVIOR_COORDINATOR_SOURCE_DIR}/task.cpp
  ${BEHAVIOR_COORDINATOR_SOURCE_DIR}/constraint.cpp
  ${BEHAVIOR_COORDINATOR_SOURCE_DIR}/catalog.cpp
  ${BEHAVIOR_COORDINATOR_SOURCE_DIR}/prettify.cpp
)

# Files declaration
set(TEST_SOURCE_FILES
  ${BEHAVIOR_COORDINATOR_TEST_DIR}/test.py
)

# Files declaration
set(YAML_CREATOR_SOURCE_FILES
  ${BEHAVIOR_COORDINATOR_SOURCE_DIR}/yamlCreator.cpp
)

set(BEHAVIOR_COORDINATOR_HEADER_FILES
  ${BEHAVIOR_COORDINATOR_INCLUDE_DIR}/behavior_coordinator.h
  ${BEHAVIOR_COORDINATOR_INCLUDE_DIR}/behavior.h
  ${BEHAVIOR_COORDINATOR_INCLUDE_DIR}/task.h
  ${BEHAVIOR_COORDINATOR_INCLUDE_DIR}/constraint.h
  ${BEHAVIOR_COORDINATOR_INCLUDE_DIR}/catalog.h
  ${BEHAVIOR_COORDINATOR_INCLUDE_DIR}/prettify.h
)

### Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  roslib
  std_msgs
  behavior_coordinator_msgs
  behavior_execution_manager_msgs
)

###################################
## catkin specific configuration ##
###################################
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ${BEHAVIOR_COORDINATOR_INCLUDE_DIR}
  CATKIN_DEPENDS
  roscpp
  std_msgs
  behavior_coordinator_msgs
  behavior_execution_manager_msgs
  DEPENDS yaml-cpp
  rospy
  rospkg
)

###########
## Build ##
###########
include_directories(
  ${BEHAVIOR_COORDINATOR_INCLUDE_DIR}
  ${BEHAVIOR_COORDINATOR_SOURCE_DIR}
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(behavior_coordinator_lib ${BEHAVIOR_COORDINATOR_SOURCE_FILES} ${BEHAVIOR_COORDINATOR_HEADER_FILES})
add_dependencies(behavior_coordinator_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(behavior_coordinator_lib ${catkin_LIBRARIES})

add_executable(behavior_coordinator ${BEHAVIOR_COORDINATOR_SOURCE_DIR}/behavior_coordinator_main.cpp)
add_dependencies(behavior_coordinator ${catkin_EXPORTED_TARGETS})
target_link_libraries(behavior_coordinator behavior_coordinator_lib)
target_link_libraries(behavior_coordinator ${catkin_LIBRARIES})
target_link_libraries(behavior_coordinator yaml-cpp)

add_library(yamlCreator_lib ${YAML_CREATOR_SOURCE_FILES})
add_dependencies(yamlCreator_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(yamlCreator_lib ${catkin_LIBRARIES})
add_executable(yamlCreator ${BEHAVIOR_COORDINATOR_SOURCE_DIR}/yamlCreator.cpp)
target_link_libraries(yamlCreator yamlCreator_lib)
target_link_libraries(yamlCreator yaml-cpp)

catkin_install_python(PROGRAMS ${BEHAVIOR_COORDINATOR_TEST_DIR}/test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(
    behavior_coordinator_test
    launch/behavior_coordinator_test.launch
    ${BEHAVIOR_COORDINATOR_TEST_DIR}/behavior_coordinator_test.cpp
  )
  target_link_libraries(behavior_coordinator_test ${catkin_LIBRARIES})
  target_link_libraries(behavior_coordinator_test behavior_coordinator_lib)
  target_link_libraries(behavior_coordinator_test yamlCreator_lib)
  target_link_libraries(behavior_coordinator_test yaml-cpp)
endif()
