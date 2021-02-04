cmake_minimum_required(VERSION 2.8.3)
project(behavior_manager)

### Use version 2011 of C++ (c++11). By default ROS uses c++98
add_definitions(-std=c++11)
add_definitions(-g)

# Directories definition
set(BEHAVIOR_MANAGER_SOURCE_DIR
  src/source
)

set(BEHAVIOR_MANAGER_INCLUDE_DIR
  src/include
)

set(BEHAVIOR_MANAGER_TEST_DIR
  src/test
)

# Files declaration
set(BEHAVIOR_MANAGER_SOURCE_FILES
  ${BEHAVIOR_MANAGER_SOURCE_DIR}/behavior_manager.cpp
  ${BEHAVIOR_MANAGER_SOURCE_DIR}/behavior_manager_main.cpp
  ${BEHAVIOR_MANAGER_SOURCE_DIR}/behavior.cpp
  ${BEHAVIOR_MANAGER_SOURCE_DIR}/task.cpp
  ${BEHAVIOR_MANAGER_SOURCE_DIR}/constraint.cpp
  ${BEHAVIOR_MANAGER_SOURCE_DIR}/catalog.cpp
  ${BEHAVIOR_MANAGER_SOURCE_DIR}/prettify.cpp
)

# Files declaration
set(TEST_SOURCE_FILES
  ${BEHAVIOR_MANAGER_TEST_DIR}/test.py
)

# Files declaration
set(YAML_CREATOR_SOURCE_FILES
  ${BEHAVIOR_MANAGER_SOURCE_DIR}/yamlCreator.cpp
)

set(BEHAVIOR_MANAGER_HEADER_FILES
  ${BEHAVIOR_MANAGER_INCLUDE_DIR}/behavior_manager.h
  ${BEHAVIOR_MANAGER_INCLUDE_DIR}/behavior.h
  ${BEHAVIOR_MANAGER_INCLUDE_DIR}/task.h
  ${BEHAVIOR_MANAGER_INCLUDE_DIR}/constraint.h
  ${BEHAVIOR_MANAGER_INCLUDE_DIR}/catalog.h
  ${BEHAVIOR_MANAGER_INCLUDE_DIR}/prettify.h
)

### Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  roslib
  std_msgs
  behavior_manager_msg
)

###################################
## catkin specific configuration ##
###################################
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ${BEHAVIOR_MANAGER_INCLUDE_DIR}
  CATKIN_DEPENDS
  roscpp
  std_msgs
  DEPENDS yaml-cpp
  rospy
  rospkg
)

###########
## Build ##
###########
include_directories(
  ${BEHAVIOR_MANAGER_INCLUDE_DIR}
  ${BEHAVIOR_MANAGER_SOURCE_DIR}
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(behavior_manager_lib ${BEHAVIOR_MANAGER_SOURCE_FILES} ${BEHAVIOR_MANAGER_HEADER_FILES})
add_dependencies(behavior_manager_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(behavior_manager_lib ${catkin_LIBRARIES})

add_executable(behavior_manager ${BEHAVIOR_MANAGER_SOURCE_DIR}/behavior_manager_main.cpp)
add_dependencies(behavior_manager ${catkin_EXPORTED_TARGETS})
target_link_libraries(behavior_manager behavior_manager_lib)
target_link_libraries(behavior_manager ${catkin_LIBRARIES})
target_link_libraries(behavior_manager yaml-cpp)

add_library(yamlCreator_lib ${YAML_CREATOR_SOURCE_FILES})
add_dependencies(yamlCreator_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(yamlCreator_lib ${catkin_LIBRARIES})
add_executable(yamlCreator ${BEHAVIOR_MANAGER_SOURCE_DIR}/yamlCreator.cpp)
target_link_libraries(yamlCreator yamlCreator_lib)
target_link_libraries(yamlCreator yaml-cpp)

catkin_install_python(PROGRAMS ${BEHAVIOR_MANAGER_TEST_DIR}/test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(
    behavior_manager_test
    launch/behavior_manager_test.launch
    ${BEHAVIOR_MANAGER_TEST_DIR}/behavior_manager_test.cpp
  )
  target_link_libraries(behavior_manager_test ${catkin_LIBRARIES})
  target_link_libraries(behavior_manager_test behavior_manager_lib)
  target_link_libraries(behavior_manager_test yamlCreator_lib)
  target_link_libraries(behavior_manager_test yaml-cpp)
endif()
