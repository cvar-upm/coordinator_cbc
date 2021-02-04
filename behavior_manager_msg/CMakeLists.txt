cmake_minimum_required(VERSION 2.8.3)
project(behavior_manager_msg)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation)

add_message_files(
  DIRECTORY msg
  FILES
  TaskCommand.msg
  TaskBehavior.msg
  ActivationChange.msg
  ListOfRunningTasks.msg
  TaskStopped.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  StartTask.srv
  StopTask.srv
)

generate_messages(
  DEPENDENCIES geometry_msgs std_msgs
)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS message_runtime geometry_msgs
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
