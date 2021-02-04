#include <gtest/gtest.h>
#include "behavior_manager.h"
#include "../include/catalog.h"
#include "../include/yamlCreator.h"
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <ros/package.h>
using namespace std;

ros::NodeHandle* n;
ros::Publisher deactivate_behavior_pub;
std::string path;

bool startTask(std::string task_name, int priority, std::string parameters) {
  ros::ServiceClient start_task_srv = n->serviceClient<behavior_manager_msg::StartTask>("/drone1/start_task");
  behavior_manager_msg::StartTask msg;
  //behavior_manager_msg::TaskCommand task_cmd;
  msg.request.task.name = task_name;
  msg.request.task.priority = priority;
  msg.request.task.parameters = parameters;
  if (start_task_srv.call(msg)){
    return msg.response.ack;
  }
  return false;
}

bool stopTask(std::string task_name) {
  ros::ServiceClient stop_task_srv = n->serviceClient<behavior_manager_msg::StopTask>("stop_task");
  behavior_manager_msg::StopTask msg;
  msg.request.name = task_name;
  if (stop_task_srv.call(msg)){
    return true;
  }
  return msg.response.ack;
}

std::string getActiveBehavior(std::string task_name){
  aerostack_msgs::ListOfRunningTasks running_list = *ros::topic::waitForMessage<aerostack_msgs::ListOfRunningTasks>("/drone1/list_of_running_tasks",*n, ros::Duration(1));
  ros::spinOnce();
  for(int i = 0; i<running_list.list_of_running_tasks.size(); i++){
    if(running_list.list_of_running_tasks[i].task_command.name == task_name){
      return running_list.list_of_running_tasks[i].behavior;
    }
  }
  return "";
}

bool simulateBehaviorFinished(std::string task_name, int cause, int delay_ms){
  std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
  std::string behavior_name = getActiveBehavior(task_name);
  behaviorlib_msg::BehaviorActivationFinished msg;
  msg.name = behavior_name;
  msg.termination_cause = cause;
  if(deactivate_behavior_pub.getNumSubscribers()>0){
    deactivate_behavior_pub.publish(msg);
    ros::spinOnce();
    return true;
  }
  return false;
}

TEST(BehaviorManagerTest0, Existing_Behavior) {
  bool status = startTask("TAKE_OFF", 2, "");
  status = status && simulateBehaviorFinished("TAKE_OFF", behaviorlib_msg::BehaviorActivationFinished::GOAL_ACHIEVED, 100);
  status = status && startTask("FOLLOW_PATH", 2, "");
  status = status && simulateBehaviorFinished("FOLLOW_PATH", behaviorlib_msg::BehaviorActivationFinished::INTERRUPTED, 100);
  //status = status && stopTask("FOLLOW_PATH");
  status = status && startTask("FOLLOW_PATH", 2, "");
  status = status && startTask("FOLLOW_PATH", 2, "");
  status = status && simulateBehaviorFinished("FOLLOW_PATH", behaviorlib_msg::BehaviorActivationFinished::WRONG_PROGRESS, 100);
  //status = status && stopTask("FOLLOW_PATH");
  status = status && startTask("LAND", 2, "");
  status = status && simulateBehaviorFinished("LAND", behaviorlib_msg::BehaviorActivationFinished::GOAL_ACHIEVED, 100);
  EXPECT_TRUE(status);
}

TEST(BehaviorManagerTest2, Nonexisting_Behavior) {
  bool status = startTask("NON_EXISTING_TASK", 2, "");
  EXPECT_FALSE(status);
}

TEST(BehaviorManagerTest3, Correct_Behavior_Catalog) {
  std::string catalog_path = path + "/src/test/behavior_catalog.yaml";
  Catalog* catalog = new Catalog(catalog_path);
  bool status = true;
  if(catalog->tasks.size() != 21){
    status = false;
  }
  if(catalog->behaviors.size() != 47){
    status = false;
  }
  if(catalog->constraints.size() != 33){
    status = false;
  }
  EXPECT_TRUE(status);
}

TEST(BehaviorManagerTest4, Imposible_Behavior_Catalog) {
  std::string catalog_path = path + "/src/test/behavior_catalog_impossible.yaml";
  Catalog* catalog = new Catalog(catalog_path);
  bool status = true;
  if(catalog->errors.size() > 0){
    status = false;
  }
  EXPECT_FALSE(status);
}

TEST(BehaviorManagerTest5, Missing_task_Behavior_Catalog) {
  std::string catalog_path = path + "/src/test/behavior_catalog_missing_task.yaml";
  Catalog* catalog = new Catalog(catalog_path);
  bool status = true;
  if(catalog->errors.size() > 0){
    status = false;
  }
  EXPECT_FALSE(status);
}

TEST(BehaviorManagerTest6, Bad_Behavior_Catalog) {
  std::string catalog_path = path + "/src/test/bad_behavior_catalog.yaml";
  Catalog* catalog = new Catalog(catalog_path);
  bool status = true;
  if(catalog->errors.size() > 0){
    status = false;
  }
  EXPECT_FALSE(status);
}

TEST(BehaviorManagerTest7, Generate_Behavior_Catalog) {
  Tester creator;
  creator.T = 12;
  creator.B = 3;
  creator.R = 2;
  creator.L = 3;
  creator.M = 3;
  creator.behavior_catalog_path = path + "/src/test/creator.yaml";
  creator.CreateYaml();
  std::string catalog_path = path + "/src/test/creator.yaml";
  Catalog* catalog = new Catalog(catalog_path);
  bool status = true;
  if(catalog->errors.size() > 0){
    status = false;
  }
  EXPECT_TRUE(status);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "tester");
  n = new ros::NodeHandle;
  path = ros::package::getPath("behavior_manager");
  deactivate_behavior_pub = n->advertise<behaviorlib_msg::BehaviorActivationFinished>("/drone1/behavior_activation_finished", 0);
  testing::InitGoogleTest(&argc, argv);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  return RUN_ALL_TESTS();
  delete n;
}