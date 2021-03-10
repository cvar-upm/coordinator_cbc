#include <gtest/gtest.h>
#include "behavior_coordinator.h"
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
std::list<std::pair<std::string,std::string>> list_of_tasks;
bool stopThread = false;

bool startTask(std::string task_name, int priority, std::string parameters) {
  ros::ServiceClient start_task_srv = n->serviceClient<behavior_coordinator_msgs::StartTask>("/drone1/start_task");
  behavior_coordinator_msgs::StartTask msg;
  //behavior_coordinator_msgs::TaskCommand task_cmd;
  msg.request.task.name = task_name;
  msg.request.task.priority = priority;
  msg.request.task.parameters = parameters;
  if (start_task_srv.call(msg)){
    for(int i = 0; i<msg.response.task.size(); i++){
      bool found = false;
      for(std::list<std::pair<std::string,std::string>>::iterator list_of_tasks_iterator = list_of_tasks.begin(); list_of_tasks_iterator != list_of_tasks.end(); list_of_tasks_iterator++){
        if(list_of_tasks_iterator->first == msg.response.task[i]){
          found = true;
          list_of_tasks_iterator->second = msg.response.behavior[i];
        }
      }
      if(!found){
        std::pair<std::string,std::string> newTask;
        newTask.first = msg.response.task[i];
        newTask.second = msg.response.behavior[i];
        list_of_tasks.push_back(newTask);
      }
    }
    return msg.response.ack;
  }
  return false;
}

bool stopTask(std::string task_name) {
  ros::ServiceClient stop_task_srv = n->serviceClient<behavior_coordinator_msgs::StopTask>("stop_task");
  behavior_coordinator_msgs::StopTask msg;
  msg.request.name = task_name;
  if (stop_task_srv.call(msg)){
    return true;
  }
  return msg.response.ack;
}

std::string getActiveBehavior(std::string task_name){
  for(std::list<std::pair<std::string,std::string>>::iterator list_of_tasks_iterator = list_of_tasks.begin(); list_of_tasks_iterator != list_of_tasks.end(); list_of_tasks_iterator++){
    if(list_of_tasks_iterator->first == task_name){
      return list_of_tasks_iterator->second;
    }
  }
  return "";
}

bool simulateBehaviorFinished(std::string task_name, int cause, int delay_ms){
  std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
  std::string behavior_name = getActiveBehavior(task_name);
  behavior_execution_manager_msgs::BehaviorActivationFinished msg;
  msg.name = behavior_name;
  msg.termination_cause = cause;
  if(deactivate_behavior_pub.getNumSubscribers()>0){
    deactivate_behavior_pub.publish(msg);
    ros::spinOnce();
    return true;
  }
  return false;
}

void bmanager(BehaviorCoordinator manager)
{
  ros::Rate r(100); // 100 hz
  while (ros::ok()){
    if(stopThread){
      manager.stopBehaviorCoordinator();
      return;
    }
    ros::spinOnce();
    manager.checkDefaultActivations();
    manager.checkTimeouts();
    r.sleep();
  }
}

TEST(BehaviorCoordinatorTest0, Real_Catalog) {
  stopThread = false;
  int argc=0;
  char *argv[0];
  ros::init(argc, argv, ros::this_node::getName());
  std::cout << ros::this_node::getName() << std::endl;
  BehaviorCoordinator manager;
  try
  {
    manager.init();
    manager.catalog_path = path + "/src/test/behavior_catalog.yaml";
  }
  catch (std::exception &exception)
  {
    manager.stopBehaviorCoordinator();
  }
  std::thread second (bmanager,manager);
  bool status = startTask("TAKE_OFF", 2, "");
  //status = status && simulateBehaviorFinished("TAKE_OFF", behavior_execution_manager_msgs::BehaviorActivationFinished::GOAL_ACHIEVED, 100);
  status = status && startTask("FOLLOW_PATH", 2, "");
  /*
  status = status && simulateBehaviorFinished("FOLLOW_PATH", behavior_execution_manager_msgs::BehaviorActivationFinished::INTERRUPTED, 100);
  //status = status && stopTask("FOLLOW_PATH");
  status = status && startTask("FOLLOW_PATH", 2, "");
  status = status && startTask("FOLLOW_PATH", 2, "");
  status = status && simulateBehaviorFinished("FOLLOW_PATH", behavior_execution_manager_msgs::BehaviorActivationFinished::WRONG_PROGRESS, 100);
  //status = status && stopTask("FOLLOW_PATH");
  status = status && startTask("LAND", 2, "");
  status = status && simulateBehaviorFinished("LAND", behavior_execution_manager_msgs::BehaviorActivationFinished::GOAL_ACHIEVED, 100);*/
  stopThread = true;
  second.join();
  EXPECT_TRUE(status);
}

/*
TEST(BehaviorCoordinatorTest1, 6_3_3_3_0_Catalog) {
  Tester creator;
  creator.T = 3;
  creator.B = 3;
  creator.R = 3;
  creator.L = 3;
  creator.M = 0;
  creator.behavior_catalog_path = path + "/src/test/creator.yaml";
  creator.CreateYaml();
  std::string catalog_path = path + "/src/test/creator.yaml";
  std::chrono::system_clock::time_point activationTime = std::chrono::system_clock::now();
  Catalog* catalog = new Catalog(catalog_path);
  std::cout<<"Catalog loading took "<<std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - activationTime).count()<<" ms"<<std::endl;
  bool status = true;
  if(catalog->errors.size() > 0){
    status = false;
  }
  if(status){
    std::string catalog_path = path + "/src/test/behavior_catalog.yaml";
    std::chrono::system_clock::time_point activationTime = std::chrono::system_clock::now();
    Catalog* catalog = new Catalog(catalog_path);
    stopThread = false;
    int argc=0;
    char *argv[0];
    ros::init(argc, argv, ros::this_node::getName());
    std::cout << ros::this_node::getName() << std::endl;
    BehaviorCoordinator manager;
    try
    {
      manager.init();
      manager.catalog_path = catalog_path;
    }
    catch (std::exception &exception)
    {
      manager.stopBehaviorCoordinator();
    }
    std::thread second (bmanager,manager);
    bool status = startTask("task0_0", 2, "");
    status = status && startTask("task2_0", 2, "");
    status = status && startTask("task1_1", 2, "");
    status = status && startTask("task2_2", 2, "");
    stopThread = true;
    second.join();
  }
  EXPECT_TRUE(status);
}
*/
/*
TEST(BehaviorCoordinatorTest1, 120_3_3_3_0_Catalog) {
  Tester creator;
  creator.T = 3;
  creator.B = 2;
  creator.R = 2;
  creator.L = 3;
  creator.M = 0;
  creator.behavior_catalog_path = path + "/src/test/creator.yaml";
  creator.CreateYaml();
  std::string catalog_path = path + "/src/test/creator.yaml";
  std::chrono::system_clock::time_point activationTime = std::chrono::system_clock::now();
  Catalog* catalog = new Catalog(catalog_path);
  std::cout<<"Catalog loading took "<<std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - activationTime).count()<<" ms"<<std::endl;
  bool status = true;
  if(catalog->errors.size() > 0){
    status = false;
  }
  if(status){
    stopThread = false;
    int argc=0;
    char *argv[0];
    ros::init(argc, argv, ros::this_node::getName());
    std::cout << ros::this_node::getName() << std::endl;
    BehaviorCoordinator manager;
    try
    {
      manager.init();
      manager.catalog_path = catalog_path;
    }
    catch (std::exception &exception)
    {
      manager.stopBehaviorCoordinator();
    }
    std::thread second (bmanager,manager);
    bool status = startTask("task0_0", 2, "");
    
    status = status && startTask("empty", 2, "");
    status = status && startTask("task0_1", 2, "");
    status = status && startTask("empty", 2, "");
    status = status && startTask("task0_0", 2, "");
    status = status && startTask("empty", 2, "");
    status = status && startTask("task0_1", 2, "");
    
    stopThread = true;
    second.join();
  }
  EXPECT_TRUE(status);
}
*/
int main(int argc, char **argv){
  ros::init(argc, argv, "tester");
  n = new ros::NodeHandle;
  path = ros::package::getPath("behavior_coordinator");
  deactivate_behavior_pub = n->advertise<behavior_execution_manager_msgs::BehaviorActivationFinished>("/drone1/behavior_activation_finished", 0);
  testing::InitGoogleTest(&argc, argv);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  return RUN_ALL_TESTS();
  delete n;
}