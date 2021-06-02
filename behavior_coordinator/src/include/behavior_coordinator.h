/*!********************************************************************************
 * \brief     behavior_manger definition file
 * \authors   Pablo Santamaria, Martin Molina
 * \copyright Copyright (c) 2020 Universidad Politecnica de Madrid
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#ifndef BEHAVIOR_COORDINATOR_H
#define BEHAVIOR_COORDINATOR_H

#include <string>
#include <list>
#include <algorithm>
#include <thread>
#include <set>

// ROS
#include <ros/ros.h>
#include <ros/this_node.h>
#include <ros/master.h>

// Aerostack
#include "../include/task.h"
#include "../include/constraint.h"
#include "../include/behavior.h"
#include "../include/catalog.h"
#include <behavior_coordinator_msgs/StartTask.h>
#include <behavior_coordinator_msgs/StopTask.h>
#include <behavior_coordinator_msgs/TaskStopped.h>
//#include <behavior_coordinator_msgs/ListOfRunningTasks.h>
//#include <behavior_coordinator_msgs/TaskCommand.h>
#include <behavior_execution_manager_msgs/BehaviorActivationFinished.h>
#include <behavior_execution_manager_msgs/ActivateBehavior.h>
#include <behavior_execution_manager_msgs/DeactivateBehavior.h>
#include <behavior_coordinator_msgs/ActivationChange.h>
#include <behavior_coordinator_msgs/BehaviorCoordinatorLog.h>

class BehaviorCoordinator
{
public:
  BehaviorCoordinator();
  ~BehaviorCoordinator();

  int DEFAULT_ACTIVATION_DELAY = 80; //ms
  int numberOfNoGoodConstraints = 0;
  int MAXIMUM_PERFORMANCE = 100;
  int MAXIMUM_NO_GOOD_CONSTRAINTS = 10;
  int MAXIMUM_SEARCH_TIME = 10; //ms

  Catalog* catalog;
  bool first_call = true;
  std::list<std::pair<Task, std::list<Behavior*>>> bestBehaviorAssignment;
  std::list<std::pair<Task*, std::chrono::time_point<std::chrono::system_clock>>> reactiveTasksToStart;
  std::chrono::time_point<std::chrono::system_clock> startSearchTime, endSearchTime;
  std::list<std::pair<Task, std::list<Behavior*>>> resetActivationList;
  std::list<Behavior*> lastAssignment;
  
  std::string robot_id;
  std::string catalog_path;
  std::string robot_namespace;
  std::string behavior_activation_finished_str;
  std::string start_task_str;
  std::string stop_task_str;
  std::string task_stopped_str;
  std::string consult_available_behaviors_str;
  //std::string list_of_running_tasks_str;
  std::string activation_change_str;
  ros::ServiceClient activate_behavior_srv;
  ros::ServiceClient deactivate_behavior_srv;
  ros::ServiceServer start_task_srv;
  ros::ServiceServer stop_task_srv;
  ros::Subscriber behavior_activation_finished_sub;
  ros::Publisher task_stopped_pub;
  //ros::Publisher list_of_running_tasks_pub;
  ros::Publisher activation_change_pub;
  ros::Publisher behavior_coordinator_log_pub;

  void behaviorActivationFinishedCallback(const behavior_execution_manager_msgs::BehaviorActivationFinished &message);
  bool activateTaskCallback(behavior_coordinator_msgs::StartTask::Request &request, behavior_coordinator_msgs::StartTask::Response &response);
  bool deactivateTaskCallback(behavior_coordinator_msgs::StopTask::Request &request, behavior_coordinator_msgs::StopTask::Response &response);

  // DroneProcess
  void init();
  void resetDomains();

  //DRIVE
  bool propagateConstraints();
  bool generateAssignment();
  bool testPerformanceConstraints();
  bool generateConsistentAssignment();
  Behavior* selectValueToExplore(Task* tasksit, std::list<Behavior*> searchDomain);
  bool checkAssignmentFound();
  std::list<std::list<Behavior*>> retrieveDomains();
  void restoreDomains(std::list<std::list<Behavior*>> savedDomains);
  bool generateBestAssignment(int priority, std::pair<Task*, std::list<Behavior*>> desiredDomain);
  void initializeSearch(int priority, std::pair<Task*, std::list<Behavior*>> desiredDomain);
  void removeNoGoodConstraints();
  void initializeSearchTime();
  void calculateTasksPerformance();
  void calculatePerformance();
  void retrieveAssignment(std::list<std::pair<Task, std::list<Behavior*>>>& assignment);
  bool addNoGood(std::list<std::pair<Task, std::list<Behavior*>>> assignment);
  int evaluateGlobalPerformance();
  bool exceededMaximumSearchTime();
  bool generateAndExecuteAssignment(int priority, std::pair<Task*, std::list<Behavior*>> desiredDomain);
  bool executeAssignment();
  void resetActivations();
  void setUp();
  std::list<Behavior**> calculateTaskDependencies(Task* taskIterator);
  Behavior** getMaxSuitabilityValue(Task* taskIterator);
  bool checkConstraint(Constraint* constraint);
  std::list<std::pair<Task, std::list<Behavior*>>> generateAssignmentExplainingViolation(Constraint* constraint);
  double evaluateObjectiveFunction1MinActive();
  double evaluateObjectiveFunction1MinChanges();
  double evaluateObjectiveFunction1MaxSuitability();
  double evaluateObjectiveFunction2();
  void loadTasksConstraints();
  bool activateBehavior(Behavior* behavior, std::string arguments, int priority);
  bool deactivateBehavior(Behavior behavior);
  void deactivateBehaviorFailed(Behavior behavior);
  void stopBehaviorCoordinator();
  void initializeIncompatibleTasks();
  void checkReactiveStarts();
  void checkTimeouts();
  //void publishListOfRunningTasks();
  void removeReactiveTask(Task * requestedTask);

  //DEBUG
  void printBestBehaviorAssignment();
  int solutions = 0;
  std::list<Constraint*> agendaG;
  std::list<Constraint*> noGoodconstraints;
  bool testing;

private:

  bool DEBUG;

  //Communication variables
  ros::NodeHandle node_handle;
  ros::Publisher execution_request_pub;
  std::string execution_request_str;
};

#endif
