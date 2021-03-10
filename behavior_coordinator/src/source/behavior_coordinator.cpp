/*!********************************************************************************
 * \brief     This file implements the behavior_coordinator class
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

#include "../include/behavior_coordinator.h"
#include <pluginlib/class_list_macros.h>

BehaviorCoordinator::BehaviorCoordinator(){
  DEBUG = false;
}

BehaviorCoordinator::~BehaviorCoordinator(){
}

void BehaviorCoordinator::init(){
  setUp();
  std::cout<<"Behavior coordinator started"<<std::endl;
}

void BehaviorCoordinator::stopBehaviorCoordinator(){
  node_handle.shutdown();
  start_task_srv.shutdown();
  stop_task_srv.shutdown();
  task_stopped_pub.shutdown();
  //list_of_running_tasks_pub.shutdown();
  behavior_activation_finished_sub.shutdown();
}

void BehaviorCoordinator::setUp(){
  ros::NodeHandle private_nh;
  private_nh.getParam("robot_namespace", robot_namespace);
  private_nh.getParam("testing", testing);
  private_nh.getParam("catalog_path", catalog_path);
  private_nh.param<std::string>("behavior_activation_finished_topic", behavior_activation_finished_str,
                                "behavior_activation_finished");
  private_nh.param<std::string>("start_task", start_task_str, "start_task");
  private_nh.param<std::string>("stop_task", stop_task_str, "stop_task");
  private_nh.param<std::string>("task_stopped", task_stopped_str, "task_stopped");
  //private_nh.param<std::string>("list_of_running_tasks", list_of_running_tasks_str, "list_of_running_tasks");
  private_nh.param<std::string>("activation_change", activation_change_str, "activation_change");
  
  start_task_srv = node_handle.advertiseService("/" + robot_namespace + "/" + start_task_str,
                                                &BehaviorCoordinator::activateTaskCallback, this);
  stop_task_srv = node_handle.advertiseService("/" + robot_namespace + "/" + stop_task_str,
                                                &BehaviorCoordinator::deactivateTaskCallback, this);
  task_stopped_pub = node_handle.advertise<behavior_coordinator_msgs::TaskStopped>("/" + robot_namespace +
                                           "/" +task_stopped_str, 1);
  /*
  list_of_running_tasks_pub = node_handle.advertise<behavior_coordinator_msgs::ListOfRunningTasks>("/" + robot_namespace +
                                                    "/" +list_of_running_tasks_str, 1);
  */
  activation_change_pub = node_handle.advertise<behavior_coordinator_msgs::ActivationChange>("/" + robot_namespace +
                                                    "/" +activation_change_str, 1);
  behavior_activation_finished_sub = node_handle.subscribe('/' + robot_namespace + '/' + behavior_activation_finished_str,
                                                           100, &BehaviorCoordinator::behaviorActivationFinishedCallback, this);
  private_nh.shutdown();
}

/*
void BehaviorCoordinator::publishListOfRunningTasks(){
  behavior_coordinator_msgs::ListOfRunningTasks list_of_running_tasks_msg;
  for (std::list<Task*>::iterator tasksIterator = catalog->tasks.begin(); tasksIterator != catalog->tasks.end(); ++tasksIterator){
    if((*tasksIterator)->active){
      behavior_coordinator_msgs::TaskBehavior task_and_behavior;
      behavior_coordinator_msgs::TaskCommand running_task;
      running_task.name = (*tasksIterator)->name;
      running_task.parameters = (*tasksIterator)->arguments;
      running_task.priority = (*tasksIterator)->priority;
      task_and_behavior.task_command = running_task;
      task_and_behavior.behavior = (*tasksIterator)->activeBehavior->name;
      list_of_running_tasks_msg.list_of_running_tasks.push_back(task_and_behavior);
    }
  }
  if(list_of_running_tasks_msg.list_of_running_tasks.size()>0){
    list_of_running_tasks_pub.publish(list_of_running_tasks_msg);
  }
}
*/

void BehaviorCoordinator::checkTimeouts(){
  if(!first_call){
    for (std::list<Task*>::iterator tasksIterator = catalog->tasks.begin(); tasksIterator != catalog->tasks.end(); ++tasksIterator){
      if((*tasksIterator)->active && (*tasksIterator)->category != "recurrent"
      && std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - (*tasksIterator)->activationTime).count() > (*tasksIterator)->timeout){
        std::cout<<"[Timeout expired]"<<std::endl;
        for (std::list<Behavior*>::iterator lastAssignmentIterator = lastAssignment.begin();
        lastAssignmentIterator != lastAssignment.end(); ++lastAssignmentIterator){
          if ((*lastAssignmentIterator)->task == *tasksIterator){
            lastAssignment.erase(lastAssignmentIterator);
            break;
          }
        }
        if(deactivateBehavior(*(*tasksIterator)->activeBehavior)){
          (*tasksIterator)->activationTime = std::chrono::system_clock::now();
          std::pair<Task*, std::list<Behavior*>> desiredDomain;
          desiredDomain.first = *tasksIterator;
          desiredDomain.second.push_back((*tasksIterator)->inactive);
          generateAndExecuteAssignment((*tasksIterator)->priority, desiredDomain);
        }
        else{
          deactivateBehaviorFailed(*(*tasksIterator)->activeBehavior);
        }
      }
    }
  }
}

void BehaviorCoordinator::deactivateBehaviorFailed(Behavior behavior){
  behavior.task->active = false;
}

void BehaviorCoordinator::checkDefaultActivations(){
  if(defaultTasksToActivate.size()>0){
    std::list<std::pair<Task*, std::chrono::time_point<std::chrono::system_clock>>>::iterator defaultTasksToActivateIterator = defaultTasksToActivate.begin();
    while(defaultTasksToActivateIterator != defaultTasksToActivate.end()){
      if(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - defaultTasksToActivateIterator->second).count()>DEFAULT_ACTIVATION_DELAY){
        std::pair<Task*, std::list<Behavior*>> desiredDomain;
        desiredDomain.first = defaultTasksToActivateIterator->first;
        desiredDomain.second.clear();
        std::cout<<"[Default activation]"<<std::endl;
        defaultTasksToActivateIterator->first->defaultActivated = true;
        defaultTasksToActivateIterator->first->priority = 1;
        defaultTasksToActivateIterator->second = std::chrono::system_clock::now();
        std::cout<<"  "<<desiredDomain.first->name<<" (priority: "<< defaultTasksToActivateIterator->first->priority<<")"<<std::endl;
        for (std::list<Behavior*>::iterator behaviorsIterator = defaultTasksToActivateIterator->first->candidateBehaviors.begin();
        behaviorsIterator != defaultTasksToActivateIterator->first->candidateBehaviors.end(); ++behaviorsIterator){
          if(*behaviorsIterator != defaultTasksToActivateIterator->first->inactive){
            desiredDomain.second.push_back(*behaviorsIterator);
          }
        }
        std::list<std::pair<Task*, std::chrono::time_point<std::chrono::system_clock>>>::iterator test = defaultTasksToActivateIterator;
        test++;
        if(!generateAndExecuteAssignment(1,desiredDomain)){
          defaultTasksToActivateIterator=defaultTasksToActivate.erase(defaultTasksToActivateIterator);
        }
        defaultTasksToActivateIterator=test;
      }
      else{
        defaultTasksToActivateIterator++;
      }
    }
  }
}

/*
-----------
 Callbacks
-----------
*/

bool BehaviorCoordinator::activateTaskCallback(behavior_coordinator_msgs::StartTask::Request &request, behavior_coordinator_msgs::StartTask::Response &response){
  if(first_call){
    first_call = false;
    catalog = new Catalog(catalog_path);
    initializeIncompatibleTasks();
    for (std::list<Task*>::iterator tasksIterator = catalog->tasks.begin(); tasksIterator != catalog->tasks.end(); ++tasksIterator){
      for (std::list<Constraint*>::iterator constraintsit = catalog->constraints.begin(); constraintsit != catalog->constraints.end(); ++constraintsit){
        if((*tasksIterator)==(*constraintsit)->mainTask){
          (*tasksIterator)->mainInConstraints.push_back((*constraintsit));
        }
        else if((std::find((*constraintsit)->otherTasks.begin(), (*constraintsit)->otherTasks.end(), (*tasksIterator)) != (*constraintsit)->otherTasks.end())){
          (*tasksIterator)->secondaryInConstraints.push_back((*constraintsit));
        }
      }
    }
    //std::cout<<"CONSTRAINTS: "<<catalog->constraints.size()<<std::endl;
  }
  
  std::cout << "[Activation request]"<< std::endl;
  std::cout << "  " << request.task.name;
  if((request.task.parameters).size()>0){
    std::cout << " " << request.task.parameters;
  }
  std::cout <<" (priority: " <<int(request.task.priority)<<")"<< std::endl;
  Task* requestedTask;
  bool taskExists = false;
  for (std::list<Task*>::iterator tasksIterator = catalog->tasks.begin(); tasksIterator != catalog->tasks.end(); ++tasksIterator){
    if ((*tasksIterator)->name == request.task.name){
      taskExists = true;
      requestedTask = *tasksIterator;
      break;
    }
  }
  if(taskExists && int(request.task.priority) >= requestedTask->priority){
    requestedTask->requested = true;
    if(requestedTask->active && request.task.priority >= int(requestedTask->priority)){
      std::cout<<"[Behaviors activations]"<<std::endl;
      requestedTask->arguments = request.task.parameters;
      requestedTask->priority = int(request.task.priority);
      Behavior* reactivate_behavior = &*requestedTask->activeBehavior;
      if(deactivateBehavior(*requestedTask->activeBehavior) && activateBehavior(reactivate_behavior, requestedTask->arguments, int(request.task.priority))){
        reactivate_behavior->task->change_type = behavior_coordinator_msgs::ActivationChange::REQUESTED_DEACTIVATION;
        response.ack = true;
        removeDefaultTask(requestedTask);
        return true;
      }
    }
    std::pair<Task*, std::list<Behavior*>> desiredDomain;
    desiredDomain.first = requestedTask;
    desiredDomain.second.clear();
    for (std::list<Behavior*>::iterator behaviorsIterator = requestedTask->candidateBehaviors.begin();
    behaviorsIterator != requestedTask->candidateBehaviors.end(); ++behaviorsIterator){
      if(*behaviorsIterator != requestedTask->inactive){
        desiredDomain.second.push_back(*behaviorsIterator);
      }
    }
    requestedTask->arguments = request.task.parameters;
    requestedTask->priority = int(request.task.priority);
    response.ack = generateAndExecuteAssignment(int(request.task.priority),desiredDomain);
  }
  else{
    response.ack = false;
    response.error_message = "No possible activations found. (The required behaviors nodes might not be active or incompatible request with current activations)";
  }
  for (std::list<Behavior*>::iterator lastAssignmentIterator = lastAssignment.begin();
  lastAssignmentIterator != lastAssignment.end(); ++lastAssignmentIterator){
    response.task.push_back((*lastAssignmentIterator)->task->name);
    response.behavior.push_back((*lastAssignmentIterator)->name);
  }
  if(response.ack){
    removeDefaultTask(requestedTask);
  }
  return true;
}

void BehaviorCoordinator::removeDefaultTask(Task * requestedTask){
  for(std::list<Task*>::iterator incompatibleTasksIterator = requestedTask->incompatibleTasks.begin();
  incompatibleTasksIterator != requestedTask->incompatibleTasks.end(); ++incompatibleTasksIterator){
    std::list<std::pair<Task*, std::chrono::time_point<std::chrono::system_clock>>>::iterator defaultTasksToActivateIterator = defaultTasksToActivate.begin();
    while(defaultTasksToActivateIterator != defaultTasksToActivate.end()){
      if(defaultTasksToActivateIterator->first == *incompatibleTasksIterator){
        defaultTasksToActivateIterator = defaultTasksToActivate.erase(defaultTasksToActivateIterator);
      }
      else{
        defaultTasksToActivateIterator++;
      }
    }
  }
}

bool BehaviorCoordinator::deactivateTaskCallback(behavior_coordinator_msgs::StopTask::Request &request, behavior_coordinator_msgs::StopTask::Response &response){
  std::string taskName = request.name;
  std::cout << "[Deactivation request]"<< std::endl;
  std::cout << "  " << taskName << std::endl;
  for (std::list<Behavior*>::iterator lastAssignmentIterator = lastAssignment.begin();
  lastAssignmentIterator != lastAssignment.end(); ++lastAssignmentIterator){
    if ((*lastAssignmentIterator)->task->name == taskName){
      std::pair<Task*, std::list<Behavior*>> desiredDomain;
      std::string behavior = (*lastAssignmentIterator)->name;
      desiredDomain.first = (*lastAssignmentIterator)->task;
      desiredDomain.second.clear();
      desiredDomain.second.push_back((*lastAssignmentIterator)->task->inactive);
      if(generateAndExecuteAssignment((*lastAssignmentIterator)->task->priority,desiredDomain)){
        response.ack = true;
      }
      else{
        response.ack = false;
      }
      break;
    }
  }
  return true;
}

void BehaviorCoordinator::behaviorActivationFinishedCallback(const behavior_execution_manager_msgs::BehaviorActivationFinished &message){
  std::string behavior = message.name;
  bool found = false;
  for (std::list<Behavior*>::iterator lastAssignmentIterator = lastAssignment.begin();
  lastAssignmentIterator != lastAssignment.end(); ++lastAssignmentIterator){
    if ((*lastAssignmentIterator)->name == behavior){
      found = true;
      std::cout<<"[Behavior activation finished]"<<std::endl;
      std::cout<<"  "<<behavior;
      switch(message.termination_cause){
        case behavior_execution_manager_msgs::BehaviorActivationFinished::GOAL_ACHIEVED:
            std::cout<<" (GOAL_ACHIEVED)"<<std::endl;
            (*lastAssignmentIterator)->task->change_type = behavior_coordinator_msgs::ActivationChange::GOAL_ACHIEVED_SELF_DEACTIVATION;
            break;
        case behavior_execution_manager_msgs::BehaviorActivationFinished::TIME_OUT:
            std::cout<<" (TIME_OUT)"<<std::endl;
            (*lastAssignmentIterator)->task->change_type = behavior_coordinator_msgs::ActivationChange::TIME_OUT_DEACTIVATION;
            break;
        case behavior_execution_manager_msgs::BehaviorActivationFinished::WRONG_PROGRESS:
            std::cout<<" (WRONG_PROGRESS)"<<std::endl;
            (*lastAssignmentIterator)->task->change_type = behavior_coordinator_msgs::ActivationChange::WRONG_PROGRESS_SELF_DEACTIVATION;
            break;
        case behavior_execution_manager_msgs::BehaviorActivationFinished::PROCESS_FAILURE:
            std::cout<<" (PROCESS_FAILURE)"<<std::endl;
            (*lastAssignmentIterator)->task->change_type = behavior_coordinator_msgs::ActivationChange::PROCESS_FAILURE_SELF_DEACTIVATION;
            break;
        case behavior_execution_manager_msgs::BehaviorActivationFinished::INTERRUPTED:
            if((*lastAssignmentIterator)->task->change_type == behavior_coordinator_msgs::ActivationChange::REQUESTED_DEACTIVATION){
              std::cout<<" (REQUESTED_DEACTIVATION)"<<std::endl;
              return;
            }
            std::cout<<" (UNKNOWN_DEACTIVATION)"<<std::endl;
            (*lastAssignmentIterator)->task->change_type = behavior_coordinator_msgs::ActivationChange::UNKNOWN_DEACTIVATION;
            break;
        default:
            std::cout<<message.error_message;
      }
      behavior_coordinator_msgs::ActivationChange activation_change_msg;
      activation_change_msg.change_type = (*lastAssignmentIterator)->task->change_type;
      activation_change_msg.task_behavior.behavior = behavior;
      activation_change_msg.task_behavior.task_command.name = (*lastAssignmentIterator)->task->name;
      activation_change_msg.task_behavior.task_command.parameters = (*lastAssignmentIterator)->task->arguments;
      activation_change_msg.task_behavior.task_command.priority = (*lastAssignmentIterator)->task->priority;
      activation_change_pub.publish(activation_change_msg);
      (*lastAssignmentIterator)->task->change_type = -1;
      bool found = false;
      for(std::list<Task*>::iterator incompatibleTasksIterator = (*lastAssignmentIterator)->task->incompatibleDefaultTasks.begin();
        incompatibleTasksIterator != (*lastAssignmentIterator)->task->incompatibleDefaultTasks.end(); ++incompatibleTasksIterator){
          bool defaultTaskWaiting = false;
          for(std::list<std::pair<Task*, std::chrono::time_point<std::chrono::system_clock>>>::iterator defaultTasksToActivateIterator = defaultTasksToActivate.begin();
          defaultTasksToActivateIterator != defaultTasksToActivate.end(); ++defaultTasksToActivateIterator){
            if(defaultTasksToActivateIterator->first == *incompatibleTasksIterator && !(*incompatibleTasksIterator)->active){
              defaultTaskWaiting = true;
            }
          }
          if(!defaultTaskWaiting){
            std::pair<Task*, std::chrono::time_point<std::chrono::system_clock>> newTask(*incompatibleTasksIterator,std::chrono::system_clock::now());
            defaultTasksToActivate.push_back(newTask);
          }
        }
        for(std::list<std::pair<Task*, std::chrono::time_point<std::chrono::system_clock>>>::iterator defaultTasksToActivateIterator = defaultTasksToActivate.begin();
        defaultTasksToActivateIterator != defaultTasksToActivate.end(); ++defaultTasksToActivateIterator){
          if(defaultTasksToActivateIterator->first == (*lastAssignmentIterator)->task){
            defaultTasksToActivate.erase(defaultTasksToActivateIterator);
            break;
          }
        }
      if(message.termination_cause != behavior_execution_manager_msgs::BehaviorActivationFinished::GOAL_ACHIEVED){ //(TIME_OUT) || (WRONG_PROGRESS) || (PROCESS_FAILURE) || (INTERRUPTED) || (UNKNOWN)
        std::pair<Task*, std::list<Behavior*>> desiredDomain;
        desiredDomain.first = (*lastAssignmentIterator)->task;
        desiredDomain.second.clear();
        for(std::list<Behavior*>::iterator behaviorsIterator = (*lastAssignmentIterator)->task->candidateBehaviors.begin();
        behaviorsIterator != (*lastAssignmentIterator)->task->candidateBehaviors.end(); ++behaviorsIterator){
          if((*behaviorsIterator) != (*lastAssignmentIterator)->task->activeBehavior && (*behaviorsIterator) != (*lastAssignmentIterator)->task->inactive){
            desiredDomain.second.push_back(*behaviorsIterator);
          }
        }
        if(desiredDomain.second.size()>0){
          int priority = 0;
          int maxPriority = 0;
          for (std::list<Task*>::iterator tasksIterator = catalog->tasks.begin(); tasksIterator != catalog->tasks.end(); ++tasksIterator){
            if((*tasksIterator)->active && maxPriority < (*tasksIterator)->priority){
              maxPriority = (*tasksIterator)->priority;
            }
          }
          while(priority <= maxPriority && !found){
            found = generateAndExecuteAssignment(priority,desiredDomain);
            priority++;
          }
        }
      }
      if(!found){
        (*lastAssignmentIterator)->task->active = false;
        (*lastAssignmentIterator)->task->priority = 0;
        std::pair<Task*, std::list<Behavior*>> desiredDomain;
        lastAssignment.erase(lastAssignmentIterator);
        lastAssignment.push_back((*lastAssignmentIterator)->task->inactive);
        behavior_coordinator_msgs::TaskStopped task_stopped_msg;
        task_stopped_msg.name = (*lastAssignmentIterator)->task->name;
        task_stopped_msg.termination_cause = message.termination_cause;
        task_stopped_pub.publish(task_stopped_msg);
        desiredDomain.first = (*lastAssignmentIterator)->task;
        desiredDomain.second.clear();
        desiredDomain.second.push_back((*lastAssignmentIterator)->task->inactive);
        Behavior* finishedBehavior = (*lastAssignmentIterator);
      }
      break;
    }
  }
  if(!found){
    for (std::list<Behavior>::iterator behaviorsIterator = catalog->behaviors.begin(); behaviorsIterator != catalog->behaviors.end(); ++behaviorsIterator){
      if ((*behaviorsIterator).name == behavior){
        std::cout<<"[Behavior activation finished]"<<std::endl;
        std::cout<<"  "<<behavior;
        (*behaviorsIterator).active = false;
        if((*behaviorsIterator).task->change_type != behavior_coordinator_msgs::ActivationChange::TIME_OUT_SELF_DEACTIVATION){
          if((*behaviorsIterator).task->requested){
            (*behaviorsIterator).task->change_type = behavior_coordinator_msgs::ActivationChange::REQUESTED_DEACTIVATION;
            std::cout<<" (REQUESTED_DEACTIVATION)"<<std::endl;
          }
          else{
            std::cout<<" (AUTOMATIC_DEACTIVATION)"<<std::endl;
          }
        }
        else{
          std::cout<<" (TIME_OUT_SELF_DEACTIVATION)"<<std::endl;
        }
        break;
      }
    }
  }
}

/*
-----------
 Starting and stopping behaviors
-----------
*/

bool BehaviorCoordinator::activateBehavior(Behavior* behavior, std::string arguments, int priority){
  std::string behavior_name = behavior->name;
  std::cout<<"  Activate "<<behavior_name<<std::endl;
  std::transform(behavior_name.begin(), behavior_name.end(), behavior_name.begin(), ::tolower);
  std::string behavior_path;
  behavior_path =  "/" + robot_namespace + "/" + behavior->package + "/behavior_" + behavior_name + "/activate_behavior";
  ros::ServiceClient behavior_cli = node_handle.serviceClient<behavior_execution_manager_msgs::ActivateBehavior>(behavior_path);
  behavior_execution_manager_msgs::ActivateBehavior activate_behavior_msg;
  activate_behavior_msg.request.arguments = arguments;
  activate_behavior_msg.request.timeout = 1000;
  if (!testing && !behavior_cli.call(activate_behavior_msg)){
    std::cout<<"ERROR ACTIVATING THE BEHAVIOR: "<<behavior->name<<std::endl;
    behavior_coordinator_msgs::TaskStopped task_stopped_msg;
    task_stopped_msg.name = behavior->task->name;
    task_stopped_msg.termination_cause = behavior_execution_manager_msgs::BehaviorActivationFinished::INTERRUPTED;
    task_stopped_pub.publish(task_stopped_msg);
    return false;
  }
  behavior->task->activationTime = std::chrono::system_clock::now();
  behavior_coordinator_msgs::ActivationChange activation_change_msg;
  if(behavior->task->requested == true){
    activation_change_msg.change_type = behavior_coordinator_msgs::ActivationChange::REQUESTED_ACTIVATION;
  }
  else if(behavior->task->defaultActivated == true){
    activation_change_msg.change_type = behavior_coordinator_msgs::ActivationChange::DEFAULT_ACTIVATION;
  }
  else{
    activation_change_msg.change_type = behavior_coordinator_msgs::ActivationChange::AUTOMATIC_ACTIVATION;
  }
  activation_change_msg.task_behavior.behavior = behavior->name;
  activation_change_msg.task_behavior.task_command.name = behavior->task->name;
  activation_change_msg.task_behavior.task_command.parameters = behavior->task->arguments;
  activation_change_msg.task_behavior.task_command.priority = behavior->task->priority;
  activation_change_pub.publish(activation_change_msg);
  for (std::list<Behavior*>::iterator lastAssignmentIterator = lastAssignment.begin(); lastAssignmentIterator != lastAssignment.end(); ++lastAssignmentIterator){
    if ((*lastAssignmentIterator)->task == behavior->task){
      lastAssignment.erase(lastAssignmentIterator);
      break;
    }
  }
  lastAssignment.push_back(behavior);
  behavior->task->active = true;
  behavior->task->activeBehavior = behavior;
  for (std::list<Behavior*>::iterator lastAssignmentIterator = lastAssignment.begin(); lastAssignmentIterator != lastAssignment.end(); ++lastAssignmentIterator){
    if(behavior->name == (*lastAssignmentIterator)->name){
      for(std::list<Task*>::iterator incompatibleTasksIterator = (*lastAssignmentIterator)->task->incompatibleDefaultTasks.begin();
      incompatibleTasksIterator != (*lastAssignmentIterator)->task->incompatibleDefaultTasks.end(); ++incompatibleTasksIterator){
        bool defaultTaskWaiting = false;
        for(std::list<std::pair<Task*, std::chrono::time_point<std::chrono::system_clock>>>::iterator defaultTasksToActivateIterator = defaultTasksToActivate.begin();
        defaultTasksToActivateIterator != defaultTasksToActivate.end(); ++defaultTasksToActivateIterator){
          if(defaultTasksToActivateIterator->first == *incompatibleTasksIterator && !(*incompatibleTasksIterator)->active){
            defaultTaskWaiting = true;
          }
        }
        if(!defaultTaskWaiting){
          std::pair<Task*, std::chrono::time_point<std::chrono::system_clock>> newTask(*incompatibleTasksIterator,std::chrono::system_clock::now());
          defaultTasksToActivate.push_back(newTask);
        }
      }
      for(std::list<std::pair<Task*, std::chrono::time_point<std::chrono::system_clock>>>::iterator defaultTasksToActivateIterator = defaultTasksToActivate.begin();
      defaultTasksToActivateIterator != defaultTasksToActivate.end(); ++defaultTasksToActivateIterator){
        if(defaultTasksToActivateIterator->first == (*lastAssignmentIterator)->task){
          defaultTasksToActivate.erase(defaultTasksToActivateIterator);
          break;
        }
      }
      break;
    }
  }
  for(std::list<Task*>::iterator incompatibleTasksIterator = behavior->task->incompatibleTasks.begin();
  incompatibleTasksIterator != behavior->task->incompatibleTasks.end(); ++incompatibleTasksIterator){
    std::list<std::pair<Task*, std::chrono::time_point<std::chrono::system_clock>>>::iterator defaultTasksToActivateIterator = defaultTasksToActivate.begin();
    while(defaultTasksToActivateIterator != defaultTasksToActivate.end()){
      if(defaultTasksToActivateIterator->first == *incompatibleTasksIterator || defaultTasksToActivateIterator->first == behavior->task){
        defaultTasksToActivateIterator = defaultTasksToActivate.erase(defaultTasksToActivateIterator);
        break;
      }
      else{
        defaultTasksToActivateIterator++;
      }
    }
  }
  if(testing){
    return true;
  }
  return activate_behavior_msg.response.ack;
}

bool BehaviorCoordinator::deactivateBehavior(Behavior behavior){
  std::string behavior_name = behavior.name;
  std::cout<<"  Deactivate "<<behavior_name<<std::endl;
  std::transform(behavior_name.begin(), behavior_name.end(), behavior_name.begin(), ::tolower);
  std::string behavior_path;
  if(behavior.package != "") {
    behavior_path = "/" + robot_namespace + "/" + behavior.package + "/behavior_" + behavior_name + "/deactivate_behavior";
  }
  else{
    behavior_path = "/" + robot_namespace + "/behavior_" + behavior_name + "/deactivate_behavior";
  }
  ros::ServiceClient behavior_cli = node_handle.serviceClient<behavior_execution_manager_msgs::DeactivateBehavior>(behavior_path);
  behavior_execution_manager_msgs::DeactivateBehavior deactivate_msg;
  if(!testing && !behavior_cli.call(deactivate_msg)){
    std::cout<<"ERROR DEACTIVATING THE BEHAVIOR: "<<behavior.name<<std::endl;
    std::cout<<"Behavior: [" + behavior.name + "] is not executing";
    behavior_coordinator_msgs::TaskStopped task_stopped_msg;
    task_stopped_msg.name = behavior.task->name;
    task_stopped_msg.termination_cause = behavior_execution_manager_msgs::BehaviorActivationFinished::INTERRUPTED;
    task_stopped_pub.publish(task_stopped_msg);
    return false;
  }
  if(!testing && !deactivate_msg.response.ack){
    return false;
  }
  if(behavior.task->requested){
    behavior.task->change_type = behavior_coordinator_msgs::ActivationChange::REQUESTED_DEACTIVATION;
  }
  else{
    behavior.task->change_type = behavior_coordinator_msgs::ActivationChange::AUTOMATIC_DEACTIVATION;
  }
  behavior_coordinator_msgs::ActivationChange activation_change_msg;
  activation_change_msg.change_type = behavior.task->change_type;
  activation_change_msg.task_behavior.behavior = behavior.name;
  activation_change_msg.task_behavior.task_command.name = behavior.task->name;
  activation_change_msg.task_behavior.task_command.parameters = behavior.task->arguments;
  activation_change_msg.task_behavior.task_command.priority = behavior.task->priority;
  activation_change_pub.publish(activation_change_msg);
  behavior_coordinator_msgs::TaskStopped task_stopped_msg;
  task_stopped_msg.name = behavior.task->name;
  task_stopped_msg.termination_cause = behavior_execution_manager_msgs::BehaviorActivationFinished::INTERRUPTED;
  task_stopped_pub.publish(task_stopped_msg);
  behavior.task->active = false;
  behavior.task->activeBehavior = behavior.task->inactive;
  for (std::list<Behavior*>::iterator lastAssignmentIterator = lastAssignment.begin(); lastAssignmentIterator != lastAssignment.end(); ++lastAssignmentIterator){
    if(behavior.name == (*lastAssignmentIterator)->name){
      lastAssignment.erase(lastAssignmentIterator);
      lastAssignment.push_back((*lastAssignmentIterator)->task->inactive);
      behavior.task->active = false;
      behavior.task->priority = 0;
      behavior_coordinator_msgs::TaskStopped task_stopped_msg;
      task_stopped_msg.name = (*lastAssignmentIterator)->task->name;
      task_stopped_msg.termination_cause = behavior_execution_manager_msgs::BehaviorActivationFinished::INTERRUPTED;
      task_stopped_pub.publish(task_stopped_msg);
      for(std::list<Task*>::iterator incompatibleTasksIterator = (*lastAssignmentIterator)->task->incompatibleDefaultTasks.begin();
      incompatibleTasksIterator != (*lastAssignmentIterator)->task->incompatibleDefaultTasks.end(); ++incompatibleTasksIterator){
        bool defaultTaskWaiting = false;
        for(std::list<std::pair<Task*, std::chrono::time_point<std::chrono::system_clock>>>::iterator defaultTasksToActivateIterator = defaultTasksToActivate.begin();
        defaultTasksToActivateIterator != defaultTasksToActivate.end(); ++defaultTasksToActivateIterator){
          if(defaultTasksToActivateIterator->first == *incompatibleTasksIterator && !(*incompatibleTasksIterator)->active){
            defaultTaskWaiting = true;
          }
        }
        if(!defaultTaskWaiting){
          std::pair<Task*, std::chrono::time_point<std::chrono::system_clock>> newTask(*incompatibleTasksIterator,std::chrono::system_clock::now());
          defaultTasksToActivate.push_back(newTask);
        }
      }
      for(std::list<std::pair<Task*, std::chrono::time_point<std::chrono::system_clock>>>::iterator defaultTasksToActivateIterator = defaultTasksToActivate.begin();
      defaultTasksToActivateIterator != defaultTasksToActivate.end(); ++defaultTasksToActivateIterator){
        if(defaultTasksToActivateIterator->first == (*lastAssignmentIterator)->task){
          defaultTasksToActivate.erase(defaultTasksToActivateIterator);
          break;
        }
      }
      break;
    }
  }
  return true;
}

/*
-----------
 Behavior_coordinator functions
-----------
*/

bool BehaviorCoordinator::generateAndExecuteAssignment(int priority, std::pair<Task*, std::list<Behavior*>> desiredDomain){
  if(generateBestAssignment(priority, desiredDomain)){
    //printBestBehaviorAssignment();
    if(executeAssignment()){
      retrieveAssignment(resetActivationList);
      return true;
    }
    else{
      std::cout<<"Unexpected activation error"<<std::endl;
      std::cout<<"Reseting activations"<<std::endl;
      resetActivations();
      return false;
    }
  }
  else{
    return false;
  }
}


bool BehaviorCoordinator::generateBestAssignment(int priority, std::pair<Task*, std::list<Behavior*>> desiredDomain){
  //loadTasksConstraints();
  solutions = 0;

  auto startInitializeSearch = std::chrono::system_clock::now();
  initializeSearch(priority, desiredDomain);
  int InitializeTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()-startInitializeSearch).count();
/*
  std::cout<<"[Result of initializeSearch()]"<<std::endl;
  catalog->printInitialDomain();
*/
  unsigned long long int searchSpaceSize = 1;
  unsigned long long int CatalogSearchSpaceSize = 1;
  bool possibleOverflow = true;
  for (std::list<Task*>::iterator tasksIterator = catalog->tasks.begin(); tasksIterator != catalog->tasks.end(); ++tasksIterator){
    searchSpaceSize *= (*tasksIterator)->initialDomain.size();
    CatalogSearchSpaceSize *= (*tasksIterator)->candidateBehaviors.size();
    if((*tasksIterator)->initialDomain.size()==0){
      possibleOverflow = false;
    }
  }
  if(searchSpaceSize > 0){
    std::cout << "  Search space: " << searchSpaceSize << std::endl;
    //std::cout << "  Catalog Search space: " << CatalogSearchSpaceSize << std::endl;
  }
  else if(possibleOverflow){
    std::cout << "  Search space: ";
    for (std::list<Task*>::iterator tasksIterator = catalog->tasks.begin(); tasksIterator != catalog->tasks.end(); ++tasksIterator){
      if(tasksIterator != catalog->tasks.begin()){
        std::cout << "*";
      }
      std::cout <<(*tasksIterator)->initialDomain.size();
    }
    std::cout << std::endl;
  }
  int bestPerformance = -1;
  double bestValueObjectiveFunctionMaxEfficacy = -1;
  double bestEvaluateObjectiveFunction1MinChanges = 100;
  double bestValueObjectiveFunction2 = -1;
  bool assignmentFound =false;
  bool otherPossibleAssignments = true;
  //resetDomains();
  while (bestPerformance<MAXIMUM_PERFORMANCE && numberOfNoGoodConstraints<MAXIMUM_NO_GOOD_CONSTRAINTS && !exceededMaximumSearchTime() && otherPossibleAssignments){
    if (!generateAssignment() && !checkAssignmentFound() || (searchSpaceSize > 0 && solutions>searchSpaceSize)){
      otherPossibleAssignments = false;
    }
    else{
      if(checkAssignmentFound()){
        assignmentFound = true;
        double valueObjectiveFunction1MinChanges;
        double valueObjectiveFunctionMaxEfficacy = evaluateObjectiveFunction1MaxEfficacy();
        //valueObjectiveFunction1MinChanges = evaluateObjectiveFunction1MinActive();
        valueObjectiveFunction1MinChanges = evaluateObjectiveFunction1MinChanges();
        double valueObjectiveFunction2 = evaluateObjectiveFunction2();
        if(valueObjectiveFunction1MinChanges < bestEvaluateObjectiveFunction1MinChanges){
          retrieveAssignment(bestBehaviorAssignment); // Delete the previous list
          bestValueObjectiveFunctionMaxEfficacy = valueObjectiveFunctionMaxEfficacy;
          bestValueObjectiveFunction2 = valueObjectiveFunction2;
          bestEvaluateObjectiveFunction1MinChanges = valueObjectiveFunction1MinChanges;
        }
        else if(valueObjectiveFunction1MinChanges == bestEvaluateObjectiveFunction1MinChanges){
          if(valueObjectiveFunctionMaxEfficacy > bestValueObjectiveFunctionMaxEfficacy){
            retrieveAssignment(bestBehaviorAssignment); // Delete the previous list
            bestValueObjectiveFunctionMaxEfficacy = valueObjectiveFunctionMaxEfficacy;
            bestValueObjectiveFunction2 = valueObjectiveFunction2;
            bestEvaluateObjectiveFunction1MinChanges = valueObjectiveFunction1MinChanges;
          }
          else if (valueObjectiveFunctionMaxEfficacy == bestValueObjectiveFunctionMaxEfficacy && valueObjectiveFunction2 > bestValueObjectiveFunction2){
            retrieveAssignment(bestBehaviorAssignment); // Delete the previous list
            bestValueObjectiveFunctionMaxEfficacy = valueObjectiveFunctionMaxEfficacy;
            bestValueObjectiveFunction2 = valueObjectiveFunction2;
            bestEvaluateObjectiveFunction1MinChanges = valueObjectiveFunction1MinChanges;
          }
        }
      }
    }
  }
  std::cout << "  Analyzed solutions: " << solutions << " combinations" << std::endl;
  std::cout << "  Initialize Search time: " << InitializeTime << " μs" << std::endl;
  std::cout << "  Search time: " << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()-startSearchTime).count() << " μs" << std::endl;

  return assignmentFound;
}

bool BehaviorCoordinator::generateAssignment(){
  resetDomains();
  loadTasksConstraints();
  //std::cout<<arcs.size()<<" , "<<agendaG.size()<<std::endl;
  if (generateConsistentAssignment()){
    if (testPerformanceConstraints()){
      std::list<std::pair<Task, std::list<Behavior*>>> assignment;
      retrieveAssignment(assignment);
      /*
      std::cout<<"[Result of retrieveAssignment()]"<<std::endl;
      catalog->printValues();
      */
      addNoGood(assignment);
      return true;
    }
    return false;
  }
  return false;
}

bool BehaviorCoordinator::generateConsistentAssignment(){
  if (!propagateConstraints() || exceededMaximumSearchTime()){
    return false;
  }
  if (checkAssignmentFound()){
    return true;
  }
  std::list<Task*>::iterator tasksIterator = catalog->tasks.begin();
  std::list<std::list<Behavior*>> savedDomains = retrieveDomains();
  bool valueFound = false;
  while (tasksIterator != catalog->tasks.end()){
    if((*tasksIterator)->domain.size()>1){
      valueFound=true;
      break;
    }
    tasksIterator++;
  }
  if(valueFound){
    std::list<Behavior*> searchDomain = (*tasksIterator)->domain;
    while (searchDomain.size()>0){
      std::chrono::time_point<std::chrono::system_clock> startvalue;
      Behavior* value = selectValueToExplore(*tasksIterator, searchDomain);
      for (std::list<Behavior*>::iterator domainIterator_delete = searchDomain.begin(); domainIterator_delete != searchDomain.end(); ++domainIterator_delete){
        if ((*domainIterator_delete) == value){
          domainIterator_delete= searchDomain.erase(domainIterator_delete);
          break;
        }
      }
      agendaG.clear();
      for (std::list<Constraint*>::iterator constraintsit = value->task->secondaryInConstraints.begin(); constraintsit != value->task->secondaryInConstraints.end(); ++constraintsit){
        agendaG.push_back(*constraintsit);
        (*constraintsit)->queued = true;
      }
      for (std::list<Constraint*>::iterator constraintsit = noGoodconstraints.begin(); constraintsit != noGoodconstraints.end(); ++constraintsit){
        agendaG.push_back(*constraintsit);
        (*constraintsit)->queued = true;
      }
      (*tasksIterator)->domain.clear();
      (*tasksIterator)->domain.push_back(value);
      if(generateConsistentAssignment()){
        return true;
      }
      else{
        if(searchDomain.size()>0){
          restoreDomains(savedDomains);
        }
      }
    }
  }
  bool change = false;
  return change;
}


bool BehaviorCoordinator::propagateConstraints(){
  //std::list<Constraint*> agenda(agendaG);
  for(std::list<Constraint*>::iterator agendaIterator = agendaG.begin(); agendaIterator != agendaG.end();agendaIterator = agendaG.erase(agendaIterator)){
    Task* temp = (*agendaIterator)->revise();
    if(temp!=NULL){
      for (std::list<Constraint*>::iterator arcsIterator = temp->secondaryInConstraints.begin(); arcsIterator != temp->secondaryInConstraints.end(); ++arcsIterator){
        //if(!(std::find(agenda.begin(), agenda.end(), (*arcsIterator)) != agenda.end())){ //Requires
        if(!(*arcsIterator)->queued){
          (*arcsIterator)->queued = true;
          agendaG.push_back((*arcsIterator));
        }
      }
    }
  }
  for (std::list<Task*>::iterator tasksIterator = catalog->tasks.begin(); tasksIterator != catalog->tasks.end(); ++tasksIterator){
    if((*tasksIterator)->domain.size()==0){
      return false;
    }
  }
  return true;
}

void BehaviorCoordinator::initializeSearch(int priority, std::pair<Task*, std::list<Behavior*>> desiredDomain){
  for (std::list<Task*>::iterator tasksIterator = catalog->tasks.begin(); tasksIterator != catalog->tasks.end(); ++tasksIterator){
    
    (*tasksIterator)->resetInitialDomain(testing);

    if((*tasksIterator)->active){
      if((*tasksIterator)->priority > priority){
        (*tasksIterator)->setInitialDomainWithSingleBehavior();
        if(priority > (*tasksIterator)->priority && !(*tasksIterator)->automaticActivation){
          (*tasksIterator)->initialDomain.push_back((*tasksIterator)->inactive);
        }
      }
    }
    else{
      if(!(*tasksIterator)->automaticActivation){
        (*tasksIterator)->setInitialDomainWithSingleInactiveValue();
      }
    }
  }
  desiredDomain.first->initialDomain.clear();
  for (std::list<Behavior*>::iterator behaviorsIterator = desiredDomain.second.begin();
  behaviorsIterator != desiredDomain.second.end(); ++behaviorsIterator){
    if(testing || (*behaviorsIterator) == (*behaviorsIterator)->task->inactive || (*behaviorsIterator)->checkSituation()){
      desiredDomain.first->initialDomain.push_back(*behaviorsIterator);
    }
  }
/*
  catalog->printDebugInfo();
*/
  removeNoGoodConstraints();
  initializeSearchTime();
}

bool BehaviorCoordinator::executeAssignment(){
  std::cout<<"[Behaviors activations]"<<std::endl;
  std::list<std::pair<Task, std::list<Behavior*>>> activationList;
  std::list<std::pair<Task, std::list<Behavior*>>>::iterator reducedAssignmentIterator = bestBehaviorAssignment.begin();
  while(reducedAssignmentIterator != bestBehaviorAssignment.end()){
    bool repeated_behavior = false;
    bool task_active = false;
    Behavior* found;
    for (std::list<Behavior*>::iterator lastAssignmentIterator = lastAssignment.begin(); lastAssignmentIterator != lastAssignment.end(); ++lastAssignmentIterator){
      if ((*lastAssignmentIterator)->task == reducedAssignmentIterator->second.front()->task){
        if ((*lastAssignmentIterator) != reducedAssignmentIterator->second.front()->task->inactive){
          found = (*lastAssignmentIterator);
          task_active = true;
        }
        if ((*lastAssignmentIterator) == reducedAssignmentIterator->second.front()){
          repeated_behavior = true;
        }
      }
    }
    if(!repeated_behavior){
      if(reducedAssignmentIterator->second.front() != reducedAssignmentIterator->first.inactive){
        if(task_active){
          if(!deactivateBehavior(*reducedAssignmentIterator->second.front()->task->activeBehavior) && !deactivateBehavior(*reducedAssignmentIterator->second.front()->task->activeBehavior)){
              return false;
            }
        }
        activationList.push_back(*reducedAssignmentIterator);
      }
      else if(task_active){
        deactivateBehavior(*found);
      }
    }
    else{
      reducedAssignmentIterator->second.front()->task->activationTime = std::chrono::system_clock::now();
    }
    reducedAssignmentIterator=bestBehaviorAssignment.erase(reducedAssignmentIterator);
  }
  for(std::list<std::pair<Task, std::list<Behavior*>>>::iterator activationListIterator = activationList.begin(); activationListIterator != activationList.end();++activationListIterator){
    if(activateBehavior(activationListIterator->second.front(),
    activationListIterator->second.front()->task->arguments, activationListIterator->second.front()->task->priority)){
      activationListIterator->second.front()->task->activationTime = std::chrono::system_clock::now();
    }
    else{
      return false;
    }
  }
  return true;
}

std::list<std::pair<Task, std::list<Behavior*>>> BehaviorCoordinator::generateAssignmentExplainingViolation(Constraint* constraint){
  std::list<std::pair<Task, std::list<Behavior*>>> explanation;
  std::list<Behavior**> result = calculateTaskDependencies(constraint->getRequiringBehavior()->task);
  Behavior* elvalue = constraint->getRequiringBehavior()->task->domain.front();
  result.push_front(&elvalue);
  for(std::list<Behavior**>::iterator resultIterator = result.begin(); resultIterator != result.end(); ++resultIterator){
    std::pair<Task, std::list<Behavior*>> newValue;
    newValue.first = *(**resultIterator)->task;
    std::list<Behavior*> lista;
    lista.push_back(*(*resultIterator));
    newValue.second = lista;
    explanation.push_back(newValue);
  }
  return explanation;
}

bool BehaviorCoordinator::testPerformanceConstraints(){
  calculateTasksPerformance();
  for (std::list<Constraint*>::iterator constraintsit = catalog->constraints.begin(); constraintsit != catalog->constraints.end(); ++constraintsit){
    if((*constraintsit)->getConstraintType()==1 && !(*constraintsit)->checkConstraint()){
      std::list<std::pair<Task, std::list<Behavior*>>> assignmentExplainingViolation = generateAssignmentExplainingViolation(*constraintsit);
      addNoGood(assignmentExplainingViolation); //This no-good is added to avoid generating an assignment that fails the performance test
      return false;
    }
  }
  return true;
}

double BehaviorCoordinator::evaluateObjectiveFunction1MinActive(){ //This function minimizes the number of active tasks for the maximum efficacy
  double result = 1;
  for (std::list<Task*>::iterator tasksIterator = catalog->tasks.begin(); tasksIterator != catalog->tasks.end(); ++tasksIterator){
    if(!(*tasksIterator)->domain.front()->efficacy == 0){
      result = result * (*tasksIterator)->domain.front()->efficacy;
    }
  }
  return result;
}

double BehaviorCoordinator::evaluateObjectiveFunction1MinChanges(){ //This function minimizes changes relative to the current situation
  int changes = 0;
  for (std::list<Task*>::iterator tasksIterator = catalog->tasks.begin(); tasksIterator != catalog->tasks.end(); ++tasksIterator){
    if((*tasksIterator)->requested && (*tasksIterator)->active){
      for (std::list<Behavior*>::iterator lastAssignmentIterator = lastAssignment.begin();
      lastAssignmentIterator != lastAssignment.end(); ++lastAssignmentIterator){
        if(*tasksIterator == (*lastAssignmentIterator)->task && (*tasksIterator)->domain.front() != (*lastAssignmentIterator)
        && (*tasksIterator)->domain.front() == (*tasksIterator)->inactive){
          changes++;
          break;
        }
      }
    }
  }
  return changes;
}

double BehaviorCoordinator::evaluateObjectiveFunction1MaxEfficacy(){ //This function maximizes global efficacy
  double result = 1;
  bool nonInactiveValueExists = false;
  for (std::list<Task*>::iterator tasksIterator = catalog->tasks.begin(); tasksIterator != catalog->tasks.end(); ++tasksIterator){
    if(!(*tasksIterator)->domain.front()->efficacy == 0){
      result = result * (*tasksIterator)->domain.front()->efficacy;
      nonInactiveValueExists = true;
    }
  }
  if(!nonInactiveValueExists){
    result = 0;
  }
  return result;
}

double BehaviorCoordinator::evaluateObjectiveFunction2(){
  double t = 0, a = 0;
  for (std::list<Task*>::iterator tasksIterator = catalog->tasks.begin(); tasksIterator != catalog->tasks.end(); ++tasksIterator){
    t++;
    if((*tasksIterator)->domain.size()==1 && (*tasksIterator)->domain.front()->name != "inactive"){
      a++;
    }
  }
  return (t-a)/t;
}

bool BehaviorCoordinator::exceededMaximumSearchTime(){
  if(MAXIMUM_SEARCH_TIME>std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-startSearchTime).count()){
    return false;
  }
  return true;
}

int BehaviorCoordinator::evaluateGlobalPerformance(){
  int resultPerformance = 0;
  int number = 0;
  for (std::list<Task*>::iterator tasksIterator = catalog->tasks.begin(); tasksIterator != catalog->tasks.end(); ++tasksIterator){
    if((*tasksIterator)->performance != 0 && (*tasksIterator)->performance < resultPerformance){
      resultPerformance = (*tasksIterator)->performance;
    }
  }
  return resultPerformance;
}

bool BehaviorCoordinator::addNoGood(std::list<std::pair<Task, std::list<Behavior*>>> assignment){
  solutions++;
  bool newNoGoodAdded = false;
  std::list<std::pair<Task, std::list<Behavior*>>> reducedAssignment(assignment);
  for (std::list<std::pair<Task, std::list<Behavior*>>>::iterator reducedAssignmentIterator = reducedAssignment.begin();
  reducedAssignmentIterator != reducedAssignment.end(); ++reducedAssignmentIterator){
    if((*reducedAssignmentIterator).first.candidateBehaviors.size() == 1){
      reducedAssignmentIterator = reducedAssignment.erase(reducedAssignmentIterator);
    }
  }
  if(reducedAssignment.size() == 0){
    std::cout<<"Unexpected no-good constraint with empty assignment"<<std::endl;
    return true;
  }
  std::list<Behavior*> newBehaviors;
  for (std::list<std::pair<Task, std::list<Behavior*>>>::iterator reducedAssignmentIterator = reducedAssignment.begin();
  reducedAssignmentIterator != reducedAssignment.end(); ++reducedAssignmentIterator){
    if((*reducedAssignmentIterator).first.initialDomain.size() > 1){
      newNoGoodAdded = true;
      newBehaviors.push_back((*reducedAssignmentIterator).second.front());
    }
  }
  if(!newBehaviors.size()>0){
    return false;
  }
  Task* newTask = newBehaviors.front()->task;
  Constraint* noGood = new NoGoodConstraint(newTask, newBehaviors);
  noGood->mainTask = newTask;
  /*
  std::cout<<"New NoGood Constraint included: ";
  noGood->printConstraint();
  */
  numberOfNoGoodConstraints++;
  catalog->constraints.push_back(noGood);
  noGoodconstraints.push_back(noGood);
  agendaG.push_back(noGood);
  return newNoGoodAdded;
}

void BehaviorCoordinator::retrieveAssignment(std::list<std::pair<Task, std::list<Behavior*>>>& assignment){
  assignment.clear();
  for (std::list<Task*>::iterator tasksIterator = catalog->tasks.begin(); tasksIterator != catalog->tasks.end(); ++tasksIterator){
    std::pair<Task, std::list<Behavior*>> newPair;
    newPair.first=**tasksIterator;
    newPair.second=(*tasksIterator)->domain;
    assignment.push_back(newPair);
  }
}

void BehaviorCoordinator::calculateTasksPerformance(){
  for (std::list<Task*>::iterator tasksIterator = catalog->tasks.begin(); tasksIterator != catalog->tasks.end(); ++tasksIterator){
    double maxEfficacy = -1;
    for (std::list<Behavior*>::iterator domainIterator = (*tasksIterator)->domain.begin(); domainIterator != (*tasksIterator)->domain.end(); ++domainIterator){
      if((*domainIterator)->efficacy > maxEfficacy){
        maxEfficacy = (*domainIterator)->efficacy;
      }
    }
    std::list<Behavior**> result = calculateTaskDependencies(*tasksIterator);
    for(std::list<Behavior**>::iterator resultIterator = result.begin(); resultIterator != result.end(); ++resultIterator){
      maxEfficacy = maxEfficacy * (**resultIterator)->efficacy;
    }
    (*tasksIterator)->performance = maxEfficacy*100;
  }
}

void BehaviorCoordinator::initializeSearchTime(){
  startSearchTime = std::chrono::high_resolution_clock::now();
}

void BehaviorCoordinator::removeNoGoodConstraints(){
  numberOfNoGoodConstraints = 0;
  noGoodconstraints.clear();
  std::list<Constraint*>::iterator constraintsIterator = catalog->constraints.begin();
  while(constraintsIterator != catalog->constraints.end()){
    if((*constraintsIterator)->getConstraintType()==2){
      constraintsIterator = catalog->constraints.erase(constraintsIterator);
    }
    else{
      constraintsIterator++;
    }
  }
}

Behavior** BehaviorCoordinator::getMaxEfficacyValue(Task* taskIterator){
  Behavior** maxEfficacyValue = NULL;
  for(std::list<Behavior*>::iterator domainIterator = taskIterator->domain.begin(); domainIterator != taskIterator->domain.end(); ++domainIterator){
    if(maxEfficacyValue==NULL || (*domainIterator)->efficacy > (*maxEfficacyValue)->efficacy){
      maxEfficacyValue = &*domainIterator;
    }
  }
  return maxEfficacyValue;
}

std::list<Behavior**> BehaviorCoordinator::calculateTaskDependencies(Task* taskIterator){
  std::list<Behavior**> newList;
  Behavior** maxEfficacyValue = getMaxEfficacyValue(taskIterator);
  for (std::list<Constraint*>::iterator constraintsIterator = catalog->constraints.begin(); constraintsIterator != catalog->constraints.end(); ++constraintsIterator){
    if((*constraintsIterator)->getConstraintType()==1 && taskIterator == (*constraintsIterator)->getRequiringBehavior()->task
    && (*constraintsIterator)->getRequiringBehavior() == (*maxEfficacyValue)){
      bool alreadyListed = false;
      for(std::list<Behavior**>::iterator newListIterator = newList.begin(); newListIterator != newList.end(); ++newListIterator){
        if(*newListIterator == getMaxEfficacyValue((*constraintsIterator)->getRequiredTask())){
          alreadyListed = true;
        }
      }
      if(!alreadyListed){
        newList.push_front(getMaxEfficacyValue((*constraintsIterator)->getRequiredTask()));
        std::list<Behavior**> result = calculateTaskDependencies((*constraintsIterator)->getRequiredTask());
        for(std::list<Behavior**>::iterator resultIterator = result.begin(); resultIterator != result.end(); ++resultIterator){
          alreadyListed = false;
          for(std::list<Behavior**>::iterator newListIterator = newList.begin(); newListIterator != newList.end(); ++newListIterator){
            if(newListIterator == resultIterator){
              alreadyListed = true;
            }
          }
          if(!alreadyListed){
            newList.push_back(*resultIterator);
          }
        }
      }
    }
  }
  return newList;
}

void BehaviorCoordinator::restoreDomains(std::list<std::list<Behavior*>> savedDomains){
  std::list<std::list<Behavior*>>::iterator savedDomainsit = savedDomains.begin();
  while( savedDomainsit != savedDomains.end()){
    (*savedDomainsit->begin())->task->domain = *savedDomainsit;
    savedDomainsit++;
  }
}

void BehaviorCoordinator::resetActivations(){
  bestBehaviorAssignment = resetActivationList;
  executeAssignment();
}

bool BehaviorCoordinator::checkAssignmentFound(){
  for (std::list<Task*>::iterator tasksIterator = catalog->tasks.begin(); tasksIterator != catalog->tasks.end(); ++tasksIterator){
    if((*tasksIterator)->domain.size() != 1){
      return false;
    }
    (*tasksIterator)->assignment = (*(*tasksIterator)->domain.begin());
  }
  return true;
}

Behavior* BehaviorCoordinator::selectValueToExplore(Task* tasksIterator,std::list<Behavior*> searchDomain){
  if ((*tasksIterator).active){
    for (std::list<Behavior*>::iterator domainIterator = searchDomain.begin(); domainIterator != searchDomain.end(); ++domainIterator){
      if ((*tasksIterator).activeBehavior != NULL && (*domainIterator) == (*tasksIterator).activeBehavior){
        return (*domainIterator);
      }
    }
    for (std::list<Behavior*>::iterator domainIterator = searchDomain.begin(); domainIterator != searchDomain.end(); ++domainIterator){
      if ((*domainIterator)->name != "inactive"){
        return (*domainIterator);
      }
    }
    for (std::list<Behavior*>::iterator domainIterator = searchDomain.begin(); domainIterator != searchDomain.end(); ++domainIterator){
      if ((*domainIterator)->name == "inactive"){
        return (*domainIterator);
      }
    }
  }
  else{
    for (std::list<Behavior*>::iterator domainIterator = searchDomain.begin(); domainIterator != searchDomain.end(); ++domainIterator){
      if ((*domainIterator)->name == "inactive"){
        return (*domainIterator);
      }
    }
    int maxEfficacy = -1;
    Behavior* bestValue;
    bool found = false;
    for (std::list<Behavior*>::iterator domainIterator = searchDomain.begin(); domainIterator != searchDomain.end(); ++domainIterator){
      if ((*domainIterator)->efficacy > maxEfficacy){
        maxEfficacy = (*domainIterator)->efficacy;
        bestValue = (*domainIterator);
        found = true;
      }
    }
    return bestValue;
  }
}

std::list<std::list<Behavior*>> BehaviorCoordinator::retrieveDomains(){
  std::list<std::list<Behavior*>> savedDomains;
  for (std::list<Task*>::iterator tasksIterator = catalog->tasks.begin(); tasksIterator != catalog->tasks.end(); ++tasksIterator){
    savedDomains.push_back((*tasksIterator)->domain);
  }
  return savedDomains;
}

void BehaviorCoordinator::resetDomains(){
  for (std::list<Task*>::iterator tasksIterator = catalog->tasks.begin(); tasksIterator != catalog->tasks.end(); ++tasksIterator){
    (*tasksIterator)->domain = (*tasksIterator)->initialDomain;
  }
}

void BehaviorCoordinator::printBestBehaviorAssignment(){
  for(std::list<std::pair<Task, std::list<Behavior*>>>::iterator reducedAssignmentIterator = bestBehaviorAssignment.begin();
  reducedAssignmentIterator != bestBehaviorAssignment.end(); ++reducedAssignmentIterator){
    std::cout<<"  "<<reducedAssignmentIterator->first.name<<"={";
    for (std::list<Behavior*>::iterator valuesIterator = reducedAssignmentIterator->second.begin();
    valuesIterator != reducedAssignmentIterator->second.end(); ++valuesIterator){
      if(valuesIterator != reducedAssignmentIterator->second.begin()){
        std::cout<<",";
      }
      std::cout<<(*valuesIterator)->name;
    }
    std::cout<<"}"<<std::endl;
  }
}

void BehaviorCoordinator::loadTasksConstraints(){
  agendaG.clear();
  for (std::list<Constraint*>::iterator constraintsit = catalog->constraints.begin(); constraintsit != catalog->constraints.end(); ++constraintsit){
    agendaG.push_back(*constraintsit);
    (*constraintsit)->queued = true;
  }
}

void BehaviorCoordinator::initializeIncompatibleTasks(){
  for (std::list<Constraint*>::iterator constraintsit = catalog->constraints.begin(); constraintsit != catalog->constraints.end(); ++constraintsit){
    if((*constraintsit)->getConstraintType()==0){
      for (std::list<Task*>::iterator tasksIterator = catalog->tasks.begin(); tasksIterator != catalog->tasks.end(); ++tasksIterator){
        if((*constraintsit)->checkTaskInConstraint(*tasksIterator)){
          for (std::list<Task*>::iterator tasksIterator2 = catalog->tasks.begin(); tasksIterator2 != catalog->tasks.end(); ++tasksIterator2){
            if((*constraintsit)->checkTaskInConstraint(*tasksIterator2)
            && !(std::find((*tasksIterator)->incompatibleTasks.begin(), (*tasksIterator)->incompatibleTasks.end(), *tasksIterator2) != (*tasksIterator)->incompatibleTasks.end())){
              (*tasksIterator)->incompatibleTasks.push_back(*tasksIterator2);
              if((*tasksIterator2)->isDefault && !((*tasksIterator2)->candidateBehaviors.size()==1 &&
              (*tasksIterator2)->candidateBehaviors.front() == (*tasksIterator2)->inactive)){
                (*tasksIterator)->incompatibleDefaultTasks.push_back(*tasksIterator2);
              }
            }
          }
        }
      }
    }
  }
}
