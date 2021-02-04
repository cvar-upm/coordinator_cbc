/*!********************************************************************************
 * \brief     This file implements the constraint class
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

#include "../include/constraint.h"
#include <pluginlib/class_list_macros.h>

Constraint::Constraint(){
}

Constraint::~Constraint(){
}

//INCOMPATIBILITY

IncompatibilityConstraint::IncompatibilityConstraint(std::list<Task*> arg_incompatibleTasks){
  incompatibleTasks = arg_incompatibleTasks;
}

std::string IncompatibilityConstraint::getConstraint_str(){
  std::string result = "incompatible(";
  for (std::list<Task*>::iterator tasksit = this->incompatibleTasks.begin(); tasksit != this->incompatibleTasks.end(); ++tasksit){
    if(tasksit != this->incompatibleTasks.begin()){
      result += ",";
    }
    result += (*tasksit)->name;
  }
  result +=")";
  return result;
}

void IncompatibilityConstraint::printConstraint(){
  std::cout<<"  incompatible(";
  for (std::list<Task*>::iterator tasksit = this->incompatibleTasks.begin(); tasksit != this->incompatibleTasks.end(); ++tasksit){
    if(tasksit != this->incompatibleTasks.begin()){
      std::cout<<",";
    }
    std::cout<<(*tasksit)->name;
  }
  std::cout<<")"<<std::endl;
}

bool IncompatibilityConstraint::checkTaskInConstraint(Task* task){
  for (std::list<Task*>::iterator tasksIterator = incompatibleTasks.begin(); tasksIterator != incompatibleTasks.end(); ++tasksIterator){
    if(*tasksIterator == task){
      return true;
    }
  }
  return false;
}

Constraint* IncompatibilityConstraint::turnConstraint(){
  std::list<Task*> invertedIncompatibleTasks;
  invertedIncompatibleTasks.push_back(incompatibleTasks.back());
  invertedIncompatibleTasks.push_back(incompatibleTasks.front());
  Constraint* result= new IncompatibilityConstraint(invertedIncompatibleTasks);
  return result;
}

bool IncompatibilityConstraint::IncompatibilityConstraint::revise(){
  bool new_change = false;
  bool inactiveTask = false;
  //Check if the second task only has inactive on its domain
  for (std::list<Behavior*>::iterator domainit = incompatibleTasks.back()->domain.begin();
  domainit != incompatibleTasks.back()->domain.end(); ++domainit){
    if((*domainit)->name == "inactive"){
      inactiveTask = true;
      break;
    }
  }
  if(!inactiveTask){
    std::list<Behavior*>::iterator domainit = incompatibleTasks.front()->domain.begin();
    while(domainit != incompatibleTasks.front()->domain.end()){
      if((*domainit)->name != "inactive"){
        domainit=incompatibleTasks.front()->domain.erase(domainit);
        new_change = true;
      }
      else{
        domainit++;
      }
    }
  }
  return new_change;
}

std::string* IncompatibilityConstraint::getFirst(){
  return &incompatibleTasks.front()->name;
}

std::string* IncompatibilityConstraint::getSecond(){
  return &incompatibleTasks.back()->name;
}

int IncompatibilityConstraint::getConstraintType(){
  return 0;
}

///////////////////REQUIREMENT

RequirementConstraint::RequirementConstraint(Behavior* arg_behavior, Task* arg_task, double arg_performance){
  requiringBehavior = arg_behavior;
  requiredTask = arg_task;
  requiredPerformance = arg_performance;
}

std::string RequirementConstraint::getConstraint_str(){
  std::string result = "requires("+this->requiringBehavior->name+"("+requiringBehavior->task->name+")"
           +","+this->requiredTask->name+","+std::to_string(this->requiredPerformance)+")";
  return result;
}

void RequirementConstraint::printConstraint(){
  std::cout<<"  requires("<<this->requiringBehavior->name<<"("<<requiringBehavior->task->name<<")"
           <<","<<this->requiredTask->name<<","<<this->requiredPerformance<<")"<<std::endl;
}

Constraint* RequirementConstraint::turnConstraint(){
  return NULL;
}

bool RequirementConstraint::checkTaskInConstraint(Task* task){
  if(requiringBehavior->task == task || requiredTask == task){
    return true;
  }
  return false;
}

double RequirementConstraint::getMaxEfficacyValue(Task* taskIterator){
  double maxEfficacyValue = 0;
  for(std::list<Behavior*>::iterator domainIterator = taskIterator->domain.begin();
  domainIterator != taskIterator->domain.end(); ++domainIterator){
    if((*domainIterator)->efficacy > maxEfficacyValue){
      maxEfficacyValue = (*domainIterator)->efficacy;
    }
  }
  return maxEfficacyValue;
}

bool RequirementConstraint::checkConstraint(){
  bool new_change = false;
  for (std::list<Behavior*>::iterator domainit = requiringBehavior->task->domain.begin();
  domainit != requiringBehavior->task->domain.end(); ++domainit){
    if((*domainit) == requiringBehavior){
      if(requiredTask->performance < requiredPerformance*100){
        return false;
      }
    }
  }
  return true;
}

bool RequirementConstraint::revise(){
  bool new_change = false;
  if(requiringBehavior->task->domain.size() == 1 && requiringBehavior->task->domain.front() == requiringBehavior){
    for (std::list<Behavior*>::iterator domainit = requiredTask->domain.begin();
    domainit != requiredTask->domain.end(); ++domainit){
      if((*domainit)->efficacy < requiredPerformance || (*domainit) == requiredTask->inactive){
        domainit=requiredTask->domain.erase(domainit);
        new_change = true;
        break;
      }
    }
  }
  if(requiredPerformance != 0 && getMaxEfficacyValue(requiredTask) < requiredPerformance){
    for(std::list<Behavior*>::iterator domainit = requiringBehavior->task->domain.begin();
    domainit != requiringBehavior->task->domain.end();domainit++){
      if((*domainit) == requiringBehavior){
        requiringBehavior->task->domain.erase(domainit);
        return true;
      }
    }
  }
  return new_change;
}

std::string* RequirementConstraint::getFirst(){
  return &requiringBehavior->name;
}

std::string* RequirementConstraint::getSecond(){
  return &requiredTask->name;
}

int RequirementConstraint::getConstraintType(){
  return 1;
}

Behavior* RequirementConstraint::getRequiringBehavior(){
  return requiringBehavior;
}

Task* RequirementConstraint::getRequiredTask(){
  return requiredTask;
}

RequirementConstraint* RequirementConstraint::generateAssignmentExplainingViolation(){
  return this;
}

///////////////////NOGOOD

NoGoodConstraint::NoGoodConstraint(Task* arg_task, std::list<Behavior*> arg_behavior){
  assignmement.first = arg_task;
  assignmement.second = arg_behavior;
}

NoGoodConstraint::~NoGoodConstraint(){
}

std::string NoGoodConstraint::getConstraint_str(){
  std::string result = "noGood(";
  for (std::list<Behavior*>::iterator behaviorsIterator = assignmement.second.begin();
  behaviorsIterator != assignmement.second.end(); ++behaviorsIterator){
    if((*behaviorsIterator) != (*assignmement.second.begin())){
      result+=",";
    }
    result+=(*behaviorsIterator)->task->name+"="+(*behaviorsIterator)->name;
  }
  result+=")";
  return result;
}

void NoGoodConstraint::printConstraint(){
  std::cout<<"  noGood(";
  for (std::list<Behavior*>::iterator behaviorsIterator = assignmement.second.begin();
  behaviorsIterator != assignmement.second.end(); ++behaviorsIterator){
    if((*behaviorsIterator) != (*assignmement.second.begin())){
      std::cout<<",";
    }
    std::cout<<(*behaviorsIterator)->task->name<<"="<<(*behaviorsIterator)->name;
  }
  std::cout<<")"<<std::endl;
}

bool NoGoodConstraint::checkTaskInConstraint(Task* task){
  return false;
}

bool NoGoodConstraint::revise(){
  for(std::list<Behavior*>::iterator valuesIterator = assignmement.second.begin();
  valuesIterator != assignmement.second.end(); ++valuesIterator){
    if((*valuesIterator)->task->domain.size()!=1 || (*valuesIterator)->task->domain.front()!=(*valuesIterator)){
      return false;
    }
  }
  assignmement.first->domain.erase( assignmement.first->domain.begin());
  return true;
}

Constraint* NoGoodConstraint::turnConstraint(){
  return NULL;
}

int NoGoodConstraint::getConstraintType(){
  return 2;
}

std::string* NoGoodConstraint::getFirst(){
  return &assignmement.first->name;
}

std::string* NoGoodConstraint::getSecond(){
  return &assignmement.second.front()->name;
}