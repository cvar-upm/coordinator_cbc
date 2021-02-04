/*!********************************************************************************
 * \brief     This file implements the task class
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

#include <pluginlib/class_list_macros.h>
#include "../include/task.h"
#include "../include/behavior.h"
#include "../include/constraint.h"

Task::Task(){
}

Task::~Task(){
}

Task::Task(std::string arg_name){
  name = arg_name;
  performance = 0;
}

void Task::resetInitialDomain(bool testing){
  initialDomain.clear();
  for (std::list<Behavior*>::iterator behaviorsIterator = candidateBehaviors.begin();
  behaviorsIterator != candidateBehaviors.end(); ++behaviorsIterator){
    if(testing && (*behaviorsIterator)!=(*behaviorsIterator)->task->inactive){
      initialDomain.push_back(*behaviorsIterator);
    }
    else if((*behaviorsIterator)!=(*behaviorsIterator)->task->inactive && (*behaviorsIterator)->checkSituation()){
      initialDomain.push_back(*behaviorsIterator);
    }
  }
  initialDomain.push_back(inactive);
}

void Task::setInitialDomainWithSingleBehavior(){
  initialDomain.clear();
  initialDomain.push_back(activeBehavior);
}

void Task::setInitialDomainWithSingleInactiveValue(){
  initialDomain.clear();
  initialDomain.push_back(inactive);
}

void Task::setInitialDomainWithIntersection(std::list<Behavior*> arg_values){
  std::list<Behavior*>::iterator initialDomainIterator = initialDomain.begin();
  while (initialDomainIterator != initialDomain.end()){
    bool found = false;
    for (std::list<Behavior*>::iterator arg_valuesIterator = arg_values.begin();
    arg_valuesIterator != arg_values.end(); ++arg_valuesIterator){
      if(arg_valuesIterator == initialDomainIterator){
        found = true;
        break;
      }
    }
    if(!found){
      initialDomainIterator = initialDomain.erase(initialDomainIterator);
    }
    else{
      initialDomainIterator++;
    }
  }
}

void Task::calculatePerformance(){
  if(!performanceCalculated){
    performanceCalculated = true;
    if(assignment->name == "inactive"){
      performance = assignment->efficacy;
    }
    else{
      performance = assignment->efficacy*obtainPerformanceOfRequiredTasks()/100;
    }
  }
}

int Task::obtainPerformanceOfRequiredTasks(){
  int performanceOfRequiredTasks = 100;
  for (std::list<Constraint*>::iterator constraintsit = constraints.begin(); constraintsit != constraints.end(); ++constraintsit){
    if ((*constraintsit)->getConstraintType() == 1 && (*constraintsit)->getRequiringBehavior() == assignment){
      Task* requiredTask = (*constraintsit)->getRequiredTask();
      requiredTask->calculatePerformance();
      if (requiredTask->performance < (*constraintsit)->getRequiredTask()->performance){
        performanceViolation = (*constraintsit);
        try{
          throw performanceViolationFound();
        }
        catch (performanceViolationFound& e){
          performanceViolation->generateAssignmentExplainingViolation();
        }
      }
      performanceOfRequiredTasks = std::min(performance, performanceOfRequiredTasks);
    }
  }
  return performanceOfRequiredTasks;
}

/*
void Task::generatePerformanceExplanation(){
  if (task.assignment.type == "inactive"){
    return NULL;
  }
  else{
    explanation = NULL;
    for constraint in task.constraints
      if (constraint.type = requirementConstraint) and
        (constraint.requiringBehavior = task.assignment.behavior)
      then
        requiredTask := constraint.requiredTask
        pair := new(pair)
        pair[1] := requiredTask, 
        pair[2] := requiredTask.assignment       
        addAssignment(explanation, pair)
        nextExplanation := generatePerformanceExplanation(requiredTask)
        explanation := concatenateAssignment(explanation, nextExplanation)
    return explanation
  }
}
*/

void Task::printIncompatibleTasks(){
  std::cout<<name<<std::endl;
  for(std::list<Task*>::iterator incompatibleTasksIterator = incompatibleTasks.begin();
  incompatibleTasksIterator != incompatibleTasks.end(); ++incompatibleTasksIterator){
    std::cout<<"  "<<(*incompatibleTasksIterator)->name<<std::endl;
  }
}

void Task::printIncompatibleDefaultTasks(){
  std::cout<<name<<std::endl;
  for(std::list<Task*>::iterator incompatibleDefaultTasksIterator = incompatibleDefaultTasks.begin();
  incompatibleDefaultTasksIterator != incompatibleDefaultTasks.end(); ++incompatibleDefaultTasksIterator){
    std::cout<<"  "<<(*incompatibleDefaultTasksIterator)->name<<std::endl;
  }
}