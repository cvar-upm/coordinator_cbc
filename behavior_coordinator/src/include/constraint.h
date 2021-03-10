/*!********************************************************************************
 * \brief     constraint definition file
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


#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <string>
#include <list>

// ROS
#include <ros/ros.h>

// Aerostack
#include "../include/task.h"
#include "../include/behavior.h"

class Constraint
{
public:
  Constraint();
  ~Constraint();


  virtual void printConstraint(){};
  virtual std::string getConstraint_str(){};
  virtual Task* revise(){};
  virtual std::string* getFirst(){};
  virtual std::string* getSecond(){};
  virtual Behavior* getRequiringBehavior(){};
  virtual Task* getRequiredTask(){};
  virtual int getConstraintType(){}; //0 incompatibility, 1 requirement, 2 nogood
  virtual Constraint* generateAssignmentExplainingViolation(){};
  virtual bool checkConstraint(){};
  virtual bool checkTaskInConstraint(Task* task){};
  virtual Task* getFirstTask(){};
  virtual Task* getSecondTask(){};
  virtual bool hasTask(Task* task){};
  
  bool queued = false;
  Task* mainTask;
  std::list<Task*> otherTasks;

  //DRIVE
private:
};

class IncompatibilityConstraint : public Constraint
{
public:
  IncompatibilityConstraint();
  ~IncompatibilityConstraint();
  IncompatibilityConstraint(std::list<Task*> arg_incompatibleTasks);
  void printConstraint();
  std::string getConstraint_str();
  Task* revise();
  std::string* getFirst();
  std::string* getSecond();
  int getConstraintType();
  bool checkTaskInConstraint(Task* task);
  Task* getFirstTask();
  Task* getSecondTask();
  bool hasTask(Task* task);

  //DRIVE
  std::list<Task*> incompatibleTasks;
private:
};

class RequirementConstraint : public Constraint
{
public:
  RequirementConstraint();
  ~RequirementConstraint();
  RequirementConstraint(Behavior* arg_behavior, Task* arg_task, double arg_performance);
  void printConstraint();
  std::string getConstraint_str();
  Task* revise();
  bool checkConstraint();
  std::string* getFirst();
  std::string* getSecond();
  int getConstraintType();
  Behavior* getRequiringBehavior();
  Task* getRequiredTask();
  RequirementConstraint* generateAssignmentExplainingViolation();
  double getMaxEfficacyValue(Task* taskIterator);
  bool checkTaskInConstraint(Task* task);
  bool hasTask(Task* task);

  //DRIVE
  Behavior* requiringBehavior;
  Task* requiredTask;
  double requiredPerformance;
private:
};

class NoGoodConstraint : public Constraint
{
public:
  NoGoodConstraint();
  ~NoGoodConstraint();
  NoGoodConstraint(Task* arg_task, std::list<Behavior*> arg_behavior);
  void printConstraint();
  std::string getConstraint_str();
  Task* revise();
  std::string* getFirst();
  std::string* getSecond();
  int getConstraintType();
  bool checkTaskInConstraint(Task* task);
  bool hasTask(Task* task);

  //DRIVE
  std::pair<Task*,std::list<Behavior*>> assignmement;
private:
};
#endif