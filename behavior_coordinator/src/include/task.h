/*!********************************************************************************
 * \brief     task definition file
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

#ifndef TASK_H
#define TASK_H

#include <string>
#include <list>
#include <exception>
#include <chrono>

// ROS
#include <ros/ros.h>

class Behavior;
class Constraint;

class Task
{
public:
  Task();
  Task(std::string arg_name);
  ~Task();

  //DRIVE
  std::string name;
  std::string category;
  bool isDefault = false;
  bool defaultActivated = false;
  bool requested = false;
  int change_type;
  std::list<std::pair<std::string,std::string>> parameters;
  std::string arguments;
  bool active = false;
  std::list<Behavior*> candidateBehaviors;
  Behavior* inactive;
  bool automaticActivation = true;
  Behavior* activeBehavior;
  std::list<Behavior*> initialDomain;
  std::list<Behavior*> domain;
  Behavior* assignment;
  int performance;
  int timeout;
  bool performanceCalculated;
  std::list<Constraint*> constraints;
  std::list<Constraint*> mainInConstraints;
  std::list<Constraint*> secondaryInConstraints;
  Constraint* performanceViolation;
  std::list<Task*> incompatibleDefaultTasks;
  std::list<Task*> incompatibleTasks;
  std::list<Task*> requiredTasks;
  std::chrono::system_clock::time_point activationTime;
  ros::ServiceClient checkSituation_srv;

  int priority = 0;

  void resetInitialDomain(bool testing);
  void setInitialDomainWithSingleBehavior();
  void setInitialDomainWithSingleInactiveValue();
  void setInitialDomainWithIntersection(std::list<Behavior*> arg_values);
  void calculatePerformance();
  int obtainPerformanceOfRequiredTasks();
  void printIncompatibleTasks();
  void printIncompatibleDefaultTasks();
private:
};

struct performanceViolationFound : public std::exception {
   const char * what () const throw () {
      return "performanceViolationFound";
   }
};

#endif