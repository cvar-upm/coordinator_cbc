/*!********************************************************************************
 * \brief     behavior definition file
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

#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <string>

// ROS
#include <ros/ros.h>

// Aerostack
#include <behavior_execution_manager_msgs/CheckSituation.h>
#include <behavior_execution_manager_msgs/CheckActivation.h>

class Task;

class Behavior
{
public:
  Behavior();
  Behavior(std::string arg_name,bool active,Task* task,double suitability);

  ros::NodeHandle nh;

  bool activate();
  bool deactivate();
  bool checkSituation();
  bool checkActivation();
  void initServices();

  //DRIVE
  std::string name;
  std::string behavior_name_tolower;
  std::string nameSpace;
  std::string package;
  bool nodeIsActive = true;
  bool active;
  Task* task;
  double suitability = 0;
  std::string check_situation_str;
  std::string check_activation_str;
  ros::ServiceClient checkSituation_srv;
  ros::ServiceClient checkActivation_srv;
  std::string robot_id;
  std::string robot_namespace;
  
private:
};

#endif