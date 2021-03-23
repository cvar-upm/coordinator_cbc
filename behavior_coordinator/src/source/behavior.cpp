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


#include "../include/task.h"
#include "../include/behavior.h"
#include <pluginlib/class_list_macros.h>

Behavior::Behavior(){
  ros::NodeHandle private_nh;
  private_nh.getParam("robot_id", robot_id);
  private_nh.getParam("robot_namespace", robot_namespace);
  private_nh.param<std::string>("check_situation_str", check_situation_str, "check_activation_conditions");
  private_nh.shutdown();
}

Behavior::Behavior(std::string arg_name,bool arg_active,Task* arg_task,double arg_suitability){
  ros::NodeHandle private_nh;
  private_nh.getParam("robot_id", robot_id);
  private_nh.getParam("robot_namespace", robot_namespace);
  private_nh.param<std::string>("check_situation_str", check_situation_str, "check_activation_conditions");
  private_nh.shutdown();
  name=arg_name;
  active=arg_active;
  task=arg_task;
  suitability=arg_suitability;
}

bool Behavior::activate(){
  active = true;
  return true;
}

bool Behavior::deactivate(){
  active = false;
  return true;
}

void Behavior::initCheckSituation(){
  behavior_name_tolower = name;
  std::transform(behavior_name_tolower.begin(), behavior_name_tolower.end(), behavior_name_tolower.begin(), ::tolower);
  checkSituation_srv = nh.serviceClient<behavior_execution_manager_msgs::CheckSituation>("/" + robot_namespace + robot_id + "/" + package + "/behavior_" +
                                                                                behavior_name_tolower + "/" + check_situation_str,true);
}

bool Behavior::checkSituation(){
  if(ros::service::exists("/" + robot_namespace + robot_id + "/" + package + "/behavior_" + behavior_name_tolower + "/" + check_situation_str,true)){
    behavior_execution_manager_msgs::CheckSituation msg;
    checkSituation_srv.call(msg);
    return bool(msg.response.situation_occurs);
  }
  return false;
}