/*!********************************************************************************
 * \brief     This file implements the tester class
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

#include <../include/yamlCreator.h>

void Tester::setup(){
  ros::NodeHandle private_nh;
  private_nh.getParam("T", T);
  private_nh.getParam("B", B);
  private_nh.getParam("R", R);
  private_nh.getParam("L", L);
  private_nh.getParam("M", M);
  private_nh.getParam("behavior_catalog_path",behavior_catalog_path);
  std::cout<<"CREATING YAML FILE WITH PARAMETERS: "<<std::endl;
  std::cout<<"  T: "<<T<<std::endl;
  std::cout<<"  B: "<<B<<std::endl;
  std::cout<<"  R: "<<R<<std::endl;
  std::cout<<"  L: "<<L<<std::endl;
  std::cout<<"  M: "<<M<<std::endl;
}

void Tester::CreateYaml(){
  YAML::Node annotationNode;
  YAML::Node default_values;
  YAML::Node default_tasks;
  YAML::Node automatic_activation;
  automatic_activation["automatic_activation"] = "TRUE";
  default_tasks["tasks"].push_back(automatic_activation);
  default_values.push_back(default_tasks);

  YAML::Node tasks;
  YAML::Node behaviors;

  srand(time(NULL));
  for (int l = 0; l<L; l++){
    for (int t = 0; t<T; t++){
      YAML::Node task;
      std::string task_str = "task" + std::to_string(l) + "_" + std::to_string(t);
      task["task"] = task_str;
      tasks.push_back(task);
      for (int b = 0; b<B; b++){
        YAML::Node behavior;
        std::string behavior_str = "behavior" + std::to_string(l) + "_" + std::to_string(t) + "_" + std::to_string(b);
        behavior["behavior"]=behavior_str;
        behavior["task"]=task_str;
        YAML::Node requires;
        if(l<L-1){
          for (int r = 0; r<R; r++){
            YAML::Node requirement;
            int randomTask = t;
            if(M==0){
              while(randomTask == t){
                randomTask = std::rand()%T;
              }
            }
            else{
              randomTask = std::rand()%(((T/M)*(t/(T/M)+1)+1) - ((T/M)*(t/(T/M))+1)) + (T/M)*(t/(T/M));
            }
            requirement["task"] = "task" + std::to_string(std::rand()%((L-1) - (l+1) + 1) + l+1) + "_" + std::to_string(randomTask);
            requires.push_back(requirement);
          }
        }
        YAML::Node efficacy;
        behavior["efficacy"] = std::to_string(std::rand()%(100 - 60 + 1) + 60);
        behavior["requires"] = requires;
        behaviors.push_back(behavior);
      }
    }
  }
  annotationNode["default_values"] = default_values;
  annotationNode["behaviors"] = behaviors;
  annotationNode["tasks"] = tasks;

  if(M>0){
    YAML::Node exclusivity_constraints;
    for (int i = 0; i<M*L; i++){
      YAML::Node exclusivity_constraint;
      for (int m = 0; m<M; m++){
        //exclusivity_constraint["mutually_exclusive"].push_back("task" + std::to_string(std::rand()%L) + "_" + std::to_string(std::rand()%((5) - 3) + 2));
        exclusivity_constraint["mutually_exclusive"].push_back("task" + std::to_string(std::rand()%L) + "_" + std::to_string(std::rand()%((((T/M)*(m+1)+1)) - ((T/M)*m+1)) + ((T/M)*m)));
      }
      exclusivity_constraints.push_back(exclusivity_constraint);
    }
    annotationNode["exclusivity_constraints"] = exclusivity_constraints;
  }

  std::ofstream fout(behavior_catalog_path);
  fout << annotationNode;
}

int main(int argc, char** argv){
  ros::init(argc, argv, ros::this_node::getName());
  std::cout << ros::this_node::getName() << std::endl;
  Tester test;
  test.setup();
  test.CreateYaml();
  return 0;
}