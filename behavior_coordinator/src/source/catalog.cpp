/*!********************************************************************************
 * \brief     This file implements the catalog class
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

#include <fstream>

#include "../include/catalog.h"
#include "../include/constraint.h"
#include <pluginlib/class_list_macros.h>

Catalog::Catalog(std::string path_file){
  ros::NodeHandle private_nh;
  private_nh.getParam("testing", testing);
  try{
    yaml_node = YAML::LoadFile(path_file);
    if(yaml_node.IsNull()){
      std::cout << ERROR << "[ERROR]: There is a problem with the file" << STOP << std::endl;
    }
    else{
      if(yaml_node["default_values"]){
        for (YAML::const_iterator default_valuesIterator=yaml_node["default_values"].begin();
        default_valuesIterator!=yaml_node["default_values"].end();++default_valuesIterator) {
          if((*default_valuesIterator)["tasks"]){
            ReactiveValue newReactiveValue;
            for (YAML::const_iterator tasksIterator=(*default_valuesIterator)["tasks"].begin();
            tasksIterator!=(*default_valuesIterator)["tasks"].end();++tasksIterator) {
              if((*tasksIterator)["execution_goal"]){
                newReactiveValue.execution_goal = (*tasksIterator)["execution_goal"].as<std::string>();
              }
              if((*tasksIterator)["timeout"]){
                newReactiveValue.timeout = (*tasksIterator)["timeout"].as<int>();
              }
              if((*tasksIterator)["start_on_request"]){
                newReactiveValue.start_on_request = (*tasksIterator)["start_on_request"].as<std::string>() == "TRUE";
              }
              if((*tasksIterator)["reactive_start"]){
                newReactiveValue.reactive_start_value = (*tasksIterator)["reactive_start"].as<std::string>() == "TRUE";
              }
            }
            newReactiveValue.type = "tasks";
            default_values.push_back(newReactiveValue);
          }
          if((*default_valuesIterator)["behaviors"]){
            ReactiveValue newReactiveValue;
            for (YAML::const_iterator behaviorsIterator=(*default_valuesIterator)["behaviors"].begin();
            behaviorsIterator!=(*default_valuesIterator)["behaviors"].end();++behaviorsIterator) {
              if((*behaviorsIterator)["suitability"]){
                newReactiveValue.suitability = (*behaviorsIterator)["suitability"].as<double>()/100;
              }
              else{
                newReactiveValue.suitability = 0;
              }
            }
            newReactiveValue.type = "behaviors";
            default_values.push_back(newReactiveValue);
          }
        }
      }
      if(yaml_node["behaviors"]){
        for (YAML::const_iterator behaviorsIterator=yaml_node["behaviors"].begin();
        behaviorsIterator!=yaml_node["behaviors"].end();++behaviorsIterator) {
          Behavior newBehavior;
          if((*behaviorsIterator)["task"]){
            bool taskFound = false;
            for (std::list<Task>::iterator tasksIterator = initialTasks.begin(); tasksIterator != initialTasks.end(); ++tasksIterator){
              if(tasksIterator->name == (*behaviorsIterator)["task"].as<std::string>()){
                taskFound = true;
              }
            }
            if(!taskFound){
              Task newTask((*behaviorsIterator)["task"].as<std::string>());
              for(std::list<ReactiveValue>::iterator reactive_startValuesIterator = default_values.begin();
              reactive_startValuesIterator != default_values.end(); ++reactive_startValuesIterator){
                if(reactive_startValuesIterator->type == "tasks"){
                  newTask.timeout = reactive_startValuesIterator->timeout;
                  break;
                }
              }
              initialTasks.push_back(newTask);
            }
          }
          else if((*behaviorsIterator)["behavior"]){
            errors.push_back("The behavior: " + (*behaviorsIterator)["behavior"].as<std::string>() + " has no task defined");
          }
          for (std::list<Task>::iterator tasksIterator = initialTasks.begin(); tasksIterator != initialTasks.end(); ++tasksIterator){
            if(tasksIterator->name == (*behaviorsIterator)["task"].as<std::string>()){
              newBehavior.task=&(*tasksIterator);
              break;
            }
          }
          if((*behaviorsIterator)["behavior"]){
            newBehavior.name = (*behaviorsIterator)["behavior"].as<std::string>();
          }
          if((*behaviorsIterator)["package"]){
            newBehavior.package = (*behaviorsIterator)["package"].as<std::string>();
          }
          if((*behaviorsIterator)["suitability"]){
            newBehavior.suitability = (*behaviorsIterator)["suitability"].as<double>()/100;
          }
          else{
            for(std::list<ReactiveValue>::iterator reactive_startValuesIterator = default_values.begin();
            reactive_startValuesIterator != default_values.end(); ++reactive_startValuesIterator){
              if(reactive_startValuesIterator->type == "behaviors"){
                newBehavior.suitability = reactive_startValuesIterator->suitability;
                break;
              }
            }
          }
          behaviors.push_back(newBehavior);
        }
      }
      if(yaml_node["tasks"]){
        for (YAML::const_iterator tasksIterator=yaml_node["tasks"].begin();tasksIterator!=yaml_node["tasks"].end();++tasksIterator) {
          Task* task;
          if((*tasksIterator)["task"]){
            bool taskFound = false;
            for (std::list<Task>::iterator searchTasksIterator = initialTasks.begin(); searchTasksIterator != initialTasks.end(); ++searchTasksIterator){
              if(searchTasksIterator->name == (*tasksIterator)["task"].as<std::string>()){
                task=&(*searchTasksIterator);
                taskFound = true;
              }
            }
            if(!taskFound){
              Task newTask((*tasksIterator)["task"].as<std::string>());
              initialTasks.push_back(newTask);
              task=&(newTask);
            }
          }
          if((*tasksIterator)["start_on_request"]){
            task->startOnRequest = (*tasksIterator)["start_on_request"].as<std::string>() == "TRUE";
          }
          else{
            for(std::list<ReactiveValue>::iterator reactive_startValuesIterator = default_values.begin();
            reactive_startValuesIterator != default_values.end(); ++reactive_startValuesIterator){
              if(reactive_startValuesIterator->type == "tasks"){
                task->startOnRequest = reactive_startValuesIterator->start_on_request;
                break;
              }
            }
          }
          if((*tasksIterator)["timeout"]){
            task->timeout = (*tasksIterator)["timeout"].as<int>();
          }
          else{
            for(std::list<ReactiveValue>::iterator reactive_startValuesIterator = default_values.begin();
            reactive_startValuesIterator != default_values.end(); ++reactive_startValuesIterator){
              if(reactive_startValuesIterator->type == "tasks"){
                task->timeout = reactive_startValuesIterator->timeout;
                break;
              }
            }
          }
          if((*tasksIterator)["reactive_start"]){
            task->isReactive_started = (*tasksIterator)["reactive_start"].as<std::string>() == "TRUE";
          }
          else{
            for(std::list<ReactiveValue>::iterator reactive_startValuesIterator = default_values.begin();
            reactive_startValuesIterator != default_values.end(); ++reactive_startValuesIterator){
              if(reactive_startValuesIterator->type == "tasks"){
                task->isReactive_started = reactive_startValuesIterator->reactive_start_value;
                break;
              }
            }
          }
          if((*tasksIterator)["execution_goal"]){
            task->execution_goal = (*tasksIterator)["execution_goal"].as<std::string>();
          }
          if((*tasksIterator)["parameters"]){
            for (YAML::const_iterator parametersIterator=(*tasksIterator)["parameters"].begin();
            parametersIterator!=(*tasksIterator)["parameters"].end();++parametersIterator) {
              std::pair<std::string,std::string> parameter;
              if((*parametersIterator)["parameter"]){
                parameter.first = (*parametersIterator)["parameter"].as<std::string>();
              }
              if((*parametersIterator)["allowed_value"]){
                parameter.second = (*parametersIterator)["allowed_value"].as<std::string>();
              }
              task->parameters.push_back(parameter);
            }
          }
          if(task->startOnRequest && task->isReactive_started){
            errors.push_back("The task: " + task->name + " has incompatible fields start_on_request: TRUE and reactive_start: TRUE (reactive_start: FALSE is required for this configuration)");
          }
        }
      }
    }
    for (YAML::const_iterator behaviorsIterator=yaml_node["behaviors"].begin();
    behaviorsIterator!=yaml_node["behaviors"].end();++behaviorsIterator) {
      if((*behaviorsIterator)["requires"]){
        for (YAML::const_iterator requiresIterator=(*behaviorsIterator)["requires"].begin();
        requiresIterator!=(*behaviorsIterator)["requires"].end();++requiresIterator) {
          Behavior* nB = NULL;
          Task* nT = NULL;
          for (std::list<Behavior>::iterator behaviorsit = behaviors.begin(); behaviorsit != behaviors.end(); ++behaviorsit){
            if(behaviorsit->name == (*behaviorsIterator)["behavior"].as<std::string>()){
              nB = &(*behaviorsit);
            }
          }
          bool taskFound = false;
          for (std::list<Task>::iterator tasksIterator = initialTasks.begin(); tasksIterator != initialTasks.end(); ++tasksIterator){
            if(tasksIterator->name == (*requiresIterator)["task"].as<std::string>()){
              taskFound = true;
            }
          }
          if(!taskFound){
            errors.push_back("The required task: " + (*requiresIterator)["task"].as<std::string>() + " is not declared");
          }
          else{
            for (std::list<Task>::iterator tasksIterator = initialTasks.begin(); tasksIterator != initialTasks.end(); ++tasksIterator){
              if(tasksIterator->name == (*requiresIterator)["task"].as<std::string>()){
                nT = &(*tasksIterator);
              }
            }
            double minimum_performance=0;
            if((*requiresIterator)["minimum_performance"]){
              minimum_performance = (*requiresIterator)["minimum_performance"].as<double>()/100;
            }
            Constraint* requirement= new RequirementConstraint(nB,nT,minimum_performance);
            requirement->mainTask = nB->task;
            requirement->otherTasks.push_back(nT);
            Constraint* requirement2= new RequirementConstraint(nB,nT,minimum_performance);
            requirement2->mainTask = nT;
            requirement2->otherTasks.push_back(nB->task);
            nB->task->requiredTasks.push_back(nT);
            constraints.push_back(requirement);
            constraints.push_back(requirement2);
          }
        }
      }
    }
  }
  catch (YAML::Exception exception){
    errors.push_back(exception.what());
  }
  ordenateTasks();
  if(!testing){
    discardInactiveNodes();
  }
  loadValues();
  deleteInactiveTasks();
  loadConstraints();
  checkCatalog();
  if(errors.size()>0){
    //std::cout<<"ERRORS FOUND IN CATALOG"<<std::endl;
    prettify.printError("ERRORS FOUND IN CATALOG");
    std::cout << ERROR;
    for(std::list<std::string>::iterator errorsIterator = errors.begin(); errorsIterator != errors.end(); ++errorsIterator){
      std::cout << *errorsIterator << std::endl;
      //prettify.printError(*errorsIterator);
    }
    std::cout << STOP;
  }
  else{
    std::cout<<"CATALOG SUCCESSFULLY LOADED"<<std::endl;
  }
}

bool Catalog::checkCatalog(){
  for (std::list<Constraint*>::iterator searchIncompatibleConstraints = constraints.begin(); searchIncompatibleConstraints != constraints.end(); ++searchIncompatibleConstraints){
    if((*searchIncompatibleConstraints)->getConstraintType()==0){
      for (std::list<Constraint*>::iterator searchRequirementConstraints = constraints.begin(); searchRequirementConstraints != constraints.end(); ++searchRequirementConstraints){
        if((*searchRequirementConstraints)->getConstraintType()==1){
          if(*searchRequirementConstraints != *searchIncompatibleConstraints 
          && ((*searchIncompatibleConstraints)->checkTaskInConstraint((*searchRequirementConstraints)->getRequiringBehavior()->task) && (*searchIncompatibleConstraints)->checkTaskInConstraint((*searchRequirementConstraints)->getRequiredTask()))){
            errors.push_back("Conflicting constraints:\n  " + (*searchRequirementConstraints)->getConstraint_str() + "\n " + "and" + "\n  " + (*searchIncompatibleConstraints)->getConstraint_str());
          }
        }
      }
    }
  }
  return true;
}

void Catalog::ordenateTasks(){
  for(std::list<Task>::iterator initialTasksIterator = initialTasks.begin(); initialTasksIterator != initialTasks.end(); ++initialTasksIterator){
    if(initialTasksIterator->requiredTasks.size()==0){
      tasks.push_back(&*initialTasksIterator);
    }
  }
  bool changes = true;
  while(changes){
    changes = false;
    for(std::list<Task>::iterator initialTasksIterator = initialTasks.begin(); initialTasksIterator != initialTasks.end(); ++initialTasksIterator){
      if(initialTasksIterator->requiredTasks.size()>0){
        int numRequires = initialTasksIterator->requiredTasks.size();
        bool exit = false;
        for(std::list<Task*>::iterator requiredTasksIterator = (*initialTasksIterator).requiredTasks.begin(); requiredTasksIterator != (*initialTasksIterator).requiredTasks.end() && !exit; ++requiredTasksIterator){
          for(std::list<Task*>::iterator tasksIterator = tasks.begin(); tasksIterator != tasks.end(); ++tasksIterator){
            if((*tasksIterator) == &*initialTasksIterator){
              exit = true;
              break;
            }
            else if((*tasksIterator) == (*requiredTasksIterator)){
              numRequires-=1;
            }
          }
        }
        if(numRequires == 0 && !exit){
          tasks.push_back(&*initialTasksIterator);
        }
      }
    }
    if(tasks.size()!=initialTasks.size()){
      changes = true;
    }
  }
}

void Catalog::loadConstraints(){
  if(yaml_node.IsNull()){
    errors.push_back("There is a problem with the file");
  }
  else{
    if(yaml_node["exclusivity_constraints"]){
      for (YAML::const_iterator constraintsIterator=yaml_node["exclusivity_constraints"].begin();
      constraintsIterator!=yaml_node["exclusivity_constraints"].end();++constraintsIterator) {
        if((*constraintsIterator)["mutually_exclusive"]){
          std::list<Task*> incompatibleTasks;
          for (YAML::const_iterator exclusiveIterator=(*constraintsIterator)["mutually_exclusive"].begin();
          exclusiveIterator!=(*constraintsIterator)["mutually_exclusive"].end();++exclusiveIterator) {
            Task* newTask;
            for (std::list<Task*>::iterator tasksIterator = tasks.begin(); tasksIterator != tasks.end(); ++tasksIterator){
              if((*tasksIterator)->name == (*exclusiveIterator).as<std::string>()){
                newTask = *tasksIterator;
                incompatibleTasks.push_back(newTask);
              }
            }
          }
          for(std::list<Task*>::iterator incompatibleTasksIterator = incompatibleTasks.begin();
          incompatibleTasksIterator != incompatibleTasks.end(); ++incompatibleTasksIterator){
            std::list<Task*>::iterator remainingIncompatibleTasksIterator = incompatibleTasksIterator;
            remainingIncompatibleTasksIterator++;
            while(remainingIncompatibleTasksIterator != incompatibleTasks.end()){
              std::list<Task*> newIncompatibleConstraint;
              newIncompatibleConstraint.push_back(*incompatibleTasksIterator);
              newIncompatibleConstraint.push_back(*remainingIncompatibleTasksIterator);
              Constraint* incompatible = new IncompatibilityConstraint(newIncompatibleConstraint);
              incompatible->mainTask = *incompatibleTasksIterator;
              incompatible->otherTasks.push_back(*remainingIncompatibleTasksIterator);
              Constraint* incompatible2 = new IncompatibilityConstraint(newIncompatibleConstraint);
              incompatible2->mainTask = *remainingIncompatibleTasksIterator;
              incompatible2->otherTasks.push_back(*incompatibleTasksIterator);
              constraints.push_back(incompatible);
              constraints.push_back(incompatible2);
              remainingIncompatibleTasksIterator++;
            }
          }
        }
      }
    }
  }
}

void Catalog::deleteInactiveTasks(){
  for(std::list<Task*>::iterator tasksIterator = tasks.begin(); tasksIterator != tasks.end(); ++tasksIterator){
    if((*tasksIterator)->candidateBehaviors.size()==1 && (*tasksIterator)->candidateBehaviors.front()==(*tasksIterator)->inactive){
      tasksIterator = tasks.erase(tasksIterator);
    }
  }
}

void Catalog::discardInactiveNodes(){
  std::vector<std::string> v;
  if(!ros::master::getNodes(v)){
    std::cout<<"ERROR"<<std::endl;
  }
  else{
    std::list<Behavior>::iterator behaviorsIterator = behaviors.begin();
    while(behaviorsIterator != behaviors.end()){
      std::string behavior_name = behaviorsIterator->name;
      std::transform(behavior_name.begin(), behavior_name.end(), behavior_name.begin(), ::tolower);
      if(!(ros::service::exists("/" + behaviorsIterator->robot_namespace + behaviorsIterator->robot_id +
                                          "/" + behaviorsIterator->package + "/behavior_" + behavior_name+ "/activate_behavior",true))){
        std::list<Constraint*>::iterator constraintsit = constraints.begin();
        while(constraintsit != constraints.end()){
          if((*constraintsit)->getConstraintType()==1 && (*constraintsit)->getRequiringBehavior()==&*behaviorsIterator){
            constraintsit = constraints.erase(constraintsit);
          }
          else{
            constraintsit++;
          }
        }
        behaviorsIterator = behaviors.erase(behaviorsIterator);
      }
      else{
        behaviorsIterator++;
      }
    }
  }
}

void Catalog::loadValues(){
  for (std::list<Behavior>::iterator behaviorsit = behaviors.begin(); behaviorsit != behaviors.end(); ++behaviorsit){
    behaviorsit->initServices();
    Behavior* newValue(&(*behaviorsit));
    for (std::list<Task*>::iterator tasksIterator = tasks.begin(); tasksIterator != tasks.end(); ++tasksIterator){
      if((*tasksIterator)->name == behaviorsit->task->name){
        (*tasksIterator)->candidateBehaviors.push_back(&(*behaviorsit));
        (*tasksIterator)->initialDomain.push_back(&(*behaviorsit));
      }
    }
  }
  for (std::list<Task>::iterator tasksIterator = initialTasks.begin(); tasksIterator != initialTasks.end(); ++tasksIterator){
    Behavior inactive("inactive",false,&*tasksIterator,1);
    behaviors.push_back(inactive);
    (*tasksIterator).inactive = &behaviors.back();
    (*tasksIterator).candidateBehaviors.push_back(&behaviors.back());
    (*tasksIterator).initialDomain.push_back(&behaviors.back());
  }
}

Catalog::Catalog(){
}

Catalog::~Catalog(){
}

void Catalog::printDebugInfo(){
  std::cout<<"[TASKS]"<<std::endl;
  for (std::list<Task*>::iterator tasksIterator = tasks.begin(); tasksIterator != tasks.end(); ++tasksIterator){
    std::cout<<"  "<<(*tasksIterator)->name<<std::endl;
  }

  std::cout<<"[CONSTRAINTS]"<<std::endl;
  printConstraints();

  std::cout<<"[BEHAVIORS]"<<std::endl;
  for (std::list<Behavior>::iterator behaviorsit = behaviors.begin(); behaviorsit != behaviors.end(); ++behaviorsit){
    std::cout<<"  "<<behaviorsit->name<<", "<<behaviorsit->suitability<<" (task: "<<behaviorsit->task->name<<")"<<std::endl;
  }
}
void Catalog::printValues(){
  for (std::list<Task*>::iterator tasksIterator = tasks.begin(); tasksIterator != tasks.end(); ++tasksIterator){
    //if((*(*tasksIterator)->domain.begin())->name!="inactive"){
      std::cout<<"  "<<(*tasksIterator)->name<<"={";
      for (std::list<Behavior*>::iterator domainIterator = (*tasksIterator)->domain.begin();
      domainIterator != (*tasksIterator)->domain.end(); ++domainIterator){
        if(domainIterator != (*tasksIterator)->domain.begin()){
          std::cout<<",";
        }
        std::cout<<(*domainIterator)->name;
      }
      std::cout<<"}"<<std::endl;
    //}
  }
}


void Catalog::printConstraints(){
  for (std::list<Constraint*>::iterator constraintsit = constraints.begin(); constraintsit != constraints.end(); ++constraintsit){
    (*constraintsit) -> printConstraint();
  }
}

void Catalog::printInitialDomain(){
  for (std::list<Task*>::iterator tasksIterator = tasks.begin(); tasksIterator != tasks.end(); ++tasksIterator){
    std::cout<<"  "<<(*tasksIterator)->name<<"={";
    for (std::list<Behavior*>::iterator domainIterator = (*tasksIterator)->initialDomain.begin();
    domainIterator != (*tasksIterator)->initialDomain.end(); ++domainIterator){
      if(domainIterator != (*tasksIterator)->initialDomain.begin()){
        std::cout<<", ";
      }
      std::cout<<(*domainIterator)->name;
    }
    std::cout<<"}"<<std::endl;
  }
}

void Catalog::printCandidateBehaviors(){
  std::cout<<"[CANDIDATE BEHAVIORS]"<<std::endl;
  for (std::list<Task*>::iterator tasksIterator = tasks.begin(); tasksIterator != tasks.end(); ++tasksIterator){
    std::cout<<"  "<<(*tasksIterator)->name<<"={";
    for (std::list<Behavior*>::iterator candidateBehaviorsIterator = (*tasksIterator)->candidateBehaviors.begin();
    candidateBehaviorsIterator != (*tasksIterator)->candidateBehaviors.end(); ++candidateBehaviorsIterator){
      if(candidateBehaviorsIterator != (*tasksIterator)->candidateBehaviors.begin()){
        std::cout<<",";
      }
      std::cout<<(*candidateBehaviorsIterator)->name;
    }
    std::cout<<"}"<<std::endl;
  }
}