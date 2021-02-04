# Brief
The behavior manger is in charge of the coordination and execution of each task received.
For the coordination, the process calculates the best consistent assignment of behaviors required to execute the requested task,
as defined in the catalog, taking in to account each behaviors efficacy, dependencies and incompatibilities.
Once the assignment has been found, the behavior manager executes all the new calculated behaviors and keeps track of it's execution
status (finnished correctly, failed, interrupted, etc...).
In addition, the execution of all the running behaviors is monitorized in order to get the best possible execution and always mantain
consistency, for example if a behavior fails, does not work as expected or it's eficacy decreases, an other solution might be required
in order to continue the same task or stop it.

# Services
- **start_task** ([behavior_manager_msg/StartTask](https://bitbucket.org/visionaerialrobotics/aerostack_msgs/src/master/srv/StartTask.srv))  
Starts a given task.

- **stop_task** ([behavior_manager_msg/StopTask](https://bitbucket.org/visionaerialrobotics/aerostack_msgs/src/master/srv/StopTask.srv))  
Stops a given task.

# Published topics
- **list_of_running_tasks** ([aerostack_msgs/ListOfRunningTasks](https://bitbucket.org/visionaerialrobotics/aerostack_msgs/src/master/msg/ListOfRunningTasks.msg))  
Publishes the list of running tasks and the behaviors currently executing the task. This topic is updated constantly from the moment a task starts until it is 
stopped (and there are no more task running).

# Subscribed topics
- **task_stopped**  ([behavior_manager_msg/TaskStopped](https://bitbucket.org/visionaerialrobotics/aerostack_msgs/src/master/msg/TaskStopped.msg))  
Informs about the result of the execution of a task that has stopped.

# Tests
The following tests are performed:

* **Test 1:** .

---
# Contributors
**Maintainer:** Pablo Santamaria
**Author:** Pablo Santamaria
**Author:** Martin Molina
