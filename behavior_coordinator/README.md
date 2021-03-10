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
- **start_task** ([behavior_coordinator_msgs/StartTask](https://github.com/cvar-upm/coordinator_cbc/blob/main/behavior_coordinator_msgs/srv/StartTask.srv))  
Starts a given task.

- **stop_task** ([behavior_coordinator_msgs/StopTask](https://github.com/cvar-upm/coordinator_cbc/blob/main/behavior_coordinator_msgs/srv/StopTask.srv))  
Stops a given task.

# Published topics

# Subscribed topics
- **task_stopped**  ([behavior_coordinator_msgs/TaskStopped](https://github.com/cvar-upm/coordinator_cbc/blob/main/behavior_coordinator_msgs/msg/TaskStopped.msg))  
Informs about the result of the execution of a task that has stopped.

---
# Contributors
**Maintainer:** Pablo Santamaria
**Author:** Pablo Santamaria
**Author:** Martin Molina
