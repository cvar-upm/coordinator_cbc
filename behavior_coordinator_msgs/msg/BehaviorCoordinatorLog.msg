# new behavior_controller action

Header header
int8 entity                    # Code of the action's entity
int8 event                     # Code of the action's event
string name  	               # name of the behavior or the task 
string parameters              # parameters of the behavior or the task
int8 priority         	       # priority of the task
int8 termination_cause         # int with the termination_cause reason
int8 failure                   # TASK_STARTED,TASK_STOPPED,BEHAVIOR_ACTIVATED,BEHAVIOR_DEACTIVATED may fail, 0 if succes 1 if fail
# Possible values for entity
int8 TASK       = 1
int8 BEHAVIOR   = 2

# Posible values for event

int8 START_TASK_REQUEST   		= 1
int8 TASK_STARTED         		= 2
int8 STOP_TASK_REQUEST    		= 3
int8 TASK_STOPPED         		= 4
int8 BEHAVIOR_ACTIVATION_REQUEST	= 5
int8 BEHAVIOR_DEACTIVATION_REQUEST 	= 6
int8 BEHAVIOR_ACTIVATED   		= 7
int8 BEHAVIOR_DEACTIVATED 		= 8
int8 START_TASK_REACTIVE		= 9
int8 BEHAVIOR_ACTIVATION_FINISHED	= 10
int8 EVENT_TIME_OUT			= 11



# Possible values for termination_cause

int8 GOAL_ACHIEVED	          = 1
int8 TIME_OUT           	  = 2
int8 WRONG_PROGRESS     	  = 3
int8 PROCESS_FAILURE    	  = 4
int8 INTERRUPTED        	  = 5
int8 REQUESTED_DEACTIVATION	  = 6
int8 AUTOMATIC_DEACTIVATION	  = 7
int8 TIME_OUT_SELF_DEACTIVATION	  = 8
int8 IMPOSSIBLE_ACTIVATION	  = 9
int8 TASK_NOT_FOUND		  = 10
