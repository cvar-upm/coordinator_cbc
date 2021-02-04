#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from std_msgs.msg import String
import time

from behavior_manager_msg.srv import StartTask, StopTask
from behavior_manager_msg.msg import TaskCommand, TaskStopped, ListOfRunningTasks
from behaviorlib_msg import BehaviorActivationFinished

def activate_a_task(name, priority, parameters):
    rospy.wait_for_service('/drone111/start_task')
    try:
        activateTask = rospy.ServiceProxy('/drone111/start_task', StartTask)
        msg = TaskCommand()
        msg.name = name
        msg.parameters = parameters
        msg.priority = priority
        resp1 = activateTask(msg)
        return resp1.ack
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def deactivate_a_task(name):
    rospy.wait_for_service('/drone111/stop_task')
    try:
        deactivateTask = rospy.ServiceProxy('/drone111/stop_task', StopTask)
        msg = name
        resp1 = deactivateTask(msg)
        return resp1.ack
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

#(0)-TASK_NAME["string"]
#(1)-PRIORITY[int]
#(2)-PARAMETERS["string"]
#(3)-START[True(start task)/False(stop task)]
#(4)-NO_ERROR_SIMULATION[True(do not simulate error)/False(simulate error)]
#(5)-DELAY[int(seconds)]
#(6)-TERMINATION_CAUSE[int(1 GOAL_ACHIEVED, 2 INTERRUPTED,...)]

# TESTS
#operaciones = [["TAKE_OFF",3,"",True,True,0.5,1],["FOLLOW_PATH",3,"path': [[2, 2, 1]]}",True,False,4,1],["LAND",1,"",True,True,1,1],["MPC_MOTION_CONTROL",3,"",False,False,0.5,5],["FOLLOW_PATH",3,"",False,False,0.5,5],["LAND",3,"",True,True,1,1]]
#operaciones = [["TAKE_OFF",3,"",True,True,0],["ROTATE",3,"",True,True,0],["FOLLOW_PATH",2,"{'path': [[3, 3, 1]]}",True,False,2],["FOLLOW_PATH",3,"{'path': [[0, 0, 1]]}",True,True,0],["LAND",3,"",True,True,0],["TAKE_OFF",3,"",True,True,15]]
#Error en primer follow_path
#operaciones = [["TAKE_OFF",3,"",True,True,0,1],["ROTATE",3,"",True,True,0,1],["FOLLOW_PATH",2,"{'path': [[3, 3, 1]]}",True,False,2,1],["FOLLOW_PATH",3,"{'path': [[0, 0, 1]]}",False,True,2,3],["LAND",3,"",True,True,0,1],["TAKE_OFF",3,"",True,True,15,1]]
#Error check
#operaciones = [["TAREA2",3,"",True,False,0],["TAREA4",2,"",True,False,0],["TAREA0",2,"",True,False,0],["BEHAVIOR1",2,"",False,True,1,3]]
#operaciones = [["TAKE_OFF",3,"",True,False,0.5],["HOVER",3,"",True,False,1],["LAND",3,"",True,True,1],["FOLLOW_PATH",2,"{'path': [[2, 2, 1]]}",True,False,3],["FOLLOW_PATH",3,"{'path': [[0, 0, 1]]}",True,True,0],["LAND",3,"",True,True,0]]
#old
#operaciones = [["TAKE_OFF",2,"",True,True,0.5,1],["FOLLOW_PATH",2,"{'path: [ [1.1,1.1,1.0], [2.2,2.2,1.0], [3.3,3.3,1.0] ]}",True,False,4,1],["LAND",1,"",True,True,0.5,1],["SCAN_LIDAR_2D",2,"",False,False,0.5,3],["SCAN_COSTMAP",2,"",True,True,0.5,1]]
#Vuelo real
operaciones = [["TAKE_OFF",3,"",True,True,1,BehaviorActivationFinished.GOAL_ACHIEVED],["FOLLOW_PATH",2,"{'path': [[3, 3, 1]]}",True,True,2,BehaviorActivationFinished.GOAL_ACHIEVED],["FOLLOW_PATH",3,"{'path': [[0, 0, 1]]}",False,True,2,BehaviorActivationFinished.WRONG_PROGRESS],["LAND",3,"",True,True,0,BehaviorActivationFinished.GOAL_ACHIEVED],["TAKE_OFF",3,"",True,True,4,BehaviorActivationFinished.GOAL_ACHIEVED]]

i = 0

def getActiveBehavior(task):
    msg = rospy.wait_for_message("/drone111/list_of_running_tasks", ListOfRunningTasks)
    behavior = ""
    for taskIterate in msg.list_of_running_tasks:
        if taskIterate.task_command.name == task:
            behavior = taskIterate.behavior
    return behavior

def simulateBehaviorFinished(task, cause, pause):
    publisher = rospy.Publisher("/drone111/behavior_activation_finished", BehaviorActivationFinished,queue_size=5)
    r = rospy.Rate(10) #10hz
    msg = BehaviorActivationFinished()
    msg.name = task
    msg.termination_cause = cause
    time.sleep(pause)
    print(task, cause)
    publisher.publish(msg)
    r.sleep()
    return 0

def callback(data):
    global i
    if i<len(operaciones):
        time.sleep(operaciones[i][5])
        task = operaciones[i][0]
        priority = operaciones[i][1]
        parameters = operaciones[i][2]
        i=i+1
        if operaciones[i-1][3]==True:
            print("Requesting %s, %d, %s"%(task,priority,parameters))
            res = activate_a_task(task,priority,parameters)
            activated_behavior = task
            print(res)
            if res:
                print(activated_behavior)
                print("")
                if operaciones[i-1][4]!=True:
                    time.sleep(operaciones[i-1][5])
                    callback(data)
                else:
                    if operaciones[i-1][5] != 0:
                        print("simulate GOAL_ACHIEVED")
                        behavior = getActiveBehavior(task)
                        simulateBehaviorFinished(behavior, operaciones[i-1][6], operaciones[i-1][5])
            else:
                time.sleep(operaciones[i-1][5])
                callback(data)
        else:
            if operaciones[i-1][4]==False:
                print("simulate ERROR")
                behavior = getActiveBehavior(task)
                simulateBehaviorFinished(behavior, operaciones[i-1][6], operaciones[i-1][5])
                callback(data)
            else:
                print("Deactivating %s"%(task))
                behavior = getActiveBehavior(task)
                res = deactivate_a_task(task)
                deactivated_behavior = task
                print(deactivated_behavior)
                if res:
                    simulateBehaviorFinished(behavior, operaciones[i-1][6], operaciones[i-1][5])
                else:
                    callback(data)

rospy.init_node('listener', anonymous=True)
print(rospy.get_param_names())
rospy.Subscriber("/drone111/task_stopped", TaskStopped, callback)
callback(None)
rospy.spin()
