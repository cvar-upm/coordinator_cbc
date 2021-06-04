#include "Behavior_Coordinator_Logger.h"
#include <ros/ros.h>
#include <ros/this_node.h>
#include <ros/master.h>

#include <thread>

#include <QApplication>


void spinnerThread()
{
  ros::Rate r(10);
  while (ros::ok())
  {
   ros::spinOnce();
   r.sleep();
  }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, ros::this_node::getName());

    QApplication a(argc, argv);

    Behavior_Coordinator_Logger w;

    w.show();

    std::thread thr(&spinnerThread);

    std::thread thr2(&Behavior_Coordinator_Logger::checkActiveBehaviors,&w );
  
    return a.exec();
}
