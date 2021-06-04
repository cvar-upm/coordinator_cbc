#ifndef BEHAVIOR_COORDINATOR_LOGGER_H
#define BEHAVIOR_COORDINATOR_LOGGER_H

#include <QWidget>
#include <QDialog>
#include <qlayout.h>
#include <QLabel>
#include <ros/ros.h>
//#include <behavior_coordinator_msgs/ActivationChange.h>
#include <behavior_coordinator_msgs/BehaviorCoordinatorLog.h>
#include <QTableWidgetItem>
#include <behavior_execution_manager_msgs/CheckActivation.h>
#include <thread>
#include <QScrollBar>

QT_BEGIN_NAMESPACE
namespace Ui { class Behavior_Coordinator_Logger; }
QT_END_NAMESPACE

class Behavior_Coordinator_Logger : public QWidget
{
    Q_OBJECT

public:
    Behavior_Coordinator_Logger(QWidget *parent = nullptr);
    ~Behavior_Coordinator_Logger();
    void checkActiveBehaviors();

public Q_SLOTS:

    void displayInfo(QTableWidgetItem *);
    void hideActiveBehaviors();
    void pauseMessages();


private:

    Ui::Behavior_Coordinator_Logger *ui;
    int actions_counter=0;
    bool showing;

    std::vector<std::string> all_behaviors_active;
    std::string possible_behaviors;
    std::vector<std::string> possibleBehaviors;
    int active_behavior_counter=0;
    bool shutdonw=true;

    void setEvent(const behavior_coordinator_msgs::BehaviorCoordinatorLog& msg,const int row);
    ros::NodeHandle n;
    std::string drone_id_namespace;
    std::string behavior_coordination_log;
    ros::Subscriber behavior_coordination_log_sub ;
    ros::ServiceClient check_behavior_situation_srv;

    QDialog * dlg;
    QLabel * label;
    QVBoxLayout * layout;
    //array keeping all msgs received
    std::vector<behavior_coordinator_msgs::BehaviorCoordinatorLog> msgs_recieved;

    //this funtion will set the widgets for the task selected
    void newBehaviorCoordiantorLogCallback(const behavior_coordinator_msgs::BehaviorCoordinatorLog& msg);

    void showInfo(const behavior_coordinator_msgs::BehaviorCoordinatorLog msg, int);
    void resizeEvent(QResizeEvent* event);

    void addNewActiveBehavior(std::string, std::string );
    void deactiveActiveBehavior(std::string );

    std::string getBehaviorName(std::string chain);
    
};
#endif // BEHAVIOR_COORDINATOR_LOGGER_H
