#include "Behavior_Coordinator_Logger.h"
#include "ui_Behavior_Coordinator_Logger.h"

#include <iostream>
#include <QDebug>
#include <algorithm>

Behavior_Coordinator_Logger::Behavior_Coordinator_Logger(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Behavior_Coordinator_Logger)
{
    ui->setupUi(this);
    ui->loggerTable->setColumnCount(5);
   
    ui->loggerTable->horizontalHeader()->setVisible(true);
    ui->loggerTable->setHorizontalHeaderItem(0,new QTableWidgetItem(tr("N")));
    ui->loggerTable->setHorizontalHeaderItem(1,new QTableWidgetItem(tr("E")));
    ui->loggerTable->setHorizontalHeaderItem(2,new QTableWidgetItem(tr("Event")));
    ui->loggerTable->setHorizontalHeaderItem(3,new QTableWidgetItem(tr("Name")));
    ui->loggerTable->setHorizontalHeaderItem(4,new QTableWidgetItem(tr("Parameters")));
    ui->loggerTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Fixed);

    ui->loggerTable->setRowCount(1);
    

    ui->activeTable->setColumnCount(2);
    ui->activeTable->horizontalHeader()->setVisible(true);
    ui->activeTable->setHorizontalHeaderItem(0,new QTableWidgetItem(tr("Behavior name")));
    ui->activeTable->setHorizontalHeaderItem(1,new QTableWidgetItem(tr("Parameters")));

    QFont font = ui->loggerTable->horizontalHeader()->font();
    font.setPointSize( 10 );
    ui->loggerTable->horizontalHeader()->setFont( font );

    font = ui->activeTable->horizontalHeader()->font();
    font.setPointSize( 10 );
    ui->activeTable->horizontalHeader()->setFont( font );

    
    QHeaderView* header=ui->loggerTable->verticalHeader();
    header->setDefaultSectionSize(20);

    QHeaderView* header2=ui->activeTable->verticalHeader();
    header2->setDefaultSectionSize(20);
    QHeaderView* horizontalHeaderActiveTableui=ui->activeTable->horizontalHeader();
    horizontalHeaderActiveTableui->setSectionResizeMode(QHeaderView::Stretch);

    n.param<std::string>("robot_namespace", drone_id_namespace, "drone111");
    n.param<std::string>("activation_change", behavior_coordination_log, "behavior_coordinator_log");

    behavior_coordination_log_sub = n.subscribe("/" + drone_id_namespace + "/" + behavior_coordination_log, 1000, &Behavior_Coordinator_Logger::newBehaviorCoordiantorLogCallback, this);
    
    qDebug() << connect(ui->loggerTable, SIGNAL(itemClicked(QTableWidgetItem *)), this, SLOT(displayInfo(QTableWidgetItem *)));
    connect(ui->hideButtom,SIGNAL(clicked()),this,SLOT(hideActiveBehaviors()));
    connect(ui->pauseButtom,SIGNAL(clicked()),this, SLOT(pauseMessages()));
    std::cout << "CONCETA PORFA";

    this->setWindowTitle(tr("Behavior coordinator logger"));

    showing=false;
    ui->label->hide();
    ui->activeTable->hide();
   
    n.getParam("all_behaviors",possible_behaviors);


    if(possible_behaviors.size()>10)
    {
        std::string delimiter = "\n";
        size_t pos = 0;
        std::string token;
        while ((pos = possible_behaviors.find(delimiter)) != std::string::npos) 
        {
            token = possible_behaviors.substr(0, pos);
            std::cout << token << std::endl;
            possibleBehaviors.push_back(token);
            possible_behaviors.erase(0, pos + delimiter.length());

        }
        
    }
    else
    {
        std::cout << "hay elementos" << std::endl;
    }
    
    
}

Behavior_Coordinator_Logger::~Behavior_Coordinator_Logger()
{
    delete ui;
}

std::string Behavior_Coordinator_Logger::getBehaviorName(std::string chain)
{
    std::string delimiter="/behavior_";
    size_t pos= 0;
    std::string token;
    pos= chain.find(delimiter);
    chain.erase(0,pos+delimiter.length());
    delimiter="/";
    pos= chain.find(delimiter);
    chain.erase(pos, chain.length());
    for(int i=0; i< chain.length(); i++)
    {
        chain[i]=toupper(chain[i]);
    }
    return chain;
}

void Behavior_Coordinator_Logger::checkActiveBehaviors()
{
    
    ros::Rate loop_rate(5);
    std::string path;
    std::vector<std::string>::iterator it;
    std::string behavior;

    while (ros::ok())
    {
       
        for (int i = 0; i< possibleBehaviors.size(); i++)
        {
            path =  possibleBehaviors[i];
            check_behavior_situation_srv = n.serviceClient<behavior_execution_manager_msgs::CheckActivation>(path);
            behavior_execution_manager_msgs::CheckActivation check_activation_msg;
            check_behavior_situation_srv.call(check_activation_msg);
            behavior=getBehaviorName(path);
            
            it= std::find(all_behaviors_active.begin(),all_behaviors_active.end(),behavior);
            if( check_activation_msg.response.is_active && it == all_behaviors_active.end())
            {   
                std::cout << "porque no te metes\n";
                addNewActiveBehavior(behavior,check_activation_msg.response.parameters);   
            }
            else if( !check_activation_msg.response.is_active && it != all_behaviors_active.end())
            {
                deactiveActiveBehavior(behavior);
            }
            
        }

        loop_rate.sleep();
    }
    
}



void Behavior_Coordinator_Logger::newBehaviorCoordiantorLogCallback(const behavior_coordinator_msgs::BehaviorCoordinatorLog& msg){


    msgs_recieved.push_back(msg);

    ui->loggerTable->setRowCount(ui->loggerTable->rowCount()+1);

    QTableWidgetItem *entity;

    QFont font;
     
    font.setPointSize(9);

    if( msg.entity == behavior_coordinator_msgs::BehaviorCoordinatorLog::TASK)
    {
        entity = new QTableWidgetItem(tr("T"));
    }
    else
    {
        entity = new QTableWidgetItem(tr("B"));   
    }

    QTableWidgetItem *name = new QTableWidgetItem(QString::fromStdString(msg.name));
    name->setFont(font);
    QTableWidgetItem *parameters = new QTableWidgetItem(QString::fromStdString(msg.parameters));
    parameters->setFont(font);

    ui->loggerTable->setSelectionBehavior(QAbstractItemView::SelectRows);
     
    ui->loggerTable->setItem(this->actions_counter,0,new QTableWidgetItem(QString::number(this->actions_counter+1)));
    ui->loggerTable->item(this->actions_counter, 0)-> setTextAlignment(Qt::AlignCenter);
    ui->loggerTable->setItem(this->actions_counter,1,entity);
    ui->loggerTable->item(this->actions_counter, 1)-> setTextAlignment(Qt::AlignCenter);
    this->setEvent(msg,this->actions_counter);
    ui->loggerTable->setItem(this->actions_counter,3,name);
    ui->loggerTable->setItem(this->actions_counter,4,parameters);
    actions_counter++;
    ui->loggerTable->scrollToItem(name,QAbstractItemView::PositionAtTop);

    
}


void Behavior_Coordinator_Logger::setEvent(const behavior_coordinator_msgs::BehaviorCoordinatorLog& msg,const int row)
{
    QTableWidgetItem *event;
    switch (msg.event)
    {
    case behavior_coordinator_msgs::BehaviorCoordinatorLog::START_TASK_REQUEST:
        event = new QTableWidgetItem(tr("Start request"));
        break;
    case behavior_coordinator_msgs::BehaviorCoordinatorLog::TASK_STARTED:
        event = new QTableWidgetItem(tr("Task started"));
        event->setForeground(QBrush(QColor(255,0,0)));
        break;
    case behavior_coordinator_msgs::BehaviorCoordinatorLog::STOP_TASK_REQUEST:
        event = new QTableWidgetItem(tr("Stop task request"));
        break;
    case behavior_coordinator_msgs::BehaviorCoordinatorLog::TASK_STOPPED:
        event = new QTableWidgetItem(tr("Task stopped"));
        if(msg.failure ==1) event->setForeground(QBrush(Qt::darkGreen));
        else event->setForeground(QBrush(QColor(255,0,0)));
        break;
    case behavior_coordinator_msgs::BehaviorCoordinatorLog::BEHAVIOR_ACTIVATION_REQUEST:
        event = new QTableWidgetItem(tr("Activation request"));
        break;
    case behavior_coordinator_msgs::BehaviorCoordinatorLog::BEHAVIOR_DEACTIVATION_REQUEST:
        event = new QTableWidgetItem(tr("Deactivation request"));
        break;
    case behavior_coordinator_msgs::BehaviorCoordinatorLog::BEHAVIOR_ACTIVATED:
        event = new QTableWidgetItem(tr("Activated"));
        if(msg.failure ==1) event->setForeground(QBrush(QColor(255,0,0)));
        else event->setForeground(QBrush(Qt::darkGreen));
        break;
    case behavior_coordinator_msgs::BehaviorCoordinatorLog::BEHAVIOR_DEACTIVATED:
        event = new QTableWidgetItem(tr("Deactivated"));
        if(msg.failure ==1) event->setForeground(QBrush(QColor(255,0,0)));
        else event->setForeground(QBrush(Qt::darkGreen));
        break;
    case behavior_coordinator_msgs::BehaviorCoordinatorLog::START_TASK_REACTIVE:
        event = new QTableWidgetItem(tr("Reactive start"));
        break;
    case behavior_coordinator_msgs::BehaviorCoordinatorLog::BEHAVIOR_ACTIVATION_FINISHED:
        event = new QTableWidgetItem(tr("Activation finished"));
        if(msg.termination_cause == (behavior_coordinator_msgs::BehaviorCoordinatorLog::GOAL_ACHIEVED )|| 
        msg.termination_cause == behavior_coordinator_msgs::BehaviorCoordinatorLog::AUTOMATIC_DEACTIVATION )
            event->setForeground(QBrush(Qt::darkGreen));
        else 
            event->setForeground(QBrush(QColor(255,0,0)));
        break;
    case behavior_coordinator_msgs::BehaviorCoordinatorLog::EVENT_TIME_OUT:
        event = new QTableWidgetItem(tr("Time out"));
        break;
    default:
        break;
    }
    ui->loggerTable->setItem(row,2,event);
}


void Behavior_Coordinator_Logger::displayInfo(QTableWidgetItem * itemSelected)
{

    const behavior_coordinator_msgs::BehaviorCoordinatorLog msg_selected= msgs_recieved[itemSelected->row()];

    showInfo(msg_selected,itemSelected->row());

    std::cout << "displayInfo ejecutandose" << std::endl;

    
}

void Behavior_Coordinator_Logger::showInfo(const behavior_coordinator_msgs::BehaviorCoordinatorLog msg,int row)
{

    dlg = new QDialog(this);
    label= new QLabel();
    layout = new QVBoxLayout();
    layout->addWidget(label);
    dlg->setLayout(layout);
    //dlg->setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
    dlg->setWindowTitle("Event details");

    std::string labelText="";

    if(msg.entity == behavior_coordinator_msgs::BehaviorCoordinatorLog::TASK)
    {
        labelText+="Entity: Task\n";
        labelText+="Priority: ";
        labelText+=std::to_string(msg.priority);
        labelText+="\n";
        std::cout << labelText << std::endl;
    }
    else
    {
        labelText+="Entity: Behavior\n";
    }

    QTableWidgetItem * event= ui->loggerTable->item(row,2);

    labelText+= "Event: "+event->text().toStdString()+" \n";    

    QTableWidgetItem * name= ui->loggerTable->item(row,3);

    labelText+= "Name: "+name->text().toStdString()+" \n";

    if(msg.parameters != "")
    {
        labelText+= "Parameters: "+msg.parameters+" \n";
    }


    if (msg.termination_cause )
    {
        labelText+="Termination cause: ";
        switch (msg.termination_cause)
        {
        case behavior_coordinator_msgs::BehaviorCoordinatorLog::GOAL_ACHIEVED:
            labelText+="Goal achieved\n";
            break;
        case behavior_coordinator_msgs::BehaviorCoordinatorLog::TIME_OUT:
            labelText+="Time out\n";
            break;
        case behavior_coordinator_msgs::BehaviorCoordinatorLog::WRONG_PROGRESS:
            labelText+="Wrong progress\n";
            break;
        case behavior_coordinator_msgs::BehaviorCoordinatorLog::PROCESS_FAILURE:
            labelText+="Process failure\n";
            break;
        case behavior_coordinator_msgs::BehaviorCoordinatorLog::INTERRUPTED:
            labelText+="Interrupted\n";
            break;
        case behavior_coordinator_msgs::BehaviorCoordinatorLog::REQUESTED_DEACTIVATION:
            labelText+="Requested deactivation\n";
            break;
        case behavior_coordinator_msgs::BehaviorCoordinatorLog::AUTOMATIC_DEACTIVATION:
            labelText+="Automatic deactivation\n";
            break;
        case behavior_coordinator_msgs::BehaviorCoordinatorLog::TIME_OUT_SELF_DEACTIVATION:
            labelText+="Time out self deactivation\n";
            break;
        case behavior_coordinator_msgs::BehaviorCoordinatorLog::IMPOSSIBLE_ACTIVATION:
            labelText+="The task could not be activated\n";
            break;
        case behavior_coordinator_msgs::BehaviorCoordinatorLog::TASK_NOT_FOUND:
            labelText+="The task has not been found\n";
            break;
        default:
            break;
        }
    }
    if (msg.event == behavior_coordinator_msgs::BehaviorCoordinatorLog::START_TASK_REACTIVE)
    {
        labelText+="This task was requested by the behavior_coordinator\n";
    }
    

    if(labelText != "")
    {
        label->setText(QString::fromStdString(labelText));
        dlg->show();
    }


}


void Behavior_Coordinator_Logger::resizeEvent(QResizeEvent* event)
    {

        ui->loggerTable->setColumnWidth(0, ui->loggerTable->width()*1.5 / 30);
        ui->loggerTable->setColumnWidth(1, ui->loggerTable->width()*1.5 / 30);
        ui->loggerTable->setColumnWidth(2, ui->loggerTable->width()*8.5 / 30);
        ui->loggerTable->setColumnWidth(3, ui->loggerTable->width()*12.5 / 30);
        ui->loggerTable->setColumnWidth(4, ui->loggerTable->width()*6 / 30);
        QWidget::resizeEvent(event);

    }

void Behavior_Coordinator_Logger::hideActiveBehaviors()
{
    if(showing)
    {
        ui->label->hide();
        ui->activeTable->hide();
        ui->hideButtom->setText(tr("Show active behaviors"));
    }
    else
    {
        ui->label->show();
        ui->activeTable->show();
        ui->hideButtom->setText(tr("Hide active behaviors"));
    }
    showing=!showing;
}

void Behavior_Coordinator_Logger::addNewActiveBehavior(std::string name, std::string parameters)
{

    std::cout << "anadiendo " << name << std::endl;
    QFont font;
    font.setPointSize(9);
    all_behaviors_active.push_back(name);
    ui->activeTable->setRowCount(active_behavior_counter+1);
    QTableWidgetItem *nameItem = new QTableWidgetItem(QString::fromStdString(name));
    nameItem->setFont(font);
    QTableWidgetItem *parametersItem = new QTableWidgetItem(QString::fromStdString(parameters));
    parametersItem->setFont(font);
    ui->activeTable->setItem(this->active_behavior_counter,0,nameItem);
    ui->activeTable->setItem(this->active_behavior_counter,1,parametersItem);
    ui->activeTable->item(this->active_behavior_counter, 1)-> setTextAlignment(Qt::AlignCenter);
    active_behavior_counter++;


}

void Behavior_Coordinator_Logger::deactiveActiveBehavior(std::string name)
{

    std::cout << "eliminando" << name << std::endl;
    int j =0;
	std::vector<std::string>::iterator it;
	bool found=false;
	for ( it = all_behaviors_active.begin() ; it != all_behaviors_active.end() && !found; ++it)
    {
    	if (*it ==name){
    		all_behaviors_active.erase(it);
    		found=true;
    	}
    	else
        {
    		j++;

    	}
    }
    ui->activeTable->removeRow(j);
	active_behavior_counter--;
    
}
void Behavior_Coordinator_Logger::pauseMessages()
{
    if( shutdonw)
    {
        ui->pauseButtom->setText("Continue log display");
        behavior_coordination_log_sub.shutdown();
    }    

    else 
    {
        ui->pauseButtom->setText("Pause log display");
        behavior_coordination_log_sub = n.subscribe("/" + drone_id_namespace + "/" + behavior_coordination_log, 1000, &Behavior_Coordinator_Logger::newBehaviorCoordiantorLogCallback, this);
    }   
        
    shutdonw=!shutdonw;
}
        