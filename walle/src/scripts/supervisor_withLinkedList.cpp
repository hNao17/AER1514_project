//Created 2017-03-17
//Node that monitors 1) QR codes and 2) Turtlebot states
//Can initiate state change request to navigation and auto-docking nodes
//Prints / speaks the list of qr words
////////////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include "qrLinkedList.h"
#include "qrLinkedList.cpp"

/** Global Variables **/
bool time_exceeded = false;
bool startDock = false;
bool printList = false;
double allowable_time = 600.0;

//ros::Subscriber sub_barcode;
ros::Subscriber sub_atHomeStatus;
ros::Subscriber sub_dockingStatus;
ros::Publisher pub_exploreStatus;
ros::Publisher pub_dockONStatus;



/** Function Declarations **/
//void barcode_callback(const std_msgs::String& zbarWord);
void atHome_callback(const std_msgs::Bool& msg_atHome);
void dockingStatus_callback(const std_msgs::Bool& msg_atDock);

int main(int argc, char** argv)
{
    ros::init(argc,argv,"supervisor_node");
    ros::NodeHandle nh3;

    ROS_INFO_STREAM("Starting Supervisor node");
    qrLinkedList* l=new qrLinkedList();
    //subscribe to barcode, atHomeStatus, dockingStatus
    //sub_barcode = nh3.subscribe("barcode",1000,barcode_callback);
    sub_atHomeStatus = nh3.subscribe("atHomeStatus",1000,atHome_callback);
    sub_dockingStatus = nh3.subscribe("dockingStatus",1000,dockingStatus_callback);

    //publish to exploreStatus, dockON_Status topics
    pub_exploreStatus = nh3.advertise<std_msgs::Bool>("/exploreStatus",1000);
    pub_dockONStatus = nh3.advertise<std_msgs::Bool>("/dockON_Status",1000);

    double start_time = ros::Time::now().toSec();
    double current_time;

    std_msgs::Bool msg_exploreON;
    std_msgs::Bool msg_dockON;

    //exploreState is ON
    //monitor incoming QR codes
    /*set exploreState to OFF when the allowable explore time has been exceeded and
    at least 1 qr code has been found */
    ros::Rate rate(10);
    while(!time_exceeded)
    {
        ROS_INFO_STREAM("Current Robot State: EXPLORE");
        current_time=ros::Time::now().toSec();
        ROS_INFO_STREAM("Elasped Time = "<<current_time-start_time<<"[s]");

        if(current_time-start_time > allowable_time)
        {
            time_exceeded = true;
            ROS_INFO_STREAM("Turning EXPLORE off");
        }

        msg_exploreON.data = time_exceeded;
        msg_dockON.data = startDock;

        pub_exploreStatus.publish(msg_exploreON);
        pub_dockONStatus.publish(msg_dockON);

        ros::spinOnce();

    }


    while(!startDock)
    {
        ROS_INFO_STREAM("Current Robot State: RETURN_HOME");
        ros::spinOnce();
        msg_dockON.data = startDock;
        pub_dockONStatus.publish(msg_dockON);

    }

    ROS_INFO_STREAM("Turning RETURN_HOME off");

    while(!printList)
    {
        ROS_INFO_STREAM("Current Robot State: AUTODOCKING");
        ros::spinOnce();

    }
    //robot is docked; print qr list
    ROS_INFO_STREAM("Turning AUTODOCKING off");
    ROS_INFO_STREAM("Current Robot State: PRINT_QRLIST");
    //l.printWordList();

    ros::spin();

}

/*void barcode_callback(const std_msgs::String& zbarWord)
{
    bool duplicate;
    qrNode* q=new qrNode;

    //only check "non-space" string values
    if(zbarWord.data!="")
    {
       duplicate = l.searchList(zbarWord.data);

       //only add the new word to the list if it hasn't already been found
       if(!duplicate)
       {
           q->position_x = -1;
           q->position_y = -1;
           q->word = zbarWord.data;

           l.insertNode(q);
       }
    }

    ROS_INFO_STREAM("# of QR Codes: "<<l.getSize());
}*/

void atHome_callback(const std_msgs::Bool& msg_atHome)
{
    if(!msg_atHome.data)
        ROS_INFO_STREAM("RETURN_HOME state is still active");
    else
        startDock = true;
}

void dockingStatus_callback(const std_msgs::Bool& msg_atDock)
{
    if(!msg_atDock.data)
        ROS_INFO_STREAM("AUTODOCKING state is still active");
    else
        printList = true;
}
