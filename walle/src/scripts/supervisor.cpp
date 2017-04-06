//Created 2017-03-17
//Updated 2017-03-24
//Updated 2017-04-06
//Node that monitors 1) QR codes and 2) Turtlebot states
//Can initiate state change request to navigation, auto-docking, dockDetect nodes
//Prints / speaks the list of qr words
////////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include "qrCode.h"
#include <string>
#include <iostream>
#include <fstream>

/** Global Variables **/
bool time_exceeded = false;
bool startDock = false;
bool printList = false;

double allowable_time = 75.0;

const int listSize = 50;
qrCode qrList[listSize];
int listCounter=0;

double xCurr;
double yCurr;
double thetaCurr;
double x_qrTol = 1.5;
double y_qrTol = 1.5;
double theta_qrTol = 45.0;

const double rad2Degrees = 180/M_PI;

ros::Subscriber sub_barcodeWebcam;
ros::Subscriber sub_barcodeAstra;
ros::Subscriber sub_atHomeStatus;
ros::Subscriber sub_dockingStatus;
ros::Subscriber sub_robotPose;

ros::Publisher pub_exploreStatus;
ros::Publisher pub_dockONStatus;
ros::Publisher pub_dockSucceedStatus;
ros::Publisher pub_yawAngle;

/** Function Declarations **/
void barcode_callback(const std_msgs::String& zbarWord);
void atHome_callback(const std_msgs::Bool& msg_atHome);
void dockingStatus_callback(const std_msgs::Bool& msg_atDock);
void poseRobotCallback(const geometry_msgs::PoseWithCovarianceStamped& msgPose);
bool searchList(std::string word);
void printWordList();
void saveWordList();

int main(int argc, char** argv)
{
    ros::init(argc,argv,"supervisor_node");
    ros::NodeHandle nh3;

    ROS_INFO_STREAM("Starting Supervisor node");

    //subscribe to webcame/qrcode, astra/qrcode, atHomeStatus, dockingStatus, amcl_pose
    //sub_barcodeWebcam = nh3.subscribe("webcam/barcode",1000,barcode_callback);
    //sub_barcodeAstra = nh3.subscribe("astra/barcode",1000,barcode_callback);
    sub_barcodeWebcam = nh3.subscribe("webcam/qrcode",1000,barcode_callback); //zbar lite
    sub_barcodeAstra = nh3.subscribe("astra/qrcode",1000,barcode_callback); //zbar lite
    sub_atHomeStatus = nh3.subscribe("atHomeStatus",1000,atHome_callback);
    sub_dockingStatus = nh3.subscribe("dockingStatus",1000,dockingStatus_callback);
    sub_robotPose = nh3.subscribe("amcl_pose",1000,poseRobotCallback);

    //publish to exploreStatus, dockON_Status, dockSucceed_Status topics
    pub_exploreStatus = nh3.advertise<std_msgs::Bool>("/exploreStatus",1000);
    pub_dockONStatus = nh3.advertise<std_msgs::Bool>("/dockON_Status",1000);
    pub_dockSucceedStatus = nh3.advertise<std_msgs::Bool>("/dockSucceed_Status",1000);
    pub_yawAngle = nh3.advertise<std_msgs::Float64>("/yawAngle",1000);

    double start_time = ros::Time::now().toSec();
    double current_time;

    std_msgs::Bool msg_exploreON;
    std_msgs::Bool msg_dockON;
    std_msgs::Bool msg_dockSucceed;
    std_msgs::Float64 msg_yawAngle;

    //exploreState is ON
    //monitor incoming QR codes
    /*set exploreState to OFF when the allowable explore time has been exceeded and
    at least 1 qr code has been found */

    //Explore State
    ros::Rate rs1(10); //loop rate = 10 [Hz]
    ROS_INFO_STREAM("Current Robot State: EXPLORE");
    current_time=ros::Time::now().toSec();
    ROS_INFO_STREAM("Elasped Time = "<<(current_time-start_time)/60<<"[minutes]");
    while(!time_exceeded)
    {
        current_time=ros::Time::now().toSec();
        //ROS_INFO_STREAM("Elasped Time = "<<current_time-start_time<<"[s]");
        //ROS_INFO_STREAM("Number of Words = "<<listCounter);

        if(current_time-start_time > allowable_time)
        {
            time_exceeded = true;
            ROS_INFO_STREAM("Turning EXPLORE off");
        }

        msg_exploreON.data = time_exceeded;
        msg_dockON.data = startDock;
        msg_yawAngle.data = thetaCurr;

        pub_exploreStatus.publish(msg_exploreON);
        pub_dockONStatus.publish(msg_dockON);
        pub_yawAngle.publish(msg_yawAngle);
        ros::spinOnce();

        rs1.sleep();

    }

    //Return Home State
    ROS_INFO_STREAM("Current Robot State: RETURN_HOME");
    current_time=ros::Time::now().toSec();
    ROS_INFO_STREAM("Elasped Time = "<<(current_time-start_time)/60<<"[minutes]");
    while(!startDock)
    {
        //ROS_INFO_STREAM("Number of Words = "<<listCounter);

        ros::spinOnce();
        msg_dockON.data = startDock;
        pub_dockONStatus.publish(msg_dockON);
        msg_yawAngle.data = thetaCurr;
        pub_yawAngle.publish(msg_yawAngle);
        rs1.sleep();

    }

    saveWordList();
    ROS_INFO_STREAM("Turning RETURN_HOME off");
    printWordList();

    //Docking State
    ROS_INFO_STREAM("Current Robot State: AUTODOCKING");
    current_time=ros::Time::now().toSec();
    ROS_INFO_STREAM("Elasped Time = "<<(current_time-start_time)/60<<"[minutes]");
    while(!printList)
    {
        ros::spinOnce();
        msg_dockSucceed.data = printList;
        pub_dockSucceedStatus.publish(msg_dockSucceed);

        rs1.sleep();

    }
    //robot is docked; print qr list
    ROS_INFO_STREAM("Turning AUTODOCKING off");
    //ROS_INFO_STREAM("Current Robot State: PRINT_QRLIST");
    ROS_INFO_STREAM("Turtlebot mission is complete");

    current_time=ros::Time::now().toSec();
    ROS_INFO_STREAM("Total Mission Time = "<<(current_time-start_time)/60<<"[minutes]");

    ros::spin();

}

void barcode_callback(const std_msgs::String& zbarWord)
{
    bool duplicate;
    int i=0;

    //only check "non-space" string values
    if(zbarWord.data!="")
    {
      duplicate = searchList(zbarWord.data);

       //only add the new word to the list if it hasn't already been found
       if(!duplicate)
       {
           qrList[listCounter].word=zbarWord.data;
           qrList[listCounter].position_x=xCurr;
           qrList[listCounter].position_y=yCurr;
           qrList[listCounter].angle=thetaCurr;
           ROS_INFO_STREAM("Successfully added "<<qrList[listCounter].word<<"at ("<<qrList[listCounter].position_x<<"'"<<qrList[listCounter].position_y<<")"<<" to list");
           listCounter++;
           ROS_INFO_STREAM("Number of Words = "<<listCounter);
       }
    }

    ROS_INFO_STREAM("# of QR Codes: "<<listCounter);
}

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

void poseRobotCallback(const geometry_msgs::PoseWithCovarianceStamped& msgPose)
{
    /*ROS_INFO_STREAM("Current turtlebot position: ("
                    <<msgAMCL.pose.pose.position.x <<","
                    <<msgAMCL.pose.pose.position.y <<","
                    <<msgAMCL.pose.pose.position.z <<")");*/


	xCurr = msgPose.pose.pose.position.x;
	yCurr = msgPose.pose.pose.position.y;

    tf::Quaternion qCurr (msgPose.pose.pose.orientation.x,
                          msgPose.pose.pose.orientation.y,
             		      msgPose.pose.pose.orientation.z,
               		      msgPose.pose.pose.orientation.w);

	tf::Matrix3x3 m(qCurr);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	thetaCurr = yaw*rad2Degrees;
}

bool searchList(std::string word)
{
    if(listCounter==0)
    {
        ROS_INFO_STREAM("List is empty.");
        return false;
    }

    for(int i = 0; i < listCounter; i++)
    {
        if(qrList[i].word==word)
        {
            if( abs(qrList[i].position_x-xCurr) < x_qrTol && abs(qrList[i].position_y-yCurr) < y_qrTol)
            {
                if(abs(qrList[i].angle-thetaCurr) < theta_qrTol)
                {
                    ROS_INFO_STREAM(word<<" already exists in list at current Turtlebot position. Do not add.");
                    return true;
                }

            }

        }
    }

    ROS_INFO_STREAM(word<<" does not exist in list");
    return false;

}

void printWordList()
{
    if(listCounter==0)
        ROS_INFO_STREAM("List is empty");
    else
    {
        ROS_INFO_STREAM("Word"<<"\t"<<"X Position"<<"\t"<<"Y Position"<<"\t"<<"Theta");
        for(int i = 0; i < listCounter; i++)
        {
            //ROS_INFO_STREAM("Word "<<i+1<<": "<<qrList[i].word);
            ROS_INFO_STREAM(qrList[i].word<<"\t"<<qrList[i].position_x<<"\t\t"<<qrList[i].position_y<<"\t\t"<<qrList[i].angle<<"\t\t");
        }
    }

}

void saveWordList()
{
    ROS_INFO_STREAM("Trying to save qr list");

    if(listCounter >= 1)
    {
        //std::ofstream fout("home/na052/catkin_ws/src/AER1514_project/walle/src/scripts/qrList.txt");
        ROS_INFO_STREAM("List has more than one qr code");
        std::ofstream fout;
        fout.open("qrMasterList.txt"); //home/na052/catkin_ws/src/AER1514_project/walle/src/scripts/

        if(fout.is_open())
        {
            ROS_INFO_STREAM("Fout is open");

            for(int j=0;j<listCounter-1;j++)
            {
                fout<<"\n"<<qrList[j].word;
            }

            fout.close();

            ROS_INFO_STREAM("QR list sucessfully saved to hard disk");
        }

    }
}
