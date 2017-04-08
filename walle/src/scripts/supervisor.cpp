//Created 2017-03-17
//Updated 2017-03-24
//Updated 2017-04-06
//Updated 2017-04-07
//Node that monitors 1) QR codes and 2) Turtlebot states
//Can initiate state change request to navigation, auto-docking, dockDetect nodes
//Writes remaining explore time & QR code information to external text files
//Imports allowable explore time & previously captured QR information at startup
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

double allowable_time;

const int listSize = 50;
qrCode qrList[listSize];
int listCounter=0;

double xCurr;
double yCurr;
double thetaCurr;
double x_qrTol = 2.25;
double y_qrTol = 2.25;
double theta_qrTol = 135.0;

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
void importWordList();
void writeTime(double timeRemaining);
double importTime();

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

    importWordList();
    ROS_INFO_STREAM("Printing previously captured qr codes");
    printWordList();

    std_msgs::Bool msg_exploreON;
    std_msgs::Bool msg_dockON;
    std_msgs::Bool msg_dockSucceed;
    std_msgs::Float64 msg_yawAngle;

    allowable_time = importTime();
    ROS_INFO_STREAM("Allowable Explore Time ="<<allowable_time<<" [seconds]");

    ////////Explore State/////////////////////////////////////////////////////
    /*set explore state to OFF when the allowable explore time has been exceeded and
    at least 1 qr code has been found */
    ros::Rate rs1(10); //loop rate = 10 [Hz]
    double start_time = ros::Time::now().toSec();
    double current_time;

    ROS_INFO_STREAM("Current Robot State: EXPLORE");
    current_time=ros::Time::now().toSec();
    ROS_INFO_STREAM("Elasped Time = "<<(current_time-start_time)/60<<"[minutes]");
    while(!time_exceeded)
    {
        current_time=ros::Time::now().toSec();

        if(current_time-start_time > allowable_time)
        {
            time_exceeded = true;
            ROS_INFO_STREAM("Turning EXPLORE off");
        }

        //write the allowable explore time remaining to an external text file, every 60 [s]
        if((int)(current_time-start_time)%60  == 0)
        {
            ROS_INFO_STREAM("Elasped Time = "<<(current_time-start_time)/60<<"[minutes]");

            if(!time_exceeded)
            {
                writeTime(allowable_time-(current_time-start_time));
            }


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
    ///////////////////////////////////////////////////////////////////////

    saveWordList();

    ///////Return Home State///////////////////////////////////////////////
    //set return home state to OFF when the Turtlebot has returned to its home position
    ROS_INFO_STREAM("Current Robot State: RETURN_HOME");
    current_time=ros::Time::now().toSec();
    ROS_INFO_STREAM("Elasped Time = "<<(current_time-start_time)/60<<"[minutes]");
    while(!startDock)
    {

        ros::spinOnce();
        msg_dockON.data = startDock;
        pub_dockONStatus.publish(msg_dockON);
        msg_yawAngle.data = thetaCurr;
        pub_yawAngle.publish(msg_yawAngle);

        rs1.sleep();

    }
    //////////////////////////////////////////////////////////////////////

    saveWordList();

    ROS_INFO_STREAM("Turning RETURN_HOME off");
    printWordList();

    //////////Docking State/////////////////////////////////////////////////
    //set docking state to OFF when the Turtlebot has sucessfully docked
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
    /////////////////////////////////////////////////////////////////////////

    ROS_INFO_STREAM("Turning AUTODOCKING off");
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
           saveWordList();
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

	//convert negative angles to 0 < theta < 2*PI range
    //if(yaw < 0)
        //yaw = 2*M_PI + yaw;

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
            	double acuteAngleDiff = abs(qrList[i].angle-thetaCurr);

            	if (acuteAngleDiff>180.0)
            	{
            		acuteAngleDiff = abs(360.0 - acuteAngleDiff);
            	}

                if(acuteAngleDiff < theta_qrTol)
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
        std::cout<<"\n********************Mission Summary*********************";
        std::cout<<"\nNumber of Captured QR Codes = "<<listCounter;
        std::cout<<"\nIndex"<<"\t"<<"Word"<<"\t\t"<<"X Position"<<"\t"<<"Y Position"<<"\t"<<"Theta";
        for(int i = 0; i < listCounter; i++)
        {

            std::cout<<"\n"<<i+1<<"\t"<<qrList[i].word<<"\t\t"<<qrList[i].position_x<<"\t\t"<<qrList[i].position_y<<"\t\t"<<qrList[i].angle<<"\t\t";
        }
        std::cout<<"\n********************************************************\n";
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
        fout.open("/home/venu/catkin_ws/src/AER1514_project/walle/src/scripts/qrMasterList.txt"); //home/na052/catkin_ws/src/AER1514_project/walle/src/scripts/

        if(fout.is_open())
        {
            ROS_INFO_STREAM("Fout is open");

            for(int j=0;j<listCounter;j++)
            {

                fout<<qrList[j].word<<"\t\t"<<qrList[j].position_x<<"\t\t"<<qrList[j].position_y<<"\t\t"<<qrList[j].angle<<"\n";
            }

            fout.flush();
            fout.close();

            ROS_INFO_STREAM("QR list sucessfully saved to hard disk");
        }

    }
}

void importWordList()
{

    std::ifstream infile;
    infile.open("/home/venu/catkin_ws/src/AER1514_project/walle/src/scripts/qrMasterList.txt");

    double xPoint;
    double yPoint;
    double theta;
    std::string word;

    if(infile.is_open())
    {
        if(infile.peek() == std::ifstream::traits_type::eof())
        {

            ROS_INFO_STREAM("File is empty");
            infile.close();
        }
        else
        {

            ROS_INFO_STREAM("Importing previous words");
            while(infile>>word>>xPoint>>yPoint>>theta)
            {

                ROS_INFO_STREAM(word<<"\t"<<xPoint<<"\t"<<yPoint<<"\t"<<theta);

                qrList[listCounter].word=word;
                qrList[listCounter].position_x=xPoint;
                qrList[listCounter].position_y=yPoint;
                qrList[listCounter].angle=theta;
                listCounter++;

                //ROS_INFO_STREAM(word<<"\t"<<xPoint<<"\t"<<yPoint<<"\t"<<theta);
            }
            ROS_INFO_STREAM("Nmber of imported words= "<<listCounter);
            infile.close();
        }

    }

    else
    {
        ROS_WARN_STREAM("File is not open");
        infile.close();
    }

}


void writeTime(double timeRemaining)
{


        //ROS_INFO_STREAM("writing current time");
        std::ofstream ftime;
        ftime.open("/home/venu/catkin_ws/src/AER1514_project/walle/src/scripts/allowableTime.txt"); //home/na052/catkin_ws/src/AER1514_project/walle/src/scripts/

        if(ftime.is_open())
        {
            //ROS_INFO_STREAM("Fout is open");

            //ROS_INFO_STREAM("writing word "<<qrList[j].word);
            ftime<<timeRemaining;


            ftime.flush();
            ftime.close();


        }

}

double importTime()
{

    std::ifstream intime;
    intime.open("/home/venu/catkin_ws/src/AER1514_project/walle/src/scripts/allowableTime.txt");
    //  std::ios::ate

//    double time;
//    intime>>time;
//    ROS_INFO_STREAM("file is open with content ="<<time);


    if(intime.is_open())
    {
        if(intime.peek() == std::ifstream::traits_type::eof())
        {

            ROS_INFO_STREAM("File is empty");
            return 0;
        }
        else
        {
//        	ROS_INFO_STREAM("file is open and not empty");

            double time;

            intime>>time;

//            while(!intime.eof()){
//                ROS_INFO_STREAM("imported time ="<<time);
//                intime>>time;
//            }

            ROS_INFO_STREAM("Successfully imported time ="<<time);

            intime.close();
            return time;
        }

    }

    else
    {
        ROS_WARN_STREAM("File is not open");

        intime.close();
        return 0;
    }

    intime.close();
}


