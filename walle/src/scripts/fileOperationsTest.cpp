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

double allowable_time;

const int listSize = 50;
qrCode qrList[listSize];
int listCounter=0;

double xCurr;
double yCurr;
double thetaCurr;
double x_qrTol = 1.5;
double y_qrTol = 1.5;
double theta_qrTol = 90.0;

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
	ros::init(argc,argv,"fileOper");

	//while(1)
	//{
//		importTime();
	importWordList();
        ros::spinOnce();
	//}
        
}
    
double importTime(){
    
    std::ifstream intime;
    intime.open("/home/venu/catkin_ws/src/AER1514_project/walle/src/scripts/allowableTime.txt");
    //  std::ios::ate

//    double time;
//    intime>>time;
//    ROS_INFO_STREAM("file is open with content ="<<time);


    if(intime.is_open())
    {
        if(intime.peek() == std::ifstream::traits_type::eof()){
            
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

void importWordList(){

    std::ifstream infile;
    infile.open("/home/venu/catkin_ws/src/AER1514_project/walle/src/scripts/qrMasterList.txt");

    double xPoint;
    double yPoint;
    double theta;
    std::string word;



    if(infile.is_open())
    {
        if(infile.peek() == std::ifstream::traits_type::eof()){

            ROS_INFO_STREAM("file is empty");
            infile.close();
        }
        else
        {

            ROS_INFO_STREAM("importing previous words");
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
            ROS_INFO_STREAM("number of imported words: "<<listCounter);
            infile.close();
        }

    }

    else
    {
        ROS_WARN_STREAM("File is not open");
        infile.close();
    }

}
