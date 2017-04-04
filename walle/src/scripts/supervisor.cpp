//Created 2017-03-17
//Updated 2017-03-24
//Node that monitors 1) QR codes and 2) Turtlebot states
//Can initiate state change request to navigation, auto-docking, dockDetect nodes
//Prints / speaks the list of qr words
////////////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <string>

/** Global Variables **/
bool time_exceeded = false;
bool startDock = false;
bool printList = false;

double allowable_time = 600.0;

const int listSize = 50;
std::string qrList[listSize];
int listCounter=0;

ros::Subscriber sub_barcodeWebcam;
ros::Subscriber sub_barcodeAstra;
ros::Subscriber sub_atHomeStatus;
ros::Subscriber sub_dockingStatus;
ros::Publisher pub_exploreStatus;
ros::Publisher pub_dockONStatus;
ros::Publisher pub_dockSucceedStatus;

/** Function Declarations **/
void barcode_callback(const std_msgs::String& zbarWord);
void atHome_callback(const std_msgs::Bool& msg_atHome);
void dockingStatus_callback(const std_msgs::Bool& msg_atDock);
bool searchList(std::string word);
void printWordList();

int main(int argc, char** argv)
{
    ros::init(argc,argv,"supervisor_node");
    ros::NodeHandle nh3;

    ROS_INFO_STREAM("Starting Supervisor node");

    //subscribe to barcode, atHomeStatus, dockingStatus
    //sub_barcodeWebcam = nh3.subscribe("webcam/barcode",1000,barcode_callback);
    //sub_barcodeAstra = nh3.subscribe("astra/barcode",1000,barcode_callback);
    sub_barcodeWebcam = nh3.subscribe("webcam/qrcode",1000,barcode_callback); //zbar lite
    sub_barcodeAstra = nh3.subscribe("astra/qrcode",1000,barcode_callback); //zbar lite
    sub_atHomeStatus = nh3.subscribe("atHomeStatus",1000,atHome_callback);
    sub_dockingStatus = nh3.subscribe("dockingStatus",1000,dockingStatus_callback);

    //publish to exploreStatus, dockON_Status, dockSucceed_Status topics
    pub_exploreStatus = nh3.advertise<std_msgs::Bool>("/exploreStatus",1000);
    pub_dockONStatus = nh3.advertise<std_msgs::Bool>("/dockON_Status",1000);
    pub_dockSucceedStatus = nh3.advertise<std_msgs::Bool>("/dockSucceed_Status",1000);

    double start_time = ros::Time::now().toSec();
    double current_time;

    std_msgs::Bool msg_exploreON;
    std_msgs::Bool msg_dockON;
    std_msgs::Bool msg_dockSucceed;

    //exploreState is ON
    //monitor incoming QR codes
    /*set exploreState to OFF when the allowable explore time has been exceeded and
    at least 1 qr code has been found */
    ros::Rate rs1(10); //loop rate = 10 [Hz]
    ROS_INFO_STREAM("Current Robot State: EXPLORE");
    while(!time_exceeded)
    {
        current_time=ros::Time::now().toSec();
        //ROS_INFO_STREAM("Elasped Time = "<<current_time-start_time<<"[s]");
        ROS_INFO_STREAM("Number of Words = "<<listCounter);

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

        rs1.sleep();

    }


    ROS_INFO_STREAM("Current Robot State: RETURN_HOME");
    while(!startDock)
    {
        ROS_INFO_STREAM("Number of Words = "<<listCounter);

        ros::spinOnce();
        msg_dockON.data = startDock;
        pub_dockONStatus.publish(msg_dockON);

        rs1.sleep();

    }

    ROS_INFO_STREAM("Turning RETURN_HOME off");

    printWordList();

    ROS_INFO_STREAM("Current Robot State: AUTODOCKING");
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
           qrList[listCounter]=zbarWord.data;
           ROS_INFO_STREAM("Successfully added "<<qrList[listCounter]<<" to list");
           listCounter++;
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

bool searchList(std::string word)
{
    if(listCounter==0)
    {
        ROS_INFO_STREAM("List is empty.");
        return false;
    }

    for(int i = 0; i < listCounter; i++)
    {
        if(qrList[i]==word)
        {
            ROS_INFO_STREAM(word<<" already exists in list. Do not add.");
            return true;
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
        for(int i = 0; i < listCounter; i++)
        {
            ROS_INFO_STREAM("Word "<<i+1<<": "<<qrList[i]);
        }
    }

}
