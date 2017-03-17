#include <ros/ros.h>
//#include "walle/exploreState.h"
#include <std_msgs/Bool.h>

ros::Subscriber sub;

//void customMsg_callback(const walle::exploreState& msg);
void customMsg_callback(const std_msgs::Bool& msg);

int main(int argc, char** argv)
{
    ros::init(argc,argv,"time_test_subscriber");
    ros::NodeHandle nh;

    sub=nh.subscribe("exploreStatus",1000,customMsg_callback);

    ros::Rate rate(1);
    ros::spin();
}

//void customMsg_callback(const walle::exploreState& msg)
void customMsg_callback(const std_msgs::Bool& msg)
{
    //if(msg.return_home==false)
    if(!msg.data)
        ROS_INFO_STREAM("Robot is still exploring environment");
    else
        ROS_INFO_STREAM("Robot is returning to docking station");
}

