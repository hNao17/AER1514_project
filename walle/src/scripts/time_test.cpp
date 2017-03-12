#include <ros/ros.h>
#include "walle/exploreState.h"
//#include<rosgraph_msgs/Clock.h>
//#include <wall_timer.h>

bool time_exceeded = false;
double allowable_time = 50.0;
//initialise time_exceeded message publisher
ros::Publisher pub;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"time_test");
    ros::NodeHandle nh;

    pub=nh.advertise<walle::exploreState>("/publishMsg",1000);

    double start_time = ros::Time::now().toSec();
    double current_time;

    walle::exploreState msg;

    ros::Rate rate(10);
    while(time_exceeded==false)
    {
        current_time=ros::Time::now().toSec();

        if(current_time-start_time > allowable_time)
            time_exceeded = true;

        //publish message
        msg.return_home=time_exceeded;
        pub.publish(msg);
        ROS_INFO_STREAM("Elasped Time = "<<current_time-start_time<<"[s]");
    }

    ROS_INFO_STREAM("Allowable search time has been exceeded");
    //ros::spin();
}


