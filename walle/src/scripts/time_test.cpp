#include <ros/ros.h>
//#include<rosgraph_msgs/Clock.h>
//#include <wall_timer.h>

bool time_exceeded = false;
double allowable_time = 50.0;
//initialise time_exceeded message publisher

int main(int argc, char** argv)
{
    ros::init(argc,argv,"time_test");
    ros::NodeHandle nh;

    double start_time = ros::Time::now().toSec();
    double current_time;

    //ros::Rate rate(0.1);
    while(time_exceeded==false)
    {
        current_time=ros::Time::now().toSec();

        if(current_time-start_time > allowable_time)
            time_exceeded = true;

        //publish message

        ROS_INFO_STREAM("Elasped Time = "<<current_time-start_time<<"[s]");
    }

    ROS_INFO_STREAM("Allowable search time has been exceeded");
    //ros::spin();
}


