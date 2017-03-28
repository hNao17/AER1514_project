//Created 2017-03-17
//Updated 2017-03-24
//Node that allows Turtlebot to explore UTIAS
//Uses an a priori set of waypoints that are read in from a text file
///////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <fstream>
#include <string>
#include <tf/tf.h>
#include <std_msgs/Bool.h>

/**Global Variables**/
const int num_waypoints=50;
const int dim_waypoint=3;
double path [num_waypoints][dim_waypoint];

bool exploreON = true;
bool returnHome = false;
bool atHome= false;

const double x_home=31.5;
const double y_home=4.0;
const double theta_home=0.0;

ros::Subscriber sub_pose;
ros::Subscriber sub_exploreStatus;
ros::Publisher pub_atHomeStatus;
ros::Publisher pub_velNav;

double x_current;
double y_current;

double vel_x=-0.2;

/** function declarations **/
void readWaypoints();
void moveToGoal(double xGoal, double yGoal, double yawGoal);
static tf::Quaternion toQuaternion(double pitch, double roll, double yaw);
void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped& msgAMCL);
void exploreStatus_callback(const std_msgs::Bool& msg_exploreStatus);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "navigation_node");
	ros::NodeHandle nh1;

    //import waypoints from text file
	readWaypoints();

	//subscribers to AMCL_pose, exploreStatus topics
	sub_pose = nh1.subscribe("amcl_pose",1000,poseAMCLCallback);
	sub_exploreStatus = nh1.subscribe("exploreStatus",1000,exploreStatus_callback);

	//publish atHome_status to supervisor node
	pub_atHomeStatus = nh1.advertise<std_msgs::Bool>("/atHomeStatus",1000);
	pub_velNav = nh1.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity",100);

	//Turtlebot moves backwards away from docking station
	ROS_INFO_STREAM("Backing away from docking station");
	double timeStart = ros::Time::now().toSec();
	double timeCurrent;
    geometry_msgs::Twist msg_vel;
	while(timeCurrent - timeStart < 3.0)
    {


        msg_vel.linear.x = vel_x;
        msg_vel.angular.z = 0;

        pub_velNav.publish(msg_vel);

        timeCurrent = ros::Time::now().toSec();

    }

    vel_x =0;
    msg_vel.linear.x = vel_x;
    pub_velNav.publish(msg_vel);
	ros::spinOnce();

	int i=0;

	//main exploration loop
    while(i<num_waypoints && exploreON==true)
	{
		//position command
		ROS_INFO_STREAM("Waypoint #"<<i+1);
		moveToGoal(path[i][0], path[i][1], path[i][2]);

		//determine position error
		ros::spinOnce(); //check AMCL Callback
		ROS_INFO_STREAM("X Position Error: "<<path[i][0]-x_current);
		ROS_INFO_STREAM("Y Position Error: "<<path[i][1]-y_current);
		ROS_INFO_STREAM("Next destination");

		i++;
	}

    ROS_INFO_STREAM("Number of waypoints visted: "<<i);
    returnHome = true;

    while(!atHome)
    {
        ROS_INFO_STREAM("Exploration complete. Going home.");
        moveToGoal(x_home,y_home,theta_home);
    }


    std_msgs::Bool msg_atHome;
    msg_atHome.data=atHome;
    pub_atHomeStatus.publish(msg_atHome);

    //navigation is complete; leave node in idle state
    ROS_INFO_STREAM("Navigation node is finished. Entering stand-by mode");
    ros::spin();

}

void readWaypoints()
{
    std::ifstream infile("/home/na052/catkin_ws/src/AER1514_project/walle/src/scripts/waypoint_textfiles/masterWaypoints8.txt.csv");

    double xPoint;
    double yPoint;
    double theta;
    int waypointCounter=0;

    if(infile.is_open())
    {
        //store the (x,y,theta) deta from each line into the path array
        while(infile>>xPoint>>yPoint>>theta)
        {
            path[waypointCounter][0]=xPoint;
            path[waypointCounter][1]=yPoint;
            path[waypointCounter][2]=theta;
            waypointCounter++;
        }

        infile.close();

        /*for(int i=0; i < num_waypoints; i++)
        {
            ROS_INFO_STREAM("X: "<<path[i][0]<<", Y: "<<path[i][1]<<", Theta: "<<path[i][2]);
        }*/
    }

    else
    {
        ROS_WARN_STREAM("File is not open");
    }
}

void moveToGoal(double xGoal, double yGoal, double yawGoal)
{
    //define a client for to send goal requests to the move_base server through a SimpleActionClient
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;

	//set up the frame parameters
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	//store desired (x,y,theta) values in move_base message
	goal.target_pose.pose.position.x =  xGoal;
	goal.target_pose.pose.position.y =  yGoal;
	goal.target_pose.pose.position.z =  0.0;

	tf::Quaternion qQR;
    qQR = toQuaternion(0,0,yawGoal);
    quaternionTFToMsg(qQR, goal.target_pose.pose.orientation); // stores qQR in goal orientation

	ROS_INFO("Sending goal location ...");
	ac.sendGoal(goal);

	//only cancel goal while the Turtlebot is in an explore state
	if(exploreON==false && returnHome==false)
	{
	    ac.cancelGoal();
	    ROS_WARN_STREAM("Canceling goal b/c allowable search time has been exceeded.");
	}

    else
    {
        //give the Turtlebot an unlimited amount of time to return home
        if(returnHome==true)
            ac.waitForResult();
        else
            ac.waitForResult(ros::Duration(75.0));

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("You have reached the destination");

            if(returnHome)
                atHome = true;
        }
        else
        {
            ROS_INFO("The robot failed to reach the destination");
        }
    }

}

static tf::Quaternion toQuaternion(double pitch, double roll, double yaw)
{
    //Quaterniond q;
	double t0 = std::cos(yaw * 0.5);
	double t1 = std::sin(yaw * 0.5);
	double t2 = std::cos(roll * 0.5);
	double t3 = std::sin(roll * 0.5);
	double t4 = std::cos(pitch * 0.5);
	double t5 = std::sin(pitch * 0.5);

//	q.w() = t0 * t2 * t4 + t1 * t3 * t5;
//	q.x() = t0 * t3 * t4 - t1 * t2 * t5;
//	q.y() = t0 * t2 * t5 + t1 * t3 * t4;
//	q.z() = t1 * t2 * t4 - t0 * t3 * t5;
	tf::Quaternion q(t0 * t3 * t4 - t1 * t2 * t5,t0 * t2 * t5 + t1 * t3 * t4,t1 * t2 * t4 - t0 * t3 * t5,t0 * t2 * t4 + t1 * t3 * t5);
	return q;

}

void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped& msgAMCL)
{
    ROS_INFO_STREAM("Current turtlebot position: ("
                    <<msgAMCL.pose.pose.position.x <<","
                    <<msgAMCL.pose.pose.position.y <<","
                    <<msgAMCL.pose.pose.position.z <<")");


	x_current = msgAMCL.pose.pose.position.x;
	y_current = msgAMCL.pose.pose.position.y;
}

void exploreStatus_callback(const std_msgs::Bool& msg_exploreStatus)
{

    if(!msg_exploreStatus.data)
    {
        exploreON = true;
        ROS_INFO_STREAM("Explore state is still active. Continue searching for QR codes");
    }

    else
    {
        exploreON = false;
        ROS_INFO_STREAM("Allowable explore time is over. Return to docking station");
    }


}
