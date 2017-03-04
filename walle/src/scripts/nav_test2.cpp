#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <fstream>
#include <string>

/** global variables **/
const int num_waypoints=71;
const int dim_waypoint=3;
double path [num_waypoints][dim_waypoint];

ros::Subscriber sub;
double x_current;
double y_current;

const double x_home=31.5;
const double y_home=2.0;
const double theta_home=90.0;

/** function declarations **/
void readWaypoints();
void moveToGoal(double xGoal, double yGoal, double yawGoal);
void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped& msgAMCL);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "nav_test2");
	ros::NodeHandle nh;

    	//import waypoints from text file
	readWaypoints();

	//determine spawn location, check AMCL callback
	sub = nh.subscribe("amcl_pose",1000,poseAMCLCallback);
	ros::spinOnce();

	for (int i=0; i<num_waypoints; i++)
	{
		//position command
		ROS_INFO_STREAM("Waypoint #"<<i+1);
		moveToGoal(path[i][0], path[i][1], path[i][2]);

		//determine position error
		ros::spinOnce(); //check AMCL Callback
		ROS_INFO_STREAM("X Position Error: "<<path[i][0]-x_current);
		ROS_INFO_STREAM("Y Position Error: "<<path[i][1]-y_current);
		ROS_INFO_STREAM("Next destination");
	}

	//return home
    	ROS_INFO_STREAM("Exploration Complete. Going home.");
	moveToGoal(x_home,y_home,theta_home);
}

void readWaypoints()
{
    std::ifstream infile("/home/na052/AER1514/src/walle/src/scripts/masterWaypoints3.txt.csv");

    double xPoint;
    double yPoint;
    double theta;
    int waypointCounter=0;

    if(infile.is_open())
    {
        while(infile>>xPoint>>yPoint>>theta)
        {
            path[waypointCounter][0]=xPoint;
            path[waypointCounter][1]=yPoint;
            path[waypointCounter][2]=theta;
            waypointCounter++;
        }

        infile.close();

        for(int i=0; i < num_waypoints; i++)
        {
            ROS_INFO_STREAM("X: "<<path[i][0]<<", Y: "<<path[i][1]<<", Theta: "<<path[i][2]);
        }
    }

    else
    {
        ROS_INFO_STREAM("File is not open");
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

	/* moving towards the goal*/

	goal.target_pose.pose.position.x =  xGoal;
	goal.target_pose.pose.position.y =  yGoal;
	goal.target_pose.pose.position.z =  0.0;

	//convert degrees to quaternion
	if (yawGoal == 90.0)
	{
		goal.target_pose.pose.orientation.x = 0.0;
		goal.target_pose.pose.orientation.y = 0.0;
		goal.target_pose.pose.orientation.z = sqrt(2);
		goal.target_pose.pose.orientation.w = sqrt(2);
	}
	else if(yawGoal == 180.0)
	{
		goal.target_pose.pose.orientation.x = 0.0;
		goal.target_pose.pose.orientation.y = 0.0;
		goal.target_pose.pose.orientation.z = 1.0;
		goal.target_pose.pose.orientation.w = 0.009;
	}
    	else if(yawGoal == 135.0)
	{
		goal.target_pose.pose.orientation.x = 0.0;
		goal.target_pose.pose.orientation.y = 0.0;
		goal.target_pose.pose.orientation.z = 0.924;
		goal.target_pose.pose.orientation.w = 0.383;
	}

	ROS_INFO("Sending goal location ...");
	ac.sendGoal(goal);

	ac.waitForResult(ros::Duration(10.0));

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("You have reached the destination");
	}
	else
	{
		ROS_INFO("The robot failed to reach the destination");
	}

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
