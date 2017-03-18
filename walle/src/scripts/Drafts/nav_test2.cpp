#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <fstream>
#include <string>
#include <tf/tf.h>

/** global variables **/
const int num_waypoints=10;
const int dim_waypoint=3;
double path [num_waypoints][dim_waypoint];

ros::Subscriber sub;
double x_current;
double y_current;

const double x_home=31.5;
const double y_home=4.5;
const double theta_home=90.0;

bool returnHome=false;

/** function declarations **/
void readWaypoints();
void moveToGoal(double xGoal, double yGoal, double yawGoal);
static tf::Quaternion toQuaternion(double pitch, double roll, double yaw);
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
	returnHome = true;
    ROS_INFO_STREAM("Exploration Complete. Going home.");
	moveToGoal(x_home,y_home,theta_home);
	ros::spinOnce();
}

void readWaypoints()
{
    std::ifstream infile("/home/na052/catkin_ws/src/AER1514_project/walle/src/scripts/waypoint_textfiles/masterWaypoints7.txt.csv");

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
    tf::Quaternion qQR;
    qQR = toQuaternion(0,0,yawGoal);
    quaternionTFToMsg(qQR, goal.target_pose.pose.orientation); // stores qQR in goal orientation

	ROS_INFO("Sending goal location ...");
	ac.sendGoal(goal);

    if(returnHome==false)
        ac.waitForResult(ros::Duration(30.0));
    else
        ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("You have reached the destination");
	}
	else
	{
		ROS_INFO("The robot failed to reach the destination");
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
