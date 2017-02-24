#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

//global variables
double x_des;
double y_des;
double x_current;
double y_current;

ros::Subscriber sub;
 

/** function declarations **/
void moveToGoal(double xGoal, double yGoal);
void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped& msgAMCL);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "nav_test1");
	ros::NodeHandle nh;

	//determine spawn location, check AMCL callback
	sub = nh.subscribe("amcl_pose",1000,poseAMCLCallback);	
	ros::spinOnce();

	while(ros::ok())
	{
		//user enters desired xy position
		std::cout<<"Enter destination x-coordinate: ";
		std::cin>>x_des;
		
		std::cout<<"Enter destination y-coordinate: ";
		std::cin>>y_des; 
		
		//position command
		moveToGoal(x_des, y_des); 

		//determine position error
		ros::spinOnce(); //check AMCL Callback
		ROS_INFO_STREAM("X Position Error: "<<x_des-x_current);
		ROS_INFO_STREAM("Y Position Error: "<<y_des-y_current);

		ROS_INFO_STREAM("Next destination");
	}

}

void moveToGoal(double xGoal, double yGoal)
{

	//define a client for to send goal requests to the move_base server through a SimpleActionClient
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
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
	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = 0.0;
	goal.target_pose.pose.orientation.w = 1.0;

	ROS_INFO("Sending goal location ...");
	ac.sendGoal(goal);

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

void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped& msgAMCL)
{

	/*tf::Quaternion q (odom.pose.pose.orientation.x, 
			  odom.pose.pose.orientation.y,
			  odom.pose.pose.orientation.z,
			  odom.pose.pose.orientation.w);

	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);*/

	ROS_INFO_STREAM("Current turtlebot position: ("
                               <<msgAMCL.pose.pose.position.x <<","
			       <<msgAMCL.pose.pose.position.y <<","
			       <<msgAMCL.pose.pose.position.z <<")");

	/*ROS_INFO_STREAM("Current turtlebot orientation: (0,0,"
                               <<yaw*r2D<<")"); */

	x_current = msgAMCL.pose.pose.position.x;
	y_current = msgAMCL.pose.pose.position.y;
}

