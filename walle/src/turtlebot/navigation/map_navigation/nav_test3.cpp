#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

//global variables
double x_des;
double y_des;
double x_current;
double y_current;
double wp_time=100.0;
double scan_time=4.0;
double current_time=0.0;
double initial_time=0.0;
bool detect_collision;

ros::Publisher pub_vel;
ros::Subscriber sub_pose;
ros::Subscriber sub_odom; 

/** function declarations **/
void moveToGoal(double xGoal, double yGoal);
void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped& msgAMCL);
void scanRotate();

//void odomCallback(const nav_msgs::Odometry& msgOdom);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "nav_test3");
	ros::NodeHandle nh;

	pub_vel = nh.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity",100);

	//determine spawn location, check AMCL callback
	sub_pose = nh.subscribe("amcl_pose",1000,poseAMCLCallback);	
	//sub_odom = nh.subscribe("odom",1000,odomCallback);	
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
	goal.target_pose.pose.orientation.z = sqrt(2);
	goal.target_pose.pose.orientation.w = sqrt(2);

	ROS_INFO("Sending goal location ...");

	ac.sendGoal(goal);

	initial_time=ros::Time::now().toSec();
	ac.waitForResult(ros::Duration(10.0));

	//ros::spinOnce(); //check odomCallback

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		current_time = ros::Time::now().toSec();
		ROS_INFO("You have reached the destination");
		scanRotate(); //initiate QR detect scan
	}
	else
	{
		ac.cancelGoal();
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

void scanRotate()
{
	geometry_msgs::Twist vel_msg;
	vel_msg.linear.x =0;	
	vel_msg.angular.z=M_PI/2;

	initial_time = ros::Time::now().toSec();
	ROS_INFO_STREAM("Initating QR detect");
	while ((current_time-initial_time) < scan_time)
	{
		current_time = ros::Time::now().toSec();
		
		pub_vel.publish(vel_msg);
	}
	
	ROS_INFO_STREAM("Finished QR detect");
}
/*void odomCallback(const nav_msgs::Odometry& msgOdom)
{

	if ( abs(msgOdom.twist.twist.linear.x) < 0.05 )
	{
		detect_collision = true;
		ROS_ERROR_STREAM("Warning: Collision occured!");
	}
	else
	{
		ROS_INFO_STREAM("Current x velocity = "<< msgOdom.twist.twist.linear.x);
		detect_collision = false;
	}

}*/

