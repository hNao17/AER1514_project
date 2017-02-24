#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>

//global variables
const double num_waypoints=45;
const double dim_waypoint=3;
double path [][3] = { {31.4,3.0,90.0},
	              {31.4,4.0,90.0},
		      {31.4,5.0,90.0},
                      {31.4,6.0,90.0},
                      {31.4,7.0,90.0},
		      {31.4,8.0,90.0},
		      {31.4,9.0,90.0},
		      {31.4,10.0,90.0},
		      {31.4,11.0,90.0},
		      {31.4,12.0,90.0},
		      {31.4,13.0,90.0},
		      {31.4,14.0,90.0},
		      {31.4,15.0,90.0},
		      {31.4,16.0,90.0},
		      {31.4,17.0,90.0},
		      {31.4,18.0,90.0},
		      {31.4,19.0,90.0},
		      {31.4,20.0,90.0},
   		      {31.4,21.0,90.0},
		      {31.4,22.0,90.0},
		      {31.4,23.0,90.0},
		      {31.4,24.0,90.0},
		      {31.4,25.0,90.0},
		      {31.4,26.0,90.0},
                      {31.4,27.0,90.0},
		      {31.4,28.0,90.0},
		      {30.4,28.0,180.0},
		      {29.4,28.0,180.0},
		      {28.4,28.0,180.0},
		      {27.4,28.0,180.0},
		      {26.4,28.0,180.0},
		      {25.4,28.0,180.0},
		      {24.4,28.0,180.0},
		      {23.4,28.0,180.0},
		      {22.4,28.0,180.0},
		      {21.4,28.0,180.0},
		      {20.4,28.0,180.0},
		      {19.4,28.0,180.0},
		      {18.4,28.0,180.0},
		      {17.4,28.0,180.0},
		      {16.4,28.0,180.0},
		      {15.4,28.0,180.0},
		      {14.4,28.0,180.0},
		      {13.4,28.0,180.0} };
			/*{13.4,	29.0,90},
			{13.4,	30.0,90},
			{13.4,	31.0,90},
			{13.4,	32.0,90},
			{13.4,	33,90},


			{13.4,	34.0,90},
			{13.4,	35.0,90},
			{13.4,	36.0,90},
			{13.4,	37.0,90},
			{13.4,	38.0,90},
			{8.9,	38.0,90},
			{8.9,	39.0,90},
			{8.9,	40.0,90},
			{8.9,	41.0,90},
			{8.9,	42.0,90},
			{8.9,	43.0,90},
			{8.9,	44.0,90},
			{8.9,	45.0,90},
			{8.9,	46.0,90},
			{8.9,	47.0,90},
			{8.9,	48.0,90},
			{8.9,	49.0,90},
			{8.9,	50.0,90},
			{8.9,	51.0,90} };*/

ros::Subscriber sub;
ros::Publisher pub_vel;
double x_current;
double y_current;

double scan_time=4.0;
double current_time=0.0;
double initial_time=0.0;

/** function declarations **/
void moveToGoal(double xGoal, double yGoal, double yawGoal);
void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped& msgAMCL);
void scanRotate();

int main(int argc, char** argv)
{
	ros::init(argc, argv, "nav_test2");
	ros::NodeHandle nh;

	//determine spawn location, check AMCL callback
	sub = nh.subscribe("amcl_pose",1000,poseAMCLCallback);
	pub_vel = nh.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity",100);	
	ros::spinOnce();

	for (int i=0; i<num_waypoints; i++)
	{
		//position command
		ROS_INFO_STREAM("Waypoint #"<<i);
		moveToGoal(path[i][0], path[i][1], path[i][2]);

		//determine position error
		ros::spinOnce(); //check AMCL Callback
		ROS_INFO_STREAM("X Position Error: "<<path[i][0]-x_current);
		ROS_INFO_STREAM("Y Position Error: "<<path[i][1]-y_current);		
		ROS_INFO_STREAM("Next destination");		
	}
}

void moveToGoal(double xGoal, double yGoal, double yawGoal)
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

	ROS_INFO("Sending goal location ...");
	ac.sendGoal(goal);

	ac.waitForResult(ros::Duration(10.0));

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("You have reached the destination");
		scanRotate();
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


