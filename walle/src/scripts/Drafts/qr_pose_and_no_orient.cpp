
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

//global variables
const double num_waypoints=6;
const double dim_waypoint=3;
double path [][3] = { {31.4,3.0,90.0},
	              {31.4,4.0,90.0},
		      {31.4,5.0,90.0},
                      {31.4,6.0,90.0},
                      {31.4,7.0,90.0},
		      {31.4,8.0,90.0}};


ros::Subscriber sub;
ros::Subscriber sub_amcl;
ros::Publisher pub_vel;
double x_current;
double y_current;

double scan_time=4.0;
double current_time=0.0;
double initial_time=0.0;


bool qrDetect = false;

// the point, in the camera frame
//tf::Vector3 point(x, y, z);
tf::Vector3 qr_rel_pose(0, 0, 0);
tf::Vector3 base_pose(0, 0, 0);

// request the transform between the two frames
//tf::TransformListener listener;
//tf::StampedTransform transform;

// the time to query the server is the stamp of the incoming message
//ros::Time time = cloud_cup->header.stamp;

// frame of the incoming message
//std::string frame = cloud_cup->header.frame_id;



/** function declarations **/
void moveToGoal(double xGoal, double yGoal, double yawGoal);
void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped& msgAMCL);
void vispStatusCallback(const std_msgs::Int8& msgVispStatus);
void vispPoseCallback(const geometry_msgs::PoseStamped& msgVispPose);
void scanRotate();

int main(int argc, char** argv)
{
	ros::init(argc, argv, "qr_pose");
	ros::NodeHandle nh;

	// request the transform between the two frames
	tf::TransformListener listener;
	tf::StampedTransform transform;

	// the point, in the camera frame
	//tf::Vector3 point(1.0, 0, 0);

	ros::Rate rate(2);

	//determine spawn location, check AMCL callback
	sub = nh.subscribe("/visp_auto_tracker/object_position",1000,vispPoseCallback);
	sub_amcl = nh.subscribe("amcl_pose",1000,poseAMCLCallback);
	//pub_vel = nh.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity",100);
	ros::spinOnce();

/*
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

*/

	while(ros::ok())
	{

		try{
		  listener.waitForTransform( "map","camera_depth_optical_frame", ros::Time(0), ros::Duration(3.0));
		  listener.lookupTransform("map","camera_depth_optical_frame", ros::Time(0), transform);
		}
		catch (tf::TransformException ex) {
		  ROS_WARN("Base to camera transform unavailable %s", ex.what());
		}

		// the point, in the base link frame
		tf::Vector3 qr_pose = transform * qr_rel_pose;
		tf::Vector3 approach_vector = (base_pose-qr_pose);
		approach_vector.setZ(0.0);	// Neglect the height difference
		approach_vector = approach_vector/approach_vector.length(); // Normalize to unit vector
		tf::Vector3 qr_goal = qr_pose + 1*approach_vector;
		qr_goal.setZ(0.0); //Neglect the Height

		ROS_INFO_STREAM("Camera X in MAP Frame :"<<transform.getOrigin().x());
		ROS_INFO_STREAM("Camera Y in MAP Frame :"<<transform.getOrigin().y());
		ROS_INFO_STREAM("Camera Z in MAP Frame :"<<transform.getOrigin().z());
		ROS_INFO_STREAM("QR X in Camera Frame :"<<qr_rel_pose.x());
		ROS_INFO_STREAM("QR Y in Camera Frame :"<<qr_rel_pose.y());
		ROS_INFO_STREAM("QR Z in Camera Frame :"<<qr_rel_pose.z());
		ROS_INFO_STREAM("QR X in MAP Frame :"<<qr_pose.x());
		ROS_INFO_STREAM("QR Y in MAP Frame :"<<qr_pose.y());
		ROS_INFO_STREAM("QR Z in MAP Frame :"<<qr_pose.z());
		ROS_INFO_STREAM("Base X in MAP Frame :"<<base_pose.x());
		ROS_INFO_STREAM("Base Y in MAP Frame :"<<base_pose.y());
		ROS_INFO_STREAM("Base Z in MAP Frame :"<<base_pose.z());
		ROS_INFO_STREAM("QR Goal X in MAP Frame :"<<qr_goal.x());
		ROS_INFO_STREAM("QR Goal Y in MAP Frame :"<<qr_goal.y());
		ROS_INFO_STREAM("QR Goal Z in MAP Frame :"<<qr_goal.z());
		ROS_INFO_STREAM("=====");
		ros::spinOnce();
		rate.sleep();
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

	base_pose.setX(msgAMCL.pose.pose.position.x);
	base_pose.setY(msgAMCL.pose.pose.position.y);
	base_pose.setZ(0.0);
}

void scanRotate()
{
	geometry_msgs::Twist vel_msg;
	vel_msg.linear.x =0;
	vel_msg.angular.z=M_PI/2;

	initial_time = ros::Time::now().toSec();
	ROS_INFO_STREAM("Initating QR detect");
	while ((current_time-initial_time) < scan_time || qrDetect == true)
	{
		current_time = ros::Time::now().toSec();
		pub_vel.publish(vel_msg);
		//check if visp has detected qr code
		ros::spinOnce();

	}

		vel_msg.linear.x =0;
		vel_msg.angular.z=0;
		pub_vel.publish(vel_msg);

	ROS_INFO_STREAM("Finished QR detect");
}

void vispStatusCallback(const std_msgs::Int8& msgVispStatus)
{
	if (msgVispStatus.data == 3 || msgVispStatus.data == 4)
		qrDetect=true;

}

void vispPoseCallback(const geometry_msgs::PoseStamped& msgVispPose)
{

	//ROS_INFO_STREAM("Current position from Camera: ("<<msgVispPose.pose.position.x<<","<<msgVispPose.pose.position.y <<"," <<msgVispPose.pose.position.z <<")");
	qr_rel_pose.setX(msgVispPose.pose.position.x);
	qr_rel_pose.setY(msgVispPose.pose.position.y);
	qr_rel_pose.setZ(msgVispPose.pose.position.z);

}

