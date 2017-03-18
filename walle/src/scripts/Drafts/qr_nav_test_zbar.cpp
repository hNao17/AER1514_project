#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <tf/tf.h>

//global variables
const double num_waypoints=8;
const double dim_waypoint=3;
double path [][3] = { {31.5,10.5,90.0},
                      {31.5,18.5,90.0},
                      {31.5,25.5,90.0},
                      {31.5,33.5,90.0},
                      {31.5,25.5,90.0},
                      {31.5,18.5,90.0},
                      {31.5,10.5,90.0},
                      {31.5,4.5,270.0}};

ros::Subscriber sub_barcode;
ros::Subscriber sub_amcl_pose;

ros::Publisher pub_vel;

double x_current;
double y_current;
double theta_current;
bool scanON = false;

//int qr_detect_counter=0;

/** function declarations **/
void moveToGoal(double xGoal, double yGoal, double yawGoal);
static tf::Quaternion toQuaternion(double pitch, double roll, double yaw);
void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped& msgAMCL);
void vispStatusCallback(const std_msgs::Int8& msgVispStatus);
void vispWordCallback(const std_msgs::String& msgVispWord);
void zbar_barcodeCallback(const std_msgs::String& zbarWord);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "qr_nav_test_zbar");
	ros::NodeHandle nh;

	//determine spawn location, check AMCL callback
	sub_amcl_pose = nh.subscribe("amcl_pose",1000,poseAMCLCallback);
	sub_barcode = nh.subscribe("barcode",1000,zbar_barcodeCallback);
	pub_vel = nh.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity",100);
	ros::spinOnce();

	for (int i=0; i<num_waypoints; i++)
	{
		//position command
		ROS_INFO_STREAM("Waypoint #"<<i+1);
		moveToGoal(path[i][0], path[i][1], path[i][2]);

        ros::spinOnce();
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

    tf::Quaternion qQR;

    qQR = toQuaternion(0,0,yawGoal);

    quaternionTFToMsg(qQR, goal.target_pose.pose.orientation); // stores qQR in goal orientation

	ROS_INFO("Sending goal location ...");
	ac.sendGoal(goal);

    ac.waitForResult(ros::Duration(75.0));
	//qr_detect_counter=0;

	/*double current_time, start_time;

	start_time = ros::Time::now().toSec();

	while ((current_time-start_time)<15.0)
    {
        scanON = true;
        current_time = ros::Time::now().toSec();
        ros::spinOnce();
        if(qr_detect_counter >=1)
        {
            ROS_INFO_STREAM("Initiating QR Scan");
            ac.cancelGoal();
            ros::Duration(5.0).sleep();
            start_time = ros::Time::now().toSec();
            ac.sendGoal(goal);
            qr_detect_counter=0;

            ROS_INFO_STREAM("Finished QR Scan");
        }

    }*/

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("You have reached the destination");

	}
	else
	{
		ROS_INFO("The robot failed to reach the destination");
	}

	ROS_INFO("Next Waypoint");
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

	tf::Quaternion q (msgAMCL.pose.pose.orientation.x,
                      msgAMCL.pose.pose.orientation.y,
             		  msgAMCL.pose.pose.orientation.z,
               		  msgAMCL.pose.pose.orientation.w);

	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	ROS_INFO_STREAM("Current turtlebot position: ("
                   <<msgAMCL.pose.pose.position.x <<","
			       <<msgAMCL.pose.pose.position.y <<","
			       <<msgAMCL.pose.pose.position.z <<")");

    //theta_current = yaw;

	ROS_INFO_STREAM("Current turtlebot orientation: "<<theta_current<<" [radians]");

	x_current = msgAMCL.pose.pose.position.x;
	y_current = msgAMCL.pose.pose.position.y;
}

void zbar_barcodeCallback(const std_msgs::String& zbarWord)
{
    if(zbarWord.data!="")
        ROS_INFO_STREAM("QR Code word: "<<zbarWord.data);
}







