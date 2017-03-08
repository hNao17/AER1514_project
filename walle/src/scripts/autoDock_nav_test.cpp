#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <kobuki_msgs/AutoDockingAction.h>

//global variables
const double num_waypoints=2;
const double dim_waypoint=3;
double path [][3] = { {31.5,10.0,90.0},
                      {31.5,14.0,90.0}};

const double x_home=31.5;
const double y_home=4.0;
const double theta_home=270.0;


double x_current;
double y_current;
double theta_current;

bool scanON = false;

ros::Subscriber sub_amcl_pose;

int qr_detect_counter=0;

/** function declarations **/
void moveToGoal(double xGoal, double yGoal, double yawGoal);
void moveToDock();
static tf::Quaternion toQuaternion(double pitch, double roll, double yaw);
void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped& msgAMCL);
void vispStatusCallback(const std_msgs::Int8& msgVispStatus);
void vispWordCallback(const std_msgs::String& msgVispWord);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "qr_nav_test_rotatedLaptop");
	ros::NodeHandle nh;

	//determine spawn location, check AMCL callback
	sub_amcl_pose = nh.subscribe("amcl_pose",1000,poseAMCLCallback);
	//ros::spinOnce();

	for (int i=0; i<num_waypoints; i++)
	{
		//position command
		ROS_INFO_STREAM("Waypoint #"<<i+1);
		moveToGoal(path[i][0], path[i][1], path[i][2]);

		ROS_INFO_STREAM("X Position Error: "<<path[i][0]-x_current);
		ROS_INFO_STREAM("Y Position Error: "<<path[i][1]-y_current);
		ROS_INFO_STREAM("Next destination");
	}

    //return home
    ROS_INFO_STREAM("Exploration Complete. Going home.");
	moveToGoal(x_home,y_home,theta_home);

	ROS_INFO_STREAM("Initiating auto-dock.");
	moveToDock();


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

void moveToDock()
{
    actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction> ac_dock("dock_drive_action", true);


    ROS_INFO("Waiting for the AutoDocking action server to come up");
    ac_dock.waitForServer();

	kobuki_msgs::AutoDockingGoal goal_dock;

	ac_dock.sendGoal(goal_dock);
	ac_dock.waitForResult(ros::Duration(60.0));

	if(ac_dock.getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO_STREAM("Docking complete");
        //return true;
    }
    else
    {
        ROS_INFO_STREAM("Auto-docking failed");
        ac_dock.cancelGoal();
        //return false;
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
