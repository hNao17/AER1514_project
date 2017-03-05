#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>
#include <tf/tf.h>

//global variables
const double num_waypoints=7;
const double dim_waypoint=3;
double path [][3] = { {31.5,3.0,90.0},
                      {31.5,4.0,90.0},
		              {31.5,5.0,90.0},
		              {31.5,5.0,90.0},
		              {31.5,6.0,90.0},
		              {31.5,7.0,90.0},
		              {31.5,8.0,90.0},};

ros::Subscriber sub_amcl_pose;
ros::Subscriber sub_visp_status;
ros::Publisher pub_vel;
double x_current;
double y_current;
double theta_current;

//double scan_time=8.0;
//double current_time=0.0;
//double initial_time=0.0;

bool scanON = false;
//bool qrDetect = false;
int qr_detect_counter=0;


/** function declarations **/
void moveToGoal(double xGoal, double yGoal, double yawGoal);
void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped& msgAMCL);
void vispStatusCallback(const std_msgs::Int8& msgVispStatus);
//void vispPoseCallback(const geometry_msgs::PoseStamped& msgVispPose);
void scanRotate();

int main(int argc, char** argv)
{
	ros::init(argc, argv, "qr_nav_test");
	ros::NodeHandle nh;

	//determine spawn location, check AMCL callback
	sub_amcl_pose = nh.subscribe("amcl_pose",1000,poseAMCLCallback);
	sub_visp_status = nh.subscribe("visp_auto_tracker/status",1000,vispStatusCallback);
	pub_vel = nh.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity",100);
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
	if(yawGoal == 0.0)
    {
        goal.target_pose.pose.orientation.x = 0.0;
		goal.target_pose.pose.orientation.y = 0.0;
		goal.target_pose.pose.orientation.z = 0.0;
		goal.target_pose.pose.orientation.w = 1.0;
    }
	else if (yawGoal == 90.0)
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
    else if(yawGoal == 270.0)
	{
		goal.target_pose.pose.orientation.x = 0.0;
		goal.target_pose.pose.orientation.y = 0.0;
		goal.target_pose.pose.orientation.z = sqrt(2);
		goal.target_pose.pose.orientation.w = -sqrt(2);
	}
    else if(yawGoal == 359.0)
	{
		goal.target_pose.pose.orientation.x = 0.0;
		goal.target_pose.pose.orientation.y = 0.0;
		goal.target_pose.pose.orientation.z = 0.0087;
		goal.target_pose.pose.orientation.w = -1.0;
	}

	ROS_INFO("Sending goal location ...");
	ac.sendGoal(goal);

	ac.waitForResult(ros::Duration(30.0));

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("You have reached the destination");
		scanRotate();
		//qr_detect_counter=0;
	}
	else
	{
		ROS_INFO("The robot failed to reach the destination");
	}

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

    theta_current = yaw;

	ROS_INFO_STREAM("Current turtlebot orientation: "<<theta_current<<" [radians]");

	x_current = msgAMCL.pose.pose.position.x;
	y_current = msgAMCL.pose.pose.position.y;
}

void scanRotate()
{
	geometry_msgs::Twist vel_msg;
	vel_msg.linear.x =0;
	vel_msg.angular.z=M_PI/4;

	double theta_previous;
	double theta_complete=0;
	double del_theta;

	//initial_time = ros::Time::now().toSec();
	ROS_INFO_STREAM("Initating QR detect");
	ros::Rate loop_rate(100);
	ros::spinOnce();

	theta_previous = theta_current;
	//while ((current_time-initial_time) < scan_time && qrDetect == false)
    while(theta_complete < 2*M_PI)
	{
	    scanON = true;
		//current_time = ros::Time::now().toSec();
		pub_vel.publish(vel_msg);
		ros::spinOnce();

        //wait for qr code to be processed
        if(qr_detect_counter >=5)
        {
            vel_msg.angular.z=0;
            pub_vel.publish(vel_msg);
            ros::Duration(5.0).sleep();

            //start moving
            vel_msg.angular.z=M_PI/4;
            pub_vel.publish(vel_msg);

            qr_detect_counter = 0;
        }

		del_theta = theta_current - theta_previous;

		if(abs(del_theta)>M_PI)
            del_theta+= 2*M_PI;

        theta_complete +=  del_theta;
        theta_previous = theta_current;

	}

	//QR code found; stop rotating
	vel_msg.linear.x =0;
	vel_msg.angular.z=0;
	pub_vel.publish(vel_msg);

    scanON = false;
	ROS_INFO_STREAM("Finished QR detect");
}

void vispStatusCallback(const std_msgs::Int8& msgVispStatus)
{
	//ROS_INFO_STREAM("Visp Callback Started");
	if (msgVispStatus.data == 3 || msgVispStatus.data == 4)
	{
	    //ROS_INFO_STREAM("visp status: "<<msgVispStatus.data);
	    if(scanON==true)
        {
            ROS_INFO_STREAM("Found QR Code");
            ROS_INFO_STREAM("qr detect counter: "<<qr_detect_counter);
            qr_detect_counter++; //sample visp_auto_tracker
        }

	}


}

/*void vispPoseCallback(const geometry_msgs::PoseStamped& msgVispPose)
{
}*/


