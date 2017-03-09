#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <iostream>
#include <fstream>
#include <string>

//global variables
const int num_waypoints=10;
const int dim_waypoint=3;
double path [num_waypoints][dim_waypoint];

ros::Subscriber sub_amcl_pose;
ros::Subscriber sub_visp_status;
ros::Subscriber sub_visp_word;

ros::Publisher pub_vel;
double x_current;
double y_current;
double theta_current;

bool scanON = false;
bool qrProcessing = false;
int qr_detect_counter=0;


/** function declarations **/
void readWaypoints();
void moveToGoal(double xGoal, double yGoal, double yawGoal);
static tf::Quaternion toQuaternion(double pitch, double roll, double yaw);
void scanRotate();
void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped& msgAMCL);
void vispStatusCallback(const std_msgs::Int8& msgVispStatus);
void vispWordCallback(const std_msgs::String& msgVispWord);


int main(int argc, char** argv)
{
	ros::init(argc, argv, "qr_nav_test");
	ros::NodeHandle nh;

    //import waypoints from text file
	readWaypoints();

	//determine spawn location, check AMCL callback
	sub_amcl_pose = nh.subscribe("amcl_pose",1000,poseAMCLCallback);
	sub_visp_status = nh.subscribe("visp_auto_tracker/status",1000,vispStatusCallback);
	sub_visp_word = nh.subscribe("visp_auto_tracker/code_message",100,vispWordCallback);
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

void readWaypoints()
{
    std::ifstream infile("/home/na052/catkin_ws/src/AER1514_project/walle/src/scripts/waypoint_textfiles/masterWaypoints5.txt.csv");

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

	ac.waitForResult(ros::Duration(30.0));

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("You have reached the destination");
		qr_detect_counter=0;
		scanRotate();
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

void scanRotate()
{
	geometry_msgs::Twist vel_msg;
	vel_msg.linear.x =0;
	vel_msg.angular.z=M_PI/4;

	double theta_previous;
	double theta_at_stop;
	double theta_complete=0;
	double del_theta;

	//initial_time = ros::Time::now().toSec();
	ROS_INFO_STREAM("Initating QR detect");
	ros::Rate loop_rate(100);
	ros::spinOnce();

	theta_previous = theta_current;

    while(theta_complete < 2*M_PI)
	{
	    scanON = true;
		pub_vel.publish(vel_msg);
		ros::spinOnce();

        //wait for qr code to be processed
        if(qr_detect_counter >=5)
        {
            qrProcessing = true;
            theta_at_stop = theta_complete;

            vel_msg.angular.z=0;
            pub_vel.publish(vel_msg);
            ros::Duration(5.0).sleep();

            //start moving after QR code has been processed
            while((theta_complete-theta_at_stop) < M_PI/3 && theta_complete < 2*M_PI)
            {
                ROS_INFO_STREAM("Relative Angle:"<<theta_complete-theta_at_stop);
                vel_msg.angular.z=M_PI/4;
                pub_vel.publish(vel_msg);

                ros::spinOnce();

                del_theta = theta_current - theta_previous;

                if(abs(del_theta)>M_PI)
                    del_theta+= 2*M_PI;

                theta_complete +=  del_theta;
                theta_previous = theta_current;
            }

            qr_detect_counter = 0;
        }

        //qr_detect_counter can be incremented again
        qrProcessing = false;

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

	//moving to next waypoint
    scanON = false;
	ROS_INFO_STREAM("Finished QR detect");
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

void vispStatusCallback(const std_msgs::Int8& msgVispStatus)
{
	//ROS_INFO_STREAM("Visp Callback Started");
	if (msgVispStatus.data == 3 || msgVispStatus.data == 4)
	{
        //Only search for QR codes if stopped at a waypoint and the QR code is not currently being processed
	    if(scanON==true && qrProcessing ==false)
        {
            ROS_INFO_STREAM("Found QR Code");
            ROS_INFO_STREAM("qr detect counter: "<<qr_detect_counter);
            qr_detect_counter++;
        }

	}

}

void vispWordCallback(const std_msgs::String& msgVispWord)
{
    if(msgVispWord.data!="")
        ROS_INFO_STREAM("QR Word ="<<msgVispWord.data);
}




