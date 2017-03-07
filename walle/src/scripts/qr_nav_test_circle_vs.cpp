#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

//global variables
const double num_waypoints=5;
const double dim_waypoint=3;
double path [][3] = { {31.5,4.0,90.0},
                      {31.5,5.0,90.0},
		              {31.5,6.0,90.0},
		              {31.5,7.0,90.0},
                      {31.5,8.0,90.0}};

ros::Subscriber sub_amcl_pose;
ros::Subscriber sub_visp_status;
ros::Subscriber sub_visp_word;
ros::Subscriber sub_visp_pose;

ros::Publisher pub_vel;
double x_current;
double y_current;
double theta_current;


// Global Variables
tf::Vector3 qr_rel_pose(0, 0, 0);
tf::Vector3 base_pose(0, 0, 0);
bool scanON = false;
bool qrProcessing = false;
int qr_detect_counter=0;


/** function declarations **/
void moveToGoal(double xGoal, double yGoal, double yawGoal);
void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped& msgAMCL);
void vispStatusCallback(const std_msgs::Int8& msgVispStatus);
void vispWordCallback(const std_msgs::String& msgVispWord);
void vispPoseCallback(const geometry_msgs::PoseStamped& msgVispPose);
void scanRotate();
static tf::Quaternion toQuaternion(double pitch, double roll, double yaw);
static tf::Vector3 qr_goal_calculate();
static void toEulerianAngle(const tf::Quaternion& q, double& roll, double& pitch, double& yaw);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "qr_nav_test_vs");
	ros::NodeHandle nh;
    
	//determine spawn location, check AMCL callback
	sub_amcl_pose = nh.subscribe("amcl_pose",1000,poseAMCLCallback);
	sub_visp_status = nh.subscribe("visp_auto_tracker/status",1000,vispStatusCallback);
	sub_visp_word = nh.subscribe("visp_auto_tracker/code_message",100,vispWordCallback);
	sub_visp_pose = nh.subscribe("/visp_auto_tracker/object_position",1000,vispPoseCallback);
    pub_vel = nh.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity",100);
	//ros::spinOnce();

	for (int i=0; i<num_waypoints; i++)
	{
		//position command
		ROS_INFO_STREAM("Waypoint #"<<i+1);
		moveToGoal(path[i][0], path[i][1], path[i][2]);
        qr_detect_counter=0;
        scanRotate();
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
//    goal.target_pose.pose.orientation.x = qQR.x;
//    goal.target_pose.pose.orientation.y = qQR.y;
//    goal.target_pose.pose.orientation.z = qQR.z;
//    goal.target_pose.pose.orientation.w = qQR.w;

	/*/convert degrees to quaternion
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
	}*/

	ROS_INFO("Sending goal location ...");
	ac.sendGoal(goal);

	ac.waitForResult(ros::Duration(30.0));

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("You have reached the destination");
		//qr_detect_counter=0;
		//scanRotate();
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
    
	base_pose.setX(msgAMCL.pose.pose.position.x);
	base_pose.setY(msgAMCL.pose.pose.position.y);
    base_pose.setZ(0.0);
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

            vel_msg.linear.x=0;
            vel_msg.angular.z=0;
            pub_vel.publish(vel_msg);
            //ros::Duration(5.0).sleep();
            
            /*****************************/
            //need to exit here to QR approach mode
            tf::Vector3 qr_nav_goal(0,0,0);
            qr_nav_goal = qr_goal_calculate();
            moveToGoal(qr_nav_goal.x(),qr_nav_goal.y(),theta_current);
            
            /*
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
             */

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
	//q = q_temp;
	return q;
}

static void toEulerianAngle(const tf::Quaternion& q, double& roll, double& pitch, double& yaw)
{
	double ysqr = q.y() * q.y();

	// roll (x-axis rotation)
	double t0 = +2.0 * (q.w() * q.x() + q.y() * q.z());
	double t1 = +1.0 - 2.0 * (q.x() * q.x() + ysqr);
	roll = std::atan2(t0, t1);

	// pitch (y-axis rotation)
	double t2 = +2.0 * (q.w() * q.y() - q.z() * q.x());
	t2 = t2 > 1.0 ? 1.0 : t2;
	t2 = t2 < -1.0 ? -1.0 : t2;
	pitch = std::asin(t2);

	// yaw (z-axis rotation)
	double t3 = +2.0 * (q.w() * q.z() + q.x() * q.y());
	double t4 = +1.0 - 2.0 * (ysqr + q.z() * q.z());  
	yaw = std::atan2(t3, t4);
}

static tf::Vector3 qr_goal_calculate()
{
    // request the transform between the two frames
	  tf::TransformListener listener;
    tf::StampedTransform transform;

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
		tf::Vector3 qr_goal = qr_pose + 0.5*approach_vector;
		qr_goal.setZ(0.0); //Neglect the Height

		ROS_INFO_STREAM("Camera X in MAP Frame :"<<transform.getOrigin().x());
		ROS_INFO_STREAM("Camera Y in MAP Frame :"<<transform.getOrigin().y());
		ROS_INFO_STREAM("Camera Z in MAP Frame :"<<transform.getOrigin().z());
		ROS_INFO_STREAM("QR X in Camera Frame :"<<qr_rel_pose.x());
		ROS_INFO_STREAM("QR Y in Camera Frame :"<<qr_rel_pose.y());
		ROS_INFO_STREAM("QR Z in Camera Frame :"<<qr_rel_pose.z());
        ROS_INFO_STREAM("");
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

		//tf::Vector3 qr_goal(0,0,0);
        return qr_goal;
}

void vispPoseCallback(const geometry_msgs::PoseStamped& msgVispPose)
{

	//ROS_INFO_STREAM("Current position from Camera: ("<<msgVispPose.pose.position.x<<","<<msgVispPose.pose.position.y <<"," <<msgVispPose.pose.position.z <<")");
	qr_rel_pose.setX(msgVispPose.pose.position.x);
	qr_rel_pose.setY(msgVispPose.pose.position.y);
	qr_rel_pose.setZ(msgVispPose.pose.position.z);
	
}
