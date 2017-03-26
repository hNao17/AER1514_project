//Created 2017-03-17
//Updated 2017-03-24
//Node that allows Turtlebot to perform auto-docking with kobuki docking station
//If docking is unsuccessful, Turtlebot uses visual servoing to drive to docking station
/////////////////////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <kobuki_msgs/AutoDockingAction.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

/** Global Variables **/
bool dockON = false;
bool docking_complete = false;

ros::Subscriber sub_dockONStatus;
ros::Subscriber sub_x_position_err;
ros::Subscriber sub_size;
ros::Subscriber sub_blobDetect_status;
ros::Publisher pub_dockingStatus;
ros::Publisher pub_vel;

const double vel_x = 0.1;
const double k_theta = 0.01;
const double size_threshold = 45.0;
const double timeout_for_dockDetect = 3.0;
float blob_size;
double error_posX;
bool blobDetect;

const double dock_startX = 31.0;
const double dock_startY = 4.5;
const double dock_startTheta = 0.0;
double x_current;
double y_current;

/** Function Declarations **/
bool moveToDock();
bool moveToGoal(double xGoal, double yGoal, double yawGoal);
void visualPositioning(double posX_error);
static tf::Quaternion toQuaternion(double pitch, double roll, double yaw);
void docking_callback(const std_msgs::Bool& msg_startDocking);
void dockError_callback(const std_msgs::Float32& msg_dockError);
void dockSize_callback(const std_msgs::Float32& msg_blobSize);
void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped& msgAMCL);
void blobDetect_callback(const std_msgs::Bool& msg_startVS);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "autoDocking_node");
	ros::NodeHandle nh2;

	//subscribe to dockStart, dockDetect/x_position_err, dockDetect/size, dockDetect/detection_flag topics
	sub_dockONStatus = nh2.subscribe("dockON_Status",1000,docking_callback);
	sub_x_position_err = nh2.subscribe("dockDetect/x_position_err",1000,dockError_callback);
	sub_size = nh2.subscribe("dockDetect/size",1000,dockSize_callback);
    sub_blobDetect_status = nh2.subscribe("dockDetect/detection_flag", 1000,blobDetect_callback);

	//publisher to dockStatus, geometry/Twist topics
	pub_dockingStatus = nh2.advertise<std_msgs::Bool>("/dockingStatus",1000);
	pub_vel = nh2.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity",100);

	ros::Rate rate(10);
	while(!dockON)
    {
        ROS_INFO_STREAM("Waiting for Docking approval");
        ros::spinOnce();
    }

    //robot is now at home position
    docking_complete=moveToDock();
    std_msgs::Bool msg_atDock;
    msg_atDock.data = docking_complete;

    //robot successfully docked on first attempt
    if(docking_complete)
    {
        pub_dockingStatus.publish(msg_atDock);
        ros::spinOnce();
    }

    //robot failed to dock on first attempt
    else
    {
    	//keep attempting auto-docking until successful
    	while(!docking_complete)
    	{

			// Random point reposition
			bool reposition_success = moveToGoal(dock_startX, dock_startY, dock_startTheta);
			ros::spinOnce();

			if (reposition_success)
			{
				ROS_INFO_STREAM("Reposition Succeeded. Initiating Visual Servoing");

				//Initiate Visual Servoing
				double begin_time = ros::Time::now().toSec();
				double time_elapsed = 0;

				while(time_elapsed < timeout_for_dockDetect)
				{
					ROS_INFO_STREAM("Waiting for blob detect flag");

					if(blobDetect)
					{
						ROS_INFO_STREAM(" Dock Detected. Initiating steering commands");
						//Move Turtlebot along a straight line b/t the docking station and random point
						//Stop when the docking station pixel size occupies a majority of the camera plane
						while(blob_size < size_threshold)
						{
							visualPositioning(error_posX);
							ros::spinOnce();
						}
						//if success
						//Assuming visual servoing succeeds
							//robot is ready to commence auto-docking again
							ROS_INFO_STREAM(" Visual Servo Complete. Re-Initiating Auto Docking");
							docking_complete=moveToDock();
							msg_atDock.data = docking_complete;
							pub_dockingStatus.publish(msg_atDock);

						//else

						break; // break out of timer while loop
					}

					time_elapsed = ros::Time::now().toSec() - begin_time;
					ros::spinOnce();
				}

			}

			pub_dockingStatus.publish(msg_atDock);
			ros::spinOnce();

    	}


		ros::spinOnce();

    }

    //robot is now docked at station; leave node in idle state
    ROS_INFO_STREAM("Docking node is finished. Entering stand-by mode");
    dockON = false;
    ros::spin();

}

bool moveToDock()
{
    //define a client for to send docking requests to the AutoDocking server through a SimpleActionClient
    actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction> ac_dock("dock_drive_action", true);

    ROS_INFO("Waiting for the AutoDocking action server to come up");
    ac_dock.waitForServer();

	kobuki_msgs::AutoDockingGoal goal_dock;

	ac_dock.sendGoal(goal_dock);
	ac_dock.waitForResult(ros::Duration(30.0));

	if(ac_dock.getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO_STREAM("Docking complete");
        return true;
    }
    else
    {
        ROS_INFO_STREAM("Auto-docking failed");
        ac_dock.cancelGoal();
        return false;
    }
}

bool moveToGoal(double xGoal, double yGoal, double yawGoal)
{
    //define a client for to send goal requests to the move_base server through a SimpleActionClient
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_mb2("move_base", true);

	//wait for the action server to come up
	while(!ac_mb2.waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Waiting for the move_base 2 action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;

	//set up the frame parameters
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	//store desired (x,y,theta) values in move_base message
	goal.target_pose.pose.position.x =  xGoal;
	goal.target_pose.pose.position.y =  yGoal;
	goal.target_pose.pose.position.z =  0.0;

	tf::Quaternion qQR;
    qQR = toQuaternion(0,0,yawGoal);
    quaternionTFToMsg(qQR, goal.target_pose.pose.orientation); // stores qQR in goal orientation

	ROS_INFO("Sending goal location ...");
	ac_mb2.sendGoal(goal);
    ac_mb2.waitForResult(ros::Duration(75.0));

        if(ac_mb2.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("You have reached the dock start location");
            return true;

        }
        else
        {
            ROS_INFO("The robot failed to reach the the dock start location");
            return false;
        }

}

void visualPositioning(double error)
{
    geometry_msgs::Twist msg_vel;

    msg_vel.linear.x = vel_x;
    msg_vel.angular.z = -k_theta*error;

    pub_vel.publish(msg_vel);
    ROS_INFO_STREAM("Visual Servo Commands:: Velocity ="<<msg_vel.linear.x<<" Steering ="<<msg_vel.angular.z);

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

void docking_callback(const std_msgs::Bool& msg_startDocking)
{
    if(!msg_startDocking.data)
    {
        dockON = false;
        ROS_INFO_STREAM("Hold auto-docking. Robot is not at home position.");
    }

    else
    {
        dockON = true;
        ROS_INFO_STREAM("Begin auto-docking action.");
    }
}

void dockError_callback(const std_msgs::Float32& msg_dockError)
{
    ROS_INFO_STREAM("Current dock error = "<<msg_dockError.data<<"[pixels]");
    error_posX = msg_dockError.data;
}

void dockSize_callback(const std_msgs::Float32& msg_blobSize)
{
    ROS_INFO_STREAM("Current dock size in camera window = "<<msg_blobSize.data<<"[pixels^2]");
    blob_size = msg_blobSize.data;
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

void blobDetect_callback(const std_msgs::Bool& msg_startVS)
{
    if(!msg_startVS.data)
    {
        blobDetect = false;
        ROS_INFO_STREAM("Blob detect hasn't found docking station");
    }

    else
    {
        blobDetect = true;
        ROS_INFO_STREAM("Docking station found. Begin visual servoing");
    }

}
