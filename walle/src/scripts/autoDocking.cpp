//Created 2017-03-17
//Node that allows Turtlebot to perform auto-docking with kobuki docking station
////////////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <kobuki_msgs/AutoDockingAction.h>
#include <std_msgs/Bool.h>

/** Global Variables **/
bool dockON = false;
bool docking_complete = false;

ros::Subscriber sub_dockONStatus;
ros::Publisher pub_dockingStatus;

/** Function Declarations **/
bool moveToDock();
void docking_callback(const std_msgs::Bool& msg_startDocking);
//void moveToGoal(double xGoal, double yGoal, double yawGoal);
//static tf::Quaternion toQuaternion(double pitch, double roll, double yaw);
//void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped& msgAMCL);


int main(int argc, char** argv)
{
	ros::init(argc, argv, "autoDocking_node");
	ros::NodeHandle nh2;

	//subscriber to dockStart topic
	sub_dockONStatus = nh2.subscribe("dockON_Status",1000,docking_callback);

	//publisher to dockStatus topic
	pub_dockingStatus = nh2.advertise<std_msgs::Bool>("/dockingStatus",1000);

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
        //keep attempting auto-docking until sucessful
        while(!docking_complete)
        {

            msg_atDock.data = docking_complete;

            pub_dockingStatus.publish(msg_atDock);

            //perform visual servoing
            //reposition robot if necessary

            ros::spinOnce();
            docking_complete=moveToDock();
        }
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
	ac_dock.waitForResult(ros::Duration(60.0));

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
