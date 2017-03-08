#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <kobuki_msgs/AutoDockingAction.h>
//#include <move_base_msgs/MoveBaseAction.h>

//global variables
bool docking_complete = false;
bool start_docking= false;
//ros::Subscriber sub_explore;
//ros::Publisher pub_docking;

//void exploreCompleteCallback();
bool moveToDock();
//void visualServoDock();
//void moveToLocalGoal(double x_goal, double y_goal, double theta_goal);

int main (int argc, char** argv)
{

    ros::init(argc, argv, "autoDock_test");
    ros::NodeHandle nh;

    //subscribe to exploreComplete topic being published by supervisor / navigation node
    //sub_explore=nh.subscriber("<<topic_name>>",1000,exploreCompleteCallback);
    //pub_docking = nh.publisher<<"message_type">>("<<topic_name>>",100);
    //periodically check for explore_complete flag
    //ros::Rate rate(1);
    //while(start_docking==false)
    //{
        //node is idle until explore_complete flag is true
        //ros::spinOnce();
   //}

    ROS_INFO_STREAM("Explore complete. Initiating auto-dock.");

    //create message type that this node can publish to supervisor
    //ros::Rate rate2(100);
    //while(docking_complete==false)
    //{
        docking_complete = moveToDock();
        //pub_docking.publish(<<msg_name>>);

        //if(docking_complete==false)
        //{
             // double x_local, y_local,theta_local
             // initiate visual servoing code that repositions turtebot so that is aligned with central emitter
                //visualServoDock(&x_local, &y_local, &theta_local)
             //moveToLocalGoal(x_local,y_local,theta_local)
            // ROS_INFO_STREAM("Repeating auto-dock");
        //}

    //}

     //ROS_INFO_STREAM("Docking Complete");
     //ros::spin();

}

/*void exploreCompleteCallback()
{
    //if explore_complete == true
        //set start_docking = true;
}*/

/*void visualServoDock(double* x, double* y, double* theta)
{
    //use camera to identify dock
    //determine relative distance from dock to turtlebot
    //determine global position of dock using current turtlebot position + relative distance
    //calculate local waypoint turtlebot should move to, in order to align with central emitter
    //&x = local_waypoint_x;
    //&y = local_waypoint_y;
    //&theta = local_waypoint_theta;
}*/

/*void moveToLocalGoal(double x_goal, double y_goal, double theta_goal)
{
    //repeat same code as seen in nav_test2
}*/

bool moveToDock()
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
        return true;
    }
    else
    {
        ROS_INFO_STREAM("Auto-docking failed");
        ac_dock.cancelGoal();
        return false;
    }
}
