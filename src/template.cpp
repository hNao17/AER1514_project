#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msg/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <stdlib.h>

//callback function template
void <<callback function name>> (const <<message_type>>)
{
	/*tf::Quaternion q (odom.pose.pose.orientation.x, 
			  odom.pose.pose.orientation.y,
			  odom.pose.pose.orientation.z,
			  odom.pose.pose.orientation.w);*/

	//tf::Matrix3x3 m(q);
	//double roll, pitch, yaw;
	//m.getRPY(roll, pitch, yaw);

	/*ROS_INFO_STREAM("Current turtlebot position: ("
                               <<odom.pose.pose.position.x <<","
			       <<odom.pose.pose.position.y <<","
			       <<odom.pose.pose.position.z <<")");

	ROS_INFO_STREAM("Current turtlebot orientation: (0,0,"
                               <<yaw*r2D<<")");*/

}

int main(int argc, char** argv)
{
	//ros::init(argc, argv, <node_name>);
	//ros::NodeHandle nh;
	
	//ros::Publisher pub = nh.advertise< <<message_type>> >("<<topic_name>>", <<message queue size>>);
	//ros::Subscriber sub = nh.subscribe("<<topic name>>",<<message queue size>>,<<callback function name>>);

	//tf::TransformBroadcaster br;
	//tf::TransformListener listener;
	//tf::Transform transform;

	//while(node.ok())
	//{
		////////Broadcaster pseudocode
		/*transform.setOrigin(tf::Vector3(const tfScalar &x,
						  const tfScalar &y,
						  const tfScalar &z);*/  

		//transform.setRotation( tf::Quaternion(0, 0, 0, 1) );

	   	//br.sendTransform(tf::StampedTransform(transform, 							ros::Time::now(), 							<<parent_frame_name>>, 							<<child_frame_name>>));
 		//rate.sleep();
		///////////////////////////////////

		//////////Listener pseudocode
		/*try
    		{
      			ros::Time now = ros::Time::now();
      			ros::Time past = now - ros::Duration(5.0);

      			listener.waitForTransform("<<target_frame>>", 
						  "<<source_frame>>",
						   <<current_time>>,
						   <<timeout>>,
						   <<polling_time>>,
						  "<<error_msg>>");

      			listener.lookupTransform("<<target_frame>>",
						 "<<source_frame>>",
						  <<current_time>>,
						  <<transform>>)
    		}

    		catch (tf::TransformException &ex) 
    		{
      			ROS_ERROR("%s",ex.what());
      			ros::Duration(1.0).sleep();
      			continue;
    		}*/
		//////////////////////////////////////
	//}	
			
}
