//ROS includes
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

// ROS message includes
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include <geometry_msgs/PoseStamped.h>

// ROS service includes
#include <std_srvs/Empty.h>

// ROS action lib includes
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "conveyor_belt_worker");
  	//Node handle
  	ros::NodeHandle n;
  	//cmd_vel publisher
  	ros::Publisher topicPub_vel, topicPub_IO, topicPub_pose;
	ros::Subscriber topicSub_IO, Odom_sub;
	

	geometry_msgs::Twist cmd_vel;
	
	move_base_msgs::MoveBaseGoal goal;
	geometry_msgs::Quaternion odom_quat;

	geometry_msgs::Point32 Load_point;
	geometry_msgs::Point32 Unload_point;
	
	bool load = true;
 
	topicPub_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	topicPub_pose = n.advertise<geometry_msgs::PoseStamped>("next_drive_goal", 1);

	//Service clients
  	ros::ServiceClient client_locate = n.serviceClient<std_srvs::Empty>("locate_station");
	ros::ServiceClient client_unload = n.serviceClient<std_srvs::Empty>("unload_front");
	ros::ServiceClient client_load = n.serviceClient<std_srvs::Empty>("load_front");

	std_srvs::Empty empty;
	std_srvs::Empty empty_unload;
	std_srvs::Empty empty_load;

	MoveBaseClient ac("move_base", true);

  	ROS_INFO("Ready for Working");
	
	//Points in /Map coordinates
	Load_point.x = 5.0;
	Load_point.y = 0.0;
	Load_point.z = 0.0;

	Unload_point.x = - 0.1;
	Unload_point.y = 0.0;
	Unload_point.z = 0.0;	
	
	for(int i = 0; i < 100; i++)
	{
		ROS_INFO("Try number: %i",i);
		//1. call service
		if(client_locate.call(empty))
		{
	
			ROS_INFO("Waiting for move_base server...");
			ac.waitForServer();
			ROS_INFO("move_base server online!");
			
			if(load == true) //unload then drive to loadpoint
			{
				ROS_INFO("Call unload");
				//unload then drive
				if(client_unload.call(empty_unload))
				{
					ROS_INFO("unload OK");
				}

				//3. Drive back
				//publish cmd_vel
				//ROS_INFO("Modulo: %i", i%3);
				if(i%3 == 0)
				{
					//linear vel
					
					cmd_vel.linear.x = -0.3;
					cmd_vel.linear.y = 0.0;
					cmd_vel.linear.z = 0.0;
					//angular vel
					cmd_vel.angular.x = 0.0;
					cmd_vel.angular.y = 0.0;
					cmd_vel.angular.z = 0.05;
				}
				else if(i%3 == 1)
				{
					
					cmd_vel.linear.x = -0.3;
					cmd_vel.linear.y = 0.0;
					cmd_vel.linear.z = 0.0;
					//angular vel
					cmd_vel.angular.x = 0.0;
					cmd_vel.angular.y = 0.0;
					cmd_vel.angular.z = -0.05;
				}
				else
				{
					
					cmd_vel.linear.x = -0.3;
					cmd_vel.linear.y = 0.0;
					cmd_vel.linear.z = 0.0;
					//angular vel
					cmd_vel.angular.x = 0.0;
					cmd_vel.angular.y = 0.0;
					cmd_vel.angular.z = 0.0;
				}
				ROS_INFO("driving back....");
				topicPub_vel.publish(cmd_vel);
				//wait 4 seconds to drive back	
				ros::Duration(3.0).sleep();
				cmd_vel.linear.x = 0.0;
		                cmd_vel.linear.y = 0.0;
		                cmd_vel.linear.z = 0.0;
		                //angular vel
		                cmd_vel.angular.x = 0.0;
		               	cmd_vel.angular.y = 0.0;
		                cmd_vel.angular.z = 0.0;
				topicPub_vel.publish(cmd_vel);

				tf::Quaternion odom_tmp(tf::createQuaternionFromRPY(0, 0, 2.7));
				tf::quaternionTFToMsg(odom_tmp, odom_quat);
				
				//3.1 create Goal
				goal.target_pose.header.frame_id = "/map";
				goal.target_pose.header.stamp = ros::Time::now();
				goal.target_pose.pose.position.x = Load_point.x;
				goal.target_pose.pose.position.y = Load_point.y;
				goal.target_pose.pose.position.z = 0.0;
				goal.target_pose.pose.orientation = odom_quat;
				topicPub_pose.publish(goal.target_pose);
				load = false;
			}
			else //load then drive to unload point
			{
				ROS_INFO("Call load");
				//load then drive
				if(client_load.call(empty_load))
				{
					ROS_INFO("load OK");
				}

				//3. Drive back
				//publish cmd_vel
				//ROS_INFO("Modulo: %i", i%3);
				if(i%3 == 0)
				{
					//linear vel
					
					cmd_vel.linear.x = -0.3;
					cmd_vel.linear.y = 0.0;
					cmd_vel.linear.z = 0.0;
					//angular vel
					cmd_vel.angular.x = 0.0;
					cmd_vel.angular.y = 0.0;
					cmd_vel.angular.z = 0.05;
				}
				else if(i%3 == 1)
				{
					
					cmd_vel.linear.x = -0.3;
					cmd_vel.linear.y = 0.0;
					cmd_vel.linear.z = 0.0;
					//angular vel
					cmd_vel.angular.x = 0.0;
					cmd_vel.angular.y = 0.0;
					cmd_vel.angular.z = -0.05;
				}
				else
				{
					
					cmd_vel.linear.x = -0.3;
					cmd_vel.linear.y = 0.0;
					cmd_vel.linear.z = 0.0;
					//angular vel
					cmd_vel.angular.x = 0.0;
					cmd_vel.angular.y = 0.0;
					cmd_vel.angular.z = 0.0;
				}
				ROS_INFO("driving back....");
				topicPub_vel.publish(cmd_vel);
				//wait 4 seconds to drive back	
				ros::Duration(3.0).sleep();
				cmd_vel.linear.x = 0.0;
		                cmd_vel.linear.y = 0.0;
		                cmd_vel.linear.z = 0.0;
		                //angular vel
		                cmd_vel.angular.x = 0.0;
		               	cmd_vel.angular.y = 0.0;
		                cmd_vel.angular.z = 0.0;
				topicPub_vel.publish(cmd_vel);

				tf::Quaternion odom_tmp(tf::createQuaternionFromRPY(0, 0, 0));
				tf::quaternionTFToMsg(odom_tmp, odom_quat);
				
				//3.1 create Goal
				goal.target_pose.header.frame_id = "/map";
				goal.target_pose.header.stamp = ros::Time::now();
				goal.target_pose.pose.position.x = Unload_point.x;
				goal.target_pose.pose.position.y = Unload_point.y;
				goal.target_pose.pose.position.z = 0.0;
				goal.target_pose.pose.orientation = odom_quat;
				topicPub_pose.publish(goal.target_pose);
				load = true;
			}

			//3.2 send Goal	
			ROS_INFO("Send goal");
			ac.sendGoal(goal);
		       	ROS_INFO("Waiting.... (max. 50sec)");
			ac.waitForResult(ros::Duration(50.0));
	
	  		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				ROS_INFO("Goal reached");
			}
			else
			{
				ROS_WARN("Error Goal not reached!!!!!!!!");
				return 1;
			}
		}
		else
		{
			ROS_ERROR("ERROR at try %i !!!!!!", i);
			break;
		}	
	} 
  

  return 0;
}

