//ROS includes
#include <ros/ros.h>

// ROS service includes
#include <std_srvs/Empty.h>

// ROS message includes
#include <neo_msgs/IOOut.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>

class controller
{
public:
	controller();
	~controller();
		
	//Node handle
  	ros::NodeHandle n;

  	//Service handle
  	ros::ServiceServer service_open_clamps, service_close_clamps, service_load_front, service_unload_front;
	ros::ServiceServer service_load_back, service_unload_back, service_robot_full;

	//Subscriber
	ros::Subscriber topicSub_get_IO_state;

	//Publisher
	ros::Publisher topicPub_IO;
private:
	
	//Service Callbacks
	bool openclampsServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
	bool closeclampsServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
	bool loadfrontServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
	bool unloadfrontServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
	bool loadbackServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
	bool unloadbackServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
	bool checkloadedServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

	//Topic Callbacks
	void IOCallBack(std_msgs::Int16 IOs);

	//Parameter
	bool getparams();

	//States
	bool BoardisBusy();
	bool BoardisTimeOut();
	bool RobotisFull();

	neo_msgs::IOOut IO_handle;
	bool newIOstates;
	bool isBusy, isTimeOut, isLoaded;
	
};
