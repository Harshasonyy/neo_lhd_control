#include <neo_lhd_control/lhd_control.h>

// constructor
controller::controller()
{
	
	//getparams();	

	//Subscriber
	topicSub_get_IO_state = n.subscribe("/srb_io_dig_in", 2, &controller::IOCallBack, this);
	
	//Publisher
	topicPub_IO = n.advertise<neo_msgs::IOOut>("/srb_io_set_dig_out", 1);
	
	//Services
	service_open_clamps = n.advertiseService("open_clamps", &controller::openclampsServiceCallback, this);
	service_close_clamps = n.advertiseService("close_clamps", &controller::closeclampsServiceCallback, this);
	service_load_front = n.advertiseService("load_front", &controller::loadfrontServiceCallback, this);
	service_unload_front = n.advertiseService("unload_front", &controller::unloadfrontServiceCallback, this);
	service_load_back = n.advertiseService("load_back", &controller::loadbackServiceCallback, this);
	service_unload_back = n.advertiseService("unload_back", &controller::unloadbackServiceCallback, this);
	service_robot_full = n.advertiseService("robot_check_loaded", &controller::checkloadedServiceCallback, this);
	
	newIOstates = false; 
	
}
// destructor
controller::~controller(){}
//Service open clamps Callback
bool controller::openclampsServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	if(!BoardisBusy()) //Board is ready
	{
		//Open Clamps
		ROS_INFO("Open Clamps.....");
		//set enable bit low
		IO_handle.channel = 0;
		IO_handle.active = false;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		//send Open Clamps
		IO_handle.channel = 1;
		IO_handle.active = false;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		IO_handle.channel = 2;
		IO_handle.active = true;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		IO_handle.channel = 3;
		IO_handle.active = false;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		//set enable bit high
		IO_handle.channel = 0;
		IO_handle.active = true;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		//set enable bit low
		IO_handle.channel = 0;
		IO_handle.active = false;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		while(BoardisBusy())
		{
			ROS_INFO("Waiting.....");
			ros::Duration(0.5).sleep();
			if(BoardisTimeOut())
			{
				ROS_WARN("TimeOut!");
				return false;
			}
		}
		return true;
	}
	else //Board busy => can not complete service
	{
		return false;
	}
}
//Service close clamps Callback
bool controller::closeclampsServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	if(!BoardisBusy()) //Board is ready
	{
		//Close Clamps
		ROS_INFO("Close Clamps.....");
		//set enable bit low
		IO_handle.channel = 0;
		IO_handle.active = false;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		//send Close Clamps
		IO_handle.channel = 1;
		IO_handle.active = true;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		IO_handle.channel = 2;
		IO_handle.active = true;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		IO_handle.channel = 3;
		IO_handle.active = false;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		//set enable bit high
		IO_handle.channel = 0;
		IO_handle.active = true;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		//set enable bit low
		IO_handle.channel = 0;
		IO_handle.active = false;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		while(BoardisBusy())
		{
			ROS_INFO("Waiting.....");
			ros::Duration(0.5).sleep();
		}
		return true;
	}
	else //Board busy => can not complete service
	{
		return false;
	}
}
//Service load front Callback
bool controller::loadfrontServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	if(!BoardisBusy()) //Board is ready
	{
		//load front
		ROS_INFO("Loading front.....");
		//set enable bit low
		IO_handle.channel = 0;
		IO_handle.active = false;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		//send load front
		IO_handle.channel = 1;
		IO_handle.active = false;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		IO_handle.channel = 2;
		IO_handle.active = true;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		IO_handle.channel = 3;
		IO_handle.active = true;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		//set enable bit high
		IO_handle.channel = 0;
		IO_handle.active = true;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		//set enable bit low
		IO_handle.channel = 0;
		IO_handle.active = false;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		while(BoardisBusy())
		{
			ROS_INFO("Waiting.....");
			ros::Duration(0.5).sleep();
			if(BoardisTimeOut())
			{
				ROS_WARN("TimeOut!");
				return false;
			}
		}
		return true;
	}
	else //Board busy => can not complete service
	{
		return false;
	}
}
//Service unload front Callback
bool controller::unloadfrontServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	if(!BoardisBusy()) //Board is ready
	{
		//unload front
		ROS_INFO("Unloading front.....");
		//set enable bit low
		IO_handle.channel = 0;
		IO_handle.active = false;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		//send unload front
		IO_handle.channel = 1;
		IO_handle.active = false;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		IO_handle.channel = 2;
		IO_handle.active = false;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		IO_handle.channel = 3;
		IO_handle.active = true;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		//set enable bit high
		IO_handle.channel = 0;
		IO_handle.active = true;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		//set enable bit low
		IO_handle.channel = 0;
		IO_handle.active = false;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		while(BoardisBusy())
		{
			ROS_INFO("Waiting.....");
			ros::Duration(0.5).sleep();
		}
		return true;
	}
	else //Board busy => can not complete service
	{
		return false;
	}
}
//Service load back Callback
bool controller::loadbackServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	if(!BoardisBusy()) //Board is ready
	{
		//load back
		ROS_INFO("Loading back.....");
		//set enable bit low
		IO_handle.channel = 0;
		IO_handle.active = false;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		//send load back
		IO_handle.channel = 1;
		IO_handle.active = true;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		IO_handle.channel = 2;
		IO_handle.active = true;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		IO_handle.channel = 3;
		IO_handle.active = true;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		//set enable bit high
		IO_handle.channel = 0;
		IO_handle.active = true;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		//set enable bit low
		IO_handle.channel = 0;
		IO_handle.active = false;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		while(BoardisBusy())
		{
			ROS_INFO("Waiting.....");
			ros::Duration(0.5).sleep();
		}
		return true;
	}
	else //Board busy => can not complete service
	{
		return false;
	}
}
//Service unload back Callback
bool controller::unloadbackServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	if(!BoardisBusy()) //Board is ready
	{
		//unload back
		ROS_INFO("Unloading back.....");
		//set enable bit low
		IO_handle.channel = 0;
		IO_handle.active = false;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		//send unload back
		IO_handle.channel = 1;
		IO_handle.active = true;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		IO_handle.channel = 2;
		IO_handle.active = false;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		IO_handle.channel = 3;
		IO_handle.active = true;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		//set enable bit high
		IO_handle.channel = 0;
		IO_handle.active = true;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		//set enable bit low
		IO_handle.channel = 0;
		IO_handle.active = false;
		topicPub_IO.publish(IO_handle);
		ros::Duration(0.2).sleep();
		while(BoardisBusy())
		{
			ROS_INFO("Waiting.....");
			ros::Duration(0.5).sleep();
		}
		return true;
	}
	else //Board busy => can not complete service
	{
		return false;
	}
}
//Service robot is full? Callback
bool controller::checkloadedServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	return RobotisFull();
}
//IO Callback
void controller::IOCallBack(std_msgs::Int16 IOs)
{
	//ROS_INFO("New IO Data!!!");
	int mydata = 0;
	int array[16];
	mydata = IOs.data;
	//Splitt mydata to IO states
	for (int i = 0; i < 16; ++i) 
	{  // assuming a 16 bit int
	    	array[i] = mydata & (1 << i) ? 1 : 0;
	}

	//ROS_INFO("0: %i ---- 1: %i ---- 15: %i", array[0], array[1],array[15]);
	if(array[0] == 1)
	{
		isBusy = true;
	}
	else
	{
		isBusy = false;
	}

	if(array[1] == 1)
	{
		isTimeOut = true;
	}
	else
	{
		isTimeOut = false;
	}

	if(array[15] == 1)
	{
		isLoaded = true;
	}
	else
	{
		isLoaded = false;
	}

	newIOstates = true;
}
bool controller::getparams()
{
	return true;
}
//Check if IO Board is busy
bool controller::BoardisBusy()
{
	//Wait for new data
	newIOstates = false;
	do
	{
		ros::spinOnce();
	}while(newIOstates == false);
  
	return isBusy;
}
//Check if IO Board is timeout
bool controller::BoardisTimeOut()
{
	//Wait for new data
	newIOstates = false;
	do
	{
		ros::spinOnce();
	}while(newIOstates == false);	
	
	return isTimeOut;
}
//Check if Robot is fully loaded
bool controller::RobotisFull()
{
	//Wait for new data
	newIOstates = false;
	do
	{
		ros::spinOnce();
	}while(newIOstates == false);
	
	return isLoaded;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "conveyor_belt_control_node");
  controller my_control;
  
  ROS_INFO("Ready to control conveyor belt!");
  ros::Rate loop_rate(20); // Hz 

  	

  while(my_control.n.ok())
  {
	loop_rate.sleep();
	ros::spinOnce();
  }

  return 0;
}
