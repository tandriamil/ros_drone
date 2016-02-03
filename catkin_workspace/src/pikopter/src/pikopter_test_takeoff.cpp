#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#include <limits.h>
#include <pthread.h>
#include <sys/uio.h>
#include <netinet/in.h>
#include <netdb.h>

/* Import Headers */
#include "../include/pikopter/pikopter_common.h"
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/TwistStamped.h>


/*!
 * \brief Launcher of Ros node cmd
 *
 * \param argc Number of parameters
 * \param argv The arguments
 *
 */
int main(int argc, char *argv[]) {
	// Initialize ros for this node
	ros::init(argc, argv, "pikopter_test_takeoff");
	
	// Struct for handling timeout
	struct timeval tv;
	
	// set a timeout
	tv.tv_sec = 0;
  	tv.tv_usec = 100000; // 100ms

	// Create a NodeHandle
	ros::NodeHandle nodeHandle;

	ros::Publisher velocity_pub = nodeHandle.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 100);	

	mavros_msgs::SetMode srvGuided;
	mavros_msgs::CommandTOL srvTakeOffLand;
	mavros_msgs::CommandBool srvArmed;
	geometry_msgs::TwistStamped msgMove;

	srvGuided.request.custom_mode = "GUIDED";
	srvGuided.request.base_mode = 0;
	srvTakeOffLand.request.altitude = 50;
	srvArmed.request.value = true;

	if(ros::service::call("/mavros/set_mode", srvGuided))
		ROS_INFO("Mode MAV_MODE_GUIDED activated.");
	else
		ROS_ERROR("Problem occured on calling set_mode service");

	if(ros::service::call("/mavros/cmd/arming", srvArmed))
		ROS_INFO("Arming command called.");
	else
		ROS_ERROR("Problem occured on calling arming command");

	if(ros::service::call("/mavros/cmd/takeoff", srvTakeOffLand))
		ROS_INFO("Takeoff called");
	else
		ROS_ERROR("Problem occured on calling takeoff command");

	sleep(5);
	
	msgMove.header.stamp = ros::Time::now();
	msgMove.twist.linear.x = 10;

	msgMove.twist.angular.x = 10;
	msgMove.twist.angular.y = 20;
	msgMove.twist.angular.z = 30;

	velocity_pub.publish(msgMove);

	ROS_INFO("Drone is moving :");
	ROS_INFO("Linear velocity -> x = %f, y = %f, z = %f", msgMove.twist.linear.x, msgMove.twist.linear.y, msgMove.twist.linear.z);
	ROS_INFO("Angular velocity -> x = %f, y = %f, z = %f", msgMove.twist.angular.x, msgMove.twist.angular.y, msgMove.twist.angular.z);

	sleep(15);

	msgMove.header.stamp = ros::Time::now();
	msgMove.twist.linear.x = 0;
	msgMove.twist.angular.z = 0;

	velocity_pub.publish(msgMove);
	
	ROS_INFO("Drone is stopping :");
	ROS_INFO("Linear velocity -> x = %f, y = %f, z = %f", msgMove.twist.linear.x, msgMove.twist.linear.y, msgMove.twist.linear.z);
	ROS_INFO("Angular velocity -> x = %f, y = %f, z = %f", msgMove.twist.angular.x, msgMove.twist.angular.y, msgMove.twist.angular.z);

	sleep(5);

	if(ros::service::call("/mavros/cmd/land", srvTakeOffLand))
		ROS_INFO("Land called");
	else
		ROS_ERROR("Problem occured on calling land command");
	return 0;
}