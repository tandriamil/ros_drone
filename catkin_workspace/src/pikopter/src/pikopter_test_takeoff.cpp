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

	ros::ServiceClient clientMode = nodeHandle.serviceClient<mavros_msgs::SetMode>("/mavros_msgs/set_mode");

	ros::ServiceClient clientTOL = nodeHandle.serviceClient<mavros_msgs::CommandTOL>("/mavros_msgs/cmd/takeoff");

	mavros_msgs::SetMode srvMode;
	mavros_msgs::CommandTOL srvTOL;

	srvMode.request.base_mode = 216; // MAV_MODE_GUIDED_ARMED
	srvTOL.request.altitude = 2;

	if(clientMode.call(srvMode)) {
		ROS_INFO("Mode MAV_MODE_GUIDED_ARMED activated.");
	}
	clientTOL.call(srvTOL);

	return 0;
}