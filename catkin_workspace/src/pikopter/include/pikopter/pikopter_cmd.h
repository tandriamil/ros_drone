#ifndef PIKOPTER_CMD_H
#define PIKOPTER_CMD_H


/* ################################### INCLUDES ################################### */
// Pikotper common includes
#include "pikopter_common.h"



/* ################################### CONSTANTS ################################### */
// The port used for the commands
#define PORT_CMD 5556

/* ################################### Classes ################################### */
/*!
 * \brief Jakopter commands ros node
 */
class PikopterCmd {

	// Public part
	public:
		struct sockaddr_in addr_drone_cmd;
		unsigned char cmd_buffer[PACKET_SIZE];
		int cmd_fd;
};

class ExecuteCommand {
	public:
		ExecuteCommand();
		bool takeoff();

	private:
		ros::Subscriber state_sub;
		ros::ServiceClient arming_client;
		ros::ServiceClient set_mode_client;
		ros::ServiceClient tol_client;
		mavros_msgs::State current_state;
};

#endif