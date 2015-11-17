#ifndef PIKOPTER_CMD_H
#define PIKOPTER_CMD_H


/* ################################### INCLUDES ################################### */
// Pikotper common includes
#include "pikopter_common.h"



/* ################################### CONSTANTS ################################### */
// The port used for the navdatas
#define PORT_CMD 5556

// Basic commands
#define CMD_TAKEOFF_ARG "290718208"
#define CMD_LAND_ARG "290717696"
#define CMD_EMERGENCY_ARG "290717952"



/* ################################### Classes ################################### */
/*!
 * \brief Jakopter navdatas ros node
 */
class PikopterCmd {

	// Private part
	public:
		struct sockaddr_in addr_drone_cmd;
		unsigned char cmd_buffer[PACKET_SIZE];
		int cmd_fd;
};

#endif