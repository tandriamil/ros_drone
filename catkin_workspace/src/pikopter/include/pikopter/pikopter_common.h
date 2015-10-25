#ifndef PIKOPTER_COMMON_H
#define PIKOPTER_COMMON_H


/* ################################### INCLUDES ################################### */
// System librairies used by pikopter
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>

// Ros librairies in order to use them
#include "ros_common.h"
#include "ros_com_channel.h"
#include "ros_com_master.h"



/* ################################### CONSTANTS ################################### */
// Size of the packets (navdata or command)
#define PACKET_SIZE 256

// Errors result got if error during socket manipulation
#define SOCKET_ERROR -1
#define SOCKET_ERRNO errno

// DUNO
#define MSG_NOBLOCK (1 << 0)   

// DUNO
#define CHUCK_BUFFER_SIZE 14000

// Maximum packets exchange (hours of flying)
#define MAXCMDNAVDATA 10000000



/* ################################### Classes ################################### */
/*!
 * \brief Pikopter network utilities
 */
class PikopterNetwork {

	// Public methods
	public:
		open_udp_socket(int portnum, struct sockaddr_in *serv_addr);
};

#endif