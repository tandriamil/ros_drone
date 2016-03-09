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
		bool land();
		void forward(int* accel);
		void backward(int* accel);
		bool down(int* accel);
		bool up(int* accel);
		bool left();
		bool right();
		float* convertSpeedARDroneToRate(int* speed);
		float* getCurrentAltitude();

	private:
		ros::Subscriber state_sub;
		ros::ServiceClient arming_client;
		ros::ServiceClient set_mode_client;
		ros::ServiceClient takeoff_client;
		ros::ServiceClient land_client;
		ros::Publisher velocity_pub;
		geometry_msgs::TwistStamped msgMove;
};

#endif