#ifndef PIKOPTER_CMD_H
#define PIKOPTER_CMD_H


/* ################################### INCLUDES ################################### */
// Pikotper common includes
#include "pikopter_common.h"

#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"



/* ################################### CONSTANTS ################################### */
// The port used for the commands
#define PORT_CMD 5556
#define MAX_SPEED_CMD 10
#define MAX_VEL_TURN_CMD 45


#define RATIO_Z 1

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
		void down(int* accel);
		void up(int* accel);
		void left(int* accel);
		void right(int* accel);
		void slide_left(int* accel);
		void slide_right(int* accel);
		float* convertSpeedARDroneToRate(int* speed);
		void cmd_received();

	private:
		ros::Subscriber state_sub;
		ros::ServiceClient arming_client;
		ros::ServiceClient set_mode_client;
		ros::ServiceClient takeoff_client;
		ros::ServiceClient land_client;
		ros::ServiceClient command_long_client;
		ros::Publisher velocity_pub;
		ros::Publisher attitude_pub;
		geometry_msgs::TwistStamped msgMove;
		geometry_msgs::PoseStamped msgAttitude;

		ros::Publisher navdatas;
};

#endif