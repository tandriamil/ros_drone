/* Import System */
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
#include <mavros_msgs/State.h>

#include "../include/pikopter/pikopter_cmd.h"

using namespace std;


/* Declarations */
//char *STATION_IP = NULL;
static struct sockaddr_in addr_drone;
int comfd = 0; // in read mode -- receive command
mavros_msgs::State current_state;

typedef struct command {
	string cmd;
	int seq;
	int tcmd;
	int param1;
	int param2;
	int param3;
	int param4;
	int param5;
} Command;


/* Functions */

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

/**
 * Wait for any custom or mavros service passed in parameter during a fixed timeout.
 *
 */
void waitForService(const std::string service) {
	bool mavros_available = ros::service::waitForService(service, MAVROS_WAIT_TIMEOUT);
	if (!mavros_available) {
		ROS_FATAL("Mavros not launched, timeout of %dms reached, exiting...", MAVROS_WAIT_TIMEOUT);
		ROS_INFO("Maybe the service you asked does not exist");
		exit(ERROR_ENCOUNTERED);
	}
}

/**
 * Constructor
 * Initialize all mavros services used.
 * Wait for all service to be ready.
 * NodeHandle advertising for all topics used.
 */
ExecuteCommand::ExecuteCommand() {
	ros::NodeHandle nh;
	state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    tol_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/takeoff");

    ROS_INFO("Wait for land service");
	waitForService("/mavros/cmd/land");

	ROS_INFO("Wait for set_mode service");
	waitForService("/mavros/set_mode");
	
	ROS_INFO("Wait for set_mode service");
	waitForService("/mavros/cmd/arming");

	velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 100);
}

/**
 * Convert int sent by Jakopter to a rate.
 * This int is received for forward and backward movement.
 * If the rate is positive it is a forward.
 * Otherwise it is a backward.
 */
float* ExecuteCommand::convertSpeedARDroneToRate(int* speed) {
	float* rate;
	switch(*speed) {
		case 1028443341 : 
			*rate = 0.05;
			break;
		case 1036831949 :
			*rate = 0.1;
			break;
		case 1045220557 :
			*rate = 0.2;
			break;
		case 1048576000 :
			*rate = 0.25;
			break;
		case 1056964608 :
			*rate = 0.5;
			break;
		case 1061158912 :
			*rate = 0.75;
			break;
		case 1065353216 :
			*rate = 1.0;
			break;
		case -1119040307 :
			*rate = -0.05;
			break;
		case -1110651699 :
			*rate = -0.1;
			break;
		case -1102263091 :
			*rate = -0.2;
			break;
		case -1098907648 :
			*rate = -0.25;
			break;
		case -1090519040 :
			*rate = -0.5;
			break;
		case -1086324736 :
			*rate = -0.75;
			break;
		case -1082130432 :
			*rate = -1.0;
			break;
		default :
			break;
	}
	return rate;
}

float* ExecuteCommand::getCurrentAltitude() {
	
}

/**
 * Takeoff command.
 * First, it will change mode to GUIDED mode.
 * Then it will arm the wehicle.
 * By default the altitude reached after taking off is 50 (meters).
 * If change mode or arming vehicle failed it will return false.
 * Otherwise if the vehicle has taken off it will return true.
 */
bool ExecuteCommand::takeoff() {
	mavros_msgs::SetMode srvGuided;
	mavros_msgs::CommandTOL srvTakeOffLand;
	mavros_msgs::CommandBool srvArmed;

	srvGuided.request.custom_mode = "GUIDED";
	srvGuided.request.base_mode = 0;
	srvTakeOffLand.request.altitude = 50;
	srvArmed.request.value = true;

	set_mode_client.call(srvGuided);
	if (srvGuided.response.success) {
		ROS_INFO("Guided mode enabled");
	} else {
		return false;
	}

	arming_client.call(srvArmed);
    if (srvArmed.response.success) {
    	ROS_INFO("Drone armed");
    } else {
		return false;
	}

    tol_client.call(srvTakeOffLand);
	if (srvTakeOffLand.response.success) {
		ROS_INFO("Drone flying");
	} else {
		return false;
	}
	return true;
}

/**
 * Land command.
 * First, it will change mode to GUIDED mode if it's not the case.
 * If change mode it will return false.
 * Otherwise if the vehicle has landed it will return true.

 */
bool ExecuteCommand::land() {
	mavros_msgs::SetMode srvGuided;
	mavros_msgs::CommandTOL srvTakeOffLand;
	mavros_msgs::CommandBool srvArmed;

	srvGuided.request.custom_mode = "GUIDED";
	srvGuided.request.base_mode = 0;
	srvTakeOffLand.request.altitude = 0;
	srvArmed.request.value = true;

	set_mode_client.call(srvGuided);
	if (srvGuided.response.success) {
		ROS_INFO("Guided mode enabled");
	} else {
		return false;
	}

    tol_client.call(srvTakeOffLand);
	if (srvTakeOffLand.response.success) {
		ROS_INFO("Drone lands");
	} else {
		return false;
	}
	return true;
}

/**
 * Forward command.
 * Given an int corresponding to forward/backaward movement (sent by Jakopter), convert this int to a rate.
 * With this rate we multiply by 10 so the maximum forward movement that we can received is 10 meters (1.0 * 10).
 * The minimum is 1 meter (0.1 * 10)
 */
void ExecuteCommand::forward(int* accel) {
	float* rate;

	rate = convertSpeedARDroneToRate(accel);
	msgMove.twist.linear.x = *rate * 10;
	velocity_pub.publish(msgMove);
}

/**
 * Backward command.
 * Given an int corresponding to forward/backward movement (sent by Jakopter), convert this int to a rate.
 * With this rate we multiply by 10 so the maximum backward movement that we can received is 10 meters (-1.0 * 10).
 * The minimum is 1 meter (-0.1 * 10)
 */
void ExecuteCommand::backward(int* accel) {
	float* rate;

	rate = convertSpeedARDroneToRate(accel);
	msgMove.twist.linear.x = *rate * 10;
	velocity_pub.publish(msgMove);
}

bool ExecuteCommand::down(int* accel) {
	float* rate;
	mavros_msgs::SetMode srvGuided;
	mavros_msgs::CommandTOL srvTakeOffLand;
	mavros_msgs::CommandBool srvArmed;

	rate = convertSpeedARDroneToRate(accel);

	srvGuided.request.custom_mode = "GUIDED";
	srvGuided.request.base_mode = 0;
	srvTakeOffLand.request.altitude = *rate * 10 + getCurrentAltitude();
	srvArmed.request.value = true;

	set_mode_client.call(srvGuided);
	if (srvGuided.response.success) {
		ROS_INFO("Guided mode enabled");
	} else {
		return false;
	}

    tol_client.call(srvTakeOffLand);
	if (srvTakeOffLand.response.success) {
		ROS_INFO("Drone lands");
	} else {
		return false;
	}
	return true;
}

bool ExecuteCommand::up() {
	return false;
}

bool ExecuteCommand::left() {
	return false;
}

bool ExecuteCommand::right() {
	return false;	
}



/*!
 * \brief Parsing command
 *
 * \param buf the buffer containing the command
 *
 * \return the command
 */
Command parseCommand(char *buf, ExecuteCommand executeCommand) {
	Command command;
	char cmd[PACKET_SIZE];
	int seq, tcmd, p1, p2, p3, p4, p5;
	
	static int pseq, ptcmd = 0, pp1 = 0, pp2 = 0, pp3 = 0, pp4 = 0, pp5 = 0;

	//AT*FTRIM=7
	//AT*REF=78,290718208

	command.seq = 0;
	command.tcmd = 0;
	command.param1 = 0;
	command.param2 = 0;
	command.param3 = 0;
	command.param4 = 0;
	command.param5 = 0;


	if (!buf) return command;

	// use sscan to parse the command
	if(sscanf(buf, "AT*FTRIM=%d", &seq) == 1) {
		tcmd = ERROR_ENCOUNTERED;
		if(tcmd != ptcmd) {
			fprintf(stderr, "%s\n", "AT*FTRIM");
		}
		command.cmd = "AT*FTRIM";
	}

	else if(sscanf(buf, "AT*REF=%d, %d", &seq, &tcmd) == 2) {
		switch(tcmd) {
		case 290718208:
			if(tcmd != ptcmd) {
				fprintf(stderr, "%s\n", "DECOLLAGE");
				executeCommand.takeoff();
			}
			break;

		case 290717696:
			if(tcmd != ptcmd) {
				fprintf(stderr, "%s\n","ATTERRISSAGE");
				executeCommand.land();
			}
			break;

		case 290717952:
			if(tcmd != ptcmd)
				fprintf(stderr, "%s\n","EMERGENCY");
			break;

		default:
			fprintf(stderr, "%s\n","UNKNOWN");
			break;
		}
		command.cmd = "AT*REF";
	}

	else if(sscanf(buf, "AT*PCMD=%d, %d, %d, %d, %d, %d", &seq, &p1, &p2, &p3, &p4, &p5) == 6) {
		if((p1 != pp1) || (p2 != pp2) || (p3 != pp3) || (p4 != pp4) || (p5 != pp5)) {
			if(p1 || p2 || p3 || p4 || p5) {
				if ((p1 == 1) && !p2 && (p3 < 0) && !p4 && !p5){
					fprintf(stderr, "%s\n","FORWARD");
					executeCommand.forward(&p3);
				}
				else if((p1 == 1) && !p2 && (p3 > 0) && !p4 && !p5) {
					fprintf(stderr, "%s\n","BACKWARD");
					executeCommand.backward(&p3);
				}
				else if((p1 == 1) && !p2 && !p3 && (p4 < 0) && !p5) {
					fprintf(stderr, "%s\n","DOWN");
					executeCommand.down();
				}
				else if((p1 == 1) && !p2 && !p3 && (p4 > 0) && !p5) {
					fprintf(stderr, "%s\n","UP");
					executeCommand.up();
				}
				else if((p1 == 1) && !p2 && !p3 && !p4 && (p5 < 0)) {
					fprintf(stderr, "%s\n","LEFT");
					executeCommand.left();
				}
				else if((p1 == 1) && !p2 && !p3 && !p4 && (p5 > 0)) {
					fprintf(stderr, "%s\n","RIGHT");
					executeCommand.right();
				}
				else {
					fprintf(stderr,"received PCMD: %d,%d,%d,%d,%d,%d\n",seq,p1,p2,p3,p4,p5);
				}
			}

			else {
				fprintf(stderr, "%s\n","STAY");
			}
		}
		pp1 = p1; pp2= p2; pp3= p3; pp4= p4; pp5= p5;
		command.cmd = "AT*PCMD";
	}
	
	ptcmd = tcmd;

	command.seq = seq;
	command.param1 = pp1;
	command.param2 = pp2;
	command.param3 = pp3;
	command.param4 = pp4;
	command.param5 = pp5;
	command.tcmd = ptcmd;
	

	return command;
}

//////////////////// Parrot channels
unsigned char commandBuffer[PACKET_SIZE+1];

/*!
 * \brief Launcher of Ros node cmd
 *
 * \param argc Number of parameters
 * \param argv The arguments
 *
 */
int main(int argc, char *argv[]) {
	// Initialize ros for this node
	ros::init(argc, argv, "pikopter_cmd");

	// Create a node handle (fully initialize ros)
	ros::NodeHandle cmd_node_handle;

	ros::NodeHandle cmd_private_nh("~");

	//ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 100);

	std::string ip;

	if(!cmd_private_nh.getParam("ip", ip)) {
		ROS_FATAL("Missing ip parameter");
		return ERROR_ENCOUNTERED;
	}

	char* cstr = new char[ip.length() + 1];
	strcpy(cstr, ip.c_str());

	ros::start();

	// Instance of PikopterCmd class
	PikopterCmd pik;

	int i = MAX_CMD_NAVDATA;
	socklen_t len = sizeof(addr_drone);
	
	// Struct for handling timeout
	struct timeval tv;
	
	// set a timeout
	tv.tv_sec = 0;
  	tv.tv_usec = 100000; // 100ms

  	Command command;

  	ROS_INFO("Adresse ip : %s", cstr);
	
	// Open the UDP port for the cmd node
	comfd = PikopterNetwork::open_udp_socket(PORT_CMD, &addr_drone, cstr);

  	if (setsockopt(comfd, SOL_SOCKET, SO_RCVTIMEO,&tv,sizeof(tv)) < 0) {
  		ROS_ERROR("%s", "Error timeout");
  	}

  	// send a ping DO NOT ERASE PLEASE
  	// Allows to keep connection on
  	if (sendto(comfd,"\0",1, 0, (struct sockaddr*)&addr_drone, sizeof(addr_drone)) < 0) {
  		ROS_ERROR("%s", "sendto()");
  	}


  	/* This test is no more used while we use file .launch to launch this node */
	// if(argc < 2) {
	// 	ROS_FATAL("No IP address\n use: %s \"ip_address\"\n", argv[0]);
	// 	return ERROR_ENCOUNTERED;
	// }

	ExecuteCommand executeCommand;

	delete [] cstr;

	// ROS LOOP
	while(ros::ok()) {

		// Erase buffer commandBuffer
		memset(commandBuffer, 0, PACKET_SIZE);
		
		// fprintf(stderr, "Waiting packet from %s:%d\n", inet_ntoa(addr_drone.sin_addr), ntohs(addr_drone.sin_port));
		
		// need to add a watchdog here
		int ret = recvfrom(comfd, commandBuffer, PACKET_SIZE, 0, (struct sockaddr*) &addr_drone, &len);
		
		// if we receive something...
		if(ret > 0) {			
			// Get command
			printf("%s\n", commandBuffer);
			command = parseCommand((char *) commandBuffer, executeCommand);
			
			if(!command.cmd.empty()) {
				cout << "Command received : " << command.cmd << "\n";
			}
			printf("Seq number received : %d\n", command.seq);
			printf("tcmd received : %d\n", command.tcmd);
			printf("Param 1 received : %d\n", command.param1);
			printf("Param 2 received : %d\n", command.param2);
			printf("Param 3 received : %d\n", command.param3);
			printf("Param 4 received : %d\n", command.param4);
			printf("Param 5 received : %d\n", command.param5);

			// Create a publisher and advertise any nodes listening on pikopter_mavlink topic that we are going to publish
			
			/* --- A MODIFIER --- */
			
			/* -- Le topic sur lequel ce noeud doit publier dépend de la commande reçue -- */
			// ros::Publisher cmd_pub = nodeHandle.advertise<std_msgs::String>("mavros", 1000);

			/* ------------------ */

		}

		// if nothing is received
		else {
			// if time is out
			if(errno != 11)
			// 	ROS_ERROR("%s", "Receiving command time out \n");
			
			// // if other errors occured
			// else
				ROS_ERROR("Receiving command, %d, failed (errno: %d)\n", i, errno);

			// We should send ping again... for server
			sendto(comfd, "\0", 1, 0, (struct sockaddr*) &addr_drone, sizeof(addr_drone));
		}

		ros::spinOnce();
	}

	// close UDP socket
	close(comfd);

	ros::shutdown();
	return NO_ERROR_ENCOUNTERED;
}

