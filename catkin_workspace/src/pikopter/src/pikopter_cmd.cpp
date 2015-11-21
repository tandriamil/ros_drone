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
#include "../include/pikopter/pikopter_cmd.h"
#include "../include/pikopter/pikopter_navdata.h"

/* Declarations */
char *STATION_IP = NULL;
static struct sockaddr_in addr_drone, addr_client;
int comfd = 0; // in read mode -- receive command

/* Functions */

/*!
 * \brief Parsing command
 *
 * \param buf the buffer containing the command
 *
 * \return the command
 */
std::string parseCommand(char *buf) {
	char cmd[PACKET_SIZE];
	int seq, tcmd, p1, p2, p3, p4, p5;
	static int pseq, ptcmd = 0, pp1 = 0, pp2 = 0, pp3 = 0, pp4 = 0, pp5 = 0;

	//AT*FTRIM=7
	//AT*REF=78,290718208

	if (!buf) return NULL;

	// use sscan to parse the command
	if(sscanf(buf, "AT*FTRIM=%d", &seq) == 1) {
		tcmd = -1;
		if(tcmd != ptcmd)
			return "AT*FTRIM";
	}

	else if(sscanf(buf, "AT*REF=%d, %d", &seq, &tcmd) == 2) {
		switch(tcmd) {
		case 290718208:
			if(tcmd != ptcmd)
				return "DECOLLAGE";

		case 290717696:
			if(tcmd != ptcmd)
				return "ATTERRISSAGE";
			break;

		case 290717952:
			if(tcmd != ptcmd)
				return "EMERGENCY";

		default:
			return "UNKNOWN";
		}
	}

	else if(sscanf(buf, "AT*PCMD=%d, %d, %d, %d, %d, %d", &seq, &p1, &p2, &p3, &p4, &p5) == 6) {
		if((p1 != pp1) || (p2 != pp2) || (p3 != pp3) || (p4 != pp4) || (p5 != pp5)) {
			if(p1 || p2 || p3 || p4 || p5) {
				if ((p1 == 1) && !p2 && (p3 < 0) && !p4 && !p5)
					return "FORWARD";
				else if((p1 == 1) && !p2 && (p3 > 0) && !p4 && !p5)
					return "BACKWARD";
				else if((p1 == 1) && !p2 && !p3 && (p4 < 0) && !p5)
					return "DOWN";
				else if((p1 == 1) && !p2 && !p3 && (p4 > 0) && !p5)
					return "UP";
				else if((p1 == 1) && !p2 && !p3 && !p4 && (p5 < 0))
					return "LEFT";
				else if((p1 == 1) && !p2 && !p3 && !p4 && (p5 > 0))
					return "RIGHT";
				else {
					std::stringstream ss;
					ss << "PCMD " << seq << " " << p1 << " " << p2 << " " << p3 << " " << p4 << " " << p5 ;
					std::string s = ss.str();
					return s;
				}
			}

			else {
				return "STAY";
			}
		}
	}

	return NULL;
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
	// Instance of PikopterCmd class
	PikopterCmd pik;

	int i = MAX_CMD_NAVDATA;
	socklen_t len = sizeof(addr_drone);
	
	// Struct for handling timeout
	struct timeval tv;
	
	// set a timeout
	tv.tv_sec = 0;
  	tv.tv_usec = 100000; // 100ms

  	if (setsockopt(comfd, SOL_SOCKET, SO_RCVTIMEO,&tv,sizeof(tv)) < 0) {
  		ROS_ERROR("%s", "Error timeout");
  	}

  	// send a ping DO NOT ERASE PLEASE
  	// Allows to keep connection on
  	if (sendto(comfd,"\0",1, 0, (struct sockaddr*)&addr_drone, sizeof(addr_drone)) < 0) {
  		ROS_ERROR("%s", "sendto()");
  	}

	if(argc < 2) {
		ROS_FATAL("No IP address\n use: %s \"ip_address\"\n", argv[0]);
		return -1;
	}

	// Initialize ros for this node
	ros::init(argc, argv, "pikopter_cmd");

	// Create a NodeHandle
	ros::NodeHandle nodeHandle;
	
	// Create a publisher and advertise any nodes listening on pikopter_mavlink topic that we are going to publish
	ros::Publisher cmd_pub = nodeHandle.advertise<std_msgs::String>("pikopter_mavlink", 1000);
	
	// Frequency of how fast we loop
	ros::Rate loop_rate(10);

	// Open the UDP port for the cmd node
	comfd = PikopterNetwork::open_udp_socket(PORT_CMD, &pik.addr_drone_cmd, argv[1]);

	// The first argument is the IP address of the Raspberry PI
	STATION_IP = argv[1];

	// ROS LOOP
	while(ros::ok()) {

		// Erase buffer commandBuffer
		memset(commandBuffer, 0, PACKET_SIZE);
		
		// fprintf(stderr, "Waiting packet from %s:%d\n", inet_ntoa(addr_drone.sin_addr), ntohs(addr_drone.sin_port));
		
		// need to add a watchdog here
		int ret = recvfrom(comfd, commandBuffer, PACKET_SIZE, 0, (struct sockaddr*) STATION_IP, (socklen_t*) sizeof(STATION_IP));
		
		// if we receive something...
		if(ret > 0) {			
			// Get command
			std::string command = parseCommand((char *) commandBuffer);
			
			// Building message log
			std::stringstream ssLog;
			ssLog << "RECEIVED COMMAND: [" << command << "]";
			std::string sLog = ssLog.str();
			ROS_INFO("%s", sLog.c_str());
			
			// Building String message to publish to the topic pikopter_mavlink
			std_msgs::String msg;
			msg.data = sLog;
			cmd_pub.publish(msg);
		}

		// if nothing is received
		else {
			// if time is out
			if(errno == 11)
				ROS_ERROR("%s", "Receiving command time out \n");
			
			// if other errors occured
			else
				ROS_ERROR("Receiving command, %d, failed (errno: %d)\n", i, errno);

			// We should send ping again... for server
			sendto(comfd, "\0", 1, 0, (struct sockaddr*) &addr_drone, sizeof(addr_drone));
		}

		// Pause in loop with the given value defined in ros::rate
		loop_rate.sleep();
	}

	// close UDP socket
	close(comfd);
	return 0;
}