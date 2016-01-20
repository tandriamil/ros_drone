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


/* Declarations */
char *STATION_IP = NULL;
static struct sockaddr_in addr_drone;
int comfd = 0; // in read mode -- receive command

/* Functions */

/*!
 * \brief Parsing command
 *
 * \param buf the buffer containing the command
 *
 * \return the command
 */
void parseCommand(char *buf) {
	char cmd[PACKET_SIZE];
	int seq, tcmd, p1, p2, p3, p4, p5;
	static int pseq, ptcmd = 0, pp1 = 0, pp2 = 0, pp3 = 0, pp4 = 0, pp5 = 0;

	//AT*FTRIM=7
	//AT*REF=78,290718208

	if (!buf) return;

	// use sscan to parse the command
	if(sscanf(buf, "AT*FTRIM=%d", &seq) == 1) {
		tcmd = -1;
		if(tcmd != ptcmd)
			fprintf(stderr, "%s\n", "AT*FTRIM");
	}

	else if(sscanf(buf, "AT*REF=%d, %d", &seq, &tcmd) == 2) {
		switch(tcmd) {
		case 290718208:
			if(tcmd != ptcmd)
				fprintf(stderr, "%s\n", "DECOLLAGE");

		case 290717696:
			if(tcmd != ptcmd)
				fprintf(stderr, "%s\n","ATTERRISSAGE");
			break;

		case 290717952:
			if(tcmd != ptcmd)
				fprintf(stderr, "%s\n","EMERGENCY");

		default:
			fprintf(stderr, "%s\n","UNKNOWN");
		}
	}

	else if(sscanf(buf, "AT*PCMD=%d, %d, %d, %d, %d, %d", &seq, &p1, &p2, &p3, &p4, &p5) == 6) {
		if((p1 != pp1) || (p2 != pp2) || (p3 != pp3) || (p4 != pp4) || (p5 != pp5)) {
			if(p1 || p2 || p3 || p4 || p5) {
				if ((p1 == 1) && !p2 && (p3 < 0) && !p4 && !p5)
					fprintf(stderr, "%s\n","FORWARD");
				else if((p1 == 1) && !p2 && (p3 > 0) && !p4 && !p5)
					fprintf(stderr, "%s\n","BACKWARD");
				else if((p1 == 1) && !p2 && !p3 && (p4 < 0) && !p5)
					fprintf(stderr, "%s\n","DOWN");
				else if((p1 == 1) && !p2 && !p3 && (p4 > 0) && !p5)
					fprintf(stderr, "%s\n","UP");
				else if((p1 == 1) && !p2 && !p3 && !p4 && (p5 < 0))
					fprintf(stderr, "%s\n","LEFT");
				else if((p1 == 1) && !p2 && !p3 && !p4 && (p5 > 0))
					fprintf(stderr, "%s\n","RIGHT");
				else {
					fprintf(stderr,"received PCMD: %d,%d,%d,%d,%d,%d\n",seq,p1,p2,p3,p4,p5);
				}
			}

			else {
				fprintf(stderr, "%s\n","STAY");
			}
		}
		pp1 = p1; pp2= p2; pp3= p3; pp4= p4; pp5= p5;
	}

	ptcmd = tcmd;
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
	
	// Open the UDP port for the cmd node
	comfd = PikopterNetwork::open_udp_socket(PORT_CMD, &addr_drone, argv[1]);

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

	// Create a NodeHandle
	ros::NodeHandle nodeHandle;
	
	// Create a publisher and advertise any nodes listening on pikopter_mavlink topic that we are going to publish
	ros::Publisher cmd_pub = nodeHandle.advertise<std_msgs::String>("mavros", 1000);
	
	// Frequency of how fast we loop
	ros::Rate loop_rate(10);


	// The first argument is the IP address of the Raspberry PI
	STATION_IP = argv[1];

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
			parseCommand((char *) commandBuffer);
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

		// Pause in loop with the given value defined in ros::rate
		loop_rate.sleep();
	}

	// close UDP socket
	close(comfd);

	ros::shutdown();
	return 0;
}