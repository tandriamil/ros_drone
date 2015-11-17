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

#include "../include/pikopter/pikopter_cmd.h"
#include "../include/pikopter/pikopter_navdata.h"

using namespace std;

char *STATION_IP = NULL;

static struct sockaddr_in addr_drone, addr_client;

int comfd = 0; // in read mode -- receive command

//command string
static char takeoff_arg[] = "290718208";
static char land_arg[] = "290717696";
static char emergency_arg[] = "290717952";

//NAV DATA HANDLING
unsigned char navdataBuffer[PACKET_SIZE];

// void fillNavDataBuffer(int alt) {
// 	static union navdata_t data;
// 	memset((char *) navdataBuffer,0,PACKET_SIZE);
// 	data.demo.tag = TAG_DEMO;
// 	data.demo.vbat_flying_percentage = 100;
// 	data.demo.altitude = alt;
// 	data.demo.theta = 0;
// 	data.demo.phi = 0;
// 	data.demo.psi =0;
// 	data.demo.vx=0;
// 	data.demo.vy=0;
// 	data.demo.vz=0;
// 	memcpy((char *) navdataBuffer, &data,PACKET_SIZE);
// }

//Parsing command buffer
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
		//fillNavDataBuffer(0);
	}

	else if(sscanf(buf, "AT*REF=%d, %d", &seq, &tcmd) == 2) {
		switch(tcmd) {
		case 290718208:
			if(tcmd != ptcmd)
				return "DECOLLAGE";
			//fillNavDataBuffer(700);
			break;

		case 290717696:
			if(tcmd != ptcmd)
				return "ATTERRISSAGE";
			//fillNavDataBuffer(0);
			break;

		case 290717952:
			if(tcmd != ptcmd)
				return "EMERGENCY";
			break;

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

int main(int argc, char *argv[]) {
	PikopterCmd pik;

	pik.addr_drone_cmd.sin_family = AF_INET;
	pik.addr_drone_cmd.sin_port = htons(5556);
	inet_aton("127.0.0.1", (struct in_addr*) &pik.addr_drone_cmd.sin_addr.s_addr);

	fd_set readfs;
	int ret, i = 0;
	socklen_t len = sizeof(addr_drone);

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
	ros::Rate loop_rate(10);

	// Open the UDP port for the navadata node
	comfd = PikopterNetwork::open_udp_socket(PORT_CMD, &pik.addr_drone_cmd, argv[1]);
	//bind(comfd, (struct sockaddr*) &pik.addr_drone_cmd, sizeof(pik.addr_drone_cmd));

	STATION_IP = argv[1];

	while(ros::ok()) {
		memset(commandBuffer, 0, PACKET_SIZE);
		
		// fprintf(stderr, "Waiting packet from %s:%d\n", inet_ntoa(addr_drone.sin_addr), ntohs(addr_drone.sin_port));
		
		// need to add a watchdog here
		int ret = recvfrom(comfd, commandBuffer, PACKET_SIZE, 0, (struct sockaddr*) STATION_IP, (socklen_t*) sizeof(STATION_IP));
		
		if(ret > 0) {
			// fprintf(stderr,"cmd %d       received: %s (%d)\n",i,commandBuffer,ret);
			
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

		else {
			if(errno == 11)
				ROS_ERROR("%s", "Receiving command time out \n");
			else
				ROS_ERROR("Receiving command, %d, failed (errno: %d)\n", i, errno);
			
			// perror("recvfrom()");
			// We should send ping again... for server
			//cmd_pub.publish(msg);
			//sendto(comfd, "\0", 1, 0, (struct sockaddr*) &addr_drone, sizeof(addr_drone));
		}

		loop_rate.sleep();	
	}

	close(comfd);
	return 0;
}