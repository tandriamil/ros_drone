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

void fillNavDataBuffer(int alt) {
	static union navdata_t data;
	memset((char *) navdataBuffer,0,PACKET_SIZE);
	data.demo.tag = TAG_DEMO;
	data.demo.vbat_flying_percentage = 100;
	data.demo.altitude = alt;
	data.demo.theta = 0;
	data.demo.phi = 0;
	data.demo.psi =0;
	data.demo.vx=0;
	data.demo.vy=0;
	data.demo.vz=0;
	memcpy((char *) navdataBuffer, &data,PACKET_SIZE);
}

//Parsing command buffer
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
			fprintf(stderr,"Found FTRIM\n");
		fillNavDataBuffer(0);
	}

	else if(sscanf(buf, "AT*REF=%d, %d", &seq, &tcmd) == 2) {
		switch(tcmd) {
		case 290718208:
			if(tcmd != ptcmd)
				fprintf(stderr, "received DECOLLAGE %d\n", ptcmd);
			fillNavDataBuffer(700);
			break;

		case 290717696:
			if(tcmd != ptcmd)
				fprintf(stderr, "received ATTERRISSAGE\n");
			fillNavDataBuffer(0);
			break;

		case 290717952:
			if(tcmd != ptcmd)
				fprintf(stderr, "received EMERGENCY\n");
			break;

		default:
			fprintf(stderr, "Found ????\n");
		}
	}

	else if(sscanf(buf, "AT*PCMD=%d, %d, %d, %d, %d, %d", &seq, &p1, &p2, &p3, &p4, &p5) == 6) {
		if((p1 != pp1) || (p2 != pp2) || (p3 != pp3) || (p4 != pp4) || (p5 != pp5)) {
			if(p1 || p2 || p3 || p4 || p5) {
				if ((p1 == 1) && !p2 && (p3 < 0) && !p4 && !p5)
					fprintf(stderr, "received FORWARD\n");
				else if((p1 == 1) && !p2 && (p3 > 0) && !p4 && !p5)
					fprintf(stderr, "received BACKWARD\n");
				else if((p1 == 1) && !p2 && !p3 && (p4 < 0) && !p5)
					fprintf(stderr, "received DOWN\n");
				else if((p1 == 1) && !p2 && !p3 && (p4 > 0) && !p5)
					fprintf(stderr, "received UP\n");
				else if((p1 == 1) && !p2 && !p3 && !p4 && (p5 < 0))
					fprintf(stderr, "received LEFT\n");
				else if((p1 == 1) && !p2 && !p3 && !p4 && (p5 > 0))
					fprintf(stderr, "received RIGHT\n");
				else 
					fprintf(stderr, "received PCMD: %d, %d, %d, %d, %d, %d\n", seq, p1, p2, p3, p4, p5);
			}

			else {
				fprintf(stderr, "received STAY\n");
			}
		}
		pp1 = p1; pp2 = p2; pp3 = p3; pp4 = p4; pp5 = p5;
	}
	ptcmd = tcmd;
}

//////////////////// Parrot channels
unsigned char commandBuffer[PACKET_SIZE+1];

int openCmdTcpChannel() {
	int i = MAX_CMD_NAVDATA;
	socklen_t len = sizeof(addr_drone);
	struct timeval tv;
	
	// set a timeout
	tv.tv_sec = 0;
	tv.tv_usec = 100000; // 100ms
	
	if(setsockopt(comfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
		perror("Error timeout");
	}

	// send a ping
	if(sendto(comfd, "\0", 1, 0, (struct sockaddr*) &addr_drone, sizeof(addr_drone)) < 0) {
		perror("sendto()");
	}

	while(i) {
		i--;
		memset(commandBuffer, 0, PACKET_SIZE);
		// fprintf(stderr, "Waiting packet from %s:%d\n", inet_ntoa(addr_drone.sin_addr), ntohs(addr_drone.sin_port));
		// need to add a watchdog here
		int ret = recvfrom(comfd, commandBuffer, PACKET_SIZE, 0, (struct sockaddr*) &addr_drone, &len);
		
		if(ret > 0) {
			// fprintf(stderr,"cmd %d       received: %s (%d)\n",i,commandBuffer,ret);
			parseCommand((char *) commandBuffer);
		}

		else {
			if(errno == 11)
				fprintf(stderr, "Receiving command time out \n");
			else
				fprintf(stderr, "Receiving command, %d, failed (%d)\n", i, errno);
			
			// perror("recvfrom()");
			// We should send ping again... for server
			sendto(comfd, "\0", 1, 0, (struct sockaddr*) &addr_drone, sizeof(addr_drone));
			usleep(100*1000); // wait 0.1 second before going to next step
		}

		usleep(30*1000);
	}

	if(comfd) close(comfd);
	
	return 0;
}

int main(int argc, char *argv[]) {
	PikopterNetwork pik;

	fd_set readfs;
	int ret;

	if(argc < 2) {
		fprintf(stderr, "No IP addres\n use: %s \"ip_address\"\n", argv[0]);
		return -1;
	}

	// Initialize ros for this node
	ros::init(argc, argv, "pikopter_cmd");

	STATION_IP = argv[1];    
	fprintf(stderr, "starting pikopter server (%s)...\n", STATION_IP);

	// CA MARCHE PAS -> Erreur affichée :
	// référence indéfinie vers « PikopterNetwork::open_udp_socket(int, sockaddr_in*, char*) »
	comfd = pik.open_udp_socket(PORT_CMD, &addr_drone, STATION_IP);

	openCmdTcpChannel();

	return 0;
}