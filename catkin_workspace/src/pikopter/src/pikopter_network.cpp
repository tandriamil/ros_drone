// Include the common.h
#include "../include/pikopter/pikopter_common.h"


/*!
 * \brief Open an UDP socket
 *
 * \param portnum The port number of the socket to create
 * \param serv_addr The adress of the server
 * \param station_ip The ip of the station
 *
 * \return The file descriptor or -1 if error
 */
int PikopterNetwork::open_udp_socket(int portnum, struct sockaddr_in *serv_addr, char *station_ip) {
	
	// The file descriptor
	int listenfd = ERROR_ENCOUNTERED;

	// Only if an address is given
	if (serv_addr) {
		fprintf(stderr, "INFO: Starting socket\n");

		// Create an UDP socket at the port
		listenfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

		// Put the serv_addr memory to 0
		memset(serv_addr, '0', sizeof(struct sockaddr_in));

		// And now put it as an AF_INET socket and put the port number
		serv_addr->sin_family = AF_INET;
		serv_addr->sin_port = htons(portnum);

		// If en error occurs during converting the station's IP into formatted IP format
		if (inet_aton(station_ip, &(serv_addr->sin_addr)) == 0) {
			fprintf(stderr, "ERROR: inet_aton() failed (port %d)\n", portnum);
			exit(1);
		}
		fprintf(stderr, "INFO: connect socket (%s:%d) done\n", station_ip, portnum);
	}

	// Return the value of the fd or -1 if error
	return listenfd;
}