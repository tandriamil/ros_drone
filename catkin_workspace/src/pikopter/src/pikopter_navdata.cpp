// Include pikopter navdata headers
#include "../include/pikopter/pikopter_navdata.h"


/*!
 * \brief Main function
 *
 * \param argc The number of arguments
 * \param argv The arguments
 */
int main(int argc, char **argv) {

	// Just return the main loop of PikopterNavdata
	PikopterNavdata pn;
	return pn.main_loop(argc, argv);
}


/*!
 * \brief Main loop for the navdata node
 *
 * \param argc The number of arguments
 * \param argv The arguments
 */
int PikopterNavdata::main_loop(int argc, char **argv) {

	// Initialize ros for this node
	ros::init(argc, argv, "pikopter_navdata");

	// Create a node handle (fully initialize ros)
	ros::NodeHandle navdata_node_handle;

	// Check the command syntax
	if (argc != 2) {
		ROS_FATAL("Command syntax is:\n \t%s \"ip_address\"\n", argv[0]);
		return -1;
	}

	// Put the rate for this node
	ros::Rate loop_rate(NAVDATA_INTERVAL);

	// Debug message
	ROS_DEBUG("Ros initialized with a rate of %u", NAVDATA_INTERVAL);

	// Open the UDP port for the navadata node
	navdata_fd = PikopterNetwork::open_udp_socket(PORT_NAVDATA, &addr_drone_navdata, argv[1]);

	// Get the length of the socket
	socklen_t len = sizeof(addr_drone_navdata);

	// Then the main loop for the node
	while(ros::ok()) {

		// Try to send a packet
		if (sendto(navdata_fd, navdata_buffer, PACKET_SIZE, 0, (struct sockaddr*)&addr_drone_navdata, sizeof(addr_drone_navdata)) < 0) {
			ROS_ERROR("Error during sending a navdata packet");
		}

		// Wait the next wake up
		loop_rate.sleep();
	}

	// Close the navdata fd at the end
	if (navdata_fd) close(navdata_fd);

	// Return the correct end status
	return 0;
}