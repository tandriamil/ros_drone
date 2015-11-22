// Include pikopter navdata headers
#include "../include/pikopter/pikopter_navdata.h"


/*!
 * \brief Constructor of PikopterNavdata
 *
 * \param The ip adress on which we create the udp socket
 */
PikopterNavdata::PikopterNavdata(char *ip_adress) {

	// Open the UDP port for the navadata node
	navdata_fd = PikopterNetwork::open_udp_socket(PORT_NAVDATA, &addr_drone_navdata, ip_adress);

	// The other attributes got their memory allocated automatically
}


/*!
 * \brief Destructor of PikopterNavdata
 */
PikopterNavdata::~PikopterNavdata() {

	// Close the UDP socket
	close(navdata_fd);

	// The other attributes got their memory deallocated automatically
}


/*!
 * \brief Function called when a message is published on X node
 */
/***Mettre le sendto() dans la fonction chatterCallback **/
void PikopterNavdata::chatterCallback(const std_msgs::String::ConstPtr& msg) {
	ROS_INFO("GOT MESSAGE FROM MAVLINK: [%s]", msg->data.c_str());
}


/*!
 * \brief Main function
 *
 * \param argc The number of arguments
 * \param argv The arguments
 */
int main(int argc, char **argv) {

	/* ######################### Initialization ######################### */

	// Create a pikopter navdata object
	PikopterNavdata *pn = new PikopterNavdata(argv[1]);
	
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
	ros::Rate loop_rate(NAVDATA_LOOP_RATE);

	// Debug message
	ROS_DEBUG("Ros initialized with a rate of %u", NAVDATA_LOOP_RATE);

	// Here we receive the navdatas from pikopter_mavlink
	//ros::Subscriber sub = navdata_node_handle.subscribe("mavlink", 1000, pn->chatterCallback);

	// Here we send it to jakopter
	if (sendto(pn->navdata_fd, pn->navdata_buffer, PACKET_SIZE, 0, (struct sockaddr*)&pn->addr_drone_navdata, sizeof(pn->addr_drone_navdata)) < 0) {
		ROS_ERROR("Error during sending a navdata packet");
	}

	/**
	 * ros::spin() will enter a loop, pumping callbacks.  With this version, all
	 * callbacks will be called from within this thread (the main one).  ros::spin()
	 * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	 */
	ros::spin();

	// Destroy the PikopterNavdata object before leaving the program
	delete pn;

	// Return the correct end status
	return 0;
}