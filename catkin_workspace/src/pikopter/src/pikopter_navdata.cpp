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
 * \brief Fill the navdata buffer
 */
void fillNavDataBuffer(char* buffer, int alt) {

	// Clean navdata_buffer
	memset(buffer, 0, PACKET_SIZE);

	// We create a navdata_t union and fill it
	static union navdata_t data;
	data.demo.tag = TAG_DEMO;
	data.demo.vbat_flying_percentage = 100;
	data.demo.altitude = alt;
	data.demo.theta = 0;
	data.demo.phi = 0;
	data.demo.psi = 0;
	data.demo.vx = 0;
	data.demo.vy = 0;
	data.demo.vz = 0;

	// And then we copy its content into the buffer
	memcpy(buffer, &data, PACKET_SIZE);
}


/*!
 * \brief Function called when a message is published on X node
 */
/*void chatterCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {

	ROS_INFO("GET LOCATION !") ;
	ROS_INFO("latitude : [%f]", msg->latitude) ;
	ROS_INFO("longitude : [%f]", msg->longitude) ;
	ROS_INFO("altitude : [%f]", msg->altitude) ;

	//Traitement des données recues
}*/


/*!
 * \brief Main function
 *
 * \param argc The number of arguments
 * \param argv The arguments
 */
int main(int argc, char **argv) {

	/* ######################### Initialization ######################### */

	// Check the command syntax
	if (argc != 2) {
		ROS_FATAL("Command syntax is:\n \trosrun pikopter pikopter_navdata \"ip_address\"\n");
		return -1;
	}

	// Create a pikopter navdata object
	PikopterNavdata *pn = new PikopterNavdata(argv[1]);

	// Initialize ros for this node
	ros::init(argc, argv, "pikopter_navdata");

	// Create a node handle (fully initialize ros)
	ros::NodeHandle navdata_node_handle;

	// Put the rate for this node
	ros::Rate loop_rate(NAVDATA_LOOP_RATE);

	// Debug message
	ROS_DEBUG("Ros initialized with a rate of %u", NAVDATA_LOOP_RATE);

	// Get the length of the socket
	socklen_t len = sizeof(pn->addr_drone_navdata);

	// Here we receive the navdatas from pikopter_mavlink
	//**ros::Subscriber sub = navdata_node_handle.subscribe("mavros/global_position/global", 10, chatterCallback);

	/**
	 * ros::spin() will enter a loop, pumping callbacks.  With this version, all
	 * callbacks will be called from within this thread (the main one).  ros::spin()
	 * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	 */
	//ros::spin();

	// Here we'll spin and send navdatas periodically
	while(ros::ok()) {

		// We fill the navdata buffer here
		fillNavDataBuffer((char*)pn->navdata_buffer, 100);

		// And then we send it
		if (sendto(pn->navdata_fd, pn->navdata_buffer, PACKET_SIZE, 0, (struct sockaddr*)&pn->addr_drone_navdata, sizeof(pn->addr_drone_navdata)) < 0) {
			perror("Send of navdata packet didn't work");
		}

		// Pause in loop with the given value defined in ros::rate
		loop_rate.sleep();
	}

	// Destroy the PikopterNavdata object before leaving the program
	delete pn;

	// Return the correct end status
	return 0;
}