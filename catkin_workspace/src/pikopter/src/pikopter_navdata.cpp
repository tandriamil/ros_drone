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
	if (navdata_fd == ERROR_ENCOUNTERED){
		ROS_FATAL("Fatal error during the opening of the navdata socket\n");
		exit(EXIT_FAILURE);
	}

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
void PikopterNavdata::fillNavdata() {

	// We create a navdata_t union and fill it
	static union navdata_t data;
	data.demo.tag = TAG_DEMO;
	data.demo.vbat_flying_percentage = 100;
	data.demo.altitude = 100;
	data.demo.theta = 0;
	data.demo.phi = 0;
	data.demo.psi = 0;
	data.demo.vx = 0;
	data.demo.vy = 0;
	data.demo.vz = 0;

	/* ##### Enter Critical Section ##### */
	navdata_mutex.lock();

	// Clean the buffer than put the new values into it
	memset(navdata_buffer, 0, PACKET_SIZE);
	memcpy(navdata_buffer, &data, PACKET_SIZE);

	/* ##### Exit Critical Section ##### */
	navdata_mutex.unlock();
}


/*!
 * \brief Send the navdata
 */
void PikopterNavdata::sendNavdata() {

	/* ##### Enter Critical Section ##### */
	navdata_mutex.lock();

	// Try to send the navdata
	ssize_t sent_size = sendto(navdata_fd, navdata_buffer, PACKET_SIZE, 0, (struct sockaddr*)&addr_drone_navdata, sizeof(addr_drone_navdata));

	/* ##### Exit Critical Section ##### */
	navdata_mutex.unlock();

	// Display error if there's one
	if (sent_size < 0) {
		perror("Send of navdata packet didn't work");
		ROS_ERROR("Send of navdata packet didn't work");
	}

}


/*!
 * \brief Function called when a message is published on X node
 */
void PikopterNavdata::getAltitude(const sensor_msgs::NavSatFix::ConstPtr& msg) 
{
	static union navdata_t data ;

	ROS_INFO("GET altitude !") ;

/* ##### Enter Critical Section ##### */
	navdata_mutex.lock();

	data.demo.altitude = msg->altitude;

	printf("data.demo.tag : %d\n",data.demo.tag) ;
	printf("data.demo.vbat_flying_percentage : %d\n",data.demo.vbat_flying_percentage) ;
	printf("data.demo.altitude : %d\n",data.demo.altitude) ;
	printf("data.demo.theta : %f\n",data.demo.theta) ;
	printf("data.demo.phi : %f\n", data.demo.phi) ;
	printf("data.demo.psi : %f\n", data.demo.psi) ;
	printf("data.demo.vx : %f\n", data.demo.vx) ;
	printf("data.demo.vy : %f\n", data.demo.vy) ;
	printf("data.demo.vz : %f\n", data.demo.vz) ;

/* ##### Exit Critical Section ##### */
	navdata_mutex.unlock();
}


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
		return ERROR_ENCOUNTERED;
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

	// Here we receive the navdatas from pikopter_mavlink
	ros::Subscriber sub = navdata_node_handle.subscribe("mavros/global_position/rel_alt", 10,&PikopterNavdata::getAltitude, pn);

	/**
	 * ros::spin() will enter a loop, pumping callbacks.  With this version, all
	 * callbacks will be called from within this thread (the main one).  ros::spin()
	 * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	 */
	//ros::spin();

	// Here we'll spin and send navdatas periodically
	while(ros::ok()) {

		// We fill the navdata buffer here
			//pn->fillNavdata();

		// And then we send it
		pn->sendNavdata();

		// Pause in loop with the given value defined in ros::rate
		loop_rate.sleep();
	}

	// Destroy the PikopterNavdata object before leaving the program
	delete pn;

	// Return the correct end status
	return NO_ERROR_ENCOUNTERED;
}
