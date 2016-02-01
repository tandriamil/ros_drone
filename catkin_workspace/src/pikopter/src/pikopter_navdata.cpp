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
		ROS_FATAL("Fatal error during the opening of the navdata socket");
		ROS_FATAL("Fatal error code %d", navdata_fd);
		exit(EXIT_FAILURE);
	}

	// Initialise the navdata
	initNavdata();

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
 * \brief Init the navdata buffer
 */
void PikopterNavdata::initNavdata() {

	// We fill the current navdata
	navdata_current.demo.tag = TAG_DEMO;
	navdata_current.demo.vbat_flying_percentage = 100;
	navdata_current.demo.altitude = 100;
	navdata_current.demo.theta = 0;
	navdata_current.demo.phi = 0;
	navdata_current.demo.psi = 0;
	navdata_current.demo.vx = 0;
	navdata_current.demo.vy = 0;
	navdata_current.demo.vz = 0;

}


/*!
 * \brief Send the navdata
 */
void PikopterNavdata::sendNavdata() {

	/* ##### Enter Critical Section ##### */
	navdata_mutex.lock();

	// Try to send the navdata
	ssize_t sent_size = sendto(navdata_fd, (unsigned char *)&navdata_current, PACKET_SIZE, 0, (struct sockaddr*)&addr_drone_navdata, sizeof(addr_drone_navdata));

	/* ##### Exit Critical Section ##### */
	navdata_mutex.unlock();

	// Display error if there's one
	if (sent_size < 0) ROS_ERROR("Send of navdata packet didn't work");

}


/*!
 * \brief Function called when a message is published on X node
 */
void PikopterNavdata::getAltitude(const std_msgs::Float64::ConstPtr& msg)  {

	ROS_DEBUG("Entered altitude with value=%f", (float)msg->data);

	/* ##### Enter Critical Section ##### */
	navdata_mutex.lock();

	navdata_current.demo.altitude = (int32_t)msg->data;

	/* ##### Exit Critical Section ##### */
	navdata_mutex.unlock();
}


/*!
 * \brief Function called when a message is published on X node
 */
void PikopterNavdata::display() {

	ROS_DEBUG("Current state of the Navdata:");

	/* ##### Enter Critical Section ##### */
	navdata_mutex.lock();

	ROS_DEBUG("\tdata.demo.tag : %d",navdata_current.demo.tag);
	ROS_DEBUG("\tdata.demo.vbat_flying_percentage : %d",navdata_current.demo.vbat_flying_percentage);
	ROS_DEBUG("\tdata.demo.altitude : %d",navdata_current.demo.altitude);
	ROS_DEBUG("\tdata.demo.theta : %f",navdata_current.demo.theta);
	ROS_DEBUG("\tdata.demo.phi : %f", navdata_current.demo.phi);
	ROS_DEBUG("\tdata.demo.psi : %f", navdata_current.demo.psi);
	ROS_DEBUG("\tdata.demo.vx : %f", navdata_current.demo.vx);
	ROS_DEBUG("\tdata.demo.vy : %f", navdata_current.demo.vy);
	ROS_DEBUG("\tdata.demo.vz : %f", navdata_current.demo.vz);
	ROS_DEBUG(" ");

	/* ##### Exit Critical Section ##### */
	navdata_mutex.unlock();
}


/*!
 * \brief Put the battery datas into the navdata
 */
void PikopterNavdata::handleBattery(const mavros_msgs::BatteryStatus::ConstPtr& msg) {

	ROS_DEBUG("Entered battery with value=%d", (int)(msg->remaining * 100));

	/* ##### Enter Critical Section ##### */
	navdata_mutex.lock();

	// Put the correct battery status then
	navdata_current.demo.vbat_flying_percentage = (uint32_t)(msg->remaining * 100);

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
		ROS_FATAL("Command syntax is: \trosrun pikopter pikopter_navdata \"ip_address\"");
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
	ROS_INFO("Ros initialized with a rate of %u", NAVDATA_LOOP_RATE);

	// Here we receive the navdatas from pikopter_mavlink
	ros::Subscriber sub_mavros_global_position_rel_alt = navdata_node_handle.subscribe("mavros/global_position/rel_alt", 10, &PikopterNavdata::getAltitude, pn);

	// Here we receive the battery state
	ros::Subscriber sub_mavros_battery = navdata_node_handle.subscribe("mavros/battery", 10, &PikopterNavdata::handleBattery, pn);

	// Here we'll spin and send navdatas periodically
	while(ros::ok()) {

		// Display the state of the navdata (for debug)
		pn->display();

		// And then we send it
		pn->sendNavdata();

		// Spin once
		ros::spinOnce();

		// Pause in loop with the given value defined in ros::rate
		loop_rate.sleep();
	}

	ROS_INFO("Exited the ros::ok() loop. Goodbye!");

	// Destroy the PikopterNavdata object before leaving the program
	delete pn;

	// Return the correct end status
	return NO_ERROR_ENCOUNTERED;
}