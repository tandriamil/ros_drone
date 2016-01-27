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

/* ##### Enter Critical Section ##### */
	navdata_mutex.lock();

	navdata_current.demo.altitude = msg->altitude;

/* ##### Exit Critical Section ##### */
	navdata_mutex.unlock();
}


/*!
 * \brief Function called when a message is published on X node
 */
void PikopterNavdata::display() 
{

	ROS_INFO("Display navdata:") ;

/* ##### Enter Critical Section ##### */
	navdata_mutex.lock();

	printf("data.demo.tag : %d\n",navdata_current.demo.tag) ;
	printf("data.demo.vbat_flying_percentage : %d\n",navdata_current.demo.vbat_flying_percentage) ;
	printf("data.demo.altitude : %d\n",navdata_current.demo.altitude) ;
	printf("data.demo.theta : %f\n",navdata_current.demo.theta) ;
	printf("data.demo.phi : %f\n", navdata_current.demo.phi) ;
	printf("data.demo.psi : %f\n", navdata_current.demo.psi) ;
	printf("data.demo.vx : %f\n", navdata_current.demo.vx) ;
	printf("data.demo.vy : %f\n", navdata_current.demo.vy) ;
	printf("data.demo.vz : %f\n", navdata_current.demo.vz) ;

/* ##### Exit Critical Section ##### */
	navdata_mutex.unlock();
}


/*!
 * \brief Put the battery datas into the navdata
 */
void PikopterNavdata::handleBattery(const mavros_msgs::BatteryStatus::ConstPtr& msg) {

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
	//ros::Subscriber sub = navdata_node_handle.subscribe("mavros/global_position/rel_alt", 10,&PikopterNavdata::getAltitude, pn);

	// Here we receive the battery state
	ros::Subscriber sys_status_battery = navdata_node_handle.subscribe("mavros/sys_status/battery", 10, &PikopterNavdata::handleBattery, pn);

	// Here we'll spin and send navdatas periodically
	while(ros::ok()) {

		// Display the state of the navdata (for debug)
		pn->display();

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
