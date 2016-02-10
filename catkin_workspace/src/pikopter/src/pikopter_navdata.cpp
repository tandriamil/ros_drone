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

	// Initialise the navdata datas
	initNavdata();

	// Ask mavros the rate on which it wants to receive the datas
	askMavrosRate();  // Will wait mavros to be launched before continuing the execution

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
void PikopterNavdata::askMavrosRate() {

	// Check that the service does exist
	if (!ros::service::exists("/mavros/set_stream_rate", true)) {  // Second parameter is whether we print the error or not
		ROS_INFO("Can't put the stream rate for navdatas because /mavros/set_stream_rate service is unavailable. Maybe mavros isn't launched yet, we'll wait for it.");
	}

	// We'll wait for it then
	bool mavros_available = ros::service::waitForService("/mavros/set_stream_rate", MAVROS_WAIT_TIMEOUT);
	if (!mavros_available) {
		ROS_FATAL("Mavros not launched, timeout of %dms reached, exiting...", MAVROS_WAIT_TIMEOUT);
		delete this;
		exit(ERROR_ENCOUNTERED);
	}

	// Create a StreamRate service handler to call the request
	mavros_msgs::StreamRate sr;

	// TODO: Find the correct options to ask only what we need for the moment
	sr.request.stream_id = mavros_msgs::StreamRateRequest::STREAM_ALL;
	sr.request.message_rate = (uint16_t)5;
	sr.request.on_off = (uint8_t)1;

	// Call the service
	if (ros::service::call("/mavros/set_stream_rate", sr)) ROS_INFO("Mavros rate asked");
	else ROS_ERROR("Call on set_stream_rate service failed");

}


/*!
 * \brief Init the navdata buffer
 */
void PikopterNavdata::initNavdata() {

	// We fill the current navdata
	navdata_current.demo.tag = TAG_DEMO;
	navdata_current.demo.vbat_flying_percentage = DEFAULT_NAVDATA_DEMO_VBAT_FLYING_PERCENTAGE;
	navdata_current.demo.altitude = DEFAULT_NAVDATA_DEMO_ALTITUDE;
	navdata_current.demo.theta = DEFAULT_NAVDATA_DEMO_THETA;
	navdata_current.demo.phi = DEFAULT_NAVDATA_DEMO_PHI;
	navdata_current.demo.psi = DEFAULT_NAVDATA_DEMO_PSI;
	navdata_current.demo.vx = DEFAULT_NAVDATA_DEMO_VX;
	navdata_current.demo.vy = DEFAULT_NAVDATA_DEMO_VY;
	navdata_current.demo.vz = DEFAULT_NAVDATA_DEMO_VZ;

	ROS_INFO("Navdata demo datas initialized to default values");

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
	if (sent_size < 0) ROS_ERROR("Send of navdata packet didn't work properly");

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

	ROS_DEBUG("\tdata.demo.tag : %d", navdata_current.demo.tag);
	ROS_DEBUG("\tdata.demo.vbat_flying_percentage : %d", navdata_current.demo.vbat_flying_percentage);
	ROS_DEBUG("\tdata.demo.altitude : %d", navdata_current.demo.altitude);
	ROS_DEBUG("\tdata.demo.theta : %f", navdata_current.demo.theta);
	ROS_DEBUG("\tdata.demo.phi : %f", navdata_current.demo.phi);
	ROS_DEBUG("\tdata.demo.psi : %f", navdata_current.demo.psi);
	ROS_DEBUG("\tdata.demo.vx : %f", navdata_current.demo.vx);
	ROS_DEBUG("\tdata.demo.vy : %f", navdata_current.demo.vy);
	ROS_DEBUG("\tdata.demo.vz : %f", navdata_current.demo.vz);

	/* ##### Exit Critical Section ##### */
	navdata_mutex.unlock();
}


/*!
 * \brief Put the battery datas into the navdata
 */
void PikopterNavdata::handleBattery(const mavros_msgs::BatteryStatus::ConstPtr& msg) {

	//ROS_DEBUG("Entered battery with value=%d", (int)(msg->remaining * BATTERY_PERCENTAGE));
	ROS_DEBUG("Entered battery with value=%d", (int)(msg->voltage / BATTERY_PERCENTAGE));

	/* ##### Enter Critical Section ##### */
	navdata_mutex.lock();

	// Put the correct battery status then
	//navdata_current.demo.vbat_flying_percentage = (uint32_t)(msg->remaining * BATTERY_PERCENTAGE);
	navdata_current.demo.vbat_flying_percentage = (uint32_t)(msg->voltage / BATTERY_PERCENTAGE);

	/* ##### Exit Critical Section ##### */
	navdata_mutex.unlock();

}


void PikopterNavdata::getState(const mavros_msgs::ExtendedState::ConstPtr& msg)
{
/* ##### Enter Critical Section ##### */
	navdata_mutex.lock();

	switch(msg->vtol_state)
	{
		//navdata_current.demo.ctrl_states = FLY ;
		case mavros_msgs::ExtendedState::VTOL_STATE_UNDEFINED :
		{
			//Etat inconnue
			ROS_DEBUG("VTOL_STATE_UNDEFINED") ;
		}

		case mavros_msgs::ExtendedState::VTOL_STATE_TRANSITION_TO_FW :
		{
			//Etat transition en avant
			ROS_DEBUG("VTOL_STATE_TRANSITION_TO_FW") ;
		}

		case mavros_msgs::ExtendedState::VTOL_STATE_TRANSITION_TO_MC :
		{
			ROS_DEBUG("VTOL_STATE_TRANSITION_TO_MC") ;
		}

		case mavros_msgs::ExtendedState::VTOL_STATE_MC :
		{
			ROS_DEBUG("VTOL_STATE_MC") ;
		}

		case mavros_msgs::ExtendedState::VTOL_STATE_FW :
		{
			//Etat en avant
			ROS_DEBUG("VTOL_STATE_FW") ;
		}
	}

	switch(msg->landed_state)
	{
		//navdata_current.demo.ctrl_states = LAND ;
		case mavros_msgs::ExtendedState::LANDED_STATE_UNDEFINED :
		{
			//Etat inconnue
			ROS_DEBUG("LANDED_STATE_UNDEFINED") ;
		}
		case mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND :
		{
			//etat a terre
			ROS_DEBUG("LANDED_STATE_ON_GROUND") ;
		}
		case mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR :
		{
			//...se pose ou décolage
			ROS_DEBUG("LANDED_STATE_IN_AIR") ;
		}
	}
/* ##### Exit Critical Section ##### */
	navdata_mutex.unlock();

}


/*!
 * \brief Put the velocity datas into the navdata
 */
void PikopterNavdata::handleVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg) {

	ROS_DEBUG("Entered velocity with (x = %f, y = %f, z = %f)", msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.y);

	/* ##### Enter Critical Section ##### */
	navdata_mutex.lock();

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
	// if (argc != 2) {
	// 	ROS_FATAL("Command syntax is: \trosrun pikopter pikopter_navdata \"ip_address\"");
	// 	return ERROR_ENCOUNTERED;
	// }

	// Initialize ros for this node
	ros::init(argc, argv, "pikopter_navdata");

	// Create a node handle (fully initialize ros)
	ros::NodeHandle navdata_node_handle;

	std::string ip;


	navdata_node_handle.getParam("/pikopter_navdata/ip", ip);

	ROS_INFO("IP non convertie : %s" , ip.c_str());

	char* cstr = new char[ip.length() + 1];
	strcpy(cstr, ip.c_str());

	ROS_INFO("IP convertie : %s" , cstr);

	// Create a pikopter navdata object
	PikopterNavdata *pn = new PikopterNavdata(cstr);

	delete [] cstr;

	// Put the rate for this node
	ros::Rate loop_rate(NAVDATA_LOOP_RATE);

	// Debug message
	ROS_INFO("Navdata node initialized with a rate of %u", NAVDATA_LOOP_RATE);


	/* ##### All the subscribers to receive datas ##### */
	// Here we receive the navdatas from pikopter_mavlink
	ros::Subscriber sub_mavros_global_position_rel_alt = navdata_node_handle.subscribe("mavros/global_position/rel_alt", SUB_BUF_SIZE_GLOBAL_POS_REL_ALT, &PikopterNavdata::getAltitude, pn);

	// Here we receive the battery state
	ros::Subscriber sub_mavros_battery = navdata_node_handle.subscribe("mavros/battery", SUB_BUF_SIZE_BATTERY, &PikopterNavdata::handleBattery, pn);

	// Here we receive the velocity
	ros::Subscriber sub_mavros_global_position_gp_vel = navdata_node_handle.subscribe("mavros/local_position/velocity", SUB_BUF_SIZE_GLOBAL_POS_GP_VEL, &PikopterNavdata::handleVelocity, pn);

	//Here we receive state of drone
	ros::Subscriber sub_mavros_extended_state = navdata_node_handle.subscribe("mavros/extended_state", SUB_BUF_SIZE_EXTENDED_STATE, &PikopterNavdata::getState, pn);

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

	ROS_INFO("Exited the ros::ok() loop of navdata node. Goodbye!");

	// Destroy the PikopterNavdata object before leaving the program
	delete pn;

	// Return the correct end status
	return NO_ERROR_ENCOUNTERED;
}