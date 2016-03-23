// Include pikopter navdata headers
#include "../include/pikopter/pikopter_navdata.h"


/*!
 * \brief Constructor of PikopterNavdata
 *
 * \param ip_adress The ip adress on which we create the udp socket
 * \param in_demo True if in demo mode, false if not
 */
PikopterNavdata::PikopterNavdata(char *ip_adress, bool in_demo) {

	// Open the UDP port for the navadata node
	navdata_fd = PikopterNetwork::open_udp_socket(PORT_NAVDATA, &addr_drone_navdata, ip_adress);
	if (navdata_fd == ERROR_ENCOUNTERED) {
		ROS_FATAL("Fatal error during the opening of the navdata socket");
		ROS_FATAL("Fatal error code %d", navdata_fd);
		exit(EXIT_FAILURE);
	}

	// Put the mode
	demo_mode = in_demo;

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
 * \brief Get the information about the mode used here
 *
 * \return True if in demo mode, false if not
 */
bool PikopterNavdata::inDemoMode() {

	// Just return the value of the PN
	return demo_mode;
}


/*!
 * \brief Init the navdata buffer
 */
void PikopterNavdata::askMavrosRate() {

	// Check that the service does exist
	if (!ros::service::exists("/mavros/set_stream_rate", true)) {  // Second paramter is whether we print the error or not
		ROS_DEBUG("Can't put the stream rate for navdatas because /mavros/set_stream_rate service is unavailable. Maybe mavros isn't launched yet, we'll wait for it.");
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
	if (ros::service::call("/mavros/set_stream_rate", sr)) ROS_DEBUG("Mavros rate asked");
	else ROS_ERROR("Call on set_stream_rate service failed");

}


/*!
 * \brief Increment the sequence number of the navdata packet
 */
void PikopterNavdata::incrementSequenceNumber() {

	/* ##### Enter Critical Section ##### */
	navdata_mutex.lock();

	// Increment the sequence number
	++navdata_current.demo.sequence;

	/* ##### Exit Critical Section ##### */
	navdata_mutex.unlock();

}


/*!
 * \brief Init the navdata buffer
 */
void PikopterNavdata::initNavdata() {

	// We fill the current navdata
	navdata_current.demo.tag = TAG_DEMO;
	navdata_current.demo.header = DEFAULT_NAVDATA_DEMO_HEADER;  // Not done into pikopter server
	navdata_current.demo.sequence = DEFAULT_NAVDATA_DEMO_SEQUENCE;  // Not done into pikopter server
	navdata_current.demo.size = PACKET_SIZE;  // Not done in the pikopter server
	navdata_current.demo.vbat_flying_percentage = DEFAULT_NAVDATA_DEMO_VBAT_FLYING_PERCENTAGE;
	navdata_current.demo.altitude = DEFAULT_NAVDATA_DEMO_ALTITUDE;
	navdata_current.demo.theta = DEFAULT_NAVDATA_DEMO_THETA;
	navdata_current.demo.phi = DEFAULT_NAVDATA_DEMO_PHI;
	navdata_current.demo.psi = DEFAULT_NAVDATA_DEMO_PSI;
	navdata_current.demo.vx = DEFAULT_NAVDATA_DEMO_VX;
	navdata_current.demo.vy = DEFAULT_NAVDATA_DEMO_VY;
	navdata_current.demo.vz = DEFAULT_NAVDATA_DEMO_VZ;
	navdata_current.demo.vision_defined = DEFAULT_NAVDATA_DEMO_VISION;
	navdata_current.demo.ctrl_state = DEFAULT_NAVDATA_DEMO_CTRL_STATE;
	// navdata_current.demo.ardrone_state; //Not done in the pikopter

	ROS_DEBUG("Navdata demo datas initialized to default values");

	display();

}


/*!
 * \brief Send the navdata
 */
void PikopterNavdata::sendNavdata() {

	// Temporary buffer to send the navdata
	unsigned char tmp_buff[PACKET_SIZE];


	/* ##### Enter Critical Section ##### */
	navdata_mutex.lock();

	// Copy the content of the navdata buffer
	memcpy(tmp_buff, (void *)&navdata_current, PACKET_SIZE);

	/* ##### Exit Critical Section ##### */
	navdata_mutex.unlock();

	// Try to send the navdata
	ssize_t sent_size = sendto(navdata_fd, tmp_buff, PACKET_SIZE, 0, (struct sockaddr*)&addr_drone_navdata, sizeof(addr_drone_navdata));


	// Display error if there's one
	if (sent_size < 0) ROS_ERROR("Send of navdata packet didn't work properly");

	// Increment the sequence number
	incrementSequenceNumber();  // Not done into pikopter server

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

	/* ##### Enter Critical Section ##### */
	navdata_mutex.lock();

	// Just get the current sequence number
	int sequence = navdata_current.demo.sequence;

	/* ##### Exit Critical Section ##### */
	navdata_mutex.unlock();

	// Only if the wanted display rate
	if (sequence%NAVDATA_DISPLAY_RATE == 0) {

		ROS_DEBUG("Current state of the Navdata:");

		/* ##### Enter Critical Section ##### */
		navdata_mutex.lock();

		ROS_DEBUG("\n");
		ROS_DEBUG("Navdata n°%d\n", navdata_current.demo.sequence);
		ROS_DEBUG("\t Header : %d\n", navdata_current.demo.header);
		ROS_DEBUG("\t Tag : %d\n", navdata_current.demo.tag);
		ROS_DEBUG("\t Mask : %d\n", navdata_current.demo.ardrone_state);
		ROS_DEBUG("\t Sequence number : %d\n", navdata_current.demo.sequence);
		ROS_DEBUG("\t Battery : %d\n", navdata_current.demo.vbat_flying_percentage);
		ROS_DEBUG("\t Fly state: %x\n", navdata_current.demo.ctrl_state);
		ROS_DEBUG("\t Altitude : %d\n", navdata_current.demo.altitude);
		ROS_DEBUG("\t Theta : %f\n", navdata_current.demo.theta);
		ROS_DEBUG("\t Phi : %f\n", navdata_current.demo.phi);
		ROS_DEBUG("\t Psi : %f\n", navdata_current.demo.psi);
		ROS_DEBUG("\t Vx : %f\n", navdata_current.demo.vx);
		ROS_DEBUG("\t Vy : %f\n", navdata_current.demo.vy);
		ROS_DEBUG("\t Vz : %f\n", navdata_current.demo.vz);
		ROS_DEBUG("\n");

		/* ##### Exit Critical Section ##### */
		navdata_mutex.unlock();	
	}	
}


/*!
 * \brief Put the battery datas into the navdata
 */
void PikopterNavdata::handleBattery(const mavros_msgs::BatteryStatus::ConstPtr& msg) {

	ROS_DEBUG("Entered battery with value=%d", (int)(msg->remaining * BATTERY_PERCENTAGE));
	//ROS_DEBUG("Entered battery with value=%d", (int)(msg->voltage / BATTERY_PERCENTAGE));

	/* ##### Enter Critical Section ##### */
	navdata_mutex.lock();

	// Put the correct battery status then
	navdata_current.demo.vbat_flying_percentage = (uint32_t)(msg->remaining * BATTERY_PERCENTAGE);
	//navdata_current.demo.vbat_flying_percentage = (uint32_t)(msg->voltage / BATTERY_PERCENTAGE);

	/* ##### Exit Critical Section ##### */
	navdata_mutex.unlock();

}


/*!
 * \brief Get the state of the drone
 */
void PikopterNavdata::getExtendedState(const mavros_msgs::ExtendedState::ConstPtr& msg) {

	// Check if we got strange states
	if ((msg->vtol_state > 0) && (msg->landed_state > 0))
		ROS_WARN("Strange state where the drone is considered as flying and landing at the same time. vtol_state = %d and landed_state = %d", msg->vtol_state, msg->landed_state);
	else if ((msg->vtol_state == 0) && (msg->landed_state == 0))
		ROS_WARN("Strange state where the drone is considered as not flying nor landing.");

	/* ##### Enter Critical Section ##### */
	navdata_mutex.lock();
 
 	//Pour l'etat en vol
	switch(msg->vtol_state)
	{
		navdata_current.demo.ctrl_state = FLY ;
		
		//Etat inconnue 
		case mavros_msgs::ExtendedState::VTOL_STATE_UNDEFINED :
		{	
			navdata_current.demo.ctrl_state = DEFAULT ;
			
			ROS_DEBUG("VTOL_STATE_UNDEFINED") ;
		}
		//Etat transition en avant
		case mavros_msgs::ExtendedState::VTOL_STATE_TRANSITION_TO_FW :
		{
			ROS_DEBUG("VTOL_STATE_TRANSITION_TO_FW") ;
		}

		//Etat de transition vers MC
		case mavros_msgs::ExtendedState::VTOL_STATE_TRANSITION_TO_MC :
		{
			ROS_DEBUG("VTOL_STATE_TRANSITION_TO_MC") ;
		}

		//Etat en MC??
		case mavros_msgs::ExtendedState::VTOL_STATE_MC :
		{
			ROS_DEBUG("VTOL_STATE_MC") ;
		}

		//Etat en avant
		case mavros_msgs::ExtendedState::VTOL_STATE_FW :
		{
			navdata_current.demo.ctrl_state = MOVE ;		
			
			ROS_DEBUG("VTOL_STATE_FW") ;
		}
	}

	// If landed
	switch(msg->landed_state)
	{
		navdata_current.demo.ctrl_state = LAND ;

		//Etat inconnue
		case mavros_msgs::ExtendedState::LANDED_STATE_UNDEFINED :
		{
			navdata_current.demo.ctrl_state = DEFAULT ;
			ROS_DEBUG("LANDED_STATE_UNDEFINED") ;
		}
		
		case mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND :
		{
			//etat a terre (TAKEOFF_GROUND)

			navdata_current.demo.ctrl_state = TAKEOFF ;
			
			ROS_DEBUG("LANDED_STATE_ON_GROUND") ;
		}
		
		case mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR :
		{
			//...se pose ou décolage (TAKEOFF_AUTO)

			navdata_current.demo.ctrl_state = TAKEOFF ;
			
			ROS_DEBUG("LANDED_STATE_IN_AIR") ;
		}
	}
/* ##### Exit Critical Section ##### */
	navdata_mutex.unlock();
}


/*!
 * \brief Put the velocity datas into the navdata
 *
 * \remark The values got from this function are estimated by the drone.
 *         It could be better to get those values from local_position topics
 *         which fetch those datas form GPS position
 */
void PikopterNavdata::handleVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg) {

	ROS_DEBUG("Entered velocity with (x = %f, y = %f, z = %f)", msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.y);

	/* ##### Enter Critical Section ##### */
	navdata_mutex.lock();

	// Updatas velocity datas
	navdata_current.demo.vx = (float32_t)msg->twist.linear.x;
	navdata_current.demo.vy = (float32_t)msg->twist.linear.y;
	navdata_current.demo.vz = (float32_t)msg->twist.linear.z;

	/* ##### Exit Critical Section ##### */
	navdata_mutex.unlock();

}


/*!
 * \brief Put the imu position datas into the navdata
 */
void PikopterNavdata::handleOrientation(const geometry_msgs::PoseStamped::ConstPtr& msg) {

	ROS_DEBUG("Entered orientation with (x = %f, y = %f, z = %f, w = %f)", msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.y, msg->pose.orientation.w);

	// Get the quaternion values
	tf2::Quaternion quaternion (msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);

	// Cast it as a 3x3 matrix
	tf2::Matrix3x3 matrix (quaternion);

	// Then get the euler values converted from this matrix
	double roll, pitch, yaw;
	matrix.getEulerYPR(yaw, pitch, roll);

	/* ##### Enter Critical Section ##### */
	navdata_mutex.lock();

	// Updatas velocity datas
	navdata_current.demo.theta = (float32_t)pitch;
	navdata_current.demo.phi = (float32_t)roll;
	navdata_current.demo.psi = (float32_t)yaw;

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

	// Initialize ros for this node
	ros::init(argc, argv, "pikopter_navdata");

	// Create a node handles (fully initialize ros)
	ros::NodeHandle navdata_node_handle;
	ros::NodeHandle navdata_private_node_handle("~");

	// Here, get the IP address
	std::string ip;
	if (!navdata_private_node_handle.getParam("ip", ip)) {
		ROS_FATAL("Navdata is missing its ip address");
		return ERROR_ENCOUNTERED;
	}

	// Create the table and store the ip into it
	char cstr[ip.length() + 1];
	strcpy(cstr, ip.c_str());

	// Create a pikopter navdata object
	// TODO: Later, we would be abble to choose between normal or demo mode
	PikopterNavdata *pn = new PikopterNavdata(cstr, true);

	// Get the rate for this node in function of the mode
	int rate = (pn->inDemoMode()) ? NAVDATA_DEMO_LOOP_RATE : NAVDATA_LOOP_RATE;

	// Put this rate
	ros::Rate loop_rate(rate);
	ROS_DEBUG("Navdata node initialized with a rate of %u", rate);


	/* ##### All the subscribers to receive datas ##### */
	// Here we receive the navdatas from pikopter_mavlink
	ros::Subscriber sub_mavros_global_position_rel_alt = navdata_node_handle.subscribe("mavros/global_position/rel_alt", SUB_BUF_SIZE_GLOBAL_POS_REL_ALT, &PikopterNavdata::getAltitude, pn);

	// Here we receive the battery state
	ros::Subscriber sub_mavros_battery = navdata_node_handle.subscribe("mavros/battery", SUB_BUF_SIZE_BATTERY, &PikopterNavdata::handleBattery, pn);

	// Here we receive the velocity
	ros::Subscriber sub_mavros_local_position_gp_vel = navdata_node_handle.subscribe("mavros/local_position/velocity", SUB_BUF_SIZE_LOCAL_POS_GP_VEL, &PikopterNavdata::handleVelocity, pn);

	// Here we receive the imu position
	ros::Subscriber sub_mavros_local_position_pose = navdata_node_handle.subscribe("mavros/local_position/pose", SUB_BUF_SIZE_LOCAL_POS_POSE, &PikopterNavdata::handleOrientation, pn);

	//Here we receive state of drone
	ros::Subscriber sub_mavros_extended_state = navdata_node_handle.subscribe("mavros/extended_state", SUB_BUF_SIZE_EXTENDED_STATE, &PikopterNavdata::getExtendedState, pn);

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

	ROS_DEBUG("Exited the ros::ok() loop of navdata node. Goodbye!");

	// Destroy the PikopterNavdata object before leaving the program
	delete pn;

	// Return the correct end status
	return NO_ERROR_ENCOUNTERED;
}
