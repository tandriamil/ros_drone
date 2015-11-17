// Include pikopter navdata headers
#include "../include/pikopter/pikopter_mavlink.h"
#include "../include/pikopter/pikopter_common.h"

/*!
 * \brief Callback function called when a new message arrive on pikopter_cmd topic
 *
 * \param msg the message to display when callback is called
 */
void handleMessageFromCmd(const std_msgs::String::ConstPtr& msg) {
	ROS_INFO("GOT MESSAGE FROM CMD: [%s]", msg->data.c_str());
}

/*!
 * \brief Main function
 *
 * \param argc The number of arguments
 * \param argv The arguments
 */
int main(int argc, char **argv) {
	// TODO: The whole code of pikopter_mavlink
	// - Receive navdatas from mavlink
	// - Send it back to pikopter_navdata (CF subscriber/publisher)
	// - Receive commands from pikopter_cmd
	// - Send it to mavlink
	
	// Initialize ros for this node
	ros::init(argc, argv, "pikopter_mavlink");
	
	// Create a node handle (fully initialize ros)
	ros::NodeHandle nodeHandle;
	
	// Create a subcriber which listen on pikopter_cmd topic
	ros::Subscriber mavlink_sub = nodeHandle.subscribe<std_msgs::String>("pikopter_cmd", 1000, handleMessageFromCmd);

	// Loop which calling message callbacks as fast as possible
	ros::spin();
	
	// Create a publisher and advertise any nodes listening on pikopter_navdata topic that we are going to publish
	ros::Publisher mavlink_pub = nodeHandle.advertise<std_msgs::String>("pikopter_navdata", 1000);

	// Define how fast the program which loop
	ros::Rate loop_rate(10);

	int count = 0;

	while (ros::ok()) {
		// Example message publishing 
		// TODO modify it with the good things
		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
		ROS_INFO("%s", msg.data.c_str());
		// End of example message

		mavlink_pub.publish(msg);

		// To be sure that callbacks will be called (in the case where we have subscriber and publisher in the same node)
		ros::spinOnce();
		
		// Sleep during the time defined in loop_rate
		loop_rate.sleep();
		
		++count; // For the example
	}

	return 0;
}