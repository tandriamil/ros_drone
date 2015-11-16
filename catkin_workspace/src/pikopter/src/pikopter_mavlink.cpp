// Include pikopter navdata headers
#include "../include/pikopter/pikopter_mavlink.h"
#include "../include/pikopter/pikopter_common.h"

void handleMessage(const std_msgs::String::ConstPtr& msg) {
	ROS_INFO("GOT MESSAGE: [%s]", msg->data.c_str());
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
	ros::NodeHandle nodeHandle;
	ros::Subscriber cmd_pub = nodeHandle.subscribe<std_msgs::String>("pikopter_mavlink", 1000, handleMessage);
	ros::Rate loop_rate(10);
	ros::spin();
	return 0;
}