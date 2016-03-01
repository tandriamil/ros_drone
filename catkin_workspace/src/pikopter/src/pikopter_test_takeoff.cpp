#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#include <limits.h>
#include <pthread.h>
#include <sys/uio.h>
#include <netinet/in.h>
#include <netdb.h>

/* Import Headers */
#include "../include/pikopter/pikopter_common.h"
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void waitForService(const std::string service) {
	bool mavros_available = ros::service::waitForService(service, MAVROS_WAIT_TIMEOUT);
	if (!mavros_available) {
		ROS_FATAL("Mavros not launched, timeout of %dms reached, exiting...", MAVROS_WAIT_TIMEOUT);
		ROS_INFO("Maybe the service you asked does not exist");
		exit(ERROR_ENCOUNTERED);
	}
}

/*!
 * \brief Launcher of Ros node cmd
 *
 * \param argc Number of parameters
 * \param argv The arguments
 *
 */
int main(int argc, char *argv[]) {
	// Initialize ros for this node
	ros::init(argc, argv, "pikopter_test_takeoff");
	
	// Struct for handling timeout
	struct timeval tv;
	
	// set a timeout
	tv.tv_sec = 0;
  	tv.tv_usec = 100000; // 100ms

	ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient tol_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/takeoff");

	ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 100);	

	mavros_msgs::SetMode srvGuided;
	mavros_msgs::CommandTOL srvTakeOffLand;
	mavros_msgs::CommandBool srvArmed;
	geometry_msgs::TwistStamped msgMove;


	srvGuided.request.custom_mode = "GUIDED";
	srvGuided.request.base_mode = 0;
	srvTakeOffLand.request.altitude = 50;
	srvArmed.request.value = true;

	ROS_INFO("Wait for land service");
	waitForService("/mavros/cmd/land");

	ROS_INFO("Wait for set_mode service");
	waitForService("/mavros/set_mode");
	
	ROS_INFO("Wait for set_mode service");
	waitForService("/mavros/cmd/arming");

	ros::Rate rate(20.0);
	ros::Time last_request = ros::Time::now();
	ros::Time loop_time = ros::Time::now();

	bool takeoff = false;
	bool land = false;

    while(ros::ok()){
        if(current_state.mode != "GUIDED" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(srvGuided) &&
                srvGuided.response.success){
                ROS_INFO("Guided mode enabled");
            }
            last_request = ros::Time::now();
        } else {
            if(!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(srvArmed) &&
                    srvArmed.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if(ros::Time::now() - last_request > ros::Duration(5.0) && !takeoff){
    		if(tol_client.call(srvTakeOffLand) &&
	    		srvTakeOffLand.response.success){
    			takeoff = true;
	    		sleep(10);
	    		ROS_INFO("Vehicle has takeoff");
	    	}
	    	last_request = ros::Time::now();
		}

        ros::spinOnce();
        rate.sleep();
    }

    return 0;

	// sleep(5);

	// // Set mode to GUIDED
	// if(ros::service::call("/mavros/set_mode", srvGuided) && srvGuided.response.success) {
	// 	ROS_INFO("Mode MAV_MODE_GUIDED activated.");
	// }

	// else {
	// 	ROS_ERROR("Problem occured on calling set_mode service");
	// 	exit(ERROR_ENCOUNTERED);
	// }

	// // Arming motors
	// if(ros::service::call("/mavros/cmd/arming", srvArmed) && srvArmed.response.success) {
	// 	ROS_INFO("Arming command called.");
	// }

	// else {
	// 	ROS_ERROR("Problem occured on calling arming command");
	// 	exit(ERROR_ENCOUNTERED);
	// }

	// // Calling takeoff service
	// if(ros::service::call("/mavros/cmd/takeoff", srvTakeOffLand) && srvTakeOffLand.response.success) {
	// 	ROS_INFO("Takeoff called");
	// }

	// else {
	// 	ROS_ERROR("Problem occured on calling takeoff command");
	// 	exit(ERROR_ENCOUNTERED);
	// }

	// sleep(5);
	
	// msgMove.header.stamp = ros::Time::now();
	// msgMove.twist.linear.x = 10;

	// msgMove.twist.angular.x = 100;
	// msgMove.twist.angular.y = 20;
	// msgMove.twist.angular.z = 30;

	// velocity_pub.publish(msgMove);

	// ROS_INFO("Linear velocity -> x = %f, y = %f, z = %f", msgMove.twist.linear.x, msgMove.twist.linear.y, msgMove.twist.linear.z);
	// ROS_INFO("Angular velocity -> x = %f, y = %f, z = %f", msgMove.twist.angular.x, msgMove.twist.angular.y, msgMove.twist.angular.z);

	// // Drone will move during 15 seconds
	// sleep(15);

	// msgMove.header.stamp = ros::Time::now();
	// msgMove.twist.linear.x = 0;
	// msgMove.twist.angular.z = 0;

	// velocity_pub.publish(msgMove);
	
	// ROS_INFO("Linear velocity -> x = %f, y = %f, z = %f", msgMove.twist.linear.x, msgMove.twist.linear.y, msgMove.twist.linear.z);
	// ROS_INFO("Angular velocity -> x = %f, y = %f, z = %f", msgMove.twist.angular.x, msgMove.twist.angular.y, msgMove.twist.angular.z);

	// sleep(5);

	// if(ros::service::call("/mavros/cmd/land", srvTakeOffLand))
	// 	ROS_INFO("Land called");
	// else
	// 	ROS_ERROR("Problem occured on calling land command");
	// return 0;
}