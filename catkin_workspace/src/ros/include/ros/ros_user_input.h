#ifndef ROS_USERINPUT_H
#define ROS_USERINPUT_H

#include "ros_com_channel.h"
#include "ros_com_master.h"
#include "ros_utils.h"

#define USERINPUT_INTERVAL 	1/3 // interval in seconds
#define USERINPUT_TIMEOUT 2000
#define USERINPUT_FILENAME "/tmp/jakopter_user_input.sock"

int user_input_connect();
int user_input_disconnect();


#endif
