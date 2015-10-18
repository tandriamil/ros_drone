#ifndef ROS_COM_MASTER_H
#define ROS_COM_MASTER_H
#include "ros_com_channel.h"

enum jakopter_channels {
	CHANNEL_MASTER,
	CHANNEL_NAVDATA,
	CHANNEL_DISPLAY,
	CHANNEL_LEAPMOTION,
	CHANNEL_USERINPUT,
	CHANNEL_COORDS,
	CHANNEL_CALLBACK,
	CHANNEL_NETWORK_INPUT,
	CHANNEL_NETWORK_OUTPUT,
	NB_CHANNELS
};

jakopter_com_channel_t* jakopter_com_add_channel(int id, size_t size);
jakopter_com_channel_t* jakopter_com_get_channel(int id);
int jakopter_com_remove_channel(int id);

#endif
