#ifndef ROS_COM_CHANNEL_H
#define ROS_COM_CHANNEL_H

#include "ros_utils.h"

typedef struct jakopter_com_channel_t jakopter_com_channel_t;

jakopter_com_channel_t* jakopter_com_create_channel(size_t size);
void jakopter_com_destroy_channel (jakopter_com_channel_t** cc);
void jakopter_com_write_int(jakopter_com_channel_t* cc, size_t offset, int value);
void jakopter_com_write_float(jakopter_com_channel_t* cc, size_t offset, float value);
void jakopter_com_write_char(jakopter_com_channel_t* cc, size_t offset, char value);
void jakopter_com_write_buf(jakopter_com_channel_t* cc, size_t offset, void* data, size_t size);
int jakopter_com_read_int(jakopter_com_channel_t* cc, size_t offset);
float jakopter_com_read_float(jakopter_com_channel_t* cc, size_t offset);
char jakopter_com_read_char(jakopter_com_channel_t* cc, size_t offset);
void* jakopter_com_read_buf(jakopter_com_channel_t* cc, size_t offset, size_t size, void* dest);
double jakopter_com_get_timestamp(jakopter_com_channel_t* cc);

#endif