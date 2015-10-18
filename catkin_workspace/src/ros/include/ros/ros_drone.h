#ifndef ROS_DRONE_H
#define ROS_DRONE_H

#include "ros_common.h"
#include "ros_navdata.h"
#include "ros_com_channel.h"
#include "ros_com_master.h"

#define PACKET_SIZE     256
#define PORT_CMD        5556

/* 30 ms in ns */
#define TIMEOUT_CMD      30000000
#define NAVDATA_ATTEMPT  10
#define HEIGHT_THRESHOLD 500

#define HEAD_LEN    10
#define ARG_LEN     30
#define ARGS_MAX     7

/* Headers of AT* messages */
#define HEAD_REF        "REF"
#define HEAD_PCMD       "PCMD"
#define HEAD_CONFIG     "CONFIG"
#define HEAD_CTRL       "CTRL"
#define HEAD_COM_WATCHDOG "COMWDG"
#define HEAD_FTRIM      "FTRIM"
#define HEAD_CALIB      "CALIB"

enum video_channels {
	VID_HORIZONTAL,
	VID_VERTICAL,
	VID_HORIZON_VERTICAL,
	VID_VERTICAL_HORIZON,
	VID_SWITCH,
	VID_CHANNELS
};

int jakopter_connect(const char* drone_ip);
int jakopter_takeoff();
int jakopter_land();
int jakopter_emergency();
int jakopter_reinit();
int jakopter_disconnect();
int jakopter_rotate_left(float speed);
int jakopter_rotate_right(float speed);
int jakopter_slide_left(float speed);
int jakopter_slide_right(float speed);
int jakopter_forward(float speed);
int jakopter_backward(float speed);
int jakopter_up(float speed);
int jakopter_down(float speed);
int jakopter_stay();
int jakopter_move(float l_to_r, float b_to_f, float vertical_speed, float angular_speed);
int jakopter_switch_camera(unsigned int id);
const char* jakopter_log_command();
int jakopter_flat_trim();
int jakopter_calib();
int init_navdata_bootstrap();
int config_ack();

#endif
