#include <stdio.h>
#include <stdlib.h>
#include <ncurses.h>
#include <fcntl.h>
#include <unistd.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

#define CMDFILENAME "/tmp/jakopter_user_input.sock"

void navdataCallback(const std_msgs::String::ConstPtr& msg) {
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv){
	char c;
	FILE *cmd;
	//initscr();

	// This method must be called to use other ROS methods
	ros::init(argc, argv, "cmd");

	// Create a handler for this node
	ros::NodeHandle n;
	
	while (ros::ok()) {
		c = getch();

		if (c == '\t') {
			//endwin();
			return 0;
		}

		cmd = fopen(CMDFILENAME, "w");

		// Subscribes to the navdata topic
		// Returns a Subscriber object
		ros::Subscriber sub = n.subscribe("pikopter", 1000, navdataCallback);

		// Enters a loop, calling message callbacks as fast as possible
		ros::spin();
		
		fprintf(cmd, "%c\n", (int) c);
		fclose(cmd);
	}
	unlink(CMDFILENAME);

	return 0;
}