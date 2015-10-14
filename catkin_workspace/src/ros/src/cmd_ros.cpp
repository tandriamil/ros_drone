#include <stdio.h>
#include <stdlib.h>
#include <curses.h>
#include <fcntl.h>
#include <unistd.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

#define CMDFILENAME "/tmp/jakopter_user_input.sock"

int main(){
	char c;
	FILE *cmd;
	initscr();
	
	while (ros::ok()) {
		c = getch();

		if (c == '\t') {
			endwin();
			return 0;
		}

		cmd = fopen(CMDFILENAME, "w");
		fprintf(cmd, "%c\n", (int) c);
		fclose(cmd);
	}
	unlink(CMDFILENAME);

	return 0;
}