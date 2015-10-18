#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include <pthread.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/un.h>

#include "../include/ros/ros_user_input.h"

jakopter_com_channel_t* user_input_channel;

pthread_t user_input_thread;
//FBO static bool recv_ready = false;
/* Guard that stops any function if connection isn't initialized.*/
bool stopped_user_input = true;

struct sockaddr_un addr_user_input;
int sock_user_input;

int read_cmd()
{
	int ret = -1;
	char c;
	FILE *keyboard_cmd = NULL;
	keyboard_cmd = fopen(USERINPUT_FILENAME,"r");
	if (keyboard_cmd) {
		fscanf(keyboard_cmd,"%c",&c);
		ret = (int) c;
		fclose(keyboard_cmd);
	}

	return ret;
}

void* user_input_routine(void* args)
{
	int keyboard_key = 0;
	int param2 = 0;
	int last_key = 0;
	int pparam2 = 0;


	while (!stopped_user_input) {
		keyboard_key = read_cmd();

		if (keyboard_key < 0)
			pthread_exit(NULL);

		param2 = 0; // not used yet

		if ((keyboard_key != last_key) || (pparam2 != param2)) {
			// write only when you have a new value
			jakopter_com_write_int(user_input_channel, 0, (int) keyboard_key);
			jakopter_com_write_int(user_input_channel, 4, (int) param2);
			last_key = keyboard_key;
			pparam2 = param2;
		}

		// wait before doing it again
		usleep(USERINPUT_INTERVAL*1000);
	}
	pthread_exit(NULL);
}

int user_input_connect()
{
	if (!stopped_user_input)
		return -1;

	stopped_user_input = false;

	user_input_channel = jakopter_com_add_channel(CHANNEL_USERINPUT, 2*sizeof(int));
	jakopter_com_write_int(user_input_channel, 0, 0);
	jakopter_com_write_int(user_input_channel, 4, 0);

	if (pthread_create(&user_input_thread, NULL, user_input_routine, NULL) < 0) {
		perror("[~][user_input] Can't create thread");
		return -1;
	}

	int i = 0;

	return -(i >= USERINPUT_TIMEOUT);
}

int user_input_disconnect()
{
	if (!stopped_user_input) {
		stopped_user_input = true;
		int ret = pthread_join(user_input_thread, NULL);

		jakopter_com_remove_channel(CHANNEL_USERINPUT);

		return ret;
	}
	else {
		fprintf(stderr, "[~][user_input] Communication already stopped\n");
		return -1;
	}
}

