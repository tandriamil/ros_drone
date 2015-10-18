#include <stdbool.h>
#include <stdio.h>
#include <pthread.h>

#include "../include/ros/ros_com_master.h"
#include "../include/ros/ros_utils.h"

static bool isInitialized = true;

/*array holding the com channels. Initially empty.*/
static jakopter_com_channel_t* master[NB_CHANNELS] = {NULL};

/*mutex to guard creation and deletion of channels
(things might go wrong if both happen at the same time)*/
static pthread_mutex_t master_mutex = PTHREAD_MUTEX_INITIALIZER;

/*Use this variable for checks. If the number of channels ever becomes
dynamic, that will be less code to modify.*/
static int nb_max_chan = NB_CHANNELS;

/*initialization isn't needed anymore, since the master channel is static.*/
int jakopter_com_init_master(int nb_chan_max)
{
	return 0;
}

int jakopter_com_master_is_init()
{
	return isInitialized;
}

JAKO_EXPORT jakopter_com_channel_t* jakopter_com_add_channel(int id, size_t size)
{
	pthread_mutex_lock(&master_mutex);
	//debug checks
	if (id >= nb_max_chan || id < 0) {
		fprintf(stderr, "[com_channel] Error : can't add channel with out-of-bounds id : %d\n", id);
		return NULL;
	}
	if (master[id] != NULL) {
		fprintf(stderr, "[com_channel] Error : channel of id %d already exists\n", id);
		return NULL;
	}

	jakopter_com_channel_t* new_chan = jakopter_com_create_channel(size);
	master[id] = new_chan;
	pthread_mutex_unlock(&master_mutex);

	return new_chan;
}

JAKO_EXPORT jakopter_com_channel_t* jakopter_com_get_channel(int id)
{
	if (id >= nb_max_chan || id < 0) {
		fprintf(stderr, "[com_channel] Error : out-of-bounds id provided : %d\n", id);
		return NULL;
	}

	return master[id];
}

JAKO_EXPORT int jakopter_com_remove_channel(int id)
{
	pthread_mutex_lock(&master_mutex);
	//get the channel's pointer and free it
	jakopter_com_channel_t* chan = jakopter_com_get_channel(id);
	//debug check
	if (chan == NULL) {
		fprintf(stderr, "[com_channel] Error : couldn't retrieve channel of id %d\n", id);
		return -1;
	}
	jakopter_com_destroy_channel(&chan);
	//set the pointer in the master table to NULL so that we know it's free
	master[id] = NULL;
	pthread_mutex_unlock(&master_mutex);

	return 0;
}

/*We don't need the init/destroy mechanism anymore*/
void jakopter_com_destroy_master()
{

}

