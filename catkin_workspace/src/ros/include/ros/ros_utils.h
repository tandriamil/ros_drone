#ifndef ROS_UTILS_H
#define ROS_UTILS_H

/* C-style public-like keyword */
#if defined(__GNUC__) && __GNUC__ >= 4
#define JAKO_EXPORT __attribute__ ((visibility("default")))
#else
#define JAKO_EXPORT
#endif

/* #undef JAKOPTER_VERSION_MAJOR */
#define JAKOPTER_VERSION_MINOR 2
/* #undef JAKOPTER_VERSION_PATCH */

#define CMAKE_INSTALL_DATADIR "share"

//TODO: switch to the font installed on the system if available
#define FONT_PATH "/Users/bodin/JakopterXtended/resources/FreeSans.ttf"

/* Max number of digit into an integer plus the sign and \0. */
#define INT_LEN     13
/* 2 integers plus the dot and colon*/
#define TSTAMP_LEN  2*INT_LEN+2
/* Max number of digit we use in a float (nb digits of an int + 6 decimal digits) and \0*/
#define FLOAT_LEN INT_LEN+6

/* OS X does not have clock_gettime, use clock_get_time*/
#ifdef __APPLE__
#include <sys/time.h>
#include <sys/resource.h>
#include <mach/mach.h>
#include <mach/clock.h>
#include <mach/mach_time.h>

typedef enum {
	CLOCK_REALTIME,
	CLOCK_MONOTONIC,
	CLOCK_PROCESS_CPUTIME_ID,
	CLOCK_THREAD_CPUTIME_ID
} clockid_t;

int clock_gettime(clockid_t clk_id, struct timespec *tp);
#endif

#endif
