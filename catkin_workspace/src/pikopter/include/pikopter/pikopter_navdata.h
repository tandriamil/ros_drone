#ifndef PIKOPTER_NAVDATA_H
#define PIKOPTER_NAVDATA_H


/* ################################### INCLUDES ################################### */
// Pikotper common includes
#include "pikopter_common.h"



/* ################################### CONSTANTS ################################### */
// The port used for the navdatas
#define PORT_NAVDATA 5554

// Interval in seconds
#define NAVDATA_INTERVAL 1/15

// Loop rate in hertz
#define NAVDATA_LOOP_RATE 33  // Basically, every 30*1000 microseconds, so 33 times per seconds

// A tag to say if it's a demo or not
#define TAG_DEMO 0

// Tag for the checksum packet in full mode
#define TAG_CKS 0
#define NAVDATA_NREADS_INT 4
#define NAVDATA_NREADS_FLOAT 6

#define float32_t float
#define float64_t double

// 2 spaces
#define DEMO_LEN NAVDATA_NREADS_INT*(INT_LEN+1)+(NAVDATA_NREADS_FLOAT*FLOAT_LEN+1)+2



/* ################################### TYPE DEF ################################### */

/* ########## State values ########## */
// Main value in the first 16 bits
typedef enum {
	DEFAULT,
	INIT,
	LANDED,
	FLY,
	HOVER,
	CTRL_USELESS_1,
	CTRL_USELESS_2,
	TAKEOFF,
	MOVE,
	LAND,
	LOOP
} ctrl_states;

// Specific value in the last 16 bits
typedef enum {
	FLY_OK,
	FLY_LOST_ALT,
	FLY_LOST_ALT_GO_DOWN,
	FLY_ALT_OUT_ZONE,
	COMBINED_YAW,
	BRAKE,
	NO_VISION
} fly_states;

// Hover states
typedef enum {
	HOVER_OK,
	HOVER_YAW,
	HOVER_YAW_LOST_ALT,
	HOVER_YAW_LOST_ALT_GO_DOWN,
	HOVER_ALT_OUT_ZONE,
	HOVER_YAW_ALT_OUT_ZONE,
	HOVER_LOST_ALT,
	HOVER_LOST_ALT_GO_DOWN,
	LOST_COM,
	LOST_COM_LOST_ALT,
	LOST_COM_LOST_ALT_TOO_LONG,
	LOST_COM_ALT_OK,
	MAGNETO_CALIB,
	DEMO
} hover_states;

// Takeoff states
typedef enum {
	TAKEOFF_GROUND,
	TAKEOFF_AUTO
} takeoff_states;

// Move states
typedef enum {
	GOTO_OK = 0,
	GOTO_LOST_ALT,
	GOTO_YAW
} move_states;

// Land states
typedef enum {
	CLOSED_LOOP,
	OPEN_LOOP,
	OPEN_LOOP_FAST
} land_states;

// Loop states
typedef enum {
	IMPULSION,
	OPEN_LOOP_CTRL,
	PLANIF_CTRL
} loop_states;


/* ########## Navdata structures ########## */
// Some options for the navdata paquets
struct navdata_option {
	uint16_t  tag;  // Type of the packet: DEMO, etc.
	uint16_t  size;
	uint8_t   data[];
};

// The navdata paquet itself
struct navdata {
	uint32_t    header;  // Always 88776655
	uint32_t    ardrone_state;  // Bit mask defined in SDK config.h
	uint32_t    sequence;  // Sequence number of the packet
	bool		vision_defined;  // True: vision computed by ardrone onboard chip
	struct navdata_option options[1];  // Static pointer to generic options
};

// Navdata paquet for demo
struct navdata_demo {
	uint32_t   header;  // Always 88776655
	uint32_t   ardrone_state;  // Bit mask defined in SDK config.h
	uint32_t   sequence;  // Sequence number of the packet
	bool	   vision_defined;  // True: vision computed by ardrone onboard chip

	// Here are the specific values for demo mode
	uint16_t   tag;  // Type of the packet: must be TAG_DEMO
	uint16_t   size;  // Size of the packet in bytes
	uint32_t   ctrl_state;  // Flying state (landed, flying, hovering, etc.)
	uint32_t   vbat_flying_percentage;  // Battery voltage filtered (mV)

	// UAV (Unmanned Aerial Vehicle ~ drone) state
	float32_t  theta;  // UAV's pitch in milli-degrees
	float32_t  phi;  // UAV's roll in milli-degrees
	float32_t  psi;  // UAV's yaw in milli-degrees
	int32_t    altitude;  // UAV's altitude in millimeters
	float32_t  vx;  // UAV's estimated linear velocity in millimeters/s
	float32_t  vy;  // UAV's estimated linear velocity in millimeters/s
	float32_t  vz;  // UAV's estimated linear velocity in millimeters/s

	// Deprecated on ARdrone2.0
	uint32_t   num_frames;
	float32_t  detection_camera_rot[9];
	float32_t  detection_camera_trans[3];
	uint32_t   detection_tag_index;
	uint32_t   detection_camera_type;  // NOT DEPRECATED, Type of tag searched in detection
	float32_t  drone_camera_rot[9];
	float32_t  drone_camera_trans[3];
};

// The navdata structure, contains the basic struc and the demo one
union navdata_t {
	struct navdata raw;
	struct navdata_demo demo;
};


/* ################################### Classes ################################### */
/*!
 * \brief Pikopter navdata ros node
 */
class PikopterNavdata {

	// Public part
	public:
		int main_loop(int argc, char **argv);

	// Private part
	private:
		struct sockaddr_in addr_drone_navdata;
		unsigned char navdata_buffer[PACKET_SIZE];
		int navdata_fd;
};

#endif