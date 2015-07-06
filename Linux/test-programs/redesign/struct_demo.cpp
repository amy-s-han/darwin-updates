#include <stdint.h>

struct JointData {
	uint8_t  flags;
	uint16_t goal;
	uint16_t p, i, d;
};

enum {
	NUM_JOINTS = 20,
	FLAG_GOAL_CHANGED = 0x01,
	FLAG_P_CHANGED = 0x02,
	FLAG_I_CHANGED = 0x04,
	FLAG_D_CHANGED = 0x08,
	FLAG_ENABLE = 0x80,
};

void foo() {
	
	// declare an array of joint data
	JointData joints[NUM_JOINTS];

	// motor 3 is enabled, has new goal position
	joints[3].goal = 1027;
	joints[3].flags = FLAG_ENABLE | FLAG_GOAL_CHANGED;


	// later:
	int packet_count = 0;
	uint8_t change_flags = 0;

	// figure out how many packets and what data gets sent
	for (int i=0; i<NUM_JOINTS; ++i) {
		const JointData& ji = joints[i];
		bool enabled = ji.flags & FLAG_ENABLE; // pick off top bit to see if joint enabled
		uint8_t changed = ji.flags & ~FLAG_ENABLE; // pick off bottom 7 bits
		if (enabled && changed) { // if this motor is enabled and has new data
			change_flags |= changed; // add in changes from this motor to set of all changes
			packet_count += 1; // increment number of packets to send
		}
	}

	// at this point can tell how many bytes per motor to send

	uint8_t buf[MAX_PACKET_LENGTH*MAX_NUM_PACKETS + HEADER_LEN + FOOTER_LEN];
	int buf_offset = 0;

	// todo: fill in header

	for (int i=0; i<NUM_JOINTS; ++i) {
		// TODO fill in buf with packets
		const JointData& ji = joints[i];
		bool enabled = ji.flags & FLAG_ENABLE; // pick off top bit to see if joint enabled
		uint8_t changed = ji.flags & ~FLAG_ENABLE; // pick off bottom 7 bits
		if (enabled && changed) { // if this motor is enabled and has new data
		}
	}

	// todo: fill in checksum

	// todo: send buf


}

