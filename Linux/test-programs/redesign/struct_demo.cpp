#include <stdint.h>

struct JointData {
	uint8_t  flags;
	uint16_t goal;
	uint8_t p, i, d;
};

enum {
	NUM_JOINTS = 20,
	FLAG_GOAL_CHANGED = 0x01,
	FLAG_GAINS_CHANGED = 0x02,
	FLAG_ENABLE = 0x80,
};


// if just goal changed, write 2 bytes
// if just gains chamnged write 3 bytes
// if both changed write all 5 bytes
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
	unsigned char numparams = 0;
	unsigned char lenparam;
	unsigned char txpacket[128] = {0, };

	// todo: fill in header

    if (change_flags == FLAG_GOAL_CHANGED){  // If any of Goals changed but none of the PIDs changed
    	lenparam = 2;
		for (int i=0; i<NUM_JOINTS; ++i) {   
			JointData& ji = joints[i];
			if (ji.flags == FLAG_ENABLE + FLAG_GOAL_CHANGED){    // If this joint's Goal has changed, add to sync write
				buf[buf_offset++] = i;
				buf[buf_offset++] = GetLowByte(goal);
				buf[buf_offset++] = GetHighByte(goal);
				ji.flags = FLAG_ENABLE; // cleared the changed bits cause we are about to send this
				numparams += 3;
			}
		}
    }

    else if (change_flags == FLAG_GAINS_CHANGED){ // If any of the gains changed but none of the Goals changed
    	lenparam = 3;
		for (int i=0; i<NUM_JOINTS; ++i) {   
			JointData& ji = joints[i];
			if (ji.flags == FLAG_ENABLE + FLAG_GAINS_CHANGED){    // If this joint's gains have changed, add to sync write
				buf[buf_offset++] = i;
				buf[buf_offset++] = ji.d;
				buf[buf_offset++] = ji.i;
				buf[buf_offset++] = ji.p;
				ji.flags = FLAG_ENABLE; // cleared the changed bits cause we are about to send this
				numparams += 4;
			}
		}
    }

    else if (change_flags == FLAG_GAINS_CHANGED + FLAG_GOAL_CHANGED){ // If both PID and Goal have changed
    	lenparam = 5;
		for (int i=0; i<NUM_JOINTS; ++i) {   
			JointData& ji = joints[i];
			if (ji.flags & FLAG_GOAL_CHANGED || ji.flags & FLAG_GAINS_CHANGED  && ji.flags & FLAG_ENABLE){    // If this joint's Goal has changed, add to sync write
				buf[buf_offset++] = i;
				buf[buf_offset++] = ji.d;
				buf[buf_offset++] = ji.i;
				buf[buf_offset++] = ji.p;
				buf[buf_offset++] = GetLowByte(goal);
				buf[buf_offset++] = GetHighByte(goal);
				ji.flags = FLAG_ENABLE; // cleared the changed bits cause we are about to send this
				numparams += 6;
			}
		}
    }

    SyncWrite(txpacket, 0x03, buf, numparams, lenparam);
    port->ClearPort();
    port->WritePort(txpacket, numparams + 8);


	// todo: fill in checksum

	// todo: send buf


}

