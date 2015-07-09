// write word: txpacket = {0xFF, 0xFF, joint_id, 5, INST_WRITE, start address, value low, value high, checksum}

#include <stdio.h>
#include <unistd.h>
#include <cstdlib>
#include "ports.h"
#include <stdint.h>
#include <math.h>

using namespace std;

#define LENGTH (3)
#define PARAMETER (5)
#define MAXNUM_TXPARAM (256)
#define MAXNUM_RXPARAM (1024)
#define BULK_READ (146)
#define READ (0)
#define WRITE (0)
#define EYES (0)
#define motor_LED (0)
#define REGWRITE (0)
#define total (1000)

struct JointData {
    uint8_t  flags;
    uint16_t goal;
    uint8_t p, i, d;
};

enum {
    FLAG_GOAL_CHANGED = 0x01,
    FLAG_GAINS_CHANGED = 0x02,
    FLAG_ENABLE = 0x80,
    NUM_JOINTS = 20
};


unsigned char CalculateChecksum(unsigned char *packet)
{
    unsigned char checksum = 0x00;
    for(int i=2; i<packet[LENGTH]+3; i++ )
        checksum += packet[i];
    return (~checksum);
}

int MakeColor(int red, int green, int blue)
{
    int r = red & 0xFF;
    int g = green & 0xFF;
    int b = blue & 0xFF;

    return (int)(((b>>3)<<10)|((g>>3)<<5)|(r>>3));
}

int GetLowByte(int word)
{
	unsigned short temp;
    temp = word & 0xff;
    return (int)temp;
}

int GetHighByte(int word)
{
	unsigned short temp;
    temp = word & 0xff00;
    return (int)(temp >> 8);
}

int MakeWord(int lowbyte, int highbyte)
{
    unsigned short word;

    word = highbyte;
    word = word << 8;
    word = word + lowbyte;

    return (int)word;
}


void MakePacket(unsigned char* packet, unsigned char ID, unsigned char parambytes, unsigned char instruction, unsigned char address, unsigned char* params){

    unsigned char len = parambytes + 6; // Last index of array (where checksum goes)--so not truly the length
    packet[0] = 0xFF;                   // Heading
    packet[1] = 0xFF;
    packet[2] = ID;                     // Motors are 1-20, CM730 is something else
    packet[3] = len-3;                  // Length of packet except for headings, ID, Checksum
    packet[4] = instruction;            
    packet[5] = address;                // Where to carry out instruction

    for(unsigned char i = 0; i < parambytes; i++)  // Puts in all the data
        packet[6+i] = params[i];

    packet[len] = CalculateChecksum(packet);
}

//samples data 5 times and returns the last read word
int ReadWord(Port *port, unsigned char joint, unsigned char address){

    unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0, };

    //txpacket = {0xFF, 0xFF, id, 4, INST_READ, start address, 2, checksum};
    unsigned char txpacket[] = {0xFF, 0xFF, joint, 0x04, 0x02, address, 0x02, 0};

    int length = txpacket[LENGTH] + 4;
    
    txpacket[length-1] = CalculateChecksum(txpacket);

    port->ClearPort();

    port->WritePort(txpacket, length);

    int to_length = txpacket[PARAMETER+1] + 6; 

    //printf("length is %d\n", length);
    //printf("to_length = %d\n\n", to_length);

    int get_length = 0;
    int new_length = 0;
    int counter = 0;
    bool go = true;
    int word;
    bool result = false;

    while(go){

        new_length = port->ReadPort(&rxpacket[get_length], to_length - get_length);

        get_length += new_length;

        if(get_length == to_length){ //received a packet of correct length
            if(rxpacket[0] == 0xFF && rxpacket[1] == 0xFF){
                //check checksum of incoming packet
                unsigned char checksum = CalculateChecksum(rxpacket);
                if(rxpacket[get_length -1] == checksum){
                    //printf("Sucessful read\n");
                    result = true;
                    
                    word = MakeWord((int)rxpacket[PARAMETER], (int)rxpacket[PARAMETER + 1]);
                    //printf("Motor position: %d\n\n", word);

                    if(counter >= 0){
                      go = false;
                      break;
                    }
                    counter ++;
                }
            }

        usleep(50000);
        get_length = 0;
        port->ClearPort();
        port->WritePort(txpacket, length);
        
        }
    }

    if(!go && result){
        printf("Sucessful read and return\n");
        return word;
    } else if (!go && !result){
        printf("Failed read, returning bad number\n");
        return -9999999;
    }

}

void SyncWrite(unsigned char* packet, unsigned char instruction, unsigned char* params, unsigned char numparams, unsigned char paramlength){

    unsigned char len = numparams + 8;
    packet[0] = 0xFF;    // Heading
    packet[1] = 0xFF;
    packet[2] = 0xFE;    // Broadcast ID
    packet[3] = len-4;   // Length of packet except for headings, ID, Checksum
    packet[4] = 0x83;    // Sync Write
    packet[5] = instruction; 
    packet[6] = paramlength;  // Hopefully this is right?
    for(unsigned char i = 0; i < numparams; i++)
        packet[7+i] = params[i];
    packet[len-1] = CalculateChecksum(packet);
}


int Init_to_Pose(Port* port){
    int temp = 0;  // To be returned
    unsigned char IDnum;
    unsigned char initpacket[108] = {0, };
    unsigned char initparams[100] = {0, };
    // Torque
    for(IDnum = 0; IDnum < 21; IDnum++){
       initparams[2*IDnum] = IDnum+1;
       initparams[2*IDnum+1] = 0x01;
    }
    unsigned char paramlength = 1;
    unsigned char initnumparams = 40;
    SyncWrite(initpacket, 0x18, initparams, initnumparams, paramlength);
    port->ClearPort();
    port->WritePort(initpacket, 48);
    usleep(2000);

    // Starting position and speed
    for(int z = 0; z < 20; z++){
        initparams[5*z] = z+1;
        initparams[5*z+1] = 0x00;
        initparams[5*z+2] = 0x08;
        initparams[5*z+3] = 0x40;
        initparams[5*z+4] = 0x00;
    }
    SyncWrite(initpacket, 0x1E, initparams, 100, 4);
    usleep(2000);
    port->ClearPort();
    if(port->WritePort(initpacket, 108)==108)
        temp = 1;
        
    // Red means stop
    int color = MakeColor(255, 0, 0);
    unsigned char colorparams[2] = {GetLowByte(color), GetHighByte(color)}; 

    MakePacket(initpacket, 0xC8, 2, 0x03, 0x1C, colorparams);
    port->ClearPort();
    port->WritePort(initpacket, 9);
    sleep(1);
    
    // Green means go
    color = MakeColor(0, 255, 0); 
    initpacket[6] = GetLowByte(color);
    initpacket[7] = GetHighByte(color);
    initpacket[8] = CalculateChecksum(initpacket);
    port->ClearPort();
    port->WritePort(initpacket, 9);
    usleep(2000);   
  
    return temp;
/*
    for(int z = 0; z < 20; z++){
        initparams[3*z] = z+1;
        initparams[3*z+1] = 0x00;
        initparams[3*z+2] = 0x00;
    }
    
    SyncWrite(initpacket, 0x20, initparams, 60, 2);
    port->ClearPort();
    port->WritePort(initpacket, 68);

    for(int z = 0; z < 20; z++){
        initparams[2*z] = z+1;
        initparams[2*z+1] = 0xFF;
    }
    
    SyncWrite(initpacket, 0x1C, initparams, 40, 1);
    port->ClearPort();
    port->WritePort(initpacket, 48);
*/
}  

// These take in a 20 long array
// Indexing is off by one, so the data for the nth joint is in the (n-1)th index
void Set_Enables(JointData* joints, uint8_t* data){
    for(int i=1; i<NUM_JOINTS+1; i++){
	JointData& ji = joints[i];
	if(data[i-1] == 0)
        ji.flags &= 0x7F; // MSB is enable data and the rest are ones so other flags are preserved
	else
	    ji.flags |= FLAG_ENABLE;
    }
}

int Set_P_Data(JointData* joints, uint8_t* data){
    int counter = 0;
    for(int i=1; i<NUM_JOINTS+1; i++){
	JointData& ji = joints[i];
	if((ji.flags & FLAG_ENABLE) && (ji.p != data[i-1])){
	    ji.p = data[i-1];
	    ji.flags |= FLAG_GAINS_CHANGED;
            counter ++;
   	}
    }
    return counter;
}

int Set_I_Data(JointData* joints, uint8_t* data){
    int counter = 0;
    for(int i=1; i<NUM_JOINTS+1; i++){
	JointData& ji = joints[i];
	if((ji.flags & FLAG_ENABLE) && (ji.i != data[i-1])){
	    ji.i = data[i-1];
	    ji.flags |= FLAG_GAINS_CHANGED;
            counter ++;
   	}
    }
    return counter;
}

int Set_D_Data(JointData* joints, uint8_t* data){
    int counter = 0;
    for(int i=1; i<NUM_JOINTS+1; i++){
	JointData& ji = joints[i];
	if((ji.flags & FLAG_ENABLE) && (ji.d != data[i-1])){
	    ji.d = data[i-1];
	    ji.flags |= FLAG_GAINS_CHANGED;
            counter ++;
   	}
    }
    return counter;
}

int Set_Pos_Data(JointData* joints, uint16_t* data){
    int counter = 0;
    for(int i=1; i<NUM_JOINTS+1; i++){
	JointData& ji = joints[i];
	   if((ji.flags & FLAG_ENABLE) && (ji.goal != data[i-1])){
	       ji.goal = data[i-1];
	       ji.flags |= FLAG_GOAL_CHANGED;
           counter ++;
   	    }
    }
    return counter;
}

// These only change data for one joint
// and there isn't any indexing trickery 
void Set_Enables(JointData joint, uint8_t data){
    if(data == 0)
        joint.flags &= 0x7F; // MSB is enable data and the rest are ones so other flags are preserved
    else
        joint.flags |= FLAG_ENABLE;
}

void Set_P_Data(JointData joint, uint8_t data){
    joint.p = data;
    joint.flags |= FLAG_GAINS_CHANGED;
}

void Set_I_Data(JointData joint, uint8_t data){
    joint.i = data;
    joint.flags |= FLAG_GAINS_CHANGED;
}

void Set_D_Data(JointData joint, uint8_t data){
    joint.d = data;
    joint.flags |= FLAG_GAINS_CHANGED;
}

void Set_Pos_Data(JointData joint, uint16_t data){
    joint.goal = data;
    joint.flags |= FLAG_GOAL_CHANGED;
}

void Update_Motors(Port* port, JointData* joints){
    int packet_count = 0;
    uint8_t change_flags = 0;

    // figure out how many packets and what data gets sent
    for (int i=0; i<NUM_JOINTS+1; ++i) {
        const JointData& ji = joints[i];
	bool enabled = ji.flags & FLAG_ENABLE; // pick off top bit to see if joint enabled
	uint8_t changed = ji.flags & ~FLAG_ENABLE; // pick off bottom 7 bits
	   if (enabled && changed) { // if this motor is enabled and has new data
	       change_flags |= changed; // add in changes from this motor to set of all changes
	       packet_count += 1; // increment number of packets to send
	   }
    }

    printf("packet count is: %d\n", packet_count);

    // at this point can tell how many bytes per motor to send

    printf("Got to checkpoint B \n");
    
    uint8_t buf[120] = {0, };
    int buf_offset = 0;
    unsigned char numparams = 0;
    unsigned char lenparam;
    unsigned char txpacket[128] = {0, };
    unsigned char inst;
    // todo: fill in header

    if (change_flags == FLAG_GOAL_CHANGED){  // If any of Goals changed but none of the PIDs changed
       lenparam = 2;

       printf("Got to checkpoint 1 \n");
       inst = 0x1E;
       for (int i=0; i<NUM_JOINTS+1; ++i) {   
	    JointData& ji = joints[i];
	    if (ji.flags == FLAG_ENABLE + FLAG_GOAL_CHANGED){    // If this joint's Goal has changed, add to sync write
                buf[buf_offset++] = i;
	        buf[buf_offset++] = GetLowByte(ji.goal);
	        buf[buf_offset++] = GetHighByte(ji.goal);
	    ji.flags = FLAG_ENABLE; // cleared the changed bits cause we are about to send this
	    numparams += 3;
	    }
        }
    }

    else if (change_flags == FLAG_GAINS_CHANGED){ // If any of the gains changed but none of the Goals changed
    	lenparam = 3;
        inst = 0x1A;
        printf("Got to checkpoint 2 \n");
	for (int i=0; i<NUM_JOINTS+1; ++i) {   
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
    	lenparam = 6;
        inst = 0x1A;
        printf("Got to checkpoint 3 \n");
	for (int i=0; i<NUM_JOINTS+1; ++i) {   
	    JointData& ji = joints[i];
	    if (ji.flags & FLAG_GOAL_CHANGED || ji.flags & FLAG_GAINS_CHANGED  && ji.flags & FLAG_ENABLE){    // If this joint's Goal has changed, add to sync write
		buf[buf_offset++] = i;
		buf[buf_offset++] = ji.d;
		buf[buf_offset++] = ji.i;
		buf[buf_offset++] = ji.p;
		buf[buf_offset++] = 0x00;  // I hate everything
		buf[buf_offset++] = GetLowByte(ji.goal);
		buf[buf_offset++] = GetHighByte(ji.goal);
		ji.flags = FLAG_ENABLE; // cleared the changed bits cause we are about to send this
		numparams += 7;
	    }
	}
    }

    if(packet_count){
        SyncWrite(txpacket, inst, buf, numparams, lenparam);
    
        for(int i=0; i < 15; i++)
            printf("packet[%d] = %d\n", i, txpacket[i]);

        printf("writing: %d\n", numparams + 8);
        port->ClearPort();
        int wut = port->WritePort(txpacket, numparams + 8);
        printf("wrote: %d\n", wut);
        usleep(2000);
    }
}

void foo(Port* port, JointData* joints) {
    for(int i = 0; i<NUM_JOINTS+1; i++){
	JointData& ji = joints[i];
	ji.flags = 0;
        ji.goal = 2048;
        ji.p = 0x32;
        ji.i = 0;
        ji.d = 0;
    }

    uint8_t enables[20] = {0, };
    uint8_t pgains[20] = {0, };
    uint8_t igains[20] = {0, };
    uint8_t dgains[20] = {0, };
    uint16_t goalpos[20] = {2048, };


    Set_Enables(joints, enables);
    int poscount = Set_Pos_Data(joints, goalpos);
//    Set_P_Data(joints, pgains);
//    Set_I_Data(joints, igains);
//    Set_D_Data(joints, dgains);

    printf("positions: %d\n", poscount);

    Update_Motors(port, joints);
}

int main(int argc, char** argv){
    Port* port = new Port("/dev/ttyUSB0");
    if(port->OpenPort() == false){
        fprintf(stderr, "\n Fail to open port\n");
        fprintf(stderr, " CM-730 is used by another program or do not have root privileges.\n\n");
        return 0;
    }

    //dxl power on stuff
    //(WriteByte(CM730::ID_CM, CM730::P_DXL_POWER, 1, 0)

    unsigned char dxltxpacket[] = {0xFF, 0xFF, 0xC8, 0x04, 0x03, 0x18, 0x01, 0};
    dxltxpacket[7] = CalculateChecksum(dxltxpacket);
    port->WritePort(dxltxpacket, 8);
    usleep(2000);

    printf("Finshed dxl power up. Press enter\n");
    getchar();

    Init_to_Pose(port);
    getchar();   
  
    unsigned char packet[100] = {0, };

    int p_word;
    int i_word;
    int d_word;

    unsigned char joint = 20;

    p_word = ReadWord(port, joint, 0x1A);
    i_word = ReadWord(port, joint, 0x1B);
    d_word = ReadWord(port, joint, 0x1C);

    printf("p gain is: %d\n", GetLowByte(p_word));
    printf("i gain is: %d\n", GetLowByte(i_word));
    printf("d gain is: %d\n", GetLowByte(d_word));

    JointData joints[21];
    foo(port, joints);
    
    usleep(5000);
    p_word = ReadWord(port, joint, 0x1A);
    i_word = ReadWord(port, joint, 0x1B);
    d_word = ReadWord(port, joint, 0x1C);

    printf("p gain is: %d\n", GetLowByte(p_word));
    printf("i gain is: %d\n", GetLowByte(i_word));
    printf("d gain is: %d\n", GetLowByte(d_word));

if(EYES){
    int color = MakeColor(230, 0, 230);
    
    // May want to put the cycling through colors back in
    // for a cool demo.

    // LED packet
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0xC8;
    packet[3] = 0x05;
    packet[4] = 0x03;
    packet[5] = 0x1A;
    packet[6] = GetLowByte(color);
    packet[7] = GetHighByte(color);
    packet[8] = CalculateChecksum(packet); 

    port->ClearPort();
    port->WritePort(packet, 9);
    usleep(2000);
   
    packet[5] = 0x1C;
    packet[8] = CalculateChecksum(packet);
    port->ClearPort();
    port->WritePort(packet, 9);
    usleep(2000);

    getchar();

    // 0x01C for eye LEDs, 0x1A for head LED
    unsigned char txpacket[] = {0xFF, 0xFF, 0xC8, 0x05, 0x03, 0x1C, 0xE0, 0x03, 0};
    txpacket[8] = CalculateChecksum(txpacket);
    printf("%d \n", txpacket[8]);

    int value = port->WritePort(txpacket, 9);


    printf("press enter\n");
    getchar();

    double r[total] = {0, };
    double g[total] = {0, };
    double b[total] = {0, };

    int count;

    for(count = 0; count < total; count++){
        r[count] = 255*sin(6.28*count/(total/2));
        g[count] = 255*sin(6.28*count/(total/11));
        b[count] = 255*sin(6.28*count/(total/21));
    }

    count = 0;
    int newcolor;

    while(count < total){
        port->ClearPort();
        newcolor = MakeColor(r[count], g[count], b[count]);
        txpacket[6] = GetLowByte(newcolor);
        txpacket[7] = GetHighByte(newcolor);
        txpacket[8] = 0;
        txpacket[8] = CalculateChecksum(txpacket);
        port->WritePort(txpacket, 9);
        usleep(10000);
        count++;
    } 
}

    //do we want a time out???
    //port->SetPacketTimeout(length) 

if(READ){

    unsigned char jointID = 0x1;
    int word;

  for(jointID = 1; jointID < 21; jointID++){
    word = ReadWord(port, jointID, 0x24);
    printf("Motor %d: %X - %X \n", jointID, GetHighByte(word), GetLowByte(word));
}
    if(word == -9999999){
        printf("bad read, terminating program.\n");
        port->ClosePort();
        return 0;
    }
 //   printf("Read angle as: %d, press enter to write new angle", word);


    // add some amount to the angle that was read in
    // split into low and high bytes
    //   word += 100;
    //   int value_low = GetLowByte(word);
    //   int value_high = GetHighByte(word);

    // now write position

    // txpacket_write = {0xFF, 0xFF, joint_id, 5, INST_WRITE, start address, value low, value high, checksum};
    // 			 {header, header, id, length = 5, INST_WRITE = 3, ...}
    // want start_address to be MX28::P_GOAL_POSITION_L
  
  //  unsigned char txpacket_write[] = {0xFF, 0xFF, 0x05, 0x05, 0x03, 0x1E, GetLowByte(word), GetHighByte(word), 0};
  //  int txlength = txpacket_write[LENGTH] + 4;
    
  //  txpacket_write[8] = CalculateChecksum(txpacket_write);
   
 //   port->ClearPort();
 //   port->WritePort(txpacket_write, 9);
 //   usleep(2000);

}

if(WRITE){
    int angle = 0;
    while(1){

        packet[2] = 0x05;
        packet[5] = 0x1E;
        packet[6] = 0xD0;
        packet[7] = 0x07;
        packet[8] = CalculateChecksum(packet);
        port->ClearPort();
        port->WritePort(packet, 9);
        usleep(2000);

        sleep(1);

        angle = ReadWord(port, 0x6, 0x24);
        printf("Angle is: %d", angle);
        if(getchar() == 'a')
	    break;

        packet[7] = 0x09;
        packet[8] = CalculateChecksum(packet);
        port->ClearPort();
        port->WritePort(packet, 9);
        usleep(2000);

	sleep(1);

        angle = ReadWord(port, 0x06, 0x24);
        printf("Angle is: %d", angle);
        if(getchar() == 'a')
	    break;
    }

}

if(REGWRITE){

    unsigned char regpack[] = {0xFF, 0xFF, 0x05, 0x04, 0x04, 0x19, 0x01, 0x00};
    regpack[7] = CalculateChecksum(regpack);

    unsigned char action[] = {0xFF, 0xFF, 0x05, 0x02, 0x05, 0};
    action[5] = CalculateChecksum(action);

    port->ClearPort();
    port->WritePort(regpack, 8);
    usleep(2000);
   
    getchar();

    port->ClearPort();
    port->WritePort(action, 6);
    usleep(2000);
}

    printf("Press the ENTER key to close port!\n");
    getchar();

    port->ClosePort();

}

