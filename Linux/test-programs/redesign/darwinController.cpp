// darwinController.cpp - contains a library of useful functions
// that allow the user to control Darwin-OP

#include <cstdlib>

#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>
#include <stdint.h>

#include <inttypes.h> //for debugging: 

#include "darwinController.h"


#define MAXNUM_TXPARAM  (256)
#define MAXNUM_RXPARAM  (1024)
#define ID              (2)
#define LENGTH          (3)
#define INSTRUCTION     (4)
#define ERRBIT          (4)
#define PARAMETER       (5)


BulkReadData::BulkReadData() :
        start_address(0),
        length(0),
        error(-1)
{
    for(int i = 0; i < 49; i++) // MAXNUM ADDRESS = 49
        table[i] = 0;
}

BulkReadData::~BulkReadData(){}

int BulkReadData::ReadByte(int address){
    if(address >= start_address && address < (start_address + length))
        return (int)table[address];

    return 0;
}

int BulkReadData::ReadWord(int address){
    if(address >= start_address && address < (start_address + length))
        return MakeWord(table[address], table[address+1]);

    return 0;
}

int BulkReadData::MakeWord(int lowbyte, int highbyte){
    unsigned short word;

    word = highbyte;
    word = word << 8;
    word = word + lowbyte;

    return (int)word;
}

//Timing class:

Timing::Timing(){}
Timing::~Timing(){}

// Return: current time in miliseconds
double Timing::getCurrentTime(){
    gettimeofday(&tv, NULL);

    // I think this returns miliseconds 
    return ((double)tv.tv_sec*1000.0 + (double)tv.tv_usec/1000.0);
}

// Returns time passed in miliseconds
double Timing::TimePassed(double StartTime){
    double time;
    time = getCurrentTime() - StartTime;
    if(time < 0.0){
        printf("error\n");
    }

    return time;
}


bool Timing::isTimeOut(double packetStartTime, double packetWaitTime){
    double time;
    time = getCurrentTime() - packetStartTime;
    if(time < 0.0){
        printf("error\n");
        //need to set packet start time to getCurrentTime();
    }

    if(time > packetWaitTime){
        return true;
    }

    return false;
}

// calculates the wake up time and stores it in LoopTime
void Timing::IncrementTime(struct timespec *LoopTime, double PeriodSec){
    LoopTime->tv_nsec += PeriodSec * 1e9;

    if(LoopTime->tv_nsec > 1000000000L){
        LoopTime->tv_sec += LoopTime->tv_nsec / 1000000000L;
        LoopTime->tv_nsec = LoopTime->tv_nsec % 1000000000L;
    }
}

// Return: False if woken up before wake time
bool Timing::LoopTimeControl(struct timespec *LoopTime){
    
    if(clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, LoopTime, NULL) == 0){
        return true;
    } else {
        return false;
    }
}


DarwinController::DarwinController(){

     // create bulk read data structure
    for(int i = 0; i < 254; i++){
        BulkData[i] = BulkReadData();
    }

    //initialize JointData struct:

    for(int i = 0; i<NUM_JOINTS+1; i++){
        JointData& ji = joints[i];
        ji.flags = 0;
        ji.goal = -9999; //TODO: SET TO SOMETHING SAFER
        ji.p = 0x05;     //TODO SET TO SOMETHING SAFER????
        ji.i = 0;
        ji.d = 0;
    }

    uint8_t enables[20] = {0, }; // disable all motors at first
    uint8_t pgains[20] = {0, };
    uint8_t igains[20] = {0, };
    uint8_t dgains[20] = {0, };

    for(int i = 0; i < 20; i++){ //default Robotis dgain is 32
        pgains[i] = 32; 
    }


    Set_Enables(enables);
    Set_P_Data(pgains);
    Set_I_Data(igains);
    Set_D_Data(dgains);

}

DarwinController::~DarwinController(){}

bool DarwinController::PowerDXL(){
    unsigned char dxltxpacket[] = {0xFF, 0xFF, ID_CM, 0x04, WRITE, DXL_POWER, 0x01, 0};
    dxltxpacket[7] = CalculateChecksum(dxltxpacket);
    int result = port.WritePort(dxltxpacket, 8); // Robotis uses writebyte
    //robotis also has a sleep for 300msec.

    usleep(500000);

    if(result != 0){
        return true;
    } else {
        return false;
    }
}

bool DarwinController::Initialize(const char* name){

    if(port.OpenPort(name) == false){
        fprintf(stderr, "\n Fail to open port\n");
        fprintf(stderr, " CM-730 is used by another program or does not have root privileges.\n\n");
        return false;
    }

    if(PowerDXL() == false){ //failed to power up Dynamixel
        fprintf(stderr, "Fail to change Dynamixel power\n");
        return false;
    } else {
        printf("Dynamixel powered up\n");
        return true;
    }

    // create bulk read data structure
    for(int i = 0; i < 254; i++){
        BulkData[i] = BulkReadData();
    }
}

void DarwinController::ClosePort(){
    port.ClosePort();
}


//InitToPose - gently moves Darwin into a ready position
bool DarwinController::InitToPose(){

    unsigned char initpacket[108] = {0, };
    unsigned char initparams[100] = {0, };

    // Red eyes - Initialization in progress
    int color = MakeColor(255, 0, 0);
    unsigned char colorparams[2] = {GetLowByte(color), GetHighByte(color)}; 

    MakePacket(initpacket, 0xC8, 2, 0x03, 0x1C, colorparams);
    port.ClearPort();
    port.WritePort(initpacket, 9);

    // Torque enable
    for(int IDnum = 0; IDnum < 21; IDnum++){
       initparams[2*IDnum] = (unsigned char)(IDnum+1); // +1 because motors start at 1
       initparams[2*IDnum+1] = 0x01;
    }

    unsigned char paramlength = 1;
    unsigned char initnumparams = 40;
    
    if(SyncWrite(initpacket, 0x18, initparams, initnumparams, paramlength) != 48){
        printf("Failed Torque Enable in InitToPose!\n");
        return false;
    }

    usleep(2000);

    // Starting position and speed
    for(int z = 0; z < 20; z++){
        initparams[5*z] = z+1;
        initparams[5*z+1] = 0x00;
        initparams[5*z+2] = 0x08; // "zero" neutral position is 2048
        initparams[5*z+3] = 0x40;
        initparams[5*z+4] = 0x00;
    }
    if(SyncWrite(initpacket, 0x1E, initparams, 100, 4) != 108){
        printf("Failed to init to pose\n");
        return false;
    }

    sleep(1);

    // Green eyes - Finished initializing
    color = MakeColor(0, 255, 0); 
    initpacket[6] = GetLowByte(color);
    initpacket[7] = GetHighByte(color);

    colorparams[0] = GetLowByte(color);
    colorparams[1] = GetHighByte(color);
    MakePacket(initpacket, 0xC8, 2, 0x03, 0x1C, colorparams);

    port.ClearPort();
    port.WritePort(initpacket, 9);
  
    return true;
}


void DarwinController::MakeBulkPacket(unsigned char *BulkReadTxPacket){

    int number = 0;

    BulkReadTxPacket[0] = 0xFF;
    BulkReadTxPacket[1] = 0xFF;
    BulkReadTxPacket[ID] = 0xFE;
    BulkReadTxPacket[INSTRUCTION] = 0x92;
    BulkReadTxPacket[PARAMETER] = 0x0;
    
    if(Ping(0xC8, 0)){
        BulkReadTxPacket[PARAMETER+3*number+1] = 30; //length
        BulkReadTxPacket[PARAMETER+3*number+2] = 0xC8; // ID_CM
        BulkReadTxPacket[PARAMETER+3*number+3] = 24; //P_DXL_POWER
        number++;
    }

    if(Ping(70, 0)){
        BulkReadTxPacket[PARAMETER+3*number+1] = 10;    // length
        BulkReadTxPacket[PARAMETER+3*number+2] = 0x70;  // ID_L_FSR
        BulkReadTxPacket[PARAMETER+3*number+3] = 0x1A;  // start address P_FSR1_L
        number++;
    }

    if(Ping(0x6F, 0)){
        BulkReadTxPacket[PARAMETER+3*number+1] = 10;     // length
        BulkReadTxPacket[PARAMETER+3*number+2] = 0x6F;   // id ID_R_FSR
        BulkReadTxPacket[PARAMETER+3*number+3] = 0x1A;   // start address P_FSR1_L
        number++;
    }
    
    for(int id = 1; id < 20; id++){ //NUMBER OF JOINTS = 20
       
       BulkReadTxPacket[PARAMETER+3*number+1] = 23;  // length
       BulkReadTxPacket[PARAMETER+3*number+2] = id; // id
       BulkReadTxPacket[PARAMETER+3*number+3] = 26;  // start at CCW_COMPLIANCE_MARGIN
       number++;
        
    }

    BulkReadTxPacket[LENGTH]          = (number * 3) + 3;  

    int length = BulkReadTxPacket[LENGTH] + 4;
    BulkReadTxPacket[length - 1] = CalculateChecksum(BulkReadTxPacket);

}



/*********************************************************
 * Turns everything you need for a packet into a packet. *
 * Does not cover bulk read or sync write.               *
 *********************************************************/
void DarwinController::MakePacket(unsigned char* packet, unsigned char motor_ID, unsigned char parambytes, unsigned char instruction, unsigned char address, unsigned char* params){

    unsigned char len = parambytes + 6; // Last index of array (where checksum goes)--so not truly the length
    packet[0] = 0xFF;                   // Heading
    packet[1] = 0xFF;
    packet[ID] = motor_ID;                     // Motors are 1-20, CM730 is something else
    packet[LENGTH] = len-3;                  // Length of packet except for headings, ID, Checksum
    packet[INSTRUCTION] = instruction;            
    packet[PARAMETER] = address;                // Where to carry out instruction

    for(unsigned char i = 0; i < parambytes; i++)  // Puts in all the data
        packet[6+i] = params[i];

    packet[len] = CalculateChecksum(packet);
}

void DarwinController::FinishPacket(unsigned char *txpacket){
    txpacket[0] = 0xFF;                  
    txpacket[1] = 0xFF;

    int length = txpacket[LENGTH] + 4;
    txpacket[length - 1] = CalculateChecksum(txpacket);
}


int DarwinController::ReadWrite(unsigned char *txpacket, unsigned char *rxpacket){

    int buf = 30;
    int length = 0;
    int count = 0;
    unsigned char info[MAXNUM_RXPARAM] = {0, };

    length = txpacket[LENGTH] + 4;
    
    port.ClearPort();

    int to_length = 0;
    int num = 0; 

    if(port.WritePort(txpacket, length) == length){ //write to port
        //printf("in readwrite length: %d\n", length);

        if(txpacket[INSTRUCTION] == SYNC_WRITE){
            return length;
        }

        if(txpacket[INSTRUCTION] == BULK_READ){ //set bulk read vars
            num = (txpacket[LENGTH]-3) / 3;
    
            for(int i = 0; i < num; i++){

                int _id = txpacket[PARAMETER+(3*i)+2];
                int _len = txpacket[PARAMETER+(3*i)+1];
                int _addr = txpacket[PARAMETER+(3*i)+3];

                to_length += _len + 6;
                BulkData[_id].length = _len;
                BulkData[_id].start_address = _addr;
            }

        } else if(txpacket[INSTRUCTION] == READ){

            to_length = txpacket[PARAMETER+1] + 6; 

        } else {

            to_length = 6;
            
        }

        int get_length = 0;
        int fail_counter = 0; // fail counter for ping
        int length = 0;
        
        //printf("getlength: %d, tolength: %d\n", get_length, to_length);

        // set packet time out:
        double packetStartTime = Time.getCurrentTime();
        double packetWaitTime = 0.012 * (double)length + 5.0;

        while(1){ // loop for receiving packet
            if(fail_counter >= 5){
                if(txpacket[INSTRUCTION] == PING){ 
                    //failed reading 5 times. return.
                    printf("failed ping\n");
                }
                return 0; // culprit. things go wrong if this returns -1. 
            }

            length = port.ReadPort(&rxpacket[get_length], to_length - get_length);

            get_length += length;

            if(get_length == to_length){ //received a packet of correct length
                if(txpacket[INSTRUCTION] == BULK_READ){
                    break;
                } else if(rxpacket[0] == 0xFF && rxpacket[1] == 0xFF){
                    //check checksum of incoming packet
                    unsigned char checksum = CalculateChecksum(rxpacket);
                    if(rxpacket[get_length -1] == checksum){
                        if(txpacket[INSTRUCTION] == PING){
                            printf("Successful ping\n");
                        }
                        //successful read / ping
                        return length;
                    }
                }
                // didn't get packet, write out to port again
                get_length = 0;
                port.ClearPort();
                port.WritePort(txpacket, length);
            } else { //check time out status
                if(Time.isTimeOut(packetStartTime, packetWaitTime)){
                    if(get_length == 0){
                        //printf("timed out\n");
                    } else {
                        //printf("rxpacket corrupt\n");
                    }
                    if(txpacket[INSTRUCTION] != BULK_READ){
                        fail_counter++;
                        get_length = 0;
                        packetStartTime = Time.getCurrentTime();
                        port.ClearPort();
                        port.WritePort(txpacket, length);
                    }
                }       
            }
        }

        if(txpacket[INSTRUCTION] != BULK_READ){
            // if it isn't bulkread, must return here
            return length;
        } 

        for(int i = 0; i < num; i++){
            int _id = txpacket[PARAMETER+(3*i)+2];
            BulkData[_id].error = -1;
        }

        int return_length = get_length;

        while(1){ // this loop is purely for bulkread
            int i;

            for(i = 0; i< get_length - 1; i++){
                if(rxpacket[i] == 0xFF && rxpacket[i+1] == 0xFF){
                    break;
                } else if(i == (get_length - 2) && rxpacket[get_length - 1] == 0xFF){
                    break;
                }
            }

          if(i == 0){ //header is at beginning of packet
                // Check checksum
                unsigned char checksum = CalculateChecksum(rxpacket);

	//	        printf("rxpacket[ID]: %d\n", rxpacket[ID]); // for debugging
                if(rxpacket[LENGTH + rxpacket[LENGTH]] == checksum){
                    for(int j = 0; j < (rxpacket[LENGTH]-2); j++){
                        BulkData[rxpacket[ID]].table[BulkData[rxpacket[ID]].start_address + j] = rxpacket[PARAMETER + j];
 //                       printf("j: %d, rxpacket: %d\n", j, rxpacket[PARAMETER + j]); // for debugging
                        info[count++] = rxpacket[PARAMETER + j];
                    }

                    BulkData[rxpacket[ID]].error = (int)rxpacket[ERRBIT];

                    int cur_packet_length = LENGTH + 1 + rxpacket[LENGTH];

                    to_length = get_length - cur_packet_length;

                    for(int j = 0; j <= to_length; j++){
                        rxpacket[j] = rxpacket[j+cur_packet_length];
                    }
                    get_length = to_length;
                    num--;

                } else {
                    //printf("rx corrupt\n");
                    for(int j = 0; j <= to_length - 2; j++){
                        rxpacket[j] = rxpacket[j+2];
                    }
                    to_length = get_length -= 2;
                } 

               
                if(num == 0){
                    break;
                } else if(get_length <= 6) {
                    if(num != 0){
                        // printf("rx corrupt\n");
                    }
                    break;
                }
            } else {
                for(int j = 0; j < (get_length - i); j++){
                    rxpacket[j] = rxpacket[j+i];
                }
                get_length -= i;
            }
	  
        }
 
        // Sort into structs here
        for(int i = 0; i<20; i++){
            ReadData& rd = jointRead[i];
            rd.d = info[buf];
            rd.i = info[buf+1];
            rd.p = info[buf+2];

            rd.goal_pos = MakeWord(info[buf+4], info[buf+5]);
            rd.max_speed = MakeWord(info[buf+6], info[buf+7]);
            rd.torque_limit = MakeWord(info[buf+8], info[buf+9]);
            rd.cur_pos = MakeWord(info[buf+10], info[buf+11]);
            rd.cur_speed = MakeWord(info[buf+12], info[buf+13]);
            rd.load = MakeWord(info[buf+14], info[buf+15]);

            rd.registered = info[buf+18];
            rd.moving = info[buf+20];

            // 23 uchars per motor
            buf = buf + 23;
        }

        return return_length;
    }

    return length;
}


/******************************************************
 * Takes in an instruction and list of params         *
 * Params needs to be in the form:                    *
 *    {ID_1, data_1, ID_2, data_2, ...}               *
 * There can be any amount of motors and in any order *
 * Note that paramlength does not include the ID      *
 ******************************************************/
int DarwinController::SyncWrite(unsigned char* packet, unsigned char instruction, unsigned char* params, unsigned char numparams, unsigned char paramlength){
   
    unsigned char len = numparams + 7; //Last index of array (where checksum goes) 
    packet[0] = 0xFF;    // Heading
    packet[1] = 0xFF;
    packet[ID] = 0xFE;    // Broadcast ID
    packet[LENGTH] = len-3;   // Length of packet except for headings, ID, Checksum
    packet[INSTRUCTION] = 0x83;    // Sync Write
    packet[5] = instruction;  // What is happening at each motor
    packet[6] = paramlength;  // Number of bytes written to each motor
    for(unsigned char i = 0; i < numparams; i++){
        packet[7+i] = params[i];
    }
    packet[len] = CalculateChecksum(packet);

    unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0, };
    
    //printf("In syncwrite len: %d\n", len);

    return ReadWrite(packet, rxpacket);
    //return port.WritePort(packet, len+1); /alt method

}

int DarwinController::BulkRead(unsigned char *rxpacket){

    return ReadWrite(BulkReadTxPacket, rxpacket);
}

// return value???
int DarwinController::WriteByte(int id, int address, int value){
    unsigned char txpacket[MAXNUM_TXPARAM + 10] = {0, };

    txpacket[ID]           = (unsigned char)id;
    txpacket[INSTRUCTION]  = WRITE;
    txpacket[PARAMETER]    = (unsigned char)address;
    txpacket[PARAMETER+1]  = (unsigned char)value;
    txpacket[LENGTH]       = 4;

    int txlength = txpacket[LENGTH] + 4;
    FinishPacket(txpacket);

    port.ClearPort();
    int result = port.WritePort(txpacket, txlength);

    if(result == txlength){ // do we want to return bool???
        return result;
    } else {
        return 0;
    }

    // robotis also sets an error bit -> do we want to do that?
}

// return value???
// where value will be split into high and low bytes in function
int DarwinController::WriteWord(int id, int address, int value){
    unsigned char txpacket[MAXNUM_TXPARAM + 10] = {0, };

    txpacket[ID]           = (unsigned char)id;
    txpacket[INSTRUCTION]  = WRITE;
    txpacket[PARAMETER]    = (unsigned char)address;
    txpacket[PARAMETER+1]  = (unsigned char)GetLowByte(value);
    txpacket[PARAMETER+2]  = (unsigned char)GetHighByte(value);
    txpacket[LENGTH]       = 5;

    int txlength = txpacket[LENGTH] + 4;
    FinishPacket(txpacket);

    port.ClearPort();
    int result = port.WritePort(txpacket, txlength);

    if(result == txlength){ // do we want to return bool???
        return result;
    } else {
        return 0;
    }

    // do we want to listen to rx packet?
}


int DarwinController::ReadByte(int id, int address, int *word){
    unsigned char txpacket[MAXNUM_TXPARAM + 10] = {0, };
    unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0, };

    txpacket[ID]           = (unsigned char)id;
    txpacket[INSTRUCTION]  = READ;
    txpacket[PARAMETER]    = (unsigned char)address;
    txpacket[PARAMETER+1]  = 1;
    txpacket[LENGTH]       = 4;

    int txlength = txpacket[LENGTH] + 4;

    //makePacket(txpacket);

    int result = ReadWrite(txpacket, rxpacket);

    if(result == txlength){ // how to do this???
        *word = (int)rxpacket[PARAMETER];
    }

    return result;
}

int DarwinController::ReadWord(int id, int address, int *word){
    unsigned char txpacket[MAXNUM_TXPARAM + 10] = {0, };
    unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0, };

    txpacket[ID]           = (unsigned char)id;
    txpacket[INSTRUCTION]  = READ;
    txpacket[PARAMETER]    = (unsigned char)address;
    txpacket[PARAMETER+1]  = 2;
    txpacket[LENGTH]       = 4;

    int txlength = txpacket[LENGTH] + 4;

    //makePacket(txpacket);

    int result = ReadWrite(txpacket, rxpacket);

    if(result == txlength){
        *word = MakeWord((int)rxpacket[PARAMETER], (int)rxpacket[PARAMETER + 1]);

    }

    return result;
}


bool DarwinController::Ping(int id, int *error){
    unsigned char txpacket[MAXNUM_TXPARAM + 10] = {0, };
    unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0, };


    txpacket[ID]           = (unsigned char)id;
    txpacket[INSTRUCTION]  = 1;
    txpacket[LENGTH]       = 2;
    
    // int length = txpacket[LENGTH] + 4;
    // txpacket[length-1] = CalculateChecksum(txpacket);

    FinishPacket(txpacket);

    int result = ReadWrite(txpacket, rxpacket);
    if( result == 0){
        return false;
    } else {
        return true;
    }
}


unsigned char DarwinController::CalculateChecksum(unsigned char *packet)
{
    unsigned char checksum = 0x00;
    for(int i=2; i<packet[LENGTH]+3; i++ )
        checksum += packet[i];
    return (~checksum);
}

int DarwinController::GetLowByte(int word){
    unsigned short temp;
    temp = word & 0xff;
    return (int)temp;
}

int DarwinController::GetHighByte(int word){
    unsigned short temp;
    temp = word & 0xff00;
    return (int)(temp >> 8);
}


int DarwinController::MakeWord(int lowbyte, int highbyte){
    unsigned short word;

    word = highbyte;
    word = word << 8;
    word = word + lowbyte;

    return (int)word;
}

/******************************************************
 * Converts 255 value rgb values into a 2 byte color  *
 * Useful for Head (0x1A) and Eye (0x1C) LEDs         *
 ******************************************************/
int DarwinController::MakeColor(int red, int green, int blue){
    int r = red & 0xFF;
    int g = green & 0xFF;
    int b = blue & 0xFF;

    return (int)(((b>>3)<<10)|((g>>3)<<5)|(r>>3));
}

double DarwinController::Ticks2DegAngle(int ticks){
    if(ticks == 2048){
        return 0.0;
    }

    return (ticks - 2048) / (4096/360);
}

int DarwinController::DegAngle2Ticks(double angle){

    return (int)(2048 + angle * (4096/360));
}

double DarwinController::Ticks2RadAngle(int ticks){
    if(ticks == 2048){
        return 0.0;
    }

    return (ticks - 2048) / (4096 / (2*M_PI));
}

int DarwinController::RadAngle2Ticks(double angle){
    return (int)(2048 + angle * (4096 / (2*M_PI)));
}

// The following are individual motor instructions:

// Converts an angle given in degrees into motor ticks and sends write packet to motor 
int DarwinController::SetJointAngle(unsigned char joint_ID, int goal_angle){
    unsigned char angle_packet[9] = {0, };
    int goal_ticks = goal_angle * 11.378; // 4096/360
    unsigned char params[2] = {GetLowByte(goal_ticks), GetHighByte(goal_ticks)}; 
    MakePacket(angle_packet, joint_ID, 2, WRITE, GOAL_POSITION_L, params);
    unsigned char rxpacket[MAXNUM_RXPARAM] = {0, };
    return ReadWrite(angle_packet, rxpacket);
}

int DarwinController::SetMoveSpeed(unsigned char joint_ID, int move_speed){
    unsigned char speed_packet[9] = {0, };
    unsigned char params[2] = {GetLowByte(move_speed), GetHighByte(move_speed)}; 
    MakePacket(speed_packet, joint_ID, 2, WRITE, GOAL_POSITION_L, params);
    unsigned char rxpacket[MAXNUM_RXPARAM] = {0, };
    return ReadWrite(speed_packet, rxpacket);
}

int DarwinController::Set_P_Gain(unsigned char joint_ID, unsigned char P_Value){
    unsigned char P_packet[8] = {0, };
    MakePacket(P_packet, joint_ID, 1, WRITE, P_GAIN, &P_Value); // Pass in pointer b/c it expects a char*
    unsigned char rxpacket[MAXNUM_RXPARAM] = {0, };
    return ReadWrite(P_packet, rxpacket);
}

int DarwinController::Set_I_Gain(unsigned char joint_ID, unsigned char I_Value){
    unsigned char I_packet[8] = {0, };
    MakePacket(I_packet, joint_ID, 1, WRITE, I_GAIN, &I_Value); // Pass in pointer b/c it expects a char*
    unsigned char rxpacket[MAXNUM_RXPARAM] = {0, };
    return ReadWrite(I_packet, rxpacket);
}

int DarwinController::Set_D_Gain(unsigned char joint_ID, unsigned char D_Value){
    unsigned char D_packet[8] = {0, };
    MakePacket(D_packet, joint_ID, 1, WRITE, D_GAIN, &D_Value); // Pass in pointer b/c it expects a char*
    unsigned char rxpacket[MAXNUM_RXPARAM] = {0, };
    return ReadWrite(D_packet, rxpacket);
}

int DarwinController::Set_PID_Gain(unsigned char joint_ID, unsigned char P_Value, unsigned char I_Value, unsigned char D_Value){
    unsigned char PID_packet[10] = {0, };
    unsigned char params[3] = {P_Value, I_Value, D_Value};
    MakePacket(PID_packet, joint_ID, 1, WRITE, D_GAIN, params);

    unsigned char rxpacket[MAXNUM_RXPARAM] = {0, };
    return ReadWrite(PID_packet, rxpacket);
}

// is_enabled is a 0 for no torque or 1 for torque
int DarwinController::Set_Torque_Enable(unsigned char joint_ID, unsigned char is_enabled){
    unsigned char Torque_packet[8] = {0, };
    MakePacket(Torque_packet, joint_ID, 1, WRITE, TORQUE_ENABLE, &is_enabled);
    unsigned char rxpacket[MAXNUM_RXPARAM] = {0, };
    return ReadWrite(Torque_packet, rxpacket);
}

// The following is for syncwriting a set of conditions:

// TODO: Figure out range of reasonable speeds!
bool DarwinController::SetAllJointSpeeds(int speed){

    unsigned char speedTxPacket[MAXNUM_TXPARAM];
    unsigned char speedParams[60]; // 3 params each for 20 motors

    unsigned char lowbyte = GetLowByte(speed);
    unsigned char highbyte = GetHighByte(speed);

    for(int i = 0; i < NUM_JOINTS; i++){
        speedParams[3*i] = i+1;
        speedParams[3*i+1] = lowbyte;
        speedParams[3*i+2] = highbyte;
    }

    int result = SyncWrite(speedTxPacket, 0x20, speedParams, 60, 2);

    if(result == 68){
        return true;
    } else {
        return false;
    }
}

bool DarwinController::SetAllJointSpeeds(unsigned char highbyte, unsigned char lowbyte){
    unsigned char speedTxPacket[MAXNUM_TXPARAM];
    unsigned char speedParams[60]; // 3 params each for 20 motors

    for(int i = 0; i < NUM_JOINTS; i++){
        speedParams[3*i] = i+1;
        speedParams[3*i+1] = lowbyte;
        speedParams[3*i+2] = highbyte;
    }

    int result = SyncWrite(speedTxPacket, 0x20, speedParams, 60, 2);

    if(result == 68){
        return true;
    } else {
        return false;
    }
}

//for JointData struct

// These set values for all motors and require and input array of 20
// Index 0 of the array corresponds to the value for motor 1
// Index 0 of JointData joints[] corresponds to motor 0 -> not in use.

// TODO: Check for vaid input data array that is 20 long. 

// 0 to disable motor and !0 to enable (or just pass in 1 to enable..)
void DarwinController::Set_Enables(uint8_t* data){
    for(int i=1; i<NUM_JOINTS+1; i++){
        JointData& ji = joints[i];
        if(data[i-1] == 0){
            ji.flags &= 0x7F; // MSB is enable data and the rest are ones so other flags are preserved
        } else{
            ji.flags |= FLAG_ENABLE;
        }
    }
}

int DarwinController::Set_P_Data(uint8_t* data){
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

int DarwinController::Set_I_Data(uint8_t* data){
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

int DarwinController::Set_D_Data(uint8_t* data){
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

int DarwinController::Set_Pos_Data(uint16_t* data){
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

void DarwinController::Set_Enables(unsigned char motor_ID, uint8_t data){
    JointData& ji = joints[motor_ID];
    if(data == 0)
        ji.flags &= 0x7F; // MSB is enable data and the rest are ones so other flags are preserved
    else
        ji.flags |= FLAG_ENABLE;
 
}


// These set individual motor values
void DarwinController::Set_P_Data(unsigned char motor_ID, uint8_t value){
    JointData& ji = joints[motor_ID];
    ji.p = value;
    ji.flags |= FLAG_GAINS_CHANGED;
}


void DarwinController::Set_I_Data(unsigned char motor_ID, uint8_t value){
    JointData& ji = joints[motor_ID];
    ji.i = value;
    ji.flags |= FLAG_GAINS_CHANGED;
}

void DarwinController::Set_D_Data(unsigned char motor_ID, uint8_t value){
    JointData& ji = joints[motor_ID];
    ji.d = value;
    ji.flags |= FLAG_GAINS_CHANGED;
}

int DarwinController::Set_Pos_Data(unsigned char motor_ID, uint16_t value){
    JointData& ji = joints[motor_ID];
    ji.goal = value;
    ji.flags |= FLAG_GOAL_CHANGED;
}

// call this to write the changes in the JointData struct out to port
void DarwinController::Update_Motors(){

    int packet_count = 0;
    uint8_t change_flags = 0;

    // figure out how many packets and what data gets sent
    for (int i=1; i<NUM_JOINTS+1; ++i) {
        const JointData& ji = joints[i];

        // pick off top bit to see if joint enabled
        bool enabled = ji.flags & FLAG_ENABLE; 

        // pick off bottom 7 bits
        uint8_t changed = ji.flags & ~FLAG_ENABLE; 

        // if this motor is enabled and has new data
        if (enabled && changed) { 

            // add in changes from this motor to set of all changes
            change_flags |= changed; 

            // increment number of packets to send
            packet_count += 1; 
        }
    }
    
    // at this point can tell how many bytes per motor to send
    uint8_t buf[120] = {0, };
    int buf_offset = 0;
    unsigned char numparams = 0;
    unsigned char lenparam;
    unsigned char txpacket[128] = {0, };
    unsigned char inst;

    // If any of Goals changed but none of the PIDs changed
    if (change_flags == FLAG_GOAL_CHANGED){  
        lenparam = 2;
        inst = 0x1E; //starting address for goal position Low
        for (int i=0; i<NUM_JOINTS+1; ++i) {   
            JointData& ji = joints[i];

            // If this joint's Goal has changed, add to sync write
            if (ji.flags == FLAG_ENABLE + FLAG_GOAL_CHANGED){    
                buf[buf_offset++] = i;
                buf[buf_offset++] = GetLowByte(ji.goal);
                buf[buf_offset++] = GetHighByte(ji.goal);

                // cleared the changed bits cause we are about to send this
                ji.flags = FLAG_ENABLE; 
                numparams += 3;
            }
        }
    }

     // If any of the gains changed but none of the Goals changed
    else if (change_flags == FLAG_GAINS_CHANGED){
        lenparam = 3;
        inst = 0x1A; // starting address for pid gains
        for (int i=0; i<NUM_JOINTS+1; ++i) {   
            JointData& ji = joints[i];

            // If this joint's gains have changed, add to sync write
            if (ji.flags == FLAG_ENABLE + FLAG_GAINS_CHANGED){    
                buf[buf_offset++] = i;
                buf[buf_offset++] = ji.d;
                buf[buf_offset++] = ji.i;
                buf[buf_offset++] = ji.p;

                // cleared the changed bits cause we are about to send this
                ji.flags = FLAG_ENABLE; 
                numparams += 4;
            }
        }
    }

    // If both PID and Goal have changed
    else if (change_flags == FLAG_GAINS_CHANGED + FLAG_GOAL_CHANGED){ 
        lenparam = 6;
        inst = 0x1A; //starting address for pid gains
        for (int i=0; i<NUM_JOINTS+1; ++i) {   
            JointData& ji = joints[i];

            // If this joint's Goal has changed, add to sync write
            if (ji.flags & FLAG_GOAL_CHANGED || ji.flags & FLAG_GAINS_CHANGED  && ji.flags & FLAG_ENABLE){    
                buf[buf_offset++] = i;
                buf[buf_offset++] = ji.d;
                buf[buf_offset++] = ji.i;
                buf[buf_offset++] = ji.p;
                buf[buf_offset++] = 0x00;  // I hate everything
                buf[buf_offset++] = GetLowByte(ji.goal); // shhh it's ok
                buf[buf_offset++] = GetHighByte(ji.goal);

                // cleared the changed bits cause we are about to send this
                ji.flags = FLAG_ENABLE; 
                numparams += 7;
            }
        }
    }

    if(packet_count){
        SyncWrite(txpacket, inst, buf, numparams, lenparam);
    }
}

//sample usage of joint data structure. take out of darwinController later.

// void DarwinController::foo() {
//     for(int i = 0; i<NUM_JOINTS+1; i++){
//     JointData& ji = joints[i];
//     ji.flags = 0;
//         ji.goal = 2048;
//         ji.p = 0x32;
//         ji.i = 0;
//         ji.d = 0;
//     }

//     uint8_t enables[20] = {0, };
//     uint8_t pgains[20] = {0, };
//     uint8_t igains[20] = {0, };
//     uint8_t dgains[20] = {0, };
//     uint16_t goalpos[20] = {2048, };


//     Set_Enables(enables);
//     int poscount = Set_Pos_Data(goalpos);
// //    Set_P_Data(pgains);
// //    Set_I_Data(igains);
// //    Set_D_Data(dgains);

//     printf("positions: %d\n", poscount);

//     Update_Motors();
// }



