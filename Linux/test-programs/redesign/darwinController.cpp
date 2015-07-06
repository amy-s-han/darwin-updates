
#include <stdio.h>
#include <sys/time.h>

#include "darwinController.h"


#define MAXNUM_TXPARAM 	(256)
#define MAXNUM_RXPARAM 	(1024)
#define ID		        (2)
#define LENGTH         	(3)
#define INSTRUCTION 	(4)
#define ERRBIT		    (4)
#define PARAMETER	    (5)




BulkReadData::BulkReadData() :
        start_address(0),
        length(0),
        error(-1)
{
    for(int i = 0; i < 49; i++) // MAXNUM ADDRESS = 49
        table[i] = 0;
}

BulkReadData::~BulkReadData(){
    //TODO: FINISH DESTRUCTOR
}

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


DarwinController::DarwinController(){
	// what to do here?
}

DarwinController::~DarwinController(){
	//what to do here?
}

bool DarwinController::PowerDXL(){
    unsigned char dxltxpacket[] = {0xFF, 0xFF, ID_CM, 0x04, WRITE, DXL_POWER, 0x01, 0};
    dxltxpacket[7] = CalculateChecksum(dxltxpacket);
    int result = port->WritePort(dxltxpacket, 8); // Robotis uses writebyte
    //robotis also has a sleep for 300msec.
    if(result != 0){
        return true;
    } else {
        return false;
    }
}

bool DarwinController::Initialize(const char* name){
	port = new Port(name);
	if(port->OpenPort() == false){
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
    port->ClosePort();
}


unsigned char DarwinController::CalculateChecksum(unsigned char *packet)
{
    unsigned char checksum = 0x00;
    for(int i=2; i<packet[LENGTH]+3; i++ )
        checksum += packet[i];
    return (~checksum);
}

double DarwinController::getCurrentTime(){
    struct timeval tv;
    gettimeofday(&tv, NULL);

    return ((double)tv.tv_sec*1000.0 + (double)tv.tv_usec/1000.0);
}

bool DarwinController::isTimeOut(double packetStartTime, double packetWaitTime){
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

//general read/write:
int DarwinController::ReadWrite(unsigned char *txpacket, unsigned char *rxpacket){

    int length = txpacket[LENGTH] + 4;
    port->ClearPort();

    int to_length = 0;
    int num = 0; 

    if(port->WritePort(txpacket, length) == length){ //write to port

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
        
        printf("getlength: %d, tolength: %d\n", get_length, to_length);
        // set packet time out:
        double packetStartTime = getCurrentTime();
        double packetWaitTime = 0.012 * (double)length + 5.0;

        while(1){ // loop for receiving packet
            if(fail_counter >= 5){
                if(txpacket[INSTRUCTION] == PING){ 
                    //failed reading 5 times. return.
                    printf("failed ping\n");
                }
                return 0;
           }

            length = port->ReadPort(&rxpacket[get_length], to_length - get_length);

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
                port->ClearPort();
                port->WritePort(txpacket, length);
            }  else { //check time out status
                if(isTimeOut(packetStartTime, packetWaitTime)){
                    if(get_length == 0){
                        printf("timed out\n");
                    } else {
                        printf("rxpacket corrupt\n");
                    }
                    if(txpacket[INSTRUCTION] != BULK_READ){
                        fail_counter++;
                        get_length = 0;
                        packetStartTime = getCurrentTime();
                        port->ClearPort();
                        port->WritePort(txpacket, length);
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
                printf("rxpacket[ID]: %d\n", rxpacket[ID]);
                if(rxpacket[LENGTH + rxpacket[LENGTH]] == checksum){
                    for(int j = 0; j < (rxpacket[LENGTH]-2); j++){
                        BulkData[rxpacket[ID]].table[BulkData[rxpacket[ID]].start_address + j] = rxpacket[PARAMETER + j];
                        printf("j: %d, rxpacket: %d\n", j, rxpacket[PARAMETER + j]);
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
                    printf("rx corrupt\n");
                    for(int j = 0; j <= get_length - 2; j++){
                        rxpacket[j] = rxpacket[j+2];
                    }
                    to_length = get_length -= 2;
                } 

                if(num == 0){
                    break;
                } else if(get_length <= 6) {
                    if(num != 0){
                        printf("rx corrupt\n");
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
    return length;
    }
}


/******************************************************
 * Takes in an instruction and list of params         *
 * Params needs to be in the form:                    *
 *    {ID_1, data_1, ID_2, data_2, ...}               *
 * There can be any amount of motors and in any order *
 * Note that paramlength does not include the ID      *
 ******************************************************/
int DarwinController::SyncWrite(unsigned char* packet, unsigned char instruction, unsigned char* params, unsigned char numparams, unsigned char paramlength, Port* port){
   
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
    return ReadWrite(packet, rxpacket);
}

// return value???
int DarwinController::WriteByte(int id, int address, int value){
    unsigned char txpacket[MAXNUM_TXPARAM + 10] = {0, };
    unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0, };

    txpacket[ID]           = (unsigned char)id;
    txpacket[INSTRUCTION]  = WRITE;
    txpacket[PARAMETER]    = (unsigned char)address;
    txpacket[PARAMETER+1]  = (unsigned char)value;
    txpacket[LENGTH]       = 4;

    int txlength = txpacket[LENGTH] + 4;
    // port->ClearPort();
    // int result = port->WritePort(txpacket, txlength);

    FinishPacket(txpacket);

    int result = ReadWrite(txpacket, rxpacket);

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
    unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0, };

    txpacket[ID]           = (unsigned char)id;
    txpacket[INSTRUCTION]  = WRITE;
    txpacket[PARAMETER]    = (unsigned char)address;
    txpacket[PARAMETER+1]  = (unsigned char)GetLowByte(value);
    txpacket[PARAMETER+2]  = (unsigned char)GetHighByte(value);
    txpacket[LENGTH]       = 5;

    int txlength = txpacket[LENGTH] + 4;
    // port->ClearPort();
    // int result = port->WritePort(txpacket, txlength);
    
    //makePacket(txpacket);

    int result = ReadWrite(txpacket, rxpacket);

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







