// test program for bulkreading

#include <stdio.h>
#include <sys/time.h>
#include "ports.h"
#include <stdint.h>


using namespace std;

#define ID                  (2)
#define LENGTH              (3)
#define INSTRUCTION         (4)
#define ERRBIT              (4)
#define PARAMETER           (5)

#define MAXNUM_TXPARAM      (256)
#define MAXNUM_RXPARAM      (1024)

struct ReadData {
        uint8_t  p;
        uint8_t  i;
        uint8_t  d;

        uint16_t goal_pos;
        uint16_t max_speed;
        uint16_t cur_pos;
        uint16_t cur_speed;
        uint16_t load;
        uint16_t torque_limit;

        uint8_t  registered;
        uint8_t  moving;
};

int MakeWord(int lowbyte, int highbyte){
    unsigned short word;

    word = highbyte;
    word = word << 8;
    word = word + lowbyte;

    return (int)word;
}

unsigned char CalculateChecksum(unsigned char *packet)
{
    unsigned char checksum = 0x00;
    for(int i=2; i<packet[LENGTH]+3; i++ )
        checksum += packet[i];
    return (~checksum);
}

double getCurrentTime(){
    struct timeval tv;
    gettimeofday(&tv, NULL);

    return ((double)tv.tv_sec*1000.0 + (double)tv.tv_usec/1000.0);
}

bool isTimeOut(double packetStartTime, double packetWaitTime){
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

bool Ping(int id, int *error, Port *port) {
    unsigned char txpacket[MAXNUM_TXPARAM + 10] = {0, };
    unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0, };


    txpacket[0]            = 0xFF;
    txpacket[1]            = 0xFF;
    txpacket[ID]           = (unsigned char)id;
    txpacket[INSTRUCTION]  = 1;
    txpacket[LENGTH]       = 2;
    
    int length = txpacket[LENGTH] + 4;
    
    txpacket[length-1] = CalculateChecksum(txpacket);

    port->ClearPort();

    port->WritePort(txpacket, length);

    int to_length = 6;

    int get_length = 0;
    int new_length = 0;
    int fail_counter = 0;
    bool go = true;
    bool result = false;

    // set packet time out:
    double packetStartTime = getCurrentTime();
    double packetWaitTime = 0.012 * (double)length + 5.0;

    while(go){
        if(fail_counter >= 5){
            printf("failed ping\n");
            break;
        }

        new_length = port->ReadPort(&rxpacket[get_length], to_length - get_length);

        get_length += new_length;

        if(get_length == to_length){ //received a packet of correct length
            if(rxpacket[0] == 0xFF && rxpacket[1] == 0xFF){
                //check checksum of incoming packet
                unsigned char checksum = CalculateChecksum(rxpacket);
                if(rxpacket[get_length -1] == checksum){
                    printf("Successful ping\n");
                    result = true;
                    break;
                }
            }
            get_length = 0;
            port->ClearPort();
            port->WritePort(txpacket, length);
        } 
        if(isTimeOut(packetStartTime, packetWaitTime)){
	        //printf("timeout!\n");
            fail_counter++;
            get_length = 0;
            packetStartTime = getCurrentTime();
            port->ClearPort();
            port->WritePort(txpacket, length);
        }       
        
    }

    return result;
}

void bulkread(Port *port, unsigned char info[]){

    // [0xFF, 0xFF, ID, LENGTH, INSTRUCTION/ERRBIT, PARAMETER, ...]
    // m_BulkReadTxPacket[0xFF, 0xFF, 0xFE, LENGTH, 0x92, 0x0, 

    // make bulkread packet
    int count = 0;
    int number = 0;
    unsigned char BulkReadTxPacket[MAXNUM_TXPARAM + 10] = {0, };
    unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0, };

    BulkReadTxPacket[0] = 0xFF;
    BulkReadTxPacket[1] = 0xFF;
    BulkReadTxPacket[ID] = 0xFE;
    BulkReadTxPacket[INSTRUCTION] = 0x92;
    BulkReadTxPacket[PARAMETER] = 0x0;
    
    if(Ping(0xC8, 0, port)){
        BulkReadTxPacket[PARAMETER+3*number+1] = 30; //length
        BulkReadTxPacket[PARAMETER+3*number+2] = 0xC8; // ID_CM
        BulkReadTxPacket[PARAMETER+3*number+3] = 24; //P_DXL_POWER
        number++;
    }

    if(Ping(70, 0, port)){
        BulkReadTxPacket[PARAMETER+3*number+1] = 10;    // length
        BulkReadTxPacket[PARAMETER+3*number+2] = 0x70;  // ID_L_FSR
        BulkReadTxPacket[PARAMETER+3*number+3] = 0x1A;  // start address P_FSR1_L
        number++;
    }

    if(Ping(0x6F, 0, port)){
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

    //finish making bulkread packet
    

    port->ClearPort(); //calls tcflush(m_Socket_fd, TCIFLUSH)
	if(port->WritePort(BulkReadTxPacket, length) == length){
        int to_length = 0;
        int num = (BulkReadTxPacket[LENGTH]-3) / 3;
	
        for(int i = 0; i < num; i++){
            int _id = BulkReadTxPacket[PARAMETER+(3*i)+2];
            int _len = BulkReadTxPacket[PARAMETER+(3*i)+1];
            int _addr = BulkReadTxPacket[PARAMETER+(3*i)+3];

            to_length += _len + 6;
            port->BulkData[_id].length = _len;
            port->BulkData[_id].start_address = _addr;
        }
        //set packet time out:
        double packetStartTime = getCurrentTime();
        double packetWaitTime = 0.012 * (double)length + 5.0;

        int get_length = 0;

    	printf("getlength: %d, tolength: %d\n", get_length, to_length);
    	getchar();

        while(1){
            length = port->ReadPort(&rxpacket[get_length], to_length - get_length);

            get_length += length;

            if(get_length == to_length){
                break;
            } else {
                if(isTimeOut(packetStartTime, packetWaitTime)){
                    if(get_length == 0){
                        printf("timed out\n");
                    } else {
		                //printf("rxpacket corrupt\n");
                    }
                }
            }
        }

        for(int i = 0; i < num; i++){
            int _id = BulkReadTxPacket[PARAMETER+(3*i)+2];
            port->BulkData[_id].error = -1;
        }

    	/*
    	for(int i = 0; i<587; i++){
    	   printf("%d ", rxpacket[i]);
    	  getchar();
    	}
    	*/

        while(1){
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
                        port->BulkData[rxpacket[ID]].table[port->BulkData[rxpacket[ID]].start_address + j] = rxpacket[PARAMETER + j];
                        printf("j: %d, rxpacket: %d\n", j, rxpacket[PARAMETER + j]);
                        info[count++] = rxpacket[PARAMETER + j];
                    }

                    port->BulkData[rxpacket[ID]].error = (int)rxpacket[ERRBIT];

                    int cur_packet_length = LENGTH + 1 + rxpacket[LENGTH];
                    to_length = get_length - cur_packet_length;

                    for(int j = 0; j <= to_length; j++){
                        rxpacket[j] = rxpacket[j+cur_packet_length];
                    }
                    get_length = to_length;
                    num--;
                } else {
		            //printf("rx corrupt\n");
                    for(int j = 0; j <= get_length - 2; j++){
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

    }

}



int main(int argc, char** argv){
   
    Port* port = new Port("/dev/ttyUSB0");
    if(port->OpenPort() == false){
        fprintf(stderr, "\n Fail to open port\n");
        fprintf(stderr, " CM-730 is used by another program or do not have root privileges.\n\n");
        return 0;
    }

    unsigned char info[MAXNUM_RXPARAM] = {0, };
    for(int i = 0; i<MAXNUM_RXPARAM; i++)
	info[i] = -99;

    //dxl power on stuff
    //(WriteByte(CM730::ID_CM, CM730::P_DXL_POWER, 1, 0)

    ReadData joint_read_data[21];


    unsigned char dxltxpacket[] = {0xFF, 0xFF, 0xC8, 0x04, 0x03, 0x18, 0x01, 0};
    dxltxpacket[7] = CalculateChecksum(dxltxpacket);
    port->WritePort(dxltxpacket, 8);
    printf("Finished dxl power up. Press enter\n");
    getchar();
    int buf = 30;
    bulkread(port, info);
    for(int i = 0; i<20; i++){
        // 23 uchars per motor
        ReadData& rd = joint_read_data[i];
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

        buf = buf + 23;
    }

    for(int i = 0; i<20; i++){
        ReadData& rd = joint_read_data[i];
        printf("%d ", rd.d);
        printf("%d ", rd.i);
        printf("%d ", rd.p);
        printf("%d ", rd.goal_pos);
        printf("%d ", rd.max_speed);
        printf("%d ", rd.torque_limit);
        printf("%d ", rd.cur_pos);
        printf("%d ", rd.cur_speed);
        printf("%d ", rd.load);
        printf("%d ", rd.registered);
        printf("%d ", rd.moving);
    }

}
