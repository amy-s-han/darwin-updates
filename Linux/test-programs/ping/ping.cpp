
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include "ports.h"

using namespace std;

#define ID 					(2)
#define LENGTH 				(3)
#define INSTRUCTION			(4)
#define PARAMETER 			(5)

#define MAXNUM_TXPARAM 		(256)
#define MAXNUM_RXPARAM 		(1024)

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


	txpacket[0] 		   = 0xFF;
    txpacket[1]			   = 0xFF;
    txpacket[ID]           = (unsigned char)id;
    txpacket[INSTRUCTION]  = 1;
    txpacket[LENGTH]       = 2;
    
	int length = txpacket[LENGTH] + 4;
    
    txpacket[length-1] = CalculateChecksum(txpacket);

    port->ClearPort();

    port->WritePort(txpacket, length);

    int to_length = 6;

    printf("length is %d\n", length);
    printf("to_length = %d\n\n", to_length);

    int get_length = 0;
    int new_length = 0;
    int fail_counter = 0;
    bool result = false;

    // set packet time out:
    double packetStartTime = getCurrentTime();
    double packetWaitTime = 0.012 * (double)length + 5.0;
    printf("Wait time is: %f\n", packetWaitTime);

    while(1){
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
                    printf("Successful read\n");
                    result = true;
                    break;
                }
            }
	        get_length = 0;
	        port->ClearPort();
	        port->WritePort(txpacket, length);
        } 
        if(isTimeOut(packetStartTime, packetWaitTime)){
        	printf("timeout!\n");
        	fail_counter++;
	        get_length = 0;
	        packetStartTime = getCurrentTime();
	        port->ClearPort();
	        port->WritePort(txpacket, length);
        }       
        
    }

    for(int i = 0; i < 10; i ++){
    	printf("Index %d: %d\n", i, rxpacket[i]);
    }

    return result;
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

    printf("Finshed dxl power up. Press enter\n");
    getchar();

    bool result = Ping(0x14, 0, port);

    printf("Ping result: %d\n", result);

}