
// read word: txpacket = {0xFF, 0xFF, id, 4, INST_READ, start address, 2, checksum}

#include <stdio.h>
#include <unistd.h>
#include <cstdlib>
#include "ports.h"

using namespace std;

#define LENGTH (3)
#define PARAMETER (5)

#define MAXNUM_TXPARAM (256)
#define MAXNUM_RXPARAM (1024)

unsigned char CalculateChecksum(unsigned char *packet)
{
    unsigned char checksum = 0x00;
    for(int i=2; i<packet[LENGTH]+3; i++ )
        checksum += packet[i];
    return (~checksum);
}


int MakeWord(int lowbyte, int highbyte)
{
    unsigned short word;

    word = highbyte;
    word = word << 8;
    word = word + lowbyte;

    return (int)word;
}


int main(int argc, char** argv){
    int number_reads;
    if(argc == 1){
        //default read 5 instances of joint data
        number_reads = 5;
	printf("Default number of reads is 5\n");
    } else {
        number_reads = atoi(argv[1]);
        printf("argv[1]: %d\n", number_reads);
    }


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

    unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0, };
    //unsigned char txpacket[MAXNUM_TXPARAM + 10] = {0, };
    //txpacket = {0xFF, 0xFF, id, 4, INST_READ, start address, 2, checksum};

    unsigned char txpacket[] = {0xFF, 0xFF, 0x14, 0x04, 0x02, 0x24, 0x02, 0};
    int length = txpacket[LENGTH] + 4;
    
    txpacket[length-1] = CalculateChecksum(txpacket);

    port->ClearPort();

    port->WritePort(txpacket, length);

    int to_length = txpacket[PARAMETER+1] + 6; 

    printf("length is %d\n", length);
    printf("to_length = %d\n\n", to_length);

    //do we want a time out???
    //port->SetPacketTimeout(length) 

    int get_length = 0;
    int new_length = 0;
    int counter = 0;
    bool go = true;
   

  while(go){
    // printf("top of while. get_length = %d and to_length = %d\n", get_length, to_length);

        new_length = port->ReadPort(&rxpacket[get_length], to_length - get_length);

        get_length += new_length;

	// printf("new length from port read: %d\n", new_length);

        if(get_length == to_length){ //received a packet of correct length
            if(rxpacket[0] == 0xFF && rxpacket[1] == 0xFF){
                //check checksum of incoming packet
                unsigned char checksum = CalculateChecksum(rxpacket);
                if(rxpacket[get_length -1] == checksum){
                    printf("Sucessful read\n");
                    for(int i = 0; i< 9; i++){
                      printf("item %d: %u \n", i, rxpacket[i]);
                    }
                    
                    int word = MakeWord((int)rxpacket[PARAMETER], (int)rxpacket[PARAMETER + 1]);
                    printf("Motor position: %d\n\n", word);

                    if(counter >= number_reads){
                      printf("break\n");
                      go = false;
                      break;
                    }
                    counter ++;
                }
            }
        printf("Sleeping\n");
        sleep(1);
        get_length = 0;
        //printf("get_length = %d, to_length = %d\n", get_length, to_length);
        port->ClearPort();
        port->WritePort(txpacket, length);
        
        }
    }

    printf("Press the ENTER key to close port!\n");
    getchar();

    port->ClosePort();

}

