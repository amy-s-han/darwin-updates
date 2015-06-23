
// read word: txpacket = {0xFF, 0xFF, id, 4, INST_READ, start address, 2, checksum}

#include <stdio.h>
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



int main(){

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

    unsigned char txpacket[] = {0xFF, 0xFF, 0x01, 0x04, 0x02, 0x24, 0x02, 0};
    int length = txpacket[LENGTH] + 4;

    printf("length is %d\n", length);
    getchar();
    
    txpacket[length-1] = CalculateChecksum(txpacket);

    port->ClearPort();


    port->WritePort(txpacket, length);

    int to_length = txpacket[PARAMETER+1] + 6; 

    //do we want a time out???
    //port->SetPacketTimeout(length) 

    int get_length = 0;
    int counter = 0;
    while(1){

        length = port->ReadPort(&rxpacket[get_length], to_length - get_length);

        get_length += length;

        if(get_length == to_length){
            if(rxpacket[0] == 0xFF && rxpacket[1] == 0xFF){
                //check checksum
                unsigned char checksum = CalculateChecksum(rxpacket);
                if(rxpacket[get_length -1] == checksum){
                    printf("Sucessful read\n");
                for(int i = 0; i< 9; i++){
                  printf("item %d: %u \n", i, rxpacket[i]);
                }
                if(counter == 10){
                  break;
                }
                counter ++;
                }
            }

        }
    }

    
    for(int i = 0; i< 15; i++){
      printf("item %d: %d\n", i, rxpacket[i]);
    }



    printf("Press the ENTER key to close port!\n");
    getchar();

    port->ClosePort();

}

