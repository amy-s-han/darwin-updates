/* 
 * a HelloWorld sort of program
 */

#include <stdio.h>
#include "ports.h"

using namespace std;

#define LENGTH (3)

int MakeColor(int red, int green, int blue);
unsigned char CalculateChecksum(unsigned char *packet);

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

    
    int newcolor = MakeColor(255, 0, 0);
    printf("new color: %d\n", newcolor);

    

    

    unsigned char txpacket[] = {0xFF, 0xFF, 0xC8, 0x05, 0x03, 0x1C, 0x03, 0xE0, 0};
    txpacket[8] = CalculateChecksum(txpacket);
    printf("%d \n", txpacket[8]);

    printf("press enter\n");
    getchar();

    int value = port->WritePort(txpacket, 9);

    printf("%d\n", value);

    

    printf("Press the ENTER key to close port!\n");
    getchar();

    port->ClosePort();

}

unsigned char CalculateChecksum(unsigned char *packet)
{
    unsigned char checksum = 0x00;
    for(int i=2; i<packet[LENGTH]+3; i++ )
        checksum += packet[i];
    return (~checksum);
}

// for making color for led light

int MakeColor(int red, int green, int blue)
{
    int r = red & 0xFF;
    int g = green & 0xFF;
    int b = blue & 0xFF;

    return (int)(((b>>3)<<10)|((g>>3)<<5)|(r>>3));
}

