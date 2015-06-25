/* 
 * a HelloWorld sort of program
 */

#include <stdio.h>
#include <unistd.h>
#include "ports.h"
#include <math.h>

using namespace std;

#define LENGTH (3)
#define total (2000)


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

    port->ClearPort();

    // 0x01C for eye LEDs, 0x1A for head LED
    unsigned char txpacket[] = {0xFF, 0xFF, 0xC8, 0x05, 0x03, 0x1C, 0xE0, 0x03, 0};
    txpacket[8] = CalculateChecksum(txpacket);
    printf("%d \n", txpacket[8]);

    int value = port->WritePort(txpacket, 9);


    printf("press enter\n");
    getchar();

    // My attempt at cycling through colors 
    // Merp doesn't work yet


    double i[total] = {0, };
    double j[total] = {0, };
    double k[total] = {0, };

    int count;

    for(count = 0; count < total; count++){
        i[count] = 255*sin(6.28*count/(total/2));
        j[count] = 255*sin(6.28*count/(total/11));
        k[count] = 255*sin(6.28*count/(total/21));
    }

    int red = 0;
    int green = 0;
    int blue = 0;

    count = 0;
    int newcolor;


    while(count < total){
        port->ClearPort();
        newcolor = MakeColor(i[count], j[count], k[count]);
        txpacket[6] = GetLowByte(newcolor);
        txpacket[7] = GetHighByte(newcolor);
        txpacket[8] = 0;
        txpacket[8] = CalculateChecksum(txpacket);
        port->WritePort(txpacket, 9);
        usleep(10000);
        count++;
    }



     

    printf("Press the ENTER key to close port!\n");
    getchar();

    port->ClosePort();

}

