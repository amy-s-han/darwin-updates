/* 
 * a HelloWorld sort of program
 */

#include <stdio.h>
#include "ports.h"

using namespace std;


int main(){

    Port* port = new Port("/dev/ttyUSB0");
    if(port->OpenPort() == false){
        fprintf(stderr, "\n Fail to open port\n");
        fprintf(stderr, " CM-730 is used by another program or do not have root privileges.\n\n");
        return 0;
    }

    unsigned char txpacket[] = {0xFF, 0xFF, 0xC8, 0x05, 0x03, 0x1A, 0xE0, 0x03, 0x32};
    port->WritePort(txpacket, 9);


    printf("Press the ENTER key to close port!\n");
    getchar();

    port->ClosePort();

}

// for making color for led light
/*
int CM730::MakeColor(int red, int green, int blue)
{
    int r = red & 0xFF;
    int g = green & 0xFF;
    int b = blue & 0xFF;

    return (int)(((b>>3)<<10)|((g>>3)<<5)|(r>>3));
}
*/