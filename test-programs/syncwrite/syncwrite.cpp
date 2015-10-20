
//write word: txpacket = {0xFF, 0xFF, joint_id, 5, INST_WRITE, start address, value low, value high, checksum}

#include <math.h>
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

//*******************************************************************************
// Takes in an instruction and list of params (write bytes only, no motor ID's) *
// If each instruction needs 1 param, then                                      *
//   Param[i] is the instruction for the (i+1)th motor                          *
// In general, if each instruction needs n params, then                         *
//   Param[n*i]-Param[n*i+n] are the n params for the (i+1)th motor             *
//*******************************************************************************

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

    printf("pre-checksum packet is: \n", packet);
    for (int x = 0; x < len; x ++){
      printf("%x ", packet[x]);
    }

    packet[len-1] = CalculateChecksum(packet);

    printf("\n\n\npre-checksum packet is: \n", packet);
    for (int x = 0; x < len; x ++){
      printf("%x ", packet[x]);
    }
}

int main(int argc, char** argv){
   
    int quick_change = 2;

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


    unsigned char paramlist[40] = {0, };
       for(unsigned char q = 0; q < 40; q++){
         if(q%2==0)
         paramlist[q] = q/2+1;
       else
         paramlist[q] = 0;
    }
 
    unsigned char packet[100] = {0, };
    unsigned char numparams = 40;
    unsigned char lenparam = 1;


    // 0x19 is LED, 0x18 is torque enable 
    SyncWrite(packet, 0x19, paramlist, numparams, lenparam);

    port->ClearPort();
    port->WritePort(packet, numparams + 8);

    printf("Press the ENTER key to close port!\n");
    getchar();

    port->ClosePort();
    
}

