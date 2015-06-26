// write word: txpacket = {0xFF, 0xFF, joint_id, 5, INST_WRITE, start address, value low, value high, checksum}

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

    // motor initializations:
    // need to set MX28::P_TORQUE_ENABLE, MX28::P_P_GAIN

    unsigned char torque_txpacket[] = {0xFF, 0xFF, 0x14, 0x05, 0x03, 0x18, 0, 0, 0};
    unsigned char p_gain_txpacket[] = {0xFF, 0xFF, 0x14, 0x05, 0x03, 0x1C, 0x32, 0, 0};

    unsigned char i_gain_txpacket[] = {0xFF, 0xFF, 0x14, 0x05, 0x03, 0x1B, 0x1F, 0xFF, 0};

    //read position first:
    unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0, };

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
   	int word;

  	while(go){

        new_length = port->ReadPort(&rxpacket[get_length], to_length - get_length);

        get_length += new_length;

        if(get_length == to_length){ //received a packet of correct length
            if(rxpacket[0] == 0xFF && rxpacket[1] == 0xFF){
                //check checksum of incoming packet
                unsigned char checksum = CalculateChecksum(rxpacket);
                if(rxpacket[get_length -1] == checksum){
                    printf("Sucessful read\n");
                    for(int i = 0; i< 9; i++){
                      printf("item %d: %u \n", i, rxpacket[i]);
                    }
                    
                    word = MakeWord((int)rxpacket[PARAMETER], (int)rxpacket[PARAMETER + 1]);
                    printf("Motor position: %d\n\n", word);

                    if(counter >= 5){
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
        port->ClearPort();
        port->WritePort(txpacket, length);
        
        }
    }

    printf("Read angle as: %d, press enter to write new angle\n", word);
    getchar();

    // add some amount to the angle that was read in
    // split into low and high bytes
    word += 175;
 

    printf("New angle will be: %d, press enter to write new angle\n", word);
    getchar();

    port->ClearPort();
    port->WritePort(p_gain_txpacket, 9);
    port->ClearPort();
    port->WritePort(i_gain_txpacket, 9);

    int value_low = GetLowByte(word);

    int value_high = GetHighByte(word);

    // now write position

    //txpacket_write = {0xFF, 0xFF, joint_id, 5, INST_WRITE, start address, value low, value high, checksum};
    // 			 {header, header, id, length = 5, INST_WRITE = 3, ...}
    // want start_address to be MX28::P_GOAL_POSITION_L
    unsigned char txpacket_write[] = {0xFF, 0xFF, 0x14, 0x05, 0x03, 0x1E, (unsigned char)value_low, (unsigned char)value_high, 0};
    int txlength = txpacket_write[LENGTH] + 4;
    
    txpacket_write[txlength-1] = CalculateChecksum(txpacket_write);

    port->ClearPort();

    port->WritePort(txpacket_write, txlength);

    printf("Press the ENTER key to close port!\n");
    getchar();

    port->ClosePort();

}

