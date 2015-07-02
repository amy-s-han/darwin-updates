
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include "ports.h"

using namespace std;

#define ID                  (2)
#define LENGTH              (3)
#define INSTRUCTION         (4)
#define ERRBIT              (4)
#define PARAMETER           (5)
#define DEFAULT_BAUDNUMBER  (1)

#define PING           (1)
#define READ           (2)
#define WRITE          (3)
#define REG_WRITE      (4)
#define ACTION         (5)
#define RESET          (6)
#define SYNC_WRITE     (131)   // 0x83
#define BULK_READ      (146)   // 0x92

#define MAXNUM_TXPARAM      (256)
#define MAXNUM_RXPARAM      (1024)


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

int MakeWord(int lowbyte, int highbyte)
{
    unsigned short word;

    word = highbyte;
    word = word << 8;
    word = word + lowbyte;

    return (int)word;
}

//general read:
int read(Port *port, unsigned char *txpacket, unsigned char *rxpacket){

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
                port->BulkData[_id].length = _len;
                port->BulkData[_id].start_address = _addr;
            }

        } else if(txpacket[INSTRUCTION] == READ){

            to_length = txpacket[PARAMETER+1] + 6; 

        } else {

            to_length = 6;
            
        }

        int get_length = 0;
        int fail_counter = 0; // fail counter for ping
        int length = 0;
        
        // set packet time out:
        double packetStartTime = getCurrentTime();
        double packetWaitTime = 0.012 * (double)length + 5.0;

        while(1){ // loop for receiving packet
            if(fail_counter >= 5){ //failed reading 5 times. return.
                if(txpacket[INSTRUCTION] == PING){
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

                    fail_counter++;
                    get_length = 0;
                    packetStartTime = getCurrentTime();
                    port->ClearPort();
                    port->WritePort(txpacket, length);
                }       
            }
        }

        if(txpacket[INSTRUCTION] != BULK_READ){
            // if it isn't bulkread, must return here
            return length;
        } 

        for(int i = 0; i < num; i++){
            int _id = txpacket[PARAMETER+(3*i)+2];
            port->BulkData[_id].error = -1;
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
                        port->BulkData[rxpacket[ID]].table[port->BulkData[rxpacket[ID]].start_address + j] = rxpacket[PARAMETER + j];
                        printf("j: %d, rxpacket: %d\n", j, rxpacket[PARAMETER + j]);
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

    }
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

    int result = read(port, txpacket, rxpacket);
    printf("result inside ping: %d\n", result);
    if( result == 0){
        printf("inside ping - failed ping\n");
        return false;
    } else {
        printf("inside ping - successful ping\n");
        return true;
    }
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

    unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0, };
    //unsigned char txpacket[MAXNUM_TXPARAM + 10] = {0, };
    //txpacket = {0xFF, 0xFF, id, 4, INST_READ, start address, 2, checksum};

    unsigned char txpacketread[] = {0xFF, 0xFF, 0x14, 0x04, 0x02, 0x24, 0x02, 0};
    int length = txpacketread[LENGTH] + 4;
    
    txpacketread[length-1] = CalculateChecksum(txpacketread);


    // make bulkread packet
    int number = 0;

    unsigned char BulkReadTxPacket[MAXNUM_TXPARAM + 10] = {0, };

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

    length = BulkReadTxPacket[LENGTH] + 4;

    BulkReadTxPacket[length - 1] = CalculateChecksum(BulkReadTxPacket);


    int result = read(port, BulkReadTxPacket, rxpacket);

    printf("result: %d\n", result);

    int word = MakeWord((int)rxpacket[PARAMETER], (int)rxpacket[PARAMETER + 1]);
    printf("Motor position: %d\n\n", word);


    printf("Press the ENTER key to close port!\n");
    getchar();

    port->ClosePort();
      
}