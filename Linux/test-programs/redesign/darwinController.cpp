
#include <stdio.h>
#include <sys/time.h>

#include "darwinController.h"


#define MAXNUM_TXPARAM 	(256)
#define MAXNUM_RXPARAM 	(1024)
#define ID		(2)
#define LENGTH         	(3)
#define INSTRUCTIONS	(4)
#define ERRBIT		(4)
#define PARAMETER	(5)




BulkReadData::BulkReadData() :
        start_address(0),
        length(0),
        error(-1)
{
    for(int i = 0; i < 49; i++) // MAXNUM ADDRESS = 49
        table[i] = 0;
}

BulkReadData::~BulkReadData(){
    //TODO: FINISH DESTRUCTOR
}

int BulkReadData::ReadByte(int address){
    if(address >= start_address && address < (start_address + length))
        return (int)table[address];

    return 0;
}

int BulkReadData::ReadWord(int address){
    if(address >= start_address && address < (start_address + length))
        return MakeWord(table[address], table[address+1]);

    return 0;
}

int BulkReadData::MakeWord(int lowbyte, int highbyte){
    unsigned short word;

    word = highbyte;
    word = word << 8;
    word = word + lowbyte;

    return (int)word;
}





DarwinController::DarwinController(){
	// what to do here?
}

DarwinController::~DarwinController(){
	//what to do here?
}


unsigned char DarwinController::CalculateChecksum(unsigned char *packet)
{
    unsigned char checksum = 0x00;
    for(int i=2; i<packet[LENGTH]+3; i++ )
        checksum += packet[i];
    return (~checksum);
}

double DarwinController::getCurrentTime(){
    struct timeval tv;
    gettimeofday(&tv, NULL);

    return ((double)tv.tv_sec*1000.0 + (double)tv.tv_usec/1000.0);
}

bool DarwinController::isTimeOut(double packetStartTime, double packetWaitTime){
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

bool DarwinController::PowerDXL(){
    unsigned char dxltxpacket[] = {0xFF, 0xFF, ID_CM, 0x04, WRITE, DXL_POWER, 0x01, 0};
    dxltxpacket[7] = CalculateChecksum(dxltxpacket);
    int result = port->WritePort(dxltxpacket, 8); // Robotis uses writebyte
    //robotis also has a sleep for 300msec.
    if(result != 0){
        return true;
    } else {
        return false;
    }
}

bool DarwinController::Initialize(const char* name){
	port = new Port(name);
	if(port->OpenPort() == false){
		fprintf(stderr, "\n Fail to open port\n");
        fprintf(stderr, " CM-730 is used by another program or does not have root privileges.\n\n");
        return false;
	}

	if(PowerDXL() == false){ //failed to power up Dynamixel
        fprintf(stderr, "Fail to change Dynamixel power\n");
        return false;
	} else {
        printf("Dynamixel powered up\n");
        return true;
    }

    // create bulk read data structure
    for(int i = 0; i < 254; i++){
        BulkData[i] = BulkReadData();
    }
}

void DarwinController::ClosePort(){
    port->ClosePort();
}



