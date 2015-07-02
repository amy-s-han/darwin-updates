/* 
 * open port and bulk read
 */


#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include "ports.h"


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



Port::Port(const char* name){

    strcpy(m_PortName, name);
    m_Socket_fd = -1;
    m_ByteTransferTime = 0;

     // create bulk read data structure
    for(int i = 0; i < 254; i++){
        BulkData[i] = BulkReadData();
    }

}

Port::~Port(){
    ClosePort();
}

bool Port::OpenPort()
{

	struct termios newtio;
    struct serial_struct serinfo;
	double baudrate = 1000000.0; //bps (1Mbps)
    
    ClosePort();

    if((m_Socket_fd = open(m_PortName, O_RDWR|O_NOCTTY|O_NONBLOCK)) < 0){
        printf("failed!/n");
        ClosePort();
        return false;
    }

	
	printf("success!\n");

	// You must set 38400bps!
	memset(&newtio, 0, sizeof(newtio));
    newtio.c_cflag      = B38400|CS8|CLOCAL|CREAD;
    newtio.c_iflag      = IGNPAR;
    newtio.c_oflag      = 0;
    newtio.c_lflag      = 0;
    newtio.c_cc[VTIME]  = 0;
    newtio.c_cc[VMIN]   = 0;
    tcsetattr(m_Socket_fd, TCSANOW, &newtio);


	printf("Set %.1fbps ", baudrate);

	// Set non-standard baudrate
    if(ioctl(m_Socket_fd, TIOCGSERIAL, &serinfo) < 0){
        printf("failed!/n");
        ClosePort();
        return false;
    }

    serinfo.flags &= ~ASYNC_SPD_MASK;
    serinfo.flags |= ASYNC_SPD_CUST;
    serinfo.custom_divisor = serinfo.baud_base / baudrate;
	
    if(ioctl(m_Socket_fd, TIOCSSERIAL, &serinfo) < 0)
	{
		printf("failed!/n");
        ClosePort();
        return false;
	}

	printf("success!\n");

	tcflush(m_Socket_fd, TCIFLUSH);

    m_ByteTransferTime = (1000.0 / baudrate) * 12.0;
	
    return true;

}

void Port::ClosePort()
{
    if(m_Socket_fd != -1)
        close(m_Socket_fd);
    m_Socket_fd = -1;
}

void Port::ClearPort()
{
    tcflush(m_Socket_fd, TCIFLUSH);
}

int Port::WritePort(unsigned char* packet, int numPacket)
{
    return write(m_Socket_fd, packet, numPacket);
}

int Port::ReadPort(unsigned char* packet, int numPacket)
{
    return read(m_Socket_fd, packet, numPacket);
}


