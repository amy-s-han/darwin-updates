/* CM730.cpp - 
    handles the communication with the CM730 board on Darwin-OP. 
    It is called by darwinController.cpp. The user should rarely 
    have to directly interact with this file.

    Summer 2015 - Amy Han and Daniel Rovner
*/


#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>


#include "CM730.h"


Port::Port(){

    //strcpy(m_PortName, name);
    m_Socket_fd = -1;
    m_ByteTransferTime = 0;

}

Port::~Port(){

}

bool Port::OpenPort(const char* name){

	struct termios newtio;
    struct serial_struct serinfo;
	double baudrate = 1000000.0; //bps (1Mbps)
    
    ClosePort();

    if((m_Socket_fd = open(name, O_RDWR|O_NOCTTY|O_NONBLOCK)) < 0){
        printf("failed!/n");
        ClosePort();
        return false;
    }

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

	tcflush(m_Socket_fd, TCIFLUSH);

    m_ByteTransferTime = (1000.0 / baudrate) * 12.0;
	
    printf("Open port success!\n");

    return true;

}

void Port::ClosePort(){
    if(m_Socket_fd != -1)
        close(m_Socket_fd);
    m_Socket_fd = -1;
}

void Port::ClearPort()
{
    tcflush(m_Socket_fd, TCIFLUSH);
}

void Port::DrainPort(){
    tcdrain(m_Socket_fd);
}

int Port::WritePort(unsigned char* packet, int numPacket){
    return write(m_Socket_fd, packet, numPacket);
}

int Port::ReadPort(unsigned char* packet, int numPacket){
    return read(m_Socket_fd, packet, numPacket);
}


