#ifndef _DARWINCONTROLLER_H_
#define _DARWINCONTROLLER_H_


#include "CM730.h"

class BulkReadData {
    public:
        int start_address;
        int length;
        int error;
        unsigned char table[MX28_MAXNUM_ADDRESS]; //maxnum address = 49

        BulkReadData();
        ~BulkReadData();

        int ReadByte(int address);
        int ReadWord(int address);
        int MakeWord(int lowbyte, int highbyte);

};

class DarwinController {
	private:

	public: 
		Port *port;
		BulkReadData BulkData[ID_BROADCAST]; //254

		DarwinController();
		~DarwinController();

        unsigned char CalculateChecksum(unsigned char *packet);
        double getCurrentTime();
        bool isTimeOut(double packetStartTime, double packetWaitTime);

        bool PowerDXL();
		bool Initialize(const char* name);
        void ClosePort();

        /*
        int WriteWord(????);
        int WriteByte(????);
        int ReadWord(?????);
        int ReadByte(?????);
        */





};


#endif
