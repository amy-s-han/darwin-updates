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

        bool PowerDXL();
		bool Initialize(const char* name);
        void ClosePort();



        unsigned char CalculateChecksum(unsigned char *packet);
        double getCurrentTime();
        bool isTimeOut(double packetStartTime, double packetWaitTime);
        int GetLowByte(int word);
        int GetHighByte(int word);

        void MakePacket(unsigned char* packet, unsigned char motor_ID, unsigned char parambytes, unsigned char instruction, unsigned char address, unsigned char* params);
        void FinishPacket(unsigned char *txpacket);
        
        int ReadWrite(unsigned char *txpacket, unsigned char *rxpacket);
        int SyncWrite(unsigned char* packet, unsigned char instruction, unsigned char* params, unsigned char numparams, unsigned char paramlength, Port* port);

        int WriteByte(int id, int address, int value);
        int WriteWord(int id, int address, int value);

        int ReadByte(int id, int address, int *word);
        int ReadWord(int id, int address, int *word);

        int MakeWord(int lowbyte, int highbyte);
        int MakeColor(int red, int green, int blue);

        int SetJointAngle(unsigned char joint_ID, int goal_angle);

        int SetMoveSpeed(unsigned char joint_ID, int move_speed);
        int Set_P_Gain(unsigned char joint_ID, unsigned char P_Value);
        int Set_I_Gain(unsigned char joint_ID, unsigned char I_Value);
        int Set_D_Gain(unsigned char joint_ID, unsigned char D_Value);
        int Set_PID_Gain(unsigned char joint_ID, unsigned char P_Value, unsigned char I_Value, unsigned char D_Value);
        int Set_Torque_Enable(unsigned char joint_ID, unsigned char is_enabled);






};


#endif
