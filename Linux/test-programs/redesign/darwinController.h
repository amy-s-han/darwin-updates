#ifndef _DARWINCONTROLLER_H_
#define _DARWINCONTROLLER_H_

#include <stdint.h>


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

struct JointData {
        uint8_t  flags;
        uint16_t goal;
        uint8_t p, i, d;
};


class DarwinController {
    private:

    public: 
        Port port;
        
            JointData joints[NUM_JOINTS];

        BulkReadData BulkData[ID_BROADCAST]; //254
        unsigned char BulkReadTxPacket[266];

        DarwinController();
        ~DarwinController();

        bool PowerDXL();
        bool Initialize(const char* name);
        void ClosePort();

        bool InitToPose();

        unsigned char CalculateChecksum(unsigned char *packet);
        double getCurrentTime();
        bool isTimeOut(double packetStartTime, double packetWaitTime);
        int GetLowByte(int word);
        int GetHighByte(int word);

        void MakeBulkPacket(unsigned char *BulkReadTxPacket);

        void MakePacket(unsigned char* packet, unsigned char motor_ID, unsigned char parambytes, unsigned char instruction, unsigned char address, unsigned char* params);
        void FinishPacket(unsigned char *txpacket);
        
        int ReadWrite(unsigned char *txpacket, unsigned char *rxpacket);
        int SyncWrite(unsigned char* packet, unsigned char instruction, unsigned char* params, unsigned char numparams, unsigned char paramlength);
        int BulkRead(unsigned char *rxpacket);

        int WriteByte(int id, int address, int value);
        int WriteWord(int id, int address, int value);

        int ReadByte(int id, int address, int *word);
        int ReadWord(int id, int address, int *word);

        int MakeWord(int lowbyte, int highbyte);
        bool Ping(int id, int *error);
        int MakeColor(int red, int green, int blue);

        double Ticks2DegAngle(int ticks);
        int DegAngle2Ticks(double angle);
        double Ticks2RadAngle(int ticks);
        int RadAngle2Ticks(double angle);

        int SetJointAngle(unsigned char joint_ID, int goal_angle);
        int SetMoveSpeed(unsigned char joint_ID, int move_speed);
        int Set_P_Gain(unsigned char joint_ID, unsigned char P_Value);
        int Set_I_Gain(unsigned char joint_ID, unsigned char I_Value);
        int Set_D_Gain(unsigned char joint_ID, unsigned char D_Value);
        int Set_PID_Gain(unsigned char joint_ID, unsigned char P_Value, unsigned char I_Value, unsigned char D_Value);
        int Set_Torque_Enable(unsigned char joint_ID, unsigned char is_enabled);


        // For JointData struct
        // These set values for all motors and require and input array of 20
        void Set_Enables(uint8_t* data);
        int Set_P_Data(uint8_t* data);
        int Set_I_Data(uint8_t* data);
        int Set_D_Data(uint8_t* data);

        // These set individual motor values
        void Set_Enables(unsigned char motor_ID, uint8_t value);
        void Set_P_Data(unsigned char motor_ID, uint8_t value);
        void Set_I_Data(unsigned char motor_ID, uint8_t value);
        void Set_D_Data(unsigned char motor_ID, uint8_t value);
        int Set_Pos_Data(unsigned char motor_ID, uint16_t value);

        int Set_Pos_Data(uint16_t* data);
        void Update_Motors();

        void foo();



};


#endif
