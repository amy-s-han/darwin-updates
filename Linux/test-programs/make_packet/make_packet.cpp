
#define MAXNUM_TXPARAM 	(256)
#define MAXNUM_RXPARAM 	(1024)
#define ID		(2)
#define LENGTH         	(3)
#define INSTRUCTION	(4)
#define ERRBIT		(4)
#define PARAMETER	(5)

enum{
	MODEL_NUMBER_L		= 0,
	MODEL_NUMBER_H		= 1,
	VERSION			= 2,
	ID			= 3,
	BAUD_RATE		= 4,
	RETURN_DELAY_TIME	= 5,
	CW_ANGLE_LIMIT_L	= 6,
	CW_ANGLE_LIMIT_H	= 7,
	CCW_ANGLE_LIMIT_L	= 8,
	CCW_ANGLE_LIMIT_H	= 9,
	TEMP_LIMIT		= 11,
	LOWEST_LIMIT_VOLTAGE	= 12,
	HIGHEST_LIMIT_VOLTAGE	= 13,
	MAX_TORQUE_L		= 14,
	MAX_TORQUE_H		= 15,
	STATUS_RETURN_LEVEL	= 16,
	ALARM_LED		= 17,
	ALARM_SHUTDOWN		= 18,
	MULTI_TURN_OFFSET_L	= 20,
	MULTI_TURN_OFFSET_H	= 21,
	RESOLUTION_DIVIDER	= 22,
	TORQUE_ENABLE		= 24,
	LED			= 25,
	D_GAIN			= 26,
	I_GAIN			= 27,
	P_GAIN			= 28,
	GOAL_POSITION_L		= 30,
	GOAL_POSITION_H		= 31,
	MOVING_SPEED_L		= 32,
	MOVING_SPEED_H		= 33,
	TORQUE_LIMIT_L		= 34,
	TORQUE_LIMIT_H		= 35,
	PRESENT_POSITION_L	= 36,
	PRESENT_POSITION_H	= 37,
	PRESENT_SPEED_L		= 38,
	PRESENT_SPEED_H		= 39,
	PRESENT_LOAD_L		= 40,
	PRESENT_LOAD_H		= 41,
	PRESENT_VOLTAGE		= 42,
	PRESENT TEMPERATURE	= 43,
	REGISTERED		= 44,
	MOVING			= 46,
	LOCK			= 47,
	PUNCH_L			= 48,
	PUNCH_H			= 49,
	MX28_MAXNUM_ADDRESS
};

enum{
	R_SHOULDER_PITCH	= 1,
	L_SHOULDER_PITCH	= 2,
	R_SHOULDER_ROLL 	= 3,
	L_SHOULDER_ROLL 	= 4,
	R_ELBOW         	= 5,
	L_ELBOW         	= 6,
	R_HIP_YAW       	= 7,
	L_HIP_YAW       	= 8,
	R_HIP_ROLL      	= 9,
	L_HIP_ROLL     		= 10,
	R_HIP_PITCH  	  	= 11,
	L_HIP_PITCH	   	= 12,
	R_KNEE      	  	= 13,
	L_KNEE        		= 14,
	R_ANKLE_PITCH		= 15,
	L_ANKLE_PITCH   	= 16,
	R_ANKLE_ROLL    	= 17,
	L_ANKLE_ROLL    	= 18,
	HEAD_PAN        	= 19,
	HEAD_TILT       	= 20,
	NUM_JOINTS
};

enum{
	PING			= 1,
	READ			= 2,
	WRITE			= 3,
	REG_WRITE		= 4,
	ACTION			= 5,
	RESET			= 6,
	SYNC_WRITE		= 131,
	BULK_READ		= 146,
};

enum{
	MODEL_NUMBER_L		= 0,
	MODEL_NUMBER_H		= 1,
	VERSION			= 2,
	ID			= 3,
	BAUD_RATE		= 4,
	RETURN_DELAY_TIME	= 5,			
	RETURN_LEVEL		= 16,
	DXL_POWER		= 24,
	LED_PANNEL		= 25,
	LED_HEAD_L		= 26,
	LED_HEAD_H		= 27,
	LED_EYE_L		= 28,
	LED_EYE_H		= 29,
	BUTTON			= 30,			
	GYRO_Z_L		= 38,
	GYRO_Z_H		= 39,
	GYRO_Y_L		= 40,
	GYRO_Y_H		= 41,
	GYRO_X_L		= 42,
	GYRO_X_H		= 43,
	ACCEL_X_L		= 44,
	ACCEL_X_H		= 45,
	ACCEL_Y_L		= 46,
	ACCEL_Y_H		= 47,
	ACCEL_Z_L		= 48,
	ACCEL_Z_H		= 49,
	VOLTAGE			= 50,
	LEFT_MIC_L		= 51,
	LEFT_MIC_H		= 52,
	ADC2_L			= 53,
	ADC2_H			= 54,
	ADC3_L			= 55,
	ADC3_H			= 56,
	ADC4_L			= 57,
	ADC4_H			= 58,
	ADC5_L			= 59,
	ADC5_H			= 60,
	ADC6_L			= 61,
	ADC6_H			= 62,
	ADC7_L			= 63,
	ADC7_H			= 64,
	ADC8_L			= 65,
	ADC8_H			= 66,
	RIGHT_MIC_L		= 67,
	RIGHT_MIC_H		= 68,
	ADC10_L			= 69,
	ADC10_H			= 70,
	ADC11_L			= 71,
	ADC11_H			= 72,
	ADC12_L			= 73,
	ADC12_H			= 74,
	ADC13_L			= 75,
	ADC13_H			= 76,
	ADC14_L			= 77,
	ADC14_H			= 78,
	ADC15_L			= 79,
	ADC15_H			= 80,
	MAX_NUM_ADDRESS	
};

enum{
	ID_CM			= 200,
	ID_BROADCAST		= 254
};

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

/*********************************************************
 * Turns everything you need for a packet into a packet. *
 * Does not cover bulk read or sync write.               *
 *********************************************************/
void MakePacket(unsigned char* packet, unsigned char motor_ID, unsigned char parambytes, unsigned char instruction, unsigned char address, unsigned char* params){

    unsigned char len = parambytes + 6; // Last index of array (where checksum goes)--so not truly the length
    packet[0] = 0xFF;                   // Heading
    packet[1] = 0xFF;
    packet[ID] = motor_ID;                     // Motors are 1-20, CM730 is something else
    packet[LENGTH] = len-3;                  // Length of packet except for headings, ID, Checksum
    packet[INSTRUCTION] = instruction;            
    packet[ADDRESS] = address;                // Where to carry out instruction

    for(unsigned char i = 0; i < parambytes; i++)  // Puts in all the data
        packet[6+i] = params[i];

    packet[len] = CalculateChecksum(packet);
}

/******************************************************
 * Takes in an instruction and list of params         *
 * Params needs to be in the form:                    *
 *    {ID_1, data_1, ID_2, data_2, ...}               *
 * There can be any amount of motors and in any order *
 * Note that paramlength does not include the ID      *
 ******************************************************/
void SyncWrite(unsigned char* packet, unsigned char instruction, unsigned char* params, unsigned char numparams, unsigned char paramlength, Port* port){
   
    unsigned char len = numparams + 7; //Last index of array (where checksum goes) 
    packet[0] = 0xFF;    // Heading
    packet[1] = 0xFF;
    packet[ID] = 0xFE;    // Broadcast ID
    packet[LENGTH] = len-3;   // Length of packet except for headings, ID, Checksum
    packet[INSTRUCTION] = 0x83;    // Sync Write
    packet[5] = instruction;  // What is happening at each motor
    packet[6] = paramlength;  // Number of bytes written to each motor
    for(unsigned char i = 0; i < numparams; i++)
        packet[7+i] = params[i];
    packet[len] = CalculateChecksum(packet);
    unsigned char rxpacket[MAXNUM_RXPARAM] = {0, }
    ReadWrite(packet, rxpacket);
}

/******************************************************
 * Converts 255 value rgb values into a 2 byte color  *
 * Useful for Head (0x1A) and Eye (0x1C) LEDs         *
 ******************************************************/
int MakeColor(int red, int green, int blue)
{
    int r = red & 0xFF;
    int g = green & 0xFF;
    int b = blue & 0xFF;

    return (int)(((b>>3)<<10)|((g>>3)<<5)|(r>>3));
}


// Converts an angle given in degrees into motor ticks and sends write packet to motor 
void SetJointAngle(unsigned char joint_ID, int goal_angle){
    unsigned char angle_packet[9] = {0, };
    int goal_ticks = goal_angle * 11.378; // 4096/360
    unsigned char params[2] = {GetLowByte(goal_ticks), GetHighByte(goal_ticks)}; 
    MakePacket(angle_packet, joint_ID, 2, WRITE, GOAL_POSITION_L, params);
    unsigned char rxpacket[MAXNUM_RXPARAM] = {0, }
    ReadWrite(angle_packet, rxpacket);
}

void SetMoveSpeed(unsigned char joint_ID, int move_speed){
    unsigned char speed_packet[9] = {0, };
    unsigned char params[2] = {GetLowByte(move_speed), GetHighByte(move_speed)}; 
    MakePacket(speed_packet, joint_ID, 2, WRITE, GOAL_POSITION_L, params);
    unsigned char rxpacket[MAXNUM_RXPARAM] = {0, }
    ReadWrite(speed_packet, rxpacket);
}

void Set_P_Gain(unsigned char joint_ID, unsigned char P_Value){
    unsigned char P_packet[8] = {0, };
    MakePacket(P_packet, joint_ID, 1, WRITE, P_GAIN, &P_Value); // Pass in pointer b/c it expects a char*
    unsigned char rxpacket[MAXNUM_RXPARAM] = {0, }
    ReadWrite(P_packet, rxpacket);
}

void Set_I_Gain(unsigned char joint_ID, unsigned char I_Value){
    unsigned char I_packet[8] = {0, };
    MakePacket(I_packet, joint_ID, 1, WRITE, I_GAIN, &I_Value); // Pass in pointer b/c it expects a char*
    unsigned char rxpacket[MAXNUM_RXPARAM] = {0, }
    ReadWrite(I_packet, rxpacket);
}

void Set_D_Gain(unsigned char joint_ID, unsigned char D_Value){
    unsigned char D_packet[8] = {0, };
    MakePacket(D_packet, joint_ID, 1, WRITE, D_GAIN, &D_Value); // Pass in pointer b/c it expects a char*
    unsigned char rxpacket[MAXNUM_RXPARAM] = {0, }
    ReadWrite(D_packet, rxpacket);
}

void Set_PID_Gain(unsigned char joint_ID, unsigned char P_Value, unsigned char I_Value, unsigned char D_Value){
    unsigned char PID_packet[10] = {0, };
    unsigned char params[3] = {P_Value, I_Value, D_Value};
    MakePacket(PID_packet, joint_ID, 1, WRITE, D_GAIN, params);

    unsigned char rxpacket[MAXNUM_RXPARAM] = {0, }
    ReadWrite(PID_packet, rxpacket);
}

// is_enabled is a 0 for no torque or 1 for torque
void Set_Torque_Enable(unsigned char joint_ID, unsigned char is_enabled){
    unsigned char Torque_packet[8] = {0, };
    MakePacket(Torque_packet, joint_ID, 1, WRITE, TORQUE_ENABLE, &is_enabled);
    unsigned char rxpacket[MAXNUM_RXPARAM] = {0, }
    ReadWrite(Torque_packet, rxpacket);
}








