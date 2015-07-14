
#ifndef _CM730_H_
#define _CM730_H_

enum{
	MODEL_NUMBER_L			= 0,
	MODEL_NUMBER_H			= 1,
	VERSION					= 2,
	ID						= 3,
	BAUD_RATE				= 4,
	RETURN_DELAY_TIME		= 5,
	CW_ANGLE_LIMIT_L		= 6,
	CW_ANGLE_LIMIT_H		= 7,
	CCW_ANGLE_LIMIT_L		= 8,
	CCW_ANGLE_LIMIT_H		= 9,
	TEMP_LIMIT				= 11,
	LOWEST_LIMIT_VOLTAGE	= 12,
	HIGHEST_LIMIT_VOLTAGE	= 13,
	MAX_TORQUE_L			= 14,
	MAX_TORQUE_H			= 15,
	STATUS_RETURN_LEVEL		= 16,
	ALARM_LED				= 17,
	ALARM_SHUTDOWN			= 18,
	MULTI_TURN_OFFSET_L		= 20,
	MULTI_TURN_OFFSET_H		= 21,
	RESOLUTION_DIVIDER		= 22,
	TORQUE_ENABLE			= 24,
	LED						= 25,
	D_GAIN					= 26,
	I_GAIN					= 27,
	P_GAIN					= 28,
	GOAL_POSITION_L			= 30,
	GOAL_POSITION_H			= 31,
	MOVING_SPEED_L			= 32,
	MOVING_SPEED_H			= 33,
	TORQUE_LIMIT_L			= 34,
	TORQUE_LIMIT_H			= 35,
	PRESENT_POSITION_L		= 36,
	PRESENT_POSITION_H		= 37,
	PRESENT_SPEED_L			= 38,
	PRESENT_SPEED_H			= 39,
	PRESENT_LOAD_L			= 40,
	PRESENT_LOAD_H			= 41,
	PRESENT_VOLTAGE			= 42,
	PRESENT_TEMPERATURE		= 43,
	REGISTERED				= 44,
	MOVING					= 46,
	LOCK					= 47,
	PUNCH_L					= 48,
	PUNCH_H					= 49,
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
	L_HIP_PITCH	   		= 12,
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
	PING			=1,
	READ			=2,
	WRITE			=3,
	REG_WRITE		=4,
	ACTION			=5,
	RESET			=6,
	SYNC_WRITE		=131,
	BULK_READ		=146,
};

enum{		
	RETURN_LEVEL		= 16,
	DXL_POWER			= 24,
	LED_PANNEL			= 25,
	LED_HEAD_L			= 26,
	LED_HEAD_H			= 27,
	LED_EYE_L			= 28,
	LED_EYE_H			= 29,
	BUTTON				= 30,			
	GYRO_Z_L			= 38,
	GYRO_Z_H			= 39,
	GYRO_Y_L			= 40,
	GYRO_Y_H			= 41,
	GYRO_X_L			= 42,
	GYRO_X_H			= 43,
	ACCEL_X_L			= 44,
	ACCEL_X_H			= 45,
	ACCEL_Y_L			= 46,
	ACCEL_Y_H			= 47,
	ACCEL_Z_L			= 48,
	ACCEL_Z_H			= 49,
	VOLTAGE				= 50,
	LEFT_MIC_L			= 51,
	LEFT_MIC_H			= 52,
	ADC2_L				= 53,
	ADC2_H				= 54,
	ADC3_L				= 55,
	ADC3_H				= 56,
	ADC4_L				= 57,
	ADC4_H				= 58,
	ADC5_L				= 59,
	ADC5_H				= 60,
	ADC6_L				= 61,
	ADC6_H				= 62,
	ADC7_L				= 63,
	ADC7_H				= 64,
	ADC8_L				= 65,
	ADC8_H				= 66,
	RIGHT_MIC_L			= 67,
	RIGHT_MIC_H			= 68,
	ADC10_L				= 69,
	ADC10_H				= 70,
	ADC11_L				= 71,
	ADC11_H				= 72,
	ADC12_L				= 73,
	ADC12_H				= 74,
	ADC13_L				= 75,
	ADC13_H				= 76,
	ADC14_L				= 77,
	ADC14_H				= 78,
	ADC15_L				= 79,
	ADC15_H				= 80,
	MAX_NUM_ADDRESS	
};

enum{
	ID_CM				= 200,
	ID_BROADCAST		= 254
};


enum {
    FLAG_GOAL_CHANGED = 0x01,
    FLAG_GAINS_CHANGED = 0x02,
    FLAG_ENABLE = 0x80,
};


class Port {
	
	private:
		int m_Socket_fd;
		//char m_PortName[20]; 
		double m_ByteTransferTime;		

	public:
		Port();
		~Port();
		bool OpenPort(const char* name);
		void ClosePort();
		void ClearPort();
		int WritePort(unsigned char* packet, int numPacket);
		int ReadPort(unsigned char* packet, int numPacket);

};


#endif