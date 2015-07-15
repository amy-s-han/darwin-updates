
#include <stdio.h>
#include <unistd.h>
#include "darwinController.h"

#define MAXNUM_TXPARAM 	(256)
#define MAXNUM_RXPARAM 	(1024)
#define ID		        (2)
#define LENGTH         	(3)
#define INSTRUCTION 	(4)
#define ERRBIT		    (4)
#define PARAMETER	    (5)

int main(int argc, char** argv){
   
	DarwinController darCon = DarwinController();

	if(darCon.Initialize("/dev/ttyUSB0") == false){
		fprintf(stderr, "Failed to initialize\n");
		return 0;
	}


	printf("\n~~~ Testing MakeBulkPacket ~~~\n");
    darCon.MakeBulkPacket(darCon.BulkReadTxPacket);

    
	printf("\n~~~ Testing InitToPose ~~~\n");
	darCon.InitToPose();

	printf("Press ENTER to continue testing.\n");
	getchar();
	

    printf("\n~~~ Testing Read ~~~\n");
	unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0, };
	unsigned char txpacketread[] = {0, 0, 0x14, 0x04, 0x02, 0x24, 0x02, 0};

	darCon.FinishPacket(txpacketread);

    int result = darCon.ReadWrite(txpacketread, rxpacket);
    int word = 0;

	if(result == 0){
		printf("Failed read! \n");
	} else {
		word = darCon.MakeWord((int)rxpacket[PARAMETER], (int)rxpacket[PARAMETER + 1]);
    	printf("Successful read. Motor position: %d\n\n", word);

	}

	printf("Press ENTER to continue testing.\n");
	getchar();

	printf("\n~~~ Testing Write ~~~\n");

	
	while(result == 0){
		result = darCon.ReadWrite(txpacketread, rxpacket);
	}

	if(word == 0){
		word = darCon.MakeWord((int)rxpacket[PARAMETER], (int)rxpacket[PARAMETER + 1]);
	}
	
	word += 200;
    int value_low = darCon.GetLowByte(word);
    int value_high = darCon.GetHighByte(word);

	unsigned char txpacketwrite[] = {0, 0, 0x14, 0x05, 0x03, 0x1E, 0, 0, 0};
	txpacketwrite[PARAMETER+1] = value_low;
	txpacketwrite[PARAMETER+2] = value_high;

	darCon.FinishPacket(txpacketwrite);

	int result2 = darCon.ReadWrite(txpacketwrite, rxpacket);
	if(result2 == 0){
		printf("failed write!\n");
	} else {
		printf("Successful write. result2: %d\n", result2);
	}

	printf("Press ENTER to continue testing.\n");
	getchar();

	printf("\n~~~ Testing BulkRead ~~~\n");
	int result3 = darCon.BulkRead(rxpacket);

	printf("result3: %d\n", result3);

	
	printf("Press ENTER to close port\n");
	getchar();

	darCon.ClosePort();


}
