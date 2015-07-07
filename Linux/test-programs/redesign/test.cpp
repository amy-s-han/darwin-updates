
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
   
	DarwinController* darCon = new DarwinController();

	if(darCon->Initialize("/dev/ttyUSB0") == false){
		fprintf(stderr, "Failed to initialize\n");
		return 0;
	}

	usleep(5000);

	printf("~~~ Testing MakeBulkPacket ~~~\n");
    darCon->MakeBulkPacket(darCon->BulkReadTxPacket);

    printf("~~~ Testing Read ~~~\n");
	unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0, };
	unsigned char txpacketread[] = {0, 0, 0x14, 0x04, 0x02, 0x24, 0x02, 0};

	darCon->FinishPacket(txpacketread);

    int result = darCon->ReadWrite(txpacketread, rxpacket);
    int word;

	if(result == 0){
		printf("Failed read! \n");
	} else {
		word = darCon->MakeWord((int)rxpacket[PARAMETER], (int)rxpacket[PARAMETER + 1]);
    	printf("Successful read. Motor position: %d\n\n", word);

	}

	printf("~~~ Testing Write ~~~\n");

	
	while(result == 0){
		//usleep(5000);
		result = darCon->ReadWrite(txpacketread, rxpacket);
	}
	
	word += 200;
    int value_low = darCon->GetLowByte(word);
    int value_high = darCon->GetHighByte(word);

	unsigned char txpacketwrite[] = {0, 0, 0x05, 0x05, 0x03, 0x1E, 0, 0, 0};
	txpacketwrite[PARAMETER+1] = value_low;
	txpacketwrite[PARAMETER+2] = value_high;

	darCon->FinishPacket(txpacketwrite);

	int result2 = darCon->ReadWrite(txpacketwrite, rxpacket);
	printf("result2: %d\n", result2);

    
	printf("Press ENTER to close port\n");
	getchar();

	darCon->ClosePort();

	delete(darCon);

}
