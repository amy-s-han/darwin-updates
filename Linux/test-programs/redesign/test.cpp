
#include <stdio.h>
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


    darCon->MakeBulkPacket(darCon->BulkReadTxPacket);

	unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0, };
	unsigned char txpacketread[] = {0, 0, 0x14, 0x04, 0x02, 0x24, 0x02, 0};

	darCon->FinishPacket(txpacketread);

    int result = darCon->ReadWrite(txpacketread, rxpacket);

	printf("result: %d\n", result);

    int word = darCon->MakeWord((int)rxpacket[PARAMETER], (int)rxpacket[PARAMETER + 1]);
    printf("Motor position: %d\n\n", word);

	printf("Press ENTER to close port\n");
	getchar();

	darCon->ClosePort();

	delete(darCon);

}
