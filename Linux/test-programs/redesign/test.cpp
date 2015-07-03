
#include <stdio.h>
#include "darwinController.h"



int main(int argc, char** argv){
   
	DarwinController* darCon = new DarwinController();

	if(darCon->Initialize("/dev/ttyUSB0") == false){
		fprintf(stderr, "Failed to initialize\n");
		return 0;
	}


	printf("Press ENTER to close port\n");
	getchar();

	darCon->ClosePort();

}
