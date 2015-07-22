// very crappy PID Tuning helper

#include <iostream>
#include <sstream>
#include <fstream>

#include <assert.h>

#include "darwinController.h"

using namespace std;



int main(int argc, char** argv){


    DarwinController darCon = DarwinController();

    if(darCon.Initialize("/dev/ttyUSB0") == false){
        fprintf(stderr, "Failed to initialize\n");
        return 0;
    }

    if(!darCon.InitToPose()){
    	fprintf(stderr, "Failed to initialize pose\n");
    	return 0;
    }

    //Set all joints to be enabled in jointData
    uint8_t enables[20];
    
    for(int i=0; i<20; i++){
      enables[i] = 1;
    }

    darCon.Set_Enables(enables);

    // set gains for now

    uint8_t pgains[20] = {0, };
    uint8_t igains[20] = {0, };
    uint8_t dgains[20] = {0, };

    // default gains: P: 32, I: 0, D: 0. 
    for(int i = 0; i < 20; i++){
        pgains[i] = 32; // what to set gains to initially???
        igains[i] = 0;
    }

    darCon.Set_P_Data(pgains);
    darCon.Set_I_Data(igains);
    darCon.Set_D_Data(dgains);

    darCon.Update_Motors();

    int p = 0;
    int i = 0;
    int d = 0;

    printf("Welcome to PID tuning! Press Enter to continue\n");
    getchar();

    bool notDone = true;

    int response;

    while(notDone){

	    printf("PID values are: P: %d, I: %d, D: %d.\n", p, i, d);

	    printf("PID values are ints between 0 ~ 254\n");

	    cout << "Enter int P value: " << endl;
	    cin >> p;

	    cout << "Enter int I value: " << endl;
	    cin >> i;

	    cout << "Enter int D value: " << endl;
	    cin >> d;


	    for(int i = 0; i < 0; i++){
	        pgains[i] = 32; // what to set gains to initially???
	        igains[i] = 0;
	    }

	    darCon.Set_P_Data(pgains);
	    darCon.Set_I_Data(igains);
	    darCon.Set_D_Data(dgains);

	    darCon.Update_Motors();

	    printf("Motors updated. Press Enter.\n");
	    getchar();

	    darCon.Set_Pos_Data(5, 2248);
	    darCon.Update_Motors();

	    printf("Press Enter\n");
	    getchar();

	    darCon.Set_Pos_Data(5, 2048);
	    darCon.Update_Motors();




	    cout << "If done, enter 1, else enter 0." << endl;
	    cin >> response;

	    if(response == 1){
	    	notDone = false;
	    	break;
	    }

	}

	printf("Final PID values are: P: %d, I: %d, D: %d.\n", p, i, d);

	printf("Press ENTER to close port\n");
    getchar();

    darCon.ClosePort();




}