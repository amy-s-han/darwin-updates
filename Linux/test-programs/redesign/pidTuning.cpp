// very crappy PID Tuning helper

#include <iostream>
#include <sstream>
#include <fstream>

#include <assert.h>
#include <unistd.h>

#include "darwinController.h"

using namespace std;

int input_validation(){
    int resp;
    bool notdone = true;
    
    while(notdone){
    
        cin >> resp;
    
        if(resp >= 0 && resp <= 254){
            return resp;
        } else {
            printf("Gain has to be between 0 ~ 254. Try again.\n");
        }
    }
}


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

    sleep(1);

    //Set all joints to be enabled in jointData
    uint8_t enables[20];
    
    for(int i = 0; i < 20; i++){
      enables[i] = 1;
    }

    darCon.Set_Enables(enables);

    // set gains for now

    uint8_t pgains[20] = {0, };
    uint8_t igains[20] = {0, };
    uint8_t dgains[20] = {0, };

    for(int i = 0; i < 20; i++){
      pgains[i] = 32;
    }

    darCon.Set_P_Data(pgains);
    darCon.Set_I_Data(igains);
    darCon.Set_D_Data(dgains);

    darCon.Update_Motors();

    int p = 32;
    int i = 0;
    int d = 0;

    printf("Welcome to PID tuning! Press Enter to continue\n");
    getchar();

    bool notDone = true;

    int response;
    string ans;

    while(notDone){

	    printf("PID values are: P: %d, I: %d, D: %d.\n", p, i, d);

	    printf("PID values are ints between 0 ~ 254\n");

        while(1){
            printf("Run again with current values? y/n\n");
            cin >> ans;

            if(ans == "y" || "n"){
                break;
            }
        }   
        
        if(ans == "y"){
            printf("Enter int P value: \n");
            p = input_validation();
    
            printf("Enter int I value: \n");
            i = input_validation();
    
            printf("Enter int D value: \n");
            d = input_validation();
    
    
            for(int i = 0; i < 20; i++){
                pgains[i] = p; // what to set gains to initially???
                igains[i] = i;
                dgains[i] = d;
            }
        }

	    darCon.Set_P_Data(pgains);
	    darCon.Set_I_Data(igains);
	    darCon.Set_D_Data(dgains);

	    darCon.Update_Motors();

        sleep(0.5);

	    printf("Motors updated. Press Enter to raise arm.\n");
	    getchar();

	    darCon.Set_Pos_Data(R_ELBOW, 2248);
	    darCon.Update_Motors();

        sleep(1);

        int lift = darCon.ReadJointAngle(R_ELBOW);

	    printf("Press Enter to lower arm.\n");
	    getchar();

	    darCon.Set_Pos_Data(5, 2048);
	    darCon.Update_Motors();

        sleep(1);

        int lower = darCon.ReadJointAngle(R_ELBOW);
	    
        printf("lift Goal: 2248. Actual: %d. Diff: %d\n", lift, 2248-lift);
        printf("lower Goal: 2048. Actual: %d. Diff: %d\n", lower, lower - 2048);

	    printf("Please enter 1 to quit, 0 to continue.\n");

        while(1){
            cin >> response;
            if(response == 0 || response == 1){
                break;
            } else {
                printf("Please enter 1 to quit, 0 to continue.\n");
            }
        }

	    //darCon.InitToPose();

	    if(response == 1){
            printf("Resp = 1\n");
	    	notDone = false;
	    	break;
	    }

	}

	printf("Final PID values are: P: %d, I: %d, D: %d.\n", p, i, d);

	printf("Press ENTER to close port\n");
    getchar();

    darCon.ClosePort();




}
