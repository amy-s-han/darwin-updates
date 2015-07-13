
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>
#include <assert.h>
#include <sys/stat.h>

#include "darwinController.h"

#define MAXNUM_TXPARAM 	(256)
#define MAXNUM_RXPARAM 	(1024)
#define ID		        (2)
#define LENGTH         	(3)
#define INSTRUCTION 	(4)
#define ERRBIT		    (4)
#define PARAMETER	    (5)

using namespace std;

enum { ALT_NUM_JOINTS = 20 } ;

//make a playback struct~~~
struct Playback{

    bool isPlaying;
    const char* file;
    int offset_counter; //keeps track of index within playback file
    double dt;

    size_t njoints;
    size_t nticks;

    vector<double> angles_rad;

};


bool parse_file(Playback& play) {

    ifstream istr(play.file);
    if (!istr.is_open()) {
        cerr << "error opening file " << play.file << "!\n";
        return false;
    }

    string header_str;

    // parse our header
    istr >> header_str;

    if (header_str != "TRAJ") {
        cerr << "expected TRAJ header!\n";
        return false;
    }

    if (!(istr >> play.njoints >> play.nticks >> play.dt)) {
        cerr << "error parsing header!\n";
        return false;
    }

    if (play.njoints != ALT_NUM_JOINTS) {
        cerr << "incorrect # joints: got " << play.njoints << ", expected " << ALT_NUM_JOINTS << "\n";
        return false;
    }

    if (play.dt <= 0) {
        cerr << "expected dt > 0!\n";
        return false;
    }

    if (!play.nticks) {
        cerr << "refuse to read empty trajectory!\n";
        return false;
    }

    play.angles_rad.resize(play.nticks * play.njoints);

    for (size_t t = 0; t < play.nticks; ++t) {
        for (size_t i = 0; i < play.njoints; ++i) { 
            if (!(istr >> play.angles_rad[play.offset_counter])) {
                cerr << "error parsing angle!\n";
                return false;
            }
	    

            ++play.offset_counter;
        }
    }
        

    return true;

}

bool IsDone(Playback play){
    return !play.isPlaying && play.offset_counter >= play.angles_rad.size();
}

int main(int argc, char** argv){

    if (argc != 2) {
        cerr << "usage: " << argv[0] << " TRAJ_FILENAME.txt\n";
        return 1;
    }

    Playback play = {false, argv[1], 0}; //initialize first 3 vars in struct

    if (!parse_file(play)){
        cerr << "no trajectory loaded, exiting.\n";
        return 1;
    }

    DarwinController darCon = DarwinController();

    if(darCon.Initialize("/dev/ttyUSB0") == false){
        fprintf(stderr, "Failed to initialize\n");
        return 0;
    }

    // temp set speed -> seriously consider keeping this???

    unsigned char speedTxPacket[MAXNUM_TXPARAM];
    unsigned char speedParams[60]; // 3 params each for 20 motors

    for(int i = 0; i < NUM_JOINTS; i++){
        speedParams[3*i] = i+1;
        speedParams[3*i+1] = 0x40;
        speedParams[3*i+2] = 0x00;
    }

    int speed_syncwrite_result = darCon.SyncWrite(speedTxPacket, 0x20, speedParams, 60, 2);

    printf("NUM_JOINTS: %d and ALT_NUM_JOINTS: %d\n\n", NUM_JOINTS, ALT_NUM_JOINTS);

    assert( play.angles_rad.size() == play.nticks * ALT_NUM_JOINTS );

    //Set all joints to be enabled in jointData

    uint8_t enables[20];
    
    for(int i=0; i<20; i++){
      enables[i] = 1;
    }

    darCon.Set_Enables(enables);


    // reset speed or gains?


    uint8_t pgains[20] = {0, };
    uint8_t igains[20] = {0, };
    uint8_t dgains[20] = {0, };

    for(int i = 0; i < 20; i++){
      pgains[i] = 5; // what to set gains to initially???
      igains[i] = 1;
    }

    darCon.Set_P_Data(pgains);
    darCon.Set_I_Data(igains);
    darCon.Set_D_Data(dgains);

     darCon.Update_Motors();

    usleep(100000); // not sure why i want this sleep


    play.offset_counter = 0; //reset to 0!

    printf("play.angles_rad.size() = %d\n", int(play.angles_rad.size()));

    //Load first time tick into Joint Data
    uint16_t goalpos[20];

    for(int i = 0; i < ALT_NUM_JOINTS; i++){
      //printf("In for loop: %d\n", i);
      //printf("from angles_rad: %f\n", play.angles_rad[play.offset_counter]);
      
        double cur_angle = play.angles_rad[play.offset_counter];
    //printf("cur_angle in ticks: %d\n", darCon.RadAngle2Ticks(cur_angle));
        goalpos[i] = darCon.RadAngle2Ticks(cur_angle);
        play.offset_counter++;
    }

    printf("Finished loading first tick\n");

    for(int i = 0; i < ALT_NUM_JOINTS; i++){
        printf("goal %d: %d\n", i, goalpos[i]);
    }

    printf("Press enter to Set_Pos_Data\n");
    getchar();


    int poscount = darCon.Set_Pos_Data(goalpos);
    printf("poscount: how many goal positions were successfully changed: %d\n", poscount);

    // initialize to first tick
    darCon.Update_Motors();

    printf("Finished initializing. Press Enter to continue.\n");
    getchar();

    //start playing
    if(!play.angles_rad.empty()){
        play.isPlaying = true;
    }

    while(play.isPlaying){

        //put trajectory data into jointData

        for (int i=0; i<ALT_NUM_JOINTS; ++i) {

            if(play.offset_counter > play.angles_rad.size()){ //check if done
                play.isPlaying = false;
                break;
            }


            //set the m_Joint to reflect joint angles from the current time tick
            // note motor indices start at 1, so need to add i+1 for motor_number

            double cur_angle = play.angles_rad[play.offset_counter];

            goalpos[i] = darCon.RadAngle2Ticks(cur_angle);

            // do i still need this?
            if (cur_angle < -8 || cur_angle > 8) {
                // error
                printf("ERRORRRRR!");
            }

            ++play.offset_counter;
        }

        poscount = darCon.Set_Pos_Data(goalpos);

        //updateMotors to write out all changes. 
        darCon.Update_Motors();

        sleep(0.008); //sleep 8ms before next time tick -> still worried about this.

    }

    printf("Press ENTER to close port\n");
    getchar();

    darCon.ClosePort();

}
