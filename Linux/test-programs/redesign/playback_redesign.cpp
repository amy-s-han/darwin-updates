// playback_redesign 

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

//make a playback struct~~~
struct Playback{

    bool isPlaying;
    const char* file; //filename
    int offset_counter; //keeps track of index within playback file
    double dt;

    size_t njoints;
    size_t nticks;

    vector<double> angles_rad;

    double PeriodSec;

};

// parses the trajectory txt file into a buffer
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

    if (play.njoints != NUM_JOINTS) {
        cerr << "incorrect # joints: got " << play.njoints << ", expected " << NUM_JOINTS << "\n";
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

    if (argc != 2) { // check command line arguments
        cerr << "usage: " << argv[0] << " TRAJ_FILENAME.txt\n";
        return 1;
    }

    Playback play = {false, argv[1], 0}; //initialize first 3 vars in struct

    play.PeriodSec = 0.008; // In seconds

    if (!parse_file(play)){ // parse the file
        cerr << "no trajectory loaded, exiting.\n";
        return 1;
    }

    DarwinController darCon = DarwinController();

    if(darCon.Initialize("/dev/ttyUSB0") == false){
        fprintf(stderr, "Failed to initialize\n");
        return 0;
    }

    // set speed 
    if (!darCon.SetAllJointSpeeds(0x00, 0x40)){
        cerr << "Failed to set speed\n";
        return 1;
    }

    assert( play.angles_rad.size() == play.nticks * NUM_JOINTS );

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

    play.offset_counter = 0; //reset to 0!

    printf("play.angles_rad.size() = %d\n", int(play.angles_rad.size()));

    //Load first time tick into Joint Data
    uint16_t goalpos[20];

    for(int i = 0; i < NUM_JOINTS; i++){

        double cur_angle = play.angles_rad[play.offset_counter];

        goalpos[i] = darCon.RadAngle2Ticks(cur_angle);
        play.offset_counter++;
    }

    int poscount = darCon.Set_Pos_Data(goalpos);

    // initialize to first tick
    darCon.Update_Motors();

    darCon.port.DrainPort(); // Is it good to keep this?

    printf("Press Enter to start playback\n");
    getchar();

    if(!darCon.SetAllJointSpeeds(0x00, 0x00)){ 
        printf("Could not reset speed to 0x00.\n");
	return 1;
    }

    darCon.Update_Motors(); // write out new speed

    //start playing
    if(!play.angles_rad.empty()){
        play.isPlaying = true;
    }
    
    double TimePassed;

    // array to store all the times. 
    double times[play.nticks+play.njoints];

    // Implement timing struct for clock_nanosleep
    struct timespec LoopTime;
    clock_gettime(CLOCK_MONOTONIC, &LoopTime);

    while(play.isPlaying){

        //put trajectory data into jointData
        for (int i=0; i<NUM_JOINTS; ++i) {

            if(play.offset_counter > play.angles_rad.size()){ //check if done
                play.isPlaying = false;
                break;
            }

            //set jointData to reflect joint angles from the current time tick
           
            double cur_angle = play.angles_rad[play.offset_counter];

            goalpos[i] = darCon.RadAngle2Ticks(cur_angle);


            if (cur_angle < -8 || cur_angle > 8) {
                printf("ERRORRRRR!");
            }

            ++play.offset_counter;
        }

        poscount = darCon.Set_Pos_Data(goalpos);

        // write out all changes. 
        darCon.Update_Motors();

        darCon.Time.IncrementTime(&LoopTime, play.PeriodSec);
        
        //breaks when the correct amount of time has passed
        while(!darCon.Time.LoopTimeControl(&LoopTime)); 

    }

    printf("Press ENTER to close port\n");
    getchar();

    darCon.ClosePort();
}
