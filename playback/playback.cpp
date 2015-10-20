#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>
#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <assert.h>
#include <sys/stat.h>

#include "Camera.h"
#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"
#include "playback_module.h"

using namespace std;

/* Initialization stuff */

int motion_initialization(MotionModule* playback){
  //////////////////// Framework Initialize ////////////////////////////
    LinuxCM730* linux_cm730 = new LinuxCM730("/dev/ttyUSB0");
    CM730* cm730 = new CM730(linux_cm730);
    if(MotionManager::GetInstance()->Initialize(cm730) == false){
        printf("Fail to initialize Motion Manager!\n");
        return -1;
    }

    MotionManager::GetInstance()->AddModule((MotionModule*)playback);	

    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();


    // how to get smooth initialization. 
    // initialize the CM730, initialize the MotionManager, add the playback object
    // to the motion manager, and don't start the motion timer until after
    // this is called in the Playback object to reflect the 
    // very first tick of the trajectory so that the JointData is accurately
    // reflecting goal position


    // this is an offset into the params below
    int n = 0;

    // this is what gets written to the CM730 eventually, 5 ints per joint
    int param[JointData::NUMBER_OF_JOINTS * 5];

    // temporary variables per joint
    int wGoalPosition, wStartPosition, wDistance;

      // for each joint
    for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
    {

        // get current value (rotation encoder ticks)
        wStartPosition = MotionStatus::m_CurrentJoints.GetValue(id);

        // get initial value from motion module
        wGoalPosition = playback->m_Joint.GetValue(id);

        printf("joint %d start is %d, goal is %d\n", id, wStartPosition, wGoalPosition);

        // get the absolute value of the distance between start & goal (in ticks)
        if( wStartPosition > wGoalPosition )
            wDistance = wStartPosition - wGoalPosition;
        else
            wDistance = wGoalPosition - wStartPosition;

        // divide distance by 4 - this is totally dumb because wDistance /= 4 would read so much better
        wDistance >>= 2;

        // enforce a min distance of 8
        if( wDistance < 8 )
            wDistance = 8;

        // first int among params is the joint id
        param[n++] = id;

        // next two ints among params are the low and high bytes of the goal position
        param[n++] = CM730::GetLowByte(wGoalPosition);
        param[n++] = CM730::GetHighByte(wGoalPosition);

        // next two ints among params are the low and high bytes of the distance
        // which I beleive is not actually used as a distance here, but rather
        // as a gain or speed control (there is an implicit "per second" going on here
        // , I have a hunch)
        param[n++] = CM730::GetLowByte(wDistance);
        param[n++] = CM730::GetHighByte(wDistance);
    }

    // communicate directly with the CM730 board to use presumably 
    // the MX28::P_GOAL_POSITION_L command, 
    // my guess is that 5 is the # params per packet
    // my guess is that NUMBER_OF_JOINTS-1 is the # packets
    // very last thing is ptr to param packet data
    cm730->SyncWrite(MX28::P_GOAL_POSITION_L, 5, JointData::NUMBER_OF_JOINTS - 1, param);  

      
    printf("Press the ENTER key to begin!\n");
    getchar();

    MotionManager::GetInstance()->SetEnable(true); // Sets motor speeds to 0. Crucial after syncwrite


    /*
    struct stat sb;

    // not sure where to put this initialization stuff? probably just keep it here
    if (stat("offsets.ini", &sb) == 0 && S_ISREG(sb.st_mode)) {
      minIni* ini = new minIni("offsets.ini");
      MotionManager::GetInstance()->LoadINISettings(ini);
      printf("parsed offsets.ini!\n");
    }
    */

    return 0;

}

enum { NUM_JOINTS = 20 } ;

int main(int argc, char** argv){
    Playback* playback = new Playback(argv[1]);

    if (argc != 2) {
        std::cerr << "usage: " << argv[0] << " TRAJ_FILENAME.txt\n";
        return 1;
    }

    if (!playback->parse_file()) {
        std::cerr << "no trajectory loaded, exiting.\n";
        return 1;
    }

    assert( playback->angles_rad.size() == playback->nticks * NUM_JOINTS );

    std::cout << "loaded " << argv[1] << "\n";

    if (motion_initialization(playback)){
        printf("Failed to initialize control!\n");
        return -1;
    }

    CM730::MakeBulkReadPacket();
    CM730::BulkRead();

    playback->Start();

    while (!playback->IsDone()) {
        usleep(100000);
    }

    return 0;

}
