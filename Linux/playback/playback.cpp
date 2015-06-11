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
  if(MotionManager::GetInstance()->Initialize(cm730) == false)
  {
    printf("Fail to initialize Motion Manager!\n");
    return -1;
  }

  MotionManager::GetInstance()->AddModule((MotionModule*)playback);	


  LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
  motion_timer->Start();

  struct stat sb;

  // not sure where to put this initialization stuff? probably just keep it here
  if (stat("offsets.ini", &sb) == 0 && S_ISREG(sb.st_mode)) {
    minIni* ini = new minIni("offsets.ini");
    MotionManager::GetInstance()->LoadINISettings(ini);
    printf("parsed offsets.ini!\n");
  }

  return 0;

}

enum { NUM_JOINTS = 20 } ;


int main(int argc, char** argv)
{
  Playback* playback = new Playback(argv[1]);

  if (argc != 2) {
    std::cerr << "usage: " << argv[0] << " TRAJ_FILENAME.txt\n";
    return 1;
  }

  if (!playback->parse_file()) {
    std::cerr << "no trajectory loaded, exiting.\n";
    return 1;
  }

  std::cout << "loaded " << argv[1] << "\n";

  if (motion_initialization(playback)){
    printf("Failed to initialize control!\n");
    return -1;
  }

  // this is frightening but I think it needs to wait for motion initialization to do stuff
  sleep(1);

  // convert delta t from trajectory to microseconds
  size_t dt_usec = size_t(playback->traj.dt * 1e6);

  // the offset of the current tick within trajectory data
  size_t offset = 0;

  assert( playback.traj.angles_rad.size() == playback.traj.nticks * NUM_JOINTS );


  return 0;

}
