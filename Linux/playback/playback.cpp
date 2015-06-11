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

int motion_initialization(){
  //////////////////// Framework Initialize ////////////////////////////
  LinuxCM730* linux_cm730 = new LinuxCM730("/dev/ttyUSB0");
  CM730* cm730 = new CM730(linux_cm730);
  if(MotionManager::GetInstance()->Initialize(cm730) == false)
  {
    printf("Fail to initialize Motion Manager!\n");
    return -1;
  }
  MotionManager::GetInstance()->AddModule((MotionModule*)Playback::GetInstance());	


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

// where to put this class? can define in playback_module
class SimpleTrajectory {
public:

  double dt;
  size_t njoints;
  size_t nticks;

  std::vector<double> angles_rad;

};

enum { NUM_JOINTS = 20 } ;


int main(int argc, char** argv)
{

  SimpleTrajectory traj;

  if (argc != 2) {
    std::cerr << "usage: " << argv[0] << " TRAJ_FILENAME.txt\n";
    return 1;
  }

  if (!Playback::GetInstance()->parse_file(argv[1], traj)) {
    std::cerr << "no trajectory loaded, exiting.\n";
    return 1;
  }

  std::cout << "loaded " << argv[1] << "\n";

  if (motion_initialization()){
    printf("Failed to initialize control!\n");
    return -1;
  }

  // this is frightening but I think it needs to wait for motion initialization to do stuff
  sleep(1);

  // convert delta t from trajectory to microseconds
  size_t dt_usec = size_t(traj.dt * 1e6);

  // the offset of the current tick within trajectory data
  size_t offset = 0;

  assert( traj.angles_rad.size() == traj.nticks * NUM_JOINTS );

  // loop over trajectory
  for (size_t t=0; t<traj.nticks; ++t) {

    // put trajectory data into motors
    for (int i=0; i<NUM_JOINTS; ++i) {
      // note motor indices start at 1, so need to add i+1 for motor_number
      //Body::GetInstance()->m_Joint.SetRadian(i+1, traj.angles_rad[offset]);
      ++offset;
    }

    usleep(dt_usec);

  }

  return 0;

}
