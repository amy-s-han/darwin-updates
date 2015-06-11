/* 
 * playback_module.cpp
 *
 */

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

//???????????????????????? use robot namespace or std namespace?
using namespace Robot; 

Playback* Playback::m_UniqueInstance = new Playback(std::string &filename);

Playback::Playback(std::string &filename){
	//constructor stuff
}
		
~Playback(){
	//destructor
}

void Playback::Initialize(){
	offset_counter = 0; //reset offset to 0
	//set all joints to be enabled
	m_Joint.SetEnableBody(true);

	MotionManager::GetInstance()->SetEnable(true);
  	for (int i=1; i<=20; i++){
    	Playback::GetInstance()->m_Joint.SetEnable(i,true,true);
    	Playback::GetInstance()->m_Joint.SetAngle(i,0);
  	}

}

void Playback::Process(){
	//set the m_Joint to reflect joint angles from the current tick
	//if current tick is not the last one, increment the current tick

	// here is head.cpp's process():
	// if(m_Joint.GetEnable(JointData::ID_HEAD_PAN) == true)
	// 	m_Joint.SetAngle(JointData::ID_HEAD_PAN, m_PanAngle);

	// if(m_Joint.GetEnable(JointData::ID_HEAD_TILT) == true)
	// 	m_Joint.SetAngle(JointData::ID_HEAD_TILT, m_TiltAngle);

}

bool parse_file(const char* filename, SimpleTrajectory& traj) {

  std::ifstream istr(filename);
  if (!istr.is_open()) {
    std::cerr << "error opening file " << filename << "!\n";
    return false;
  }

  std::string header_str;

  // parse our header
  istr >> header_str;

  if (header_str != "TRAJ") {
    std::cerr << "expected TRAJ header!\n";
    return false;
  }

  if (!(istr >> traj.njoints >> traj.nticks >> traj.dt)) {
    std::cerr << "error parsing header!\n";
    return false;
  }

  if (traj.njoints != NUM_JOINTS) {
    std::cerr << "incorrect # joints: got " << traj.njoints << ", expected " << NUM_JOINTS << "\n";
    return false;
  }

  if (traj.dt <= 0) {
    std::cerr << "expected dt > 0!\n";
    return false;
  }

  if (!traj.nticks) {
    std::cerr << "refuse to read empty trajectory!\n";
    return false;
  }

  size_t offset = 0;

  traj.angles_rad.resize(traj.nticks * traj.njoints);

  for (size_t t=0; t<traj.nticks; ++t) {
    for (size_t i=0; i<traj.njoints; ++i) { 
      if (!(istr >> traj.angles_rad[offset])) {
	std::cerr << "error parsing angle!\n";
	return false;
      }
      ++offset;
    }
  }
  
  return true;

}


// Maybe we want these?
// void LoadINISettings(minIni* ini);
// void LoadINISettings(minIni* ini, const std::string &section);
// void SaveINISettings(minIni* ini);
// void SaveINISettings(minIni* ini, const std::string &section);



