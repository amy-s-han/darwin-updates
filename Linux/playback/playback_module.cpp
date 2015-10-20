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

using namespace Robot; 

enum { NUM_JOINTS = 20 } ;

Playback::Playback(const char* filename){
    file = filename; 
    isPlaying = false;
}
		
Playback::~Playback(){
    //destructor
}

void Playback::Initialize(){

    isPlaying = false;
  	offset_counter = 0; //reset offset to 0

  	//set all joints to be enabled
  	m_Joint.SetEnableBody(true);

  	MotionManager::GetInstance()->SetEnable(true);
      	for (int i=1; i<=20; i++){ //remember that motor indexing starts at 1
          	m_Joint.SetEnable(i,true,true);
          	
            //setting motor angles for the first tick
            m_Joint.SetRadian(i, angles_rad[i-1]); 
      	}

}

void Playback::Start() {
    if (!angles_rad.empty()) {
        isPlaying = true;
    }
}

void Playback::Process(){

    if (!isPlaying) { return; }

  	// put trajectory data into motors
  	for (int i=0; i<NUM_JOINTS; ++i) {

    	  if(offset_counter>angles_rad.size()){ //check if done
            isPlaying = false;
      	    return;
    	  }


        //set the m_Joint to reflect joint angles from the current tick
    	  // note motor indices start at 1, so need to add i+1 for motor_number
    	  double cur_angle = angles_rad[offset_counter];
    	  m_Joint.SetRadian(i+1, cur_angle);
    	  if (cur_angle < -8 || cur_angle > 8) {
      	  	// error
      	  	printf("ERRORRRRR!");
    	  }
    	  ++offset_counter;
  	}

}

bool Playback::parse_file() {

    std::ifstream istr(file);
    if (!istr.is_open()) {
        std::cerr << "error opening file " << file << "!\n";
        return false;
    }

    std::string header_str;

    // parse our header
    istr >> header_str;

    if (header_str != "TRAJ") {
        std::cerr << "expected TRAJ header!\n";
        return false;
    }

    if (!(istr >> njoints >> nticks >> dt)) {
        std::cerr << "error parsing header!\n";
        return false;
    }

    if (njoints != NUM_JOINTS) {
        std::cerr << "incorrect # joints: got " << njoints << ", expected " << NUM_JOINTS << "\n";
        return false;
    }

    if (dt <= 0) {
        std::cerr << "expected dt > 0!\n";
        return false;
    }

    if (!nticks) {
        std::cerr << "refuse to read empty trajectory!\n";
        return false;
    }

    angles_rad.resize(nticks * njoints);

    for (size_t t=0; t<nticks; ++t) {
        for (size_t i=0; i<njoints; ++i) { 
            if (!(istr >> angles_rad[offset_counter])) {
                std::cerr << "error parsing angle!\n";
                return false;
            }
            ++offset_counter;
        }
    }
    
    return true;

}

bool Playback::IsDone(){
    return !isPlaying && offset_counter >= angles_rad.size();
}
