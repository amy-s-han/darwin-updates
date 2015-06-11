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

#include "Body.h"
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



// Maybe we want these?
// void LoadINISettings(minIni* ini);
// void LoadINISettings(minIni* ini, const std::string &section);
// void SaveINISettings(minIni* ini);
// void SaveINISettings(minIni* ini, const std::string &section);



