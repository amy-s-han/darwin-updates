/* 
 * playback_module.h
 *
 */


#ifndef _PLAYBACK_MODULE_H
#define _PLAYBACK_MODULE_H

#include <string.h>

#include "MotionModule.h"

//????????????????????????
namespace Robot {

	class Playback : public MotionModule {
	private:
		static Playback * m_UniqueInstance;
		bool parse_file(const char* filename, SimpleTrajectory& traj);

	public: 
		static Playback* GetInstance(){ return m_UniqueInstance; }

		// constructor ??? 
		Playback(std::string &filname);

		int offset_counter; //keeps track of index within playback file

		~Playback();

		void Initialize();
		void Process();



		// Maybe we want these?
		// void LoadINISettings(minIni* ini);
  		// void LoadINISettings(minIni* ini, const std::string &section);
  		// void SaveINISettings(minIni* ini);
  		// void SaveINISettings(minIni* ini, const std::string &section);


	};
}

#endif /*_PLAYBACK_MODULE_H*/
