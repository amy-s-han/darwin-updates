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

	// where to put this class? can define in playback_module
	class SimpleTrajectory {
		public:

		double dt;
		size_t njoints;
		size_t nticks;

		std::vector<double> angles_rad;

		SimpleTrajectory();

	};

	class Playback : public MotionModule {

	public: 
		bool isDone;
		SimpleTrajectory traj;
		const char* file; // check on pointer to string
		int offset_counter; //keeps track of index within playback file

		Playback(const char* filename);
		~Playback();

		void Initialize();
		void Process();
		bool parse_file();
		// probably don't need filename if filename is now a public var



		// Maybe we want these?
		// void LoadINISettings(minIni* ini);
  		// void LoadINISettings(minIni* ini, const std::string &section);
  		// void SaveINISettings(minIni* ini);
  		// void SaveINISettings(minIni* ini, const std::string &section);


	};

}

#endif /*_PLAYBACK_MODULE_H*/
