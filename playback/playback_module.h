/* 
 * playback_module.h
 *
 */


#ifndef _PLAYBACK_MODULE_H
#define _PLAYBACK_MODULE_H

#include <string.h>

#include "MotionModule.h"

namespace Robot {

	class Playback : public MotionModule {
	
	private:
		bool isPlaying;
		const char* file;   //file name
		int offset_counter; //keeps track of index within playback file
		double dt;

	public: 
		
		size_t njoints;
		size_t nticks;

		std::vector<double> angles_rad;

		Playback(const char* filename);
		~Playback();

		void Initialize();
		void Process();
		bool parse_file();
		bool IsDone();
		void Start();

	};

}

#endif /*_PLAYBACK_MODULE_H*/
