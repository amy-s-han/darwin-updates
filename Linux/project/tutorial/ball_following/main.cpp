/*
 * main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>
#include "Camera.h"
#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"

#define INI_FILE_PATH       "../../../../Data/config.ini"
#define U2D_DEV_NAME        "/dev/ttyUSB0"

using namespace Robot;


void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

int main(void)
{
    printf( "\n===== Ball following Tutorial for DARwIn =====\n\n");

    change_current_dir();

    Image* rgb_ball = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);

    minIni* ini = new minIni(INI_FILE_PATH);

    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCamera::GetInstance()->LoadINISettings(ini);

    mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

    ColorFinder* ball_finder = new ColorFinder();
    ball_finder->LoadINISettings(ini);
    httpd::ball_finder = ball_finder;

    BallTracker tracker = BallTracker();
    BallFollower follower = BallFollower();
	follower.DEBUG_PRINT = true;

	//////////////////// Framework Initialize ////////////////////////////
	LinuxCM730 linux_cm730(U2D_DEV_NAME);
	CM730 cm730(&linux_cm730);
	if(MotionManager::GetInstance()->Initialize(&cm730) == false)
	{
		printf("Fail to initialize Motion Manager!\n");
			return 0;
	}
    MotionManager::GetInstance()->LoadINISettings(ini);
    Walking::GetInstance()->LoadINISettings(ini);

	MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
	MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());
    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();
	/////////////////////////////////////////////////////////////////////

	// how to get smooth initialization. 
    // initialize the CM730, initialize the MotionManager, add the playback object
    // to the motion manager, and don't start the motion timer until after
    // this is called

    // this is an offset into the params below
    int n = 0;

    // this is what gets written to the CM730 eventually, 5 ints per joint
    int param[JointData::NUMBER_OF_JOINTS * 5];

    // temporary variables per jointin the Playback object to reflect the 
    // very first tick of the trajectory so that the JointData is accurately
    // reflecting goal position
    int wGoalPosition, wStartPosition, wDistance;

    // for each joint
    for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
    {

        // get current value (rotation encoder ticks)
        wStartPosition = MotionStatus::m_CurrentJoints.GetValue(id);

        // get initial value from motion module
        wGoalPosition = Walking::GetInstance()->m_Joint.GetValue(id);
        
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
    cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 5, JointData::NUMBER_OF_JOINTS - 1, param);    

    
    printf("Press the ENTER key to begin!\n");
    getchar();
	
    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
    Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	MotionManager::GetInstance()->SetEnable(true);

    while(1)
    {
        Point2D pos;
        LinuxCamera::GetInstance()->CaptureFrame();

        memcpy(rgb_ball->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);

        tracker.Process(ball_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame));
        follower.Process(tracker.ball_position);

        for(int i = 0; i < rgb_ball->m_NumberOfPixels; i++)
        {
            if(ball_finder->m_result->m_ImageData[i] == 1)
            {
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 0] = 255;
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 1] = 0;
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 2] = 0;
            }
        }

        streamer->send_image(rgb_ball);
    }

    return 0;
}
