#pragma once

//#define IDS_MODE
//#define VID_MODE
#define DEBUG_MODE
#define VERBOSE_MODE
//#define ADAPTIVE_MODE
//#define STOP_L_MODE
#define FPS_COUNT
//#define FUNCTION_TIMING
//#define TEST_SHM

#define FRAME_TIME 10
#define FPS_AMOUNT 500

#define MAX_STOP_ANGLE 50
#define ROI_STOP_X 30
#define ROI_STOP_Y 190

#ifdef IDS_MODE
#undef VID_MODE
#endif

#ifndef DEBUG_MODE
#undef VERBOSE_MODE
#undef TEST_SHM
#endif

#ifdef IDS_MODE         // IDS
#define CAM_RES_X 752
#define CAM_RES_Y 400
#else
#ifdef VID_MODE        // File
#define CAM_RES_X 640
#define CAM_RES_Y 360
#else                   // Kurokesu
#define CAMERA_INDEX 1
#define CAM_RES_X 640
#define CAM_RES_Y 360
#endif
#endif
