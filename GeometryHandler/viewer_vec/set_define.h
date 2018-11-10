#ifndef __SET_DEFINE_H__ 
#define __SET_DEFINE_H__ 


#ifndef M_PI
#define M_PI 3.1415923
#endif

#define EYE_OFFSET_X 70
#define EYE_OFFSET_Y 180
#define EYE_AXIS_DIR_X -1
#define EYE_AXIS_DIR_Y -1
#define CM2PIXEL 40
#define MM2PIXEL 4

#define SMEM_OFFSET 0x20
#define VIEW_RANGE 60.0

#define MAX_DOUBLE 32654
#define MIN_DOUBLE -32654

#define TRUE 1
#define FALSE 0

#define ALIVE 0
#define DEAD  1
#define PRE   2
#define BACK  3
#define BASE  4
#define DONE  5

//#ifndef OUT
//#define OUT   6
//#endif

//#define ALIVE 0
//#define DEAD  1
//#define PRE   2
//#define BACK  3
//#define OUT   4


#define RIGHT 0
#define LEFT 1

#define MAIN 0
#define SUB 1

#define START 0
#define STOP 1


#define FINISH 0
#define CONTINUE 1
#define DISPLAY_CONTINUE 2

//#define STOP_CLUSTER
//#define STOP_NUM 100
#define STOP_NUM 2

//#define NO_DISPLAY

#define DISPLAY_CONTINUE_START 100000 
//#define DISPLAY_CONTINUE_MODE

//#define WITHOUT_HEAP

//#define TRIANGLE_HIERARCHY

//#define LOD_MODE
//#define FRAME_DATA_SAVE


#define TIME_WRITE
//#define MOVE_VIEW_FROM_FILE
//#define WRITE_MOVE_VIEW

//#define WITHOUT_VIEW_FRUSTUM
//#define WITHOUT_BACKFACE_CULLING
//#define WITHOUT_LOD


#endif
