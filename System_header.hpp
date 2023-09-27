#ifndef Fundamental_Libs
#define Fundamental_Libs

#include <thread>
#include <future>
#include <mutex>
#include <shared_mutex>
#include <condition_variable>
#include <string>
#include <set>

#include <czmq.h>
#include <atomic>
#include <unordered_set>
#include <unordered_map>
#include <deque>
#include <algorithm>

#include <fstream>
#include <iostream>

#include <memory>
#include <cmath>
#include <utility>
#include <vector>


//Computer_Vision
#include<opencv4/opencv2/opencv.hpp>
#include<opencv4/opencv2/highgui.hpp>
#include<opencv4/opencv2/calib3d.hpp>
#include<opencv4/opencv2/core/core.hpp>
#include<opencv4/opencv2/aruco.hpp>


// Tensorflow
#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/string_util.h"
#include "tensorflow/lite/examples/label_image/get_top_n.h"\
#include "tensorflow/lite/model.h"

using namespace cv;
using namespace std;


#define time_for_movement  3.1 // time laps for 1m movement with 85% speed;
#define time_for_spin  4.2// time laps for 360* spin at 85% speed;
#define Angles_to_scan 6
#define Turn_Radious 0.4 // From <0.1 to 1>

const float SSA = time_for_spin / Angles_to_scan;
const uint16_t DC_motor_SPEED = 85;


extern unordered_map<string,pair<string, uint16_t>>Port_map;


extern atomic <float> Global_Ultrasonic;
extern atomic <bool> ROBOT_OPERATIONAL;
extern atomic <float> angle_of_attack;
extern atomic <bool> my_vision_is_augmented;

extern string movement_command, move_instruction;
extern bool flagMOV_ready, SeeK_n_Destroy;



extern mutex global_mx1, Error_mx, Head_mx;
extern condition_variable gl_Conditonal1, Error_cv, Head_cv;


enum Orientation {
                    Scan_Area, reCalirbate, Forward,
                    Backward, Right, Left, Dance,
                    Forward_right, Backward_Right,
                    Forward_Left, Backward_left, 
                    Intermission, Flip
                };

enum Action_Mode{
                  Initializing,  // Boot-up routine  
                  Lost_in_Space, // recalibrate spacial awarness
                  Kill_Human,    // Animation + Vocal Delivery
                  Beacon_Aprox,  // Computer vision + Maths
                  Be_Cute,       // Animation + Vocal Delivery
                  Browse_Internet, // MISC
                  Something_went_Wrong // Undefined Behaviour
                };




  #ifndef Error
  #define Error
  
  #include "Error.hpp"

  #endif


  #ifndef Spacial
  #define Spacial

  #include "Traversal.hpp"

  #endif

#endif