// This is the file where the robot will be programmed for its autonomy. The Pi5 for the original computer vision
// vision program broke so everything will be programmed through the esp-32. The esp32 will have encoders to verify
// any sliping movements from the wheels and help with localization so that the robot can move to different points on
// the field. 



// To save time on executing lines of code, the program will be directly calling the GPIO pins from the esp32 allowing for faster speeds

#include "Motors/fastfunctions.h"