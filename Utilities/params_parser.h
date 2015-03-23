
/*
This class reads camera parameters from an ini file. The structure of this file should be strictly follow the example below. 
A "Section" is a name of a group of related parameters, such as "inter_left", which represents 

A "Key" is a name of a parameter, such as "focal_lenth_left", which is 

Example:

[camera_config]
# camera type, 1 for perspective camera, 2 for fisheye camera
camera_type = 1

# The internal parameters of the left camera
[inter_left]

# The focal lenth of the left/right camera, in pixel.
focal_lenth_x_left: 512
# The internal parameters of the left camera
focal_lenth_y_left: 512
# The focal center of the image, in pixel.
u0_left = 256
v0_left = 256
# Radial distortion coefficient
k1_left = 0.001
k2_left = 0.001
k3_left = 0.001
# Tangential distortion
p1_left = 0.001
p2_left = 0.001

[inter_right]
...

# The external parameters from left camera to right camera.
[exter_left_to_right]

# Rotation 
om_x_l_to_r = 0.3
om_y_l_to_r = 0.3
om_z_l_to_r = 0.3
# Translation
T_x_l_to_r = 100
T_y_l_to_r = 100
T_z_l_to_r = 100

[exter_left_to_right]
...
# can be discard.

*/
#ifndef PARAMSPARSER
#define PARAMSPARSER


#include <string>
#include "opencv2/core/core.hpp"
using namespace std;



class params_parser
{
public:
	params_parser(std::string file);
	~params_parser();

	/* data */
	int cameraType; // Camera type, 1 for perspective, 2 for omnidirectional
	cv::Mat cameraMatrix1, cameraMatrix2; // Camera matrix A of the first and second camera
	cv::Mat R; // Rotation matrix between the first and the second camera
	cv::Mat om; // Rotation vector
	cv::Mat T; // Translation between the first and the second camera
	cv::Mat distCoeffs1, distCoeffs2; // Distortion coefficients (k1,k2,p1,p2[,k3[,k4,k5,k6]]) of 4, 5, or 8 elements

};


#endif