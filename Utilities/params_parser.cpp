#include "params_parser.h"
#include "inifile.h"
#include "opencv2/calib3d/calib3d.hpp"
#include <iostream>
#define MAX_BUFF_SIZE 255
using namespace std;
using namespace cv;
#define  BUF_SIZE 256 
params_parser::params_parser(std::string file)
{

	char *section = "camera_config";
	char *key = "camera_type";
	const char* fileName = file.c_str();
	char  value[BUF_SIZE] = { 0 };
	// Read [camera_config]
	if ( ! read_profile_string(section, key, value, BUF_SIZE,"", fileName))
	{

		printf( " read ini file fail\n " );
	}

	cameraType = read_profile_int(section, key, 1, fileName);

	// Read [inter_left]
	section = "inter_left";
	key = "focal_lenth_x_left";
	float focal_lenth_x_left = read_profile_float(section, key, 0, fileName);
	key = "focal_lenth_y_left";
	float focal_lenth_y_left = read_profile_float(section, key, 0, fileName);
	key = "u0_left";
	float u0_left = read_profile_float(section, key, 0, fileName);
	key = "v0_left";
	float v0_left = read_profile_float(section, key, 0, fileName);
	//float cm1[3][3] = {{focal_lenth_x_left, 0, u0_left}, {0, focal_lenth_y_left, v0_left}, {0.0, 0.0, 1.0}};
	//cameraMatrix1 = cv::Mat(3,3, CV_32F, cm1);
	cameraMatrix1 = (Mat_<float>(3, 3) << focal_lenth_x_left, 0, u0_left,
										0, focal_lenth_y_left, v0_left,
										0.0, 0.0, 1.0);
	//std::cout<<cameraMatrix1<<endl;
	key = "k1_left";
	float k1_left = read_profile_float(section, key, 0, fileName);
	key = "k2_left";
	float k2_left = read_profile_float(section, key, 0, fileName);
	key = "k3_left";
	float k3_left = read_profile_float(section, key, 0, fileName);
	key = "p1_left";
	float p1_left = read_profile_float(section, key, 0, fileName);
	key = "p2_left";
	float p2_left = read_profile_float(section, key, 0, fileName);
	//float d1[5] = {k1_left, k2_left, k3_left, p1_left, p2_left};
	//distCoeffs1 = cv::Mat(1, 5, CV_32F, d1);
	distCoeffs1 = (Mat_<float>(1, 5) << k1_left, k2_left, k3_left, p1_left, p2_left);
	//cout <<distCoeffs1<<endl;
	// Read [inter_right]
	section = "inter_right";
	key = "focal_lenth_x_right";
	float focal_lenth_x_right = read_profile_float(section, key, 0, fileName);
	key = "focal_lenth_y_right";
	float focal_lenth_y_right = read_profile_float(section, key, 0, fileName);
	key = "u0_right";
	float u0_right = read_profile_float(section, key, 0, fileName);
	key = "v0_right";
	float v0_right = read_profile_float(section, key, 0, fileName);
	//float cm2[3][3] = {{focal_lenth_x_right, 0, u0_right}, {0, focal_lenth_y_right, v0_right}, {0.0, 0.0, 1.0}};
	//cameraMatrix2 = cv::Mat(3,3, CV_32F, cm2);
	cameraMatrix2 = (Mat_<float>(3,3) << focal_lenth_x_right, 0.0, u0_right,
										0.0, focal_lenth_y_right, v0_right,
										0.0, 0.0, 1.0);
	//cout << cameraMatrix2 <<endl;
	key = "k1_right";
	float k1_right = read_profile_float(section, key, 0, fileName);
	key = "k2_right";
	float k2_right = read_profile_float(section, key, 0, fileName);
	key = "k3_right";
	float k3_right = read_profile_float(section, key, 0, fileName);
	key = "p1_right";
	float p1_right = read_profile_float(section, key, 0, fileName);
	key = "p2_right";
	float p2_right = read_profile_float(section, key, 0, fileName);
	//float d2[5] = {k1_right, k2_right, k3_right, p1_right, p2_right};
	//distCoeffs2 = cv::Mat(1, 5, CV_32F, d2);
	distCoeffs2 = (Mat_<float>(1, 5) << k1_right, k2_right, k3_right, p1_right, p2_right);
	//cout << distCoeffs2 <<endl;
	// Read [exter_left_to_right]
	section = "exter_left_to_right";
	key = "om_x_l_to_r";
	float om_x_l_to_r = read_profile_float(section, key, 0, fileName);
	key = "om_y_l_to_r";
	float om_y_l_to_r = read_profile_float(section, key, 0, fileName);
	key = "om_z_l_to_r";
	float om_z_l_to_r = read_profile_float(section, key, 0, fileName);
	//float om_v[3] = {om_x_l_to_r, om_y_l_to_r, om_z_l_to_r};
	//om = cv::Mat(1, 3, CV_32F, om_v);
	om = (Mat_<float>(3, 1) << om_x_l_to_r, om_y_l_to_r, om_z_l_to_r);
	cv::Rodrigues(om, R);
	//cout << om <<endl;
	//cout << R <<endl;
	key = "T_x_l_to_r";
	float T_x_l_to_r = read_profile_float(section, key, 0, fileName);
	key = "T_y_l_to_r";
	float T_y_l_to_r = read_profile_float(section, key, 0, fileName);
	key = "T_z_l_to_r";
	float T_z_l_to_r = read_profile_float(section, key, 0, fileName);
	//float t[3] = {T_x_l_to_r, T_y_l_to_r, T_z_l_to_r};
	//this->T = cv::Mat(3, 1, CV_32F, t);
	T = (Mat_<float>(3, 1) << T_x_l_to_r, T_y_l_to_r, T_z_l_to_r);
	



}
params_parser::~params_parser(){}