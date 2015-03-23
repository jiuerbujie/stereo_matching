#include "stereo.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#define PI 3.14159
using namespace cv;
using namespace std;
stereo::stereo(CameraType cType, RectifyType rType)
{
	// perspective camera is not allowed to use LONGITUDE_LATITUDE since its FOV is small.
	CV_Assert(!(cType == CT_PESPECTIVE && rType == RT_LONGITUDE_LATITUDE));
	cameraType = cType;
	rectifyType = rType;
}
stereo::~stereo(){}

void stereo::stereoRectify(cv::InputArray _K1, cv::InputArray _K2, cv::InputArray _R, cv::InputArray _T,
	cv::OutputArray _R1, cv::OutputArray _R2, cv::OutputArray _P1, cv::OutputArray _P2)
{
	Mat K1 = _K1.getMat(), K2 = _K2.getMat(), R = _R.getMat(), T = _T.getMat();
	_R1.create(3, 3, CV_32F);
	_R2.create(3, 3, CV_32F);
	Mat R1 = _R1.getMat();
	Mat R2 = _R2.getMat();
	_P1.create(3, 4, CV_32F);
	_P2.create(3, 4, CV_32F);
	Mat P1 = _P1.getMat();
	Mat P2 = _P2.getMat();

	if(K1.type()!=CV_32F)
		K1.convertTo(K1,CV_32F);
	if(K2.type()!=CV_32F)
		K2.convertTo(K2,CV_32F);
	if(R.type()!=CV_32F)
		R.convertTo(R,CV_32F);
	if(T.type()!=CV_32F)
		T.convertTo(T,CV_32F);
	if(T.rows != 3)
		T = T.t();
	// R and T is the transformation from the first to the second camera
	// Get the transformation from the second to the first camera

	Mat R_inv = R;
	Mat T_inv = T;
	
	/*float* R_inv_ptr = (float*)R_inv.data;
	float* T_inv_ptr = (float*)T_inv.data;
	float R_inv_data[3][3] = {{R_inv_ptr[0], R_inv_ptr[1], R_inv_ptr[2]}, 
	{R_inv_ptr[3], R_inv_ptr[4],R_inv_ptr[5]},
	R_inv_ptr[6], R_inv_ptr[7], R_inv_ptr[8]};
	float T_inv_data[3] = {T_inv_ptr[0], T_inv_ptr[1], T_inv_ptr[2]};*/

	//R1 = Mat(3,3,CV_32F, Scalar(0));
	
	Mat e1, e2, e3;
	/*Mat e1 = R1.row(0);
	Mat e2 = R1.row(1);
	Mat e3 = R1.row(2);*/
	e1 = T_inv.t() / norm(T_inv);
	Mat z = (Mat_<float>(1, 3) << 0,0,1);
	e2 = e1.cross(z);
	e2 = e2 / norm(e2);
	e3 = e1.cross(e2);
	e3 = e3 / norm(e3);
	e1.copyTo(R1.row(0));
	e2.copyTo(R1.row(1));
	e3.copyTo(R1.row(2));
	R2 = R_inv * R1;

	P1.setTo(Scalar(0));
	//P1.colRange(0,3) = R1.clone();
	R1.copyTo(P1.colRange(0, 3));
	P1 = K1 * P1;

	P2.setTo(Scalar(0));
	//P2.colRange(0,3) = R2.clone();
	R2.copyTo(P2.colRange(0, 3));
	P2 = K2 * P2;
	
}

void stereo::initUndistortRectifyMap(cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs, cv::InputArray _R,
	cv::InputArray _newCameraMatrix,cv::Size undisSize, int m1type, float rangeLongitude, float rangeLatitude,
	cv::OutputArray _map1, cv::OutputArray _map2)
{
	Mat K = _cameraMatrix.getMat(), disCoeffs = _distCoeffs.getMat(), R = _R.getMat(),
		newK = _newCameraMatrix.getMat();
	_map1.create(undisSize, m1type);
	_map2.create(undisSize, m1type);
	Mat map1 = _map1.getMat();
	Mat map2 = _map2.getMat();
	

	// distoration coefficients (k1,k2,p1,p2[,k3[,k4,k5,k6]])
	// [x_u;y_u] is the undistorted point in image plane, r2 = x_u^2 + y_u^2
	// [x_d;y_d] is the distorted point in image plane, then
	// x_d = x_u*(1+k1*r2+k2*r4+k3*r6)/(1+k4*r2+k5*r4+k6*r6)+2*p1*x_u*y_u+p2*(r2+2*x_u*x_u)
	// the proceeding of y_d is similar to x_d
	float k1 = disCoeffs.at<float>(0);
	float k2 = disCoeffs.at<float>(1);
	float p1 = disCoeffs.at<float>(2);
	float p2 = disCoeffs.at<float>(3);
	float k3 = disCoeffs.total() >=5 ? disCoeffs.at<float>(4) : 0;
	float k4 = disCoeffs.total() >=8 ? disCoeffs.at<float>(5) : 0;
	float k5 = disCoeffs.total() >=8 ? disCoeffs.at<float>(6) : 0;
	float k6 = disCoeffs.total() >=8 ? disCoeffs.at<float>(7) : 0;

	float fx = K.at<float>(0,0);
	float fy = K.at<float>(1,1);
	float u0 = K.at<float>(0,2);
	float v0 = K.at<float>(1,2);

	// inv(R)*inv(newK),which transform a pixel in undistorted image to undistorted coordinate
	// useless if rectifyType == RT_LONGITUDE_LATITUDE
	Mat newKR_inv = (newK*R).inv();
	const float* nkri = &newKR_inv.at<float>(0,0);
	for (int i = 0; i < undisSize.height; i++)
	{
		float* m1f = (float*)(map1.data + map1.step*i);
		float* m2f = (float*)(map2.data + map2.step*i);
		if (rectifyType == RT_PESPECTIVE)
		{
			// [_x;_y;_z] = inv(R)*inv(K)*[u;v;1]
			float _x, _y, _z; 
			// [u;v] = [j,i]
			_x = i*nkri[1] + nkri[2];
			_y = _y = i*nkri[4] + nkri[5];
			_z = i*nkri[7] + nkri[8];
			for (int j = 0; j < undisSize.width; j++, _x += nkri[0], _y += nkri[3], _z += nkri[6])
			{
				float x, y;
				if(cameraType == CT_PESPECTIVE)
				{
					// prject to image plane [x;y;1] using perspective model
					x = _x / _z, y = _y / _z;
				}
				else if(cameraType == CT_FISHEYE)
				{
					// prject to image plane [x;y;1] using equal distance model
					// first project to sphere
					float x_s = _x / sqrt(_x*_x + _y*_y + _z*_z);
					float y_s = _y / sqrt(_x*_x + _y*_y + _z*_z);
					float z_s = _z / sqrt(_x*_x + _y*_y + _z*_z);
					// On a sphere, x = sin(theta)cos(alpha), y = sin(theta)cos(alpha), z = cos(theta)
					float theta = acos(z_s);
					float calpha = x_s / sin(theta);
					float salpha = y_s / sin(theta);
					float r = theta;
					x = r * calpha;
					y = r * salpha;
				}
				float x2 = x*x, y2 = y*y,_2xy = 2*x*y;
				float r2 = x*x + y*y;
				float kr = (1 + ((k3*r2 + k2)*r2 + k1)*r2)/(1 + ((k6*r2 + k5)*r2 + k4)*r2);
				float x_d = x*kr + p1*_2xy + p2*(r2 + 2*x2);
				float y_d = y*kr + p1*(r2 + 2*y2) + p2*_2xy;
				// convert to pixel
				float u_d = fx*x_d + u0;
				float v_d = fy*y_d + v0;
				
				m1f[j] = float(u_d);
				m2f[j] = float(v_d);
			}
		}
		else if(rectifyType == RT_LONGITUDE_LATITUDE)
		{
			// only fish eye camera is allowed
			// [j,i] represent one [longitude,latitude] pair 
			for (int j = 0; j < undisSize.width; j++)
			{
				float m = (float)undisSize.width;
				float n = (float)undisSize.height;

				float longitude = rangeLongitude*j/m, latitude = rangeLatitude*i/n;
				// geometric coordinate
				float x_s = cos(longitude)*-1, y_s = sin(longitude)*cos(latitude)*-1, z_s = sin(latitude)*sin(longitude);
				
				// rotate by R.inv()
				Mat_<float> R_inv = Mat_<float>(R.inv());
				
				float _x = R_inv(0,0)*x_s + R_inv(0,1)*y_s + R_inv(0,2)*z_s;
				float _y = R_inv(1,0)*x_s + R_inv(1,1)*y_s + R_inv(1,2)*z_s;
				float _z = R_inv(2,0)*x_s + R_inv(2,1)*y_s + R_inv(2,2)*z_s;
				// project back to sphere
				x_s = _x / sqrt(_x*_x + _y*_y + _z*_z);
				y_s = _y / sqrt(_x*_x + _y*_y + _z*_z);
				z_s = _z / sqrt(_x*_x + _y*_y + _z*_z);

				//// On a sphere, x = sin(theta)cos(alpha), y = sin(theta)cos(alpha), z = cos(theta)
				//float theta = atan(sqrt(x_s*x_s + y_s*y_s)/z_s);
				//// For equal distance model r = theta, r = x*x + y*y
				//float r = theta;
				//// x = r*cos(alpha), y = r*sin(alpha)
				//float x = r * x_s / sqrt(x_s*x_s + y_s*y_s);
				//float y = r * y_s / sqrt(x_s*x_s + y_s*y_s);


				float theta = acos(z_s);
				float calpha = x_s / sin(theta);
				float salpha = y_s / sin(theta);
				float r = theta;
				float x = r * calpha;
				float y = r * salpha;


				// distortion
				float x2 = x*x, y2 = y*y,_2xy = 2*x*y;
				float r2 = x*x + y*y;
				float kr = (1 + ((k3*r2 + k2)*r2 + k1)*r2)/(1 + ((k6*r2 + k5)*r2 + k4)*r2);
				float x_d = x*kr + p1*_2xy + p2*(r2 + 2*x2);
				float y_d = y*kr + p1*(r2 + 2*y2) + p2*_2xy;
				// convert to pixel
				float u_d = fx*x_d + u0;
				float v_d = fy*y_d + v0;

				m1f[j] = float(u_d);
				m2f[j] = float(v_d);

			}
		}
	}

}

void stereo::rectifyImage(cv::InputArray _inImage1, cv::InputArray _inImage2, cv::InputArray _map11, cv::InputArray _map12,
	cv::InputArray _map21, cv::InputArray _map22, cv::OutputArray _recImage1, cv::OutputArray _recImage2)
{
	Mat img1 = _inImage1.getMat();
	Mat img2 = _inImage2.getMat();
	Mat map11 = _map11.getMat();
	Mat map12 = _map12.getMat();
	Mat map21 = _map21.getMat();
	Mat map22 = _map22.getMat();


	_recImage1.create(map11.size(), img1.type());
	Mat recImg1 = _recImage1.getMat();
	_recImage2.create(map21.size(), img2.type());
	Mat recImg2 = _recImage2.getMat();

	remap(img1, recImg1, map11, map12, INTER_LINEAR);
	remap(img2, recImg2, map21, map22, INTER_LINEAR);

	

}

void stereo::stereoMatching(cv::Mat recImage1, cv::Mat recIamge2, cv::Mat disparityMap)
{

}