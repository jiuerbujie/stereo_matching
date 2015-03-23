#include "stereo.h"
#include "../Utilities/params_parser.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

#define PI 3.1415
using namespace cv;
int main(int argc, void* argv[])
{
	//string pFile = (char*) argv[1];
	string pFile = "..\\..\\..\\Utilities\\EXAMPLE.ini";
	string imgFile1 = "..\\..\\..\\Utilities\\L0.bmp";
	string imgFile2 = "..\\..\\..\\Utilities\\R0.bmp";
	string imgRecFile1 = "..\\..\\..\\Utilities\\L0-r.bmp";
	string imgRecFile2 = "..\\..\\..\\Utilities\\R0-r.bmp";
	params_parser pp(pFile);

	stereo st(CameraType::CT_FISHEYE, RectifyType::RT_PESPECTIVE);
	/*Mat R1 = Mat(3,3,CV_32F,Scalar(0));
	Mat R2 = R1.clone();
	Mat P1 = Mat(3,4,CV_32F, Scalar(0));
	Mat P2 = P1.clone();*/
	Mat R1,R2,P1,P2,Q;
	Mat img1 = imread(imgFile1);
	Mat img2 = imread(imgFile2);
	Size imgsize = img1.size();
	//st.stereoRectify(pp.cameraMatrix1, pp.cameraMatrix2, pp.R, pp.T, R1, R2, P1, P2);
	
	//cv::stereoRectify must use data of CV_64F
	pp.cameraMatrix1.convertTo(pp.cameraMatrix1,CV_64F);
	pp.cameraMatrix2.convertTo(pp.cameraMatrix2,CV_64F);
	pp.distCoeffs1.convertTo(pp.distCoeffs1,CV_64F);
	pp.distCoeffs2.convertTo(pp.distCoeffs2,CV_64F);
	pp.R.convertTo(pp.R, CV_64F);
	pp.T.convertTo(pp.T, CV_64F);
	cv::stereoRectify(pp.cameraMatrix1,pp.distCoeffs1, pp.cameraMatrix2, pp.distCoeffs2, imgsize, pp.R, pp.T, R1, R2, P1, P2, Q);
	R1.convertTo(R1,CV_32F);
	R2.convertTo(R2,CV_32F);
	pp.cameraMatrix1.convertTo(pp.cameraMatrix1,CV_32F);
	pp.cameraMatrix2.convertTo(pp.cameraMatrix2,CV_32F);
	pp.distCoeffs1.convertTo(pp.distCoeffs1,CV_32F);
	pp.distCoeffs2.convertTo(pp.distCoeffs2,CV_32F);
	Mat map11, map12, map21, map22;
	st.initUndistortRectifyMap(pp.cameraMatrix1, pp.distCoeffs1, R1, pp.cameraMatrix1, img1.size(),CV_32F, PI, PI, map11, map12);
	st.initUndistortRectifyMap(pp.cameraMatrix2, pp.distCoeffs2, R2, pp.cameraMatrix1, img2.size(), CV_32F, PI, PI, map21, map22);
	Mat recImg1, recImg2;
	st.rectifyImage(img1, img2, map11, map12, map21, map22, recImg1, recImg2);
	imwrite(imgRecFile1, recImg1);
	imwrite(imgRecFile2, recImg2);

}