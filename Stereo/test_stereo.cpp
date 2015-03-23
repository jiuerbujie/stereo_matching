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
	//string pFile = "..\\..\\..\\Images\\EXAMPLE-f.ini";
	//string imgFile1 = "..\\..\\..\\Images\\Fish-L.jpg";
	//string imgFile2 = "..\\..\\..\\Images\\Fish-R.jpg";
	//string imgRecFile1 = "..\\..\\..\\Images\\Fish-L-rec-ll.jpg";
	//string imgRecFile2 = "..\\..\\..\\Images\\Fish-R-rec-ll.jpg";
	//string imgDis = "..\\..\\..\\Images\\Fish-dis.jpg";

	string pFile = "..\\..\\..\\Images\\EXAMPLE-p.ini";
	string imgFile1 = "..\\..\\..\\Images\\Per-L.bmp";
	string imgFile2 = "..\\..\\..\\Images\\Per-R.bmp";
	string imgRecFile1 = "..\\..\\..\\Images\\Per-L-rec-p.bmp";
	string imgRecFile2 = "..\\..\\..\\Images\\Per-R-rec-p.bmp";
	string imgDis = "..\\..\\..\\Images\\Per-dis.bmp";
	params_parser pp(pFile);

	stereo st(CameraType::CT_PESPECTIVE, RectifyType::RT_PESPECTIVE);
	Mat R1,R2,P1,P2,Q;
	Mat img1 = imread(imgFile1);
	Mat img2 = imread(imgFile2);
	Size imgsize = img1.size();

	// use rectify function of stereo class
	st.stereoRectify(pp.cameraMatrix1, pp.cameraMatrix2, pp.R, pp.T, R1, R2, P1, P2);

	//use rectify function of opencv, cv::stereoRectify must use data of CV_64F
	/*pp.cameraMatrix1.convertTo(pp.cameraMatrix1,CV_64F);
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
	pp.distCoeffs2.convertTo(pp.distCoeffs2,CV_32F);*/

	Mat map11, map12, map21, map22;
	Point2f logRange(0*PI, 1*PI);
	Point2f latRange(0*PI, 1*PI);
	st.initUndistortRectifyMap(pp.cameraMatrix1, pp.distCoeffs1, R1, pp.cameraMatrix1, img1.size(),CV_32F, map11, map12, logRange, latRange);
	st.initUndistortRectifyMap(pp.cameraMatrix2, pp.distCoeffs2, R2, pp.cameraMatrix1, img2.size(), CV_32F, map21, map22, logRange, latRange);
	Mat recImg1, recImg2;
	st.rectifyImage(img1, img2, map11, map12, map21, map22, recImg1, recImg2);
	imwrite(imgRecFile1, recImg1);
	imwrite(imgRecFile2, recImg2);
	Mat dis;
	st.stereoMatching(recImg1, recImg2, dis, 0, 100/16*16, 5, 8*5*5, 32*5*5);
	dis.convertTo(dis, CV_8U);
	imwrite(imgDis, dis);

}