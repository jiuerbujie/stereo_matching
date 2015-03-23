#ifndef STEREO_H
#define STEREO_H
#include "opencv2/core/core.hpp"
#define PI 3.14159
enum CameraType {
	CT_PESPECTIVE,
	CT_FISHEYE
};

// Support full scene stereo matching and 3D reconstruction if cameraType is se LONGt to be LONGITUDE_LATITUDE
enum RectifyType {
	RT_PESPECTIVE,
	RT_LONGITUDE_LATITUDE
};

class stereo{
public:
	stereo(CameraType cType, RectifyType rType);

	~stereo();

	/**
	*@brief Computes rectification transforms for each head of a calibrated stereo camera
	* the interface is compatible with Opencv
	*@param K1 [in] Input camera matrix for the first camera
	*@param D1 [in] Input distortion matrix for the first camera
	*@param K2 [in] Input camera matrix for the second camera
	*@param D2 [in] Input distortion matrix for the second camera
	*@param imageSize [in] Input size of input image
	*@param R1 [out] Output 3x3 rectification transform (rotation matrix) for the first camera.
	*@param R2 [out] Output 3x3 rectification transform (rotation matrix) for the second camera.
	*@param P1 [out] Output 3x4 projection matrix in the new (rectified) coordinate systems for the first camera.
	*@param P2 [out] Output 3x4 projection matrix in the new (rectified) coordinate systems for the second camera.
	*@return 1 : read success; \n 0 : read fail
	*/
	void stereoRectify(cv::InputArray K1, cv::InputArray K2, cv::InputArray R, cv::InputArray T,
						cv::OutputArray R1, cv::OutputArray R2, cv::OutputArray P1, cv::OutputArray P2);
	
	/**
	*@brief Computes the undistortion and rectification transformation map.
	*@param cameraMatrix [in] Input camera matrix for camera
	*@param distCoeffs [in] Input camera distortion coeffcients (k1,k2,p1,p2[,k3[,k4,k5,k6]])
	*@param R [in] Input rectification rotation
	*@param newCameraMatrix [in] Input new camera matrix, unused if RectifyType = LONGITUDELATITUDE
	*@param undisSize [in] Input undistorted image size, which is related to newCameraMatrix and is unused if RectifyType = LONGITUDELATITUDE
	*@param m1type [in] Input type of map, now can only be CV_32FC1
	*@param map1 [out] Output map of the u coordinate of undistorted image to distorted 
	*@param map2 [out] Output map of the v coordinate of undistorted image to distorted 
	*/
	void initUndistortRectifyMap(cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs, cv::InputArray _R,
		cv::InputArray _newCameraMatrix,cv::Size undisSize, int m1type, 
		cv::OutputArray _map1, cv::OutputArray _map2,cv::Point2f rangeLongitude=cv::Point2f(0.0, PI), cv::Point2f rangeLatitude = cv::Point2f(0.0, PI));

	/**
	*@brief Rectify images using map1 and map2 computed by initUndistortRectifyMap
	*@param inImage1 [in] The first image
	*@param inImage2 [in] The second image
	*@param map1 [in] map1 from initUndistortRectifyMap
	*@param map2 [in] map2 from initUndistortRectifyMap
	*@param recImage1 [out] rectified image 1
	*@param recImage2 [out] rectified image 2
	*/
	void rectifyImage(cv::InputArray _inImage1, cv::InputArray _inImage2, cv::InputArray _map11, cv::InputArray _map12,
		cv::InputArray _map21, cv::InputArray _map22, cv::OutputArray _recImage1, cv::OutputArray _recImage2);

	/**
	*@brief stereo matching to get disparity map
	*@param recImage1 [in] The first rectified image
	*@param recImage2 [in] The second rectified image
	*@param disparityMap [out] The output disparity map
	*/
	void stereoMatching(cv::InputArray _recImage1, cv::InputArray _recIamge2, cv::OutputArray _disparityMap, int minDisparity, int numDisparities, int SADWindowSize, int P1, int P2);

	// data
	CameraType cameraType;
	RectifyType rectifyType;

};

#endif