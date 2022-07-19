#pragma once
#include <opencv2/opencv.hpp>
#include "Utils.h"
#include "GlobalSetting.h"
#include <vector>
#include "Eye2Hand.h"
#include <opencv2/opencv.hpp>

class LaserPlaneWithOneImage {
public:
	LaserPlaneWithOneImage();
	~LaserPlaneWithOneImage();
	cv::Point3f getPointInCameraOXY(cv::Point point);

	double A_caliBoard;
	double B_caliBoard;
	double C_caliBoard;
	double D_caliBoard;
	std::string planeImagePath = "./LaserPlaneWithOneImage/plane.bmp";
	cv::Mat cameraIntri;
	cv::Mat distCoffe;
	std::vector<cv::Mat> chess2camera_R;
	std::vector<cv::Mat> chess2camera_T;
};