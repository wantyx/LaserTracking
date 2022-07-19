#pragma once
#include <opencv2/opencv.hpp>
#include "Utils.h"
#include "GlobalSetting.h"
#include <vector>
#include "Eye2Hand.h"
#include <opencv2/opencv.hpp>

class LaserImage {
public:
	LaserImage();
	~LaserImage();
	std::vector<cv::Point2f> findLowestPoint(cv::Mat laserImageV);
};