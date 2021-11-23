#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

/*
parameter：
	boardSize:标定板角点的Size
	squareSize:标定板的检点间隔
	corners:存储的角点
*/
void cal_pointInBoard(cv::Size boardSize, float squareSize, std::vector<cv::Point3f>& corners);

/*分别计算RT中的R和T*/
cv::Mat calculateH(double p[]);
cv::Mat calculateT(double p[]);