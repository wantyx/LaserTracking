#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include "GlobalSetting.h"
#include "Utils.h"

using namespace cv;
using namespace std;

class Eye2Hand
{
public:
	Eye2Hand();
	void doEye2Hand();  //执行手眼标定
	void chess2camera();
	void end2base();
	~Eye2Hand();

	cv::Mat get_eye2handRT();

private:
	cv::Mat eye2handRT;  //手眼标定的结果
	cv::Mat cameraIntri;  //相机内参
	cv::Mat distCoffe;   //相机畸变
	std::string srcPath;  //棋盘格图片路径
	int imgNumber;       //所有的图片数量
	vector<Mat> end2baseRs;
	vector<Mat> end2baseTs;
	vector<Mat> chess2camera_R;
	vector<Mat> chess2camera_T;
};

