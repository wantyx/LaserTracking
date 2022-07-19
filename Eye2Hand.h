#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include "GlobalSetting.h"
#include "Utils.h"

class Eye2Hand
{
public:
	Eye2Hand();
	void doEye2Hand();  //执行手眼标定
	void chess2camera();
	void end2base();
	void uv2xyz();
	void uv2xyz(std::vector<cv::Point2f> pointBuf);//用于测试
	~Eye2Hand();

	cv::Mat get_eye2handRT();
	cv::Mat getCameraIntri();
	cv::Mat getDistCoffe();

private:
	cv::Mat eye2handRT;  //手眼标定的结果
	cv::Mat cameraIntri;  //相机内参
	cv::Mat distCoffe;   //相机畸变
	std::string srcPath;  //棋盘格图片路径
	int imgNumber;       //所有的图片数量
	std::vector<std::vector<cv::Point2f>> pointBufs;  //所有标定图片的标定板上的点在图片中的像素值	
	std::vector<cv::Mat> end2baseRs;
	std::vector<cv::Mat> end2baseTs;
	std::vector<cv::Mat> chess2camera_R;
	std::vector<cv::Mat> chess2camera_T;
	std::vector<cv::Mat> end2baseRT;
};

