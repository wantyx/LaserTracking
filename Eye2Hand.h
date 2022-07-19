#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include "GlobalSetting.h"
#include "Utils.h"

class Eye2Hand
{
public:
	Eye2Hand();
	void doEye2Hand();  //ִ�����۱궨
	void chess2camera();
	void end2base();
	void uv2xyz();
	void uv2xyz(std::vector<cv::Point2f> pointBuf);//���ڲ���
	~Eye2Hand();

	cv::Mat get_eye2handRT();
	cv::Mat getCameraIntri();
	cv::Mat getDistCoffe();

private:
	cv::Mat eye2handRT;  //���۱궨�Ľ��
	cv::Mat cameraIntri;  //����ڲ�
	cv::Mat distCoffe;   //�������
	std::string srcPath;  //���̸�ͼƬ·��
	int imgNumber;       //���е�ͼƬ����
	std::vector<std::vector<cv::Point2f>> pointBufs;  //���б궨ͼƬ�ı궨���ϵĵ���ͼƬ�е�����ֵ	
	std::vector<cv::Mat> end2baseRs;
	std::vector<cv::Mat> end2baseTs;
	std::vector<cv::Mat> chess2camera_R;
	std::vector<cv::Mat> chess2camera_T;
	std::vector<cv::Mat> end2baseRT;
};

