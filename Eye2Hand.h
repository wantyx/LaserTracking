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
	void doEye2Hand();  //ִ�����۱궨
	void chess2camera();
	void end2base();
	~Eye2Hand();

	cv::Mat get_eye2handRT();

private:
	cv::Mat eye2handRT;  //���۱궨�Ľ��
	cv::Mat cameraIntri;  //����ڲ�
	cv::Mat distCoffe;   //�������
	std::string srcPath;  //���̸�ͼƬ·��
	int imgNumber;       //���е�ͼƬ����
	vector<Mat> end2baseRs;
	vector<Mat> end2baseTs;
	vector<Mat> chess2camera_R;
	vector<Mat> chess2camera_T;
};

