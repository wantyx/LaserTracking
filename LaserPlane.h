#pragma once
/*
* ����ƽ���࣬���ڼ��������ƽ�����������ϵ�µ�ϵ��
*/
#include <opencv2/opencv.hpp>
#include "Utils.h"
#include "GlobalSetting.h"
#include <vector>
#include "Eye2Hand.h"
#include <opencv2/opencv.hpp>

class LaserPlane
{
public:
	LaserPlane();
	~LaserPlane();
	void calibrateWithNoLaser();
	void getLaserPoints(bool test, std::string _imagePathWithLaser, int _laserImageNumber);
	void uv2xyz(bool test);
	void test1();
	void camera2base(Eye2Hand eye2hand,double* p);
	void getLaserPlane();
	void findBlue(cv::Mat eye2hand,cv::Mat cameraIntri,cv::Mat distCoffe);
	void calMeanDistance();
	void getVPoints();

	std::string imagePathWithNoLaser;  //���������ͼƬ·��
	std::string imagePathWithLaser;   //������ͼƬ��·��
	int laserImageNumber;             //һ���ж�����ͼƬ�����㼤��ƽ��
	cv::Mat cameraIntri;
	cv::Mat distCoffe;
	std::vector<cv::Mat> chess2camera_R;
	std::vector<cv::Mat> chess2camera_T;
	std::vector<std::vector<cv::Point2f>> points;  //�õ����м���ͼƬ��������ͼ���е�����ֵ
	std::vector<cv::Point3f> pointsInCameraOXYZ;   //���м�������������ϵ�µĵ�xyz
	std::vector<double> parameters;                //ƽ�����

	std::string testImagePath;
	std::vector<cv::Point2f> testPoints;
	std::vector<cv::Point3f> testPointsInCameraOXYZ;
	std::vector<cv::Point3f> testPointsInBaseOXYZ;

private:
};

class Line {
public:
	cv::Point2f point0;
	cv::Point2f point1;
	double k;
	double angle;
	
	Line(cv::Point2f point0, cv::Point2f point1, double k, double angle);
};


