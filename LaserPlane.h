#pragma once
/*
* 激光平面类，用于计算出激光平面在相机坐标系下的系数
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

	std::string imagePathWithNoLaser;  //不带激光的图片路径
	std::string imagePathWithLaser;   //带激光图片的路径
	int laserImageNumber;             //一共有多少组图片来计算激光平面
	cv::Mat cameraIntri;
	cv::Mat distCoffe;
	std::vector<cv::Mat> chess2camera_R;
	std::vector<cv::Mat> chess2camera_T;
	std::vector<std::vector<cv::Point2f>> points;  //得到所有激光图片中心线在图像中的像素值
	std::vector<cv::Point3f> pointsInCameraOXYZ;   //所有激光点在相机坐标系下的点xyz
	std::vector<double> parameters;                //平面参数

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


