#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <opencv2/gapi/imgproc.hpp>

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

/* 利用hession矩阵求解中心线 */
std::vector<double> StegerLine0(cv::Mat grayImg);

/*
将得到的vector<float>转换为point2f形式
*/
void convertToVectorPoints(const std::vector<double> Pt, std::vector<cv::Point2f>& vec);

/*
* 将三维点计算出一个三维平面
*/
bool fitPlaneByLeastSquares(std::vector<double>& Parameters, const std::vector<cv::Point3f>& Data);

void fitPlane(const std::vector<cv::Point3f> Data, double* plane);
/*
* 使用cv自带的函数计算2d平面的点所构成的一条直线
*/
void fitLine(const std::vector<cv::Point2f>& points,std::vector<float> line);

/*
* 将机械臂上的读数转换为RT矩阵
*/
cv::Mat calEnd2Base(double p[]);

/*
* 按照自定义的RGB通道比例进行转换灰度
*/
cv::Mat BGR2GRAYForLaser(cv::Mat bgr, cv::Mat gray, int b, int g, int r);

