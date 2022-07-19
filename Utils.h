#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <opencv2/gapi/imgproc.hpp>

/*
parameter��
	boardSize:�궨��ǵ��Size
	squareSize:�궨��ļ����
	corners:�洢�Ľǵ�
*/
void cal_pointInBoard(cv::Size boardSize, float squareSize, std::vector<cv::Point3f>& corners);

/*�ֱ����RT�е�R��T*/
cv::Mat calculateH(double p[]);
cv::Mat calculateT(double p[]);

/* ����hession������������� */
std::vector<double> StegerLine0(cv::Mat grayImg);

/*
���õ���vector<float>ת��Ϊpoint2f��ʽ
*/
void convertToVectorPoints(const std::vector<double> Pt, std::vector<cv::Point2f>& vec);

/*
* ����ά������һ����άƽ��
*/
bool fitPlaneByLeastSquares(std::vector<double>& Parameters, const std::vector<cv::Point3f>& Data);

void fitPlane(const std::vector<cv::Point3f> Data, double* plane);
/*
* ʹ��cv�Դ��ĺ�������2dƽ��ĵ������ɵ�һ��ֱ��
*/
void fitLine(const std::vector<cv::Point2f>& points,std::vector<float> line);

/*
* ����е���ϵĶ���ת��ΪRT����
*/
cv::Mat calEnd2Base(double p[]);

/*
* �����Զ����RGBͨ����������ת���Ҷ�
*/
cv::Mat BGR2GRAYForLaser(cv::Mat bgr, cv::Mat gray, int b, int g, int r);

