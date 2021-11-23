#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

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