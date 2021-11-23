#include "Utils.h"

void cal_pointInBoard(cv::Size boardSize, float squareSize, std::vector<cv::Point3f>& corners)
{
	for (int i = 0; i < boardSize.height; i++) {
		for (int j = 0; j < boardSize.width; j++) {
			corners.push_back(cv::Point3f(float(j * squareSize), float(i * squareSize), 0));
		}
	}
}

cv::Mat calculateH(double p[])
{
	using namespace cv;
	Mat R_x = (Mat_<double>(3, 3) << 1, 0, 0, 0, cos((p[3] / 180) * CV_PI), -sin((p[3] / 180) * CV_PI), 0, sin((p[3] / 180) * CV_PI), cos((p[3] / 180) * CV_PI));
	Mat R_y = (Mat_<double>(3, 3) << cos((p[4] / 180) * CV_PI), 0, sin((p[4] / 180) * CV_PI), 0, 1, 0, -sin((p[4] / 180) * CV_PI), 0, cos((p[4] / 180) * CV_PI));
	Mat R_z = (Mat_<double>(3, 3) << cos((p[5] / 180) * CV_PI), -sin((p[5] / 180) * CV_PI), 0, sin((p[5] / 180) * CV_PI), cos((p[5] / 180) * CV_PI), 0, 0, 0, 1);
	Mat R = R_z * R_y * R_x;
	return R;
}

cv::Mat calculateT(double p[])
{
	using namespace cv;
	Mat T = Mat(3, 1, CV_64F);
	for (int i = 0; i < 3; i++) {
		T.at<double>(i, 0) = p[i];
	}
	return T;
}
