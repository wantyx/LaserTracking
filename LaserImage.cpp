#include "LaserImage.h"
#include <cmath>
using namespace std;
using namespace cv;

cv::Mat getVPoints(cv::Mat src,cv::Rect rect) {
	if (src.empty()) {
		return src;
	}
	cv::Mat ans;
	ans = src(rect);
	return ans;
}

Vec4d lines_intersection(const Vec4i l1, const Vec4i l2)
{
	double x1 = l1[0], y1 = l1[1], x2 = l1[2], y2 = l1[3];
	double a1 = -(y2 - y1), b1 = x2 - x1, c1 = (y2 - y1) * x1 - (x2 - x1) * y1; // 一般式：a1x+b1y1+c1=0
	double x3 = l2[0], y3 = l2[1], x4 = l2[2], y4 = l2[3];
	double a2 = -(y4 - y3), b2 = x4 - x3, c2 = (y4 - y3) * x3 - (x4 - x3) * y3; // 一般式：a2x+b2y1+c2=0
	bool r = false;                                                             // 判断结果
	double x0 = 0, y0 = 0;                                                      // 交点
	double angle = 0;                                                           // 夹角
	// 判断相交
	if (b1 == 0 && b2 != 0) // l1垂直于x轴，l2倾斜于x轴
		r = true;
	else if (b1 != 0 && b2 == 0) // l1倾斜于x轴，l2垂直于x轴
		r = true;
	else if (b1 != 0 && b2 != 0 && a1 / b1 != a2 / b2)
		r = true;
	if (r)
	{
		//计算交点
		x0 = (b1 * c2 - b2 * c1) / (a1 * b2 - a2 * b1);
		y0 = (a1 * c2 - a2 * c1) / (a2 * b1 - a1 * b2);
		// 计算夹角
		double a = sqrt(pow(x4 - x2, 2) + pow(y4 - y2, 2));
		double b = sqrt(pow(x4 - x0, 2) + pow(y4 - y0, 2));
		double c = sqrt(pow(x2 - x0, 2) + pow(y2 - y0, 2));
		angle = acos((b * b + c * c - a * a) / (2 * b * c)) * 180 / CV_PI;
	}
	return Vec4d(r, x0, y0, angle);
}

cv::Point2f getPointInAngularBisector(vector<cv::Vec4i> lines, cv::Vec4d anglePoint) {
	cv::Point point1 = cv::Point(lines[0][0], lines[0][1]);
	cv::Point point2 = cv::Point(lines[0][2], lines[0][3]);
	cv::Point point3 = cv::Point(lines[1][0], lines[1][1]);
	cv::Point point4 = cv::Point(lines[1][2], lines[1][3]);
	double angle1;
	double angle2;
	if ((point2.x - point1.x) != 0) {
		double k1 = (point2.y - point1.y) / (point2.x - point1.x);
		angle1 = atan(k1);
	}
	else {
		angle1 = 3.1415 / 2;
	}
	if ((point4.x - point3.x) != 0) {
		double k2 = (point4.y - point3.y) / (point4.x - point3.x);
		angle2 = atan(k2);
	}
	else {
		angle2 = 3.1415 / 2;
	}
	double angle = (angle1 + angle2) / 2;
	double k = tan(angle);
	cv::Point2f result;
	result.x = anglePoint[1] + 20;
	result.y = anglePoint[2] + 20 * k;
	return result;
}

LaserImage::LaserImage()
{
}

LaserImage::~LaserImage()
{
}

/*
* 1.首先求解线激光中心点
* 2.求解所有激光点的像素坐标的z值
* 3.取出像素坐标低于平均值的点进行拟合出直线
*/
vector<cv::Point2f> LaserImage::findLowestPoint(cv::Mat laserImageV)
{
	vector<cv::Point2f> res;
	cv::Mat view = cv::imread("V.bmp");
	//cv::Mat view = laserImageV.clone();
	if (view.empty()) {
		std::cout << "没有找到对应的V型图片信息"  << std::endl;
		return res;
	}
	/*转灰度之后得到中心线，得到中心线所在的点在图片中的像素坐标*/
	cv::Mat grayView;
	cv::cvtColor(view, grayView, COLOR_BGR2GRAY);
	grayView = BGR2GRAYForLaser(view, grayView, 200, 200, 600);
	std::vector<double> pt1 = StegerLine0(grayView);
	std::vector<cv::Point2f> centerPoints;
	convertToVectorPoints(pt1, centerPoints);

	//可视化所有的激光线
	cv::Mat imgBinarized;
	imgBinarized = grayView.clone();
	for (int i = 0; i < imgBinarized.rows; i++) {
		for (int j = 0; j < imgBinarized.cols; j++) {
			imgBinarized.at<uchar>(i, j) = 0;
		}
	}
	for (int i = 0; i < pt1.size() / 2; i++) {
		double x, y;
		x = pt1[2 * i + 0];
		y = pt1[2 * i + 1];
		imgBinarized.at<uchar>(y, x) = 255;
	}
	cv::imwrite("gray.bmp", imgBinarized);

	//求解出对应的V型所对应的点
	cv::Rect rect(cv::Point2i(500, 500), cv::Point2i(800, 800));
	cv::Mat VMat = getVPoints(imgBinarized, rect);
	vector<cv::Vec4i> lines;
	HoughLinesP(VMat, lines, 1, CV_PI / 180, 50, 0, 50);
	double inscrement = 1;
	while (lines.size() > 2)
	{
		//std::cout << "图像中的直线不止两条，请重新规划阈值" << std::endl;
		HoughLinesP(VMat, lines, 1.5 + 0.2*inscrement, CV_PI / 180, 50+inscrement*10, 10 * inscrement, 50+10*inscrement);
		inscrement++;
	}
	while (lines.size() < 2) {
		HoughLinesP(VMat, lines, 1.5 - 0.2 * inscrement, CV_PI / 180, 50 - inscrement * 10, 0, 50 - 10 * inscrement);
		inscrement++;
	}
	//构建一张图片可视化直线
	cv::Mat viewMat = VMat.clone();
	for (int i = 0; i < viewMat.rows; i++) {
		for (int j = 0; j < viewMat.cols; j++) {
			viewMat.at<uchar>(i, j) = 0;
		}
	}
	for (int i = 0; i < lines.size(); i++) {
		cv::line(viewMat, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), 255);
		//std::cout << "point1:" << lines[i][0] << lines[i][1] << std::endl;
		//std::cout << "point2:" << lines[i][2] << lines[i][3] << std::endl;
	}

	cv::Vec4d result = lines_intersection(lines[0], lines[1]);
	cv::Point2f pointInAngularBisector = getPointInAngularBisector(lines, result);
	cv::line(viewMat, cv::Point(result[1], result[2]), pointInAngularBisector,255);

	
	res.push_back(cv::Point2f(result[1] + rect.x, result[2] + rect.y));
	res.push_back(pointInAngularBisector);


	return res;
}


