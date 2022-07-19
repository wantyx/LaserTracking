#include "LaserPlaneWithOneImage.h"

using namespace std;
using namespace cv;
LaserPlaneWithOneImage::LaserPlaneWithOneImage()
{
	if (planeImagePath.empty()) {
		std::cout << "����ͼƬ�ļ�Ϊ�գ�" << std::endl;
		return;
	}

	cv::Size imageSize;		//ͼƬ�Ĵ�С
	vector<vector<Point2f>> pointBufs;  //���б궨ͼƬ�ı궨���ϵĵ���ͼƬ�е�����ֵ	
	vector<vector<Point3f>> objectPoints(1); //�궨���ϵĵ��ڱ궨������ϵ�µ���ά����

	/*   �궨�������Ϣ   */
	cv::Size size(11, 9);
	SimpleBlobDetector::Params params;
	float squareSize = 4;
	params.minArea = 30;
	params.maxArea = 18000;
	params.minDistBetweenBlobs = 5;
	params.filterByColor = true;
	params.blobColor = 0;
	params.filterByArea = true;
	Ptr<FeatureDetector> blobDetector = SimpleBlobDetector::create(params);

	cv::Mat planeImage = cv::imread(planeImagePath);
	if (planeImage.empty()) {
		std::cout << "�Ҳ���ͼƬ·���¶�Ӧ��ͼƬ��Ϣ��" << std::endl;
		return;
	}
	imageSize = planeImage.size();
	vector<Point2f> pointBuf;
	if (cv::findCirclesGrid(planeImage, size, pointBuf, CALIB_CB_SYMMETRIC_GRID | CALIB_CB_CLUSTERING, blobDetector)){
		pointBufs.push_back(pointBuf);
		drawChessboardCorners(planeImage, size, Mat(pointBuf), true);
		namedWindow("view", 0);
		imshow("view", planeImage);
		waitKey(0);
		destroyAllWindows();
	}
	else {
		std::cout << "û���ҵ���Ӧ�����̸���Ϣ��" << std::endl;
	}
	cal_pointInBoard(size, squareSize, objectPoints[0]);
	objectPoints.resize(pointBufs.size(), objectPoints[0]); 
	double rms = cv::calibrateCamera(objectPoints, pointBufs, imageSize, cameraIntri, distCoffe, chess2camera_R, chess2camera_T);
	std::cout << "��ͼƬRMS��" << rms << std::endl;
	/*ͨ������ķ�ʽ���м���������̸�ͼ�������ı궨�����A,B,C,D*/
	cv::Mat rVec;
	cv::Mat tVec;
	cv::Mat rMat;
	rVec = chess2camera_R.at(0);
	tVec = chess2camera_T.at(0);
	Rodrigues(rVec, rMat);//�޵����˹����ת�����任Ϊ����
	double r13 = rMat.at<double>(0, 2);
	double r23 = rMat.at<double>(1, 2);
	double r33 = rMat.at<double>(2, 2);
	double t1 = tVec.at<double>(0);
	double t2 = tVec.at<double>(1);
	double t3 = tVec.at<double>(2);
	A_caliBoard = r13;
	B_caliBoard = r23;
	C_caliBoard = r33;
	D_caliBoard = r13 * t1 + r23 * t2 + r33 * t3;
	std::cout << "A:" << A_caliBoard << std::endl;
	std::cout << "B:" << B_caliBoard << std::endl;
	std::cout << "C:" << C_caliBoard << std::endl;
	std::cout << "D:" << D_caliBoard << std::endl;
}

LaserPlaneWithOneImage::~LaserPlaneWithOneImage()
{
}

cv::Point3f LaserPlaneWithOneImage::getPointInCameraOXY(cv::Point point)
{
	cv::Mat cameraIntriMat_inv = cameraIntri.inv();
	Mat undistCenterPoint = Mat::zeros(3, 1, CV_64F);
	Mat centerPointInCameraOXY = Mat::zeros(3, 1, CV_64F);
	undistCenterPoint.at<double>(0, 0) = point.x;
	undistCenterPoint.at<double>(1, 0) = point.y;
	undistCenterPoint.at<double>(2, 0) = 1;
	centerPointInCameraOXY = cameraIntriMat_inv * undistCenterPoint;
	double a = centerPointInCameraOXY.at<double>(0, 0);
	double b = centerPointInCameraOXY.at<double>(1, 0);
	double X = D_caliBoard * a / (A_caliBoard * a + B_caliBoard * b + C_caliBoard);
	double Y = D_caliBoard * b / (A_caliBoard * a + B_caliBoard * b + C_caliBoard);
	double Z = D_caliBoard / (A_caliBoard * a + B_caliBoard * b + C_caliBoard);
	Point3f pointInCameraOXY;
	pointInCameraOXY.x = X;
	pointInCameraOXY.y = Y;
	pointInCameraOXY.z = Z;
	return pointInCameraOXY;
}
