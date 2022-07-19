#include "LaserPlane.h"
#include <cmath>
using namespace std;
using namespace cv;

LaserPlane::LaserPlane()
{
	imagePathWithNoLaser = GlobalSetting::instance()->KV["imagePathWithNoLaser"];
	imagePathWithLaser = GlobalSetting::instance()->KV["imagePathWithLaser"];
	laserImageNumber = atoi(GlobalSetting::instance()->KV["laserImageNumber"].c_str());
	testImagePath = GlobalSetting::instance()->KV["testImagePath"];
}

LaserPlane::~LaserPlane()
{
}

/*
* 通过没有激光线的标定板图片，计算每一张图片对应的外参RT
*/
void LaserPlane::calibrateWithNoLaser()
{
	if (imagePathWithNoLaser.empty()) {
		std::cout << "There is no image!" << std::endl;
		return;
	}

	cv::Size imageSize;		//图片的大小
	vector<vector<Point2f>> pointBufs;  //所有标定图片的标定板上的点在图片中的像素值	
	vector<vector<Point3f>> objectPoints(1); //标定板上的点在标定板坐标系下的三维坐标

	/*   标定板参数信息   */
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
	/*开始遍历图片，并对每一站图片进行原点检测 */
	for (int i = 1; i <= laserImageNumber; i++) {
		vector<Point2f> pointBuf;
		string index = to_string(i);
		Mat src = imread(imagePathWithNoLaser + index + ".bmp");
		if (src.empty()) {
			throw "src is empty()!";
			return;
		}
		imageSize = src.size();

		bool isFound = findCirclesGrid(src, size, pointBuf, CALIB_CB_SYMMETRIC_GRID | CALIB_CB_CLUSTERING, blobDetector);
		if (isFound) {
			pointBufs.push_back(pointBuf);
			drawChessboardCorners(src, size, Mat(pointBuf), true);
			namedWindow("view", 0);
			imshow("view", src);
			waitKey(0);
			destroyAllWindows();
		}
		else {
			cout << "第" << i << "张标定图片未找到角点" << endl;
			continue;
		}
	}
	/*计算每一个点在标定板坐标系下的位置 并复制多份用于每一张图片的识别*/
	cal_pointInBoard(size, squareSize, objectPoints[0]);
	objectPoints.resize(pointBufs.size(), objectPoints[0]);
	double rms = cv::calibrateCamera(objectPoints, pointBufs, imageSize, cameraIntri, distCoffe, chess2camera_R, chess2camera_T);
	cout << "rms:" << rms << endl;
}

/*
* 通过图像处理得到激光线在图片中的像素位置 并保存到points中
*/
void LaserPlane::getLaserPoints(bool test, std::string _imagePathWithLaser ,int _laserImageNumber)
{
	if (_imagePathWithLaser.empty()) {
		std::cout << "getLaserPoints method : invalid parameters!" << std::endl;
		return;
	}
	for (int i = 1; i <= _laserImageNumber; i++) {
		std::string index = to_string(i);
		cv::Mat view1 = cv::imread(_imagePathWithLaser + index + ".bmp");
		cv::Mat view;
		undistort(view1, view, cameraIntri, distCoffe);
		if (view.empty()) {
			std::cout << "could not find image with " << _imagePathWithLaser << std::endl;
			return;
		}
		Mat undistView = view.clone();
		undistort(view, undistView, cameraIntri, distCoffe);
		view = undistView;
		/*转灰度之后得到中心线，得到中心线所在的点在图片中的像素坐标*/
		cv::Mat grayView;
		cv::cvtColor(view, grayView, COLOR_BGR2GRAY);
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
		
		cv::imwrite(index + "gray" + ".bmp", imgBinarized);

		std::vector<cv::Point2f> undistCenterPoints;
		std::vector<cv::Point2f> undistCenterPoints1;//精细化之后的点
		try
		{
			undistortPoints(centerPoints, undistCenterPoints, cameraIntri, distCoffe, cv::Mat(), cameraIntri);
		}
		catch (cv::Exception e)
		{
			std::cout << "undistortPoints method is wrong while running!" << std::endl;
			return;
		}
		
		

		if (test) {
			//测试条件下，应该将所有的像素提取到指定的三维坐标下，判断三维坐标下的z最小值
			testPoints = undistCenterPoints;
		}
		else {
			//计算点到直线的距离
			cv::Vec4f line;
			cv::fitLine(undistCenterPoints, line, cv::DIST_L2, 0, 1e-2, 1e-2);
			std::cout << line << endl;

			cv::Point point0;
			point0.x = line[2];
			point0.y = line[3];

			double k = line[1] / line[0];
			double b = -k * line[2] + line[3];

			double all_dis = 0;
			int size = 0;
			double all_dis1 = 0;
			for (int i = 0; i < undistCenterPoints.size(); i++) {
				double dis = abs(k * undistCenterPoints[i].x - undistCenterPoints[i].y + b) / sqrt(pow(k, 2) + 1);
				cout << "dist:" << dis << endl;
				if (dis < 0.3) {
					size++;
					all_dis1 += dis;
					undistCenterPoints1.push_back(undistCenterPoints[i]);
				}
				all_dis += dis;
			}
			cout << "all_dist" << all_dis << endl;

			cout << "mean_dist" << all_dis / undistCenterPoints.size() << endl;
			cout << "all_dist1" << all_dis1 << endl;
			cout << "mean_dist1" << all_dis1 / size << endl;
			points.push_back(undistCenterPoints1);
		}
		
	}
	
}

/*
* 将像素坐标转换到相机坐标系下三维坐标上
* test环境下二维到三维的转换是通过激光平面参数
* 非test环境下采用的是标定板参数进行转换
*/
void LaserPlane::uv2xyz(bool test)
{
	cv::Mat cameraIntriMat_inv = cameraIntri.inv();
	if (test) {
		if (parameters.size() < 4) {
			std::cout << "parameters' size < 4" << std::endl;
			return;
		}
		double A_caliBoard = parameters.at(0);
		double B_caliBoard = parameters.at(1);
		double C_caliBoard = parameters.at(2);
		double D_caliBoard = -parameters.at(3);
		Mat undistCenterPoint = Mat::zeros(3, 1, CV_64F);
		for (int i = 0; i < testPoints.size(); i++) {
			undistCenterPoint.at<double>(0, 0) = testPoints.at(i).x;
			undistCenterPoint.at<double>(1, 0) = testPoints.at(i).y;
			undistCenterPoint.at<double>(2, 0) = 1;
			Mat centerPointInCameraOXY = Mat::zeros(3, 1, CV_64F);
			centerPointInCameraOXY = cameraIntriMat_inv * undistCenterPoint;
			double a = centerPointInCameraOXY.at<double>(0, 0);
			double b = centerPointInCameraOXY.at<double>(1, 0);

			double X = D_caliBoard * a / (A_caliBoard * a + B_caliBoard * b + C_caliBoard);
			double Y = D_caliBoard * b / (A_caliBoard * a + B_caliBoard * b + C_caliBoard);
			double Z = D_caliBoard / (A_caliBoard * a + B_caliBoard * b + C_caliBoard);
			Point3f point;
			point.x = X;
			point.y = Y;
			point.z = Z;
			testPointsInCameraOXYZ.push_back(point);
		}
	}
	/* 非测试下 uv转xyz的方式 */
	else {
		if (chess2camera_R.size() != points.size()) {
			std::cout << "image whether has laser is not the same number!" << std::endl;
			return;
		}
		for (int i = 0; i < points.size(); i++) {
			vector<cv::Point2f> undistCenterPoints = points.at(i);
			cv::Mat rVec;
			cv::Mat tVec;
			cv::Mat rMat;
			for (int j = 0; j < undistCenterPoints.size(); j++) {
				Mat undistCenterPoint = Mat::zeros(3, 1, CV_64F);

				/*通过外参得到对应的标定板平面的参数*/
				rVec = chess2camera_R.at(i);
				tVec = chess2camera_T.at(i);
				Rodrigues(rVec, rMat);//罗德里格斯将旋转向量变换为矩阵
				double r13 = rMat.at<double>(0, 2);
				double r23 = rMat.at<double>(1, 2);
				double r33 = rMat.at<double>(2, 2);
				double t1 = tVec.at<double>(0);
				double t2 = tVec.at<double>(1);
				double t3 = tVec.at<double>(2);
				double A_caliBoard = r13;
				double B_caliBoard = r23;
				double C_caliBoard = r33;
				double D_caliBoard = r13 * t1 + r23 * t2 + r33 * t3;


				undistCenterPoint.at<double>(0, 0) = undistCenterPoints.at(j).x;
				undistCenterPoint.at<double>(1, 0) = undistCenterPoints.at(j).y;
				undistCenterPoint.at<double>(2, 0) = 1;
				Mat centerPointInCameraOXY = Mat::zeros(3, 1, CV_64F);
				centerPointInCameraOXY = cameraIntriMat_inv * undistCenterPoint;
				double a = centerPointInCameraOXY.at<double>(0, 0);
				double b = centerPointInCameraOXY.at<double>(1, 0);

				double X = D_caliBoard * a / (A_caliBoard * a + B_caliBoard * b + C_caliBoard);
				double Y = D_caliBoard * b / (A_caliBoard * a + B_caliBoard * b + C_caliBoard);
				double Z = D_caliBoard / (A_caliBoard * a + B_caliBoard * b + C_caliBoard);
				Point3f point;
				point.x = X;
				point.y = Y;
				point.z = Z;
				pointsInCameraOXYZ.push_back(point);
			}
		}
	}
	//测试蓝色点通过标定板方式得到的距离
	
}

void LaserPlane::test1() {


	Mat Tcamera;
	Mat Tundist;
	std::vector<cv::Mat> Tchess2camera_R;
	std::vector<cv::Mat> Tchess2camera_T;
	string image = "./calibrateImages/2.bmp";
	Mat src = imread(image);

	cv::Size imageSize;		//图片的大小
	vector<vector<Point2f>> pointBufs;  //所有标定图片的标定板上的点在图片中的像素值	
	vector<vector<Point3f>> objectPoints(1); //标定板上的点在标定板坐标系下的三维坐标

	/*   标定板参数信息   */
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
	vector<Point2f> pointBuf;
	if (src.empty()) {
		throw "src is empty()!";
		return;
	}
	imageSize = src.size();

	bool isFound = findCirclesGrid(src, size, pointBuf, CALIB_CB_SYMMETRIC_GRID | CALIB_CB_CLUSTERING, blobDetector);
	if (isFound) {
		pointBufs.push_back(pointBuf);
		drawChessboardCorners(src, size, Mat(pointBuf), true);
		namedWindow("view", 0);
		imshow("view", src);
		waitKey(0);
		destroyAllWindows();
	}
	cal_pointInBoard(size, squareSize, objectPoints[0]);
	objectPoints.resize(pointBufs.size(), objectPoints[0]);
	double rms = cv::calibrateCamera(objectPoints, pointBufs, imageSize, Tcamera, Tundist, Tchess2camera_R, Tchess2camera_T);
	cout << "rms:" << rms << endl;
	
	string imagePath = "./testImage/";
	Mat bluePointMat = imread(imagePath + "44_1.BMP");
	vector<Point2f> imagePoints;
	vector<Point2f> undistImagePoints;

	cv::Mat rVec;
	cv::Mat tVec;
	cv::Mat rMat;
	int i = 1;
	rVec = Tchess2camera_R.at(0);
	tVec = Tchess2camera_T.at(0);
	Rodrigues(rVec, rMat);//罗德里格斯将旋转向量变换为矩阵
	double r13 = rMat.at<double>(0, 2);
	double r23 = rMat.at<double>(1, 2);
	double r33 = rMat.at<double>(2, 2);
	double t1 = tVec.at<double>(0);
	double t2 = tVec.at<double>(1);
	double t3 = tVec.at<double>(2);
	double A = r13;
	double B = r23;
	double C = r33;
	double D = r13 * t1 + r23 * t2 + r33 * t3;

	for (int i = 0; i < bluePointMat.rows; i++) {
		for (int j = 0; j < bluePointMat.cols; j++) {
			if (bluePointMat.at<Vec3b>(i, j)[0] == 255 && bluePointMat.at<Vec3b>(i, j)[1] == 0 && bluePointMat.at<Vec3b>(i, j)[2] == 0) {
				Point2f imagePoint;
				imagePoint.x = i;
				imagePoint.y = j;
				imagePoints.push_back(imagePoint);
			}
		}
	}
	undistortPoints(pointBuf, undistImagePoints, Tcamera, Tundist, cv::Mat(), Tcamera);
	vector<double> X_vector_camera;
	vector<double> Y_vector_camera;
	vector<double> Z_vector_camera;
	for (int i = 0; i < pointBuf.size(); i++) {
		Mat imagePoint = Mat::zeros(3, 1, CV_64F);
		Mat pointInCamera = Mat::zeros(3, 1, CV_64F);
		Mat cameraIntriMat_inv = Tcamera.inv();
		imagePoint.at<double>(0, 0) = undistImagePoints.at(i).x;
		imagePoint.at<double>(1, 0) = undistImagePoints.at(i).y;
		imagePoint.at<double>(2, 0) = 1;
		pointInCamera = (cameraIntriMat_inv * imagePoint);
		double a = pointInCamera.at<double>(0, 0);
		double b = pointInCamera.at<double>(1, 0);
		X_vector_camera.push_back(D * a / (A * a + B * b + C));
		Y_vector_camera.push_back(D * b / (A * a + B * b + C));
		Z_vector_camera.push_back(D / (A * a + B * b + C));
	}
	for (int i = 0; i < X_vector_camera.size(); i++) {
		cout << "X:" << X_vector_camera.at(i) << " Y:" << Y_vector_camera.at(i) << " Z:" << Z_vector_camera.at(i) << endl;
	}
	for (int i = 0; i < X_vector_camera.size() - 1; i++) {
		double length = pow(pow(X_vector_camera.at(i) - X_vector_camera.at(i + 1), 2) + pow(Y_vector_camera.at(i) - Y_vector_camera.at(i + 1), 2) + pow(Z_vector_camera.at(i) - Z_vector_camera.at(i + 1), 2), 0.5);
		cout << length << endl;
	}
}

void LaserPlane::camera2base(Eye2Hand eye2hand,double* p)
{
	cv::Mat eye2hand_Mat = eye2hand.get_eye2handRT();
	if (eye2hand_Mat.empty()) {
		std::cout << "no eye2hand Mat exist!" << std::endl;
		return;
	}
	if (testPointsInCameraOXYZ.empty()) {
		std::cout << "no points in cameraOXYZ!" << std::endl;
		return;
	}
	cv::Mat end2base = calEnd2Base(p);
	for (int i = 0; i < testPointsInCameraOXYZ.size(); i++) {
		cv::Point3f points;
		cv::Mat temp = cv::Mat::ones(4, 1, CV_64F);
		temp.at<double>(0, 0) = testPointsInCameraOXYZ.at(i).x;
		temp.at<double>(1,0) = testPointsInCameraOXYZ.at(i).y;
		temp.at<double>(2,0) = testPointsInCameraOXYZ.at(i).z;
		cv::Mat result = end2base * eye2hand_Mat * temp;
		points.x = result.at<double>(0);
		points.y = result.at<double>(1);
		points.z = result.at<double>(2);
		testPointsInBaseOXYZ.push_back(points);
		std::cout << points << std::endl;
	}
	std::cout << testPointsInBaseOXYZ.size() << std::endl;
}

/* 通过三维点，将点映射到一个在相机坐标系下的平面中 */
void LaserPlane::getLaserPlane()
{
	if (pointsInCameraOXYZ.empty()) {
		std::cout << "pointsInCameraOXYZ is empty!" << std::endl;
		return;
	}
	fitPlaneByLeastSquares(parameters, pointsInCameraOXYZ);
	std::cout << parameters.at(0) << endl;
	std::cout << parameters.at(1) << endl;
	std::cout << parameters.at(2) << endl;
	std::cout << parameters.at(3) << endl;
	calMeanDistance();

	double p[4] = { 0,0,0,0 };
	fitPlane(pointsInCameraOXYZ, p);
	std::cout << p[0] << std::endl;
	std::cout << p[1] << std::endl;
	std::cout << p[2] << std::endl;
	std::cout << p[3] << std::endl;

}

/* 计算所有三维点到平面的平均误差 */
void LaserPlane::calMeanDistance() {
	double lens = 0;
	for (int i = 0; i < pointsInCameraOXYZ.size(); i++) {
		double len =
			(parameters.at(0) * pointsInCameraOXYZ.at(i).x +
				parameters.at(1) * pointsInCameraOXYZ.at(i).y +
				parameters.at(2) * pointsInCameraOXYZ.at(i).z +
				parameters.at(3)) / sqrt(pow(parameters.at(0), 2) + pow(parameters.at(1), 2) + pow(parameters.at(2), 2));
		//std::cout << "len:" << len << std::endl;
		lens += abs(len);
	}
	double mean = lens / pointsInCameraOXYZ.size();
	std::cout << "mean:" << mean << std::endl;
}

/*
* 得到V型槽内的点，首先将点全部映射到三维平面，求解三维平面的Z的平均值，然后得到小于该平均值的点即为V型槽内的点
*/
void LaserPlane::getVPoints()
{
	if (testPointsInBaseOXYZ.size() == 0) {
		cout << "没有三维基坐标系下的点！" << endl;
		return;
	}
	double sum = 0;
	for (int i = 0; i < testPointsInBaseOXYZ.size(); i++) {
		sum += testPointsInBaseOXYZ.at(i).z;
	}
	double avg = sum / testPointsInBaseOXYZ.size();
	vector<Point2f> VPoints;
	for (int i = 0; i < testPointsInBaseOXYZ.size(); i++) {
		if (testPointsInBaseOXYZ.at(i).z < avg) {
			VPoints.push_back(testPoints.at(i));
		}
	}
}

Line::Line(cv::Point2f point0, cv::Point2f point1, double k, double angle)
{
	this->point0 = point0;
	this->point1 = point1;
	this->k = k;
	this->angle = angle;
}

void LaserPlane::findBlue(cv::Mat eye2hand,cv::Mat cameraIntri,cv::Mat distCoffe) {


	string imagePath = "./testImage/";
	Mat src = imread(imagePath + "44_1.BMP");
	/*Mat src = src1.clone();
	undistort(src1, src, cameraIntri, distCoffe);*/
	vector<Point2f> imagePoints;
	vector<Point2f> undistImagePoints;
	Mat imagePoint = Mat::zeros(3, 1, CV_64F);
	Mat pointInCamera = Mat::zeros(3, 1, CV_64F);
	Mat cameraIntriMat_inv = cameraIntri.inv();

	double A = parameters.at(0);
	double B = parameters.at(1);
	double C = parameters.at(2);
	double D = -parameters.at(3);

	vector<double> X_vector_camera;
	vector<double> Y_vector_camera;
	vector<double> Z_vector_camera;

	int num = 0;
	for (int i = 0; i < src.rows; i++) {
		for (int j = 0; j < src.cols; j++) {
			if (src.at<Vec3b>(i, j)[0] == 255 && src.at<Vec3b>(i, j)[1] == 0 && src.at<Vec3b>(i, j)[2] == 0) {
				Point2f imagePoint;
				imagePoint.x = i;
				imagePoint.y = j;
				imagePoints.push_back(imagePoint);
			}
		}
	}
	undistortPoints(imagePoints, undistImagePoints, cameraIntri, distCoffe, cv::Mat(), cameraIntri);
	for (int i = 0; i < imagePoints.size(); i++) {
		imagePoint.at<double>(0, 0) = undistImagePoints.at(i).x;
		imagePoint.at<double>(1, 0) = undistImagePoints.at(i).y;
		imagePoint.at<double>(2, 0) = 1;
		pointInCamera = (cameraIntriMat_inv * imagePoint);
		double a = pointInCamera.at<double>(0, 0);
		double b = pointInCamera.at<double>(1, 0);
		X_vector_camera.push_back(D * a / (A * a + B * b + C));
		Y_vector_camera.push_back(D * b / (A * a + B * b + C));
		Z_vector_camera.push_back(D / (A * a + B * b + C));
	}
	for (int i = 0; i < X_vector_camera.size(); i++) {
		cout << "X:" << X_vector_camera.at(i) << " Y:" << Y_vector_camera.at(i) << " Z:" << Z_vector_camera.at(i) << endl;
	}
	for (int i = 0; i < X_vector_camera.size() - 1; i++) {
		double length = pow(pow(X_vector_camera.at(i) - X_vector_camera.at(i + 1), 2) + pow(Y_vector_camera.at(i) - Y_vector_camera.at(i + 1), 2) + pow(Z_vector_camera.at(i) - Z_vector_camera.at(i + 1), 2), 0.5);
		cout << length << endl;
	}

	double p[6] = {144.18,624.75,87.48,-164.497,-7.253,-24.717 };
	//double p[6] = { 497.49,853.59,203.09,-140.197,-10.195,-43.909 };
	Mat end2BaseRT = calEnd2Base(p);

	for (int i = 0; i < X_vector_camera.size(); i++) {
		Mat point1 = Mat(4, 1, CV_64F);
		point1.at<double>(0, 0) = X_vector_camera.at(i);
		point1.at<double>(1, 0) = Y_vector_camera.at(i);
		point1.at<double>(2, 0) = Z_vector_camera.at(i);
		point1.at<double>(3, 0) = 1;
		cout << point1 << endl;
	}

	vector<Mat> pointsInBase;
	for (int i = 0; i < X_vector_camera.size(); i++) {
		Mat firstPoint = Mat(4, 1, CV_64F);
		firstPoint.at<double>(0, 0) = X_vector_camera.at(i);
		firstPoint.at<double>(1, 0) = Y_vector_camera.at(i);
		firstPoint.at<double>(2, 0) = Z_vector_camera.at(i);
		firstPoint.at<double>(3, 0) = 1;
		Mat result = end2BaseRT * eye2hand * firstPoint;
		cout << "result:" << result << endl;
		pointsInBase.push_back(result);
	}
	



//Mat firstPointInEnd = eye2hand * firstPoint;
//Mat lastPointInEnd = eye2hand * lastPoint;
//cout << firstPointInEnd << endl;
//cout << lastPointInEnd << endl;

//
//
//Mat firstPointInBase = end2BaseRT * firstPointInEnd;
//Mat lastPointInBase = end2BaseRT * lastPointInEnd;

//cout << firstPointInBase << endl;
//cout << lastPointInBase << endl;

}