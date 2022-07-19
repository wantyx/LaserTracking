#include "Eye2Hand.h"

using namespace cv;
using namespace std;
//手眼标定类构造函数
Eye2Hand::Eye2Hand()
{
	srcPath = GlobalSetting::instance()->KV["eye2hand_srcImagePath"];
	imgNumber = atoi(GlobalSetting::instance()->KV["eye2hand_imgNumber"].c_str());
}

Eye2Hand::~Eye2Hand()
{
}

cv::Mat Eye2Hand::get_eye2handRT()
{
	return eye2handRT;
}

cv::Mat Eye2Hand::getCameraIntri()
{
	return cameraIntri;
}

cv::Mat Eye2Hand::getDistCoffe()
{
	return distCoffe;
}

void Eye2Hand::doEye2Hand()
{
	chess2camera();
	end2base();
	Mat eye2hand_R;
	Mat eye2hand_T;
	cv::calibrateHandEye(end2baseRs, end2baseTs, chess2camera_R, chess2camera_T, eye2hand_R, eye2hand_T);
	Mat RT = Mat(3, 4, CV_64F);
	Mat temp = Mat::zeros(1, 4, CV_64F);
	temp.at<double>(0, 3) = 1;
	eye2handRT = Mat(4, 4, CV_64F);
	hconcat(eye2hand_R, eye2hand_T, RT);
	vconcat(RT, temp, eye2handRT);
	std::cout << eye2handRT << std::endl;
	uv2xyz();
}

void Eye2Hand::chess2camera()
{
	if (srcPath.empty()) {
		throw "There is no srcPath!!!";
		return;
	}
	cv::Size imageSize;		//图片的大小

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
	for (int i = 1; i <= imgNumber; i++) {
		vector<Point2f> pointBuf;
		string index = to_string(i);
		Mat src = imread(srcPath + index + ".bmp");
		if (src.empty()) {
			throw "src is empty()!";
			return;
		}
		imageSize = src.size();

		bool isFound = findCirclesGrid(src, size, pointBuf, CALIB_CB_SYMMETRIC_GRID | CALIB_CB_CLUSTERING, blobDetector);
		if (isFound) {
			pointBufs.push_back(pointBuf);
			//drawChessboardCorners(src, size, Mat(pointBuf), true);
			//namedWindow("view", 0);
			//imshow("view", src);
			//waitKey(0);
			//destroyAllWindows();
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
	std::cout << "rms:" << rms << std::endl;
}


/*从机械臂中获取末端到基坐标的RT转换*/
void Eye2Hand::end2base() {
	 double p[][6] = {
		 /*{-196.44,469.58,34.21,-156.204,-31.800,14.471},*/
		 //{-128.45,523.96,37.34,-168.413,-21.53,8.937},
		 //{-105.18,528.93,47.14,-166.455,0.660,23.143},
		 {-205.25,560.77,49,171.261,-35.85,28.807},
		 {-131.54,522.73,103.78,-170.47,-12.805,26.184},
		 {-190.79,505.61,99.48,-172.88,-45.799,35.847},
		 {-145.19,547.73,104.59,-179.27,-13.24,18.918},
		 {-120.79,551.81,104.18,177.381,-29.926,21.382},
		 {-91.71,516.98,73.10,-160.789,-12.564,13.856},
		 {-118.68,521.94,81.66,-169.516,-27.88,16.64},
		 {-89.42,567.8,66.97,-174.17,3.226,14.619},
		 {-116.70,572.89,75.89,-170.04,-20.258,19.280}
	};
	/* 将上面的p全部转换为矩阵，存到对应的vector中 */
	for (int i = 0; i < sizeof(p) / sizeof(p[0]); i++) {
		Mat end2base_angle_R = calculateH(p[i]);
		Mat end2base_R;
		cv::Rodrigues(end2base_angle_R, end2base_R);
		end2baseRs.push_back(end2base_R);
		end2baseTs.push_back(calculateT(p[i]));
	}
}

void Eye2Hand::uv2xyz()
{
	Mat cameraIntriMat_inv = cameraIntri.inv();
	for (int i = 0; i < chess2camera_R.size(); i++) {
		vector<cv::Point2f> undistCenterPoints = pointBufs.at(i);
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
			Mat pp = Mat(4, 1, CV_64F);
			pp.at<double>(0, 0) = X;
			pp.at<double>(1, 0) = Y;
			pp.at<double>(2, 0) = Z;
			pp.at<double>(3, 0) = 1;
			//cout << point << endl;
			double p[][6] = {
			 {-205.25,560.77,49,171.261,-35.85,28.807},
			 {-131.54,522.73,103.78,-170.47,-12.805,26.184},
			 {-190.79,505.61,99.48,-172.88,-45.799,35.847},
			 {-145.19,547.73,104.59,-179.27,-13.24,18.918},
			 {-120.79,551.81,104.18,177.381,-29.926,21.382},
			 {-91.71,516.98,73.10,-160.789,-12.564,13.856},
			 {-118.68,521.94,81.66,-169.516,-27.88,16.64},
			 {-89.42,567.8,66.97,-174.17,3.226,14.619},
			 {-116.70,572.89,75.89,-170.04,-20.258,19.280}
			};
			for (int i = 0; i < sizeof(p) / sizeof(p[0]); i++) {
				end2baseRT.push_back(calEnd2Base(p[i]));
			}
			Mat result = end2baseRT.at(i) * eye2handRT * pp;
			//cout << "point:" << result << endl;
		}
		//cout << "====================================================" << endl;
	}
	//cout << "====================================================" << endl;
}
