#include "Eye2Hand.h"

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
}

void Eye2Hand::chess2camera()
{
	if (srcPath.empty()) {
		throw "There is no srcPath!!!";
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
			/*drawChessboardCorners(src, size, Mat(pointBuf), true);
			namedWindow("view", 0);
			imshow("view", src);
			waitKey(0);
			destroyAllWindows();*/
		}
		else {
			cout << "第" << i << "张标定图片未找到角点" << endl;
			continue;
		}
	}
	/*计算每一个点在标定板坐标系下的位置 并复制多份用于每一张图片的识别*/
	cal_pointInBoard(size, squareSize, objectPoints[0]);
	objectPoints.resize(pointBufs.size(), objectPoints[0]);
	cv::calibrateCamera(objectPoints, pointBufs, imageSize, cameraIntri, distCoffe, chess2camera_R, chess2camera_T);
}


/*从机械臂中获取末端到基坐标的RT转换*/
void Eye2Hand::end2base() {
	 double p[][6] = {
	{464.10,-440.37,90.77,178.9365,16.1724,-123.4},
	{346.45,-525.64,81.92,-176.132,42.301,-130.531},
	{379.59,-566.75,91.9,172.269,42.997,-138.912},
	{459.84,-429.66,96.44,178.35,8.6,-115.03},
	//{405.12,-410.95,67.85,-165.812,5.71,-110.69},
	{323.58,-473.92,45.65,-148.789,23.363,-97.095},
	//{488.91,-420.19,68.89,175.137,7.82,-125.32},
	{473.37,-431.83,89.49,-179.218,19.6562,-113.6023},
	{520.93,-474.12,87.91,157.2959,12.9894,-120.2483},
	//{461.55,-407.21,77.60,-179.6765,6.845,-112.648},
	{393.86,-370.15,64.14,-163.127,8.8114,-107.2023},
	{442.51,-445.44,96.28,-172.5634,7.2999,-112.527},
	//{469.88,-440.26,88.60,179.8162,12.4889,-112.3975},
	{420.92,-481.03,69.93,-170.196,32.6476,-106.405},
	{352.71,-469.80,64.33,-164.458,29.7638,-107.0845},
	{359.87,-460.33,69.92,-166.30478,14.7287,-105.5509},
	{414.30,-401.45,93.89,-175.41227,18.8998,-110.045},
	{426.72,-394.48,107.47,-173.33689,18.193,-106.7287},
	{390.58,-480.43,75.69,-167.5752,14.8072,-93.169},
	{408.19,-469.12,101.42,-175.190,33.19068,-127.1978}
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