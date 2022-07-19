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

cv::Mat calEnd2Base(double p[])
{
	using namespace cv;
	Mat R_x = (Mat_<double>(3, 3) << 1, 0, 0, 0, cos((p[3] / 180) * CV_PI), -sin((p[3] / 180) * CV_PI), 0, sin((p[3] / 180) * CV_PI), cos((p[3] / 180) * CV_PI));
	Mat R_y = (Mat_<double>(3, 3) << cos((p[4] / 180) * CV_PI), 0, sin((p[4] / 180) * CV_PI), 0, 1, 0, -sin((p[4] / 180) * CV_PI), 0, cos((p[4] / 180) * CV_PI));
	Mat R_z = (Mat_<double>(3, 3) << cos((p[5] / 180) * CV_PI), -sin((p[5] / 180) * CV_PI), 0, sin((p[5] / 180) * CV_PI), cos((p[5] / 180) * CV_PI), 0, 0, 0, 1);
	Mat R = R_z * R_y * R_x;

	Mat H = Mat::zeros(4, 4, CV_64FC1);
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			H.at<double>(i, j) = R.at<double>(i, j);
		}
		H.at<double>(i, 3) = p[i];
	}
	H.at<double>(3, 3) = 1;

	return H;
}

cv::Mat BGR2GRAYForLaser(cv::Mat bgr, cv::Mat gray, int b, int g, int r)
{
	for (int i = 0; i < bgr.rows; i++) {
		for (int j = 0; j < bgr.cols; j++) {
			int grayValue =
				(bgr.at<cv::Vec3b>(i, j)[0] * b +
				 bgr.at<cv::Vec3b>(i, j)[1] * g +
				 bgr.at<cv::Vec3b>(i, j)[2] * r)/(r+b+g);
			gray.at<uchar>(i, j) = grayValue < 200 ? 0 : grayValue;
		}
	}
	return gray;
}

std::vector<double> StegerLine0(cv::Mat grayImg)
{
	using namespace cv;
	using namespace std;
	Mat img = grayImg.clone();
	img.convertTo(img, CV_64FC1);
	GaussianBlur(img, img, Size(0, 0), 6, 6);
	//一阶偏导 进行卷积
	Mat x = (Mat_<double>(1, 2) << 1, -1);//x方向偏导
	Mat y = (Mat_<double>(2, 1) << 1, -1);//y方向偏导
	Mat dx, dy;
	filter2D(img, dx, CV_64FC1, x);
	filter2D(img, dy, CV_64FC1, y);
	//二阶偏导 进行卷积
	Mat xx = (Mat_<double>(1, 3) << 1, -2, 1);
	Mat yy = (Mat_<double>(3, 1) << 1, -2, 1);
	Mat xy = (Mat_<double>(2, 2) << 1, -1, -1, 1);
	Mat dxx, dyy, dxy;
	filter2D(img, dxx, CV_64FC1, xx);
	filter2D(img, dyy, CV_64FC1, yy);
	filter2D(img, dxy, CV_64FC1, xy);
	//hessian矩阵求解
	double maxD = -1;
	int imgcol = img.cols;
	int imgrow = img.rows;
	vector<double> Pt;
	vector<double> Pt1;
	for (int i = 0; i < imgcol; i++) {
		for (int j = 0; j < imgrow; j++) {
			if (grayImg.at<uchar>(j, i) > 190) {
				Mat hessian(2, 2, CV_64FC1);
				hessian.at<double>(0, 0) = dxx.at<double>(j, i);
				hessian.at<double>(0, 1) = dxy.at<double>(j, i);
				hessian.at<double>(1, 0) = dxy.at<double>(j, i);
				hessian.at<double>(1, 1) = dyy.at<double>(j, i);

				Mat eValue;
				Mat eVectors;
				eigen(hessian, eValue, eVectors);
				double nx, ny;
				if (fabs(eValue.at<double>(0, 0)) >= fabs(eValue.at<double>(1, 0))) {
					nx = eVectors.at<double>(0, 0);
					ny = eVectors.at<double>(0, 1);
				}
				else {
					nx = eVectors.at<double>(1, 0);
					ny = eVectors.at<double>(1, 1);
				}
				double t = -(nx * dx.at<double>(j, i) + ny * dy.at<double>(j, i)) / (nx * nx * dxx.at<double>(j, i) + 2 * nx * ny * dxy.at<double>(j, i) + ny * ny * dyy.at<double>(j, i));
				if (fabs(t * nx) <= 0.5 && fabs(t * ny) <= 0.5) {

					Pt.push_back(i);
					Pt.push_back(j);
				}
			}
		}
	}
	return Pt;
}

void convertToVectorPoints(const std::vector<double> Pt, std::vector<cv::Point2f>& vec)
{
	for (int i = 0; i < Pt.size() / 2; i++) {
		cv::Point2f point;
		point.x = Pt[i * 2 + 0];
		point.y = Pt[i * 2 + 1];
		vec.push_back(point);
	}
}

bool fitPlaneByLeastSquares(std::vector<double>& Parameters, const std::vector<cv::Point3f>& Data)
{
	using namespace Eigen;

	Parameters.clear();
	int count = Data.size();
	if (count < 3)
		return false;

	double meanX = 0, meanY = 0, meanZ = 0;
	double meanXX = 0, meanYY = 0, meanZZ = 0;
	double meanXY = 0, meanXZ = 0, meanYZ = 0;
	for (int i = 0; i < count; i++)
	{
		meanX += Data[i].x;
		meanY += Data[i].y;
		meanZ += Data[i].z;

		meanXX += Data[i].x * Data[i].x;
		meanYY += Data[i].y * Data[i].y;
		meanZZ += Data[i].z * Data[i].z;

		meanXY += Data[i].x * Data[i].y;
		meanXZ += Data[i].x * Data[i].z;
		meanYZ += Data[i].y * Data[i].z;
	}
	meanX /= count;
	meanY /= count;
	meanZ /= count;
	meanXX /= count;
	meanYY /= count;
	meanZZ /= count;
	meanXY /= count;
	meanXZ /= count;
	meanYZ /= count;

	/* eigenvector */
	Matrix3d eMat;
	eMat(0, 0) = meanXX - meanX * meanX; eMat(0, 1) = meanXY - meanX * meanY; eMat(0, 2) = meanXZ - meanX * meanZ;
	eMat(1, 0) = meanXY - meanX * meanY; eMat(1, 1) = meanYY - meanY * meanY; eMat(1, 2) = meanYZ - meanY * meanZ;
	eMat(2, 0) = meanXZ - meanX * meanZ; eMat(2, 1) = meanYZ - meanY * meanZ; eMat(2, 2) = meanZZ - meanZ * meanZ;
	Eigen::EigenSolver<Eigen::Matrix3d> xjMat(eMat);
	Matrix3d eValue = xjMat.pseudoEigenvalueMatrix();
	Matrix3d eVector = xjMat.pseudoEigenvectors();

	/* the eigenvector corresponding to the minimum eigenvalue */
	double v1 = eValue(0, 0);
	double v2 = eValue(1, 1);
	double v3 = eValue(2, 2);
	int minNumber = 0;
	if ((abs(v2) <= abs(v1)) && (abs(v2) <= abs(v3)))
	{
		minNumber = 1;
	}
	if ((abs(v3) <= abs(v1)) && (abs(v3) <= abs(v2)))
	{
		minNumber = 2;
	}
	double A = eVector(0, minNumber);
	double B = eVector(1, minNumber);
	double C = eVector(2, minNumber);
	double D = -(A * meanX + B * meanY + C * meanZ);

	/* result */
	if (C < 0)
	{
		A *= -1.0;
		B *= -1.0;
		C *= -1.0;
		D *= -1.0;
	}
	Parameters.push_back(A);
	Parameters.push_back(B);
	Parameters.push_back(C);
	Parameters.push_back(D);
	Parameters.push_back(meanX);
	Parameters.push_back(meanY);
	Parameters.push_back(meanZ);
	return true;
}

void fitPlane(const std::vector<cv::Point3f> Data, double* plane) {
	cv::Mat points = cv::Mat(Data.size(), 3, CV_64FC1);
	for (int i = 0; i < Data.size(); i++) {
		points.at<double>(i, 0) = Data.at(i).x;
		points.at<double>(i, 1) = Data.at(i).y;
		points.at<double>(i, 2) = Data.at(i).z;
	}

	using namespace cv;
	int row = points.rows;
	int col = 3;
	//求点云的质心
	Mat centroid = Mat(1, col, CV_64FC1, Scalar(0));
	for (int i = 0; i < col; i++) {
		for (int j = 0; j < row; j++) {
			centroid.at<double>(0, i) += points.at<double>(j, i);
		}
		centroid.at<double>(0, i) /= row;
	}
	Mat points2 = Mat(row, 3, CV_64FC1);
	for (int i = 0; i < row; i++) {
		for (int j = 0; j < col; j++) {
			points2.at<double>(i, j) = points.at<double>(i, j) - centroid.at<double>(0, j);
		}
	}
	Mat A = Mat(3, 3, CV_64FC1);
	Mat W = Mat(3, 3, CV_64FC1);
	Mat V = Mat(3, 3, CV_64FC1);
	Mat T = Mat(3, 3, CV_64FC1);
	/*gemm()方法：*/
	gemm(points2, points, 1, Mat(), 0, A, GEMM_1_T);
	SVD::compute(A, W, T, V, 4);
	plane[3] = 0;
	for (int i = 0; i < col; i++) {
		plane[i] = V.at<double>(2, i);
		plane[3] += plane[i] * centroid.at<double>(0, i);
	}
	std::cout << plane[0] << std::endl;
	std::cout << plane[1] << std::endl;
	std::cout << plane[2] << std::endl;
	std::cout << plane[3] << std::endl;
}

void fitLine(const std::vector<cv::Point2f>& points, std::vector<float>& line)
{
	cv::fitLine(points, line,0,0.01,0.01, 0.01);
}

