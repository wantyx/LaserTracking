// LaserTracking.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include "GlobalSetting.h"
#include "Eye2Hand.h"
#include "LaserPlane.h"
#include "LaserImage.h"
#include "LaserPlaneWithOneImage.h"
#include "Camera.h"
#include "Socket.h"
#include "TestThread.h"

int main()
{

	/* 首先必须进行手眼标定 得到相机与末端之间的转换关系 */
	Eye2Hand eye2hand;
	eye2hand.doEye2Hand();

	/* 构建激光平面在相机坐标系下的位置，构造方法中计算出该平面对应的A,B,C,D*/
	LaserPlaneWithOneImage laserPlaneWithOneImage;

	/* 带有激光线的焊缝图像类 后续通过该类中方法findLowestPoint()得到对应的焊缝的最低点*/
	LaserImage laserImage;

	/* 与机械臂示教器通信的Socket */
	Socket s;
	s.StartThread();


	/* 相机类，通过getFrame()方法捕获图像，并将图像传递给LaserImage类的对象方法中*/
	Camera camera;

	/* 构建一个虚拟的工具，求解出工具坐标系的标定，且末端在第一次拍摄激光焊缝位置的y-1,X-1的位置*/
	cv::Mat src = camera.getFrame();
	cv::Point2f lowestPoint = laserImage.findLowestPoint(src).at(0);
	cv::Point3f lowestPointInCameraOXYZ = laserPlaneWithOneImage.getPointInCameraOXY(lowestPoint);
	cv::Mat eye2handMat = eye2hand.get_eye2handRT();
	cv::Mat temp = cv::Mat::ones(4, 1, CV_64F);
	temp.at<double>(0, 0) = lowestPointInCameraOXYZ.x ;
	temp.at<double>(1, 0) = lowestPointInCameraOXYZ.y ;
	temp.at<double>(2, 0) = lowestPointInCameraOXYZ.z ;
	cv::Mat tool = eye2handMat * temp;
	//只能在这里对tool进行处理，要保证工具坐标系和末端坐标系的x，y，z三个方向的一致性
	//假设当前tool在x的-10
	tool.at<double>(0, 0) -= 2;
	std::cout << tool << std::endl;


	double p[6];
	cv::Point3f lastPoint;
	lastPoint.x = 0;
	lastPoint.y = 0;
	lastPoint.z = 0;

	while (1) {
		while (s.flagInMain) {
			cv::Mat src = camera.getFrame();
			cv::Mat eye2handMat = eye2hand.get_eye2handRT();

			cv::Mat result0 = cv::Mat::ones(4, 1, CV_64F);
			cv::Mat result1 = cv::Mat::ones(4, 1, CV_64F);
			 
			vector<cv::Point2f> lowestPointAndAngularB = laserImage.findLowestPoint(src);
			cv::Point3f lowestPoint3D;
			cv::Point3f AngularBisector;
			for (int i = 0; i < lowestPointAndAngularB.size(); i++) {
				cv::Point3f lowestPointInCameraOXYZ = laserPlaneWithOneImage.getPointInCameraOXY(lowestPointAndAngularB.at(i));
				cv::Mat temp = cv::Mat::ones(4, 1, CV_64F);
				temp.at<double>(0, 0) = lowestPointInCameraOXYZ.x;
				temp.at<double>(1, 0) = lowestPointInCameraOXYZ.y;
				temp.at<double>(2, 0) = lowestPointInCameraOXYZ.z;
				cv::Mat end = eye2handMat * temp;
				//cv::Mat result = end2base * end;


				switch (i) {
				case 0:
					result0.at<double>(0, 0) = end.at<double>(0, 0) - tool.at<double>(0, 0);
					result0.at<double>(1, 0) = end.at<double>(1, 0) - tool.at<double>(1, 0);
					result0.at<double>(2, 0) = end.at<double>(2, 0) - tool.at<double>(2, 0);
					lowestPoint3D.x = result0.at<double>(0, 0);
					lowestPoint3D.y = result0.at<double>(1, 0);
					lowestPoint3D.z = result0.at<double>(2, 0);

					break;
				case 1:
					result1.at<double>(0, 0) = end.at<double>(0, 0) - tool.at<double>(0, 0);
					result1.at<double>(1, 0) = end.at<double>(1, 0) - tool.at<double>(1, 0);
					result1.at<double>(2, 0) = end.at<double>(2, 0) - tool.at<double>(2, 0);
					AngularBisector.x = result1.at<double>(0, 0);
					AngularBisector.y = result1.at<double>(1, 0);
					AngularBisector.z = result1.at<double>(2, 0);
				}
			}


			s.sendMessage("pose", p);
			for (int index = 0; index < 3; index++) {
				p[index] = 1000 * p[index];
			}
			for (int index = 3; index < 6; index++) {
				p[index] = p[index] / 3.14 * 180;
			}
			cv::Mat end2base = calEnd2Base(p);

			cv::Mat temp0 = end2base * result0 ;
			cv::Mat temp1 = end2base * result1 ;

			cv::Point3f np;
			np.x = temp0.at<double>(0,0) - temp1.at<double>(0, 0);
			np.y = temp0.at<double>(1,0) - temp1.at<double>(1, 0);
			np.z = temp0.at<double>(2, 0) - temp1.at<double>(2, 0);
			//double cosZ = np.z / (sqrt(pow(np.x, 2) + pow(np.y, 2) + pow(np.z, 2)));
			double cosX = np.x / (sqrt(pow(np.x, 2) + pow(np.y, 2) + pow(np.z, 2)));
			//double cosY = np.y / (sqrt(pow(np.x, 2) + pow(np.y, 2) + pow(np.z, 2)));
			//double angleZ = acos(cosZ);
			double angleX = acos(cosX) ;
			//double angleY = acos(cosY);
			cv::Mat temp = cv::Mat::ones(4, 1, CV_64F);
			temp.at<double>(0, 0) = lowestPoint3D.x;
			temp.at<double>(1, 0) = lowestPoint3D.y;
			temp.at<double>(2, 0) = lowestPoint3D.z;

			//在这里获取当前的机械臂位姿
			//string pose = s.sendMessage("getPose");

			std::cout << "" << std::endl;
			cv::Mat location = end2base * temp;
			cv::Point3f pointWithP;
			pointWithP.x = location.at<double>(0, 0);
			pointWithP.y = location.at<double>(1, 0);
			pointWithP.z = location.at<double>(2, 0);
			p[0] = pointWithP.x;
			p[1] = pointWithP.y;
			p[2] = pointWithP.z;
			//p[3] = angleX;
			//p[4] = angleY;
			//p[5] = angleZ;

			std::ostringstream stringsteam;
			stringsteam << "{" << p[0] / 1000 << "," << p[1] / 1000 << "," << (p[2]) / 1000 << "," << (p[3] / 180) * 3.14 << "," << (p[4] / 180) * 3.14 << "," << (p[5] / 180) * 3.14 << "}";
			s.sendBuf = std::string(stringsteam.str());
			std::cout << "send:" << "{" << p[0] / 1000 << "," << p[1] / 1000 << "," << (p[2]) / 1000 << "," << (p[3] / 180) * 3.14 << "," << (p[4] / 180) * 3.14 << "," << (p[5] / 180) * 3.14 << "}" << std::endl;
			lastPoint = pointWithP;
			s.flag = true;
			//如果没有在示教器上设置工具坐标系，就需要在代码中自己转坐标；如果在示教器中已经设置，就直接传输最低点和位姿信息就可以了
			//end.at<double>(0, 0) -= tool.at<double>(0, 0);
			//end.at<double>(1, 0) -= tool.at<double>(1, 0);
			//end.at<double>(2, 0) -= tool.at<double>(2, 0);
			
			//当下一个点与当前点的位置关系相差较大时候，更新下一个点
			//if (sqrt(pow(lastPoint.x - pointWithP.x, 2) + pow(lastPoint.y - pointWithP.y, 2) + pow(lastPoint.z - pointWithP.z, 2)) > 5) {
			//	//更新当前的p位置
			//	
			//}
			
		}
	}
	
	
#ifdef pre
	
	//LaserPlane laserPlane;
	//laserPlane.calibrateWithNoLaser();
	//laserPlane.getLaserPoints(false,laserPlane.imagePathWithLaser,laserPlane.laserImageNumber);
	//laserPlane.uv2xyz(false);
	//laserPlane.getLaserPlane();
	//laserPlane.test1();
	//laserPlane.findBlue(eye2hand.get_eye2handRT(),eye2hand.getCameraIntri(),eye2hand.getDistCoffe());
	LaserPlaneWithOneImage laserPlaneWithOneImage;
	LaserImage laserImage;
	cv::Point2f lowestPoint = laserImage.findLowestPoint();
	//cv::Point2f testPoint(1056, 583);
	//cv::Point2f testPoint1(1156, 584);
	cv::Point2f testPoint(636, 628);
	//cv::Point2f testPoint1(676, 674);
	cv::Point3f lowestPointInCamera = laserPlaneWithOneImage.getPointInCameraOXY(testPoint);
	//cv::Point3f lowestPointInCamera1 = laserPlaneWithOneImage.getPointInCameraOXY(testPoint1);
	cv::Mat eye2handMat = eye2hand.get_eye2handRT();
	//double p[6] = {-42.92,814.91,48.30,-179.700,-3.1599,30.334};
	double p[6] = { -12.01,711.61,117.23,-154.808,-8.504,44.028 };
	cv::Mat end2base = calEnd2Base(p);
	cv::Mat temp = cv::Mat::ones(4, 1, CV_64F);
	cv::Mat temp1 = cv::Mat::ones(4, 1, CV_64F);
	temp.at<double>(0, 0) = lowestPointInCamera.x;
	temp.at<double>(1, 0) = lowestPointInCamera.y;
	temp.at<double>(2, 0) = lowestPointInCamera.z;
	//temp1.at<double>(0, 0) = lowestPointInCamera1.x;
	//temp1.at<double>(1, 0) = lowestPointInCamera1.y;
	//temp1.at<double>(2, 0) = lowestPointInCamera1.z;
	cv::Mat result = end2base * eye2handMat * temp;
	//cv::Mat result1 = end2base * eye2handMat * temp1;
	std::cout << "resutl:" << result << std::endl;
	//std::cout << "resutl1:" << result1 << std::endl;
#endif // pre

    
}
