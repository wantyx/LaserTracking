#pragma once
#include <TlFactory.h>
#include "CameraParams.h"
#include "MvCameraControl.h"
#include <opencv2/opencv.hpp>
using namespace MvCamCtrl;

class Camera
{
public:
	Camera();
	~Camera();
	cv::Mat getFrame();	
	//´´½¨Í¼Ïñ»º´æ
	MV_FRAME_OUT_INFO_EX stInfo = { 0 };
	unsigned int          g_nPayloadSize = 0;
	unsigned char* pData;


	int nRet = 1;
	CTlFactory& tlFactory = CTlFactory::GetInstance();
	
	void* m_handler = NULL;
private:

};


