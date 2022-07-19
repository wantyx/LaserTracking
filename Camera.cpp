#include "Camera.h"

Camera::Camera()
{
	unsigned int nSupportedTls = tlFactory.EnumerateTls();
	if (MV_GIGE_DEVICE == (nSupportedTls & MV_GIGE_DEVICE)) {
		MV_CC_DEVICE_INFO_LIST setDevList;
		memset(&setDevList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
		nRet = tlFactory.EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, setDevList);
		if (0 != nRet) {
			std::cout << "EnumDevice错误!" << std::endl;
			return;
		}
		if (setDevList.nDeviceNum == 0) {
			std::cout << "未找到相机设备" << std::endl;
			return;
		}
		//判断检测到的第一台设备是否能够使用
		if (!tlFactory.IsDeviceAccessible(*(setDevList.pDeviceInfo[0]))) {
			std::cout << "当前相机不可用" << std::endl;
			return;
		}
		MV_CC_CreateHandle(&m_handler, setDevList.pDeviceInfo[0]);//创建相机句柄
		nRet = MV_CC_OpenDevice(m_handler);  //通过句柄打开相机
		MV_CC_SetFloatValue(m_handler, "TriggerDelay", 0 * 1000000);//设置触发延时，连续模式下无关紧要
		//获取当前相机的像素大小
		MVCC_INTVALUE stParam;
		memset(&stParam, 0, sizeof(MVCC_INTVALUE));
		nRet = MV_CC_GetIntValue(m_handler, "PayloadSize", &stParam);
		g_nPayloadSize = stParam.nCurValue;

		nRet = MV_CC_StartGrabbing(m_handler); //开始抓取
		memset(&stInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
		pData = (unsigned char*)malloc(sizeof(unsigned char) * (g_nPayloadSize));
	}
}

Camera::~Camera()
{

}

bool isColor(int enType) {
	return (enType == PixelType_Gvsp_BGR8_Packed or enType == PixelType_Gvsp_YUV422_Packed
		or enType == PixelType_Gvsp_YUV422_YUYV_Packed or enType == PixelType_Gvsp_BayerGR8
		or enType == PixelType_Gvsp_BayerRG8 or enType == PixelType_Gvsp_BayerGB8
		or enType == PixelType_Gvsp_BayerBG8 or enType == PixelType_Gvsp_BayerGB10
		or enType == PixelType_Gvsp_BayerGB10_Packed or enType == PixelType_Gvsp_BayerBG10
		or enType == PixelType_Gvsp_BayerBG10_Packed or enType == PixelType_Gvsp_BayerRG10
		or enType == PixelType_Gvsp_BayerRG10_Packed or enType == PixelType_Gvsp_BayerGR10
		or enType == PixelType_Gvsp_BayerGR10_Packed or enType == PixelType_Gvsp_BayerGB12
		or enType == PixelType_Gvsp_BayerGB12_Packed or enType == PixelType_Gvsp_BayerBG12
		or enType == PixelType_Gvsp_BayerBG12_Packed or enType == PixelType_Gvsp_BayerRG12
		or enType == PixelType_Gvsp_BayerRG12_Packed or enType == PixelType_Gvsp_BayerGR12
		or enType == PixelType_Gvsp_BayerGR12_Packed or enType == PixelType_Gvsp_RGB8_Packed);
}

bool isMono(int enType) {
	return(enType == PixelType_Gvsp_Mono10 or enType == PixelType_Gvsp_Mono10_Packed
		or enType == PixelType_Gvsp_Mono12 or enType == PixelType_Gvsp_Mono12_Packed
		or enType == PixelType_Gvsp_Mono8);
}

int RGB2BGR(unsigned char* pRgbData, unsigned int nWidth, unsigned int nHeight)
{
	if (NULL == pRgbData)
	{
		return MV_E_PARAMETER;
	}

	for (unsigned int j = 0; j < nHeight; j++)
	{
		for (unsigned int i = 0; i < nWidth; i++)
		{
			unsigned char red = pRgbData[j * (nWidth * 3) + i * 3];
			pRgbData[j * (nWidth * 3) + i * 3] = pRgbData[j * (nWidth * 3) + i * 3 + 2];
			pRgbData[j * (nWidth * 3) + i * 3 + 2] = red;
		}
	}

	return MV_OK;
}

bool Convert2Mat(MV_FRAME_OUT_INFO_EX* pstImageInfo, unsigned char* pData, cv::Mat& resMat)
{
	cv::Mat srcImage;
	if (isMono(pstImageInfo->enPixelType))
	{
		srcImage = cv::Mat(pstImageInfo->nHeight, pstImageInfo->nWidth, CV_8UC1, pData);
	}
	else if (isColor(pstImageInfo->enPixelType))
	{
		RGB2BGR(pData, pstImageInfo->nWidth, pstImageInfo->nHeight);
		srcImage = cv::Mat(pstImageInfo->nHeight, pstImageInfo->nWidth, CV_8UC3, pData);
	}
	else
	{
		printf("unsupported pixel format\n");
		return false;
	}

	if (NULL == srcImage.data)
	{
		return false;
	}
	resMat = srcImage.clone();
	srcImage.release();
	return true;
}

cv::Mat Camera::getFrame()
{

	cv::Mat resMat;
	if(!m_handler){
		std::cout << "未找到相机设备！" << std::endl;
		return resMat;
	}
	if (MV_CC_GetOneFrameTimeout(m_handler,pData,g_nPayloadSize,&stInfo,300) == MV_OK){
		//std::cout << "Get One Frame :Width:" << stInfo.nWidth << "Height:" << stInfo.nHeight << "nFrameNum:" << stInfo.nFrameNum << std::endl;
	}
	else {
		std::cout << "未捕获到对应的图片信息" << std::endl;
		return resMat;
	}
	MV_CC_PIXEL_CONVERT_PARAM stConvertParam = MV_CC_PIXEL_CONVERT_PARAM();
	memset(&(stConvertParam), 0, sizeof(MV_CC_PIXEL_CONVERT_PARAM));
	
	//如果相机拍摄出来是灰度图
	if (isMono(stInfo.enPixelType)) {
		unsigned char* pDataForGray = (unsigned char*)malloc(stInfo.nWidth * stInfo.nHeight + 1);
		auto nConvertDataSize = stInfo.nWidth * stInfo.nHeight * 1;
		stConvertParam.nWidth = stInfo.nWidth;
		stConvertParam.nHeight = stInfo.nHeight;
		stConvertParam.pSrcData = pData;
		stConvertParam.nSrcDataLen = stInfo.nFrameLen;
		stConvertParam.enSrcPixelType = stInfo.enPixelType;
		stConvertParam.enDstPixelType = PixelType_Gvsp_Mono8;
		stConvertParam.pDstBuffer = pDataForGray;
		stConvertParam.nDstBufferSize = nConvertDataSize;
		MV_CC_ConvertPixelType(m_handler, &stConvertParam);
		Convert2Mat(&stInfo, stConvertParam.pDstBuffer, resMat);
		free(pDataForGray);
	}
	//如果相机拍摄出来是彩色图片
	else if (isColor(stInfo.enPixelType)) {
		unsigned char* pDataForRGB = (unsigned char*)malloc(stInfo.nWidth * stInfo.nHeight * 3);
		auto nConvertDataSize = stInfo.nWidth * stInfo.nHeight * 3;
		stConvertParam.nWidth = stInfo.nWidth;
		stConvertParam.nHeight = stInfo.nHeight;
		stConvertParam.pSrcData = pData;
		stConvertParam.nSrcDataLen = stInfo.nFrameLen;
		stConvertParam.enSrcPixelType = stInfo.enPixelType;
		stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
		stConvertParam.pDstBuffer = pDataForRGB;
		stConvertParam.nDstBufferSize = nConvertDataSize;
		MV_CC_ConvertPixelType(m_handler, &stConvertParam);
		Convert2Mat(&stInfo, stConvertParam.pDstBuffer, resMat);
		free(pDataForRGB);
	}
	return resMat;

}
