#include "Socket.h"


bool initialization() {
	//初始化套接字库
	WORD w_req = MAKEWORD(2, 2);//版本号
	WSADATA wsadata;
	int err;
	err = WSAStartup(w_req, &wsadata);
	if (err != 0) {
		cout << "初始化套接字库失败！" << endl;
		return false;
	}
	else {
		cout << "初始化套接字库成功！" << endl;
	}
	//检测版本号
	if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2) {
		cout << "套接字库版本号不符！" << endl;
		WSACleanup();
		return false;
	}
	else {
		cout << "套接字库版本正确！" << endl;
		return true;
	}
	
}

Socket::Socket()
{
	if (initialization()){
		//填充服务端信息
		server_addr.sin_family = AF_INET;
		server_addr.sin_addr.S_un.S_addr = inet_addr("192.168.0.100");
		//server_addr.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
		server_addr.sin_port = htons(30006);
		//创建套接字
		s_server = socket(AF_INET, SOCK_STREAM, 0);
		if (connect(s_server, (SOCKADDR*)&server_addr, sizeof(SOCKADDR)) == SOCKET_ERROR) {
			cout << "服务器连接失败！" << endl;
			WSACleanup();
		}
		else {
			cout << "服务器连接成功！" << endl;
		}
	}

}

Socket::~Socket()
{
	//关闭套接字
	closesocket(s_server);
	//释放DLL资源
	WSACleanup();
}

//开启线程
int Socket::StartThread()
{
	HANDLE hThread = (HANDLE)_beginthreadex(NULL, 0, ThreadFunc, (LPVOID)this, 0, NULL);
	//WaitForMultipleObjects(1, &hThread, TRUE, INFINITE);
	std::cout << "执行线程开启函数" << std::endl;
	return 0;
}

int Socket::SetStopFlag(bool flag)
{
	stopFlag = flag;
	return 0;
}


//此线程用于单独发送点位，不接收任何返回，只是在一开始连接接受一个start-OK
unsigned int WINAPI Socket::ThreadFunc(LPVOID lpParam)
{
	std::cout << "线程真正开启" << std::endl;
	Socket* pthis = (Socket*)lpParam;
	//首先与打标机进行通信
	char p[14];
	send(pthis->s_server, pthis->start, 20, 0);
	recv(pthis->s_server, p, 20, 0);
	//pthis->flagInMain = true;//开启其他线程的while循环

	while (1) {
		//while (!pthis->flag) {
		//	std::cout << "waiting" << std::endl;
		//}
		//while (pthis->flag) {
		//	//std::cout << "send" << std::endl;
		//	pthis->flag = false;
		//	const char* data = pthis->sendBuf.data();
		//	if (send(pthis->s_server, data, 100, 0) < 0) {
		//		std::cout << "发送失败！" << std::endl;
		//		continue;
		//	}
		//	break;
		//}
		while (true) {
			while (pthis->flag) {
				std::cout << "send" << std::endl;
				pthis->flag = false;
				const char* data = pthis->sendBuf.data();
				if (send(pthis->s_server, data, 100, 0) < 0) {
					std::cout << "发送失败！" << std::endl;
					continue;
				}
				recv(pthis->s_server, p, 14, 0);
				std::cout << "recv:" << p << std::endl;
			}
		}
		if (pthis->stopFlag) {
			std::cout << "stop" << std::endl;
			pthis->stopFlag = false;
			send(pthis->s_server, pthis->stop, 20, 0);
			recv(pthis->s_server, p, 20, 0);
			break;
		}
	}
	return 0;
}

//此方法通过发送获取姿态信息，阻塞等待机械臂返回数据，并以string的方式返回给主函数
void Socket::sendMessage(string message,double* arr)
{
	const char* getPose = message.data();
	
	if (send(s_server, getPose, 100, 0) < 0) {
		std::cout << "send failed!" << std::endl;
		return ;
	}
	char pose[100];
	recv(s_server, pose, 100, 0);
	const char* split1 = "{";
	const char* split2 = "}";
	const char* split3 = ",";
	char* buf;
	char* result = strtok_s(pose, split1,&buf);
	result = strtok_s(result, split2,&buf);
	
	int i = 0;
	while (i<6) {
		char* result1 = strtok_s(result, split3, &result);
		arr[i] = strtod(result1,&result1);
		i++;
	}
	return;
	//
	//std::cout << result << std::endl;
	//std::cout << buf << std::endl;
	//string poseString(pose);
	//return poseString;
}


