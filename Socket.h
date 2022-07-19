#pragma once
#include<iostream>
#include<winsock.h>
#include<string>
#include<iostream>
#include<process.h>
#include<Windows.h>
#pragma comment(lib,"ws2_32.lib")

using namespace std;
class Socket
{
public:
	Socket();
	~Socket();
	int StartThread();
	int SetStopFlag(bool flag);
	static unsigned int WINAPI ThreadFunc(LPVOID lpParam);
	void sendMessage(string message,double* arr);

	bool flag = false;//发送标志位，通过其他线程进行控制
	bool flagInMain = true;//控制标志位，为true时主线程执行while循环
	bool stopFlag = false;
	const char* start = "start\0";
	const char* isStart = "start-OK";
	const char* moveOk = "move-OK";
	const char* stop = "stop";
	const char* isStop = "stop-OK";
	string sendBuf = "SENDBUF";
	string recvBuf;
	//定义服务端套接字，接受请求套接字
	SOCKET s_server;
	//服务端地址客户端地址
	SOCKADDR_IN server_addr;

private:

};
