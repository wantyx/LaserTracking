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

	bool flag = false;//���ͱ�־λ��ͨ�������߳̽��п���
	bool flagInMain = true;//���Ʊ�־λ��Ϊtrueʱ���߳�ִ��whileѭ��
	bool stopFlag = false;
	const char* start = "start\0";
	const char* isStart = "start-OK";
	const char* moveOk = "move-OK";
	const char* stop = "stop";
	const char* isStop = "stop-OK";
	string sendBuf = "SENDBUF";
	string recvBuf;
	//���������׽��֣����������׽���
	SOCKET s_server;
	//����˵�ַ�ͻ��˵�ַ
	SOCKADDR_IN server_addr;

private:

};
