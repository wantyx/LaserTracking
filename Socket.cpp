#include "Socket.h"


bool initialization() {
	//��ʼ���׽��ֿ�
	WORD w_req = MAKEWORD(2, 2);//�汾��
	WSADATA wsadata;
	int err;
	err = WSAStartup(w_req, &wsadata);
	if (err != 0) {
		cout << "��ʼ���׽��ֿ�ʧ�ܣ�" << endl;
		return false;
	}
	else {
		cout << "��ʼ���׽��ֿ�ɹ���" << endl;
	}
	//���汾��
	if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2) {
		cout << "�׽��ֿ�汾�Ų�����" << endl;
		WSACleanup();
		return false;
	}
	else {
		cout << "�׽��ֿ�汾��ȷ��" << endl;
		return true;
	}
	
}

Socket::Socket()
{
	if (initialization()){
		//���������Ϣ
		server_addr.sin_family = AF_INET;
		server_addr.sin_addr.S_un.S_addr = inet_addr("192.168.0.100");
		//server_addr.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
		server_addr.sin_port = htons(30006);
		//�����׽���
		s_server = socket(AF_INET, SOCK_STREAM, 0);
		if (connect(s_server, (SOCKADDR*)&server_addr, sizeof(SOCKADDR)) == SOCKET_ERROR) {
			cout << "����������ʧ�ܣ�" << endl;
			WSACleanup();
		}
		else {
			cout << "���������ӳɹ���" << endl;
		}
	}

}

Socket::~Socket()
{
	//�ر��׽���
	closesocket(s_server);
	//�ͷ�DLL��Դ
	WSACleanup();
}

//�����߳�
int Socket::StartThread()
{
	HANDLE hThread = (HANDLE)_beginthreadex(NULL, 0, ThreadFunc, (LPVOID)this, 0, NULL);
	//WaitForMultipleObjects(1, &hThread, TRUE, INFINITE);
	std::cout << "ִ���߳̿�������" << std::endl;
	return 0;
}

int Socket::SetStopFlag(bool flag)
{
	stopFlag = flag;
	return 0;
}


//���߳����ڵ������͵�λ���������κη��أ�ֻ����һ��ʼ���ӽ���һ��start-OK
unsigned int WINAPI Socket::ThreadFunc(LPVOID lpParam)
{
	std::cout << "�߳���������" << std::endl;
	Socket* pthis = (Socket*)lpParam;
	//�������������ͨ��
	char p[14];
	send(pthis->s_server, pthis->start, 20, 0);
	recv(pthis->s_server, p, 20, 0);
	//pthis->flagInMain = true;//���������̵߳�whileѭ��

	while (1) {
		//while (!pthis->flag) {
		//	std::cout << "waiting" << std::endl;
		//}
		//while (pthis->flag) {
		//	//std::cout << "send" << std::endl;
		//	pthis->flag = false;
		//	const char* data = pthis->sendBuf.data();
		//	if (send(pthis->s_server, data, 100, 0) < 0) {
		//		std::cout << "����ʧ�ܣ�" << std::endl;
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
					std::cout << "����ʧ�ܣ�" << std::endl;
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

//�˷���ͨ�����ͻ�ȡ��̬��Ϣ�������ȴ���е�۷������ݣ�����string�ķ�ʽ���ظ�������
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


