#pragma once
/* ��ͷ�ļ� CTestThread.h */
#include<iostream>
#include<process.h>
#include<Windows.h>

class TestThread
{
public:
    TestThread();
    ~TestThread();

    int StartThread();  // ���߳�
    int SetStopFlag(bool flag); //ֹͣ�߳�

private:
    static unsigned int WINAPI ThreadFunc(LPVOID lpParam);  //�̺߳���

private:
    bool m_bStopFlag;
};
