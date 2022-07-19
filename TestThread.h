#pragma once
/* 类头文件 CTestThread.h */
#include<iostream>
#include<process.h>
#include<Windows.h>

class TestThread
{
public:
    TestThread();
    ~TestThread();

    int StartThread();  // 开线程
    int SetStopFlag(bool flag); //停止线程

private:
    static unsigned int WINAPI ThreadFunc(LPVOID lpParam);  //线程函数

private:
    bool m_bStopFlag;
};
