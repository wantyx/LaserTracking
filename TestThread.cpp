/* 类源文件 CTestThread.cpp */

#include "TestThread.h"

TestThread::TestThread()
{
    m_bStopFlag = false;
}

TestThread::~TestThread()
{
    m_bStopFlag = true;

}


int TestThread::StartThread()
{
    HANDLE hThread = (HANDLE)_beginthreadex(NULL, 0, &ThreadFunc, (LPVOID)this, 0, NULL);
    std::cout << "执行线程开启函数" << std::endl;

    return 0;
}

int TestThread::SetStopFlag(bool flag)
{
    m_bStopFlag = flag;
    return 0;
}

unsigned int WINAPI TestThread::ThreadFunc(LPVOID lpParam)
{
    std::cout << "=======" << std::endl;
    while (1) {
        std::cout << "=======" << std::endl;
    }
    TestThread* pthis = (TestThread*)lpParam;
    while (!pthis->m_bStopFlag)
    {
        //printf("ThreadFunc is running cassid is %d .\n", pthis->m_classid);
        Sleep(1000);
    }
    printf("ThreadFunc return.\n");
    return 0;
}