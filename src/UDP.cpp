/******************************************************************************\
* Copyright (C) 2019 . 江西省智能信息系统重点实验室, All rights reserved.		*
* Version: 1.0																	*
* Last Revised: 2019-12-07														*
* Editor: Luozu																	*
* UDP服务器端实现：发送关节角及抓握信息。注：关闭了发送缓冲区					*
\******************************************************************************/
#include "UDP.h"

using namespace std;

bool InitUDP (SOCKET& socketSrv, SOCKADDR_IN& addrClient)
{
	WSADATA wsd;
	int     nRet;

	// 初始化套接字动态库  
	if (WSAStartup(MAKEWORD(2, 2), &wsd) != 0)
	{
		printf("WSAStartup failed !/n");
		return false;
	}
	socketSrv = socket(AF_INET, SOCK_DGRAM, 0);
	//SOCKET socketSrv = socket(AF_INET, SOCK_DGRAM, 0);
	if (socketSrv == INVALID_SOCKET)
	{
		printf("socket() failed ,Error Code:%d/n", WSAGetLastError());
		WSACleanup();
		return false;
	}
	// 发送缓冲区设置 
	// 希望不经历由系统缓冲区到socket缓冲区的拷贝而影响程序的性能：
	int nSendBuf = 0;//设置为0
	setsockopt(socketSrv, SOL_SOCKET, SO_SNDBUF, (const char*)&nSendBuf, sizeof(int));
	SOCKADDR_IN addrSrv;

	int         len = sizeof(SOCKADDR);
	//select
	fd_set          fd_read;
	struct timeval  timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = 5;	//标准的Unix系统（BSC）的时间片是100毫秒

	////////////// 地址设置 /////////////////////
	// 设置服务器地址  
	addrSrv.sin_addr.S_un.S_addr = htonl(INADDR_ANY);
	//addrSrv.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
	addrSrv.sin_family = AF_INET;
	addrSrv.sin_port = htons(POST);

	// 设置客户端地址  指定发送地址
	addrClient.sin_family = AF_INET;
	addrClient.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
	addrClient.sin_port = htons(POST);

	// 绑定套接字  
	nRet = bind(socketSrv, (SOCKADDR*)&addrSrv, sizeof(SOCKADDR));
	if (SOCKET_ERROR == nRet)
	{
		printf("bind failed !/n");
		closesocket(socketSrv);
		WSACleanup();
		return false;
	}
	return true;
}

void SendAngle(const string &Message, const SOCKET&socketSrv, SOCKADDR_IN&addrClient) 
{
	int nClientAddLen = sizeof(addrClient);
	while (SOCKET_ERROR == sendto(socketSrv, Message.c_str(), BUF_SIZE, 0, (sockaddr *)&addrClient, nClientAddLen)) {
		cout << "send Error = " << WSAGetLastError() << endl;
	}

}