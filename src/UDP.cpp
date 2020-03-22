/******************************************************************************\
* Copyright (C) 2019 . ����ʡ������Ϣϵͳ�ص�ʵ����, All rights reserved.		*
* Version: 1.0																	*
* Last Revised: 2019-12-07														*
* Editor: Luozu																	*
* UDP��������ʵ�֣����͹ؽڽǼ�ץ����Ϣ��ע���ر��˷��ͻ�����					*
\******************************************************************************/
#include "UDP.h"

using namespace std;

bool InitUDP (SOCKET& socketSrv, SOCKADDR_IN& addrClient)
{
	WSADATA wsd;
	int     nRet;

	// ��ʼ���׽��ֶ�̬��  
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
	// ���ͻ��������� 
	// ϣ����������ϵͳ��������socket�������Ŀ�����Ӱ���������ܣ�
	int nSendBuf = 0;//����Ϊ0
	setsockopt(socketSrv, SOL_SOCKET, SO_SNDBUF, (const char*)&nSendBuf, sizeof(int));
	SOCKADDR_IN addrSrv;

	int         len = sizeof(SOCKADDR);
	//select
	fd_set          fd_read;
	struct timeval  timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = 5;	//��׼��Unixϵͳ��BSC����ʱ��Ƭ��100����

	////////////// ��ַ���� /////////////////////
	// ���÷�������ַ  
	addrSrv.sin_addr.S_un.S_addr = htonl(INADDR_ANY);
	//addrSrv.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
	addrSrv.sin_family = AF_INET;
	addrSrv.sin_port = htons(POST);

	// ���ÿͻ��˵�ַ  ָ�����͵�ַ
	addrClient.sin_family = AF_INET;
	addrClient.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
	addrClient.sin_port = htons(POST);

	// ���׽���  
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