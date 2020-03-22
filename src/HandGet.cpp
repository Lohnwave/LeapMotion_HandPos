/******************************************************************************\
* Copyright (C) 2019 . 江西省智能信息系统重点实验室, All rights reserved.		*
* Version: 3.0																	*
* Last Revised: 2019-12-08														*
* Editor: Luozu																	*
* v 1.0: 获取Leap motion 采集的手平移、姿态，并通过以太网传给服务器				*
* v 2.0: 增加手指位置采集，且分为左手采集位姿，右手采集手指位置					*
* v 3.0: 使用几何法求解逆运动学，并通过UDP将解（7个关节角，抓握信息）发往机械臂	*
\******************************************************************************/
#include "WAMikine.h"
#include "LeapListener.h" 
#include "UDP.h"
#include <eigen3/Eigen/Dense>
#include <corecrt_math_defines.h>
#include <stdio.h>     
#include <iostream>
#include <sstream>
#include <Windows.h>
#include <iomanip>
#include <fstream>
#include <vector>
#include <string>
#include <cstring>
#include <thread>
#include <mutex>

using namespace std;
using namespace Ikine;
// Base Position - Pose
const double BaseX = 500.0; // mm
const double BaseY = 0.0;
const double BaseZ = 200.0; 

const double BaseA = M_PI / 2;
const double BaseB = 0.0;
const double BaseC = 0.0;

string int2str(const float &int_temp)
{
	string string_temp;
	stringstream stream;
	stream << int_temp;
	string_temp = stream.str();   //此处也可以用 stream>>string_temp  
	return string_temp;
}
string int2str(const double &int_temp)
{
	string string_temp;
	stringstream stream;
	stream << int_temp;
	string_temp = stream.str();   //此处也可以用 stream>>string_temp  
	return string_temp;
}
void saveQ2File(string filePath, const vector<double>&Q) {
	std::ofstream file;
	string s;
	int size = Q.size();
	for (int j = 0; j < size; ++j) {
		s += int2str(Q[j]);
		s += " ";
	}
	file.open(filePath, std::fstream::in | std::fstream::out | std::fstream::app);
	file << s << "\n";
	file.close();
}
void savePos2File(string filePath, double x, double y, double z) { // 20200321 LZ
	std::ofstream file;
	string s;
	s += int2str(x);
	s += " ";
	s += int2str(y);
	s += " ";
	s += int2str(z);
	s += " ";
	file.open(filePath, std::fstream::in | std::fstream::out | std::fstream::app);
	file << s << "\n";
	file.close();
}
void printQ(const vector<double>& Q) {
	int size = Q.size();
	for (int i = 0; i < size; ++i)
		cout << Q[i] << " ";
	cout << endl;
}

void Q2Message(string & Message, vector<double>& Q, vector<double>& FingerDis) {
	int size = Q.size();
	for (int i = 0; i < size; ++i) {
		Message += int2str(Q[i]);
		Message += ",";
	}
	Message += int2str(FingerDis[0]);
	Message += ",";
	Message += int2str(FingerDis[1]);
}
int main()
{
	remove("Qvector.txt");
	remove("Pos.txt");
	vector<double> curQ = { 0.0,0.0,0.0,0.0,0.0,0.0,0.0 };
	vector<double> FingerDis = { 0.0, 0.0 };
	vector<double> Qsolution;
	Eigen::Vector3d Pos;
	// Robot's Tool Position - Pose
	double Pos_X, Pos_Y, Pos_Z;
	double  Roll, Pitch, Yaw;\

	Eigen::Quaterniond TR;
	vector<vector<double>> QV;
	

	float x0 = 0.f, y0 = 0.f, z0 = 0.f, a0 = 0.f, b0 = 0.f, c0 = 0.f; //初始位姿
	float x = 0.f, y = 0.f, z = 0.f, a = 0.f, b = 0.f, c = 0.f;
	
	SampleListener listener(130, 170); // 初始化手指距离为150
	Leap::Controller controller;
	controller.addListener(listener);
	Sleep(2000);
	Leap::Vector HandPosition_pre, HandNormal_pre, HandDirection_pre; // 上一帧的手的数据
	float finger_distance_pre01 = 0.f;							// 上一帧的01手指距离数据
	float finger_distance_pre02 = 0.f;							// 上一帧的02手指距离数据
	Leap::Vector HandPosition, HandNormal, HandDirection;				// 当前帧的手的数据
	float finger_distance01 = 0.f;								// 当前帧的01手指距离数据
	float finger_distance02 = 0.f;								// 当前帧的02手指距离数据
	Leap::Vector HandPosition_R, Eulerangel_R;						// 前后两帧的手的位姿变化量
	float finger_distance_R01 = 0;								// 前后01手指距离变化
	float finger_distance_R02 = 0;								// 前后02手指距离变化

	HandPosition.x = 0;
	HandPosition.y = 0;
	HandPosition.z = 0;
	HandPosition_pre = HandPosition;

	SOCKET socketSrv;
	SOCKADDR_IN addrClient;
	cout << endl;
	if (InitUDP(socketSrv, addrClient))
		cout << "Socket Open..." << endl;
	else
		cout << "Error: socket open failed..." << endl;

	while (1)
	{
		if (HandPosition_pre.x == 0 && HandPosition_pre.y == 0 && HandPosition_pre.z == 0)
		{
			HandPosition_pre = listener.AcqurePosition();
			HandNormal_pre = listener.AcqureNormal();
			HandDirection_pre = listener.AcqureDirection();
			finger_distance_pre01 = listener.AcqureFingerDistance01();
			finger_distance_pre02 = listener.AcqureFingerDistance02();
			// 初始位置获取  世界坐标与 LeapMotion坐标校对
			x0 = HandPosition_pre.z;
			y0 = HandPosition_pre.x;
			z0 = HandPosition_pre.y;
			a0 = HandDirection_pre.pitch();
			b0 = HandNormal_pre.roll();
			c0 = HandDirection_pre.yaw();
		}
		else {
			HandPosition = listener.AcqurePosition();
			HandNormal = listener.AcqureNormal();
			HandDirection = listener.AcqureDirection();
			finger_distance01 = listener.AcqureFingerDistance01();
			finger_distance02 = listener.AcqureFingerDistance02();

			HandPosition_R.x = HandPosition.x - HandPosition_pre.x;
			HandPosition_R.y = HandPosition.y - HandPosition_pre.y;
			HandPosition_R.z = HandPosition.z - HandPosition_pre.z;

			Eulerangel_R.x = HandDirection.pitch() - HandDirection_pre.pitch(); //弧为单位
			Eulerangel_R.y = HandNormal.roll() - HandNormal_pre.roll();
			Eulerangel_R.z = HandDirection.yaw() - HandDirection_pre.yaw();

			finger_distance_R01 = finger_distance01 - finger_distance_pre01;
			finger_distance_R02 = finger_distance02 - finger_distance_pre02;

			// 此处以毫米 弧度为单位
			if (fabs(HandPosition_R.x) > 10 || fabs(HandPosition_R.y) > 10 || fabs(HandPosition_R.z) > 10 ||
				(fabs(Eulerangel_R.x) > 0.2) ||
				(fabs(Eulerangel_R.y) > 0.2) ||
				(fabs(Eulerangel_R.z) > 0.2) ||
				(fabs(finger_distance_R01) > 10) ||
				(fabs(finger_distance_R02) > 10)) {
				// 相较于传递增量，可以避免累计误差
				x = HandPosition.z - x0;
				y = HandPosition.x - y0;
				z = HandPosition.y - z0;
				// 防止姿态变化过大影响逆运动学求解
				if (fabs(Eulerangel_R.x) < 0.5 && fabs(Eulerangel_R.y) < 0.5 && fabs(Eulerangel_R.z) < 0.5) {
					a = HandDirection.pitch() - a0;
					b = HandNormal.roll() - b0;
					c = HandDirection.yaw() - c0;
				}

				HandPosition_pre = HandPosition;
				HandNormal_pre = HandNormal;
				HandDirection_pre = HandDirection;
				finger_distance_pre01 = finger_distance01;
				finger_distance_pre02 = finger_distance02;
				// Position & Pose proofreading with Based reference Position: m
				Pos_X = (BaseX-static_cast<double>(x))/1000;
				Pos_Y = (BaseY-static_cast<double>(y))/1000;
				Pos_Z = (BaseZ+static_cast<double>(z))/1000;
				//Roll = BaseA- static_cast<double>(a);
				//Pitch = BaseB+static_cast<double>(b);
				//Yaw = BaseC-static_cast<double>(c);
				Roll = BaseA - static_cast<double>(a);
				Pitch = BaseB - static_cast<double>(b);
				Yaw = BaseC - static_cast<double>(c);

				FingerDis[0] = static_cast<double>(finger_distance01);
				FingerDis[1] = static_cast<double>(finger_distance02);
				//FingerDis[1] = static_cast<double>(finger_distance01);
				// Euler angle tranlate to Quaternion
				//Eigen::Vector3d eulerAngle(Yaw, Pitch, Roll);
				Eigen::Vector3d eulerAngle(Yaw, Roll, Pitch); 
				Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitX()));
				Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
				Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitZ()));
				TR = yawAngle*pitchAngle*rollAngle;
				if (solve_IK(Eigen::Vector3d(Pos_X, Pos_Y, Pos_Z), TR, curQ, Qsolution))
				{
					curQ = Qsolution;
					QV.push_back(Qsolution);
					//printQ(curQ);
					saveQ2File("Qvector.txt", curQ);
					savePos2File("Pos.txt", Pos_X, Pos_Y, Pos_Z);
				}
				else
					cout << "ERROR: Solving failed..." << endl;
				// UDP send Angle solutions and FingerDistance
				string Message;
				Q2Message(Message, curQ, FingerDis);
				SendAngle(Message, socketSrv, addrClient);
				std::cout << std::string(2, ' ') << std::setiosflags(ios::fixed) << std::setprecision(6) <<
					"Move: " << "X:  " << x << "\tY:  " << y << "\tZ:  " << z << "\tDistance01: " << finger_distance01 <<
					"\n\tTx:  " << a << "\tTy:  " << b << "\tTz:  " << c << "\tDistance02: " << finger_distance02 << std::endl;


			}// end send
		}// end calculation
	}// end while
	// Remove the sample listener when done
	controller.removeListener(listener);
	return 0;
}