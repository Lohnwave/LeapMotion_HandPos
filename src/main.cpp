/******************************************************************************\
* Copyright (C) 2020 . 江西省智能信息系统重点实验室, All rights reserved.		*
* Version: 1.0																	*
* Last Revised: 2020-03-22														*
* Editor: Luozu																	*
* v 1.0: 获取Leap motion 采集的手平移、姿态										*
\******************************************************************************/

#include "LeapListener.h" 
#include <stdio.h>     
#include <iostream>
#include <sstream>
#include <Windows.h>
#include <iomanip>
#include <fstream>
#include <vector>
#include <string>
#include <cstring>

// Base Position - Pose
const double BaseX = 500.0; // mm
const double BaseY = 0.0;
const double BaseZ = 200.0;

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
int main()
{
	remove("Pos.txt"); // Plam Position
	remove("F1.txt");
	remove("F2.txt");
	remove("F3.txt");
	// Robot's Tool Position - Pose
	double Pos_X, Pos_Y, Pos_Z;
	double  Roll, Pitch, Yaw;
	float x0 = 0.f, y0 = 0.f, z0 = 0.f, a0 = 0.f, b0 = 0.f, c0 = 0.f; //初始位姿
	float f1x0 = 0.f, f1y0 = 0.f, f1z0 = 0.f;
	float f2x0 = 0.f, f2y0 = 0.f, f2z0 = 0.f;
	float f3x0 = 0.f, f3y0 = 0.f, f3z0 = 0.f;
	float x = 0.f, y = 0.f, z = 0.f, a = 0.f, b = 0.f, c = 0.f;
	float f1x = 0.f, f1y = 0.f, f1z = 0.f;
	float f2x = 0.f, f2y = 0.f, f2z = 0.f;
	float f3x = 0.f, f3y = 0.f, f3z = 0.f;

	SampleListener listener(130, 170); // 初始化手指距离为150
	Leap::Controller controller;
	controller.addListener(listener);
	Sleep(2000);
	Leap::Vector HandPosition_pre, HandNormal_pre, HandDirection_pre;	// 上一帧的手的数据
	Leap::Vector HandPosition, HandNormal, HandDirection;				// 当前帧的手的数据
	Leap::Vector HandPosition_R, Eulerangel_R;							// 前后两帧的手的位姿变化量
	Leap::Vector Finger1Pos, Finger2Pos, Finger3Pos;

	float finger_distance_pre01 = 0.f;									// 上一帧的01手指距离数据
	float finger_distance_pre02 = 0.f;									// 上一帧的02手指距离数据
	
	float finger_distance01 = 0.f;										// 当前帧的01手指距离数据
	float finger_distance02 = 0.f;										// 当前帧的02手指距离数据

	float finger_distance_R01 = 0;										// 前后01手指距离变化
	float finger_distance_R02 = 0;										// 前后02手指距离变化

	Finger1Pos.x = 0;
	Finger1Pos.y = 0;
	Finger1Pos.z = 0;
	Finger2Pos.x = 0;
	Finger2Pos.y = 0;
	Finger2Pos.z = 0;
	Finger3Pos.x = 0;
	Finger3Pos.y = 0;
	Finger3Pos.z = 0;
	HandPosition.x = 0;
	HandPosition.y = 0;
	HandPosition.z = 0;
	HandPosition_pre = HandPosition;

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
			f1x0 = listener.AcqureF1().z;
			f1y0 = listener.AcqureF1().x;
			f1z0 = listener.AcqureF1().y;
			f2x0 = listener.AcqureF2().z;
			f2y0 = listener.AcqureF2().x;
			f2z0 = listener.AcqureF2().y;
			f3x0 = listener.AcqureF3().z;
			f3y0 = listener.AcqureF3().x;
			f3z0 = listener.AcqureF3().y;
		}
		else {
			HandPosition = listener.AcqurePosition();
			Finger1Pos = listener.AcqureF1();
			Finger2Pos = listener.AcqureF2();
			Finger3Pos = listener.AcqureF3();

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
			//if (fabs(HandPosition_R.x) > 10 || fabs(HandPosition_R.y) > 10 || fabs(HandPosition_R.z) > 10 ||
			//	(fabs(Eulerangel_R.x) > 0.2) ||
			//	(fabs(Eulerangel_R.y) > 0.2) ||
			//	(fabs(Eulerangel_R.z) > 0.2) ||
			//	(fabs(finger_distance_R01) > 1) ||
			//	(fabs(finger_distance_R02) > 1)) {
				// 相较于传递增量，可以避免累计误差
				// get plam Pos
				x = HandPosition.z - x0;
				y = HandPosition.x - y0;
				z = HandPosition.y - z0;
				// get finger 1 2 3
				f1x = Finger1Pos.z - f1x0;
				f1y = Finger1Pos.x - f1y0;
				f1z = Finger1Pos.y - f1z0;
				f2x = Finger2Pos.z - f2x0;
				f2y = Finger2Pos.x - f2y0;
				f2z = Finger2Pos.y - f2z0;
				f3x = Finger3Pos.z - f3x0;
				f3y = Finger3Pos.x - f3y0;
				f3z = Finger3Pos.y - f3z0;
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
				Pos_X = (BaseX - static_cast<double>(x)) / 1000;
				Pos_Y = (BaseY - static_cast<double>(y)) / 1000;
				Pos_Z = (BaseZ + static_cast<double>(z)) / 1000;

				f1x = (BaseX - static_cast<double>(f1x)) / 1000;
				f1y = (BaseY - static_cast<double>(f1y)) / 1000;
				f1z = (BaseZ + static_cast<double>(f1z)) / 1000;
				f2x = (BaseX - static_cast<double>(f2x)) / 1000;
				f2y = (BaseY - static_cast<double>(f2y)) / 1000;
				f2z = (BaseZ + static_cast<double>(f2z)) / 1000;
				f3x = (BaseX - static_cast<double>(f3x)) / 1000;
				f3y = (BaseY - static_cast<double>(f3y)) / 1000;
				f3z = (BaseZ + static_cast<double>(f3z)) / 1000;
				savePos2File("Pos.txt", Pos_X, Pos_Y, Pos_Z);
				savePos2File("F1.txt", f1x, f1y, f1z);
				savePos2File("F2.txt", f2x, f2y, f2z);
				savePos2File("F3.txt", f3x, f3y, f3z);

				std::cout << std::string(2, ' ') << std::setiosflags(ios::fixed) << std::setprecision(6) <<
					"Move: " << "X:  " << x << "\tY:  " << y << "\tZ:  " << z << "\tDistance01: " << finger_distance01 <<
					"\n\tTx:  " << a << "\tTy:  " << b << "\tTz:  " << c << "\tDistance02: " << finger_distance02 << std::endl;


			//}// end send
		}// end calculation
	}// end while
	// Remove the sample listener when done
	controller.removeListener(listener);
	return 0;
}