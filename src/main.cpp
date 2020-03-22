#include "WAMikine.h"
#include <eigen3/Eigen/Dense>
#include <corecrt_math_defines.h>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

//#include<time.h>

using namespace std;
using namespace Ikine;
using namespace Eigen;
// Draw sin line
double Gettime(int i) {
	return i * 2 * M_PI / 50;
}
Vector3d GetPos(int i) {
	double x = 0.7 - 0.6*Gettime(i) / (2 * M_PI);
	double y = -0.422;
	double z = 0.2 - 0.2*sin(Gettime(i));
	return Vector3d(x, y, z);
}

string int2str(const float &int_temp)
{
	string string_temp;
	stringstream stream;
	stream << int_temp;
	string_temp = stream.str();   //此处也可以用 stream>>string_temp  
	return string_temp;
}
void saveT2File(string filePath, const double & Time) {
	std::ofstream file;
	string s;
	s += int2str(Time);
	file.open(filePath, std::fstream::in | std::fstream::out | std::fstream::app);
	file << s << "\n";
	file.close();
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
void printQ(const vector<double>& Q) {
	int size = Q.size();
	for (int i = 0; i < size; ++i)
		cout << Q[i] << " ";
	cout << endl;
}
int main()
{
	//clock_t startTime, endTime;
	remove("costTime.txt");
	remove("Qvector.txt");
	Vector3d Pos;
	// Orientation
	// Euler Angles translate to Quaternion
	double yaw, pitch, roll;
	yaw = 0;
	pitch = 0;
	roll = M_PI / 2;
	Vector3d eulerAngle(yaw, pitch, roll);
	AngleAxisd rollAngle(AngleAxisd(eulerAngle(2), Vector3d::UnitX()));
	AngleAxisd pitchAngle(AngleAxisd(eulerAngle(1), Vector3d::UnitY()));
	AngleAxisd yawAngle(AngleAxisd(eulerAngle(0), Vector3d::UnitZ()));

	Quaterniond TR;
	TR = yawAngle*pitchAngle*rollAngle;
	vector<vector<double>> QV;
	vector<double> Q;
	vector<double> curQ = { 0.0,0.0,0.0,0.0,0.0,0.0,0.0 };
	double costTime = 0.0;
	for (int i = 0; i < 50; ++i) {
		//startTime = clock();
		if (solve_IK(GetPos(i), TR, curQ, Q))
		{
			//endTime = clock();
			//costTime = (double)(endTime - startTime);
			//saveT2File("costTime.txt",costTime);
			curQ = Q;
			QV.push_back(Q);
			printQ(curQ);
			saveQ2File("Qvector.txt", curQ);
		}
		else
			cout << "error..." << endl;
	}
	
	cout << "Press any key exit....";
	getchar();
	return 0;
}