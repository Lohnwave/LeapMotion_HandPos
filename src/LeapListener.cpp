/******************************************************************************\
* Copyright (C) 2020 . 江西省智能信息系统重点实验室, All rights reserved.		*
* Version: 3.0																	*
* Last Revised: 2020-03-22														*
* Editor: Luozu																	*
* Leapmotion监听类实现：左手采集位姿、右手采集手指抓握闭合						*
* Update: get F1 F2 F3															*
* SDK Vision： 3.2.1															*
\******************************************************************************/
#include "LeapListener.h"
using namespace Leap;
Vector SampleListener::AcqureNormal()const {
	return normal;
}
Vector SampleListener::AcqureDirection()const {
	return direction;
}
Vector SampleListener::AcqurePosition()const { // get Plam Position
	return position;
}
float SampleListener::AcqureFingerDistance01()const {
	return finger_distance_point01;
}
float SampleListener::AcqureFingerDistance02()const {
	return finger_distance_point02;
}
// LZ 20200322
Leap::Vector SampleListener::AcqureF1() const { // get index finger
	return finger1_point;
}
Leap::Vector SampleListener::AcqureF2() const { // get middle finger
	return finger2_point;
}
Leap::Vector SampleListener::AcqureF3() const { // get Thumb finger
	return finger0_point;
}
float SampleListener::getCartesianDistance(const Vector&fingerX_point, const Vector&fingerY_point) {
	float Distance;
	Distance = sqrt(pow((fingerX_point.x - fingerY_point.x), 2)
		+ pow((fingerX_point.y - fingerY_point.y), 2)
		+ pow((fingerX_point.z - fingerY_point.z), 2));
	return Distance;
}
void SampleListener::onFrame(const Controller& controller) {
	// Get the most recent frame and report some basic information
	const Frame frame = controller.frame();

	HandList hands = frame.hands();
	for (HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl) {
		// Get the first hand
		const Hand hand = *hl;
		std::string handType = hand.isLeft() ? "Left hand" : "Right hand";
		// Get the hand's normal vector, direction and position
		if (handType == "Left hand") {
			position = hand.palmPosition(); // get Hand position 手掌心位置对应WAM Joint 7
			normal = hand.palmNormal();		// get Hand Normal main get hand roll (Ty)
			direction = hand.direction();	// get Hand pitch (Tx) and yaw (Tz)
		}
		//////////////////Get fingers' distance////////////////////////
		const FingerList fingers = hand.fingers();
		//Sleep(100);
		bool isfinger = fingers.isEmpty() ? false : true;
		if (true == isfinger)
		{
			if (handType == "Left hand") {
				FingerList::const_iterator fp = fingers.begin();
				const Finger finger0 = *fp;
				const Finger finger1 = *(++fp);
				const Finger finger2 = *(++fp);
				finger0_point = finger0.tipPosition();// Thumb finger position
				finger1_point = finger1.tipPosition();// Index finger position
				finger2_point = finger2.tipPosition();// Middle finger position
													  // calculat distance between thumb/middle finger with index finger
				finger_distance_point01 = getCartesianDistance(finger1_point, finger0_point);
				finger_distance_point02 = getCartesianDistance(finger2_point, finger0_point);
			}
		}
	}// end hand
}// end onFrame

///////////// initialized /////////////////////
void SampleListener::onInit(const Controller& controller) {
	std::cout << "Initialized" << std::endl;
}
void SampleListener::onConnect(const Controller& controller) {
	std::cout << "Connected" << std::endl;
}
void SampleListener::onDisconnect(const Controller& controller) {
	// Note: not dispatched when running in a debugger.
	std::cout << "Disconnected" << std::endl;
}
void SampleListener::onExit(const Controller& controller) {
	std::cout << "Exited" << std::endl;
}
//////////////// Device information //////////////////////
void SampleListener::onFocusGained(const Controller& controller) {
	std::cout << "Focus Gained" << std::endl;
}
void SampleListener::onFocusLost(const Controller& controller) {
	std::cout << "Focus Lost" << std::endl;
}
void SampleListener::onDeviceChange(const Controller& controller) {
	std::cout << "Device Changed" << std::endl;
	const DeviceList devices = controller.devices();

	for (int i = 0; i < devices.count(); ++i) {
		std::cout << "id: " << devices[i].toString() << std::endl;
		std::cout << "  isStreaming: " << (devices[i].isStreaming() ? "true" : "false") << std::endl;
		std::cout << "  isSmudged:" << (devices[i].isSmudged() ? "true" : "false") << std::endl;
		std::cout << "  isLightingBad:" << (devices[i].isLightingBad() ? "true" : "false") << std::endl;
	}
}
void SampleListener::onServiceConnect(const Controller& controller) {
	std::cout << "Service Connected" << std::endl;
}
void SampleListener::onServiceDisconnect(const Controller& controller) {
	std::cout << "Service Disconnected" << std::endl;
}
void SampleListener::onServiceChange(const Controller& controller) {
	std::cout << "Service Changed" << std::endl;
}
void SampleListener::onDeviceFailure(const Controller& controller) {
	std::cout << "Device Error" << std::endl;
	const Leap::FailedDeviceList devices = controller.failedDevices();

	for (FailedDeviceList::const_iterator dl = devices.begin(); dl != devices.end(); ++dl) {
		const FailedDevice device = *dl;
		std::cout << "  PNP ID:" << device.pnpId();
		std::cout << "    Failure type:" << device.failure();
	}
}
void SampleListener::onLogMessage(const Controller&, MessageSeverity s, int64_t t, const char* msg) {
	switch (s) {
	case Leap::MESSAGE_CRITICAL:
		std::cout << "[Critical]";
		break;
	case Leap::MESSAGE_WARNING:
		std::cout << "[Warning]";
		break;
	case Leap::MESSAGE_INFORMATION:
		std::cout << "[Info]";
		break;
	case Leap::MESSAGE_UNKNOWN:
		std::cout << "[Unknown]";
	}
	std::cout << "[" << t << "] ";
	std::cout << msg << std::endl;
}
