#ifndef __LeapListener_H__
#define __LeapListener_H__
#include <LeapSDK\include\Leap.h>
#include <string>
using namespace std;
									/*拇指		食指	中指	无名指		小指*/
const std::string fingerNames[] = { "Thumb", "Index", "Middle", "Ring", "Pinky" };
const std::string boneNames[] = { "Metacarpal", "Proximal", "Middle", "Distal" };

class SampleListener : public Leap::Listener {
public:
	SampleListener(const float&dis01, const float&dis02)
		:finger_distance_point01(dis01), finger_distance_point02(dis02) {}
	virtual void onInit(const Leap::Controller&);
	virtual void onConnect(const Leap::Controller&);
	virtual void onDisconnect(const Leap::Controller&);
	virtual void onExit(const Leap::Controller&);
	virtual void onFrame(const Leap::Controller&);
	virtual void onFocusGained(const Leap::Controller&);
	virtual void onFocusLost(const Leap::Controller&);
	virtual void onDeviceChange(const Leap::Controller&);
	virtual void onServiceConnect(const Leap::Controller&);
	virtual void onServiceDisconnect(const Leap::Controller&);
	virtual void onServiceChange(const Leap::Controller&);
	virtual void onDeviceFailure(const Leap::Controller&);
	virtual void onLogMessage(const Leap::Controller&, Leap::MessageSeverity severity, int64_t timestamp, const char* msg);
	virtual float getCartesianDistance(const Leap::Vector&, const Leap::Vector&);
	Leap::Vector AcqureNormal()const;
	Leap::Vector AcqureDirection()const;
	Leap::Vector AcqurePosition()const;
	float AcqureFingerDistance01()const;
	float AcqureFingerDistance02()const;
private:
	Leap::Vector position;
	Leap::Vector normal;
	Leap::Vector direction;
	float finger_distance_point01;
	float finger_distance_point02;
	Leap::Vector finger0_point; // Thumb
	Leap::Vector finger1_point; // Index
	Leap::Vector finger2_point; // Middle
};

#pragma once
#endif // !__LeapListener_H__


