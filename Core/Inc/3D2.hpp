#pragma once
#include "math.h"
#include "oled.h"


template<typename T>
struct Point3 {
	T x;
	T y;
	T z;
	static Point3<T> zeros() {
		return { 0, 0, 0 };
	}
};

template<typename T>
struct Point2 {
	T x;
	T y;
	static Point2<T> zeros() {
		return { 0, 0 };
	}
};

class Graph2
{
public:
	struct Camera {
		Point3<float> pos;
		Point3<float> rotation;
		int focus;
	} camera_;
	struct Screen {
		int width;
		int height;
	} screen_;
	Graph2(Point3<float> camPos, Point2<int> screen, int focus = 700) {
		camera_.pos = camPos;
		camera_.rotation = Point3<float>::zeros();
		camera_.focus = focus;
		screen_.width = screen.x;
		screen_.height = screen.y;

	}
	~Graph2() {

	}


	void worldToCamera(Point3<float>& p) {

		float newX = p.x - camera_.pos.x;
		float newY = p.y - camera_.pos.y;
		float newZ = p.z - camera_.pos.z;
		// 沿着Y轴旋转
		float x_y = newX * cos(camera_.rotation.y) + newZ * sin(camera_.rotation.y);
		float z_y = -newX * sin(camera_.rotation.y) + newZ * cos(camera_.rotation.y);
		// 沿着X轴旋转
		float y_x = newY * cos(camera_.rotation.x) - z_y * sin(camera_.rotation.x);
		float z_x = newY * sin(camera_.rotation.x) + z_y * cos(camera_.rotation.x);
		// 沿着Z轴旋转
		float x_z = x_y * cos(camera_.rotation.z) - y_x * sin(camera_.rotation.z);
		float y_z = x_y * sin(camera_.rotation.z) + y_x * cos(camera_.rotation.z);

		p.x = x_z;
		p.y = y_z;
		p.z = z_x;
	}

	void cameraToScreen(Point3<float>& p) {
		float k = camera_.focus / p.z;
		p.x *= k;
		p.y *= k;
	}

	Point2<int> ScreenToCanvas(Point3<float>& p) {
		return { int(screen_.width / 2 + p.x), int(screen_.height / 2 - p.y) };
	}

	void drawLine3D(Point3<float> start, Point3<float> end) {
		worldToCamera(start);
		cameraToScreen(start);
		auto start_ = ScreenToCanvas(start);

		worldToCamera(end);
		cameraToScreen(end);
		auto end_ = ScreenToCanvas(end);

		drawLine(start_.x, start_.y, end_.x, end_.y);
		//drawLine(10, 20, 20, 40);

	}

private:

};

