#pragma once
#ifndef CAMERA_H
#define CAMERA_H
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

class Camera {
public:
	Camera();	//	构造函数
	void OpenFile(double start, double end);	//	打开文件
	void SaveFrame();	//	保存一帧图片
	void CloseFile();	// 关闭文件
	void NewName(double end);

public:
	cv::VideoCapture cap;
	cv::VideoWriter CamRecorder;
	int frame_width;	//	图像宽度
	int frame_height;	//	图像高度
	double fps;			//	图像帧率
	int fourcc;			//	视频格式
	string start_string;
	string oldname;
};

#endif // CAMERA_H
