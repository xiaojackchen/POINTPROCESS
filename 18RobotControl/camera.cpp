#include "camera.h"
//#include <string.h>

Camera::Camera()	// 构造函数
{
	cap.open(0);	//	打开摄像头
	//cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);		//	设置摄像头参数
	//cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
	//cap.set(cv::CAP_PROP_FPS, 30);				//	设置帧率为30
	frame_width = static_cast<int>(cap.get(CAP_PROP_FRAME_WIDTH));	// 获取摄像头参数
	frame_height = static_cast<int>(cap.get(CAP_PROP_FRAME_HEIGHT));
	fps = cap.get(CAP_PROP_FPS);
	if (fps <= 0) fps = 30.0;  // 默认帧率30，这个帧率设置的是播放器播放视频的帧率
	fourcc = VideoWriter::fourcc('M', 'J', 'P', 'G');	//	视频格式为.avi
}

void Camera::OpenFile(double start, double end) 
{
	// 创建视频写入对象
	char buffer[30];
	sprintf_s(buffer, "%.2f", start);
	string name1 = buffer;
	start_string = buffer;
	sprintf_s(buffer, "%.2f", end);
	string name2 = buffer;
	oldname = name1 + "_" + name2 + ".avi";
	CamRecorder.open(oldname, fourcc, fps, Size(frame_width, frame_height));
	CamRecorder.set(VIDEOWRITER_PROP_QUALITY, 100);
}

void Camera::SaveFrame()
{
	cv::Mat frame;
	cap >> frame;
	CamRecorder.write(frame);	// 写入当前帧到视频文件
}

void Camera::CloseFile()
{
	CamRecorder.release();
}

void Camera::NewName(double end)
{
	char buffer[30];
	sprintf_s(buffer, "%.2f", end);
	string name = buffer;
	string filename = start_string + "_" + name + ".avi";
	std::rename(oldname.c_str(), filename.c_str());
}
