#pragma once
#ifndef FILE_UTILS_H
#define FILE_UTILS_H

#include <fstream>
#include <opencv2/opencv.hpp>
#include "stdafx.h"

using namespace std;
using namespace cv;

class FileUtils {
public:
	FileUtils();	//	构造函数
	void OpenFile(string filename, std::ofstream& LogFile);	//	打开文件
	struct LogDataStruct;	// 前向声明
	void FileSaveData(std::ofstream& LogFile, LogDataStruct& LogData);
	void CloseFile(std::ofstream& LogFile);	// 关闭文件

	string filename;
	std::ofstream SelfRotationFile;	// 旋转日志文件
	std::ofstream HoriMoveFile;		// 水平移动日志文件
	std::ofstream VertiMoveFile;	// 竖直移动日志文件
	std::ofstream FindInitialFile;	// 初始位置日志文件
	std::ofstream HoriCalibrateFile;	// 水平标定日志文件
	std::ofstream VertiCalibrateFile;	// 竖直标定日志文件

	struct LogDataStruct
	{
		CTime SysTime;
		double SelfAngle;
		double Hori;
		double Verti;
		cv::Point2d point;
		//LARGE_INTEGER currentTime;
		double currentTime;
		double H_Speed;
		double V_Speed;
		double L_Speed;
		double R_Speed;
	};
};

#endif // FILE_UTILS_H
