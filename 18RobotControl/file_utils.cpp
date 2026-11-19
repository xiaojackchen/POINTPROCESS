#include "file_utils.h"
#include <afxwin.h>
#include <atlconv.h>

FileUtils::FileUtils() {};

void FileUtils::OpenFile(string filename, std::ofstream& LogFile)	// std::ofstream禁止拷贝，使用值引用方式
{
	filename = filename + ".txt";              // 拼接日志文件扩展名
	LogFile.open(filename, std::ios::app);     // 以“追加”方式打开文件
	if (!LogFile.is_open()) {                  // 打开失败则提示并返回
		string err = "无法打开" + filename + "!";
		AfxMessageBox(CString(err.c_str()));   // 将错误信息转为CString后弹出提示框
		return;
	}

	if (LogFile.is_open()) {
		LogFile << "Time\tAngle(deg)\tHori(mm)\tVerti(mm)\tpoint(x,y)\tQueryCounter\tH_Speed\tV_Speed\tL_Speed\tR_Speed\n";
	}
}

void FileUtils::FileSaveData(std::ofstream& LogFile, LogDataStruct& LogData)
{
	// 记录到文件
	if (LogFile.is_open()) {
		LogData.SysTime = CTime::GetCurrentTime();	// 取当前系统时间
		//CString systime = LogData.SysTime.Format(_T("%Y-%m-%d %H:%M:%S"));	// 格式化为 2025-10-20 15:30:45
		//string systimeL = CW2A(systime.GetString());
		CStringW systimeW = LogData.SysTime.Format(L"%Y-%m-%d %H:%M:%S");	// 格式化为 2025-10-20 15:30:45
		std::string systimeL = static_cast<const char*>(CW2A(systimeW.GetString()));

		//LogFile << systimeL << "\t"
		//	<< LogData.SelfAngle << "\t"
		//	<< LogData.Hori << "\t"
		//	<< LogData.Verti << "\t"
		//	<< LogData.point << "\t"
		//	<< LogData.currentTime.QuadPart << "\t"
		//	<< LogData.H_Speed << "\t"
		//	<< LogData.V_Speed << "\t"
		//	<< LogData.L_Speed << "\t"
		//	<< LogData.R_Speed << "\n";
		LogFile << std::left << std::setw(25) << systimeL << "\t"
			<< std::setw(12) << LogData.SelfAngle << "\t"
			<< std::setw(8) << LogData.Hori << "\t"
			<< std::setw(8) << LogData.Verti << "\t"
			<< std::setw(15) << LogData.point << "\t"
			<< std::setw(15) << LogData.currentTime << "\t"
			<< std::setw(10) << LogData.H_Speed << "\t"
			<< std::setw(10) << LogData.V_Speed << "\t"
			<< std::setw(10) << LogData.L_Speed << "\t"
			<< std::setw(10) << LogData.R_Speed << "\n";
	}
}

void FileUtils::CloseFile(std::ofstream& LogFile)
{
	LogFile.close();
}
