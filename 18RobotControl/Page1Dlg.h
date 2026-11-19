#pragma once
// CPage1Dlg 对话框
#include "Page2Dlg.h"
#include "afxwin.h"

#include <fstream>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class CPage1Dlg : public CDialogEx
{
	DECLARE_DYNAMIC(CPage1Dlg)

public:
	CPage1Dlg(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~CPage1Dlg();

	// 新增禁止拷贝------------------------------------------
    CPage1Dlg(const CPage1Dlg&) = delete;
    CPage1Dlg& operator=(const CPage1Dlg&) = delete;
	//------------------------------------------------------

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_Page1 };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持
	CPage2Dlg m_page2;
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedButton1();
	afx_msg void OnTimer(UINT_PTR nIDEvent);  // 定时器消息处理
	afx_msg void OnBnClickedButton2();
	float m_edit1;
	float m_edit2;
	float m_edit3;
	float m_edit4;
	afx_msg void OnBnClickedButton3();
	afx_msg void OnBnClickedButton4();
	void Rotate();
	void RobotInit();
	afx_msg void OnBnClickedButton5();
	CString m_static1;

	// 电机运行时间
	int m_motorT;
	afx_msg void OnBnClickedButton8();
private:
	LARGE_INTEGER m_frequency;   // 计时器频率
	LARGE_INTEGER m_lastTime4;   // 定时器4的上次触发时间


	//新增部分
	std::ofstream SampleLogFile;  // 采样数据记录文件
    
    // 添加函数声明	
	void StartLineSequence(double distance);
	bool Hori_Move(double distance, bool enableCamera = false);
	bool Verti_Move(double distance, bool enableCamera = false);
	bool SelfRotate(double SetAngle, bool enableCamera = false);
	bool RobotScan(double SetAngle, bool enableCamera = false, bool kf = false);
	void RobotScanAdaptive();
	void CalculateAngle(vector<Point2d>& PosTemp, vector<double>& angles);
	double CalculateHori();
	double CalculateVerti();
	void FindAngle();
	void FindInitial();
	void CalibrateHori();
	void CalibrateVerti();
	int Scan(bool dir);

	LARGE_INTEGER m_lastTime7;   // 机器人旋转时定时器的上次触发时间
	double m_speed;
	double Tread;		// 轮距（mm）
	LARGE_INTEGER m_lastTime13;
	LARGE_INTEGER m_lastTime14;
	LARGE_INTEGER m_lastTime10;
public:
	afx_msg void OnBnClickedButton10();
	afx_msg void OnBnClickedButton9();
	double selfrotateangle;		//控件IDC_EDIT8 (Edit Control)对应的变量，绕中心旋转角度
	afx_msg void OnBnClickedButton11();
	afx_msg void OnBnClickedButton12();
	afx_msg void OnBnClickedButton13();
	double horitemp;	//控件IDC_EDIT9 (Edit Control)对应的变量	
	double vertitemp;	//控件IDC_EDIT10 (Edit Control)对应的变量
	afx_msg void OnBnClickedButton14();
	afx_msg void OnBnClickedButton15();
	afx_msg void OnBnClickedButton16();
	afx_msg void OnBnClickedButton17();
	afx_msg void OnBnClickedButton18();
	double LineDistance;//控件IDC_EDIT7 (Edit Control)对应的变量
	afx_msg void OnBnClickedButton19();
	afx_msg void OnBnClickedButton20();
	afx_msg void OnBnClickedButton6();  // 图像处理测试
	afx_msg void OnBnClickedButton7();  // 直线方程显示
	afx_msg void OnBnClickedButton21();  // 视频加载
	afx_msg void OnBnClickedButtonLoadVideo();  // 加载视频文件并在图片控件中播放
	afx_msg void OnEnChangeEdit2();
	afx_msg void OnEnChangeEdit7();
	afx_msg void OnStnClickedStatic1();
	afx_msg void OnSize(UINT nType, int cx, int cy);  // 调整图片控件大小并居中
};