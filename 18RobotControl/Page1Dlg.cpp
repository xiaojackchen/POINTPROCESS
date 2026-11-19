// Page1Dlg.cpp : 实现文件

#include "stdafx.h"
#include "18RobotControl.h"
#include "Page1Dlg.h"
#include "afxdialogex.h"
#include "SerialPort.h"
#include "Robot.h"
//#include <vector>	// 头文件已经引用
#include <cmath>
#include <algorithm> // 用于std::min和std::max
//#include <opencv2/opencv.hpp>	// 头文件已经引用
//#include <fstream>  // 需要包含文件流头文件
#include <sstream>  // 需要包含文件流头文件
#include <iostream>  // 需要包含文件流头文件
#include <random>
#include <numeric>
#include <chrono>
#include <iomanip>
//#include <sstream>
#include <filesystem>

#include "camera.h"
#include "math_utils.h"
#include "file_utils.h"
#include "img_process.h"//已经引入img_data.h，img_thinning.h，img_lines.h
#include "img_lines.h"
#include "task.h"

using namespace cv;
using namespace std;

cv::VideoCapture cap(0); // 打开默认摄像头
Camera camera;	//	创建相机对象
FileUtils Log;
ImgData framedata;
ImgData framedata1;
ImgTask task;

// 视频文件播放状态。单独使用一套 ImgData/ImgTask，避免影响摄像头实时处理使用的 framedata/task。
static ImgTask g_videoTask;
static ImgData g_videoData;
static int g_captureFailCount = 0;
static cv::VideoCapture g_videoFileCap;  // 视频文件播放使用的采集对象
static int g_videoSourceFlag = 0;  // 0: 摄像头，1: 视频文件
static std::string g_videoFilePath;  // 已加载的视频文件路径

static cv::Mat g_lastVideoFrame;      // 最近一次读到的视频帧（急停后供直线方程显示使用）
static bool g_useFrozenFrame = false; // true 表示使用急停瞬间缓存的那一帧
// 水平/竖直按钮的交点记录状态
static std::ofstream g_intersectionLog;
static int g_intersectionRecordAxis = 0;  // 0: 未记录，1: 水平，2: 竖直
static int g_intersectionFrameIndex = 0;
static void StopIntersectionRecord();
static void WriteIntersectionRecord();

// CPage1Dlg 对话框
#define M_PI 3.14169265
#define M_PI_2 M_PI/2.0
// MPC控制参数结构体
struct MPCConfig {
	double dt = 0.5;         // 控制周期（秒）
	double L = 210.0;         // 轮距（mm）
	double v_max = 14.0;      // 最大轮速（mm/s）
	double v_min = -14.0;     // 最小轮速（mm/s）
};
double m_x0 = 0.0,m_y0 = 0.0, m_theta0 = 0.0; // 初始角度

vector<pair<double, double>> runMPCController(const vector<pair<double, double>>& path, const MPCConfig& config, vector<double>& v1_history,  // 输出参数：左轮速度历史
	vector<double>& v2_history) {  // 输出参数：右轮速度历史
	vector<pair<double, double>> trajectory = { { m_x0, m_y0 } };
	// 清空历史记录
	//    v1_history.clear();
	//    v2_history.clear();
	// MPC主循环
	for (size_t i = 0; i < path.size() - 1; ++i) {
		// 计算目标点偏差
		const auto& target = path[i + 1];
		double dx = target.first - m_x0;
		double dy = target.second - m_y0;
		// 速度计算（核心公式）
		double v = sqrt(dx*dx + dy*dy) / config.dt;          // 线速度
		double theta_desired = atan2(dy, dx);                // 期望角度
		double w = (theta_desired - m_theta0) / config.dt;     // 角速度
		// 轮速转换公式
		double v1 = (2 * v - config.L*w) / 2;  // 左轮速度
		double v2 = (2 * v + config.L*w) / 2;  // 右轮速度
		// 速度限幅
		v1 = max(config.v_min, min(v1, config.v_max));//max(config.v_min, min(v1, config.v_max))
		v2 = max(config.v_min, min(v2, config.v_max));//max(config.v_min, min(v2, config.v_max))
		// 记录速度
		v1_history.push_back(v1);
		v2_history.push_back(v2);
		// 状态更新（运动学模型）
		double v_avg = (v1 + v2) / 2;
		w = (v2 - v1) / config.L;
		m_x0 += v_avg * cos(m_theta0) * config.dt;
		m_y0 += v_avg * sin(m_theta0) * config.dt;
		m_theta0 += w * config.dt;
		// 记录轨迹
		trajectory.emplace_back(m_x0, m_y0);
	}
	return trajectory;
}
// 辅助函数：生成等差数列
vector<double> linspace(double start, double end, int num) {
	vector<double> result(num);
	double step = (end - start) / (num - 1);
	for (int i = 0; i < num; ++i) {
		result[i] = start + i * step;
	}
	return result;
}
vector<double> v1_history, v2_history;
void MPCContor() {
	// 生成目标路径
	vector<pair<double, double>> path, path1, path2;
	//vector<double> x_values = linspace(0, 10, 50);
	// 直线段部分 (0,0)到(5,0)
	vector<double> x_line = linspace(0, 100, 20); // 前25个点
	for (double x : x_line) {
		path.emplace_back(x, 0.0);
	}
	// 1/4圆弧部分 (圆心(5,5), 半径5)
	vector<double> theta = linspace(3 * M_PI / 2, 2 * M_PI, 100); // 后25个点
	double R = 50.0, Cx = path.back().first, Cy = path.back().second+R;
	for (double t : theta) {
		double x = Cx + R*cos(t);
		double y = Cy + R*sin(t);
		path1.emplace_back(x, y);
	}
	vector<double> y_line = linspace(path1.back().second, path1.back().second +100, 20); // 前25个点
	for (double y : y_line) {
		path2.emplace_back(path1.back().first, y);
	}
	// 配置参数
	MPCConfig config;
	config.dt = 0.5;    // 可在此修改参数
	config.L = 210.0;
	// 执行MPC控制
	auto trajectory = runMPCController(path, config, v1_history, v2_history);
	m_x0 = path.back().first; m_y0 = path.back().second; m_theta0 = 0;//更新状态变量
	trajectory = runMPCController(path1, config, v1_history, v2_history);
	m_x0 = path1.back().first; m_y0 = path1.back().second; m_theta0 = M_PI_2;//更新状态变量
	trajectory = runMPCController(path2, config, v1_history, v2_history);
}

IMPLEMENT_DYNAMIC(CPage1Dlg, CDialogEx)

CPage1Dlg::CPage1Dlg(CWnd* pParent /*=NULL*/)	// CPage1Dlg 类的构造函数实现，用于初始化对话框对象的成员变量。
	: CDialogEx(IDD_Page1, pParent)
	, m_edit1(0.0)
	, m_edit2(0)
	, m_edit3(0)
	, m_edit4(0)
	, m_static1(_T(""))
	, m_motorT(0)
	//新增代码----------------------------------
	, selfrotateangle(0)
	, m_speed(1.0)
	, Tread(210.0)
	, horitemp(0)
	, vertitemp(0)
	, LineDistance(0)// 控件IDC_EDIT7 (Edit Control)对应的变量
{

}

CPage1Dlg::~CPage1Dlg()
{
	//KillTimer(13);
}

void CPage1Dlg::DoDataExchange(CDataExchange* pDX)	// CPage1Dlg 类的数据交换函数实现，用于将数据从对话框控件复制到成员变量或从成员变量复制到对话框控件。
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_EDIT1, m_edit1);	// IDC_EDIT1 控件 <--> m_edit1 变量
	DDX_Text(pDX, IDC_EDIT2, m_edit2);
	DDX_Text(pDX, IDC_EDIT3, m_edit3);
	DDX_Text(pDX, IDC_EDIT4, m_edit4);
	DDX_Text(pDX, IDC_STATIC1, m_static1);
	DDX_Text(pDX, IDC_EDIT5, m_motorT);
	DDX_Text(pDX, IDC_EDIT8, selfrotateangle);
	DDX_Text(pDX, IDC_EDIT9, horitemp);
	DDX_Text(pDX, IDC_EDIT10, vertitemp);
	DDX_Text(pDX, IDC_EDIT7, LineDistance);
}

// CPage1Dlg 类的消息映射表实现，用于处理对话框的消息。
// BEGIN_MESSAGE_MAP宏定义了CPage1Dlg类的消息处理映射关系，告诉MFC框架当特定消息发生时应该调用哪个成员函数。
BEGIN_MESSAGE_MAP(CPage1Dlg, CDialogEx)
	ON_WM_SIZE()  // 窗口大小变化时，重新调整图片控件大小和位置
	ON_WM_TIMER()  // 定时器触发时的响应
	ON_BN_CLICKED(IDC_BUTTON1, &CPage1Dlg::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_BUTTON2, &CPage1Dlg::OnBnClickedButton2)
	ON_BN_CLICKED(IDC_BUTTON3, &CPage1Dlg::OnBnClickedButton3)
	ON_BN_CLICKED(IDC_BUTTON4, &CPage1Dlg::OnBnClickedButton4)
	ON_BN_CLICKED(IDC_BUTTON5, &CPage1Dlg::OnBnClickedButton5)

	ON_BN_CLICKED(IDC_BUTTON8, &CPage1Dlg::OnBnClickedButton8)
	ON_BN_CLICKED(IDC_BUTTON10, &CPage1Dlg::OnBnClickedButton10)
	ON_BN_CLICKED(IDC_BUTTON9, &CPage1Dlg::OnBnClickedButton9)
	ON_BN_CLICKED(IDC_BUTTON11, &CPage1Dlg::OnBnClickedButton11)
	ON_BN_CLICKED(IDC_BUTTON12, &CPage1Dlg::OnBnClickedButton12)
	ON_BN_CLICKED(IDC_BUTTON13, &CPage1Dlg::OnBnClickedButton13)
	ON_BN_CLICKED(IDC_BUTTON14, &CPage1Dlg::OnBnClickedButton14)
	ON_BN_CLICKED(IDC_BUTTON15, &CPage1Dlg::OnBnClickedButton15)
	ON_BN_CLICKED(IDC_BUTTON16, &CPage1Dlg::OnBnClickedButton16)
	ON_BN_CLICKED(IDC_BUTTON17, &CPage1Dlg::OnBnClickedButton17)
	ON_BN_CLICKED(IDC_BUTTON18, &CPage1Dlg::OnBnClickedButton18)
	ON_BN_CLICKED(IDC_BUTTON19, &CPage1Dlg::OnBnClickedButton19)
	ON_BN_CLICKED(IDC_BUTTON20, &CPage1Dlg::OnBnClickedButton20)
	ON_BN_CLICKED(IDC_BUTTON6, &CPage1Dlg::OnBnClickedButton6)
	ON_BN_CLICKED(IDC_BUTTON7, &CPage1Dlg::OnBnClickedButton7)
	ON_BN_CLICKED(IDC_BUTTON21, &CPage1Dlg::OnBnClickedButton21)
	ON_BN_CLICKED(IDC_BUTTON_LOADVIDEO, &CPage1Dlg::OnBnClickedButtonLoadVideo)

	ON_EN_CHANGE(IDC_EDIT2, &CPage1Dlg::OnEnChangeEdit2)
	ON_EN_CHANGE(IDC_EDIT7, &CPage1Dlg::OnEnChangeEdit7)
	ON_STN_CLICKED(IDC_STATIC1, &CPage1Dlg::OnStnClickedStatic1)
END_MESSAGE_MAP()
// 日志写入接口
bool WriteToLogEnd(const CString& filePath, const CString& message){
	try {// 创建或打开文件（追加模式）
		CStdioFile file;
		UINT nOpenFlags = CFile::modeWrite | CFile::modeCreate | CFile::modeNoTruncate;
		if (!file.Open(filePath, nOpenFlags)) {
			return false;
		}	
		file.SeekToEnd();// 定位到文件末尾
		CString logEntry = message + _T("\r\n");// 构造带换行符的日志条目（Windows换行符\r\n）	
		file.WriteString(logEntry);// 写入日志内容
		file.Close();// 显式关闭文件（析构函数也会自动关闭）
		return true;
	}
	catch (CFileException* pEx) {
		// 错误处理（可扩展记录错误信息）
		pEx->Delete();  // 必须删除异常对象
		return false;
	}
}
CSerialPort Port1;//串口对象
CSerialPort Port2;//串口对象
Robot R1;//机器人对象
// CPage1Dlg 消息处理程序
void CPage1Dlg::OnBnClickedButton1(){
	UpdateData(1);	// 从控件读取数据到变量,读取控件数据
	CString str;
	if (!Port1.InitPort(5, 115200)) {//打开串口1,波特率921600
		AfxMessageBox(_T("打开串口失败"));
		return;
	}
	if (!Port2.InitPort(6,921600)) {//打开串口1,波特率921600
		AfxMessageBox(_T("打开串口失败"));
		return;
	}
	else {
		if (!Port2.OpenListenThread()) {//打开监听线程
			AfxMessageBox(_T("打开串口监听失败"));
			return;
		}
	}
	AfxMessageBox(_T("打开串口OK"));
	str.Format(_T("%s%d"), str, __cplusplus);
	//AfxMessageBox(str);
	QueryPerformanceFrequency(&m_frequency);  // 获取高精度计时器频率
	QueryPerformanceCounter(&m_lastTime4);  // 记录启动时刻

	UpdateData(0);	// 从变量设置控件显示值,更新控件显示
}

int ncount = 0;
void CPage1Dlg::OnBnClickedButton2(){
	UpdateData(1);
	// 设置一个定时器，间隔 10 秒 (10000 毫秒)
	SetTimer(1,250, nullptr);  // 第一个参数是定时器 ID，第二个参数是间隔时间（毫秒）
	//SetTimer(1, m_edit2*1000, nullptr);  // 第一个参数是定时器 ID，第二个参数是间隔时间（毫秒）
	//Port1.MotorCtrl(0, R1.W2PSC(-0.1), 0, 0);//伸出
	//Port1.MotorCtrl(R1.W2PSC(0.1), 0, 0, 0);//向上0.2mm/s
	//Port2.Sample(1);//起弧
	UpdateData(0);
}
int Tcount = 0; int datCount = 0;
bool H_end_flag = false;//水平末端标志位
bool V_end_flag = false;//竖直末端标志位
bool H_over = false;//水平滑块初始位置调整完成标志位
void CPage1Dlg::RobotInit() {//机器人初始化
	//防止串口断开
	if (Port1.InitPort(5, 115200)||Port2.InitPort(6, 921600)) {//判断串口是否打开
		AfxMessageBox(_T("打开串口失败"));
		return;
	}
	R1.ClearAllV();          //清空所有电机线速度
	if (!H_over)
	{
		R1.LinearV[Hori] = (H_end_flag ? 5.0 : -5.0);
		if (!H_end_flag) {//没到末端，判断位置
						  //采样限位开关
			unsigned char LimitSwitch;//开关：（后，中，前，上，下）
			Port2.Sample(2);//采样限位开关
			while (!Port2.c_Recvd) Sleep(5);//等待采样完成
			Port2.c_Recvd = false;
			unsigned short UsDigital;//接收到的超声波数字量
			if (process_data(Port2.data, NULL, NULL, &UsDigital, &LimitSwitch)<1)//超声波数据为1
				return;//处理超声波及开关
			R1.Switch2Position(LimitSwitch);//获取位置信息
											//判断机器人位置
			if (!R1.Position[0])//竖直滑块到了最后
			{
				H_end_flag = true;//末端标志位置为true
				R1.Position[Back] = true;
			}
		}
		else if (R1.updata_hori_status()) {//判断机器人水平滑块是否达到目标位置
			H_end_flag = false;//末端标志位置为false
			R1.ClearAllV();          //清空所有电机线速度
			R1.V2PSC();              //更新速度到PSC
			Port1.MotorCtrl(R1.PSC);//发送控制信号
			//KillTimer(2);//关闭定时器
			//return;
			//修改成以下
			H_over = true;
		}
		UpdateData(TRUE);
		// 将 bool 和 double 类型转换为字符串
		CString strEndFlag = H_end_flag ? _T("True") : _T("False");
		CString strLinearV;
		strLinearV.Format(_T("%.2f"), R1.LinearV[Hori]);  // 格式化为小数点后两位
														  // 将它们拼接成一个字符串
		m_static1.Format(_T("H_end_flag: %s, R1.LinearV[Hori]: %s"), strEndFlag, strLinearV);

		UpdateData(FALSE);
		R1.V2PSC();              //更新速度到PSC
		Port1.MotorCtrl(R1.PSC);//发送控制信号
	}
	if(H_over)
	{
		//R1.LinearV[Vert] = (V_end_flag ? -2.0 : 2.0);
		if (!V_end_flag) { R1.LinearV[Vert] = 5.0; }
		else { R1.LinearV[Vert] = -5.0; }

		if (!V_end_flag) {//没到末端，判断位置
						  //采样限位开关
			unsigned char LimitSwitch;//开关：（后，中，前，上，下）
			Port2.Sample(2);//采样限位开关
			while (!Port2.c_Recvd) Sleep(5);//等待采样完成
			Port2.c_Recvd = false;
			unsigned short UsDigital;//接收到的超声波数字量
			if (process_data(Port2.data, NULL, NULL, &UsDigital, &LimitSwitch)<1)//超声波数据为1
				return;//处理超声波及开关
			R1.Switch2Position(LimitSwitch);//获取位置信息
											//判断机器人位置
			if (!R1.Position[4])//竖直滑块到了最后
			{
				V_end_flag = true;//末端标志位置为true
				R1.Position[Down] = true;
			}
		}
		else if (R1.updata_verti_status()) {//判断机器人水平滑块是否达到目标位置
			V_end_flag = false;//末端标志位置为false
			R1.ClearAllV();          //清空所有电机线速度
			R1.V2PSC();              //更新速度到PSC
			Port1.MotorCtrl(R1.PSC);//发送控制信号
			KillTimer(2);//关闭定时器
			H_over = false;
			return;
		}
		UpdateData(TRUE);
		// 将 bool 和 double 类型转换为字符串
		CString strEndFlag = V_end_flag ? _T("True") : _T("False");
		CString strLinearV;
		strLinearV.Format(_T("%.2f"), R1.LinearV[Vert]);  // 格式化为小数点后两位
														  // 将它们拼接成一个字符串
		m_static1.Format(_T("V_end_flag: %s, R1.LinearV[Vert]: %s"), strEndFlag, strLinearV);

		UpdateData(FALSE);
		R1.V2PSC();              //更新速度到PSC
		Port1.MotorCtrl(R1.PSC);//发送控制信号
	}
}
//直角转弯
void CPage1Dlg::Rotate() {
	if (Port1.InitPort(5, 115200) || Port2.InitPort(6, 921600)) {//判断串口是否打开
		AfxMessageBox(_T("打开串口失败"));
		return;
	}
	unsigned char LimitSwitch;//开关：（后，中，前，上，下）
	Port2.Sample(2);//采样限位开关
	while (!Port2.c_Recvd) Sleep(1);//等待采样完成
	Port2.c_Recvd = false;
	unsigned short UsDigital;//接收到的超声波数字量
	process_data(Port2.data, NULL, NULL, &UsDigital, &LimitSwitch);//处理超声波及开关
	double UsAnalog;
	UsAnalog = ADCToAnalog(UsDigital,40.0/ 13.0);//转换超声波数据，需要输入超声波比值,此处转换为输出模拟电压
	R1.Analog2Distance(UsAnalog);
	R1.UpdataRobot();
	Port1.MotorCtrl(R1.PSC);
}
//急停
void CPage1Dlg::OnBnClickedButton3(){
	Port1.MotorCtrl(0, 0, 0, 0);  // 所有电机速度为0
	KillTimer(1);
	KillTimer(2);
	KillTimer(3);
	KillTimer(4);
	KillTimer(5);
	KillTimer(6);
	KillTimer(7);
	KillTimer(8);
	KillTimer(9);
	KillTimer(10);
	KillTimer(11);
	KillTimer(12);
	KillTimer(13);
	KillTimer(14);
	if (g_videoFileCap.isOpened())
		g_videoFileCap.release();
	// 急停：若已有视频帧缓存，之后“直线方程显示”使用急停瞬间的帧
	if (!g_lastVideoFrame.empty())
		g_useFrozenFrame = true;
	StopIntersectionRecord();
}

double X0 = 0.0;
double X0Target = 0.0;
//单电机控制
void CPage1Dlg::OnBnClickedButton4(){
	KillTimer(1);
	KillTimer(2);
	KillTimer(4);

	UpdateData(TRUE);
	unsigned short VertSpeed = 0, HorSpeed = 0, LeftSpeed = 0, RightSpeed = 0;
	//左轮
	if (m_edit1 >= 5.6 || m_edit1 <= -5.6)
	{
		MessageBox(TEXT("左轮速度超过设置范围"));
	}
	else if (m_edit1<0)
	{
		LeftSpeed = -R1.W2PSC(-m_edit1);
	}
	else
	{
		LeftSpeed = R1.W2PSC(m_edit1);
	}
	//右轮
	if (m_edit2 >= 5.6 || m_edit2 <= -5.6)
	{
		MessageBox(TEXT("右轮速度超过设置范围"));
	}
	else if (m_edit2 < 0)
	{
		RightSpeed = -R1.W2PSC(-m_edit2);
	}
	else
	{
		RightSpeed = R1.W2PSC(m_edit2);
	}
	//竖直
	if (m_edit3 >= 5.9 || m_edit3 <= -5.9)
	{
		MessageBox(TEXT("竖直速度超过设置范围"));
	}
	else if (m_edit3 < 0)
	{
		VertSpeed = -R1.W2PSC(m_edit3);
	}
	else
	{
		VertSpeed = R1.W2PSC(-m_edit3);
	}
	//水平
	if (m_edit4 >= 4.9 || m_edit4 <= -4.9)
	{
		MessageBox(TEXT("水平速度超过设置范围"));
	}
	else if (m_edit4 < 0)
	{
		HorSpeed = -R1.W2PSC(-m_edit4);
	}
	else
	{
		HorSpeed = R1.W2PSC(m_edit4);
	}
	Port1.MotorCtrl(VertSpeed, HorSpeed, LeftSpeed, RightSpeed);
	char dat[100];
	std::snprintf(dat, sizeof(dat), "Angle:%.2f\nVertDis:%.2f\nHoriDis:%.2f\nX0:%.2f", R1.AngleOut, R1.VertOut, R1.HoriOut, X0);
	m_static1 = dat;
	GetDlgItem(IDC_STATIC1)->SetWindowText(m_static1);
	//if(ncount%2==1){
	//unsigned short VertSpeed = R1.W2PSC(m_edit3);//导程1(单位cm / r),向上
	//unsigned short HorSpeed = R1.W2PSC(m_edit4);
	//unsigned short LeftSpeed = R1.W2PSC(m_edit1);//前进
	//unsigned short RightSpeed = R1.W2PSC(m_edit2);
	//Port1.MotorCtrl(VertSpeed, HorSpeed, LeftSpeed, RightSpeed);//发送速度
	//}
	//else {
	//	unsigned short VertSpeed = -R1.W2PSC(m_edit3);//导程1(单位r/s)
	//	unsigned short HorSpeed = -R1.W2PSC(m_edit4);
	//	unsigned short LeftSpeed = -R1.W2PSC(m_edit1);
	//	unsigned short RightSpeed = -R1.W2PSC(m_edit2);
	//	Port1.MotorCtrl(VertSpeed, HorSpeed, LeftSpeed, RightSpeed);//发送速度
	//}
	//ncount++;
	UpdateData(FALSE);
}
//初始位置复位
void CPage1Dlg::OnBnClickedButton5(){
	// TODO: 在此添加控件通知处理程序代码
	KillTimer(1);

	R1.HoriOut = 0.0;	// 初始复位先清零，用于updata_hori_status()和updata_verti_status()
	R1.VertOut = 0.0;

	SetTimer(2, 250, nullptr);  // 第一个参数是定时器 ID，第二个参数是间隔时间（毫秒）
	H_end_flag = false;
	V_end_flag = false;
	H_over = false;
}


double Wheel = 0.0;
// 写入日志
void UsLogWrite(char dat[], unsigned int num) {//log
	std::ofstream outputFile("CtrlLog.txt", std::ios::app);
	if (!outputFile.is_open()) {  // 检查文件是否成功打开
		return;  // 返回错误代码，表示打开文件失败
	}
	outputFile.write(dat, num);
	outputFile.close();  // 确保文件正常关闭
}
void Us() {
	if (Port1.InitPort(4, 115200) || Port2.InitPort(8, 921600)) {//判断串口是否打开
		AfxMessageBox(_T("打开串口失败"));
		return;
	}
	unsigned char LimitSwitch;//开关：（后，中，前，上，下）
	Port2.Sample(2);//采样限位开关
	while (!Port2.c_Recvd) Sleep(1);//等待采样完成
	Port2.c_Recvd = false;
	unsigned short UsDigital;//接收到的超声波数字量
	process_data(Port2.data, NULL, NULL, &UsDigital, &LimitSwitch);//处理超声波及开关
	double UsAnalog;
	UsAnalog = ADCToAnalog(UsDigital, 40.0 / 13.0);//转换超声波数据，需要输入超声波比值,此处转换为输出模拟电压
	//R1.Analog2Distance(UsAnalog);
	Wheel += R1.LinearV[Left] * R1.t;
	if (UsAnalog < 2.5) {
		R1.LinearV[Left] = 0.0;
		R1.LinearV[Right] = 0.0;
	}
	else {
		R1.LinearV[Left] = 4.0;
		R1.LinearV[Right] = 4.0;
	}
	R1.V2PSC();              //更新速度到PSC
	Port1.MotorCtrl(R1.PSC);
	char dat[100];
	unsigned int num;
	num = std::snprintf(dat, sizeof(dat), "AD: %d, Us: %.2f, Wh: %.2f\n"
		, UsDigital, UsAnalog, Wheel);
	UsLogWrite(dat,num);
}

double xxx = 0.0;
int Xcount = 0;

double timestamp;
bool captureFrameWithDistance(cv::VideoCapture& cap, int frameCount, double distance, double& timestamp,Point2d &P) {
	cv::Mat frame;
	LPCWSTR folderName = L"Get_Photo";
	char dat[100];
	unsigned int num;
	// 创建文件夹
	if (CreateDirectory(folderName, NULL) || GetLastError() == ERROR_ALREADY_EXISTS) {
		num = std::snprintf(dat, sizeof(dat), "文件夹创建成功或已存在！\n"
			);
	}
	else {
		num = std::snprintf(dat, sizeof(dat), "创建文件夹失败，错误码：%d\n"
			, GetLastError());
	}
	UsLogWrite(dat, num);
	// 使用同一类型时钟（高分辨率时钟）
	auto start = std::chrono::high_resolution_clock::now();
	// 捕获帧
	if (!cap.read(frame) || frame.empty()) 	return false;
	std::snprintf(dat, sizeof(dat), "Get_Photo/%d_%.2f_mm.jpg",frameCount, distance);
	// 保存图像
	if (!cv::imwrite(dat, frame)) {
		num = std::snprintf(dat, sizeof(dat), "无法保存图像%d\n", frameCount);
		UsLogWrite(dat, num);
		return false;
	}

	//返回值point是检测到的坐标值。参数Point2d(0, 0)用于设定默认值，预留。
	P = task.getIntersection(framedata, frame, Point2d(0, 0))[0];   // 两个交点里的主交点

	auto end = std::chrono::high_resolution_clock::now();
	// 计算时间戳（直接使用高分辨率时钟）
	std::chrono::duration<double> duration = end - start;
	num = std::snprintf(dat, sizeof(dat), "图像%d,时间%f\n", frameCount, duration.count());
	UsLogWrite(dat, num);
	return true;
}
// 在图像上标记两个交点，并画出跟踪 ROI 条带与每个交点最终对应的两条直线
// P1：-86 度与 75 度直线的交点；P2：86 度与 -75 度直线的交点
static void DrawVideoOverlay(cv::Mat& img, const ImgData& data)
{
	// 跟踪 ROI：roiProcess 本帧实际裁剪的横向条带（最多保留两条，最新一条加粗并标注）
	for (size_t i = 0; i < data.roi_region.size(); ++i) {
		const cv::Rect rr = data.roi_region[i];
		const bool isCurrent = (i + 1 == data.roi_region.size());
		cv::rectangle(img, rr, cv::Scalar(0, 255, 0), isCurrent ? 2 : 1, cv::LINE_AA);
		// 把本帧真正画在图像上的 ROI 打到日志（坐标口径与 createRoiMasks 一致）
		std::cout << "显示ROI" << (isCurrent ? "(当前帧)" : "(上一帧)")
			<< ": x范围 [" << rr.x << ", " << rr.x + rr.width << "]"
			<< "  y范围 [" << rr.y << ", " << rr.y + rr.height << "]" << std::endl;
		if (isCurrent) {
			char roiText[64];
			std::snprintf(roiText, sizeof(roiText), "ROI x=%d w=%d", rr.x, rr.width);
			const int tx = (rr.x > 0 ? rr.x : 0) + 4;
			cv::putText(img, roiText, cv::Point(tx, 16), cv::FONT_HERSHEY_SIMPLEX,
				0.45, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
		}
	}

	// 最终与每个交点相关的两条直线：0 = 近垂直线，1 = 斜线
	// 精修成功的点画的是 RANSAC 拟合直线，没精修的画 Hough 线段，两者都已统一成 fitLine 形式
	const cv::Scalar finalLineColors[2][2] = {
		{ cv::Scalar(0, 255, 0),   cv::Scalar(255, 0, 0)   },   // P1: 绿 / 蓝
		{ cv::Scalar(255, 255, 0), cv::Scalar(0, 165, 255) }    // P2: 青 / 橙
	};
	const std::vector<cv::Vec4f>* const finalLines[2] = { &data.final_lines1, &data.final_lines2 };
	const double ext = std::sqrt((double)img.cols * img.cols + (double)img.rows * img.rows) + 10.0;
	for (int k = 0; k < 2; ++k) {
		const std::vector<cv::Vec4f>& ls = *finalLines[k];
		for (size_t i = 0; i < ls.size() && i < 2; ++i) {
			const double vx = ls[i][0];
			const double vy = ls[i][1];
			const double len = std::sqrt(vx * vx + vy * vy);
			if (len < 1e-6) continue;                       // 这条线本帧没检测到
			const double ux = vx / len;
			const double uy = vy / len;
			// 参考点沿方向向两端延长，保证直线贯穿整幅图像
			const cv::Point pt1(cvRound(ls[i][2] - ux * ext), cvRound(ls[i][3] - uy * ext));
			const cv::Point pt2(cvRound(ls[i][2] + ux * ext), cvRound(ls[i][3] + uy * ext));
			cv::line(img, pt1, pt2, finalLineColors[k][i], 2, cv::LINE_AA);
		}
	}

	// 第一交点：InterPointUpdate 已把它更新为最终值（含精修和单点回退）
	cv::Point2d point = data.intersection;
	bool hasPoint = point.x > 0 && point.y > 0 &&
		point.x < img.cols && point.y < img.rows;
	if (hasPoint) {
		const cv::Point pt(cvRound(point.x), cvRound(point.y));
		cv::circle(img, pt, 5, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
		char text[64];
		std::snprintf(text, sizeof(text), "P1 (%.1f, %.1f)", point.x, point.y);
		cv::putText(img, text, pt + cv::Point(8, -8), cv::FONT_HERSHEY_SIMPLEX,
			0.5, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
	}

	// 第二组直线的交点：86 度和 -75 度
	if (data.intersection2.x > 0 && data.intersection2.y > 0 &&
		data.intersection2.x < img.cols && data.intersection2.y < img.rows) {
		const cv::Point pt2(cvRound(data.intersection2.x), cvRound(data.intersection2.y));
		cv::circle(img, pt2, 5, cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
		char text2[64];
		std::snprintf(text2, sizeof(text2), "P2 (%.1f, %.1f)",
			data.intersection2.x, data.intersection2.y);
		cv::putText(img, text2, pt2 + cv::Point(8, -8), cv::FONT_HERSHEY_SIMPLEX,
			0.5, cv::Scalar(255, 0, 255), 1, cv::LINE_AA);
	}
}

// 把 cv::Mat 按比例居中绘制到 MFC 的图片控件上
static void ShowMatOnPicture(CWnd* pPic, const cv::Mat& src)
{
	if (pPic == NULL || !::IsWindow(pPic->GetSafeHwnd()) || src.empty())
		return;

	cv::Mat bgr;
	if (src.channels() == 4)
		cv::cvtColor(src, bgr, cv::COLOR_BGRA2BGR);
	else if (src.channels() == 1)
		cv::cvtColor(src, bgr, cv::COLOR_GRAY2BGR);
	else
		bgr = src;

	const int width = bgr.cols;
	const int height = bgr.rows;
	const int stride = ((width * 3 + 3) / 4) * 4;
	std::vector<uchar> buffer(static_cast<size_t>(stride) * height);
	for (int y = 0; y < height; ++y)
		memcpy(&buffer[static_cast<size_t>(y) * stride], bgr.ptr<uchar>(y),
			static_cast<size_t>(width) * 3);

	BITMAPINFO bmi = {};
	bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
	bmi.bmiHeader.biWidth = width;
	bmi.bmiHeader.biHeight = -height; // 负值表示图像从上到下存储
	bmi.bmiHeader.biPlanes = 1;
	bmi.bmiHeader.biBitCount = 24;
	bmi.bmiHeader.biCompression = BI_RGB;

	CRect rc;
	pPic->GetClientRect(&rc);
	CDC* pDC = pPic->GetDC();
	if (pDC == NULL)
		return;

	pDC->FillSolidRect(&rc, RGB(0, 0, 0));
	if (rc.Width() > 0 && rc.Height() > 0) {
		const double scale = std::min(static_cast<double>(rc.Width()) / width,
			static_cast<double>(rc.Height()) / height);
		const int drawW = std::max(1, static_cast<int>(width * scale));
		const int drawH = std::max(1, static_cast<int>(height * scale));
		const int x = (rc.Width() - drawW) / 2;
		const int y = (rc.Height() - drawH) / 2;
		::SetStretchBltMode(pDC->GetSafeHdc(), COLORONCOLOR);
		::StretchDIBits(pDC->GetSafeHdc(), x, y, drawW, drawH,
			0, 0, width, height, buffer.data(), &bmi, DIB_RGB_COLORS, SRCCOPY);
	}
	pPic->ReleaseDC(pDC);
}

void CPage1Dlg::OnTimer(UINT_PTR nIDEvent) {
	if (nIDEvent == 1)  // 定时器1
	{
		Tcount++;
		Rotate();
	}
	if (nIDEvent == 2)
	{
		RobotInit();
	}
	if (nIDEvent == 3)
	{
		Us();
		if(R1.LinearV[Left]<1) KillTimer(3);
	}
	if (nIDEvent == 4) {
		if(Xcount <= m_motorT) {
			UpdateData(TRUE);
			char dat[100];
			Port1.MotorCtrl(0, R1.W2PSC(m_edit4), 0, 0);
			LARGE_INTEGER currentTime;
			QueryPerformanceCounter(&currentTime);
			// 计算精确时间间隔（秒）
			double elapsed = static_cast<double>(currentTime.QuadPart - m_lastTime4.QuadPart)
				/ static_cast<double>(m_frequency.QuadPart);
			m_lastTime4 = currentTime;  // 更新上次触发时间
			//xxx= Xcount*2.50*m_edit4;//mm
			xxx = xxx + 5.0*m_edit4*elapsed;
			if (Xcount == 0) {
				xxx =0.0;
			}
			m_static1.Format(_T("distance: %.2f mm"),xxx);
			Point2d p;
			captureFrameWithDistance(cap, Xcount, xxx, timestamp,p);
			std::snprintf(dat, sizeof(dat), "(%f,%f\n",p.x, p.y);
			m_static1 = dat;
			UpdateData(FALSE);
			Xcount++;
		}
		else {
			Port1.MotorCtrl(0, 0, 0, 0);
			KillTimer(4);
		}
	}
	if (nIDEvent == 5) {
		if (R1.califlag == 1) 
		{ 
			CalibrateHori(); 
		}
		if (R1.califlag == 2)
		{
			CalibrateVerti();
		}
	}
	//if (nIDEvent == 6) {
	//	StartLineSequence(LineDistance);
	//}
	if (nIDEvent == 7) {
		SelfRotate(selfrotateangle, true);
	}
	if (nIDEvent == 8) {
		cv::Mat frame;
		if (cap.read(frame)) { /* cv::imshow("img",frame); */ }
	}
	if (nIDEvent == 9) {
		FindInitial();
	}
	if (nIDEvent == 10) {
		bool done = Hori_Move(horitemp);
		if (done && g_intersectionRecordAxis == 1)
			StopIntersectionRecord();
	}
	if (nIDEvent == 11) {
		bool done = Verti_Move(vertitemp);
		if (done && g_intersectionRecordAxis == 2)
			StopIntersectionRecord();
	}
	if (nIDEvent == 12) {
		FindAngle();
	}
	if (nIDEvent == 13) {
		if (!cap.isOpened()) {
			KillTimer(13);
			return;
		}

		cv::Mat frame;
		if (!cap.read(frame) || frame.empty()) {
			// 摄像头偶尔丢帧时继续尝试，连续失败则停止采集
			if (++g_captureFailCount >= 10) {
				KillTimer(13);
				g_captureFailCount = 0;
				StopIntersectionRecord();
				AfxMessageBox(_T("摄像头读取失败，已停止采集。"));
			}
			return;
		}
		g_captureFailCount = 0;

		// 对当前摄像头帧执行图像处理
		g_videoTask.getIntersection(g_videoData, frame, cv::Point2d(0, 0));
		WriteIntersectionRecord();

		// 把拟合直线和交点叠加到原始帧上，再显示到 IDC_PICTURE
		cv::Mat display = frame.clone();
		DrawVideoOverlay(display, g_videoData);
		ShowMatOnPicture(GetDlgItem(IDC_PICTURE), display);
	}
	if (nIDEvent == 14) {
		if (!g_videoFileCap.isOpened()) {
			KillTimer(14);
			return;
		}

		cv::Mat frame;
		if (!g_videoFileCap.read(frame) || frame.empty()) {
			// 视频播放结束：停止播放并回到第一帧
			KillTimer(14);
			g_videoFileCap.set(cv::CAP_PROP_POS_FRAMES, 0);
			return;
		}

		// 缓存当前视频帧，急停后“直线方程显示”使用这一帧
		g_lastVideoFrame = frame.clone();

		// 与加载图像按钮保持一致的图像处理与显示流程
		g_videoTask.getIntersection(g_videoData, frame, cv::Point2d(0, 0));
		WriteIntersectionRecord();

		cv::Mat display = frame.clone();
		DrawVideoOverlay(display, g_videoData);
		ShowMatOnPicture(GetDlgItem(IDC_PICTURE), display);
	}
	CDialogEx::OnTimer(nIDEvent);  // 调用基类的 OnTimer
}

void CPage1Dlg::OnBnClickedButton8()
{
	KillTimer(1);
	KillTimer(2);
	KillTimer(3);
	if (!cap.isOpened()) {
		std::cerr << "无法打开摄像头" << std::endl;
		return ;
	}
	// 设置摄像头参数
	cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
	cap.set(cv::CAP_PROP_FPS, 30);
	Sleep(3000);
	Xcount = 0;
	SetTimer(4, 500, nullptr);  // 第一个参数是定时器 ID，第二个参数是间隔时间（毫秒）
	//UpdateData(FALSE);
}

vector<double> sumx, sumx_rmv, pointx, horidis;
vector<double> sumy, sumy_rmv, pointy, vertidis, a, b;
void CPage1Dlg::OnBnClickedButton9()	// 水平标定
{
	// TODO: 在此添加控件通知处理程序代码
	R1.califlag = 1;	//	水平标定标志，定时器5里用于判定执行水平还是竖直标定
	R1.Hcaliflag = 0;
	R1.calicnt = 0;
	sumx.clear();		//	清空数据
	sumx_rmv.clear();
	R1.HoriDis.clear();
	R1.HoriPoints.clear();
	R1.HoriOut = 30.0;
	SetTimer(5, 1000, nullptr);	//定时器5执行水平标定程序
}

void CPage1Dlg::CalibrateHori()
{
	switch (R1.Hcaliflag)
	{
	case 0:
	{
		if (!cap.isOpened()) return;
		cv::Mat frame;// 丢弃前 N 帧，保留最新一帧
		const int N = 8;
		Log.OpenFile("HORICALIBRATE", Log.HoriCalibrateFile);

		for (int i = 0; i < 10; i++)
		{
			for (int i = 0; i < N; ++i) cap.grab();   // 只抓不解码，最快
			if (cap.retrieve(frame) && !frame.empty())	// 把 最近一次 cap.grab() 抓到的图像 解码到 frame，成功返回true
			{
				double x_temp = (task.getIntersection(framedata, frame, Point2d(0, 0)))[0].x;
				sumx.push_back(x_temp);
				Log.HoriCalibrateFile << x_temp << "\n";
			}
			else {	//	如果解码失败或者得到空帧，则忽略此次计数。确保每次得到10个数据
				i--;
			}
		}
		sumx_rmv = removeOutliersIQRSmallSample(sumx);

		for (int i = 0; i < sumx_rmv.size(); i++)
		{
			R1.HoriPoints.push_back(sumx_rmv[i]);
			R1.HoriDis.push_back(30.0 - R1.HoriOut);
		}

		Log.HoriCalibrateFile << "-----------------------" << "\n";
		for (int i = 0; i < size(sumx_rmv); i++)
		{
			Log.HoriCalibrateFile << sumx_rmv[i] << "\t";
			Log.HoriCalibrateFile << 30.0 - R1.HoriOut << "\n";
		}
		//pointx.push_back(accumulate(sumx_rmv.begin(), sumx_rmv.end(), 0.0) / sumx_rmv.size());
		//horidis.push_back(30.0 - R1.HoriOut);
		
		sumx.clear();
		sumx_rmv.clear();

		horitemp = -2.0;
		R1.isHoriMove = false;// 重置水平移动状态
		R1.HoriMoveCpFlag = false;
		SetTimer(10, 1, nullptr);	//	触发水平移动函数定时器	
		R1.Hcaliflag = 1;	//	切换到状态1等待水平移动执行完
		Log.CloseFile(Log.HoriCalibrateFile);
		break;
	}
	case 1:
	{
		if (R1.HoriMoveCpFlag)	//	等待水平移动执行完
		{
			R1.Hcaliflag = 0;	//	切换到状态0获取点x和对应的距离
			R1.calicnt++;
			if (R1.calicnt >= 10)	//	测得10组数据
			{
				R1.Hcaliflag = 2;
			}
		}
		break;
	}
	case 2:
	{
		//auto coefficients = LinearFit(pointx, horidis, 'x');	//	拟合系数并绘图
		//auto coefficients = LinearFit(a, b, 'x');	//	拟合系数并绘图
		auto coefficients = LinearFit(R1.HoriPoints, R1.HoriDis, 'x');	//	拟合系数并绘图
		R1.Hori_a = coefficients.first;
		R1.Hori_b = coefficients.second;
		KillTimer(5);
		R1.califlag = 0;
		break;
	}
	default:
		break;
	}
}

void CPage1Dlg::OnBnClickedButton18()	//	竖直标定
{
	// TODO: 在此添加控件通知处理程序代码
	R1.califlag = 2;
	R1.Vcaliflag = 0;
	R1.calicnt = 0;
	sumy.clear();
	sumy_rmv.clear();
	//pointy.clear();
	//vertidis.clear();
	R1.VertiDis.clear();
	R1.VertiPoints.clear();
	R1.VertOut = 35.0;
	SetTimer(5, 1000, nullptr);	//定时器5执行标定程序
}

void CPage1Dlg::CalibrateVerti()
{
	switch (R1.Vcaliflag)
	{
	case 0:
	{
		if (!cap.isOpened()) return;
		cv::Mat frame;// 丢弃前 N 帧，保留最新一帧
		const int N = 8;
		Log.OpenFile("VERTICALIBRATE", Log.VertiCalibrateFile);

		for (int i = 0; i < 10; i++)
		{
			for (int i = 0; i < N; ++i) cap.grab();   // 只抓不解码，最快
			if (cap.retrieve(frame) && !frame.empty())
			{
				double y_temp = (task.getIntersection(framedata, frame, Point2d(0, 0)))[0].y;
				sumy.push_back(y_temp);
				Log.VertiCalibrateFile << y_temp << "\n";
			}
			else {
				i--;
			}
		}
		sumy_rmv = removeOutliersIQRSmallSample(sumy);

		for (int i = 0; i < sumy_rmv.size(); i++)
		{
			R1.VertiPoints.push_back(sumy_rmv[i]);
			R1.VertiDis.push_back(R1.VertOut - 35.0);
		}

		Log.VertiCalibrateFile << "-----------------------" << "\n";
		for (int i = 0; i < size(sumy_rmv); i++)
		{
			Log.VertiCalibrateFile << sumy_rmv[i] << "\t";
			Log.VertiCalibrateFile << R1.VertOut - 35.0 << "\n";
		}
		//pointy.push_back(accumulate(sumy_rmv.begin(), sumy_rmv.end(), 0.0) / sumy_rmv.size());
		//vertidis.push_back(R1.VertOut - 35.0);
		
		sumy.clear();
		sumy_rmv.clear();

		vertitemp = 1.0;
		R1.isVertiMove = false;// 重置竖直移动状态
		R1.VertiMoveCpFlag = false;
		SetTimer(11, 1, nullptr);	//	触发竖直移动函数定时器
		R1.Vcaliflag = 1;	//	切换到状态1等待水平移动执行完
		Log.CloseFile(Log.VertiCalibrateFile);
		break;
	}
	case 1:
	{
		if (R1.VertiMoveCpFlag)	//	等待水平移动执行完
		{
			R1.Vcaliflag = 0;	//	切换到状态0获取点x和对应的距离
			R1.calicnt++;
			if (R1.calicnt >= 10)	//	测得10组数据
			{
				R1.Vcaliflag = 2;
			}
		}
		break;
	}
	case 2:
	{
		//auto coefficients = LinearFit(pointy, vertidis, 'y');
		//auto coefficients = LinearFit(a, b, 'y');
		auto coefficients = LinearFit(R1.VertiPoints, R1.VertiDis, 'y');
		R1.Verti_a = coefficients.first;
		R1.Verti_b = coefficients.second;
		KillTimer(5);
		R1.califlag = 0;
		break;
	}
	default:
		break;
	}
}

//double Angle0 = 0.0;
//double Angle0Target = 0.0;
//double CurrentAngle = 0.0;
//double H1dis = 125.0;
//double H1Target = 0;
//double L2dis = 240.0;
//double L2Target = 0;
//double X0 = 0.0;
//double X0Target = 0.0;
//double tempdistance = 0.0;
bool LineFlag = false;
void CPage1Dlg::StartLineSequence(double distance)
{
	UpdateData(TRUE);
	if (!LineFlag && distance > 0.05)
	{
		LineFlag = true;
		R1.distance = distance;
		X0Target = X0 + distance;
		if (distance > 0)
		{
			R1.LinearV[Left] = 2.0; // 左轮速度
			R1.LinearV[Right] = 2.0; // 右轮速度
		}
		if (distance < 0)
		{
			R1.LinearV[Left] = -2.0; // 左轮速度
			R1.LinearV[Right] = -2.0; // 右轮速度
		}
		// 应用速度控制
		R1.V2PSC();	// 线速度均转换为PSC数据
		Port1.MotorCtrl(R1.PSC[0], R1.PSC[1], R1.PSC[2], R1.PSC[3]);
		QueryPerformanceCounter(&m_lastTime10);  // 记录启动时刻,只执行一次
	}
	LARGE_INTEGER currentTime;
	QueryPerformanceCounter(&currentTime);
	// 计算精确时间间隔（秒）
	double elapsed = static_cast<double>(currentTime.QuadPart - m_lastTime10.QuadPart) / static_cast<double>(m_frequency.QuadPart);
	m_lastTime10 = currentTime;  // 更新上次触发时间
	X0 = X0 + R1.LinearV[Left] * elapsed;
	if (abs(X0Target - X0)  <= 0.05)
	{
		KillTimer(6);
		R1.ClearAllV();          //清空所有电机线速度
		R1.V2PSC();              //更新速度到PSC
		Port1.MotorCtrl(R1.PSC);  // 所有电机速度为0
	}
	char dat[100];
	std::snprintf(dat, sizeof(dat), "Angle:%.2f\nVertDis:%.2f\nHoriDis:%.2f\nX0:%.2f", R1.AngleOut, R1.VertOut, R1.HoriOut, X0);
	m_static1 = dat;
	GetDlgItem(IDC_STATIC1)->SetWindowText(m_static1);
	//UpdateData(FALSE);
}

void CPage1Dlg::OnBnClickedButton10()	// 直线行走
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);

	//// 设置定时器6用于处理图像
	//SetTimer(6, 10, nullptr); // 1ms控制周期
	//LineFlag = false;
	unsigned short VertSpeed = 0, HorSpeed = 0, LeftSpeed = 0, RightSpeed = 0;
	//左轮//右轮
	if (LineDistance < 0)
	{
		LeftSpeed = -R1.W2PSC(1.0);
		RightSpeed = -R1.W2PSC(1.0);
	}
	else
	{
		LeftSpeed = R1.W2PSC(1.0);
		RightSpeed = R1.W2PSC(1.0);
	}
	Port1.MotorCtrl(VertSpeed, HorSpeed, LeftSpeed, RightSpeed);

	X0 = X0 + LineDistance;
	char dat[100];
	std::snprintf(dat, sizeof(dat), "Angle:%.2f\nVertDis:%.2f\nHoriDis:%.2f\nX0:%.2f", R1.AngleOut, R1.VertOut, R1.HoriOut, X0);
	m_static1 = dat;
	GetDlgItem(IDC_STATIC1)->SetWindowText(m_static1);

	UpdateData(FALSE);
}

void CPage1Dlg::OnBnClickedButton11()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);	// 从控件读取数据

	// 重置旋转状态
	R1.isSelfRotate = false;
	SetTimer(7, 1, nullptr);
	//UpdateData(FALSE);
}

/*若要单独使用这个旋转功能，按照以下步骤
1、重置旋转状态
2、开启1个定时器
3、在定时器里面调用SelfRotate函数
*/
bool CPage1Dlg::SelfRotate(double SetAngle, bool enableCamera)	//以小车轮距中心为中心旋转
{
	UpdateData(TRUE);

	if(!R1.isSelfRotate && abs(SetAngle) >= 0.05)
	{
		R1.isSelfRotate = true;	
		R1.TargetAngleOut = R1.AngleOut + SetAngle;

		if (enableCamera)	// 是否录视频可选，默认不录
		camera.OpenFile(R1.AngleOut, R1.TargetAngleOut);

		Log.OpenFile("selfrotation", Log.SelfRotationFile);

		R1.LinearV[Left] = m_speed; // 左轮速度
		R1.LinearV[Right] = m_speed; // 右轮速度
		R1.V2PSC();	// 应用速度控制										 // 线速度均转换为PSC数据
		if (SetAngle >= 0)
		{
			Port1.MotorCtrl(R1.PSC[0], R1.PSC[1], R1.PSC[2], -R1.PSC[3]);
		}
		else
		{
			Port1.MotorCtrl(R1.PSC[0], R1.PSC[1], -R1.PSC[2], R1.PSC[3]);
		}
		QueryPerformanceCounter(&m_lastTime7);  // 记录启动时刻
	}

	if (enableCamera) camera.SaveFrame();

	LARGE_INTEGER currentTime;
	QueryPerformanceCounter(&currentTime);
	// 计算精确时间间隔（秒）
	double elapsed = static_cast<double>(currentTime.QuadPart - m_lastTime7.QuadPart) / static_cast<double>(m_frequency.QuadPart);
	m_lastTime7 = currentTime;  // 更新上次触发时间
	if (SetAngle > 0) {
		R1.AngleOut = R1.AngleOut + ((2.0 * m_speed) / Tread)*elapsed/ M_PI*180.0;
	}
	else
	{
		R1.AngleOut = R1.AngleOut - ((2.0 * m_speed) / Tread)*elapsed / M_PI*180.0;
	}

	FileUtils::LogDataStruct LogData;
	LogData = R1.GetData();
	LogData.SysTime = GetCurrentTime();
	LogData.point = cv::Point2d(0,0);
	double ms = (double)currentTime.QuadPart * 1000.0 / m_frequency.QuadPart; // 相对值，ms
	//LogData.currentTime = currentTime;
	LogData.currentTime = ms;
	Log.FileSaveData(Log.SelfRotationFile, LogData);

	char dat[100];
	std::snprintf(dat, sizeof(dat), "Angle:%.2f\nVertDis:%.2f\nHoriDis:%.2f\nX0:%.2f", R1.AngleOut, R1.VertOut, R1.HoriOut, X0);
	m_static1 = dat;
	GetDlgItem(IDC_STATIC1)->SetWindowText(m_static1);
	//UpdateData(FALSE);

	if ((SetAngle > 0 && R1.AngleOut >= R1.TargetAngleOut) || (SetAngle < 0 && R1.AngleOut <= R1.TargetAngleOut) || (abs(R1.AngleOut - R1.TargetAngleOut)<0.05))
	{
		KillTimer(7);	// 结束定时器
		R1.ClearAllV();          //清空所有电机线速度
		R1.V2PSC();              //更新速度到PSC
		Port1.MotorCtrl(R1.PSC);  // 所有电机速度为0
		if (enableCamera) {
			camera.CloseFile();
			camera.NewName(R1.AngleOut);
		}
		Log.CloseFile(Log.SelfRotationFile);
		return true;
	}
	return false;
}


void CPage1Dlg::OnBnClickedButton12()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);	// 从控件读取数据

	SetTimer(8, 100, nullptr);

	// 创建用于保存照片的文件夹
	LPCWSTR folderName = L"SamplePhotos";
	if (CreateDirectory(folderName, NULL) || GetLastError() == ERROR_ALREADY_EXISTS) {
		// 文件夹创建成功或已存在
	}
	else {
		// 创建文件夹失败
		AfxMessageBox(_T("无法创建 SamplePhotos 文件夹！"));
		return;
	}

	// 拍照并保存
	cv::Mat frame;
	if (cap.read(frame) && !frame.empty()) {
		// 创建文件名，包含角度信息
		char filename[200];
		std::snprintf(filename, sizeof(filename), "SamplePhotos/%.2f_%.2f_%.2f_%.2f.jpg",
			R1.AngleOut, // 角度值
			R1.VertOut,  // H1值
			R1.HoriOut,  // L2值
			X0);	// X方向位移
		cv::imwrite(filename, frame);// 保存图像
	}

	SampleLogFile.open("SampleData.txt", std::ios::out);
	if (!SampleLogFile.is_open()) {
		AfxMessageBox(_T("无法打开 SampleData.txt 文件！"));
		return;
	}

	if (SampleLogFile.is_open()) {
		SampleLogFile << "Angle(deg)\tH1\tL2\tX0\n";
	}

	// 记录到文件
	if (SampleLogFile.is_open()) {
		SampleLogFile << R1.AngleOut << "\t"
			<< R1.VertOut << "\t"
			<< R1.HoriOut << "\t"
			<< X0 << "\n";
	}
	char dat[100];
	std::snprintf(dat, sizeof(dat), "Angle:%.2f\nVertDis:%.2f\nHoriDis:%.2f\nX0:%.2f", R1.AngleOut, R1.VertOut, R1.HoriOut, X0);
	m_static1 = dat;
	GetDlgItem(IDC_STATIC1)->SetWindowText(m_static1);
	//UpdateData(FALSE);
}

// 加载图像按钮：启动摄像头采集，每 100 ms 取一帧，处理后显示在 IDC_PICTURE 上
void CPage1Dlg::OnBnClickedButtonLoadVideo()
{
	KillTimer(13);
	KillTimer(14);
	KillTimer(8);  // 避免与采样预览定时器同时读取同一个摄像头
	StopIntersectionRecord();
	g_videoSourceFlag = 0;
	g_useFrozenFrame = false;
	g_lastVideoFrame.release();
	if (g_videoFileCap.isOpened())
		g_videoFileCap.release();

	if (!cap.isOpened()) {
		if (!cap.open(0)) {
			AfxMessageBox(_T("无法打开摄像头！"));
			return;
		}
	}

	// 重新开始采集时，图像处理从首帧状态开始
	g_videoData = ImgData();
	g_videoTask.Reset();
	g_captureFailCount = 0;

	// 100 ms 定时器：每秒处理 10 帧
	SetTimer(13, 100, nullptr);
}

// 按视频标志位获取一帧：标志位为 1 读取视频帧，为 0 读取摄像头帧
static bool GetProcessFrame(cv::Mat& frame)
{
	if (g_videoSourceFlag == 1) {
		if (!g_videoFileCap.isOpened()) {
			if (g_videoFilePath.empty() || !g_videoFileCap.open(g_videoFilePath)) {
				AfxMessageBox(_T("无法打开视频文件！"));
				return false;
			}
		}
		if (!g_videoFileCap.read(frame) || frame.empty()) {
			// 读到视频末尾时回到第一帧再读
			g_videoFileCap.set(cv::CAP_PROP_POS_FRAMES, 0);
			if (!g_videoFileCap.read(frame) || frame.empty()) {
				AfxMessageBox(_T("读取视频帧失败！"));
				return false;
			}
		}
		return true;
	}

	if (!cap.isOpened() && !cap.open(0)) {
		AfxMessageBox(_T("无法打开摄像头！"));
		return false;
	}
	if (!cap.read(frame) || frame.empty()) {
		AfxMessageBox(_T("读取摄像头帧失败！"));
		return false;
	}
	return true;
}

// 将 CString 转换为 UTF-8，便于写入 txt
static std::string ToUtf8(const CString& text)
{
	if (text.IsEmpty())
		return std::string();
	int len = WideCharToMultiByte(CP_UTF8, 0, text, -1, NULL, 0, NULL, NULL);
	if (len <= 0)
		return std::string();
	std::string out(len, '\0');
	WideCharToMultiByte(CP_UTF8, 0, text, -1, &out[0], len, NULL, NULL);
	out.resize(len - 1);
	return out;
}

// 停止交点记录
static void StopIntersectionRecord()
{
	if (g_intersectionLog.is_open()) {
		g_intersectionLog.flush();
		g_intersectionLog.close();
	}
	g_intersectionRecordAxis = 0;
	g_intersectionFrameIndex = 0;
}

// 将当前帧处理得到的两个交点写入 txt
static void WriteIntersectionRecord()
{
	if (g_intersectionRecordAxis == 0 || !g_intersectionLog.is_open())
		return;

	cv::Point2d p1 = g_videoData.intersection;
	cv::Point2d p2 = g_videoData.intersection2;

	// 只有两个交点都有效时才记录
	if (p1.x <= 0 || p1.y <= 0 || p2.x <= 0 || p2.y <= 0)
		return;

	// 按 x 坐标区分左点和右点
	if (p1.x > p2.x)
		std::swap(p1, p2);

	++g_intersectionFrameIndex;

	SYSTEMTIME st;
	GetLocalTime(&st);

	CString line;
	line.Format(_T("（%.1f，%.1f）\t（%.1f，%.1f）\t%02d:%02d:%02d.%03d\t第%d帧\r\n"),
		p1.x, p1.y, p2.x, p2.y,
		st.wHour, st.wMinute, st.wSecond, st.wMilliseconds,
		g_intersectionFrameIndex);

	g_intersectionLog << ToUtf8(line);
	g_intersectionLog.flush();
}

// 开始记录：水平按钮 axis=1，竖直按钮 axis=2
static void StartIntersectionRecord(CWnd* pWnd, int axis)
{
	StopIntersectionRecord();

	if (axis != 1 && axis != 2)
		return;

	CreateDirectory(L"txt", NULL);
	const char* path = (axis == 1) ? "txt/水平交点.txt" : "txt/竖直交点.txt";
	g_intersectionLog.open(path, std::ios::out | std::ios::binary);
	if (!g_intersectionLog.is_open()) {
		AfxMessageBox(_T("无法创建交点记录文件！"));
		return;
	}

	const char utf8Bom[3] = { (char)0xEF, (char)0xBB, (char)0xBF };
	g_intersectionLog.write(utf8Bom, 3); // UTF-8 BOM
	g_intersectionLog << ToUtf8(_T("左点1\t右点2\t时间\t帧数\r\n"));
	g_intersectionLog.flush();

	g_intersectionRecordAxis = axis;
	g_intersectionFrameIndex = 0;

	// 交点记录使用摄像头帧；停止视频文件和其他摄像头预览，避免争用摄像头
	pWnd->KillTimer(14);
	pWnd->KillTimer(8);
	g_videoSourceFlag = 0;
	g_useFrozenFrame = false;
	g_lastVideoFrame.release();
	if (!cap.isOpened() && !cap.open(0)) {
		AfxMessageBox(_T("无法打开摄像头！"));
		StopIntersectionRecord();
		return;
	}

	// 从新的图像处理状态开始，摄像头帧每 100 ms 处理一次
	g_videoData = ImgData();
	g_videoTask.Reset();
	pWnd->SetTimer(13, 100, nullptr);
}

// 图像处理测试：已加载视频时读取视频帧，否则读取摄像头帧，显示预处理和 Hough 直线检测的中间结果
void CPage1Dlg::OnBnClickedButton6()
{
	cv::Mat frame;
	if (g_useFrozenFrame && !g_lastVideoFrame.empty())
		frame = g_lastVideoFrame;
	else if (!GetProcessFrame(frame))
		return;

	// testData 独立于 framedata/g_videoData，避免影响正常识别状态
	ImgData testData;
	g_showImageDebug = true;
	task.HoughgetIntersection(testData, frame, cv::Point2d(0, 0));
	g_showImageDebug = false;

	// 构造最终 Hough 检测结果图像，仅用于 OpenCV 调试窗口显示
	cv::Mat display = frame.clone();

	// 先画出 HoughLinesP 检测到的所有长直线
	for (const auto& l : testData.long_lines) {
		cv::line(display,
			cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
			cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
	}

	// 再叠加筛选后的目标直线、ROI 和交点
	DrawVideoOverlay(display, testData);

	cv::imshow("result", display);
	cv::waitKey(30);
}

// 直线方程显示：按视频标志位读取视频帧或摄像头帧，执行 Hough 检测并显示全部直线方程和倾角
void CPage1Dlg::OnBnClickedButton7()
{
	cv::Mat frame;
	if (g_useFrozenFrame && !g_lastVideoFrame.empty())
		frame = g_lastVideoFrame;
	else if (!GetProcessFrame(frame))
		return;

	// 与 FrameProcess 使用相同的 Hough 检测流程，但不影响正常识别状态
	ImgData data;
	data.inputcolor = frame;

	ImgProcess process;
	img_lines lines;
	process.preprocessImage(data);
	lines.detectLongLines(data.skeleton, data.inputcolor, data.long_lines, 30, 30, 100);

	// 按直线长度从长到短排序，长度相同的先保留
	std::vector<cv::Vec4i> allLines = data.long_lines;
	std::sort(allLines.begin(), allLines.end(),
		[](const cv::Vec4i& a, const cv::Vec4i& b) {
			return norm(Point(a[0], a[1]) - Point(a[2], a[3])) >
			       norm(Point(b[0], b[1]) - Point(b[2], b[3]));
		});

	// 计算直线倾角。直线无方向，按 0～180° 的最小夹角比较
	auto lineAngle = [&lines](cv::Vec4i l) {
		LineEquation eq = lines.getLineEquation(l);
		return eq.is_vertical ? 90.0 : atan(eq.a) * 180.0 / CV_PI;
	};
	auto angleDiff = [](double a, double b) {
		double d = std::fabs(a - b);
		if (d > 180.0) d = std::fmod(d, 180.0);
		return std::min(d, 180.0 - d);
	};

	// 角度相差 2° 以内、且到原点距离相差 50 像素以内的直线只保留最长的那一条
	std::vector<cv::Vec4i> displayLines;
	cv::Point origin(0, 0);
	for (const auto& line : allLines) {
		const double angle = lineAngle(line);
		LineEquation eqLine = lines.getLineEquation(line);
		const double dist = lines.pointToLineDistance(origin, eqLine);
		bool hasSimilar = false;
		for (const auto& kept : displayLines) {
			LineEquation eqKept = lines.getLineEquation(kept);
			const double distKept = lines.pointToLineDistance(origin, eqKept);
			if (angleDiff(angle, lineAngle(kept)) <= 2.0 &&
				std::fabs(dist - distKept) < 50.0) {
				hasSimilar = true;
				break;
			}
		}
		if (!hasSimilar)
			displayLines.push_back(line);
	}

	CString msg;
	msg.Format(_T("Hough 变换共检测到 %d 条直线，角度去重后显示 %d 条：\r\n\r\n"),
		(int)data.long_lines.size(), (int)displayLines.size());

	CString info;
	for (size_t i = 0; i < displayLines.size(); ++i) {
		LineEquation eq = lines.getLineEquation(displayLines[i]);
		double angle = eq.is_vertical ? 90.0 : atan(eq.a) * 180.0 / CV_PI;
		if (eq.is_vertical)
			info.Format(_T("%d. x = %.2f，倾角：%.2f°\r\n"), (int)(i + 1), eq.x0, angle);
		else
			info.Format(_T("%d. y = %.4fx + %.4f，倾角：%.2f°\r\n"), (int)(i + 1), eq.a, eq.b, angle);
		msg += info;
	}

	if (displayLines.empty())
		msg += _T("未检测到直线");
	else
		msg += _T("\r\n（倾角为图像坐标系下相对水平方向的夹角）");

	AfxMessageBox(msg);
}

// 视频加载：选择视频文件，逐帧执行与“加载图像”按钮相同的图像处理并显示
void CPage1Dlg::OnBnClickedButton21()
{
	CFileDialog dlg(TRUE, NULL, NULL,
		OFN_HIDEREADONLY | OFN_FILEMUSTEXIST,
		_T("视频文件 (*.avi;*.mp4;*.mov;*.mkv;*.wmv)|*.avi;*.mp4;*.mov;*.mkv;*.wmv|所有文件 (*.*)|*.*||"),
		this);
	if (dlg.DoModal() != IDOK)
		return;

	KillTimer(13);
	KillTimer(14);
	KillTimer(8);
	StopIntersectionRecord();
	if (g_videoFileCap.isOpened())
		g_videoFileCap.release();

	// OpenCV 在 Windows 下使用窄字符路径，这里按系统 ANSI 代码页转换
	CStringA pathA(dlg.GetPathName());
	g_videoFilePath = std::string(pathA.GetString());
	if (!g_videoFileCap.open(g_videoFilePath)) {
		g_videoSourceFlag = 0;
		AfxMessageBox(_T("无法打开视频文件！"));
		return;
	}
	g_videoSourceFlag = 1;
	g_useFrozenFrame = false;
	g_lastVideoFrame.release();

	g_videoData = ImgData();
	g_videoTask.Reset();

	// 100 ms 定时器：每秒处理 10 帧
	SetTimer(14, 500, nullptr);
}

//水平滑块移动指定距离
void CPage1Dlg::OnBnClickedButton13()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);	// 从控件读取数据

	StartIntersectionRecord(this, 1);  // 水平按钮：开始记录交点
	R1.isHoriMove = false;// 重置水平移动状态
	R1.HoriMoveCpFlag = false;
	SetTimer(10, 1, nullptr);
	//UpdateData(FALSE);
}

bool CPage1Dlg::Hori_Move(double distance, bool enableCamera)
{
	UpdateData(TRUE);
	if (!R1.isHoriMove && abs(distance) > 0.05)
	{
		R1.isHoriMove = true;
		//R1.TargetHoriOut = R1.HoriOut + horitemp;
		R1.TargetHoriOut = R1.HoriOut + distance;

		if (enableCamera)	// 是否录视频可选，默认不录制
		camera.OpenFile(R1.HoriOut, R1.TargetHoriOut);

		//if (horitemp >= 0) {
		if (distance > 0) {
			R1.LinearV[Hori] = 1.0; // 水平滑块速度
		}
		if (distance < 0) {
			R1.LinearV[Hori] = -1.0; // 水平滑块速度
		}
		R1.V2PSC();// 线速度均转换为PSC数据
		Port1.MotorCtrl(R1.PSC[0], R1.PSC[1], R1.PSC[2], R1.PSC[3]);// 应用速度控制
		QueryPerformanceCounter(&m_lastTime13);  // 记录启动时刻
	}

	if (enableCamera) camera.SaveFrame();

	LARGE_INTEGER currentTime;
	QueryPerformanceCounter(&currentTime);
	// 计算精确时间间隔（秒）
	double elapsed = static_cast<double>(currentTime.QuadPart - m_lastTime13.QuadPart) / static_cast<double>(m_frequency.QuadPart);
	m_lastTime13 = currentTime;  // 更新上次触发时间
	R1.HoriOut = R1.HoriOut + elapsed*R1.LinearV[Hori];

	char dat[100];
	std::snprintf(dat, sizeof(dat), "Angle:%.2f\nVertDis:%.2f\nHoriDis:%.2f\nX0:%.2f", R1.AngleOut, R1.VertOut, R1.HoriOut, X0);
	m_static1 = dat;
	GetDlgItem(IDC_STATIC1)->SetWindowText(m_static1);
	//UpdateData(FALSE);

	if ((distance > 0 && R1.HoriOut >= R1.TargetHoriOut) || (distance < 0 && R1.HoriOut <= R1.TargetHoriOut) || (abs(R1.HoriOut - R1.TargetHoriOut) <= 0.05))
	{
		R1.ClearAllV();          //清空所有电机线速度
		R1.V2PSC();              //更新速度到PSC
		Port1.MotorCtrl(R1.PSC);  // 所有电机速度为0
		KillTimer(10);
		R1.HoriMoveCpFlag = true;
		if (enableCamera) {
			camera.CloseFile();
			camera.NewName(R1.HoriOut);		// 更新保存.avi文件的名称
		}
		return true;
	}
	return false;

}

//竖直滑块移动指定距离
void CPage1Dlg::OnBnClickedButton14()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);	// 从控件读取数据

	StartIntersectionRecord(this, 2);  // 竖直按钮：开始记录交点
	R1.isVertiMove = false;// 重置水平移动状态
	R1.VertiMoveCpFlag = false;
	SetTimer(11, 1, nullptr);

	//UpdateData(FALSE);
}

bool CPage1Dlg::Verti_Move(double distance, bool enableCamera)
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);	// 从控件读取数据
	if (!R1.isVertiMove && abs(distance) > 0.05)
	{
		R1.isVertiMove = true;
		//R1.TargetVertOut = R1.VertOut + vertitemp;
		R1.TargetVertOut = R1.VertOut + distance;

		if (enableCamera)	// 是否录视频可选，默认不录
		camera.OpenFile(R1.VertOut, R1.TargetVertOut);

		if (distance > 0) {
			R1.LinearV[Vert] = -1.0; // 数值滑块速度
		}
		if (distance < 0) {
			R1.LinearV[Vert] = 1.0; // 数值滑块速度
		}
		R1.V2PSC();// 线速度均转换为PSC数据
		Port1.MotorCtrl(R1.PSC[0], R1.PSC[1], R1.PSC[2], R1.PSC[3]);// 应用速度控制
		QueryPerformanceCounter(&m_lastTime13);  // 记录启动时刻
	}

	if (enableCamera) camera.SaveFrame();

	LARGE_INTEGER currentTime;
	QueryPerformanceCounter(&currentTime);
	// 计算精确时间间隔（秒）
	double elapsed = static_cast<double>(currentTime.QuadPart - m_lastTime13.QuadPart) / static_cast<double>(m_frequency.QuadPart);
	m_lastTime13 = currentTime;  // 更新上次触发时间
	R1.VertOut = R1.VertOut + elapsed*(-R1.LinearV[Vert]);

	char dat[100];
	std::snprintf(dat, sizeof(dat), "Angle:%.2f\nVertDis:%.2f\nHoriDis:%.2f\nX0:%.2f", R1.AngleOut, R1.VertOut, R1.HoriOut, X0);//将格式化字符串写入数组dat
	m_static1 = dat;
	GetDlgItem(IDC_STATIC1)->SetWindowText(m_static1);
	//UpdateData(FALSE);

	if ((distance > 0 && R1.VertOut >= R1.TargetVertOut) || (distance < 0 && R1.VertOut <= R1.TargetVertOut) || abs(R1.VertOut - R1.TargetVertOut) < 0.05)
	{
		R1.ClearAllV();          //清空所有电机线速度
		R1.V2PSC();              //更新速度到PSC
		Port1.MotorCtrl(R1.PSC);  // 所有电机速度为0
		KillTimer(11);
		R1.VertiMoveCpFlag = true;
		if (enableCamera) {
			camera.CloseFile();
			camera.NewName(R1.VertOut);
		}
		return true;
	}
	return false;
}

//寻找初始焊接位置
void CPage1Dlg::OnBnClickedButton15()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);	// 从控件读取数据

	R1.PosTemp.clear();
	R1.angles.clear();
	R1.scanflag = 0;
	R1.FindInitialState = Robot::State::ScanTurnFlag;
	SetTimer(9, 200, nullptr);
	//UpdateData(FALSE);
}

//// 定义全局变量容器存储检测到的交点
//vector<Point2d> PosTemp;
//vector<double> angles;
//double Angle1 = 0.0;
//double Angle1Target = -20.0;
void CPage1Dlg::FindInitial()	//以小车轮距中心为中心旋转
{
	UpdateData(TRUE);
	switch (R1.FindInitialState)
	{
	case Robot::State::ScanTurnFlag: 
		R1.isSelfRotate = false;	//	重置旋转状态
		R1.AngleOut = 0.0;
		R1.FindInitialState = Robot::State::Scan;
		break;
	case Robot::State::Scan: 
		//if(RobotScan(-20.0))	// 扫描结束切换状态
		RobotScanAdaptive();	// 扫描结束切换状态
		if (R1.scanbool)	// 扫描结束切换状态
		{
			R1.FindInitialState = Robot::State::Min;
			AfxMessageBox(_T("正在计算"));
		}
		break;
	case Robot::State::Min:
		//CalculateAngle(R1.PosTempKF, R1.anglesKF);
		CalculateAngle(R1.PosTemp, R1.angles);
		R1.isSelfRotate = false;
		R1.FindInitialState = Robot::State::TurnNormal;
		break;
	case Robot::State::TurnNormal:
		if (SelfRotate(R1.SetAngle))
		{
			KillTimer(9);
			R1.FindInitialState = Robot::State::HoriMovePrevFlag;
		}
		break;
	case Robot::State::HoriMovePrevFlag:
	{
		R1.SetHoriDis = CalculateHori();
		CString mm_text;	//	更新控件值
		mm_text.Format(_T("%f"), R1.SetHoriDis);
		GetDlgItem(IDC_EDIT9)->SetWindowText(mm_text);
		R1.isHoriMove = false;// 重置水平移动状态
		R1.FindInitialState = Robot::State::HoriMoveFlag;
		break;
	}
	case Robot::State::HoriMoveFlag:
		if (Hori_Move(R1.SetHoriDis))
			R1.FindInitialState = Robot::State::VertiMovePrevFlag;
		break;
	case Robot::State::VertiMovePrevFlag:
	{
		R1.SetVertiDis = CalculateVerti();
		CString mm_text;	//	更新控件值
		mm_text.Format(_T("%f"), R1.SetVertiDis);
		GetDlgItem(IDC_EDIT10)->SetWindowText(mm_text);
		R1.isVertiMove = false;
		R1.FindInitialState = Robot::State::VertiMoveFlag;
		break;
	}
	case Robot::State::VertiMoveFlag:
		if(Verti_Move(R1.SetVertiDis))
			R1.FindInitialState = Robot::State::End;
		break;
	default:
		KillTimer(9);
		break;
	}
}

bool CPage1Dlg::RobotScan(double SetAngle, bool enableCamera, bool kf)
{
	UpdateData(TRUE);

	if (!R1.isSelfRotate)
	{
		R1.isSelfRotate = true;
		R1.TargetAngleOut = R1.AngleOut + SetAngle;
		//R1.angles.clear();	//	先清空数据，否则影响下一次计算角度
		//R1.PosTemp.clear();

		if (enableCamera)	// 是否录视频可选，默认不录
			camera.OpenFile(R1.AngleOut, R1.TargetAngleOut);

		R1.LinearV[Left] = m_speed; // 左轮速度
		R1.LinearV[Right] = m_speed; // 右轮速度
		R1.V2PSC();	// 应用速度控制										 // 线速度均转换为PSC数据
		if (SetAngle >= 0)
		{
			Port1.MotorCtrl(R1.PSC[0], R1.PSC[1], R1.PSC[2], -R1.PSC[3]);
		}
		else
		{
			Port1.MotorCtrl(R1.PSC[0], R1.PSC[1], -R1.PSC[2], R1.PSC[3]);
		}
		QueryPerformanceCounter(&m_lastTime7);  // 记录启动时刻
	}

	if (enableCamera) camera.SaveFrame();

	//	扫描环节用于计算点坐标
	cv::Mat frame;
	cap >> frame;

	LARGE_INTEGER currentTime;
	QueryPerformanceCounter(&currentTime);
	// 计算精确时间间隔（秒）
	double elapsed = static_cast<double>(currentTime.QuadPart - m_lastTime7.QuadPart) / static_cast<double>(m_frequency.QuadPart);
	m_lastTime7 = currentTime;  // 更新上次触发时间
	if (SetAngle > 0) {
		R1.AngleOut = R1.AngleOut + ((2.0 * m_speed) / Tread)*elapsed / M_PI*180.0;
	}
	else
	{
		R1.AngleOut = R1.AngleOut - ((2.0 * m_speed) / Tread)*elapsed / M_PI*180.0;
	}

	//更新完机器人角度立刻把检测到的坐标值添加到PosTemp。参数Point2d(0, 0)用于设定默认值，预留。
	//R1.PosTemp和R1.angles在CalculateAngle()函数中被清空了
	if (kf)	//	使用卡尔曼滤波
	{
		R1.PosTemp.push_back(task.getIntersectionKF(framedata, frame, Point2d(0, 0))[0]);
		R1.angles.push_back(R1.AngleOut);
	}
	else
	{
		R1.PosTemp.push_back(task.getIntersection(framedata, frame, Point2d(0, 0))[0]);
		R1.angles.push_back(R1.AngleOut);
	}
	char dat[100];
	std::snprintf(dat, sizeof(dat), "Angle:%.2f\nVertDis:%.2f\nHoriDis:%.2f\nX0:%.2f", R1.AngleOut, R1.VertOut, R1.HoriOut, X0);
	m_static1 = dat;
	GetDlgItem(IDC_STATIC1)->SetWindowText(m_static1);
	//UpdateData(FALSE);

	if ((SetAngle >= 0 && R1.AngleOut >= R1.TargetAngleOut) || (SetAngle <= 0 && R1.AngleOut <= R1.TargetAngleOut) || (abs(R1.AngleOut - R1.TargetAngleOut)<0.05))
	{
		KillTimer(7);	// 结束定时器
		R1.ClearAllV();          //清空所有电机线速度
		R1.V2PSC();              //更新速度到PSC
		Port1.MotorCtrl(R1.PSC);  // 所有电机速度为0
		if (enableCamera) {
			camera.CloseFile();
			camera.NewName(R1.AngleOut);
		}
		return true;
	}
	return false;
}

vector<double> x_values, x_values1, x_values2, x_values3;
int c = 0;
//dir为true表示顺时针转
int CPage1Dlg::Scan(bool dir)
{
		if (!R1.isSelfRotate)
		{
			R1.isSelfRotate = true;
	
			R1.LinearV[Left] = m_speed; // 左轮速度
			R1.LinearV[Right] = m_speed; // 右轮速度
			R1.V2PSC();	// 应用速度控制										 // 线速度均转换为PSC数据
			if (!dir)
			{
				Port1.MotorCtrl(R1.PSC[0], R1.PSC[1], R1.PSC[2], -R1.PSC[3]);
			}
			else
			{
				Port1.MotorCtrl(R1.PSC[0], R1.PSC[1], -R1.PSC[2], R1.PSC[3]);
			}
			QueryPerformanceCounter(&m_lastTime7);  // 记录启动时刻
		}
	
		//	扫描环节用于计算点坐标
		cv::Mat frame;
		cap >> frame;
	
		LARGE_INTEGER currentTime;
		QueryPerformanceCounter(&currentTime);
		// 计算精确时间间隔（秒）
		double elapsed = static_cast<double>(currentTime.QuadPart - m_lastTime7.QuadPart) / static_cast<double>(m_frequency.QuadPart);
		m_lastTime7 = currentTime;  // 更新上次触发时间
		if (dir) {
			R1.AngleOut = R1.AngleOut + ((2.0 * m_speed) / Tread)*elapsed / M_PI*180.0;
		}
		else
		{
			R1.AngleOut = R1.AngleOut - ((2.0 * m_speed) / Tread)*elapsed / M_PI*180.0;
		}
	
		//更新完机器人角度立刻把检测到的坐标值添加到PosTemp。参数Point2d(0, 0)用于设定默认值，预留。
		//带KF使用了卡尔曼滤波
		R1.PosTempKF.push_back(task.getIntersectionKF(framedata1, frame, Point2d(0, 0))[0]);
		x_values.push_back(R1.PosTempKF.back().x);	//	使用x值
		R1.anglesKF.push_back(R1.AngleOut);

		R1.PosTemp.push_back(task.getIntersection(framedata, frame, Point2d(0, 0))[0]);
		x_values2.push_back(R1.PosTemp.back().x);
		R1.angles.push_back(R1.AngleOut);

		char dat[100];
		std::snprintf(dat, sizeof(dat), "Angle:%.2f\nVertDis:%.2f\nHoriDis:%.2f\nX0:%d\ttrend:%d", R1.AngleOut, R1.VertOut, R1.HoriOut, R1.PosTemp.size(), R1.trend);
		m_static1 = dat;
		GetDlgItem(IDC_STATIC1)->SetWindowText(m_static1);
		//UpdateData(FALSE);

		if (x_values.size() > 50)
		{
			//x_values.erase(x_values.begin(), x_values.end() - 15);
			int n = x_values.size();
			x_values1.assign(x_values.end() - 20, x_values.end());
			x_values3.assign(x_values2.end() - 20, x_values2.end());
			//c = analyzeTrend(x_values);
			//c = simpleTrendDetection1(x_values1);
			c = simpleTrendDetection(x_values1, x_values2);
			R1.trend = c;
			return c;
		}
		return 0;	//	默认返回0
}

int possize = 0;
int flag = 0;
void CPage1Dlg::RobotScanAdaptive()
{
	switch (R1.scanflag)
	{
	case 0:
		for (int i = 0; i < 10; i++)
		{
			for (int i = 0; i < 8; ++i) cap.grab();   // 只抓不解码，最快
		}
		R1.PosTemp.clear();
		R1.angles.clear();
		R1.PosTempKF.clear();
		R1.anglesKF.clear();
		x_values.clear();
		R1.isSelfRotate = false;
		R1.AngleOut = 0.0;
		possize = 0;
		R1.trend = 0;
		flag = 0;
		R1.scanflag = 1;
		R1.scanbool = false;
		break;
	case 1:
	{
		//初次扫描，判断所处位置，返回1，2，3，4
		//1是单增，2是单减，3是先增后减，4是先减后增
		//函数只识别得到焊缝交点，并返回所处位置标志
		//R1.trend = Scan(false);	//	Scan只负责旋转角度和返回趋势判断
		Scan(false);	//	Scan只负责旋转角度和返回趋势判断
		if (R1.trend != 0 && R1.trend != 5)
		{
			//	先关闭电机，停止旋转
			R1.ClearAllV();          //清空所有电机线速度
			R1.V2PSC();              //更新速度到PSC
			Port1.MotorCtrl(R1.PSC);  // 所有电机速度为0
			R1.isSelfRotate = false;
		}
		if (R1.trend == 2)
		{
			cv::Mat img(300, 300, CV_8UC3, cv::Scalar(255, 255, 255));
			cv::putText(img, "down", cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
			//cv::imshow("2_1", img);
			possize = R1.PosTemp.size();	//	先记录一次点数据量
			R1.scanflag = 2;//	单减
		}
		if (R1.trend == 1)
		{
			cv::Mat img(300, 300, CV_8UC3, cv::Scalar(255, 255, 255));
			cv::putText(img, "up", cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
			//cv::imshow("1_1", img);
			R1.scanflag = 3;//	单增
		}
		if (R1.trend == 3)
		{
			R1.scanflag = 4;//	先增后减
		}
		if (R1.trend == 4)
		{
			cv::Mat img(300, 300, CV_8UC3, cv::Scalar(255, 255, 255));
			cv::putText(img, "down 2 up", cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
			//cv::imshow("3_1", img);
			R1.scanflag = 5;//	先减后增
			possize = R1.PosTemp.size();
		}
		break;
	}
	case 2:
		if (R1.scanflag == 2 && Scan(false) == 4)//第2种情况。先减小->由小变大
		{
			if (R1.PosTemp.size() - possize > 30)
			{
				cv::Mat img(300, 300, CV_8UC3, cv::Scalar(255, 255, 255));
				cv::putText(img, "down->down 2 up", cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
				//cv::imshow("2_2", img);
				//AfxMessageBox(_T("先减小->由小变大（min）"));
				possize = R1.PosTemp.size();
				R1.scanflag = 6;
			}
			else
			{
				cv::Mat img(300, 300, CV_8UC3, cv::Scalar(255, 255, 255));
				cv::putText(img, "down->down 2 up\nneed more points", cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
				//cv::imshow("2_2", img);
				possize = R1.PosTemp.size();
				R1.scanflag = 9;
			}
		}
		break;
	case 6:
		if (R1.scanflag == 6 && Scan(false) == 1)//第2种情况。先减小->由小变大（min）->增大
		{
			if ((R1.PosTemp.size() - possize) > 50)
			{
				cv::Mat img(300, 300, CV_8UC3, cv::Scalar(255, 255, 255));
				cv::putText(img, "down->down 2 up->up", cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
				//cv::imshow("2_3", img);
				//	先关闭电机，停止旋转
				R1.ClearAllV();          //清空所有电机线速度
				R1.V2PSC();              //更新速度到PSC
				Port1.MotorCtrl(R1.PSC);  // 所有电机速度为0
				R1.isSelfRotate = false;
				//AfxMessageBox(_T("先减小->由小变大（min）->增大"));
				R1.scanbool = true;
				R1.scanflag = 20;
			}
		}
		break;
	case 3:	//	在最低点左侧
		R1.PosTemp.clear();
		R1.angles.clear();
		R1.PosTempKF.clear();
		R1.anglesKF.clear();
		x_values.clear();
		R1.isSelfRotate = false;	//执行下一次Scan()时可以调整方向
		R1.AngleOut = 0.0;
		R1.scanflag = 7;
		break;
	case 7:
		if (R1.scanflag == 7 && Scan(true) == 4)	// 第1种情况。先增大->由小变大（min）->增大
		{
			cv::Mat img(300, 300, CV_8UC3, cv::Scalar(255, 255, 255));
			cv::putText(img, "up->down 2 up", cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
			//cv::imshow("1_2",img);
			//AfxMessageBox(_T("先增大->由小变大（min）"));
			possize = R1.PosTemp.size();
			R1.scanflag = 8;
		}
		break;
	case 8:
		if (R1.scanflag == 8 && Scan(true) == 1)
		{
			if ((R1.PosTemp.size() - possize) > 50)
			{
				//	先关闭电机，停止旋转
				R1.ClearAllV();          //清空所有电机线速度
				R1.V2PSC();              //更新速度到PSC
				Port1.MotorCtrl(R1.PSC);  // 所有电机速度为0
				R1.isSelfRotate = false;
				cv::Mat img(300, 300, CV_8UC3, cv::Scalar(255, 255, 255));
				cv::putText(img, "up->down 2 up->up", cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
				//cv::imshow("1_3", img);
				//AfxMessageBox(_T("先增大->由小变大（min）->增大"));
				R1.scanbool = true;
				R1.scanflag = 20;
			}
		}
		break;
	case 5:
		if (R1.scanflag == 5 && Scan(false) == 1)	//	第3种情况，先减后增->单增
		{
			if ((R1.PosTemp.size() - possize) > 70)
			{
				//	先关闭电机，停止旋转
				R1.ClearAllV();          //清空所有电机线速度
				R1.V2PSC();              //更新速度到PSC
				Port1.MotorCtrl(R1.PSC);  // 所有电机速度为0
				R1.PosTemp.clear();
				R1.angles.clear();
				R1.PosTempKF.clear();
				R1.anglesKF.clear();
				x_values.clear();
				R1.AngleOut = 0.0;
				R1.isSelfRotate = false;	//执行下一次Scan()时可以调整方向
				//AfxMessageBox(_T("先减小->由小变大（min）->增大"));
				cv::Mat img(300, 300, CV_8UC3, cv::Scalar(255, 255, 255));
				cv::putText(img, "down 2 up->up", cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
				//cv::imshow("3_2", img);
				R1.scanflag = 7;
			}
		}
		break;
	case 9:
		if (R1.scanflag == 9 && Scan(false) == 1)	// 第1种情况。先增大->由小变大（min）->增大
		{
			if (R1.PosTemp.size() - possize > 100)
			{
				R1.PosTemp.clear();
				R1.angles.clear();
				R1.PosTempKF.clear();
				R1.anglesKF.clear();
				x_values.clear();
				R1.isSelfRotate = false;	//执行下一次Scan()时可以调整方向
				R1.AngleOut = 0.0;
				cv::Mat img(300, 300, CV_8UC3, cv::Scalar(255, 255, 255));
				cv::putText(img, "down->down 2 up\nneed more points\nup", cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
				//cv::imshow("2_3", img);
				//AfxMessageBox(_T("先增大->由小变大（min）"));
				R1.scanflag = 7;
			}
		}
		break;
	case 20:
		double dd = 0;
		break;

	}

	char dat[100];
	std::snprintf(dat, sizeof(dat), "Angle:%.2f\nVertDis:%.2f\nHoriDis:%.2f\nX0:%d\ttrend:%d", R1.AngleOut, R1.VertOut, R1.HoriOut, R1.PosTemp.size(), R1.trend);
	m_static1 = dat;
	GetDlgItem(IDC_STATIC1)->SetWindowText(m_static1);
}

//int possize = 0;
//int trend = 0;
//int flag = 0;
//bool CPage1Dlg::RobotScanAdaptive()
//{
//	switch (R1.scanflag)
//	{
//	case 0:
//		R1.PosTemp.clear();
//		R1.angles.clear();
//		R1.PosTempKF.clear();
//		R1.anglesKF.clear();
//		R1.isSelfRotate = false;
//		R1.AngleOut = 0.0;
//		possize = 0;
//		trend = 0;
//		R1.scanflag = 1;
//		flag = 0;
//		break;
//	case 1:
//	{
//		//初次扫描，判断所处位置，返回1，2，3，4
//		//1是单增，2是单减，3是先增后减，4是先减后增
//		//函数只识别得到焊缝交点，并返回所处位置标志
//		trend = Scan(false);	//	Scan只负责旋转角度和返回趋势判断
//		if (trend == 1)
//			R1.scanflag = 2;	//	单增
//		if (trend == 2)
//			R1.scanflag = 3;	//	单减
//		if (trend == 3)
//			R1.scanflag = 4;	//	先减后增
//		if (trend == 4)
//			R1.scanflag = 5;	//	先增后减
//		if (trend != 0 && trend != 5)
//		{
//			//	先关闭电机，停止旋转
//			R1.ClearAllV();          //清空所有电机线速度
//			R1.V2PSC();              //更新速度到PSC
//			Port1.MotorCtrl(R1.PSC);  // 所有电机速度为0
//			R1.isSelfRotate = false;
//		}
//		break;
//	}
//	case 2:
//	{	//如果一开始就是增大，说明在最低点左侧。则顺时针转动
//		if (Scan(false) == 1)
//		{
//			if (R1.PosTemp.size() > 50)	//	这里确保在最低点左侧有50个采样点
//			{
//				R1.AngleOut = 0.0;
//				R1.PosTemp.clear();
//				R1.angles.clear();
//				R1.PosTempKF.clear();
//				R1.anglesKF.clear();
//				R1.isSelfRotate = false;
//				R1.scanflag = 7;
//			}
//		}
//		break;
//	}
//	case 3:
//	{
//		if (Scan(false) == 4)	//减小->先减后增。等待先减后增出现
//		{
//			if (R1.PosTemp.size() >= 40)
//			{
//				AfxMessageBox(_T("在右侧且点数大于40"));
//				possize = R1.PosTemp.size();	//	记录当前点个数，后续再检测50个点
//				R1.scanflag = 6;
//			}
//			else   //当左侧检测到的点小于40个
//			{
//				R1.isSelfRotate = false;
//				AfxMessageBox(_T("在左侧且点数小于40"));
//				R1.scanflag = 4;	// 回转5°
//			}
//		}
//		break;
//	}
//	case 4:
//	{
//		if (SelfRotate(5))	//	先减后增（说明就在最低点附近）->则顺时针转5°
//		{
//			R1.AngleOut = 0.0;
//			R1.PosTemp.clear();
//			R1.angles.clear();
//			R1.PosTempKF.clear();
//			R1.anglesKF.clear();
//			R1.isSelfRotate = false;
//			R1.scanflag = 3;
//		}
//		break;
//	}
//	case 5:
//		break;
//	case 6:
//	{
//		if (Scan(false) == 1)	//	如果x值单增且检测到最低点后再检测到50个
//		{
//			if ((R1.PosTemp.size() - possize) > 50)
//			{
//				AfxMessageBox(_T("右侧到最低点再到左侧50点"));
//				//	先关闭电机，停止旋转
//				R1.ClearAllV();          //清空所有电机线速度
//				R1.V2PSC();              //更新速度到PSC
//				Port1.MotorCtrl(R1.PSC);  // 所有电机速度为0
//				R1.isSelfRotate = false;
//				flag = 1;
//				//R1.scanflag = 9;
//			}
//		}
//		break;
//	}
//	case 7:
//	{
//		//如果一开始就是增大，说明在最低点左侧。则顺时针转动
//		if (Scan(true) == 4)	//增大->先减后增。等待先减后增出现
//		{
//			possize = R1.PosTemp.size();	//	记录当前点个数，后续再检测50个点
//			R1.scanflag = 8;
//		}
//		break;
//	}
//	case 8:
//	{
//		if (Scan(true) == 1)	//	如果x值单增且检测到最低点后再检测到50个
//		{
//			if ((R1.PosTemp.size() - possize) > 50)
//			{
//				//	先关闭电机，停止旋转
//				R1.ClearAllV();          //清空所有电机线速度
//				R1.V2PSC();              //更新速度到PSC
//				Port1.MotorCtrl(R1.PSC);  // 所有电机速度为0
//				R1.isSelfRotate = false;
//				flag = 1;
//				/*R1.scanflag = 9;*/
//			}
//		}
//		break;
//	}
//	case 9:
//	{
//
//		break;
//	}
//	default:
//		break;
//	}
//
//	if (flag == 1)
//	{
//		AfxMessageBox(_T("即将回转"));
//		return true;
//	}
//	else
//		return false;
//}

void CPage1Dlg::CalculateAngle(vector<Point2d>& PosTemp, vector<double>& angles)
{
	std::vector<coeffs> p = ransacQuadraticFit(PosTemp, 1000, 10.0, 0.5);
	plotSequence(PosTemp, p.front(), 'x');
	double x_index = round(p[0].index);
	double y_index = round(p[1].index);
	int index = static_cast<int>(round((x_index + y_index) / 2.0));
	if (angles.back() < 0)
	{
		R1.SetAngle = angles.back() - angles.back()*(p[0].index / angles.size()) + 8.5;
	}
	if (angles.back() > 0)
	{
		R1.SetAngle = abs(angles.back() - angles.back()*(p[0].index / angles.size())) + 8.5;
		//R1.SetAngle = angles.back()*(p[0].index / angles.size()) - angles.back() + 8.0;
	}
	angles.clear();	//	清空数据，不要影响下一次计算角度
	PosTemp.clear();
}

double CPage1Dlg::CalculateHori()
{
	UpdateData(TRUE);
	if (!cap.isOpened()) return 0;
	cv::Mat frame;
	const int N = 8;	// 丢弃前 N 帧，保留最新一帧
	vector<double> sumx, sumx1;
	for (int i = 0; i < 10; i++)
	{
		for (int i = 0; i < N; ++i) cap.grab();   // 只抓不解码，最快
		if (cap.retrieve(frame) && !frame.empty())
		{
			double x_temp = (task.getIntersection(framedata, frame, Point2d(0, 0)))[0].x;
			sumx.push_back(x_temp);
		}
		else {
			i--;
		}
	}
	sumx1 = removeOutliersIQRSmallSample(sumx);
	double x1 = accumulate(sumx1.begin(), sumx1.end(), 0.0) / sumx1.size();
	double mm1 = R1.Hori_a*x1 + R1.Hori_b;
	return mm1;
}

//竖直初始
void CPage1Dlg::OnBnClickedButton16()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);		// 从控件读取数据
	vertitemp = CalculateVerti();
	R1.isVertiMove = false;	// 重置水平移动状态
	SetTimer(11, 1, nullptr);
	CString mm_text;		//	更新控件值
	mm_text.Format(_T("%f"), vertitemp);
	GetDlgItem(IDC_EDIT10)->SetWindowText(mm_text);
	//UpdateData(FALSE);
}

double CPage1Dlg::CalculateVerti()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);	// 从控件读取数据
	if (!cap.isOpened()) return 0;
	cv::Mat frame;
	const int N = 8;	// 丢弃前 N 帧，保留最新一帧
	vector<double> sumy, sumy1;
	for (int i = 0; i < 10; i++)
	{
		for (int i = 0; i < N; ++i) cap.grab();   // 只抓不解码，最快
		if (cap.retrieve(frame) && !frame.empty())
		{
			double y_temp = (task.getIntersection(framedata, frame, Point2d(0, 0)))[0].y;
			sumy.push_back(y_temp);
		}
		else {
			i--;
		}
	}
	sumy1 = removeOutliersIQRSmallSample(sumy);
	double y1 = accumulate(sumy1.begin(), sumy1.end(), 0.0) / sumy1.size();
	double mm1 = R1.Verti_a *y1 + R1.Verti_b;
	return -mm1;
}

//水平初始
void CPage1Dlg::OnBnClickedButton17()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);
	horitemp = CalculateHori();
	R1.isHoriMove = false;		// 重置水平移动状态
	SetTimer(10, 1, nullptr);
	CString mm_text;			//	更新控件值
	mm_text.Format(_T("%f"), horitemp);
	GetDlgItem(IDC_EDIT9)->SetWindowText(mm_text);
	//UpdateData(FALSE);
}

void CPage1Dlg::OnBnClickedButton19()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);	// 从控件读取数据

	R1.PosTemp.clear();
	R1.angles.clear();
	R1.FindInitialState = Robot::State::ScanTurnFlag;
	SetTimer(12, 200, nullptr);
}

void CPage1Dlg::FindAngle()
{
	UpdateData(TRUE);
	switch (R1.FindInitialState)
	{
	case Robot::State::ScanTurnFlag:
		R1.isSelfRotate = false;
		R1.AngleOut = 0.0;
		R1.FindInitialState = Robot::State::Scan;
		break;
	case Robot::State::Scan:
		if (RobotScan(-20.0))	// 扫描结束切换状态
			R1.FindInitialState = Robot::State::Min;
		break;
	case Robot::State::Min:
		CalculateAngle(R1.PosTemp, R1.angles);
		R1.isSelfRotate = false;
		R1.FindInitialState = Robot::State::TurnNormal;
		break;
	case Robot::State::TurnNormal:
		if (SelfRotate(R1.SetAngle))
			KillTimer(12);
		//R1.FindInitialState = Robot::State::HoriMovePrevFlag;
		break;
	default:
		KillTimer(12);
	}
}

// 发脉冲旋转
void CPage1Dlg::OnBnClickedButton20()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);
	double mm = (selfrotateangle * M_PI * Tread) / 360.0; //将角度转为弧长

	unsigned short VertSpeed = 0, HorSpeed = 0, LeftSpeed = 0, RightSpeed = 0;
	//左轮//右轮
	if (mm < 0)
	{
		LeftSpeed = -R1.W2PSC(1.0);
		RightSpeed = R1.W2PSC(1.0);
	}
	else
	{
		LeftSpeed = R1.W2PSC(1.0);
		RightSpeed = -R1.W2PSC(1.0);
	}
	Port1.MotorCtrl(VertSpeed, HorSpeed, LeftSpeed, RightSpeed);
	R1.AngleOut = R1.AngleOut + selfrotateangle;
	char dat[100];
	std::snprintf(dat, sizeof(dat), "Angle:%.2f\nVertDis:%.2f\nHoriDis:%.2f\nX0:%.2f", R1.AngleOut, R1.VertOut, R1.HoriOut, X0);
	m_static1 = dat;
	GetDlgItem(IDC_STATIC1)->SetWindowText(m_static1);
	UpdateData(FALSE);
}




void CPage1Dlg::OnEnChangeEdit2()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
}


void CPage1Dlg::OnEnChangeEdit7()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
}


void CPage1Dlg::OnStnClickedStatic1()
{
	// TODO: 在此添加控件通知处理程序代码
}

// 将图片控件固定为 1280x720 像素，并始终放置在页面客户区的正中央
void CPage1Dlg::OnSize(UINT nType, int cx, int cy)
{
	//CDialogEx::OnSize(nType, cx, cy);


}
