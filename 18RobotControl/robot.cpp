#include "StdAfx.h" 
#include "Robot.h"
#include <fstream>  // 需要包含文件流头文件
#include <sstream>  // 需要包含文件流头文件
#include <cmath>   // C++风格的数学库
Robot::Robot() {
	Init();  // 构造函数中调用初始化
}
// 清空所有电机线速度
void Robot::ClearAllV() {
	LinearV[Vert] = 0.0;//线速度，线速度=角速度*导程，角速度=线速度/导程
	LinearV[Hori] = 0.0;
	LinearV[Left] = 0.0;
	LinearV[Right] = 0.0;
}
void Robot::Init() {
	// 滑块导程
	Lead[Vert] = 2.0;// 竖直滑块导程2mm/r
	Lead[Hori] = 5.0;// 水平滑块导程5mm/r
	Lead[Right] = 3.14159265* 8.0 / 9.0;// 轮子导程pi*8/9mm/r
	Lead[Left] = Lead[Right];// 轮子导程pi*8/9mm/r，方向是反的，在下位机进行了矫正

	// 末端执行器状态初始化
	Elong = 0.0;          // 初始伸长量为0mm，干伸长
	ElongErr = 0.0;       // 初始无偏差，干伸长偏差
	HoriAngleErr = 0.0;   // 初始无角度偏差
	DistanceErr = 0.0;    // 初始无距离偏差
	// 水平滑块初始化
	//HoriOut = 0;        // 初始伸长0mm
	HoriOut = 240.0;        // 初始伸长0mm
	TargetHoriOut = 0.0;
	HoriMax = 142.5;      // 水平滑块行程142.5mm
	// 竖直滑块初始化
	//VertOut = 0;        // 初始完全收回
	VertOut = 135.0;        // 初始完全收回
	TargetVertOut = 0.0;
	VertMax = 73.9;        // 竖直滑块行程73.9mm
	// 水平运动状态参数初始化
	HoriAngle = 0.0;       // 水平面角度初始为0rad
	s0=190.0;   // 初始焊接点到旋转中心的距离
	a0=229.5;   // 初始焊接点到左轮中心距离，45度，标注加10度，对应伸出距轮边缘215，轮子板宽14.5mm  
	b0=210.0;   // 双轮中心距210mm
	WeldSpeed = 5.0;// 焊接速度4mm/s
	t=0.250;    // 采样时间250ms
	UsDist =-1.0; // 超声波距离
	//s01=32.9; // 需转弯距离
	s01 = 50.0; // 需转弯距离
	si = s0;  // 斜边累积长度=s0+sum

	TargetAngleOut = 0.0;
	AngleOut = 0.0;
	TargetVertOutInitial = 35.0;  // 目标位置初始为0
	TargetHoriOutInitial = 30.0; // 目标位置初始为30mm
	SetAngle = 0.0;
	SetHoriDis = 0.0;
	SetVertiDis = 0.0;

	isHoriMove = false;
	isVertiMove = false;
	isSelfRotate = false;
	FindInitialState = Scan;	// 将FindInitialState枚举变量赋值为Scan
	HoriMoveCpFlag = false;
	VertiMoveCpFlag = false;
	califlag = 0;
	Hcaliflag = 0;
	Vcaliflag = 0;
	calicnt = 0;
	scanflag = 0;
	scanbool = false;
	trend = 0;
}
// 输入单位rps,电机旋转角速度转为单片机PSC信号
short Robot::W2PSC(double SetSpeed) {
	short PSC = 0;
	if (SetSpeed) {
		PSC = static_cast<short>(round(625.0 / SetSpeed));//rps
		//PSC = static_cast<int>(round(37500 / SetSpeed));//rpmin
	}
	return PSC;
}
// 全部线速度均转换为PSC数据
void Robot::V2PSC() {
	for (char i = 0; i < 4; i++) {
		PSC[i]=W2PSC(LinearV[i] / Lead[i]);
	}
}
// 转换指定的线速度为PSC数据,每次发送数据均需要更新数据
void Robot::V2PSC(MotorDirection i) {
	PSC[i] = W2PSC(LinearV[i] / Lead[i]);
}
// 写入日志
void CtrlLogWrite(char dat[], unsigned int num) {//log
	std::ofstream outputFile("CtrlLog.txt", std::ios::app);
	if (!outputFile.is_open()) {  // 检查文件是否成功打开
		return;  // 返回错误代码，表示打开文件失败
	}
	outputFile.write(dat, num);
	outputFile.close();  // 确保文件正常关闭
}
// 每个采样周期更新数据
void Robot::UpdataRobot() {
	if (UsDist > 0 && s01 > UsDist) {//超声波有数据且小于需转弯距离
		TurnFlag = true;
	}
	if(TurnFlag) {//需要转弯
		double deltaS = 0.0;
		double w = 0.0;
		if (HoriAngle < 0.785398) {
			LinearV[Hori] = WeldSpeed*sin(HoriAngle);
			deltaS = LinearV[Hori] * t;//斜边增量
			si = si + deltaS;//更新斜边长度
			w = WeldSpeed*cos(HoriAngle) / si;//旋转角速度
		}
		else{
			LinearV[Hori] = -WeldSpeed*cos(HoriAngle);
			deltaS = LinearV[Hori] * t;//斜边增量
			si = si + deltaS;//更新斜边长度
			w = WeldSpeed*sin(HoriAngle) / si;//旋转角速度
		}
		LinearV[Left] = w*(s0 - a0);		 //左轮速度
		LinearV[Right] = w*(s0 - b0 - a0); //右轮速度
		HoriAngle = HoriAngle + w*t;     //角度累积
		HoriOut = HoriOut + deltaS;      //更新水平滑块伸长长度
		if (HoriAngle > 1.5708) TurnFlag = false;
	}
	else {
		LinearV[Vert] = 0.0;//线速度，线速度=角速度*导程，角速度=线速度/导程
		LinearV[Hori] = 0.0;
		LinearV[Left] = WeldSpeed;
		LinearV[Right] = WeldSpeed;
	}
	V2PSC();         //更新所有速度
	char dat[100];
	unsigned int num;
	num = snprintf(dat, sizeof(dat), "L:%.2f, R:%.2f,HV:%.2f,H:%.2f,HA:%.2f,Us:%.2f\n"
		, LinearV[Left], LinearV[Right], LinearV[Hori], HoriOut, HoriAngle, UsDist);
	CtrlLogWrite(dat, num);
}
// 输入超声波模拟量转换为超声波距离
void Robot::Analog2Distance(double Analog) {
	if (Analog < 2.1|| Analog > 9) {
		UsDist = -1.0;
		return;
	}
	UsDist = 12.09616*Analog-0.2518;//重新拟合的曲线
}
// 提取限位开关状态，提取LimitSwitch的低5位，并按从高位到低位的方式赋值给Position[0]到Position[4]
void Robot::Switch2Position(unsigned char Switch) {
	for (int i = 0; i < 5; ++i) {
		Position[i] = (Switch >> (4 - i)) & 1;  // 获取LimitSwitch的相应位
	}
}
// 更新水平滑块运动状态
bool Robot::updata_hori_status() {
	HoriOut += LinearV[Hori]*t;
	//char dat[100];
	//unsigned int num;
	//num = snprintf(dat, sizeof(dat), "H: %.2f, TH: %.2f,V: %.2f,Time:%.2f\n"
	//	, HoriOut, TargetHoriOut, LinearV[Hori],t);
	//CtrlLogWrite(dat,num);
	if (HoriOut>0&&//判断第二次回缩
		(HoriOut > TargetHoriOutInitial || std::abs(HoriOut - TargetHoriOutInitial) < 0.05)) {//达到目标伸长距离
		return true;
	}
	return false;
}

// 更新垂直滑块运动状态
bool Robot::updata_verti_status() {
	VertOut += -(LinearV[Vert] * t);
	//char dat[100];
	//unsigned int num;
	//num = snprintf(dat, sizeof(dat), "H: %.2f, TH: %.2f,V: %.2f,Time:%.2f\n"
	//	, HoriOut, TargetHoriOut, LinearV[Hori],t);
	//CtrlLogWrite(dat,num);
	if (VertOut>0 &&//判断第二次回缩
		(VertOut > TargetVertOutInitial || std::abs(VertOut - TargetVertOutInitial) < 0.05)) {//达到目标伸长距离
		return true;
	}
	return false;
}

FileUtils::LogDataStruct Robot::GetData()
{
	FileUtils::LogDataStruct temp;
	temp.SelfAngle = AngleOut;
	temp.Hori = HoriOut;
	temp.Verti = VertOut;
	temp.H_Speed = LinearV[1];
	temp.V_Speed = LinearV[0];
	temp.L_Speed = LinearV[2];
	temp.R_Speed = LinearV[3];
	return temp;
}

float Robot::distancetocircle(float x_mm, double Lead)
{

	if (x_mm <= 0)
	{
		return 0;
	}
	if (Lead <= 0) { // 防错：导程不能为0或负数
		return 0;
	}
	return x_mm / Lead;
}

//bool Robot::Hori_Move(double distance)
//{
//	if (!isHoriMove)
//	{
//		isHoriMove = true;
//		TargetHoriOut = HoriOut + distance;
//
//		camera.OpenFile(HoriOut, TargetHoriOut);
//
//		if (distance >= 0) {
//			LinearV[Hori] = 1.0; // 水平滑块速度
//		}
//		else {
//			LinearV[Hori] = -1.0; // 水平滑块速度
//		}
//		V2PSC();// 线速度均转换为PSC数据
//		Port1.MotorCtrl(PSC[0], PSC[1], PSC[2], PSC[3]);// 应用速度控制
//		QueryPerformanceCounter(&m_lastTime13);  // 记录启动时刻
//	}
//
//	camera.SaveFrame();
//
//	LARGE_INTEGER currentTime;
//	QueryPerformanceCounter(&currentTime);
//	// 计算精确时间间隔（秒）
//	double elapsed = static_cast<double>(currentTime.QuadPart - m_lastTime13.QuadPart) / static_cast<double>(m_frequency.QuadPart);
//	m_lastTime13 = currentTime;  // 更新上次触发时间
//	HoriOut = HoriOut + elapsed*LinearV[Hori];
//	if ((distance >= 0 && HoriOut >= TargetHoriOut) || (distance <= 0 && HoriOut <= TargetHoriOut))
//	{
//		camera.CloseFile();
//		ClearAllV();          //清空所有电机线速度
//		V2PSC();              //更新速度到PSC
//		Port1.MotorCtrl(PSC);  // 所有电机速度为0
//		KillTimer(10);
//	}
//	char dat[100];
//	std::snprintf(dat, sizeof(dat), "(angle:%.2f\tH1:%.2f\tL2:%.2f\tX0:%.2f\n", Angle0, H1dis, L2dis, X0);
//	m_static1 = dat;
//	UpdateData(FALSE);
//}
