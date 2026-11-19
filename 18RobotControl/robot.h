#pragma once
#ifndef ROBOT_H
#define ROBOT_H

#include "camera.h"
#include "file_utils.h"
#include "SerialPort.h"

// 电机：Vert-竖直;Hori-水平;Left-左轮;Right-右轮
enum MotorDirection {
	Vert = 0,//竖直
	Hori = 1,//水平
	Left = 2,//左轮
	Right = 3//右轮
};
//限位开关位置
enum Limit {
	Back,   //后
	Mid,    //中
	Front,  //前
	Up,     //上
	Down    //下
};
class Robot {
public:
	Robot();
	// 清空所有电机线速度
	void Robot::ClearAllV();
	// 旋转的角速度rps转换为单片机接收的PSC信号
	short W2PSC(double SetSpeed);
	// 全部线速度转换为PSC
	void V2PSC();
	// 指定线速度转换为PSC
	void V2PSC(MotorDirection i);
	// 每个采样周期更新数据
	void UpdataRobot();
	// 计算超声波距离UsDist,输入超声波模拟量转换为超声波距离
	void Analog2Distance(double Analog);
	// 提取限位开关状态
	void Switch2Position(unsigned char Switch);
	// 更新水平滑块运动状态
	bool updata_hori_status();
	bool updata_verti_status();

	float distancetocircle(float x_mm, double Lead);
	// 获取机器人数据
	FileUtils::LogDataStruct GetData();
public:
	double LinearV[4];  //4个滑块的线速度：竖直，水平，左轮，右轮，右轮速度是反的，在导程处进行了矫正
	short PSC[4];       //单片机接收的PSC数据
	//double AngleJ5;        // 第五关节旋转角度，默认45度
	// 限位开关状态
	bool Position[5] = { false, false, false, false, false };//开关：（后，中，前，上，下）
	double t;       // 采样时间
private:// 私有辅助函数
	// 初始化函数
	void Init();
public:
	// 运动控制参数
	double Lead[4];     //4个滑块的导程：竖直，水平，左轮，右轮
	double AngularW[4]; //4个滑块电机的角速度：竖直，水平，左轮，右轮

	// 末端执行器状态
	double Elong;          // 干伸长
	double ElongErr;       // 干伸长偏差
	double HoriAngleErr;   // 水平面角度偏差
	double DistanceErr;    // 距轨迹距离偏差
	// 水平滑块状态
	double HoriOut;        // 当前伸长距离
	double TargetHoriOut;  // 目标伸长距离
	double TargetHoriOutInitial;	// 初始位置距离
	double HoriMax;		   // 最大伸长距离
	// 竖直滑块状态
	double VertOut;        // 当前伸长距离
	double TargetVertOut;  // 目标伸长距离
	double TargetVertOutInitial;	// 初始位置距离
	double VertMax;		   // 最大伸长距离
    // 水平运动状态
	double HoriAngle;      // 水平面角度
	double Ridus;          // 旋转半径
	double s0;   // 初始焊接点到旋转中心的距离
	double a0;   // 初始焊接点到左轮中心距离  
	double b0;   // 双轮中心距
	double WeldSpeed;// 焊接速度
	
	double UsDist; // 超声波距离
	double s01;  // 需转弯距离

	double s02;  // 机器人前端距离
	double si;   // 转弯后斜边累积长度s0+sum(vtsin(HoriAngle))

public:	// 以下是后来添加的
	double AngleOut;		//  机器人当前角度
	double TargetAngleOut;	//	机器人目标角度
	double SetAngle;
	double SetHoriDis;
	double SetVertiDis;
	vector<Point2d> PosTemp, PosTempKF;		//	存储检测到的坐标点
	vector<double> angles, anglesKF;		//	坐标点对应的角度
	vector<double> HoriPoints, HoriDis, VertiPoints, VertiDis;
	double Hori_a;	//	水平拟合系数 y=ax+b，其中x是点的x坐标，y是距离
	double Hori_b;
	double Verti_a;
	double Verti_b;	//	竖直拟合系数 y=ax+b，其中x是点的y坐标，y是距离
	enum State
	{ 
		ScanTurnFlag,
		Scan,
		MinTurnFlag,
		Min,
		TurnNormalTurnFlag,
		TurnNormal,
		HoriMovePrevFlag,
		HoriMoveFlag,
		VertiMovePrevFlag,
		VertiMoveFlag,
		End
	};
	State FindInitialState;	//	定义一个名为FindInitialState的枚举变量

	// 标志位
	//Camera camera;
	//CSerialPort Port1;//串口对象
	//LARGE_INTEGER m_lastTime13;
	bool TurnFlag=false;//转弯标志位
	bool isHoriMove;
	bool isVertiMove;
	bool isSelfRotate;
	bool HoriMoveCpFlag;
	bool VertiMoveCpFlag;
	int califlag;
	int Hcaliflag;
	int Vcaliflag;
	int calicnt;
	int scanflag;
	bool scanbool;
	int trend;
	

	//新增部分
public:
    // 添加旋转控制变量
    double rotationRadius;      // 旋转半径
    double rotationAngle;        // 当前旋转角度(度)
    double targetRotationAngle;  // 目标旋转角度(度)
    bool isRotating;            // 旋转状态标志
    bool clockwise;             // 旋转方向(顺时针/逆时针)	
	double distance;			// 直线行走前进或者后退的距离
}; 
#endif // ROBOT_H
