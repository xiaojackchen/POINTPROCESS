//////////////////////////////////////////////////////////////////////////  
/// COPYRIGHT NOTICE  
/// Copyright (c) 2009, 华中科技大学tickTick Group  （版权声明）2025，南昌大学  
/// All rights reserved.    
/// @file    SerialPort.cpp    
/// @brief   串口通信类的实现文件   
//  @version 2.0     
//  @author  卢俊(1.0),李发园（2.0）    
/// @E-mail：lujun.hust@gmail.com（卢俊）  
//  @date    2010/03/19   ；2025/02/10
//  2.0修订说明：修改了两个串口接收线程函数，其中一个使用事件接收，修改了串口初始化函数
//  根据机器人增加了采样函数、电机控制函数、数据处理函数、ADC处理函数、日志写入函数
//////////////////////////////////////////////////////////////////////////  
#include "StdAfx.h"  
#include <tchar.h>
#include <process.h>  
#include <iostream>  
#include "SerialPort.h"
#include "string.h"
#include <fstream>  // 需要包含文件流头文件
#include <vector>
#include <sstream>
using namespace std;
//#include<windows.h>
//double GetRate(double MaxInput,double R_Rate) {//输入传感器最大输入与电阻比值
//	return MaxInput/R_Rate;
//}
double ADCToAnalog(unsigned short adcValue,double Rate) {
	double V_ref= 3.3;//参考电压
	// 将12位ADC数值转换为模拟量
	double analogValue = (double)Rate*(adcValue * V_ref) / 4096.0;
	return analogValue;
}
char process_data(const char *data, unsigned short *arr1,
	unsigned short *arr2, unsigned short *val16, unsigned char *val8) {
	// 使用 std::string 替代手动内存分配，简化操作
	std::string copy(data);
	int nums[600];
	int count = 0;
	char *context = nullptr; // 用于 strtok_s 的上下文
	char *token = strtok_s(&copy[0], ",", &context);  // 使用可修改的字符数组
	while (token != NULL) {
		if (strlen(token) == 0) {
			token = strtok_s(NULL, ",", &context);
			continue;
		}
		char *endptr;
		long num = strtol(token, &endptr, 10);
		if (*endptr != '\0') {
			return -2; // 非法字符
		}
		nums[count++] = (int)num;
		token = strtok_s(NULL, ",", &context);
	}
	// 处理数据格式
	if (count == 513 && nums[0] == 2) {
		if (!arr1 || !arr2) return -3;
		for (int i = 0; i < 256; i++) {
			arr1[i] = (unsigned short)nums[i + 1];
			arr2[i] = (unsigned short)nums[i + 257];
		}
		return 0;
	}
	if (count == 3 && nums[0] == 1) {
		if (!val16 || !val8) return -4;
		*val16 = (unsigned short)nums[1];
		*val8 = (unsigned char)nums[2];
		return 1;
	}
	return -5; // 无效数据格式
}
bool CSerialPort::s_bExit = false;/** 线程退出标志 */
/** 当串口无数据时,sleep至下次查询间隔的时间,单位:毫秒 */
const UINT SLEEP_TIME_INTERVAL = 20;

CSerialPort::CSerialPort(void)//构造函数 
	: m_hListenThread(INVALID_HANDLE_VALUE){
	m_hComm = INVALID_HANDLE_VALUE;
	m_hListenThread = INVALID_HANDLE_VALUE;
	InitializeCriticalSection(&m_csCommunicationSync);
	CSerialPort::c_Recvd = false;
	s_bExit = false;/** 线程退出标志 */
}
CSerialPort::~CSerialPort(void){
	CloseListenTread();
	ClosePort();
	DeleteCriticalSection(&m_csCommunicationSync);
}
bool CSerialPort::InitPort(UINT portNo /*= 1*/, UINT baud /*= CBR_9600*/, char parity /*= 'N'*/,
	UINT databits /*= 8*/, UINT stopsbits /*= 1*/, DWORD dwCommEvents /*= EV_RXCHAR*/){
	/** 临时变量,将制定参数转化为字符串形式,以构造DCB结构 */
	char szDCBparam[50];
	sprintf_s(szDCBparam, "baud=%d parity=%c data=%d stop=%d", baud, parity, databits, stopsbits);
	/** 打开指定串口,该函数内部已经有临界区保护,上面请不要加保护 */
	if (!openPort(portNo))	//打开串口失败 
		return false;
	/** 进入临界段 */
	EnterCriticalSection(&m_csCommunicationSync);
	/** 是否有错误发生 */
	BOOL bIsSuccess = TRUE;
	/** 在此可以设置输入输出的缓冲区大小,如果不设置,则系统会设置默认值.
	*  自己设置缓冲区大小时,要注意设置稍大一些,避免缓冲区溢出
	*/
	if (bIsSuccess )
		bIsSuccess = SetupComm(m_hComm,2048,512);//输入2048B，输出512B
	/** 设置串口的超时时间,均设为0,表示不使用超时限制 */
	COMMTIMEOUTS  CommTimeouts;
	CommTimeouts.ReadIntervalTimeout = 0;       //串口接收到一个字节后，系统立即返回，
	//而不再等待后续字节。它适用于读取单个字节或者数据流中的每个字节都快速处理的场景。
	//CommTimeouts.ReadIntervalTimeout = MAXDWORD;//设置一个极长的时间间隔，
												//意味着在接收到第一个字节后，系统会继续等待下一个字节的到来，直到有数据可用，不会因等待超时而中断。
	CommTimeouts.ReadTotalTimeoutMultiplier = 0;
	CommTimeouts.ReadTotalTimeoutConstant = 0;
	CommTimeouts.WriteTotalTimeoutMultiplier = 0;
	CommTimeouts.WriteTotalTimeoutConstant = 0;
	if (bIsSuccess)
		bIsSuccess = SetCommTimeouts(m_hComm, &CommTimeouts);
	DCB  dcb;
	if (bIsSuccess){
		// 将ANSI字符串转换为UNICODE字符串  
	/*	DWORD dwNum = MultiByteToWideChar(CP_ACP, 0, szDCBparam, -1, NULL, 0);
		wchar_t *pwText = new wchar_t[dwNum];
		if (!MultiByteToWideChar(CP_ACP, 0, szDCBparam, -1, pwText, dwNum))
		{
			bIsSuccess = TRUE;
		}
	 */  
    GetCommState(m_hComm,&dcb);
    dcb.BaudRate = baud;
    dcb.ByteSize = 8;
    dcb.Parity =NOPARITY ;
    dcb.StopBits = 0;
	bIsSuccess=SetCommState(m_hComm,&dcb);
		/** 获取当前串口配置参数,并且构造串口DCB参数 */
//		bIsSuccess = GetCommState(m_hComm, &dcb) && BuildCommDCB((LPCSTR)pwText, &dcb);
		/** 开启RTS flow控制 */
		dcb.fRtsControl = RTS_CONTROL_ENABLE;
		/** 释放内存空间 */
//		delete[] pwText;
	}
	if (bIsSuccess){
		/** 使用DCB参数配置串口状态 */
		bIsSuccess = SetCommState(m_hComm, &dcb);
	}
	/**  清空串口缓冲区 */
	PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);
	/** 离开临界段 */
	LeaveCriticalSection(&m_csCommunicationSync);
	return bIsSuccess == TRUE;
}

bool CSerialPort::InitPort(UINT portNo, const LPDCB& plDCB){
	/** 打开指定串口,该函数内部已经有临界区保护,上面请不要加保护 */
	if (!openPort(portNo))
		return false;
	/** 进入临界段 */
	EnterCriticalSection(&m_csCommunicationSync);
	/** 配置串口参数 */
	if (!SetCommState(m_hComm, plDCB))
		return false;
	/**  清空串口缓冲区 */
	PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);
	/** 离开临界段 */
	LeaveCriticalSection(&m_csCommunicationSync);
	return true;
}

void CSerialPort::ClosePort(){
	/** 如果有串口被打开，关闭它 */
	if (m_hComm != INVALID_HANDLE_VALUE){
		CloseHandle(m_hComm);
		m_hComm = INVALID_HANDLE_VALUE;
	}
}
bool CSerialPort::openPort(UINT portNo){
	/** 进入临界段 */
	EnterCriticalSection(&m_csCommunicationSync);
	/** 如果已经打开串口，释放资源并返回 */
	if (m_hComm != INVALID_HANDLE_VALUE) {
		LeaveCriticalSection(&m_csCommunicationSync);
		return false;
	}
	/** 把串口的编号转换为设备名 */
	TCHAR szPort[50];
	_stprintf_s(szPort, _T("COM%d"), portNo);
	/** 打开指定的串口 */
	m_hComm = CreateFile(szPort,      /** 设备名,COM1,COM2等 */
		GENERIC_READ | GENERIC_WRITE, /** 访问模式,可同时读写 */
		0,                            /** 共享模式,0表示不共享 */
		NULL,                         /** 安全性设置,一般使用NULL */
		OPEN_EXISTING,                /** 该参数表示设备必须存在,否则创建失败 */
		FILE_ATTRIBUTE_NORMAL,
		NULL);
	/** 如果打开失败，释放资源并返回 */
	if (m_hComm == INVALID_HANDLE_VALUE){
		LeaveCriticalSection(&m_csCommunicationSync);
		return false;
	}
	/** 退出临界区 */
	LeaveCriticalSection(&m_csCommunicationSync);
	return true;
}

bool CSerialPort::OpenListenThread(){
	/** 检测线程是否已经开启了 */
	if (m_hListenThread != INVALID_HANDLE_VALUE)/** 线程已经开启 */
		return false;
	s_bExit = false;	
	UINT threadId;/** 线程ID */
	/** 开启串口数据监听线程 */
	m_hListenThread = (HANDLE)_beginthreadex(NULL, 0, ListenThread, this, 0, &threadId);
	if (!m_hListenThread)
		return false;
	/** 设置线程的优先级,普通线程 */
	if (!SetThreadPriority(m_hListenThread, THREAD_PRIORITY_NORMAL))
		return false;
	return true;
}
bool CSerialPort::CloseListenTread(){
	if (m_hListenThread != INVALID_HANDLE_VALUE){		
		s_bExit = true;/** 通知线程退出 */		
		Sleep(10);/** 等待线程退出 */		
		CloseHandle(m_hListenThread);/** 置线程句柄无效 */
		m_hListenThread = INVALID_HANDLE_VALUE;
	}
	return true;
}

UINT CSerialPort::GetBytesInCOM(){
	DWORD dwError = 0;  /** 错误码 */
	COMSTAT  comstat;   /** COMSTAT结构体,记录通信设备的状态信息 */
	memset(&comstat, 0, sizeof(COMSTAT));
	UINT BytesInQue = 0;
	/** 在调用ReadFile和WriteFile之前,通过本函数清除以前遗留的错误标志 */
	if (ClearCommError(m_hComm, &dwError, &comstat))
		BytesInQue = comstat.cbInQue; /** 获取在输入缓冲区中的字节数 */
	return BytesInQue;
}
// 写入日志
void LogWrite(char dat[], unsigned int num) {//log
	std::ofstream outputFile("RecvLog.txt", std::ios::app);
	if (!outputFile.is_open()) {  // 检查文件是否成功打开
		std::cerr << "无法打开文件1.txt" << std::endl;
		return ;  // 返回错误代码，表示打开文件失败
	}
	outputFile.write(dat, num);
	outputFile.close();  // 确保文件正常关闭
}
UINT WINAPI CSerialPort::ListenThread(void* pParam){//线程函数
	CSerialPort *pSerialPort = reinterpret_cast<CSerialPort*>(pParam);
	int i = 0;
	// 线程循环,轮询方式读取串口数据  
	while (!pSerialPort->s_bExit){
		UINT BytesInQue = pSerialPort->GetBytesInCOM();
		/** 如果串口输入缓冲区中无数据,则休息一会再查询 */
		if (BytesInQue == 0){
			Sleep(SLEEP_TIME_INTERVAL);
			continue;
		}
		i = 0;
		/** 读取输入缓冲区中的数据并输出显示 */
		char cRecved = 0x00;// 每次进入循环前重置接收的字符为0		
		do{
			cRecved = 0x00;
			if (pSerialPort->ReadChar(cRecved) == true)	{
				pSerialPort->data[i] = cRecved;// 将接收到的字符存储到 data 数组中
				if(pSerialPort->data[i] == '\n'){// 检查接收到的字符是否是特定的终止字符（'c'）
					LogWrite(pSerialPort->data,i);
					pSerialPort->data[i-1] = '\0';// 如果是 '\n'，将\r(\r\n)替换为字符串结束符 '\0'
					i = 0;
					pSerialPort->c_Recvd = true;
					break;
				}
				i++;
				continue;
			}
		} while (--BytesInQue);	
	}
	return 0;
}
//UINT WINAPI CSerialPort::ListenThread(void* pParam) {
//	CSerialPort* pSerialPort = reinterpret_cast<CSerialPort*>(pParam);// 将传入的参数转换为 CSerialPort 对象指针
//	HANDLE hCom = pSerialPort->m_hComm;// 获取串口句柄	
//	DWORD dwEventMask;// 定义事件掩码、接收到的字符、串口状态、错误信息等变量
//	char cRecved=0x00;
//	COMSTAT comStat;
//	DWORD dwErrors;
//	// 设置串口接收事件掩码，只关心接收到字符事件（EV_RXCHAR）,初始化时，已设置
//	SetCommMask(hCom, EV_RXCHAR);
//	// 进入监听循环，直到设置退出标志
//	while (!pSerialPort->s_bExit) {// 等待串口接收到数据的事件	
//		if (!WaitCommEvent(hCom, &dwEventMask, nullptr)) {
//			DWORD err = GetLastError();  // 获取错误代码
//										 // 如果是挂起的IO错误，则休眠 100 毫秒后继续等待
//			if (err == ERROR_IO_PENDING) {
//				Sleep(100);
//				continue;
//			}
//			else {// 如果发生其他错误，记录日志并退出
//				  //LogError("WaitCommEvent failed", err);
//				break;
//			}
//		}
//		// 如果接收到的事件不是数据接收事件，则跳过
//		if ((dwEventMask & EV_RXCHAR) == 0) {
//			continue;
//		}
//		DWORD bytesAvailable = 0;// 获取当前串口的输入队列状态
//		if (!ClearCommError(hCom, &dwErrors, &comStat)) {
//			// 如果清除串口错误失败，记录日志并继续
//			//LogError("ClearCommError failed", GetLastError());
//			continue;
//		}
//		bytesAvailable = comStat.cbInQue;// 获取队列中可读字节数	
//		for (DWORD i = 0; i < bytesAvailable; ++i) {// 遍历所有可读字节	
//			if (!pSerialPort->ReadChar(cRecved)) {// 读取一个字节
//				break;
//			}
//			EnterCriticalSection(&pSerialPort->m_csCommunicationSync);// 进入临界区，确保数据处理的同步	
//			if (pSerialPort->dataIndex >= MAX_DATA_SIZE - 1) {// 检查数据缓冲区是否已满，若满则重置为 0
//				pSerialPort->dataIndex = 0;
//			}
//			pSerialPort->data[pSerialPort->dataIndex] = cRecved;// 存储接收到的字符	
//			if (cRecved == '\n') {// 如果接收到换行符，表示数据接收完成	
//				LogWrite(pSerialPort->data, pSerialPort->dataIndex);// 记录接收到的数据		
//				pSerialPort->data[pSerialPort->dataIndex - 1] = '\0';// 将数据缓冲区的最后一个字符（换行符）替换为终止符	
//				pSerialPort->dataIndex = 0;// 重置数据索引，准备接收下一组数据			
//				pSerialPort->c_Recvd = true;// 标记数据已接收
//			}
//			else {// 如果没有接收到换行符，继续增加数据索引
//				pSerialPort->dataIndex++;
//			}
//			// 离开临界区，完成数据处理
//			LeaveCriticalSection(&pSerialPort->m_csCommunicationSync);
//		}
//	}
//	return 0;
//}

bool CSerialPort::ReadChar(char &cRecved){
	BOOL  bResult = TRUE;
	DWORD BytesRead = 0;
	if (m_hComm == INVALID_HANDLE_VALUE)
		return false;
	/** 临界区保护 */
	EnterCriticalSection(&m_csCommunicationSync);
	/** 从缓冲区读取一个字节的数据 */
	bResult = ReadFile(m_hComm, &cRecved, 1, &BytesRead, NULL);
	if ((!bResult)){
		/** 获取错误码,可以根据该错误码查出错误原因 */
		DWORD dwError = GetLastError();
		/** 清空串口缓冲区 */
		PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_RXABORT);
		//FlushFileBuffers(m_hComm);
		LeaveCriticalSection(&m_csCommunicationSync);
		return false;
	}
	/** 离开临界区 */
	LeaveCriticalSection(&m_csCommunicationSync);
	return (BytesRead == 1);
}
bool CSerialPort::WriteData(char* pData, unsigned int length){
	BOOL   bResult = TRUE;
	DWORD  BytesToSend = 0;	
	if (m_hComm == INVALID_HANDLE_VALUE)
		return false;
	/** 临界区保护 */
	EnterCriticalSection(&m_csCommunicationSync);
	/* 判断长度 */
	//length = sizeof(pData);
	/** 向缓冲区写入指定量的数据 */
	bResult = WriteFile(m_hComm, pData, length, &BytesToSend, NULL);
	if (!bResult){
		DWORD dwError = GetLastError();
		/** 清空串口缓冲区 */
		PurgeComm(m_hComm, PURGE_TXCLEAR | PURGE_TXABORT);
		LeaveCriticalSection(&m_csCommunicationSync);
		return false;
	}
	/** 离开临界区 */
	LeaveCriticalSection(&m_csCommunicationSync);
	return true;
}
// 采样命令函数
bool CSerialPort::Sample(unsigned char category, unsigned int frequency, unsigned int data2){
	char buffer[50];
	buffer[1] = '\r'; buffer[2] = '\n'; buffer[3] = '\0';
	switch (category)
	{
	case 0://停止起弧
		buffer[0] = '0';
		break;
	case 1:// 起弧信号
		buffer[0] = '1'; 
		break;
	case 2:	//超声波及限位开关	
		buffer[0] = '2'; 
		break;
	case 3:// 硬启动: 写入频率和数据个数
		snprintf(buffer, sizeof(buffer), "3,%u,%u,\r\n", 1000000/frequency, data2);
		break;
	case 4://软启动: 写入频率和数据个数
		snprintf(buffer, sizeof(buffer), "4,%u,%u,\r\n", 1000000/frequency, data2);
		break;
	default:
		//std::cout << "Unknown category!" << std::endl;
		return 1;
	}
	// 调用 WriteData 函数将数据写入
	unsigned int length = static_cast<unsigned int>(strlen(buffer));
	WriteData(buffer, length);
	LogWrite(buffer, length);
	return 0;
}

//bool CSerialPort::MotorCtrl(short Vert,short Hor,short Left,short Right)
//{
//	char buffer[50];
//	snprintf(buffer, sizeof(buffer), "%hd,%hd,%hd,%hd,\r\n", Vert, Hor, Left, Right);
//	// 调用 WriteData 函数将数据写入
//	unsigned int length = static_cast<unsigned int>(strlen(buffer));
//	WriteData(buffer, length);
//	LogWrite(buffer, length);
//	return 0;
//}

//控制多少转
bool CSerialPort::MotorCtrl_circle(short Vert, short Hor, short Left, short Right,
                                   float /*circle1*/, float /*circle2*/, float /*circle3*/, float /*circle4*/)
{
	// STM32 固件只支持 Vert,Hor,Left,Right 四字段速度控制，circle 参数被忽略
	return MotorCtrl(Vert, Hor, Left, Right);
}


bool CSerialPort::MotorCtrl(short Vert, short Hor, short Left, short Right)
{
	char buffer[64];
	snprintf(buffer, sizeof(buffer), "%hd,%hd,%hd,%hd,\r\n", Vert, Hor, Left, Right);
	unsigned int length = static_cast<unsigned int>(strlen(buffer));
	bool ok = WriteData(buffer, length);
	LogWrite(buffer, length);
	return ok;
}

// 重载版本，接受一个数组作为输入
bool CSerialPort::MotorCtrl(short motorParams[4])
{
	return MotorCtrl(motorParams[0], motorParams[1], motorParams[2], motorParams[3]);
}
