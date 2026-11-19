#pragma once
#ifndef _SerialPort_H_  
#define _SerialPort_H_  
#include <Windows.h>  
double ADCToAnalog(unsigned short adcValue, double Rate = 1.0);//

char process_data(const char *data, unsigned short *arr1,
	unsigned short *arr2, unsigned short *val16, unsigned char *val8);
class CSerialPort{
public:
	CSerialPort(void);
	~CSerialPort(void);
public:
	bool MotorCtrl_circle(short Vert, short Hor, short Left, short Right, float circle1, float circle2, float circle3, float circle4);
	bool InitPort(UINT  portNo = 1, UINT  baud = CBR_1200, char  parity = 'N', UINT  databits = 8, UINT  stopsbits = 1, DWORD dwCommEvents = EV_RXCHAR);
	bool InitPort(UINT  portNo, const LPDCB& plDCB);
	bool Sample(unsigned char category, unsigned int frequency = 0, unsigned int data2 = 0);//采样命令函数
	//电机速度函数：竖直电机，水平电机，左轮，右轮，（速度，正负表示方向）
	bool MotorCtrl(short Vert,short Hor,short Left,short Right);
	//电机速度函数：竖直电机，水平电机，左轮，右轮，（速度，正负表示方向）
	bool MotorCtrl(short motorParams[4]);
	bool OpenListenThread();//开启监听线程
	bool CloseListenTread();//关闭线程，自动
	bool WriteData(char* pData, unsigned int length);//写入数据
	UINT GetBytesInCOM();
	bool ReadChar(char &cRecved);	
	void ClosePort();//关闭串口，自动执行
	bool c_Recvd=false;//接收到数据标志位
	
	int dataIndex = 0;          // 当前数据缓冲区索引
	static const int MAX_DATA_SIZE = 2048; // 缓冲区大小
	char data[MAX_DATA_SIZE];//接收到数据
private:
	bool openPort(UINT  portNo);
	static UINT WINAPI ListenThread(void* pParam);
private:	
	HANDLE  m_hComm;/** 串口句柄 */
	static bool s_bExit;/** 线程退出标志变量 */
	volatile HANDLE  m_hListenThread;/** 线程句柄 */
	/** 同步互斥,临界区保护 */
	CRITICAL_SECTION   m_csCommunicationSync;       //!< 互斥操作串口  
};
#endif //SERIALPORT_H_ 
