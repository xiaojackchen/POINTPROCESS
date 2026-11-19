
// 18RobotControlDlg.h : 头文件
//

#pragma once
#include "afxcmn.h"
#include "Page1Dlg.h"//选项卡头文件，根据自己命名的头文件添加
#include "Page2Dlg.h"
#include "Page3Dlg.h"//第三个选项卡页面的头文件
//几个选项，添加几个头文件
// CMy18RobotControlDlg 对话框
class CMy18RobotControlDlg : public CDialogEx
{
// 构造
public:
	CMy18RobotControlDlg(CWnd* pParent = NULL);	// 标准构造函数

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_MY18ROBOTCONTROL_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支持


// 实现
protected:
	HICON m_hIcon;
	CPage1Dlg m_page1;//选项卡对象1
	CPage2Dlg m_page2;
	CPage3Dlg m_page3;//第三个选项卡对应的页面（资源 IDD_DIALOG1）
	// 生成的消息映射函数
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	CTabCtrl mc_tab;
	afx_msg void OnSelchangeTab1(NMHDR *pNMHDR, LRESULT *pResult);
};
