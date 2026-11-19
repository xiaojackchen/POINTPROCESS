// Page2Dlg.cpp : 实现文件
//

#include "stdafx.h"
#include "18RobotControl.h"
#include "Page2Dlg.h"
#include "afxdialogex.h"


// CPage2Dlg 对话框

IMPLEMENT_DYNAMIC(CPage2Dlg, CDialogEx)

CPage2Dlg::CPage2Dlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_Page2, pParent)
	, m_Baud1(0)
{

}

CPage2Dlg::~CPage2Dlg()
{
}

void CPage2Dlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_CBIndex(pDX, IDC_COMBO2, m_Baud1);
}


BEGIN_MESSAGE_MAP(CPage2Dlg, CDialogEx)
	ON_CBN_SELCHANGE(IDC_COMBO2, &CPage2Dlg::OnCbnSelchangeCombo2)
END_MESSAGE_MAP()


// CPage2Dlg 消息处理程序


void CPage2Dlg::OnCbnSelchangeCombo2()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(1);
	m_Baud1;
	UpdateData(0);
}
