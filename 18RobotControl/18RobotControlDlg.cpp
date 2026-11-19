
// 18RobotControlDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "18RobotControl.h"
#include "18RobotControlDlg.h"
#include "afxdialogex.h"
#include <iostream>
#include <fstream>


#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// 用于应用程序“关于”菜单项的 CAboutDlg 对话框

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CMy18RobotControlDlg 对话框



CMy18RobotControlDlg::CMy18RobotControlDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_MY18ROBOTCONTROL_DIALOG, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CMy18RobotControlDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_TAB1, mc_tab);
}

BEGIN_MESSAGE_MAP(CMy18RobotControlDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_NOTIFY(TCN_SELCHANGE, IDC_TAB1, &CMy18RobotControlDlg::OnSelchangeTab1)
END_MESSAGE_MAP()


// CMy18RobotControlDlg 消息处理程序

BOOL CMy18RobotControlDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 将“关于...”菜单项添加到系统菜单中。

	// IDM_ABOUTBOX 必须在系统命令范围内。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 设置此对话框的图标。  当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标
	// TODO: 在此添加额外的初始化代码
	mc_tab.InsertItem(0, _T("主界面"));
	mc_tab.InsertItem(1, _T("串口连接"));//选项卡第n个选项标题
	// 第三个选项卡：图像显示（对应 IDD_DIALOG1）
	mc_tab.InsertItem(2, _T("图像显示"));
	//添加第1个选项卡界面，选项卡对象.Create(界面ID，选项卡变量)
	m_page1.Create(MAKEINTRESOURCE(IDD_Page1), &mc_tab);
	m_page2.Create(MAKEINTRESOURCE(IDD_Page2), &mc_tab);
	// 创建第三个选项卡对应的页面，资源模板为 IDD_DIALOG1
	m_page3.Create(MAKEINTRESOURCE(IDD_DIALOG1), &mc_tab);
	m_page1.SetParent(&mc_tab);
	m_page2.SetParent(&mc_tab);
	m_page3.SetParent(&mc_tab);
	//自动设置界面大小
	CRect rectTab;
	mc_tab.GetClientRect(&rectTab);
	mc_tab.AdjustRect(FALSE, rectTab);  // 计算显示区域																		
	m_page1.SetWindowPos(NULL, rectTab.left, rectTab.top,// 调整子对话框位置	
		rectTab.Width(), rectTab.Height(), SWP_HIDEWINDOW);
	m_page2.SetWindowPos(NULL, rectTab.left, rectTab.top,
		rectTab.Width(), rectTab.Height(), SWP_HIDEWINDOW);
	// 设置第三个页面的位置和大小，初始隐藏，由选项卡切换控制显示
	m_page3.SetWindowPos(NULL, rectTab.left, rectTab.top,
		rectTab.Width(), rectTab.Height(), SWP_HIDEWINDOW);
	m_page1.ShowWindow(SW_SHOW);  // 默认显示第一个子页面
	// 重定向 cout 到文件。使用堆对象保证整个程序运行期间缓冲区始终有效
	static std::ofstream* debugLog = new std::ofstream("debug_log.txt");
	if (debugLog->is_open()) {
		std::cout.rdbuf(debugLog->rdbuf());
		std::cout << std::unitbuf;  // 每次输出立即刷新，便于实时查看
	}
	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

void CMy18RobotControlDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。  对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void CMy18RobotControlDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR CMy18RobotControlDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

void CMy18RobotControlDlg::OnSelchangeTab1(NMHDR *pNMHDR, LRESULT *pResult)
{
	// TODO: 在此添加控件通知处理程序代码
	*pResult = 0;
	m_page1.ShowWindow(SW_HIDE);//隐藏所有窗口
	m_page2.ShowWindow(SW_HIDE);
	// 切换选项卡时先隐藏所有页面
	m_page3.ShowWindow(SW_HIDE);
	switch (mc_tab.GetCurSel()) {//显示对应窗口
	case 0://选项1
		m_page1.ShowWindow(SW_SHOW); break;
	case 1://选项2
		m_page2.ShowWindow(SW_SHOW); break;
	case 2://第三个选项卡：显示 IDD_DIALOG1 页面
		m_page3.ShowWindow(SW_SHOW); break;
	default: break;
	}
}
