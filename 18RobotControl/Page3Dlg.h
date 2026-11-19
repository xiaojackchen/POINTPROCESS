#pragma once

// CPage3Dlg 对话框
// 第三个选项卡页面，绑定的对话框资源为 IDD_DIALOG1（图像显示界面）

class CPage3Dlg : public CDialogEx
{
	DECLARE_DYNAMIC(CPage3Dlg)

public:
	CPage3Dlg(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~CPage3Dlg();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_DIALOG1 };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
};
