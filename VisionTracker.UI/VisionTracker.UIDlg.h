#pragma once
#include "afxwin.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include "../VisionTracker.Core/IVisionTracker.h"

// CVisionTrackerUIDlg ��ȭ ����
class CVisionTrackerUIDlg : public CDialogEx
{
    // �����Դϴ�.
public:
    CVisionTrackerUIDlg(CWnd* pParent = NULL);    // ǥ�� �������Դϴ�.

    // ��ȭ ���� �������Դϴ�.
#ifdef AFX_DESIGN_TIME
    enum { IDD = IDD_VISIONTRACKER_UI_DIALOG };
#endif

protected:
    virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV �����Դϴ�.

    // �����Դϴ�.
protected:
    HICON m_hIcon;

    // ������ �޽��� �� �Լ�
    virtual BOOL OnInitDialog();
    afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
    afx_msg void OnPaint();
    afx_msg HCURSOR OnQueryDragIcon();
    afx_msg BOOL OnEraseBkgnd(CDC* pDC);
    afx_msg void OnDrawItem(int nIDCtl, LPDRAWITEMSTRUCT lpDrawItemStruct);
    DECLARE_MESSAGE_MAP()

public:
    // ���� �̺�Ʈ �ڵ鷯
    afx_msg void OnBnClickedButton1();  // ���� ��ư
    afx_msg void OnBnClickedButton2();  // ���� ��ư
    afx_msg void OnTimer(UINT_PTR nIDEvent);
    afx_msg void OnDestroy();
    afx_msg HBRUSH OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor);
    afx_msg void OnClickedChkTrack();

    // ���ο� �̺�Ʈ �ڵ鷯
    afx_msg void OnBnClickedButtonScanCameras();
    afx_msg void OnBnClickedButtonConnectCamera();
    afx_msg void OnBnClickedButtonDisconnectCamera();
    afx_msg void OnLbnSelchangeListCameras();

    void OnOK();  // Enter Ű ó�� ����

    // �̹��� ǥ�� �Լ�
    void ShowMatToStatic(const cv::Mat& mat, CStatic& ctrl);

    // ���� �Լ�
    float GetDlgItemFloat(int nID);
    void SetLedOn(BOOL on);
    void UpdateConnectionStatus();
    void EnableCameraControls(BOOL enable);
    void RefreshCameraList();

private:
    // ���� ��� ����
    double m_fps;
    double m_brightness;
    CStatic m_imageCtrl;
    CStatic m_ballInfoCtrl;

    // ROI ����
    int m_i_cropposx;
    int m_i_cropposy;
    int m_i_cropsizew;
    int m_i_cropsizeh;

    // ī�޶� ����
    float m_f_fps;
    float m_f_exposure;
    float m_f_gamma;
    float m_f_gain;
    int m_i_blacklevel;

    // LED ����
    CStatic m_led;
    CFont m_ledFont;
    COLORREF m_ledColor = RGB(220, 0, 0);  // ������ OFF=����
    BOOL m_isTrackingOn = FALSE;

    // ���ο� ��� ���� - ī�޶� ���� ����
    CListBox m_cameraListBox;
    CButton m_btnConnect;
    CButton m_btnDisconnect;
    CButton m_btnScanCameras;
    CStatic m_connectionStatus;
    CStatic m_cameraInfoCtrl;
    std::vector<CameraInfo> m_cameraList;
    bool m_bConnected;

    // ���� ���۸���
    CDC m_memDC;
    CBitmap m_memBitmap;
    CBitmap* m_oldBitmap;

    // �ʱ�ȭ �÷���
    BOOL m_bInitialized;

    // DIB ���� ĳ��
    std::vector<BYTE> m_dibBuffer;
    cv::Size m_lastImageSize;
};
