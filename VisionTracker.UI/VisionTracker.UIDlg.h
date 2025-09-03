#pragma once
#include "afxwin.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include "../VisionTracker.Core/IVisionTracker.h"

// CVisionTrackerUIDlg 대화 상자
class CVisionTrackerUIDlg : public CDialogEx
{
    // 생성입니다.
public:
    CVisionTrackerUIDlg(CWnd* pParent = NULL);    // 표준 생성자입니다.

    // 대화 상자 데이터입니다.
#ifdef AFX_DESIGN_TIME
    enum { IDD = IDD_VISIONTRACKER_UI_DIALOG };
#endif

protected:
    virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 지원입니다.

    // 구현입니다.
protected:
    HICON m_hIcon;

    // 생성된 메시지 맵 함수
    virtual BOOL OnInitDialog();
    afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
    afx_msg void OnPaint();
    afx_msg HCURSOR OnQueryDragIcon();
    afx_msg BOOL OnEraseBkgnd(CDC* pDC);
    afx_msg void OnDrawItem(int nIDCtl, LPDRAWITEMSTRUCT lpDrawItemStruct);
    DECLARE_MESSAGE_MAP()

public:
    // 기존 이벤트 핸들러
    afx_msg void OnBnClickedButton1();  // 시작 버튼
    afx_msg void OnBnClickedButton2();  // 정지 버튼
    afx_msg void OnTimer(UINT_PTR nIDEvent);
    afx_msg void OnDestroy();
    afx_msg HBRUSH OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor);
    afx_msg void OnClickedChkTrack();

    // 새로운 이벤트 핸들러
    afx_msg void OnBnClickedButtonScanCameras();
    afx_msg void OnBnClickedButtonConnectCamera();
    afx_msg void OnBnClickedButtonDisconnectCamera();
    afx_msg void OnLbnSelchangeListCameras();

    void OnOK();  // Enter 키 처리 방지

    // 이미지 표시 함수
    void ShowMatToStatic(const cv::Mat& mat, CStatic& ctrl);

    // 헬퍼 함수
    float GetDlgItemFloat(int nID);
    void SetLedOn(BOOL on);
    void UpdateConnectionStatus();
    void EnableCameraControls(BOOL enable);
    void RefreshCameraList();

private:
    // 기존 멤버 변수
    double m_fps;
    double m_brightness;
    CStatic m_imageCtrl;
    CStatic m_ballInfoCtrl;

    // ROI 설정
    int m_i_cropposx;
    int m_i_cropposy;
    int m_i_cropsizew;
    int m_i_cropsizeh;

    // 카메라 설정
    float m_f_fps;
    float m_f_exposure;
    float m_f_gamma;
    float m_f_gain;
    int m_i_blacklevel;

    // LED 관련
    CStatic m_led;
    CFont m_ledFont;
    COLORREF m_ledColor = RGB(220, 0, 0);  // 시작은 OFF=빨강
    BOOL m_isTrackingOn = FALSE;

    // 새로운 멤버 변수 - 카메라 연결 관련
    CListBox m_cameraListBox;
    CButton m_btnConnect;
    CButton m_btnDisconnect;
    CButton m_btnScanCameras;
    CStatic m_connectionStatus;
    CStatic m_cameraInfoCtrl;
    std::vector<CameraInfo> m_cameraList;
    bool m_bConnected;

    // 더블 버퍼링용
    CDC m_memDC;
    CBitmap m_memBitmap;
    CBitmap* m_oldBitmap;

    // 초기화 플래그
    BOOL m_bInitialized;

    // DIB 버퍼 캐싱
    std::vector<BYTE> m_dibBuffer;
    cv::Size m_lastImageSize;
};
