#include "pch.h"
#include "VisionTracker.UI.h"
#include "VisionTracker.UIDlg.h"
#include "afxdialogex.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

// 응용 프로그램 정보에 사용되는 CAboutDlg 대화 상자입니다.

class CAboutDlg : public CDialogEx
{
public:
    CAboutDlg();

    // 대화 상자 데이터입니다.
#ifdef AFX_DESIGN_TIME
    enum { IDD = IDD_ABOUTBOX };
#endif

protected:
    virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 지원입니다.

    // 구현입니다.
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

// CVisionTrackerUIDlg 대화 상자

CVisionTrackerUIDlg::CVisionTrackerUIDlg(CWnd* pParent /*=NULL*/)
    : CDialogEx(IDD_VISIONTRACKER_UI_DIALOG, pParent)
    , m_fps(0.0)
    , m_brightness(0.0)
    , m_i_cropposx(0)
    , m_i_cropposy(0)
    , m_i_cropsizew(640)
    , m_i_cropsizeh(480)
    , m_f_fps(120)
    , m_f_exposure(120)
    , m_f_gamma(0.8f)
    , m_f_gain(11.0f)
    , m_i_blacklevel(30)
    , m_bInitialized(FALSE)
    , m_bConnected(FALSE)
{
    m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CVisionTrackerUIDlg::DoDataExchange(CDataExchange* pDX)
{
    CDialogEx::DoDataExchange(pDX);
    DDX_Control(pDX, IDC_STATIC_IMAGE, m_imageCtrl);
    DDX_Text(pDX, IDC_EDIT_FPS, m_fps);
    DDX_Text(pDX, IDC_EDIT_BRIGHTNESS, m_brightness);
    DDX_Text(pDX, IDC_EDIT_cropposX, m_i_cropposx);
    DDX_Text(pDX, IDC_EDIT_cropposY, m_i_cropposy);
    DDX_Text(pDX, IDC_EDIT_cropsizeX, m_i_cropsizew);
    DDX_Text(pDX, IDC_EDIT_cropsizeY, m_i_cropsizeh);
    DDX_Text(pDX, IDC_EDIT_settingFPS, m_f_fps);
    DDX_Text(pDX, IDC_EDIT_expose, m_f_exposure);
    DDX_Text(pDX, IDC_EDIT_gamma, m_f_gamma);
    DDX_Text(pDX, IDC_EDIT_gain, m_f_gain);
    DDX_Text(pDX, IDC_EDIT_blacklevel, m_i_blacklevel);
    DDX_Control(pDX, IDC_STATIC_BALLINFO, m_ballInfoCtrl);
    DDX_Control(pDX, IDC_STATIC_LED, m_led);
    DDX_Control(pDX, IDC_LIST_CAMERAS, m_cameraListBox);
    DDX_Control(pDX, IDC_BUTTON_CONNECT_CAMERA, m_btnConnect);
    DDX_Control(pDX, IDC_BUTTON_DISCONNECT_CAMERA, m_btnDisconnect);
    DDX_Control(pDX, IDC_BUTTON_SCAN_CAMERAS, m_btnScanCameras);
    DDX_Control(pDX, IDC_STATIC_CONNECTION_STATUS, m_connectionStatus);
    DDX_Control(pDX, IDC_STATIC_CAMERA_INFO, m_cameraInfoCtrl);
}

BEGIN_MESSAGE_MAP(CVisionTrackerUIDlg, CDialogEx)
    ON_WM_SYSCOMMAND()
    ON_WM_PAINT()
    ON_WM_QUERYDRAGICON()
    ON_BN_CLICKED(IDC_BUTTON1, &CVisionTrackerUIDlg::OnBnClickedButton1)
    ON_BN_CLICKED(IDC_BUTTON2, &CVisionTrackerUIDlg::OnBnClickedButton2)
    ON_WM_TIMER()
    ON_WM_DESTROY()
    ON_WM_CTLCOLOR()
    ON_BN_CLICKED(IDC_CHK_TRACK, &CVisionTrackerUIDlg::OnClickedChkTrack)
    ON_WM_ERASEBKGND()
    ON_WM_DRAWITEM()
    ON_BN_CLICKED(IDC_BUTTON_SCAN_CAMERAS, &CVisionTrackerUIDlg::OnBnClickedButtonScanCameras)
    ON_BN_CLICKED(IDC_BUTTON_CONNECT_CAMERA, &CVisionTrackerUIDlg::OnBnClickedButtonConnectCamera)
    ON_BN_CLICKED(IDC_BUTTON_DISCONNECT_CAMERA, &CVisionTrackerUIDlg::OnBnClickedButtonDisconnectCamera)
    ON_LBN_SELCHANGE(IDC_LIST_CAMERAS, &CVisionTrackerUIDlg::OnLbnSelchangeListCameras)
END_MESSAGE_MAP()

// CVisionTrackerUIDlg 메시지 처리기

BOOL CVisionTrackerUIDlg::OnInitDialog()
{
    CDialogEx::OnInitDialog();

    // 시스템 메뉴에 "정보..." 메뉴 항목을 추가합니다.
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

    // 이 대화 상자의 아이콘을 설정합니다.
    SetIcon(m_hIcon, TRUE);      // 큰 아이콘을 설정합니다.
    SetIcon(m_hIcon, FALSE);     // 작은 아이콘을 설정합니다.

    // Picture Control의 스타일을 Owner Draw로 변경
    m_imageCtrl.ModifyStyle(SS_BLACKFRAME | SS_CENTERIMAGE, SS_OWNERDRAW);

    // 더블 버퍼링을 위한 메모리 DC 초기화
    CRect imageRect;
    m_imageCtrl.GetClientRect(&imageRect);
    CDC* pDC = m_imageCtrl.GetDC();
    if (pDC) {
        m_memDC.CreateCompatibleDC(pDC);
        m_memBitmap.CreateCompatibleBitmap(pDC, imageRect.Width(), imageRect.Height());
        m_oldBitmap = m_memDC.SelectObject(&m_memBitmap);

        // GDI+ 렌더링 품질 설정
        m_memDC.SetStretchBltMode(HALFTONE);
        SetBrushOrgEx(m_memDC.GetSafeHdc(), 0, 0, NULL);

        // 메모리 DC를 검은색으로 초기화
        CBrush blackBrush(RGB(0, 0, 0));
        m_memDC.FillRect(&imageRect, &blackBrush);

        // 초기 메시지 표시
        m_memDC.SetBkColor(RGB(0, 0, 0));
        m_memDC.SetTextColor(RGB(200, 200, 200));

        // 폰트 설정
        CFont font;
        font.CreatePointFont(100, _T("맑은 고딕"));
        CFont* pOldFont = m_memDC.SelectObject(&font);

        CString strInit = _T("카메라를 연결해주세요");
        m_memDC.DrawText(strInit, &imageRect, DT_CENTER | DT_VCENTER | DT_SINGLELINE);

        m_memDC.SelectObject(pOldFont);
        font.DeleteObject();

        m_imageCtrl.ReleaseDC(pDC);
    }

    // DIB 버퍼 예약 (성능 향상)
    m_dibBuffer.reserve(1920 * 1080 * 3); // 최대 해상도 예상치

    // Picture Control 강제 다시 그리기
    m_imageCtrl.Invalidate();
    m_imageCtrl.UpdateWindow();

    // 초기 상태 설정
    m_bConnected = FALSE;
    UpdateConnectionStatus();
    EnableCameraControls(FALSE);

    // LED 설정
    {
        // LED를 크게 보이게 글꼴 설정
        if (m_ledFont.GetSafeHandle() == nullptr) {
            m_ledFont.CreatePointFont(120, _T("Segoe UI Symbol")); // 12pt로 조정
            m_led.SetFont(&m_ledFont);
        }
        m_led.SetWindowText(_T("●")); // 동그라미 문자
        m_ledColor = RGB(220, 0, 0);  // OFF 빨강
        // 체크박스는 OFF로 시작
        m_isTrackingOn = FALSE;
        CheckDlgButton(IDC_CHK_TRACK, BST_UNCHECKED);

        // 초기 LED 그리기
        if (CWnd* pLed = GetDlgItem(IDC_STATIC_LED)) pLed->Invalidate();
    }

    // 자동 카메라 검색 (선택사항)
    PostMessage(WM_COMMAND, MAKEWPARAM(IDC_BUTTON_SCAN_CAMERAS, BN_CLICKED), 0);

    m_bInitialized = TRUE;

    return TRUE;  // 포커스를 컨트롤에 설정하지 않으면 TRUE를 반환합니다.
}

void CVisionTrackerUIDlg::OnSysCommand(UINT nID, LPARAM lParam)
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

void CVisionTrackerUIDlg::OnPaint()
{
    if (IsIconic())
    {
        CPaintDC dc(this); // 그리기를 위한 디바이스 컨텍스트입니다.

        SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

        // 클라이언트 사각형에서 아이콘을 가운데에 맞춥니다.
        int cxIcon = GetSystemMetrics(SM_CXICON);
        int cyIcon = GetSystemMetrics(SM_CYICON);
        CRect rect;
        GetClientRect(&rect);
        int x = (rect.Width() - cxIcon + 1) / 2;
        int y = (rect.Height() - cyIcon + 1) / 2;

        // 아이콘을 그립니다.
        dc.DrawIcon(x, y, m_hIcon);
    }
    else
    {
        CDialogEx::OnPaint();

        // Picture Control 영역 강제 다시 그리기
        if (m_bInitialized) {
            m_imageCtrl.Invalidate();
        }
    }
}

HCURSOR CVisionTrackerUIDlg::OnQueryDragIcon()
{
    return static_cast<HCURSOR>(m_hIcon);
}

BOOL CVisionTrackerUIDlg::OnEraseBkgnd(CDC* pDC)
{
    // Picture Control 영역 제외하고 배경 지우기
    CRect rect;
    GetClientRect(&rect);

    CRect imageRect;
    if (::IsWindow(m_imageCtrl.GetSafeHwnd())) {
        m_imageCtrl.GetWindowRect(&imageRect);
        ScreenToClient(&imageRect);

        CRgn rgn1, rgn2;
        rgn1.CreateRectRgnIndirect(&rect);
        rgn2.CreateRectRgnIndirect(&imageRect);
        rgn1.CombineRgn(&rgn1, &rgn2, RGN_DIFF);

        CBrush brush(GetSysColor(COLOR_3DFACE));
        pDC->FillRgn(&rgn1, &brush);
        return TRUE;
    }

    return CDialogEx::OnEraseBkgnd(pDC);
}

void CVisionTrackerUIDlg::OnDrawItem(int nIDCtl, LPDRAWITEMSTRUCT lpDrawItemStruct)
{
    if (nIDCtl == IDC_STATIC_IMAGE && m_memDC.GetSafeHdc())
    {
        CDC* pDC = CDC::FromHandle(lpDrawItemStruct->hDC);
        CRect rect = lpDrawItemStruct->rcItem;

        // 메모리 DC의 내용을 화면에 복사
        pDC->BitBlt(0, 0, rect.Width(), rect.Height(), &m_memDC, 0, 0, SRCCOPY);
    }
    else
    {
        CDialogEx::OnDrawItem(nIDCtl, lpDrawItemStruct);
    }
}

float CVisionTrackerUIDlg::GetDlgItemFloat(int nID)
{
    CString str;
    GetDlgItemText(nID, str);
    return static_cast<float>(_tstof(str));
}

void CVisionTrackerUIDlg::OnBnClickedButtonScanCameras()
{
    m_cameraListBox.ResetContent();
    m_cameraList.clear();

    SetDlgItemText(IDC_STATIC_SYSTEM_STATUS, _T("카메라 검색 중..."));

    // 최대 10개 카메라 검색
    m_cameraList.resize(10);
    int count = EnumerateCameras(m_cameraList.data(), 10);
    m_cameraList.resize(count);

    if (count == 0) {
        AfxMessageBox(_T("연결 가능한 카메라를 찾을 수 없습니다.\n카메라가 네트워크에 연결되어 있는지 확인해주세요."));
        m_btnConnect.EnableWindow(FALSE);
        SetDlgItemText(IDC_STATIC_SYSTEM_STATUS, _T("카메라를 찾을 수 없음"));
        return;
    }

    // 리스트박스에 추가
    for (const auto& cam : m_cameraList) {
        CString displayName(cam.displayName);
        m_cameraListBox.AddString(displayName);
    }

    // 첫 번째 항목 선택
    if (count > 0) {
        m_cameraListBox.SetCurSel(0);
        OnLbnSelchangeListCameras();
    }

    CString msg;
    msg.Format(_T("%d개의 카메라를 찾았습니다."), count);
    SetDlgItemText(IDC_STATIC_SYSTEM_STATUS, msg);
}

void CVisionTrackerUIDlg::OnBnClickedButtonConnectCamera()
{
    int sel = m_cameraListBox.GetCurSel();
    if (sel == LB_ERR) {
        AfxMessageBox(_T("카메라를 선택해주세요."));
        return;
    }

    // 연결 시도
    SetDlgItemText(IDC_STATIC_SYSTEM_STATUS, _T("카메라 연결 중..."));
    BeginWaitCursor();

    if (ConnectCameraByIndex(sel)) {
        m_bConnected = TRUE;
        UpdateConnectionStatus();
        EnableCameraControls(TRUE);

        // 현재 설정값 읽기
        //GetGain(&m_f_gain);
        //GetBlackLevel(&m_i_blacklevel);
        //GetFrameRate(&m_f_fps);
        //GetExposureTime(&m_f_exposure);
        //GetGamma(&m_f_gamma);
        UpdateData(FALSE);

        SetDlgItemText(IDC_STATIC_SYSTEM_STATUS, _T("카메라 연결 성공"));

        // 연결된 카메라 정보 표시
        CameraInfo info;
        if (GetConnectedCameraInfo(&info)) {
            CString infoStr;
            infoStr.Format(_T("연결된 카메라: %s"), CString(info.modelName));
            SetDlgItemText(IDC_STATIC_SYSTEM_STATUS, infoStr);
        }
    }
    else {
        CString errMsg(GetLastErrorMessage());
        CString fullMsg;
        fullMsg.Format(_T("카메라 연결 실패: %s"), errMsg);
        AfxMessageBox(fullMsg);
        SetDlgItemText(IDC_STATIC_SYSTEM_STATUS, _T("카메라 연결 실패"));
    }
    EndWaitCursor();
}

void CVisionTrackerUIDlg::OnBnClickedButtonDisconnectCamera()
{
    if (m_bConnected) {
        StopGrab();
        KillTimer(1);
        DisconnectCamera();
        m_bConnected = FALSE;
        UpdateConnectionStatus();
        EnableCameraControls(FALSE);
        SetDlgItemText(IDC_STATIC_SYSTEM_STATUS, _T("카메라 연결 해제됨"));

        // 화면 초기화
        CRect imageRect;
        m_imageCtrl.GetClientRect(&imageRect);
        CBrush blackBrush(RGB(0, 0, 0));
        m_memDC.FillRect(&imageRect, &blackBrush);
        m_memDC.SetBkColor(RGB(0, 0, 0));
        m_memDC.SetTextColor(RGB(200, 200, 200));
        CString strInit = _T("카메라를 연결해주세요");
        m_memDC.DrawText(strInit, &imageRect, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
        m_imageCtrl.Invalidate();
    }
}

void CVisionTrackerUIDlg::OnLbnSelchangeListCameras()
{
    int sel = m_cameraListBox.GetCurSel();
    if (sel != LB_ERR && sel < (int)m_cameraList.size()) {
        // 선택된 카메라 정보 표시
        const CameraInfo& cam = m_cameraList[sel];
        CString info;
        if (cam.deviceType == 0) { // GigE
            info.Format(_T("모델: %s\n시리얼: %s\nIP: %s"),
                CString(cam.modelName),
                CString(cam.serialNumber),
                CString(cam.ipAddress));
        }
        else { // USB
            info.Format(_T("모델: %s\n시리얼: %s\n타입: USB"),
                CString(cam.modelName),
                CString(cam.serialNumber));
        }
        SetDlgItemText(IDC_STATIC_CAMERA_INFO, info);

        m_btnConnect.EnableWindow(!m_bConnected);
    }
}

void CVisionTrackerUIDlg::UpdateConnectionStatus()
{
    if (m_bConnected) {
        m_connectionStatus.SetWindowText(_T("상태: 연결됨"));
        m_btnConnect.EnableWindow(FALSE);
        m_btnDisconnect.EnableWindow(TRUE);
        GetDlgItem(IDC_BUTTON1)->EnableWindow(TRUE);  // Start 버튼

        // LED 색상 업데이트 (연결됨 = 파란색)
        if (CWnd* pStatus = GetDlgItem(IDC_STATIC_CONNECTION_STATUS)) {
            pStatus->SetWindowText(_T("● 연결됨"));
        }
    }
    else {
        m_connectionStatus.SetWindowText(_T("상태: 연결되지 않음"));
        m_btnConnect.EnableWindow(m_cameraListBox.GetCurSel() != LB_ERR);
        m_btnDisconnect.EnableWindow(FALSE);
        GetDlgItem(IDC_BUTTON1)->EnableWindow(FALSE);  // Start 버튼
        GetDlgItem(IDC_BUTTON2)->EnableWindow(FALSE);  // Stop 버튼

        // LED 색상 업데이트 (미연결 = 빨간색)
        if (CWnd* pStatus = GetDlgItem(IDC_STATIC_CONNECTION_STATUS)) {
            pStatus->SetWindowText(_T("● 미연결"));
        }
    }
}

void CVisionTrackerUIDlg::EnableCameraControls(BOOL enable)
{
    // 카메라 설정 관련 컨트롤 활성화/비활성화
    GetDlgItem(IDC_EDIT_cropposX)->EnableWindow(enable);
    GetDlgItem(IDC_EDIT_cropposY)->EnableWindow(enable);
    GetDlgItem(IDC_EDIT_cropsizeX)->EnableWindow(enable);
    GetDlgItem(IDC_EDIT_cropsizeY)->EnableWindow(enable);
    GetDlgItem(IDC_EDIT_settingFPS)->EnableWindow(enable);
    GetDlgItem(IDC_EDIT_expose)->EnableWindow(enable);
    GetDlgItem(IDC_EDIT_gamma)->EnableWindow(enable);
    GetDlgItem(IDC_EDIT_gain)->EnableWindow(enable);
    GetDlgItem(IDC_EDIT_blacklevel)->EnableWindow(enable);
    GetDlgItem(IDC_CHK_TRACK)->EnableWindow(enable);
}

void CVisionTrackerUIDlg::RefreshCameraList()
{
    OnBnClickedButtonScanCameras();
}

void CVisionTrackerUIDlg::OnBnClickedButton1()
{
    // Start 버튼 - 연결 확인 추가
    if (!m_bConnected || !IsConnected()) {
        AfxMessageBox(_T("카메라가 연결되지 않았습니다.\n먼저 카메라를 연결해주세요."));
        return;
    }

    // 시작 버튼 - 캡처 스펙 변환 및 시작
    UpdateData(TRUE);

    // 캡처 스펙 가져오기
    int cropX = GetDlgItemInt(IDC_EDIT_cropposX);
    int cropY = GetDlgItemInt(IDC_EDIT_cropposY);
    int cropW = GetDlgItemInt(IDC_EDIT_cropsizeX);
    int cropH = GetDlgItemInt(IDC_EDIT_cropsizeY);
    m_i_cropposx = cropX;
    m_i_cropposy = cropY;
    m_i_cropsizew = cropW;
    m_i_cropsizeh = cropH;

    float fps = GetDlgItemFloat(IDC_EDIT_settingFPS);
    float exposure = GetDlgItemFloat(IDC_EDIT_expose);
    float gamma = GetDlgItemFloat(IDC_EDIT_gamma);
    float gain = GetDlgItemFloat(IDC_EDIT_gain);
    float blacklevel = GetDlgItemFloat(IDC_EDIT_blacklevel);

    // DLL을 통해 설정값 전달
    SetFrameRate(fps);
    SetExposureTime(exposure);
    SetGamma(gamma);
    SetGain(gain);
    SetBlackLevel((int)blacklevel);
    SetROI(cropX, cropY, cropW, cropH);

    // 볼 탐색 조건 설정
    {
        WhiteBallDetectionConfig config;
        config.roi = cv::Rect(0, 0, 640, 480);        // 볼 탐색 영역 ROI 제한
        config.thresholdMode = ThresholdMode::Fixed;   // Fixed, Adaptive, Otsu
        config.thresholdValue = 90;
        config.thresholdType = cv::THRESH_BINARY;      // 0~4
        config.adaptiveMethod = AdaptiveMethod::Gaussian;
        config.blockSize = 7;                          // 홀수 3 이상
        config.C = 1.0;                                // 평균에서 빼는 값
        config.minRadius = 2.5f;
        config.maxRadius = 10.0f;
        config.minCircularity = 0.8f;
        config.maxCircularity = 1.2f;
        config.useTracking = false;  // 이전 프레임 추적 사용 안함

        SetWhiteBallDetectionConfig(config);
    }

    StartGrab();                    // DLL 프레임 수신 시작
    SetTimer(1, 33, nullptr);       // 약 33ms(30fps) 주기로 타이머 호출

    GetDlgItem(IDC_BUTTON2)->EnableWindow(TRUE);  // Stop 버튼 활성화
    SetDlgItemText(IDC_STATIC_SYSTEM_STATUS, _T("캡처 시작됨"));
}

void CVisionTrackerUIDlg::OnBnClickedButton2()
{
    // 정지 버튼
    StopGrab();    // DLL 프레임 수신 중지
    KillTimer(1);  // 타이머 중지

    GetDlgItem(IDC_BUTTON2)->EnableWindow(FALSE);  // Stop 버튼 비활성화
    SetDlgItemText(IDC_STATIC_SYSTEM_STATUS, _T("캡처 중지됨"));
}

void CVisionTrackerUIDlg::ShowMatToStatic(const cv::Mat& mat, CStatic& ctrl)
{
    CRect rect;
    ctrl.GetClientRect(&rect);

    // 컨트롤이 유효하지 않으면 리턴
    if (!::IsWindow(ctrl.GetSafeHwnd()) || rect.IsRectEmpty()) {
        return;
    }

    // mat이 비어있으면 검은 화면 표시
    if (mat.empty()) {
        CBrush blackBrush(RGB(0, 0, 0));
        m_memDC.FillRect(&rect, &blackBrush);
        ctrl.Invalidate();
        return;
    }

    // 배경을 먼저 검은색으로 채움
    CBrush blackBrush(RGB(0, 0, 0));
    m_memDC.FillRect(&rect, &blackBrush);

    // 영상 비율 유지하면서 크기 계산
    double srcRatio = (double)mat.cols / mat.rows;
    double dstRatio = (double)rect.Width() / rect.Height();

    int destWidth, destHeight, destX, destY;

    if (srcRatio > dstRatio) {
        // 원본이 더 넓음 - 너비에 맞추기
        destWidth = rect.Width();
        destHeight = (int)(rect.Width() / srcRatio);
        destX = 0;
        destY = (rect.Height() - destHeight) / 2;
    }
    else {
        // 원본이 더 좁음 - 높이에 맞추기
        destHeight = rect.Height();
        destWidth = (int)(rect.Height() * srcRatio);
        destX = (rect.Width() - destWidth) / 2;
        destY = 0;
    }

    // 영상 크기 맞추기
    cv::Mat resized;
    cv::resize(mat, resized, cv::Size(destWidth, destHeight), 0, 0, cv::INTER_LINEAR);

    // BGR/GRAY to RGB 변환 및 연속 메모리 보장
    cv::Mat rgb;
    if (resized.channels() == 1) {
        cv::cvtColor(resized, rgb, cv::COLOR_GRAY2RGB);
    }
    else if (resized.channels() == 3) {
        cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
    }
    else {
        rgb = resized.clone();
    }

    // Mat을 연속 메모리로 만들기 (중요!)
    if (!rgb.isContinuous()) {
        rgb = rgb.clone();
    }

    // Windows DIB 스캔라인 정렬을 위한 패딩 계산
    int bitsPerPixel = 24;
    int bytesPerPixel = bitsPerPixel / 8;
    int widthBytes = rgb.cols * bytesPerPixel;
    int stride = ((widthBytes + 3) / 4) * 4; // 4바이트 경계 정렬
    int padding = stride - widthBytes;

    // DIB용 버퍼 생성 (패딩 포함)
    std::vector<BYTE> dibBuffer;
    if (padding > 0) {
        dibBuffer.resize(stride * rgb.rows);

        // 각 행마다 데이터 복사 (패딩 추가)
        for (int y = 0; y < rgb.rows; y++) {
            const BYTE* srcRow = rgb.ptr<BYTE>(y);
            BYTE* dstRow = &dibBuffer[y * stride];

            // 픽셀 데이터 복사
            memcpy(dstRow, srcRow, widthBytes);

            // 패딩 영역 0으로 초기화
            if (padding > 0) {
                memset(dstRow + widthBytes, 0, padding);
            }
        }
    }

    // BITMAPINFO 구조체 설정
    BITMAPINFOHEADER bmiHeader = { 0 };
    bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bmiHeader.biWidth = rgb.cols;
    bmiHeader.biHeight = -rgb.rows; // 음수로 설정하여 상하 반전 방지
    bmiHeader.biPlanes = 1;
    bmiHeader.biBitCount = 24;
    bmiHeader.biCompression = BI_RGB;
    bmiHeader.biSizeImage = stride * rgb.rows;
    bmiHeader.biXPelsPerMeter = 0;
    bmiHeader.biYPelsPerMeter = 0;
    bmiHeader.biClrUsed = 0;
    bmiHeader.biClrImportant = 0;

    BITMAPINFO bmpInfo = { 0 };
    bmpInfo.bmiHeader = bmiHeader;

    // 렌더링 품질 설정
    SetStretchBltMode(m_memDC.GetSafeHdc(), HALFTONE);
    SetBrushOrgEx(m_memDC.GetSafeHdc(), 0, 0, NULL);

    // DIB 그리기
    const void* imageData = padding > 0 ? (const void*)dibBuffer.data() : (const void*)rgb.data;

    int result = StretchDIBits(
        m_memDC.GetSafeHdc(),
        destX, destY, destWidth, destHeight,
        0, 0, rgb.cols, rgb.rows,
        imageData,
        &bmpInfo,
        DIB_RGB_COLORS,
        SRCCOPY
    );

    if (result == GDI_ERROR) {
        DWORD err = GetLastError();
        CString errMsg;
        errMsg.Format(_T("StretchDIBits failed with error: %d"), err);
        OutputDebugString(errMsg);
    }

    // Picture Control을 다시 그리도록 요청
    ctrl.Invalidate();
    ctrl.UpdateWindow();
}

void CVisionTrackerUIDlg::OnTimer(UINT_PTR nIDEvent)
{
    if (nIDEvent == 1)
    {
        // 프레임 정보 가져오기
        int w = 0, h = 0, ch = 0;
        if (!GetLatestFrameInfo(&w, &h, &ch)) {
            OutputDebugString(_T("GetLatestFrameInfo 실패\n"));
            CDialogEx::OnTimer(nIDEvent);
            return;
        }

        std::vector<unsigned char> buffer(w * h * ch);
        if (!GetLatestFrame(buffer.data(), (int)buffer.size(), &w, &h, &ch)) {
            OutputDebugString(_T("GetLatestFrame 실패\n"));
            CDialogEx::OnTimer(nIDEvent);
            return;
        }

        // 카메라 프레임 → Mat
        cv::Mat img(h, w, (ch == 1 ? CV_8UC1 : CV_8UC3), buffer.data());
        cv::Mat disp;
        if (ch == 1) {
            // 원본 그레이 → BGR 변환
            cv::cvtColor(img, disp, cv::COLOR_GRAY2BGR);
        }
        else {
            // 3채널이면 그대로 사용
            disp = img.clone(); // clone 추가
        }

        // 추적이 활성화된 경우 공 위치 표시
        if (m_isTrackingOn) {
            float x, y, radius;
            bool found;
            GetCurrentBallPosition(&x, &y, &radius, &found);

            if (found) {
                // 공의 위치에 초록색 원 그리기
                cv::circle(disp, cv::Point((int)x, (int)y), (int)radius, cv::Scalar(0, 255, 0), 2);
                cv::circle(disp, cv::Point((int)x, (int)y), 2, cv::Scalar(0, 0, 255), -1);

                // 좌표 정보 표시
                CString info;
                info.Format(_T("Ball Position: (%.0f, %.0f), Radius: %.0f"), x, y, radius);
                m_ballInfoCtrl.SetWindowText(info);
            }
            else {
                m_ballInfoCtrl.SetWindowText(_T("Ball Not Found"));
            }
        }
        else {
            m_ballInfoCtrl.SetWindowText(_T("Tracking OFF"));
        }

        // 오버레이된 영상 출력 - 더블 버퍼링 사용
        ShowMatToStatic(disp, m_imageCtrl);

        // FPS/밝기 표기
        m_fps = GetCurrentFps();
        m_brightness = GetCurrentBrightness();
        CString strFps, strBright;
        strFps.Format(_T("%.2f"), m_fps);
        strBright.Format(_T("%.2f"), m_brightness);
        SetDlgItemText(IDC_EDIT_FPS, strFps);
        SetDlgItemText(IDC_EDIT_BRIGHTNESS, strBright);
    }

    CDialogEx::OnTimer(nIDEvent);
}

void CVisionTrackerUIDlg::OnDestroy()
{
    CDialogEx::OnDestroy();

    // 더블 버퍼링 리소스 정리
    if (m_memDC.GetSafeHdc()) {
        m_memDC.SelectObject(m_oldBitmap);
        m_memBitmap.DeleteObject();
        m_memDC.DeleteDC();
    }

    // 카메라 종료
    if (m_bConnected) {
        StopGrab();
        DisconnectCamera();
    }
}

void CVisionTrackerUIDlg::OnOK()
{
    // 아무 작업도 하지 않음으로써 Enter 키로 종료되는 현상을 막음
}

HBRUSH CVisionTrackerUIDlg::OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor)
{
    HBRUSH hbr = CDialogEx::OnCtlColor(pDC, pWnd, nCtlColor);

    if (pWnd && pWnd->GetDlgCtrlID() == IDC_STATIC_LED) {
        // 글자색만 바꿔서 ● 를 빨강/초록으로 보이게
        pDC->SetTextColor(m_ledColor);

        // 배경 투명 처리
        pDC->SetBkMode(TRANSPARENT);
        return (HBRUSH)GetStockObject(HOLLOW_BRUSH);
    }
    return hbr;
}

void CVisionTrackerUIDlg::SetLedOn(BOOL on)
{
    m_isTrackingOn = on;
    m_ledColor = on ? RGB(0, 180, 0) : RGB(220, 0, 0); // 초록/빨강
    if (CWnd* pLed = GetDlgItem(IDC_STATIC_LED)) pLed->Invalidate();

    // LED 상태 텍스트 업데이트
    SetDlgItemText(IDC_STATIC_LED_STATUS, on ? _T("ON") : _T("OFF"));
}

void CVisionTrackerUIDlg::OnClickedChkTrack()
{
    const BOOL wantOn = (IsDlgButtonChecked(IDC_CHK_TRACK) == BST_CHECKED);

    // LED/상태 플래그 갱신
    SetLedOn(wantOn);

    if (wantOn) {
        StartTracking();
        SetDlgItemText(IDC_STATIC_SYSTEM_STATUS, _T("추적 활성화"));
    }
    else {
        StopTracking();
        SetDlgItemText(IDC_STATIC_SYSTEM_STATUS, _T("추적 비활성화"));
    }
}
