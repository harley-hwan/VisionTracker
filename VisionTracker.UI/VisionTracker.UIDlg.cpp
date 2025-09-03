#include "pch.h"
#include "VisionTracker.UI.h"
#include "VisionTracker.UIDlg.h"
#include "afxdialogex.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

// ���� ���α׷� ������ ���Ǵ� CAboutDlg ��ȭ �����Դϴ�.

class CAboutDlg : public CDialogEx
{
public:
    CAboutDlg();

    // ��ȭ ���� �������Դϴ�.
#ifdef AFX_DESIGN_TIME
    enum { IDD = IDD_ABOUTBOX };
#endif

protected:
    virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV �����Դϴ�.

    // �����Դϴ�.
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

// CVisionTrackerUIDlg ��ȭ ����

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

// CVisionTrackerUIDlg �޽��� ó����

BOOL CVisionTrackerUIDlg::OnInitDialog()
{
    CDialogEx::OnInitDialog();

    // �ý��� �޴��� "����..." �޴� �׸��� �߰��մϴ�.
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

    // �� ��ȭ ������ �������� �����մϴ�.
    SetIcon(m_hIcon, TRUE);      // ū �������� �����մϴ�.
    SetIcon(m_hIcon, FALSE);     // ���� �������� �����մϴ�.

    // Picture Control�� ��Ÿ���� Owner Draw�� ����
    m_imageCtrl.ModifyStyle(SS_BLACKFRAME | SS_CENTERIMAGE, SS_OWNERDRAW);

    // ���� ���۸��� ���� �޸� DC �ʱ�ȭ
    CRect imageRect;
    m_imageCtrl.GetClientRect(&imageRect);
    CDC* pDC = m_imageCtrl.GetDC();
    if (pDC) {
        m_memDC.CreateCompatibleDC(pDC);
        m_memBitmap.CreateCompatibleBitmap(pDC, imageRect.Width(), imageRect.Height());
        m_oldBitmap = m_memDC.SelectObject(&m_memBitmap);

        // GDI+ ������ ǰ�� ����
        m_memDC.SetStretchBltMode(HALFTONE);
        SetBrushOrgEx(m_memDC.GetSafeHdc(), 0, 0, NULL);

        // �޸� DC�� ���������� �ʱ�ȭ
        CBrush blackBrush(RGB(0, 0, 0));
        m_memDC.FillRect(&imageRect, &blackBrush);

        // �ʱ� �޽��� ǥ��
        m_memDC.SetBkColor(RGB(0, 0, 0));
        m_memDC.SetTextColor(RGB(200, 200, 200));

        // ��Ʈ ����
        CFont font;
        font.CreatePointFont(100, _T("���� ���"));
        CFont* pOldFont = m_memDC.SelectObject(&font);

        CString strInit = _T("ī�޶� �������ּ���");
        m_memDC.DrawText(strInit, &imageRect, DT_CENTER | DT_VCENTER | DT_SINGLELINE);

        m_memDC.SelectObject(pOldFont);
        font.DeleteObject();

        m_imageCtrl.ReleaseDC(pDC);
    }

    // DIB ���� ���� (���� ���)
    m_dibBuffer.reserve(1920 * 1080 * 3); // �ִ� �ػ� ����ġ

    // Picture Control ���� �ٽ� �׸���
    m_imageCtrl.Invalidate();
    m_imageCtrl.UpdateWindow();

    // �ʱ� ���� ����
    m_bConnected = FALSE;
    UpdateConnectionStatus();
    EnableCameraControls(FALSE);

    // LED ����
    {
        // LED�� ũ�� ���̰� �۲� ����
        if (m_ledFont.GetSafeHandle() == nullptr) {
            m_ledFont.CreatePointFont(120, _T("Segoe UI Symbol")); // 12pt�� ����
            m_led.SetFont(&m_ledFont);
        }
        m_led.SetWindowText(_T("��")); // ���׶�� ����
        m_ledColor = RGB(220, 0, 0);  // OFF ����
        // üũ�ڽ��� OFF�� ����
        m_isTrackingOn = FALSE;
        CheckDlgButton(IDC_CHK_TRACK, BST_UNCHECKED);

        // �ʱ� LED �׸���
        if (CWnd* pLed = GetDlgItem(IDC_STATIC_LED)) pLed->Invalidate();
    }

    // �ڵ� ī�޶� �˻� (���û���)
    PostMessage(WM_COMMAND, MAKEWPARAM(IDC_BUTTON_SCAN_CAMERAS, BN_CLICKED), 0);

    m_bInitialized = TRUE;

    return TRUE;  // ��Ŀ���� ��Ʈ�ѿ� �������� ������ TRUE�� ��ȯ�մϴ�.
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
        CPaintDC dc(this); // �׸��⸦ ���� ����̽� ���ؽ�Ʈ�Դϴ�.

        SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

        // Ŭ���̾�Ʈ �簢������ �������� ����� ����ϴ�.
        int cxIcon = GetSystemMetrics(SM_CXICON);
        int cyIcon = GetSystemMetrics(SM_CYICON);
        CRect rect;
        GetClientRect(&rect);
        int x = (rect.Width() - cxIcon + 1) / 2;
        int y = (rect.Height() - cyIcon + 1) / 2;

        // �������� �׸��ϴ�.
        dc.DrawIcon(x, y, m_hIcon);
    }
    else
    {
        CDialogEx::OnPaint();

        // Picture Control ���� ���� �ٽ� �׸���
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
    // Picture Control ���� �����ϰ� ��� �����
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

        // �޸� DC�� ������ ȭ�鿡 ����
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

    SetDlgItemText(IDC_STATIC_SYSTEM_STATUS, _T("ī�޶� �˻� ��..."));

    // �ִ� 10�� ī�޶� �˻�
    m_cameraList.resize(10);
    int count = EnumerateCameras(m_cameraList.data(), 10);
    m_cameraList.resize(count);

    if (count == 0) {
        AfxMessageBox(_T("���� ������ ī�޶� ã�� �� �����ϴ�.\nī�޶� ��Ʈ��ũ�� ����Ǿ� �ִ��� Ȯ�����ּ���."));
        m_btnConnect.EnableWindow(FALSE);
        SetDlgItemText(IDC_STATIC_SYSTEM_STATUS, _T("ī�޶� ã�� �� ����"));
        return;
    }

    // ����Ʈ�ڽ��� �߰�
    for (const auto& cam : m_cameraList) {
        CString displayName(cam.displayName);
        m_cameraListBox.AddString(displayName);
    }

    // ù ��° �׸� ����
    if (count > 0) {
        m_cameraListBox.SetCurSel(0);
        OnLbnSelchangeListCameras();
    }

    CString msg;
    msg.Format(_T("%d���� ī�޶� ã�ҽ��ϴ�."), count);
    SetDlgItemText(IDC_STATIC_SYSTEM_STATUS, msg);
}

void CVisionTrackerUIDlg::OnBnClickedButtonConnectCamera()
{
    int sel = m_cameraListBox.GetCurSel();
    if (sel == LB_ERR) {
        AfxMessageBox(_T("ī�޶� �������ּ���."));
        return;
    }

    // ���� �õ�
    SetDlgItemText(IDC_STATIC_SYSTEM_STATUS, _T("ī�޶� ���� ��..."));
    BeginWaitCursor();

    if (ConnectCameraByIndex(sel)) {
        m_bConnected = TRUE;
        UpdateConnectionStatus();
        EnableCameraControls(TRUE);

        // ���� ������ �б�
        //GetGain(&m_f_gain);
        //GetBlackLevel(&m_i_blacklevel);
        //GetFrameRate(&m_f_fps);
        //GetExposureTime(&m_f_exposure);
        //GetGamma(&m_f_gamma);
        UpdateData(FALSE);

        SetDlgItemText(IDC_STATIC_SYSTEM_STATUS, _T("ī�޶� ���� ����"));

        // ����� ī�޶� ���� ǥ��
        CameraInfo info;
        if (GetConnectedCameraInfo(&info)) {
            CString infoStr;
            infoStr.Format(_T("����� ī�޶�: %s"), CString(info.modelName));
            SetDlgItemText(IDC_STATIC_SYSTEM_STATUS, infoStr);
        }
    }
    else {
        CString errMsg(GetLastErrorMessage());
        CString fullMsg;
        fullMsg.Format(_T("ī�޶� ���� ����: %s"), errMsg);
        AfxMessageBox(fullMsg);
        SetDlgItemText(IDC_STATIC_SYSTEM_STATUS, _T("ī�޶� ���� ����"));
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
        SetDlgItemText(IDC_STATIC_SYSTEM_STATUS, _T("ī�޶� ���� ������"));

        // ȭ�� �ʱ�ȭ
        CRect imageRect;
        m_imageCtrl.GetClientRect(&imageRect);
        CBrush blackBrush(RGB(0, 0, 0));
        m_memDC.FillRect(&imageRect, &blackBrush);
        m_memDC.SetBkColor(RGB(0, 0, 0));
        m_memDC.SetTextColor(RGB(200, 200, 200));
        CString strInit = _T("ī�޶� �������ּ���");
        m_memDC.DrawText(strInit, &imageRect, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
        m_imageCtrl.Invalidate();
    }
}

void CVisionTrackerUIDlg::OnLbnSelchangeListCameras()
{
    int sel = m_cameraListBox.GetCurSel();
    if (sel != LB_ERR && sel < (int)m_cameraList.size()) {
        // ���õ� ī�޶� ���� ǥ��
        const CameraInfo& cam = m_cameraList[sel];
        CString info;
        if (cam.deviceType == 0) { // GigE
            info.Format(_T("��: %s\n�ø���: %s\nIP: %s"),
                CString(cam.modelName),
                CString(cam.serialNumber),
                CString(cam.ipAddress));
        }
        else { // USB
            info.Format(_T("��: %s\n�ø���: %s\nŸ��: USB"),
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
        m_connectionStatus.SetWindowText(_T("����: �����"));
        m_btnConnect.EnableWindow(FALSE);
        m_btnDisconnect.EnableWindow(TRUE);
        GetDlgItem(IDC_BUTTON1)->EnableWindow(TRUE);  // Start ��ư

        // LED ���� ������Ʈ (����� = �Ķ���)
        if (CWnd* pStatus = GetDlgItem(IDC_STATIC_CONNECTION_STATUS)) {
            pStatus->SetWindowText(_T("�� �����"));
        }
    }
    else {
        m_connectionStatus.SetWindowText(_T("����: ������� ����"));
        m_btnConnect.EnableWindow(m_cameraListBox.GetCurSel() != LB_ERR);
        m_btnDisconnect.EnableWindow(FALSE);
        GetDlgItem(IDC_BUTTON1)->EnableWindow(FALSE);  // Start ��ư
        GetDlgItem(IDC_BUTTON2)->EnableWindow(FALSE);  // Stop ��ư

        // LED ���� ������Ʈ (�̿��� = ������)
        if (CWnd* pStatus = GetDlgItem(IDC_STATIC_CONNECTION_STATUS)) {
            pStatus->SetWindowText(_T("�� �̿���"));
        }
    }
}

void CVisionTrackerUIDlg::EnableCameraControls(BOOL enable)
{
    // ī�޶� ���� ���� ��Ʈ�� Ȱ��ȭ/��Ȱ��ȭ
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
    // Start ��ư - ���� Ȯ�� �߰�
    if (!m_bConnected || !IsConnected()) {
        AfxMessageBox(_T("ī�޶� ������� �ʾҽ��ϴ�.\n���� ī�޶� �������ּ���."));
        return;
    }

    // ���� ��ư - ĸó ���� ��ȯ �� ����
    UpdateData(TRUE);

    // ĸó ���� ��������
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

    // DLL�� ���� ������ ����
    SetFrameRate(fps);
    SetExposureTime(exposure);
    SetGamma(gamma);
    SetGain(gain);
    SetBlackLevel((int)blacklevel);
    SetROI(cropX, cropY, cropW, cropH);

    // �� Ž�� ���� ����
    {
        WhiteBallDetectionConfig config;
        config.roi = cv::Rect(0, 0, 640, 480);        // �� Ž�� ���� ROI ����
        config.thresholdMode = ThresholdMode::Fixed;   // Fixed, Adaptive, Otsu
        config.thresholdValue = 90;
        config.thresholdType = cv::THRESH_BINARY;      // 0~4
        config.adaptiveMethod = AdaptiveMethod::Gaussian;
        config.blockSize = 7;                          // Ȧ�� 3 �̻�
        config.C = 1.0;                                // ��տ��� ���� ��
        config.minRadius = 2.5f;
        config.maxRadius = 10.0f;
        config.minCircularity = 0.8f;
        config.maxCircularity = 1.2f;
        config.useTracking = false;  // ���� ������ ���� ��� ����

        SetWhiteBallDetectionConfig(config);
    }

    StartGrab();                    // DLL ������ ���� ����
    SetTimer(1, 33, nullptr);       // �� 33ms(30fps) �ֱ�� Ÿ�̸� ȣ��

    GetDlgItem(IDC_BUTTON2)->EnableWindow(TRUE);  // Stop ��ư Ȱ��ȭ
    SetDlgItemText(IDC_STATIC_SYSTEM_STATUS, _T("ĸó ���۵�"));
}

void CVisionTrackerUIDlg::OnBnClickedButton2()
{
    // ���� ��ư
    StopGrab();    // DLL ������ ���� ����
    KillTimer(1);  // Ÿ�̸� ����

    GetDlgItem(IDC_BUTTON2)->EnableWindow(FALSE);  // Stop ��ư ��Ȱ��ȭ
    SetDlgItemText(IDC_STATIC_SYSTEM_STATUS, _T("ĸó ������"));
}

void CVisionTrackerUIDlg::ShowMatToStatic(const cv::Mat& mat, CStatic& ctrl)
{
    CRect rect;
    ctrl.GetClientRect(&rect);

    // ��Ʈ���� ��ȿ���� ������ ����
    if (!::IsWindow(ctrl.GetSafeHwnd()) || rect.IsRectEmpty()) {
        return;
    }

    // mat�� ��������� ���� ȭ�� ǥ��
    if (mat.empty()) {
        CBrush blackBrush(RGB(0, 0, 0));
        m_memDC.FillRect(&rect, &blackBrush);
        ctrl.Invalidate();
        return;
    }

    // ����� ���� ���������� ä��
    CBrush blackBrush(RGB(0, 0, 0));
    m_memDC.FillRect(&rect, &blackBrush);

    // ���� ���� �����ϸ鼭 ũ�� ���
    double srcRatio = (double)mat.cols / mat.rows;
    double dstRatio = (double)rect.Width() / rect.Height();

    int destWidth, destHeight, destX, destY;

    if (srcRatio > dstRatio) {
        // ������ �� ���� - �ʺ� ���߱�
        destWidth = rect.Width();
        destHeight = (int)(rect.Width() / srcRatio);
        destX = 0;
        destY = (rect.Height() - destHeight) / 2;
    }
    else {
        // ������ �� ���� - ���̿� ���߱�
        destHeight = rect.Height();
        destWidth = (int)(rect.Height() * srcRatio);
        destX = (rect.Width() - destWidth) / 2;
        destY = 0;
    }

    // ���� ũ�� ���߱�
    cv::Mat resized;
    cv::resize(mat, resized, cv::Size(destWidth, destHeight), 0, 0, cv::INTER_LINEAR);

    // BGR/GRAY to RGB ��ȯ �� ���� �޸� ����
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

    // Mat�� ���� �޸𸮷� ����� (�߿�!)
    if (!rgb.isContinuous()) {
        rgb = rgb.clone();
    }

    // Windows DIB ��ĵ���� ������ ���� �е� ���
    int bitsPerPixel = 24;
    int bytesPerPixel = bitsPerPixel / 8;
    int widthBytes = rgb.cols * bytesPerPixel;
    int stride = ((widthBytes + 3) / 4) * 4; // 4����Ʈ ��� ����
    int padding = stride - widthBytes;

    // DIB�� ���� ���� (�е� ����)
    std::vector<BYTE> dibBuffer;
    if (padding > 0) {
        dibBuffer.resize(stride * rgb.rows);

        // �� �ึ�� ������ ���� (�е� �߰�)
        for (int y = 0; y < rgb.rows; y++) {
            const BYTE* srcRow = rgb.ptr<BYTE>(y);
            BYTE* dstRow = &dibBuffer[y * stride];

            // �ȼ� ������ ����
            memcpy(dstRow, srcRow, widthBytes);

            // �е� ���� 0���� �ʱ�ȭ
            if (padding > 0) {
                memset(dstRow + widthBytes, 0, padding);
            }
        }
    }

    // BITMAPINFO ����ü ����
    BITMAPINFOHEADER bmiHeader = { 0 };
    bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bmiHeader.biWidth = rgb.cols;
    bmiHeader.biHeight = -rgb.rows; // ������ �����Ͽ� ���� ���� ����
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

    // ������ ǰ�� ����
    SetStretchBltMode(m_memDC.GetSafeHdc(), HALFTONE);
    SetBrushOrgEx(m_memDC.GetSafeHdc(), 0, 0, NULL);

    // DIB �׸���
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

    // Picture Control�� �ٽ� �׸����� ��û
    ctrl.Invalidate();
    ctrl.UpdateWindow();
}

void CVisionTrackerUIDlg::OnTimer(UINT_PTR nIDEvent)
{
    if (nIDEvent == 1)
    {
        // ������ ���� ��������
        int w = 0, h = 0, ch = 0;
        if (!GetLatestFrameInfo(&w, &h, &ch)) {
            OutputDebugString(_T("GetLatestFrameInfo ����\n"));
            CDialogEx::OnTimer(nIDEvent);
            return;
        }

        std::vector<unsigned char> buffer(w * h * ch);
        if (!GetLatestFrame(buffer.data(), (int)buffer.size(), &w, &h, &ch)) {
            OutputDebugString(_T("GetLatestFrame ����\n"));
            CDialogEx::OnTimer(nIDEvent);
            return;
        }

        // ī�޶� ������ �� Mat
        cv::Mat img(h, w, (ch == 1 ? CV_8UC1 : CV_8UC3), buffer.data());
        cv::Mat disp;
        if (ch == 1) {
            // ���� �׷��� �� BGR ��ȯ
            cv::cvtColor(img, disp, cv::COLOR_GRAY2BGR);
        }
        else {
            // 3ä���̸� �״�� ���
            disp = img.clone(); // clone �߰�
        }

        // ������ Ȱ��ȭ�� ��� �� ��ġ ǥ��
        if (m_isTrackingOn) {
            float x, y, radius;
            bool found;
            GetCurrentBallPosition(&x, &y, &radius, &found);

            if (found) {
                // ���� ��ġ�� �ʷϻ� �� �׸���
                cv::circle(disp, cv::Point((int)x, (int)y), (int)radius, cv::Scalar(0, 255, 0), 2);
                cv::circle(disp, cv::Point((int)x, (int)y), 2, cv::Scalar(0, 0, 255), -1);

                // ��ǥ ���� ǥ��
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

        // �������̵� ���� ��� - ���� ���۸� ���
        ShowMatToStatic(disp, m_imageCtrl);

        // FPS/��� ǥ��
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

    // ���� ���۸� ���ҽ� ����
    if (m_memDC.GetSafeHdc()) {
        m_memDC.SelectObject(m_oldBitmap);
        m_memBitmap.DeleteObject();
        m_memDC.DeleteDC();
    }

    // ī�޶� ����
    if (m_bConnected) {
        StopGrab();
        DisconnectCamera();
    }
}

void CVisionTrackerUIDlg::OnOK()
{
    // �ƹ� �۾��� ���� �������ν� Enter Ű�� ����Ǵ� ������ ����
}

HBRUSH CVisionTrackerUIDlg::OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor)
{
    HBRUSH hbr = CDialogEx::OnCtlColor(pDC, pWnd, nCtlColor);

    if (pWnd && pWnd->GetDlgCtrlID() == IDC_STATIC_LED) {
        // ���ڻ��� �ٲ㼭 �� �� ����/�ʷ����� ���̰�
        pDC->SetTextColor(m_ledColor);

        // ��� ���� ó��
        pDC->SetBkMode(TRANSPARENT);
        return (HBRUSH)GetStockObject(HOLLOW_BRUSH);
    }
    return hbr;
}

void CVisionTrackerUIDlg::SetLedOn(BOOL on)
{
    m_isTrackingOn = on;
    m_ledColor = on ? RGB(0, 180, 0) : RGB(220, 0, 0); // �ʷ�/����
    if (CWnd* pLed = GetDlgItem(IDC_STATIC_LED)) pLed->Invalidate();

    // LED ���� �ؽ�Ʈ ������Ʈ
    SetDlgItemText(IDC_STATIC_LED_STATUS, on ? _T("ON") : _T("OFF"));
}

void CVisionTrackerUIDlg::OnClickedChkTrack()
{
    const BOOL wantOn = (IsDlgButtonChecked(IDC_CHK_TRACK) == BST_CHECKED);

    // LED/���� �÷��� ����
    SetLedOn(wantOn);

    if (wantOn) {
        StartTracking();
        SetDlgItemText(IDC_STATIC_SYSTEM_STATUS, _T("���� Ȱ��ȭ"));
    }
    else {
        StopTracking();
        SetDlgItemText(IDC_STATIC_SYSTEM_STATUS, _T("���� ��Ȱ��ȭ"));
    }
}
