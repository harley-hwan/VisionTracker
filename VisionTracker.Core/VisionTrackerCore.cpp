#include "pch.h"
#include "IVisionTracker.h"
#include "Logger.h"
#include <io.h>
#include <fcntl.h>
#include <iostream>

#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

extern "C" {
#include "MvCameraControl.h"
}

using namespace cv;

// 전역 설정 초기화
WhiteBallDetectionConfig g_whiteBallConfig = {
    cv::Rect(0, 0, 0, 0),           // roi는 0이면 full image 처리
    ThresholdMode::Fixed,           // 기본 모드 Fixed
    200,                            // threshold value
    cv::THRESH_BINARY,              // threshold type
    AdaptiveMethod::Gaussian,       // adaptiveMethod (미사용 시 무시됨)
    7,                              // blockSize
    2.0,                            // C
    2.0f, 10.0f,                    // min/max radius
    0.7f, 1.2f,                     // min/max circularity
    true                            // useTracking 여부
};

// 내부 상태 보관용 정적 변수들
namespace {
    // 카메라 핸들
    void* g_handle = nullptr;

    // 카메라 열거 관련
    MV_CC_DEVICE_INFO_LIST g_deviceList;
    bool g_isConnected = false;
    CameraInfo g_connectedCameraInfo;

    // 최신 프레임 (스레드 보호 필요)
    cv::Mat g_latestFrame;
    std::mutex g_frameMutex;

    // Grab 상태 및 FPS 계산용 변수
    std::atomic<bool> g_running(false);
    std::chrono::steady_clock::time_point g_lastTime;
    int g_frameCount = 0;
    double g_lastFps = 0.0;

    // 볼 탐색 활성화 여부
    std::atomic<bool> g_trackingActive(false);

    // ROI 처리
    cv::Rect g_roiRect;
    bool g_useRoi = false;

    // 녹화 제어 변수
    bool g_recording = false;
    std::string g_outputFolder;
    int g_frameCounter = 0;

    // 타임스탬프
    double g_lastTimestamp = 0.0;

    // 선택된 카메라 IP
    std::string g_selectedIp = "";

    // 에러 메시지 전달용
    std::string g_lastError = "";

    // 현재 감지된 공 정보
    cv::Point2f g_currentBallCenter(-1, -1);
    float g_currentBallRadius = 0.0f;
    bool g_currentBallFound = false;
    std::mutex g_ballInfoMutex;

    // IP 변환 헬퍼 함수
    std::string ConvertIp(uint32_t ip) {
        char buf[32] = {};
        sprintf_s(buf, "%d.%d.%d.%d",
            (ip >> 24) & 0xff,
            (ip >> 16) & 0xff,
            (ip >> 8) & 0xff,
            ip & 0xff);
        return std::string(buf);
    }
}

// 이미지 수신 콜백 함수
extern "C" void __stdcall ImageCallback(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pInfo, void* pUser)
{
    if (!pInfo) return;
    if (pInfo->enPixelType != PixelType_Gvsp_BGR8_Packed && pInfo->enPixelType != PixelType_Gvsp_Mono8) return;

    cv::Mat frame;
    {
        if (pInfo->enPixelType == PixelType_Gvsp_Mono8) {
            frame = cv::Mat(pInfo->nHeight, pInfo->nWidth, CV_8UC1, pData).clone();
        }
        else if (pInfo->enPixelType == PixelType_Gvsp_BGR8_Packed) {
            frame = cv::Mat(pInfo->nHeight, pInfo->nWidth, CV_8UC3, pData).clone();
        }

        // ROI 적용
        if (g_useRoi && g_roiRect.area() > 0 &&
            g_roiRect.x >= 0 && g_roiRect.y >= 0 &&
            g_roiRect.x + g_roiRect.width <= frame.cols &&
            g_roiRect.y + g_roiRect.height <= frame.rows) {
            frame = frame(g_roiRect);
        }

        // 프레임 저장
        {
            std::lock_guard<std::mutex> lock(g_frameMutex);
            g_latestFrame = frame.clone();
            if (g_recording) {
                char filename[256];
                sprintf_s(filename, "%s/frame_%06d.png", g_outputFolder.c_str(), g_frameCounter++);
                cv::imwrite(filename, g_latestFrame);
            }

            // 타임스탬프 갱신 (초 단위)
            uint64_t timestampRaw = (static_cast<uint64_t>(pInfo->nDevTimeStampHigh) << 32) | pInfo->nDevTimeStampLow;
            g_lastTimestamp = timestampRaw / 1e9;  // 초 단위 변환
        }
    }

    // FPS 계산
    g_frameCount++;
    auto now = std::chrono::steady_clock::now();
    double elapsed = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(now - g_lastTime).count());
    if (elapsed >= 1000.0) {
        g_lastFps = static_cast<double>(g_frameCount) * 1000.0 / elapsed;
        g_frameCount = 0;
        g_lastTime = now;
    }

    // 볼 감지가 활성화된 경우
    if (g_trackingActive) {
        int tmpX = 0, tmpY = 0, tmpR = 0, tmpPts = 0;
        bool found = GetWhiteBallInfo(&tmpX, &tmpY, &tmpR, &tmpPts);

        // 프레임 카운터
        static int frameCounter = 0;
        frameCounter++;

        std::lock_guard<std::mutex> lock(g_ballInfoMutex);
        if (found) {
            g_currentBallCenter = cv::Point2f((float)tmpX, (float)tmpY);
            g_currentBallRadius = (float)tmpR;
            g_currentBallFound = true;

            // 탐지 성공 로그
            char logMsg[256];
            sprintf_s(logMsg, "[Frame %06d] Ball FOUND - Position: (%.1f, %.1f), Radius: %.1f",
                frameCounter, g_currentBallCenter.x, g_currentBallCenter.y, g_currentBallRadius);
            LOG_INFO(std::string(logMsg));
        }
        else {
            g_currentBallFound = false;

            // 탐지 실패 로그
            char logMsg[256];
            sprintf_s(logMsg, "[Frame %06d] Ball NOT FOUND", frameCounter);
            LOG_DEBUG(std::string(logMsg));
        }
    }
}

// API 구현부
extern "C" VISIONTRACKER_API const char* GetLastErrorMessage() {
    return g_lastError.c_str();
}

// 카메라 열거 함수
extern "C" VISIONTRACKER_API int EnumerateCameras(CameraInfo* cameras, int maxCount)
{
    memset(&g_deviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &g_deviceList);
    if (nRet != MV_OK) {
        g_lastError = "디바이스 열거 실패";
        return 0;
    }

    int count = std::min((int)g_deviceList.nDeviceNum, maxCount);

    for (int i = 0; i < count; ++i) {
        MV_CC_DEVICE_INFO* pDeviceInfo = g_deviceList.pDeviceInfo[i];
        CameraInfo& cam = cameras[i];
        memset(&cam, 0, sizeof(CameraInfo));

        if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
            MV_GIGE_DEVICE_INFO& stGigEInfo = pDeviceInfo->SpecialInfo.stGigEInfo;

            sprintf_s(cam.modelName, "%s", stGigEInfo.chModelName);
            sprintf_s(cam.serialNumber, "%s", stGigEInfo.chSerialNumber);
            sprintf_s(cam.ipAddress, "%d.%d.%d.%d",
                (stGigEInfo.nCurrentIp >> 24) & 0xff,
                (stGigEInfo.nCurrentIp >> 16) & 0xff,
                (stGigEInfo.nCurrentIp >> 8) & 0xff,
                stGigEInfo.nCurrentIp & 0xff);
            sprintf_s(cam.displayName, "%s [%s] - %s",
                cam.modelName, cam.serialNumber, cam.ipAddress);
            cam.deviceType = 0;
        }
        else if (pDeviceInfo->nTLayerType == MV_USB_DEVICE) {
            MV_USB3_DEVICE_INFO& stUSBInfo = pDeviceInfo->SpecialInfo.stUsb3VInfo;

            sprintf_s(cam.modelName, "%s", stUSBInfo.chModelName);
            sprintf_s(cam.serialNumber, "%s", stUSBInfo.chSerialNumber);
            sprintf_s(cam.displayName, "%s [%s] - USB",
                cam.modelName, cam.serialNumber);
            cam.deviceType = 1;
        }
    }

    return count;
}

// 인덱스로 카메라 연결
extern "C" VISIONTRACKER_API bool ConnectCameraByIndex(int index)
{
    if (g_isConnected) {
        DisconnectCamera();
    }

    if (index < 0 || index >= (int)g_deviceList.nDeviceNum) {
        g_lastError = "잘못된 카메라 인덱스";
        return false;
    }

    MV_CC_DEVICE_INFO* pDeviceInfo = g_deviceList.pDeviceInfo[index];

    int nRet = MV_CC_CreateHandle(&g_handle, pDeviceInfo);
    if (nRet != MV_OK) {
        g_lastError = "핸들 생성 실패";
        return false;
    }

    nRet = MV_CC_OpenDevice(g_handle, MV_ACCESS_Exclusive, 0);
    if (nRet != MV_OK) {
        // 재시도 로직
        int timeoutSeconds = 30;
        const int intervalMs = 2000;
        int elapsed = 0;

        while (elapsed < timeoutSeconds * 1000) {
            MV_CC_DestroyHandle(g_handle);
            g_handle = nullptr;

            nRet = MV_CC_CreateHandle(&g_handle, pDeviceInfo);
            if (nRet != MV_OK) continue;

            nRet = MV_CC_OpenDevice(g_handle, MV_ACCESS_Exclusive, 0);
            if (nRet == MV_OK) break;

            std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs));
            elapsed += intervalMs;
        }

        if (nRet != MV_OK) {
            MV_CC_DestroyHandle(g_handle);
            g_handle = nullptr;
            g_lastError = "디바이스 열기 실패";
            return false;
        }
    }

    // 콜백 등록
    nRet = MV_CC_RegisterImageCallBackEx(g_handle, ImageCallback, nullptr);
    if (nRet != MV_OK) {
        MV_CC_CloseDevice(g_handle);
        MV_CC_DestroyHandle(g_handle);
        g_handle = nullptr;
        g_lastError = "콜백 등록 실패";
        return false;
    }

    // 기본 설정 적용
    MV_CC_SetEnumValue(g_handle, "GainAuto", 0);
    MV_CC_SetEnumValue(g_handle, "ExposureAuto", 0);

    if (IsNodeSupported("GammaEnable")) {
        EnableGamma(true);
    }

    if (IsNodeSupported("BlackLevelEnable")) {
        EnableBlackLevel(true);
    }

    g_isConnected = true;

    // 연결된 카메라 정보 저장
    memset(&g_connectedCameraInfo, 0, sizeof(CameraInfo));
    if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
        MV_GIGE_DEVICE_INFO& info = pDeviceInfo->SpecialInfo.stGigEInfo;
        sprintf_s(g_connectedCameraInfo.modelName, "%s", info.chModelName);
        sprintf_s(g_connectedCameraInfo.serialNumber, "%s", info.chSerialNumber);
        sprintf_s(g_connectedCameraInfo.ipAddress, "%s", ConvertIp(info.nCurrentIp).c_str());
        sprintf_s(g_connectedCameraInfo.displayName, "%s [%s] - %s",
            g_connectedCameraInfo.modelName,
            g_connectedCameraInfo.serialNumber,
            g_connectedCameraInfo.ipAddress);
        g_connectedCameraInfo.deviceType = 0;
    }

    g_lastTime = std::chrono::steady_clock::now();
    g_frameCount = 0;
    g_lastFps = 0.0;

    return true;
}

// 카메라 연결 해제
extern "C" VISIONTRACKER_API bool DisconnectCamera()
{
    if (!g_handle) return false;

    StopGrab();
    MV_CC_CloseDevice(g_handle);
    MV_CC_DestroyHandle(g_handle);
    g_handle = nullptr;
    g_isConnected = false;

    return true;
}

// 연결 상태 확인
extern "C" VISIONTRACKER_API bool IsConnected()
{
    return g_isConnected && g_handle != nullptr;
}

// 연결된 카메라 정보 조회
extern "C" VISIONTRACKER_API bool GetConnectedCameraInfo(CameraInfo* info)
{
    if (!g_isConnected || !info) return false;
    *info = g_connectedCameraInfo;
    return true;
}

extern "C" VISIONTRACKER_API bool IsNodeSupported(const char* nodeName)
{
    if (!g_handle) return false;

    bool dummyValue = false;
    int nRet = MV_CC_GetBoolValue(g_handle, nodeName, &dummyValue);

    return (nRet == MV_OK);
}

extern "C" VISIONTRACKER_API bool SetTriggerMode(bool enable)
{
    if (!g_handle) {
        g_lastError = "카메라 핸들이 초기화되지 않았습니다.";
        return false;
    }

    MV_CC_StopGrabbing(g_handle);

    int nRet = MV_CC_SetEnumValue(g_handle, "AcquisitionMode", 2);
    if (nRet != MV_OK) {
        printf("AcquisitionMode 설정 실패: %d\n", nRet);
        return false;
    }

    nRet = MV_CC_SetEnumValue(g_handle, "TriggerMode", 0);
    if (nRet != MV_OK) {
        printf("TriggerMode 사전 설정 실패: %d\n", nRet);
        return false;
    }

    nRet = MV_CC_SetEnumValue(g_handle, "TriggerSelector", 0);
    if (nRet != MV_OK) {
        printf("TriggerSelector(FrameStart) 설정 실패 %d \n", nRet);
        return false;
    }

    MV_CC_SetEnumValue(g_handle, "TriggerSource", 0);
    MV_CC_SetEnumValue(g_handle, "TriggerActivation", 0);
    MV_CC_SetEnumValue(g_handle, "LineSelector", 0);
    MV_CC_SetEnumValue(g_handle, "LineMode", 0);

    int modeValue = enable ? 1 : 0;
    nRet = MV_CC_SetEnumValue(g_handle, "TriggerMode", modeValue);
    if (nRet != MV_OK) {
        printf("TriggerMode 설정 실패\n");
        return false;
    }

    printf("Trigger 설정 완료 (모드: %s)\n", enable ? "Slave" : "Master");

    MVCC_FLOATVALUE stFps = {};
    nRet = MV_CC_GetFloatValue(g_handle, "ResultingFrameRate", &stFps);
    if (nRet == MV_OK) {
        printf("현재 적용된 FPS: %.2f\n", stFps.fCurValue);
    }

    return true;
}

extern "C" VISIONTRACKER_API void StartGrab()
{
    if (!g_handle) return;
    g_running = true;
    MV_CC_StartGrabbing(g_handle);
    g_lastTime = std::chrono::steady_clock::now();
    g_frameCount = 0;
    g_lastFps = 0.0;
}

extern "C" VISIONTRACKER_API void StopGrab()
{
    if (!g_handle) return;
    g_running = false;
    MV_CC_StopGrabbing(g_handle);
}

extern "C" VISIONTRACKER_API bool StartRecording(const char* folderPath)
{
    g_outputFolder = folderPath;
    g_frameCounter = 0;
    g_recording = true;
    return true;
}

extern "C" VISIONTRACKER_API void StopRecording()
{
    g_recording = false;
}

extern "C" VISIONTRACKER_API bool GetLatestFrameInfo(int* width, int* height, int* channels)
{
    std::lock_guard<std::mutex> lock(g_frameMutex);
    if (g_latestFrame.empty()) return false;

    *width = g_latestFrame.cols;
    *height = g_latestFrame.rows;
    *channels = g_latestFrame.channels();
    return true;
}

extern "C" VISIONTRACKER_API bool GetLatestFrame(
    unsigned char* buffer, int bufferSize,
    int* width, int* height, int* channels)
{
    std::lock_guard<std::mutex> lock(g_frameMutex);
    if (g_latestFrame.empty()) return false;

    int needed = static_cast<int>(g_latestFrame.total() * g_latestFrame.elemSize());
    if (bufferSize < needed) return false;

    memcpy(buffer, g_latestFrame.data, needed);
    *width = g_latestFrame.cols;
    *height = g_latestFrame.rows;
    *channels = g_latestFrame.channels();
    return true;
}

extern "C" VISIONTRACKER_API double GetCurrentBrightness()
{
    std::lock_guard<std::mutex> lock(g_frameMutex);
    if (g_latestFrame.empty()) return -1.0;

    if (g_latestFrame.channels() == 1)
        return cv::mean(g_latestFrame)[0];
    else if (g_latestFrame.channels() == 3) {
        cv::Mat gray;
        cv::cvtColor(g_latestFrame, gray, cv::COLOR_BGR2GRAY);
        return cv::mean(gray)[0];
    }
    return -1.0;
}

extern "C" VISIONTRACKER_API double GetCurrentFps()
{
    return g_lastFps;
}

extern "C" VISIONTRACKER_API bool SetFrameRate(float fps)
{
    if (!g_handle) {
        printf("FPS 설정 실패\n");
        return false;
    }
    float temp_fps;
    printf("기존 FPS 조건 : ");
    GetFrameRate(&temp_fps);
    printf("%.2f로 적용 \n", temp_fps);
    printf("변경할 FPS : %f \n", fps);
    return MV_CC_SetFloatValue(g_handle, "AcquisitionFrameRate", fps) == MV_OK;
}

extern "C" VISIONTRACKER_API bool GetFrameRate(float* fps)
{
    if (!g_handle || !fps) return false;
    MVCC_FLOATVALUE val;
    if (MV_CC_GetFloatValue(g_handle, "AcquisitionFrameRate", &val) != MV_OK) return false;
    *fps = val.fCurValue;
    return true;
}

extern "C" VISIONTRACKER_API bool SetExposureTime(float exposureTime)
{
    if (!g_handle) return false;
    return MV_CC_SetFloatValue(g_handle, "ExposureTime", exposureTime) == MV_OK;
}

extern "C" VISIONTRACKER_API bool GetExposureTime(float* exposureTime)
{
    if (!g_handle || !exposureTime) return false;
    MVCC_FLOATVALUE val;
    if (MV_CC_GetFloatValue(g_handle, "ExposureTime", &val) != MV_OK) return false;
    *exposureTime = val.fCurValue;
    return true;
}

extern "C" VISIONTRACKER_API bool EnableGamma(bool enable)
{
    if (!g_handle) return false;

    int nRet = MV_CC_SetBoolValue(g_handle, "GammaEnable", enable);
    if (nRet != MV_OK) {
        printf("GammaEnable 설정 실패\n");
        g_lastError = "GammaEnable 설정 실패";
        return false;
    }

    return true;
}

extern "C" VISIONTRACKER_API bool SetGamma(float gammaValue)
{
    if (!g_handle) return false;
    return MV_CC_SetFloatValue(g_handle, "Gamma", gammaValue) == MV_OK;
}

extern "C" VISIONTRACKER_API bool GetGamma(float* gammaValue)
{
    if (!g_handle || !gammaValue) return false;
    MVCC_FLOATVALUE val;
    if (MV_CC_GetFloatValue(g_handle, "Gamma", &val) != MV_OK) return false;
    *gammaValue = val.fCurValue;
    return true;
}

extern "C" VISIONTRACKER_API bool SetROI(int x, int y, int width, int height)
{
    std::lock_guard<std::mutex> lock(g_frameMutex);
    g_roiRect = cv::Rect(x, y, width, height);
    g_useRoi = true;
    return true;
}

extern "C" VISIONTRACKER_API bool GetWhiteBallInfo(int* centerX, int* centerY, int* radius, int* pointCount)
{
    if (!centerX || !centerY || !radius || !pointCount)
        return false;

    std::lock_guard<std::mutex> lock(g_frameMutex);
    if (g_latestFrame.empty())
        return false;

    cv::Mat gray, bin;
    if (g_latestFrame.channels() == 3)
        cv::cvtColor(g_latestFrame, gray, cv::COLOR_BGR2GRAY);
    else
        gray = g_latestFrame;

    // 볼 검출용 ROI 설정 - 수정된 부분
    cv::Rect roiRect = g_whiteBallConfig.roi;

    // ROI가 설정되지 않았거나 (0,0,0,0)이면 전체 이미지 사용
    if (roiRect.width == 0 || roiRect.height == 0) {
        roiRect = cv::Rect(0, 0, gray.cols, gray.rows);
    }

    // ROI가 현재 이미지 범위를 벗어나는지 검사
    if (roiRect.x < 0) roiRect.x = 0;
    if (roiRect.y < 0) roiRect.y = 0;
    if (roiRect.x + roiRect.width > gray.cols)
        roiRect.width = gray.cols - roiRect.x;
    if (roiRect.y + roiRect.height > gray.rows)
        roiRect.height = gray.rows - roiRect.y;

    // ROI가 유효한지 최종 확인
    if (roiRect.width <= 0 || roiRect.height <= 0 ||
        roiRect.x >= gray.cols || roiRect.y >= gray.rows) {
        return false;
    }

    cv::Mat roi = gray(roiRect);
    cv::Mat work = roi.clone();

    switch (g_whiteBallConfig.thresholdMode)
    {
    case ThresholdMode::Fixed:
        cv::threshold(work, bin, g_whiteBallConfig.thresholdValue, 255, g_whiteBallConfig.thresholdType);
        break;

    case ThresholdMode::Adaptive:
        cv::adaptiveThreshold(work, bin, 255,
            static_cast<int>(g_whiteBallConfig.adaptiveMethod),
            g_whiteBallConfig.thresholdType,
            g_whiteBallConfig.blockSize,
            g_whiteBallConfig.C);
        break;

    case ThresholdMode::Otsu:
        cv::threshold(work, bin, 0, 255, g_whiteBallConfig.thresholdType | cv::THRESH_OTSU);
        break;

    case ThresholdMode::AdaptiveOtsu:
        break;
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    const float minRadius = g_whiteBallConfig.minRadius;
    const float maxRadius = g_whiteBallConfig.maxRadius;
    const float minCircularity = g_whiteBallConfig.minCircularity;
    const float maxCircularity = g_whiteBallConfig.maxCircularity;

    int bestIdx = -1;
    float bestCircularity = 0;
    cv::Point2f bestCenter;
    float bestRadius = 0;
    int bestPoints = 0;

    for (size_t i = 0; i < contours.size(); ++i)
    {
        float radius;
        cv::Point2f center;
        cv::minEnclosingCircle(contours[i], center, radius);

        if (radius < minRadius || radius > maxRadius)
            continue;

        double area = cv::contourArea(contours[i]);
        double perimeter = cv::arcLength(contours[i], true);
        if (perimeter <= 0)
            continue;

        double circularity = 4 * CV_PI * area / (perimeter * perimeter);
        if (circularity < minCircularity || circularity > maxCircularity)
            continue;

        // ROI 좌표를 원본 이미지 좌표로 변환
        center.x += roiRect.x;
        center.y += roiRect.y;

        // 가장 원형에 가까운 것을 선택
        if (circularity > bestCircularity) {
            bestIdx = (int)i;
            bestCircularity = (float)circularity;
            bestCenter = center;
            bestRadius = radius;
            bestPoints = (int)contours[i].size();
        }
    }

    if (bestIdx == -1) {
        return false;
    }

    *centerX = static_cast<int>(bestCenter.x);
    *centerY = static_cast<int>(bestCenter.y);
    *radius = static_cast<int>(bestRadius);
    *pointCount = bestPoints;

    return true;
}

extern "C" VISIONTRACKER_API void SetWhiteBallDetectionConfig(const WhiteBallDetectionConfig& config)
{
    std::lock_guard<std::mutex> lock(g_frameMutex);
    g_whiteBallConfig = config;
}

extern "C" VISIONTRACKER_API WhiteBallDetectionConfig GetWhiteBallDetectionConfig()
{
    std::lock_guard<std::mutex> lock(g_frameMutex);
    return g_whiteBallConfig;
}

extern "C" VISIONTRACKER_API bool SetGain(float gain)
{
    if (!g_handle) return false;
    return MV_CC_SetFloatValue(g_handle, "Gain", gain) == MV_OK;
}

extern "C" VISIONTRACKER_API bool GetGain(float* gain)
{
    if (!g_handle || !gain) return false;
    MVCC_FLOATVALUE val;
    if (MV_CC_GetFloatValue(g_handle, "Gain", &val) != MV_OK) return false;
    *gain = val.fCurValue;
    return true;
}

extern "C" VISIONTRACKER_API bool EnableBlackLevel(bool enable)
{
    if (!g_handle) return false;

    int nRet = MV_CC_SetBoolValue(g_handle, "BlackLevelEnable", enable);
    if (nRet != MV_OK) {
        printf("BlackLevelEnable 설정 실패\n");
        g_lastError = "BlackLevelEnable 설정 실패";
        return false;
    }

    return true;
}

extern "C" VISIONTRACKER_API bool SetBlackLevel(int blackLevelValue)
{
    if (!g_handle) return false;
    return MV_CC_SetIntValue(g_handle, "BlackLevel", blackLevelValue) == MV_OK;
}

extern "C" VISIONTRACKER_API bool GetBlackLevel(int* blackLevelValue)
{
    if (!g_handle || !blackLevelValue) return false;
    MVCC_INTVALUE stValue;
    if (MV_CC_GetIntValue(g_handle, "BlackLevel", &stValue) != MV_OK) return false;
    *blackLevelValue = stValue.nCurValue;
    return true;
}

extern "C" VISIONTRACKER_API double GetLastTimestamp()
{
    return g_lastTimestamp;
}

extern "C" VISIONTRACKER_API bool SelectCameraByIP(const char* ipAddress)
{
    if (!ipAddress) return false;
    g_selectedIp = ipAddress;
    std::cout << "g_selectedIp: " << g_selectedIp << std::endl;
    return true;
}

extern "C" VISIONTRACKER_API bool SelectCameraFromFile(const char* filepath)
{
    std::ifstream infile(filepath);
    if (!infile.is_open()) return false;

    std::string line;
    std::regex ipRegex("(\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}\\.\\d{1,3})");
    while (std::getline(infile, line)) {
        std::smatch match;
        if (std::regex_search(line, match, ipRegex)) {
            g_selectedIp = match[1];
            return true;
        }
    }
    return false;
}

extern "C" VISIONTRACKER_API void StartTracking()
{
    g_trackingActive = true;
}

extern "C" VISIONTRACKER_API void StopTracking()
{
    g_trackingActive = false;

    std::lock_guard<std::mutex> lock(g_ballInfoMutex);
    g_currentBallFound = false;
    g_currentBallCenter = cv::Point2f(-1, -1);
    g_currentBallRadius = 0.0f;
}

extern "C" VISIONTRACKER_API bool IsTrackingActive()
{
    return g_trackingActive.load();
}

extern "C" VISIONTRACKER_API void GetCurrentBallPosition(float* x, float* y, float* radius, bool* found)
{
    std::lock_guard<std::mutex> lock(g_ballInfoMutex);
    if (x) *x = g_currentBallCenter.x;
    if (y) *y = g_currentBallCenter.y;
    if (radius) *radius = g_currentBallRadius;
    if (found) *found = g_currentBallFound;
}

// 기존 InitCamera - deprecated
extern "C" VISIONTRACKER_API bool InitCamera()
{
    printf("Warning: InitCamera() is deprecated. Use EnumerateCameras() and ConnectCameraByIndex() instead.\n");
    g_lastError.clear();

    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (nRet != MV_OK || stDeviceList.nDeviceNum == 0) {
        g_lastError = "디바이스 나열 실패 또는 장치 없음.";
        return false;
    }

    return ConnectCameraByIndex(0);
}

extern "C" VISIONTRACKER_API void CloseCamera()
{
    DisconnectCamera();
}

// ===== Logger 인터페이스 구현 =====
extern "C" VISIONTRACKER_API bool InitializeLogger(const char* filePath, int logLevel, size_t maxFileSize, int maxBackupFiles)
{
    try {
        Logger::GetInstance().Initialize(
            filePath ? filePath : "VisionTracker.log",
            static_cast<LogLevel>(logLevel),
            maxFileSize,
            maxBackupFiles
        );

        LOG_INFO("Logger initialized successfully");
        return true;
    }
    catch (const std::exception& e) {
        g_lastError = std::string("Logger initialization failed: ") + e.what();
        return false;
    }
}

extern "C" VISIONTRACKER_API void ShutdownLogger()
{
    Logger::GetInstance().Flush();
    Logger::Destroy();
}

extern "C" VISIONTRACKER_API void SetLogLevel(int level)
{
    if (level >= 0 && level <= 4) {
        Logger::GetInstance().SetLogLevel(static_cast<LogLevel>(level));
    }
}

extern "C" VISIONTRACKER_API void LogDebug(const char* message)
{
    if (message) {
        LOG_DEBUG(std::string(message));
    }
}

extern "C" VISIONTRACKER_API void LogInfo(const char* message)
{
    if (message) {
        LOG_INFO(std::string(message));
    }
}

extern "C" VISIONTRACKER_API void LogWarning(const char* message)
{
    if (message) {
        LOG_WARNING(std::string(message));
    }
}

extern "C" VISIONTRACKER_API void LogError(const char* message)
{
    if (message) {
        LOG_ERROR(std::string(message));
    }
}

extern "C" VISIONTRACKER_API void LogCritical(const char* message)
{
    if (message) {
        LOG_CRITICAL(std::string(message));
    }
}

extern "C" VISIONTRACKER_API void FlushLogger()
{
    Logger::GetInstance().Flush();
}

extern "C" VISIONTRACKER_API bool AllocateConsole(const char* title)
{
    if (::AllocConsole()) {
        // stdout, stderr를 콘솔로 리다이렉트
        FILE* pCout;
        freopen_s(&pCout, "CONOUT$", "w", stdout);
        freopen_s(&pCout, "CONOUT$", "w", stderr);

        // std::cout과 동기화
        std::ios::sync_with_stdio();

        // 콘솔 제목 설정
        if (title) {
            SetConsoleTitleA(title);
        }
        else {
            SetConsoleTitleA("Vision Tracker Console");
        }

        // 콘솔 창 크기 조정
        HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
        COORD bufferSize = { 120, 3000 };  // 버퍼 크기
        SetConsoleScreenBufferSize(hConsole, bufferSize);

        SMALL_RECT windowSize = { 0, 0, 119, 30 };  // 창 크기
        SetConsoleWindowInfo(hConsole, TRUE, &windowSize);

        return true;
    }
    return false;
}

extern "C" VISIONTRACKER_API void FreeConsoleWindow()
{
    ::FreeConsole();
}