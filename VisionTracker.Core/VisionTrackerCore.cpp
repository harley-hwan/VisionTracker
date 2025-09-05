#include "pch.h"
#include "IVisionTracker.h"
#include "Logger.h"
#include <io.h>
#include <fcntl.h>
#include <iostream>
#include <atomic>
#include <mutex>
#include <thread>
#include <chrono>
#include <regex>
#include <fstream>

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

// ���� ���� �ʱ�ȭ
WhiteBallDetectionConfig g_whiteBallConfig = {
    cv::Rect(0, 0, 0, 0),
    ThresholdMode::Fixed,
    200,
    cv::THRESH_BINARY,
    AdaptiveMethod::Gaussian,
    7,
    2.0,
    2.0f, 10.0f,
    0.7f, 1.2f,
    true
};

// ���� ���� ������ - �ʼ����� ������ �������� ����
namespace {
    // ī�޶� �ڵ� �� ���� ����
    void* g_handle = nullptr;
    std::mutex g_handleMutex;  // �ڵ� ����/�����ø� ���
    std::atomic<bool> g_isConnected(false);

    // ī�޶� ����
    MV_CC_DEVICE_INFO_LIST g_deviceList;
    CameraInfo g_connectedCameraInfo;
    std::mutex g_cameraInfoMutex;

    // ������ ����
    cv::Mat g_latestFrame;
    std::mutex g_frameMutex;

    // FPS ��� - atomic���� ���
    std::atomic<bool> g_running(false);
    std::atomic<double> g_lastFps(0.0);
    std::chrono::steady_clock::time_point g_lastTime;
    std::atomic<int> g_frameCount(0);

    // �� Ž�� - atomic���� ���
    std::atomic<bool> g_trackingActive(false);
    std::atomic<bool> g_ballFound(false);
    std::atomic<float> g_ballX(-1), g_ballY(-1), g_ballRadius(0);
    std::atomic<double> g_lastDetectionTimeUs(0);

    // ROI ����
    cv::Rect g_roiRect;
    bool g_useRoi = false;
    std::mutex g_roiMutex;

    // ��ȭ ����
    bool g_recording = false;
    std::string g_outputFolder;
    std::atomic<int> g_frameCounter(0);
    std::mutex g_recordingMutex;

    // Ÿ�ӽ�����
    std::atomic<double> g_lastTimestamp(0.0);

    // ���õ� ī�޶� IP
    std::string g_selectedIp = "";
    std::mutex g_selectedIpMutex;

    // ���� �޽���
    std::string g_lastError = "";
    std::mutex g_errorMutex;

    // IP ��ȯ ���� �Լ�
    std::string ConvertIp(uint32_t ip) {
        char buf[32] = {};
        sprintf_s(buf, "%d.%d.%d.%d",
            (ip >> 24) & 0xff,
            (ip >> 16) & 0xff,
            (ip >> 8) & 0xff,
            ip & 0xff);
        return std::string(buf);
    }

    // ���� �޽��� ���� ����
    void SetLastError(const std::string& error) {
        std::lock_guard<std::mutex> lock(g_errorMutex);
        g_lastError = error;
    }

    // ��õ� ������ ���� �Լ�
    bool ConnectWithRetry(MV_CC_DEVICE_INFO* pDeviceInfo, int maxRetries = 3, int delaySeconds = 2) {
        for (int attempt = 0; attempt < maxRetries; attempt++) {
            void* tempHandle = nullptr;

            // �ڵ� ����
            int nRet = MV_CC_CreateHandle(&tempHandle, pDeviceInfo);
            if (nRet != MV_OK) {
                if (attempt < maxRetries - 1) {
                    LOG_INFO("Handle creation failed, retrying...");
                    std::this_thread::sleep_for(std::chrono::seconds(delaySeconds));
                    continue;
                }
                SetLastError("�ڵ� ���� ����");
                return false;
            }

            // ����̽� ����
            nRet = MV_CC_OpenDevice(tempHandle, MV_ACCESS_Exclusive, 0);
            if (nRet == MV_OK) {
                // ������ ���� �ڵ鿡 ����
                std::lock_guard<std::mutex> lock(g_handleMutex);
                g_handle = tempHandle;
                return true;
            }

            // ���н� �ڵ� ����
            MV_CC_DestroyHandle(tempHandle);

            if (attempt < maxRetries - 1) {
                LOG_INFO("Device open failed, retrying in " + std::to_string(delaySeconds) + " seconds...");
                std::this_thread::sleep_for(std::chrono::seconds(delaySeconds));
            }
        }

        SetLastError("����̽� ���� ���� (��� ��õ� ����)");
        return false;
    }

    // ������ �ڵ� ����
    void SafeReleaseHandle() {
        std::lock_guard<std::mutex> lock(g_handleMutex);
        if (g_handle) {
            MV_CC_StopGrabbing(g_handle);
            MV_CC_CloseDevice(g_handle);
            MV_CC_DestroyHandle(g_handle);
            g_handle = nullptr;
        }
    }
}

// �̹��� ���� �ݹ� �Լ�
extern "C" void __stdcall ImageCallback(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pInfo, void* pUser)
{
    if (!pInfo) return;
    if (pInfo->enPixelType != PixelType_Gvsp_BGR8_Packed &&
        pInfo->enPixelType != PixelType_Gvsp_Mono8) return;

    cv::Mat frame;

    // ������ Ÿ�Կ� ���� ó��
    if (pInfo->enPixelType == PixelType_Gvsp_Mono8) {
        frame = cv::Mat(pInfo->nHeight, pInfo->nWidth, CV_8UC1, pData).clone();
    }
    else if (pInfo->enPixelType == PixelType_Gvsp_BGR8_Packed) {
        frame = cv::Mat(pInfo->nHeight, pInfo->nWidth, CV_8UC3, pData).clone();
    }

    // ROI ����
    {
        std::lock_guard<std::mutex> roiLock(g_roiMutex);
        if (g_useRoi && g_roiRect.area() > 0 &&
            g_roiRect.x >= 0 && g_roiRect.y >= 0 &&
            g_roiRect.x + g_roiRect.width <= frame.cols &&
            g_roiRect.y + g_roiRect.height <= frame.rows) {
            frame = frame(g_roiRect);
        }
    }

    // ������ ����
    {
        std::lock_guard<std::mutex> lock(g_frameMutex);
        g_latestFrame = frame.clone();
    }

    // ��ȭ ó��
    {
        std::lock_guard<std::mutex> lock(g_recordingMutex);
        if (g_recording && !frame.empty()) {
            char filename[256];
            sprintf_s(filename, "%s/frame_%06d.png", g_outputFolder.c_str(), g_frameCounter.load());
            g_frameCounter++;
            cv::imwrite(filename, frame);
        }
    }

    // Ÿ�ӽ����� ����
    uint64_t timestampRaw = (static_cast<uint64_t>(pInfo->nDevTimeStampHigh) << 32) | pInfo->nDevTimeStampLow;
    g_lastTimestamp = timestampRaw / 1e9;

    // FPS ���
    g_frameCount++;
    auto now = std::chrono::steady_clock::now();
    double elapsed = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(now - g_lastTime).count());
    if (elapsed >= 1000.0) {
        g_lastFps = static_cast<double>(g_frameCount.load()) * 1000.0 / elapsed;
        g_frameCount = 0;
        g_lastTime = now;
    }

    // �� ������ Ȱ��ȭ�� ���
    if (g_trackingActive) {
        auto startTime = std::chrono::high_resolution_clock::now();

        int tmpX = 0, tmpY = 0, tmpR = 0, tmpPts = 0;
        bool found = GetWhiteBallInfo(&tmpX, &tmpY, &tmpR, &tmpPts);

        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
        g_lastDetectionTimeUs = duration.count();

        static int frameCounter = 0;
        frameCounter++;

        if (found) {
            g_ballFound = true;
            g_ballX = static_cast<float>(tmpX);
            g_ballY = static_cast<float>(tmpY);
            g_ballRadius = static_cast<float>(tmpR);

            char logMsg[256];
            sprintf_s(logMsg, "[Frame %06d] Ball FOUND - Position: (%.1f, %.1f), Radius: %.1f, Processing: %.2f ms",
                frameCounter, g_ballX.load(), g_ballY.load(), g_ballRadius.load(),
                g_lastDetectionTimeUs.load() / 1000.0);
            LOG_INFO(std::string(logMsg));
        }
        else {
            g_ballFound = false;

            char logMsg[256];
            sprintf_s(logMsg, "[Frame %06d] Ball NOT FOUND, Processing: %.2f ms",
                frameCounter, g_lastDetectionTimeUs.load() / 1000.0);
            LOG_DEBUG(std::string(logMsg));
        }
    }
}

// API ������
extern "C" VISIONTRACKER_API const char* GetLastErrorMessage() {
    std::lock_guard<std::mutex> lock(g_errorMutex);
    return g_lastError.c_str();
}

// ī�޶� ���� �Լ�
extern "C" VISIONTRACKER_API int EnumerateCameras(CameraInfo* cameras, int maxCount)
{
    memset(&g_deviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &g_deviceList);
    if (nRet != MV_OK) {
        SetLastError("����̽� ���� ����");
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

// �ε����� ī�޶� ����
extern "C" VISIONTRACKER_API bool ConnectCameraByIndex(int index)
{
    if (g_isConnected) {
        DisconnectCamera();
    }

    if (index < 0 || index >= (int)g_deviceList.nDeviceNum) {
        SetLastError("�߸��� ī�޶� �ε���");
        return false;
    }

    MV_CC_DEVICE_INFO* pDeviceInfo = g_deviceList.pDeviceInfo[index];

    // ��õ� ������ ����
    if (!ConnectWithRetry(pDeviceInfo, 3, 2)) {
        return false;
    }

    // �ݹ� ���
    int nRet = MV_CC_RegisterImageCallBackEx(g_handle, ImageCallback, nullptr);
    if (nRet != MV_OK) {
        SafeReleaseHandle();
        SetLastError("�ݹ� ��� ����");
        return false;
    }

    // �⺻ ���� ����
    MV_CC_SetEnumValue(g_handle, "GainAuto", 0);
    MV_CC_SetEnumValue(g_handle, "ExposureAuto", 0);

    if (IsNodeSupported("GammaEnable")) {
        EnableGamma(true);
    }

    if (IsNodeSupported("BlackLevelEnable")) {
        EnableBlackLevel(true);
    }

    g_isConnected = true;

    // ����� ī�޶� ���� ����
    {
        std::lock_guard<std::mutex> lock(g_cameraInfoMutex);
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
    }

    g_lastTime = std::chrono::steady_clock::now();
    g_frameCount = 0;
    g_lastFps = 0.0;

    return true;
}

// ī�޶� ���� ����
extern "C" VISIONTRACKER_API bool DisconnectCamera()
{
    if (!g_handle) return false;

    StopGrab();
    SafeReleaseHandle();
    g_isConnected = false;

    return true;
}

// ���� ���� Ȯ��
extern "C" VISIONTRACKER_API bool IsConnected()
{
    return g_isConnected && g_handle != nullptr;
}

// ����� ī�޶� ���� ��ȸ
extern "C" VISIONTRACKER_API bool GetConnectedCameraInfo(CameraInfo* info)
{
    if (!g_isConnected || !info) return false;

    std::lock_guard<std::mutex> lock(g_cameraInfoMutex);
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
        SetLastError("ī�޶� �ڵ��� �ʱ�ȭ���� �ʾҽ��ϴ�.");
        return false;
    }

    MV_CC_StopGrabbing(g_handle);

    int nRet = MV_CC_SetEnumValue(g_handle, "AcquisitionMode", 2);
    if (nRet != MV_OK) {
        printf("AcquisitionMode ���� ����: %d\n", nRet);
        return false;
    }

    nRet = MV_CC_SetEnumValue(g_handle, "TriggerMode", 0);
    if (nRet != MV_OK) {
        printf("TriggerMode ���� ���� ����: %d\n", nRet);
        return false;
    }

    nRet = MV_CC_SetEnumValue(g_handle, "TriggerSelector", 0);
    if (nRet != MV_OK) {
        printf("TriggerSelector(FrameStart) ���� ���� %d \n", nRet);
        return false;
    }

    MV_CC_SetEnumValue(g_handle, "TriggerSource", 0);
    MV_CC_SetEnumValue(g_handle, "TriggerActivation", 0);
    MV_CC_SetEnumValue(g_handle, "LineSelector", 0);
    MV_CC_SetEnumValue(g_handle, "LineMode", 0);

    int modeValue = enable ? 1 : 0;
    nRet = MV_CC_SetEnumValue(g_handle, "TriggerMode", modeValue);
    if (nRet != MV_OK) {
        printf("TriggerMode ���� ����\n");
        return false;
    }

    printf("Trigger ���� �Ϸ� (���: %s)\n", enable ? "Slave" : "Master");

    MVCC_FLOATVALUE stFps = {};
    nRet = MV_CC_GetFloatValue(g_handle, "ResultingFrameRate", &stFps);
    if (nRet == MV_OK) {
        printf("���� ����� FPS: %.2f\n", stFps.fCurValue);
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
    std::lock_guard<std::mutex> lock(g_recordingMutex);
    g_outputFolder = folderPath;
    g_frameCounter = 0;
    g_recording = true;
    return true;
}

extern "C" VISIONTRACKER_API void StopRecording()
{
    std::lock_guard<std::mutex> lock(g_recordingMutex);
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
    return g_lastFps.load();
}

extern "C" VISIONTRACKER_API bool SetFrameRate(float fps)
{
    if (!g_handle) {
        printf("FPS ���� ����\n");
        return false;
    }
    float temp_fps;
    printf("���� FPS ���� : ");
    GetFrameRate(&temp_fps);
    printf("%.2f�� ���� \n", temp_fps);
    printf("������ FPS : %f \n", fps);
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
        printf("GammaEnable ���� ����\n");
        SetLastError("GammaEnable ���� ����");
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
    std::lock_guard<std::mutex> lock(g_roiMutex);
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

    // �� ����� ROI ����
    cv::Rect roiRect = g_whiteBallConfig.roi;

    // ROI�� �������� �ʾҰų� (0,0,0,0)�̸� ��ü �̹��� ���
    if (roiRect.width == 0 || roiRect.height == 0) {
        roiRect = cv::Rect(0, 0, gray.cols, gray.rows);
    }

    // ROI�� ���� �̹��� ������ ������� �˻�
    if (roiRect.x < 0) roiRect.x = 0;
    if (roiRect.y < 0) roiRect.y = 0;
    if (roiRect.x + roiRect.width > gray.cols)
        roiRect.width = gray.cols - roiRect.x;
    if (roiRect.y + roiRect.height > gray.rows)
        roiRect.height = gray.rows - roiRect.y;

    // ROI�� ��ȿ���� ���� Ȯ��
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

        // ROI ��ǥ�� ���� �̹��� ��ǥ�� ��ȯ
        center.x += roiRect.x;
        center.y += roiRect.y;

        // ���� ������ ����� ���� ����
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
    g_whiteBallConfig = config;
}

extern "C" VISIONTRACKER_API WhiteBallDetectionConfig GetWhiteBallDetectionConfig()
{
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
        printf("BlackLevelEnable ���� ����\n");
        SetLastError("BlackLevelEnable ���� ����");
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
    return g_lastTimestamp.load();
}

extern "C" VISIONTRACKER_API bool SelectCameraByIP(const char* ipAddress)
{
    if (!ipAddress) return false;
    std::lock_guard<std::mutex> lock(g_selectedIpMutex);
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
            std::lock_guard<std::mutex> lock(g_selectedIpMutex);
            g_selectedIp = match[1];
            return true;
        }
    }
    return false;
}

extern "C" VISIONTRACKER_API void StartTracking()
{
    g_trackingActive = true;
    g_ballFound = false;
    g_ballX = -1;
    g_ballY = -1;
    g_ballRadius = 0;
}

extern "C" VISIONTRACKER_API void StopTracking()
{
    g_trackingActive = false;
    g_ballFound = false;
    g_ballX = -1;
    g_ballY = -1;
    g_ballRadius = 0;
}

extern "C" VISIONTRACKER_API bool IsTrackingActive()
{
    return g_trackingActive.load();
}

extern "C" VISIONTRACKER_API void GetCurrentBallPosition(float* x, float* y, float* radius, bool* found)
{
    if (x) *x = g_ballX.load();
    if (y) *y = g_ballY.load();
    if (radius) *radius = g_ballRadius.load();
    if (found) *found = g_ballFound.load();
}

extern "C" VISIONTRACKER_API double GetLastDetectionTimeMs()
{
    return g_lastDetectionTimeUs.load() / 1000.0;
}

// ���� InitCamera - deprecated
extern "C" VISIONTRACKER_API bool InitCamera()
{
    printf("Warning: InitCamera() is deprecated. Use EnumerateCameras() and ConnectCameraByIndex() instead.\n");
    SetLastError("");

    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (nRet != MV_OK || stDeviceList.nDeviceNum == 0) {
        SetLastError("����̽� ���� ���� �Ǵ� ��ġ ����.");
        return false;
    }

    return ConnectCameraByIndex(0);
}

extern "C" VISIONTRACKER_API void CloseCamera()
{
    DisconnectCamera();
}

// Logger �������̽� ����
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
        SetLastError(std::string("Logger initialization failed: ") + e.what());
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
        FILE* pCout;
        freopen_s(&pCout, "CONOUT$", "w", stdout);
        freopen_s(&pCout, "CONOUT$", "w", stderr);

        std::ios::sync_with_stdio();

        if (title) {
            SetConsoleTitleA(title);
        }
        else {
            SetConsoleTitleA("Vision Tracker Console");
        }

        HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
        COORD bufferSize = { 120, 3000 };
        SetConsoleScreenBufferSize(hConsole, bufferSize);

        SMALL_RECT windowSize = { 0, 0, 119, 30 };
        SetConsoleWindowInfo(hConsole, TRUE, &windowSize);

        return true;
    }
    return false;
}

extern "C" VISIONTRACKER_API void FreeConsoleWindow()
{
    ::FreeConsole();
}