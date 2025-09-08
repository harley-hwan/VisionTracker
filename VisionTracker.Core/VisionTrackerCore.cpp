#include "pch.h"
#include "IVisionTracker.h"
#include "Logger.h"
#include <io.h>
#include <fcntl.h>
#include <direct.h>
#include <iostream>
#include <atomic>
#include <mutex>
#include <thread>
#include <chrono>
#include <regex>
#include <fstream>
#include <queue>
#include <condition_variable>

#include <tbb/parallel_reduce.h>
#include <tbb/blocked_range.h>

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

// ������ ������ �� ���� ����ü
struct BallInfo {
    bool found;
    float x;
    float y;
    float radius;

    BallInfo() : found(false), x(-1), y(-1), radius(0) {}
    BallInfo(bool f, float px, float py, float r) : found(f), x(px), y(py), radius(r) {}
};

// ������ ���� Ǯ Ŭ���� - ������ ����
class FrameBufferPool {
private:
    struct BufferInfo {
        cv::Mat mat;
        std::chrono::steady_clock::time_point lastUsed;
        bool inUse;
    };

    std::vector<BufferInfo> allBuffers;
    std::queue<size_t> availableIndices;
    std::mutex poolMutex;
    std::condition_variable bufferAvailable;

    const size_t maxBuffers = 10;
    const std::chrono::seconds bufferTimeout{ 5 };

    std::thread cleanupThread;
    std::atomic<bool> stopCleanup{ false };

    void cleanupWorker() {
        while (!stopCleanup) {
            std::unique_lock<std::mutex> lock(poolMutex);
            auto now = std::chrono::steady_clock::now();

            // Ÿ�Ӿƿ��� ���� ���� �� �ε��� �籸��
            for (size_t i = 0; i < allBuffers.size(); ) {
                if (!allBuffers[i].inUse && now - allBuffers[i].lastUsed > bufferTimeout) {
                    allBuffers.erase(allBuffers.begin() + i);
                    // �ε��� �籸�� �ʿ�
                    rebuildAvailableIndices();
                }
                else {
                    ++i;
                }
            }

            lock.unlock();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    void rebuildAvailableIndices() {
        std::queue<size_t> newQueue;
        for (size_t i = 0; i < allBuffers.size(); ++i) {
            if (!allBuffers[i].inUse) {
                newQueue.push(i);
            }
        }
        availableIndices = std::move(newQueue);
    }

public:
    FrameBufferPool() : cleanupThread(&FrameBufferPool::cleanupWorker, this) {}

    ~FrameBufferPool() {
        stopCleanup = true;
        if (cleanupThread.joinable()) {
            cleanupThread.join();
        }
    }

    cv::Mat acquire(int rows, int cols, int type) {
        std::unique_lock<std::mutex> lock(poolMutex);

        // ���� ������ ���� ã��
        while (!availableIndices.empty()) {
            size_t idx = availableIndices.front();
            availableIndices.pop();

            if (idx < allBuffers.size() && !allBuffers[idx].inUse) {
                BufferInfo& info = allBuffers[idx];
                if (info.mat.rows == rows && info.mat.cols == cols && info.mat.type() == type) {
                    info.inUse = true;
                    info.lastUsed = std::chrono::steady_clock::now();
                    return info.mat;
                }
            }
        }

        // �� ���� ����
        if (allBuffers.size() < maxBuffers) {
            BufferInfo info;
            info.mat = cv::Mat(rows, cols, type);
            info.lastUsed = std::chrono::steady_clock::now();
            info.inUse = true;
            allBuffers.push_back(info);
            return info.mat;
        }

        // ���� Ǯ�� ���� �� ���, ���� ���۸� ��ٸ�
        bufferAvailable.wait(lock, [this]() {
            rebuildAvailableIndices();
            return !availableIndices.empty();
            });

        lock.unlock();
        return acquire(rows, cols, type);
    }

    void release(cv::Mat& buffer) {
        if (buffer.empty()) return;

        std::unique_lock<std::mutex> lock(poolMutex);

        for (size_t i = 0; i < allBuffers.size(); ++i) {
            if (allBuffers[i].mat.data == buffer.data) {
                allBuffers[i].inUse = false;
                allBuffers[i].lastUsed = std::chrono::steady_clock::now();
                availableIndices.push(i);
                bufferAvailable.notify_one();
                break;
            }
        }

        buffer = cv::Mat();
    }
};

// ��ǥ ��ȯ ���� Ŭ����
class CoordinateTransform {
public:
    static cv::Point2f roiToOriginal(const cv::Point2f& roiPoint, const cv::Rect& roi) {
        return cv::Point2f(roiPoint.x + roi.x, roiPoint.y + roi.y);
    }

    static cv::Point2f originalToRoi(const cv::Point2f& originalPoint, const cv::Rect& roi) {
        return cv::Point2f(originalPoint.x - roi.x, originalPoint.y - roi.y);
    }

    static cv::Rect validateROI(const cv::Rect& roi, const cv::Size& imageSize) {
        cv::Rect validROI = roi;
        if (validROI.x < 0) validROI.x = 0;
        if (validROI.y < 0) validROI.y = 0;
        if (validROI.x + validROI.width > imageSize.width)
            validROI.width = imageSize.width - validROI.x;
        if (validROI.y + validROI.height > imageSize.height)
            validROI.height = imageSize.height - validROI.y;

        if (validROI.width <= 0 || validROI.height <= 0) {
            return cv::Rect(0, 0, imageSize.width, imageSize.height);
        }
        return validROI;
    }
};

// �̱��� Vision Tracker Core Ŭ����
class VisionTrackerCore {
private:
    static VisionTrackerCore* instance;
    static std::mutex instanceMutex;

    // ī�޶� �ڵ� �� ���� ����
    void* handle = nullptr;
    std::mutex handleMutex;
    std::atomic<bool> isConnected{ false };

    // ī�޶� ����
    MV_CC_DEVICE_INFO_LIST deviceList;
    CameraInfo connectedCameraInfo;
    std::mutex cameraInfoMutex;

    // ������ ����
    cv::Mat latestFrame;
    std::mutex frameMutex;
    FrameBufferPool frameBufferPool;

    // ������ ��ȯ ĳ��
    cv::Mat cachedProcessedFrame;
    bool frameProcessingCacheValid = false;

    // FPS ���
    std::atomic<bool> running{ false };
    std::atomic<double> lastFps{ 0.0 };
    std::chrono::steady_clock::time_point lastTime;
    std::atomic<int> frameCount{ 0 };

    // �� ���� - ������ ������ ����ü ���
    std::atomic<bool> trackingActive{ false };
    std::atomic<BallInfo> ballInfo;
    std::atomic<double> lastDetectionTimeUs{ 0 };

    // ROI ����
    cv::Rect roiRect;
    bool useRoi = false;
    std::mutex roiMutex;

    // Ÿ�ӽ�����
    std::atomic<double> lastTimestamp{ 0.0 };

    // ���õ� ī�޶� IP
    std::string selectedIp;
    std::mutex selectedIpMutex;

    // ���� �޽���
    std::string lastError;
    std::mutex errorMutex;

    VisionTrackerCore() {
        memset(&deviceList, 0, sizeof(deviceList));
        memset(&connectedCameraInfo, 0, sizeof(connectedCameraInfo));
    }

    VisionTrackerCore(const VisionTrackerCore&) = delete;
    VisionTrackerCore& operator=(const VisionTrackerCore&) = delete;

public:
    ~VisionTrackerCore() {
        if (handle) {
            MV_CC_StopGrabbing(handle);
            MV_CC_CloseDevice(handle);
            MV_CC_DestroyHandle(handle);
        }
    }

    static VisionTrackerCore& getInstance() {
        std::lock_guard<std::mutex> lock(instanceMutex);
        if (!instance) {
            instance = new VisionTrackerCore();
        }
        return *instance;
    }

    static void destroyInstance() {
        std::lock_guard<std::mutex> lock(instanceMutex);
        if (instance) {
            delete instance;
            instance = nullptr;
        }
    }

    void* getHandle() {
        std::lock_guard<std::mutex> lock(handleMutex);
        return handle;
    }

    void setHandle(void* h) {
        std::lock_guard<std::mutex> lock(handleMutex);
        handle = h;
    }

    bool getIsConnected() const { return isConnected; }
    void setIsConnected(bool connected) { isConnected = connected; }

    cv::Mat getLatestFrame() {
        std::lock_guard<std::mutex> lock(frameMutex);
        return latestFrame;
    }

    void setLatestFrame(const cv::Mat& frame, bool bypassCache = false) {
        std::lock_guard<std::mutex> lock(frameMutex);

        if (!bypassCache && !frame.empty()) {
            cv::Mat buffer = frameBufferPool.acquire(frame.rows, frame.cols, frame.type());
            frame.copyTo(buffer);

            if (!latestFrame.empty()) {
                frameBufferPool.release(latestFrame);
            }

            latestFrame = buffer;
        }
        else {
            latestFrame = frame;
        }

        frameProcessingCacheValid = false;
    }

    void updateLatestFrameROI(const cv::Mat& roiFrame) {
        setLatestFrame(roiFrame, false);
    }

    std::string getLastError() {
        std::lock_guard<std::mutex> lock(errorMutex);
        return lastError;
    }

    void setLastError(const std::string& error) {
        std::lock_guard<std::mutex> lock(errorMutex);
        lastError = error;
    }

    MV_CC_DEVICE_INFO_LIST& getDeviceList() { return deviceList; }

    CameraInfo getConnectedCameraInfo() {
        std::lock_guard<std::mutex> lock(cameraInfoMutex);
        return connectedCameraInfo;
    }

    void setConnectedCameraInfo(const CameraInfo& info) {
        std::lock_guard<std::mutex> lock(cameraInfoMutex);
        connectedCameraInfo = info;
    }

    cv::Rect getROI() {
        std::lock_guard<std::mutex> lock(roiMutex);
        return roiRect;
    }

    void setROI(const cv::Rect& roi) {
        std::lock_guard<std::mutex> lock(roiMutex);
        roiRect = roi;
        useRoi = (roi.area() > 0);
        frameProcessingCacheValid = false;
    }

    bool isROIEnabled() {
        std::lock_guard<std::mutex> lock(roiMutex);
        return useRoi;
    }

    void updateFPS() {
        frameCount++;
        auto now = std::chrono::steady_clock::now();
        double elapsed = static_cast<double>(
            std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTime).count()
            );
        if (elapsed >= 1000.0) {
            lastFps = static_cast<double>(frameCount.load()) * 1000.0 / elapsed;
            frameCount = 0;
            lastTime = now;
        }
    }

    void resetFPSCounter() {
        lastTime = std::chrono::steady_clock::now();
        frameCount = 0;
        lastFps = 0.0;
    }

    double getFPS() const { return lastFps; }

    // ������ ������ �� ���� ����/��������
    void setBallInfo(bool found, float x, float y, float radius) {
        ballInfo.store(BallInfo(found, x, y, radius));
    }

    BallInfo getBallInfo() const {
        return ballInfo.load();
    }

    // ���� ����
    bool isTracking() const { return trackingActive; }
    void setTracking(bool active) { trackingActive = active; }

    void setDetectionTime(double timeUs) { lastDetectionTimeUs = timeUs; }
    double getDetectionTimeMs() const { return lastDetectionTimeUs / 1000.0; }

    void setLastTimestamp(double ts) { lastTimestamp = ts; }
    double getLastTimestamp() const { return lastTimestamp; }

    bool isRunning() const { return running; }
    void setRunning(bool r) { running = r; }

    std::string getSelectedIp() {
        std::lock_guard<std::mutex> lock(selectedIpMutex);
        return selectedIp;
    }

    void setSelectedIp(const std::string& ip) {
        std::lock_guard<std::mutex> lock(selectedIpMutex);
        selectedIp = ip;
    }

    static std::string ConvertIp(uint32_t ip) {
        char buf[32] = {};
        sprintf_s(buf, "%d.%d.%d.%d",
            (ip >> 24) & 0xff,
            (ip >> 16) & 0xff,
            (ip >> 8) & 0xff,
            ip & 0xff);
        return std::string(buf);
    }

    bool ConnectWithRetry(MV_CC_DEVICE_INFO* pDeviceInfo, int maxRetries = 3, int delaySeconds = 2) {
        for (int attempt = 0; attempt < maxRetries; attempt++) {
            void* tempHandle = nullptr;

            int nRet = MV_CC_CreateHandle(&tempHandle, pDeviceInfo);
            if (nRet != MV_OK) {
                if (attempt < maxRetries - 1) {
                    LOG_INFO("Handle creation failed, retrying...");
                    std::this_thread::sleep_for(std::chrono::seconds(delaySeconds));
                    continue;
                }
                setLastError("�ڵ� ���� ����");
                return false;
            }

            nRet = MV_CC_OpenDevice(tempHandle, MV_ACCESS_Exclusive, 0);
            if (nRet == MV_OK) {
                setHandle(tempHandle);
                return true;
            }

            MV_CC_DestroyHandle(tempHandle);

            if (attempt < maxRetries - 1) {
                LOG_INFO("Device open failed, retrying in " + std::to_string(delaySeconds) + " seconds...");
                std::this_thread::sleep_for(std::chrono::seconds(delaySeconds));
            }
        }

        setLastError("����̽� ���� ���� (��� ��õ� ����)");
        return false;
    }

    void SafeReleaseHandle() {
        std::lock_guard<std::mutex> lock(handleMutex);
        if (handle) {
            MV_CC_StopGrabbing(handle);
            MV_CC_CloseDevice(handle);
            MV_CC_DestroyHandle(handle);
            handle = nullptr;
        }
    }
};

// �̱��� static ��� �ʱ�ȭ
VisionTrackerCore* VisionTrackerCore::instance = nullptr;
std::mutex VisionTrackerCore::instanceMutex;

// �̹��� ���� �ݹ� �Լ� - ����ȭ�� ����
extern "C" void __stdcall ImageCallback(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pInfo, void* pUser)
{
    if (!pInfo || !pData) return;
    if (pInfo->enPixelType != PixelType_Gvsp_BGR8_Packed &&
        pInfo->enPixelType != PixelType_Gvsp_Mono8) return;

    auto& core = VisionTrackerCore::getInstance();

    try {
        cv::Mat frame;

        // ������ Ÿ�Կ� ���� ó�� - ���� ����� ����
        if (pInfo->enPixelType == PixelType_Gvsp_Mono8) {
            frame = cv::Mat(pInfo->nHeight, pInfo->nWidth, CV_8UC1, pData);
        }
        else if (pInfo->enPixelType == PixelType_Gvsp_BGR8_Packed) {
            frame = cv::Mat(pInfo->nHeight, pInfo->nWidth, CV_8UC3, pData);
        }

        // ROI ����
        cv::Mat roiFrame = frame;
        cv::Rect currentROI = core.getROI();
        if (core.isROIEnabled() && currentROI.area() > 0) {
            cv::Rect validROI = CoordinateTransform::validateROI(currentROI, frame.size());
            if (validROI.area() > 0) {
                roiFrame = frame(validROI);
            }
        }

        // ������ ����
        core.updateLatestFrameROI(roiFrame);

        // Ÿ�ӽ����� ����
        uint64_t timestampRaw = (static_cast<uint64_t>(pInfo->nDevTimeStampHigh) << 32) | pInfo->nDevTimeStampLow;
        core.setLastTimestamp(timestampRaw / 1e9);

        // FPS ���
        core.updateFPS();

        // �� ������ Ȱ��ȭ�� ���
        if (core.isTracking()) {
            auto startTime = std::chrono::high_resolution_clock::now();

            int tmpX = 0, tmpY = 0, tmpR = 0, tmpPts = 0;
            bool found = GetWhiteBallInfo(&tmpX, &tmpY, &tmpR, &tmpPts);

            auto endTime = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
            core.setDetectionTime(duration.count());

            static int frameCounter = 0;
            frameCounter++;

            if (found) {
                core.setBallInfo(true, static_cast<float>(tmpX),
                    static_cast<float>(tmpY), static_cast<float>(tmpR));

                char logMsg[256];
                sprintf_s(logMsg, "[Frame %06d] Ball FOUND - Position: (%.1f, %.1f), Radius: %.1f, Processing: %.2f ms",
                    frameCounter, static_cast<float>(tmpX), static_cast<float>(tmpY),
                    static_cast<float>(tmpR), duration.count() / 1000.0);
                LOG_INFO(std::string(logMsg));
            }
            else {
                core.setBallInfo(false, -1, -1, 0);

                char logMsg[256];
                sprintf_s(logMsg, "[Frame %06d] Ball NOT FOUND, Processing: %.2f ms",
                    frameCounter, duration.count() / 1000.0);
                LOG_DEBUG(std::string(logMsg));
            }
        }
    }
    catch (const cv::Exception& e) {
        LOG_ERROR("OpenCV error in callback: " + std::string(e.what()));
    }
    catch (const std::exception& e) {
        LOG_ERROR("Standard exception in callback: " + std::string(e.what()));
    }
    catch (...) {
        LOG_ERROR("Unknown exception in callback");
    }
}

// API ������
extern "C" VISIONTRACKER_API const char* GetLastErrorMessage() {
    return VisionTrackerCore::getInstance().getLastError().c_str();
}

extern "C" VISIONTRACKER_API int EnumerateCameras(CameraInfo* cameras, int maxCount)
{
    if (!cameras || maxCount <= 0) return 0;

    auto& core = VisionTrackerCore::getInstance();
    auto& deviceList = core.getDeviceList();
    memset(&deviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &deviceList);
    if (nRet != MV_OK) {
        core.setLastError("����̽� ���� ����");
        return 0;
    }

    int count = std::min((int)deviceList.nDeviceNum, maxCount);

    for (int i = 0; i < count; ++i) {
        MV_CC_DEVICE_INFO* pDeviceInfo = deviceList.pDeviceInfo[i];
        CameraInfo& cam = cameras[i];
        memset(&cam, 0, sizeof(CameraInfo));

        if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
            MV_GIGE_DEVICE_INFO& stGigEInfo = pDeviceInfo->SpecialInfo.stGigEInfo;

            sprintf_s(cam.modelName, "%s", reinterpret_cast<const char*>(stGigEInfo.chModelName));
            sprintf_s(cam.serialNumber, "%s", reinterpret_cast<const char*>(stGigEInfo.chSerialNumber));
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

            sprintf_s(cam.modelName, "%s", reinterpret_cast<const char*>(stUSBInfo.chModelName));
            sprintf_s(cam.serialNumber, "%s", reinterpret_cast<const char*>(stUSBInfo.chSerialNumber));
            sprintf_s(cam.displayName, "%s [%s] - USB",
                cam.modelName, cam.serialNumber);
            cam.deviceType = 1;
        }
    }

    return count;
}

extern "C" VISIONTRACKER_API bool ConnectCameraByIndex(int index)
{
    auto& core = VisionTrackerCore::getInstance();

    if (core.getIsConnected()) {
        DisconnectCamera();
    }

    auto& deviceList = core.getDeviceList();
    if (index < 0 || index >= (int)deviceList.nDeviceNum) {
        core.setLastError("�߸��� ī�޶� �ε���");
        return false;
    }

    MV_CC_DEVICE_INFO* pDeviceInfo = deviceList.pDeviceInfo[index];

    if (!core.ConnectWithRetry(pDeviceInfo, 3, 2)) {
        return false;
    }

    void* handle = core.getHandle();

    int nRet = MV_CC_RegisterImageCallBackEx(handle, ImageCallback, nullptr);
    if (nRet != MV_OK) {
        core.SafeReleaseHandle();
        core.setLastError("�ݹ� ��� ����");
        return false;
    }

    MV_CC_SetEnumValue(handle, "GainAuto", 0);
    MV_CC_SetEnumValue(handle, "ExposureAuto", 0);

    if (IsNodeSupported("GammaEnable")) {
        EnableGamma(true);
    }

    if (IsNodeSupported("BlackLevelEnable")) {
        EnableBlackLevel(true);
    }

    core.setIsConnected(true);

    // ����� ī�޶� ���� ����
    CameraInfo connectedInfo;
    memset(&connectedInfo, 0, sizeof(CameraInfo));

    if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
        MV_GIGE_DEVICE_INFO& info = pDeviceInfo->SpecialInfo.stGigEInfo;
        sprintf_s(connectedInfo.modelName, "%s", reinterpret_cast<const char*>(info.chModelName));
        sprintf_s(connectedInfo.serialNumber, "%s", reinterpret_cast<const char*>(info.chSerialNumber));
        sprintf_s(connectedInfo.ipAddress, "%s", VisionTrackerCore::ConvertIp(info.nCurrentIp).c_str());
        sprintf_s(connectedInfo.displayName, "%s [%s] - %s",
            connectedInfo.modelName,
            connectedInfo.serialNumber,
            connectedInfo.ipAddress);
        connectedInfo.deviceType = 0;
    }

    core.setConnectedCameraInfo(connectedInfo);
    core.resetFPSCounter();

    return true;
}

extern "C" VISIONTRACKER_API bool DisconnectCamera()
{
    auto& core = VisionTrackerCore::getInstance();
    void* handle = core.getHandle();

    if (!handle) return false;

    StopGrab();
    core.SafeReleaseHandle();
    core.setIsConnected(false);

    return true;
}

extern "C" VISIONTRACKER_API bool IsConnected()
{
    auto& core = VisionTrackerCore::getInstance();
    return core.getIsConnected() && core.getHandle() != nullptr;
}

extern "C" VISIONTRACKER_API bool GetConnectedCameraInfo(CameraInfo* info)
{
    auto& core = VisionTrackerCore::getInstance();
    if (!core.getIsConnected() || !info) return false;

    *info = core.getConnectedCameraInfo();
    return true;
}

extern "C" VISIONTRACKER_API bool IsNodeSupported(const char* nodeName)
{
    auto& core = VisionTrackerCore::getInstance();
    void* handle = core.getHandle();
    if (!handle) return false;

    bool dummyValue = false;
    int nRet = MV_CC_GetBoolValue(handle, nodeName, &dummyValue);

    return (nRet == MV_OK);
}

extern "C" VISIONTRACKER_API bool SetTriggerMode(bool enable)
{
    auto& core = VisionTrackerCore::getInstance();
    void* handle = core.getHandle();

    if (!handle) {
        core.setLastError("ī�޶� �ڵ��� �ʱ�ȭ���� �ʾҽ��ϴ�.");
        return false;
    }

    MV_CC_StopGrabbing(handle);

    int nRet = MV_CC_SetEnumValue(handle, "AcquisitionMode", 2);
    if (nRet != MV_OK) {
        printf("AcquisitionMode ���� ����: %d\n", nRet);
        return false;
    }

    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    if (nRet != MV_OK) {
        printf("TriggerMode ���� ���� ����: %d\n", nRet);
        return false;
    }

    nRet = MV_CC_SetEnumValue(handle, "TriggerSelector", 0);
    if (nRet != MV_OK) {
        printf("TriggerSelector(FrameStart) ���� ���� %d \n", nRet);
        return false;
    }

    MV_CC_SetEnumValue(handle, "TriggerSource", 0);
    MV_CC_SetEnumValue(handle, "TriggerActivation", 0);
    MV_CC_SetEnumValue(handle, "LineSelector", 0);
    MV_CC_SetEnumValue(handle, "LineMode", 0);

    int modeValue = enable ? 1 : 0;
    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", modeValue);
    if (nRet != MV_OK) {
        printf("TriggerMode ���� ����\n");
        return false;
    }

    printf("Trigger ���� �Ϸ� (���: %s)\n", enable ? "Slave" : "Master");

    MVCC_FLOATVALUE stFps = {};
    nRet = MV_CC_GetFloatValue(handle, "ResultingFrameRate", &stFps);
    if (nRet == MV_OK) {
        printf("���� ����� FPS: %.2f\n", stFps.fCurValue);
    }

    return true;
}

extern "C" VISIONTRACKER_API void StartGrab()
{
    auto& core = VisionTrackerCore::getInstance();
    void* handle = core.getHandle();
    if (!handle) return;

    core.setRunning(true);
    MV_CC_StartGrabbing(handle);
    core.resetFPSCounter();
}

extern "C" VISIONTRACKER_API void StopGrab()
{
    auto& core = VisionTrackerCore::getInstance();
    void* handle = core.getHandle();
    if (!handle) return;

    core.setRunning(false);
    MV_CC_StopGrabbing(handle);
}

extern "C" VISIONTRACKER_API bool GetLatestFrameInfo(int* width, int* height, int* channels)
{
    auto& core = VisionTrackerCore::getInstance();
    cv::Mat frame = core.getLatestFrame();

    if (frame.empty()) return false;

    *width = frame.cols;
    *height = frame.rows;
    *channels = frame.channels();
    return true;
}

extern "C" VISIONTRACKER_API bool GetLatestFrame(
    unsigned char* buffer, int bufferSize,
    int* width, int* height, int* channels)
{
    auto& core = VisionTrackerCore::getInstance();
    cv::Mat frame = core.getLatestFrame();

    if (frame.empty()) return false;

    int needed = static_cast<int>(frame.total() * frame.elemSize());
    if (bufferSize < needed) return false;

    memcpy(buffer, frame.data, needed);
    *width = frame.cols;
    *height = frame.rows;
    *channels = frame.channels();
    return true;
}

extern "C" VISIONTRACKER_API double GetCurrentBrightness()
{
    auto& core = VisionTrackerCore::getInstance();
    cv::Mat frame = core.getLatestFrame();

    if (frame.empty()) return -1.0;

    if (frame.channels() == 1)
        return cv::mean(frame)[0];
    else if (frame.channels() == 3) {
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        return cv::mean(gray)[0];
    }
    return -1.0;
}

extern "C" VISIONTRACKER_API double GetCurrentFps()
{
    return VisionTrackerCore::getInstance().getFPS();
}

extern "C" VISIONTRACKER_API bool SetFrameRate(float fps)
{
    auto& core = VisionTrackerCore::getInstance();
    void* handle = core.getHandle();

    if (!handle) {
        printf("FPS ���� ����\n");
        return false;
    }
    float temp_fps;
    printf("���� FPS ���� : ");
    GetFrameRate(&temp_fps);
    printf("%.2f�� ���� \n", temp_fps);
    printf("������ FPS : %f \n", fps);
    return MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", fps) == MV_OK;
}

extern "C" VISIONTRACKER_API bool GetFrameRate(float* fps)
{
    auto& core = VisionTrackerCore::getInstance();
    void* handle = core.getHandle();

    if (!handle || !fps) return false;
    MVCC_FLOATVALUE val;
    if (MV_CC_GetFloatValue(handle, "AcquisitionFrameRate", &val) != MV_OK) return false;
    *fps = val.fCurValue;
    return true;
}

extern "C" VISIONTRACKER_API bool SetExposureTime(float exposureTime)
{
    auto& core = VisionTrackerCore::getInstance();
    void* handle = core.getHandle();

    if (!handle) return false;
    return MV_CC_SetFloatValue(handle, "ExposureTime", exposureTime) == MV_OK;
}

extern "C" VISIONTRACKER_API bool GetExposureTime(float* exposureTime)
{
    auto& core = VisionTrackerCore::getInstance();
    void* handle = core.getHandle();

    if (!handle || !exposureTime) return false;
    MVCC_FLOATVALUE val;
    if (MV_CC_GetFloatValue(handle, "ExposureTime", &val) != MV_OK) return false;
    *exposureTime = val.fCurValue;
    return true;
}

extern "C" VISIONTRACKER_API bool EnableGamma(bool enable)
{
    auto& core = VisionTrackerCore::getInstance();
    void* handle = core.getHandle();

    if (!handle) return false;

    int nRet = MV_CC_SetBoolValue(handle, "GammaEnable", enable);
    if (nRet != MV_OK) {
        printf("GammaEnable ���� ����\n");
        core.setLastError("GammaEnable ���� ����");
        return false;
    }

    return true;
}

extern "C" VISIONTRACKER_API bool SetGamma(float gammaValue)
{
    auto& core = VisionTrackerCore::getInstance();
    void* handle = core.getHandle();

    if (!handle) return false;
    return MV_CC_SetFloatValue(handle, "Gamma", gammaValue) == MV_OK;
}

extern "C" VISIONTRACKER_API bool GetGamma(float* gammaValue)
{
    auto& core = VisionTrackerCore::getInstance();
    void* handle = core.getHandle();

    if (!handle || !gammaValue) return false;
    MVCC_FLOATVALUE val;
    if (MV_CC_GetFloatValue(handle, "Gamma", &val) != MV_OK) return false;
    *gammaValue = val.fCurValue;
    return true;
}

extern "C" VISIONTRACKER_API bool SetROI(int x, int y, int width, int height)
{
    VisionTrackerCore::getInstance().setROI(cv::Rect(x, y, width, height));
    return true;
}

extern "C" VISIONTRACKER_API bool GetWhiteBallInfo(int* centerX, int* centerY, int* radius, int* pointCount)
{
    if (!centerX || !centerY || !radius || !pointCount)
        return false;

    auto& core = VisionTrackerCore::getInstance();
    cv::Mat frame = core.getLatestFrame();

    if (frame.empty())
        return false;

    try {
        cv::Mat gray, bin;

        // 1. �׷��̽����� ��ȯ
        if (frame.channels() == 3)
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        else
            gray = frame;

        // ROI ���� - ��ǥ�� �ϰ��� ����
        cv::Rect roiRect = g_whiteBallConfig.roi;
        cv::Rect imageRect(0, 0, gray.cols, gray.rows);

        if (roiRect.width <= 0 || roiRect.height <= 0) {
            roiRect = imageRect;
        }
        else {
            roiRect = CoordinateTransform::validateROI(roiRect, gray.size());
        }

        if (roiRect.empty()) return false;

        cv::Mat roi = gray(roiRect);

        // 2. ��ó��
        cv::Mat work = roi.clone();
        cv::Scalar mean, stddev;
        cv::meanStdDev(roi, mean, stddev);

        if (stddev[0] > 30.0) {
            cv::GaussianBlur(work, work, cv::Size(3, 3), 0.5);
        }

        // 3. ����ȭ
        switch (g_whiteBallConfig.thresholdMode)
        {
        case ThresholdMode::Fixed:
            cv::threshold(work, bin, g_whiteBallConfig.thresholdValue, 255, cv::THRESH_BINARY);
            break;
        case ThresholdMode::Adaptive:
            cv::adaptiveThreshold(work, bin, 255,
                static_cast<int>(g_whiteBallConfig.adaptiveMethod),
                cv::THRESH_BINARY,
                g_whiteBallConfig.blockSize | 1,
                g_whiteBallConfig.C);
            break;
        case ThresholdMode::Otsu:
            cv::threshold(work, bin, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
            break;
        }

        // 4. �������� ���� (������ ���� ���� ����)
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (contours.size() > 10) {
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
            cv::morphologyEx(bin, bin, cv::MORPH_OPEN, kernel);
            contours.clear();
            cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        }

        if (contours.empty()) return false;

        // �Ķ���� ĳ��
        const float minRadius = g_whiteBallConfig.minRadius;
        const float maxRadius = g_whiteBallConfig.maxRadius;
        const float minCircularity = g_whiteBallConfig.minCircularity;
        const float maxCircularity = g_whiteBallConfig.maxCircularity;

        // 5. TBB�� ����� ���� ������ ����
        struct BallCandidate {
            int index = -1;
            float score = 0;
            cv::Point2f center;
            float radius = 0;
            int points = 0;
        };

        BallCandidate bestCandidate = tbb::parallel_reduce(
            tbb::blocked_range<size_t>(0, contours.size()),
            BallCandidate(),
            [&](const tbb::blocked_range<size_t>& range, BallCandidate localBest) -> BallCandidate {

                for (size_t i = range.begin(); i < range.end(); ++i)
                {
                    size_t contourSize = contours[i].size();
                    if (contourSize < 5 || contourSize > 500) continue;

                    float radius;
                    cv::Point2f center;
                    cv::minEnclosingCircle(contours[i], center, radius);

                    if (radius < minRadius || radius > maxRadius)
                        continue;

                    double area = cv::contourArea(contours[i]);
                    if (area < CV_PI * minRadius * minRadius * 0.7)
                        continue;

                    double perimeter = cv::arcLength(contours[i], true);
                    if (perimeter <= 0) continue;
                    double circularity = 4 * CV_PI * area / (perimeter * perimeter);

                    if (circularity < minCircularity || circularity > maxCircularity)
                        continue;

                    double expectedArea = CV_PI * radius * radius;
                    double areaRatio = area / expectedArea;
                    if (areaRatio < 0.75 || areaRatio > 1.05)
                        continue;

                    float score = circularity * 0.6f + areaRatio * 0.4f;

                    if (score > localBest.score) {
                        localBest.index = static_cast<int>(i);
                        localBest.score = score;
                        localBest.center = center;
                        localBest.radius = radius;
                        localBest.points = static_cast<int>(contourSize);
                    }
                }
                return localBest;
            },
            [](BallCandidate a, BallCandidate b) -> BallCandidate {
                return (a.score > b.score) ? a : b;
            }
        );

        // 6. ��� ���� �� ��ȯ
        if (bestCandidate.index == -1 || bestCandidate.score < 0.7f) {
            return false;
        }

        // ROI ��ǥ�� ���� �̹��� ��ǥ�� ��ȯ - ��ǥ�� �ϰ��� ����
        cv::Point2f originalCenter = CoordinateTransform::roiToOriginal(bestCandidate.center, roiRect);

        // ī�޶� �̹� ROI�� ������ ��� �߰� ��ȯ
        cv::Rect cameraROI = core.getROI();
        if (core.isROIEnabled() && cameraROI.area() > 0) {
            originalCenter = CoordinateTransform::roiToOriginal(originalCenter, cameraROI);
        }

        *centerX = static_cast<int>(originalCenter.x);
        *centerY = static_cast<int>(originalCenter.y);
        *radius = static_cast<int>(bestCandidate.radius);
        *pointCount = bestCandidate.points;

        return true;
    }
    catch (...) {
        return false;
    }
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
    auto& core = VisionTrackerCore::getInstance();
    void* handle = core.getHandle();

    if (!handle) return false;
    return MV_CC_SetFloatValue(handle, "Gain", gain) == MV_OK;
}

extern "C" VISIONTRACKER_API bool GetGain(float* gain)
{
    auto& core = VisionTrackerCore::getInstance();
    void* handle = core.getHandle();

    if (!handle || !gain) return false;
    MVCC_FLOATVALUE val;
    if (MV_CC_GetFloatValue(handle, "Gain", &val) != MV_OK) return false;
    *gain = val.fCurValue;
    return true;
}

extern "C" VISIONTRACKER_API bool EnableBlackLevel(bool enable)
{
    auto& core = VisionTrackerCore::getInstance();
    void* handle = core.getHandle();

    if (!handle) return false;

    int nRet = MV_CC_SetBoolValue(handle, "BlackLevelEnable", enable);
    if (nRet != MV_OK) {
        printf("BlackLevelEnable ���� ����\n");
        core.setLastError("BlackLevelEnable ���� ����");
        return false;
    }

    return true;
}

extern "C" VISIONTRACKER_API bool SetBlackLevel(int blackLevelValue)
{
    auto& core = VisionTrackerCore::getInstance();
    void* handle = core.getHandle();

    if (!handle) return false;
    return MV_CC_SetIntValue(handle, "BlackLevel", blackLevelValue) == MV_OK;
}

extern "C" VISIONTRACKER_API bool GetBlackLevel(int* blackLevelValue)
{
    auto& core = VisionTrackerCore::getInstance();
    void* handle = core.getHandle();

    if (!handle || !blackLevelValue) return false;
    MVCC_INTVALUE stValue;
    if (MV_CC_GetIntValue(handle, "BlackLevel", &stValue) != MV_OK) return false;
    *blackLevelValue = stValue.nCurValue;
    return true;
}

extern "C" VISIONTRACKER_API double GetLastTimestamp()
{
    return VisionTrackerCore::getInstance().getLastTimestamp();
}

extern "C" VISIONTRACKER_API bool SelectCameraByIP(const char* ipAddress)
{
    if (!ipAddress) return false;
    VisionTrackerCore::getInstance().setSelectedIp(ipAddress);
    std::cout << "g_selectedIp: " << ipAddress << std::endl;
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
            VisionTrackerCore::getInstance().setSelectedIp(match[1]);
            return true;
        }
    }
    return false;
}

extern "C" VISIONTRACKER_API void StartTracking()
{
    auto& core = VisionTrackerCore::getInstance();
    core.setTracking(true);
    core.setBallInfo(false, -1, -1, 0);
}

extern "C" VISIONTRACKER_API void StopTracking()
{
    auto& core = VisionTrackerCore::getInstance();
    core.setTracking(false);
    core.setBallInfo(false, -1, -1, 0);
}

extern "C" VISIONTRACKER_API bool IsTrackingActive()
{
    return VisionTrackerCore::getInstance().isTracking();
}

extern "C" VISIONTRACKER_API void GetCurrentBallPosition(float* x, float* y, float* radius, bool* found)
{
    BallInfo info = VisionTrackerCore::getInstance().getBallInfo();
    if (x) *x = info.x;
    if (y) *y = info.y;
    if (radius) *radius = info.radius;
    if (found) *found = info.found;
}

extern "C" VISIONTRACKER_API double GetLastDetectionTimeMs()
{
    return VisionTrackerCore::getInstance().getDetectionTimeMs();
}

// ���� InitCamera - deprecated
extern "C" VISIONTRACKER_API bool InitCamera()
{
    printf("Warning: InitCamera() is deprecated. Use EnumerateCameras() and ConnectCameraByIndex() instead.\n");
    auto& core = VisionTrackerCore::getInstance();
    core.setLastError("");

    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (nRet != MV_OK || stDeviceList.nDeviceNum == 0) {
        core.setLastError("����̽� ���� ���� �Ǵ� ��ġ ����.");
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
        std::string logDir = "log";
        if (_access(logDir.c_str(), 0) != 0) {
            if (_mkdir(logDir.c_str()) != 0) {
                logDir = ".";
            }
        }

        std::string logFilePath;
        if (filePath) {
            logFilePath = filePath;
        }
        else {
            logFilePath = logDir + "/VisionTracker.log";
        }

        Logger::GetInstance().Initialize(
            logFilePath,
            static_cast<LogLevel>(logLevel),
            maxFileSize,
            maxBackupFiles
        );

        LOG_INFO("Logger initialized successfully in directory: " + logDir);
        return true;
    }
    catch (const std::exception& e) {
        VisionTrackerCore::getInstance().setLastError(std::string("Logger initialization failed: ") + e.what());
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

// ���α׷� ���� �� �̱��� ����
struct VisionTrackerCoreCleanup {
    ~VisionTrackerCoreCleanup() {
        VisionTrackerCore::destroyInstance();
    }
} g_cleanup;