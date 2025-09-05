#pragma once
#ifndef PCH_H
#include <opencv2/opencv.hpp>
#include <vector>
#endif

#ifdef VISIONTRACKER_CORE_EXPORTS
#define VISIONTRACKER_API __declspec(dllexport)
#else
#define VISIONTRACKER_API __declspec(dllimport)
#endif

// Threshold ��� ������
enum class ThresholdMode
{
    Fixed,        // �Ϲ� cv::threshold
    Adaptive,     // cv::adaptiveThreshold
    Otsu,         // cv::threshold + OTSU
    AdaptiveOtsu  // (ȥ�� ��)
};

// Adaptive ��� ������
enum class AdaptiveMethod
{
    Mean = cv::ADAPTIVE_THRESH_MEAN_C,          // 0
    Gaussian = cv::ADAPTIVE_THRESH_GAUSSIAN_C   // 1
};

// ��� �� ���� ���� ����ü
struct WhiteBallDetectionConfig
{
    cv::Rect roi;                // ���� ����
    ThresholdMode thresholdMode; // �Ϲ� / ���
    int thresholdValue;          // ���̳ʸ� �Ӱ谪
    int thresholdType;           // cv::THRESH_BINARY ��
    AdaptiveMethod adaptiveMethod; // ��� ��忡�� adaptive ���
    int blockSize;               // adaptive threshold�� ��� ũ�� (Ȧ��, 3 �̻�)
    double C;                    // adaptive threshold�� ��� ���갪
    float minRadius;             // ������ �ּҰ�
    float maxRadius;             // ������ �ִ밪
    float minCircularity;        // ���� ���絵 �ּ�
    float maxCircularity;        // ���� ���絵 �ִ�
    bool useTracking;            // ���� ������ ��� �Ÿ� ���� ����
};

// ī�޶� ���� ����ü
struct CameraInfo {
    char displayName[256];
    char serialNumber[64];
    char modelName[64];
    char ipAddress[32];
    int deviceType;  // 0: GigE, 1: USB
};

// ���� ���� �ܺ� ����
extern WhiteBallDetectionConfig g_whiteBallConfig;

// C ��Ÿ�� �������̽�
extern "C" {
    // ���� �޽��� ���޿�
    VISIONTRACKER_API const char* GetLastErrorMessage();

    // ī�޶� �˻� �� ����
    VISIONTRACKER_API int EnumerateCameras(CameraInfo* cameras, int maxCount);
    VISIONTRACKER_API bool ConnectCameraByIndex(int index);
    VISIONTRACKER_API bool DisconnectCamera();
    VISIONTRACKER_API bool IsConnected();
    VISIONTRACKER_API bool GetConnectedCameraInfo(CameraInfo* info);

    // ���Ž� ī�޶� �ʱ�ȭ (deprecated)
    VISIONTRACKER_API bool IsNodeSupported(const char* nodeName);
    VISIONTRACKER_API bool InitCamera();
    VISIONTRACKER_API void CloseCamera();

    // Ʈ���� ��� ���� (Master/Slave)
    VISIONTRACKER_API bool SetTriggerMode(bool enable); // Master(false), Slave(true)

    // ���� ĸó ���� �� �ߴ�
    VISIONTRACKER_API void StartGrab();
    VISIONTRACKER_API void StopGrab();

    // �̹��� ���� ���� �� �ߴ�
    VISIONTRACKER_API bool StartRecording(const char* folderPath);
    VISIONTRACKER_API void StopRecording();

    // �ֽ� ������ ��ȸ
    VISIONTRACKER_API bool GetLatestFrameInfo(int* width, int* height, int* channels);
    VISIONTRACKER_API bool GetLatestFrame(unsigned char* buffer, int bufferSize,
        int* width, int* height, int* channels);

    // ���� ������ ����
    VISIONTRACKER_API double GetCurrentBrightness();
    VISIONTRACKER_API double GetCurrentFps();

    // ī�޶� ����
    VISIONTRACKER_API bool SetFrameRate(float fps);
    VISIONTRACKER_API bool GetFrameRate(float* fps);
    VISIONTRACKER_API bool SetExposureTime(float exposureTime);
    VISIONTRACKER_API bool GetExposureTime(float* exposureTime);
    VISIONTRACKER_API bool EnableGamma(bool enable);
    VISIONTRACKER_API bool SetGamma(float gammaValue);
    VISIONTRACKER_API bool GetGamma(float* gammaValue);
    VISIONTRACKER_API bool SetGain(float gain);
    VISIONTRACKER_API bool GetGain(float* gain);
    VISIONTRACKER_API bool EnableBlackLevel(bool enable);
    VISIONTRACKER_API bool SetBlackLevel(int blackLevelValue);
    VISIONTRACKER_API bool GetBlackLevel(int* blackLevelValue);

    // ROI ����
    VISIONTRACKER_API bool SetROI(int x, int y, int width, int height);

    // ��� �� ����
    VISIONTRACKER_API bool GetWhiteBallInfo(int* centerX, int* centerY, int* radius, int* pointCount);
    VISIONTRACKER_API void SetWhiteBallDetectionConfig(const WhiteBallDetectionConfig& config);
    VISIONTRACKER_API WhiteBallDetectionConfig GetWhiteBallDetectionConfig();

    // ���� ����
    VISIONTRACKER_API void StartTracking();
    VISIONTRACKER_API void StopTracking();
    VISIONTRACKER_API bool IsTrackingActive();
    VISIONTRACKER_API void GetCurrentBallPosition(float* x, float* y, float* radius, bool* found);

    // �� Ž�� ó�� �ð� ��ȸ (�и���)
    VISIONTRACKER_API double GetLastDetectionTimeMs();

    // ī�޶� ����
    VISIONTRACKER_API double GetLastTimestamp();
    VISIONTRACKER_API bool SelectCameraByIP(const char* ipAddress);
    VISIONTRACKER_API bool SelectCameraFromFile(const char* filepath);

    // ===== Logger �������̽� =====
    // Logger �ʱ�ȭ �� ����
    VISIONTRACKER_API bool InitializeLogger(const char* filePath, int logLevel, size_t maxFileSize, int maxBackupFiles);
    VISIONTRACKER_API void ShutdownLogger();

    // �α� ���� ���� (0=Debug, 1=Info, 2=Warning, 3=Error, 4=Critical)
    VISIONTRACKER_API void SetLogLevel(int level);

    // �α� �Լ���
    VISIONTRACKER_API void LogDebug(const char* message);
    VISIONTRACKER_API void LogInfo(const char* message);
    VISIONTRACKER_API void LogWarning(const char* message);
    VISIONTRACKER_API void LogError(const char* message);
    VISIONTRACKER_API void LogCritical(const char* message);

    // �α� �÷���
    VISIONTRACKER_API void FlushLogger();

    // �ܼ� â ����
    VISIONTRACKER_API bool AllocateConsole(const char* title);
    VISIONTRACKER_API void FreeConsoleWindow();
}