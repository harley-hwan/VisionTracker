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

// Threshold 모드 열거형
enum class ThresholdMode
{
    Fixed,        // 일반 cv::threshold
    Adaptive,     // cv::adaptiveThreshold
    Otsu,         // cv::threshold + OTSU
    AdaptiveOtsu  // (혼용 시)
};

// Adaptive 방법 열거형
enum class AdaptiveMethod
{
    Mean = cv::ADAPTIVE_THRESH_MEAN_C,          // 0
    Gaussian = cv::ADAPTIVE_THRESH_GAUSSIAN_C   // 1
};

// 흰색 공 검출 설정 구조체
struct WhiteBallDetectionConfig
{
    cv::Rect roi;                // 관심 영역
    ThresholdMode thresholdMode; // 일반 / 고급
    int thresholdValue;          // 바이너리 임계값
    int thresholdType;           // cv::THRESH_BINARY 등
    AdaptiveMethod adaptiveMethod; // 고급 모드에서 adaptive 방식
    int blockSize;               // adaptive threshold용 블록 크기 (홀수, 3 이상)
    double C;                    // adaptive threshold용 상수 감산값
    float minRadius;             // 반지름 최소값
    float maxRadius;             // 반지름 최대값
    float minCircularity;        // 원형 유사도 최소
    float maxCircularity;        // 원형 유사도 최대
    bool useTracking;            // 이전 프레임 기반 거리 추적 여부
};

// 카메라 정보 구조체
struct CameraInfo {
    char displayName[256];
    char serialNumber[64];
    char modelName[64];
    char ipAddress[32];
    int deviceType;  // 0: GigE, 1: USB
};

// 전역 설정 외부 참조
extern WhiteBallDetectionConfig g_whiteBallConfig;

// C 스타일 인터페이스
extern "C" {
    // 에러 메시지 전달용
    VISIONTRACKER_API const char* GetLastErrorMessage();

    // 카메라 검색 및 연결
    VISIONTRACKER_API int EnumerateCameras(CameraInfo* cameras, int maxCount);
    VISIONTRACKER_API bool ConnectCameraByIndex(int index);
    VISIONTRACKER_API bool DisconnectCamera();
    VISIONTRACKER_API bool IsConnected();
    VISIONTRACKER_API bool GetConnectedCameraInfo(CameraInfo* info);

    // 레거시 카메라 초기화 (deprecated)
    VISIONTRACKER_API bool IsNodeSupported(const char* nodeName);
    VISIONTRACKER_API bool InitCamera();
    VISIONTRACKER_API void CloseCamera();

    // 트리거 모드 설정 (Master/Slave)
    VISIONTRACKER_API bool SetTriggerMode(bool enable); // Master(false), Slave(true)

    // 영상 캡처 시작 및 중단
    VISIONTRACKER_API void StartGrab();
    VISIONTRACKER_API void StopGrab();

    // 이미지 저장 시작 및 중단
    VISIONTRACKER_API bool StartRecording(const char* folderPath);
    VISIONTRACKER_API void StopRecording();

    // 최신 프레임 조회
    VISIONTRACKER_API bool GetLatestFrameInfo(int* width, int* height, int* channels);
    VISIONTRACKER_API bool GetLatestFrame(unsigned char* buffer, int bufferSize,
        int* width, int* height, int* channels);

    // 현재 프레임 정보
    VISIONTRACKER_API double GetCurrentBrightness();
    VISIONTRACKER_API double GetCurrentFps();

    // 카메라 설정
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

    // ROI 설정
    VISIONTRACKER_API bool SetROI(int x, int y, int width, int height);

    // 흰색 공 검출
    VISIONTRACKER_API bool GetWhiteBallInfo(int* centerX, int* centerY, int* radius, int* pointCount);
    VISIONTRACKER_API void SetWhiteBallDetectionConfig(const WhiteBallDetectionConfig& config);
    VISIONTRACKER_API WhiteBallDetectionConfig GetWhiteBallDetectionConfig();

    // 추적 제어
    VISIONTRACKER_API void StartTracking();
    VISIONTRACKER_API void StopTracking();
    VISIONTRACKER_API bool IsTrackingActive();
    VISIONTRACKER_API void GetCurrentBallPosition(float* x, float* y, float* radius, bool* found);

    // 볼 탐지 처리 시간 조회 (밀리초)
    VISIONTRACKER_API double GetLastDetectionTimeMs();

    // 카메라 선택
    VISIONTRACKER_API double GetLastTimestamp();
    VISIONTRACKER_API bool SelectCameraByIP(const char* ipAddress);
    VISIONTRACKER_API bool SelectCameraFromFile(const char* filepath);

    // ===== Logger 인터페이스 =====
    // Logger 초기화 및 종료
    VISIONTRACKER_API bool InitializeLogger(const char* filePath, int logLevel, size_t maxFileSize, int maxBackupFiles);
    VISIONTRACKER_API void ShutdownLogger();

    // 로그 레벨 설정 (0=Debug, 1=Info, 2=Warning, 3=Error, 4=Critical)
    VISIONTRACKER_API void SetLogLevel(int level);

    // 로깅 함수들
    VISIONTRACKER_API void LogDebug(const char* message);
    VISIONTRACKER_API void LogInfo(const char* message);
    VISIONTRACKER_API void LogWarning(const char* message);
    VISIONTRACKER_API void LogError(const char* message);
    VISIONTRACKER_API void LogCritical(const char* message);

    // 로그 플러시
    VISIONTRACKER_API void FlushLogger();

    // 콘솔 창 제어
    VISIONTRACKER_API bool AllocateConsole(const char* title);
    VISIONTRACKER_API void FreeConsoleWindow();
}