#include "pch.h"
//#include <thread>
//#include <chrono>
//#include <mutex>
//#include <atomic>
//#include <string>
//#include <fstream>
//#include <vector>
//#include <regex>
//#include <sstream>
//#include <algorithm>
//#include <Windows.h> 
//#include <opencv2/opencv.hpp>
#include "IVisionTracker.h"

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

    // 카메라 '실제' 프레임 인덱스(콜백 수신 시 1씩 증가)
    static std::atomic<int> g_camFrameIndex{ 0 };

    // 볼 탐색 활성화 여부
    std::atomic<bool> g_trackingActive(false);

    // 이번 세션에서 MOVE에 진입한 적이 있는지
    static bool g_hadMoveInSession = false;

    // CSV 로깅
    std::string g_trajCsvPath;
    std::ofstream g_trajCsv;
    std::mutex g_csvMutex;
    bool g_trajCsvEnabled = false;

    const float kOutlierMaxStep = 100.0f; // 한 프레임에서 허용할 최대 이동량(px)

    bool g_moveTagged = false;      // READY→MOVE로 '처음' 전환했는지 여부
    std::string g_moveTimeTag;      // 파일명에 넣을 시각 태그

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

    // 정지 판정용 변수
    static cv::Point2f g_stillStartCenter = cv::Point2f(0, 0);
    static int g_stillStartFrame = -1;

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

    // DLL 모듈 디렉토리 가져오기
    static std::string GetThisModuleDirA() {
        HMODULE hm = nullptr;
        if (GetModuleHandleExA(
            GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
            GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
            reinterpret_cast<LPCSTR>(&GetThisModuleDirA),
            &hm)) {
            char path[MAX_PATH] = {};
            GetModuleFileNameA(hm, path, MAX_PATH);
            std::string full(path);
            size_t pos = full.find_last_of("\\/");
            return (pos == std::string::npos) ? full : full.substr(0, pos);
        }
        return std::string(".");
    }

    // 로컬 시간 태그 생성
    static std::string NowTagLocal() {
        SYSTEMTIME st;
        GetLocalTime(&st);
        char buf[64];
        sprintf_s(buf, "%04d%02d%02d_%02d%02d%02d_%03d",
            st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
        return std::string(buf);
    }

    // 파일 경로에서 디렉토리명 추출
    static std::string PathDirname(const std::string& p) {
        size_t pos = p.find_last_of("\\/");
        return (pos == std::string::npos) ? std::string(".") : p.substr(0, pos);
    }
}

// 전역 상태 변수
BallTrackingState g_trackingState = BallTrackingState::IDLE;
std::vector<BallTrajectoryEntry> g_ballTrajectory;
int g_trackingFrameCount = 0;
bool g_enableTrackingByDistance = true;
cv::Point2f g_lastKnownPosition = cv::Point2f(0, 0);

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
        g_camFrameIndex++;
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

    if (g_trackingActive) {
        int tmpX = 0, tmpY = 0, tmpR = 0, tmpPts = 0;
        GetWhiteBallInfo(&tmpX, &tmpY, &tmpR, &tmpPts);
        // 결과는 UpdateBallTrackingState()를 통해 궤적/상태 전역에 반영되어 있음
    }
}

// 상태 문자열 변환 함수
static const char* StateToCStr(BallTrackingState s) {
    switch (s) {
    case BallTrackingState::READY: return "READY";
    case BallTrackingState::MOVE:  return "MOVE";
    case BallTrackingState::DONE:  return "DONE";
    case BallTrackingState::FIND:  return "FIND";
    default: return "IDLE";
    }
}

// 공 추적 상태 업데이트 함수
void UpdateBallTrackingState(const cv::Point2f& center, float radius, bool ballFound)
{
    static int lostCount = 0;
    static int stillCount = 0;

    static const int   kStillFramesThreshold = 60;
    static const float kStillEpsPixels = 1.5f;

    switch (g_trackingState)
    {
    case BallTrackingState::IDLE:
        return;

    case BallTrackingState::FIND:
        if (ballFound) {
            lostCount = 0;
            stillCount = 0;
            if (!g_ballTrajectory.empty()) {
                g_ballTrajectory.clear();
                g_trackingFrameCount = 0;
            }

            g_trackingState = BallTrackingState::READY;
            g_lastKnownPosition = center;

            g_ballTrajectory.push_back({ g_camFrameIndex.load(), center, radius, BallTrackingState::READY });

            // CSV 기록 - flush 추가
            if (g_trajCsvEnabled) {
                std::lock_guard<std::mutex> lk(g_csvMutex);
                if (g_trajCsv.is_open()) {
                    g_trajCsv << g_camFrameIndex << ",READY"
                        << "," << std::fixed << std::setprecision(2)
                        << center.x << "," << center.y << "," << radius << "\n";
                    g_trajCsv.flush();  // 즉시 파일에 쓰기
                }
            }
        }
        break;

    case BallTrackingState::READY:
    {
        if (!ballFound) {
            lostCount++;
            if (!g_hadMoveInSession && lostCount > 5) {
                g_trackingState = BallTrackingState::FIND;
                lostCount = 0;
                stillCount = 0;
            }
            else if (lostCount > 5) {
                g_ballTrajectory.push_back({ g_camFrameIndex.load(), cv::Point2f(-1, -1), 0.0f, BallTrackingState::DONE });
                if (g_trajCsvEnabled) {
                    std::lock_guard<std::mutex> lk(g_csvMutex);
                    if (g_trajCsv.is_open()) {
                        g_trajCsv << g_camFrameIndex << ",DONE,-1,-1,0\n";
                        g_trajCsv.flush();
                    }
                }
                g_trackingState = BallTrackingState::DONE;
            }
        }
        else {
            lostCount = 0;
            float dist = static_cast<float>(cv::norm(center - g_lastKnownPosition));

            if (dist > 2.2f) {
                cv::Point2f refPt = g_lastKnownPosition;
                for (int i = static_cast<int>(g_ballTrajectory.size()) - 1; i >= 0; --i) {
                    const auto& e = g_ballTrajectory[i];
                    if (e.center.x >= 0 && e.center.y >= 0) {
                        refPt = e.center;
                        break;
                    }
                }
                float distRef = static_cast<float>(cv::norm(center - refPt));

                if (distRef >= kOutlierMaxStep) {
                    lostCount++;
                    stillCount = 0;
                }
                else {
                    g_trackingState = BallTrackingState::MOVE;
                    g_hadMoveInSession = true;
                    stillCount = 0;

                    // MOVE 시점에 CSV 파일명 변경
                    if (!g_moveTagged && g_trajCsvEnabled) {
                        std::lock_guard<std::mutex> lk(g_csvMutex);

                        if (g_trajCsv.is_open()) {
                            g_trajCsv.close();
                        }

                        g_moveTimeTag = NowTagLocal();
                        g_moveTagged = true;

                        // 같은 폴더 내에서 파일명만 변경
                        std::string oldPath = g_trajCsvPath;
                        size_t lastSlash = oldPath.find_last_of("\\/");
                        std::string folder = (lastSlash != std::string::npos) ?
                            oldPath.substr(0, lastSlash) : ".";

                        std::string finalPath = folder + "\\trajectory_MOVE_" + g_moveTimeTag + ".csv";

                        // 파일 이동
                        if (!MoveFileExA(oldPath.c_str(), finalPath.c_str(), MOVEFILE_REPLACE_EXISTING)) {
                            printf("CSV 파일 이름 변경 실패: %s -> %s\n", oldPath.c_str(), finalPath.c_str());
                            finalPath = oldPath;  // 실패 시 원래 경로 유지
                        }

                        g_trajCsvPath = finalPath;
                        g_trajCsv.open(g_trajCsvPath, std::ios::out | std::ios::app);
                        g_trajCsvEnabled = g_trajCsv.is_open();
                    }

                    g_ballTrajectory.push_back({ g_camFrameIndex.load(), center, radius, BallTrackingState::MOVE });
                    if (g_trajCsvEnabled) {
                        std::lock_guard<std::mutex> lk(g_csvMutex);
                        if (g_trajCsv.is_open()) {
                            g_trajCsv << g_camFrameIndex << ",MOVE,"
                                << std::fixed << std::setprecision(2)
                                << center.x << "," << center.y << "," << radius << "\n";
                            g_trajCsv.flush();
                        }
                    }
                    g_lastKnownPosition = center;
                }
            }
            else {
                g_ballTrajectory.push_back({ g_camFrameIndex.load(), center, radius, BallTrackingState::READY });
                if (g_trajCsvEnabled) {
                    std::lock_guard<std::mutex> lk(g_csvMutex);
                    if (g_trajCsv.is_open()) {
                        g_trajCsv << g_camFrameIndex << ",READY"
                            << "," << std::fixed << std::setprecision(2)
                            << center.x << "," << center.y << "," << radius << "\n";
                        g_trajCsv.flush();
                    }
                }
            }
        }
    }
    break;

    case BallTrackingState::MOVE:
    {
        if (ballFound) {
            cv::Point2f refPt = g_lastKnownPosition;
            for (int i = (int)g_ballTrajectory.size() - 1; i >= 0; --i) {
                const auto& e = g_ballTrajectory[i];
                if (e.center.x >= 0 && e.center.y >= 0) {
                    refPt = e.center;
                    break;
                }
            }

            float dist = static_cast<float>(cv::norm(center - refPt));
            const bool isOutlier = (dist >= kOutlierMaxStep);

            if (!isOutlier) {
                if (dist <= kStillEpsPixels) {
                    if (stillCount == 0) {
                        g_stillStartCenter = center;
                        g_stillStartFrame = g_camFrameIndex;
                    }
                    stillCount++;
                }
                else {
                    stillCount = 0;
                }

                if (stillCount >= kStillFramesThreshold) {
                    const int   kDriftWindowFrames = 20;
                    const float kDriftCancelThresh = kStillEpsPixels * 3.0f;
                    const int   kForceDoneExtra = 24;

                    bool cancelDoneByDrift = false;

                    if (g_stillStartFrame >= 0 && (g_camFrameIndex - g_stillStartFrame) >= kDriftWindowFrames) {
                        float driftFromStart = static_cast<float>(cv::norm(center - g_stillStartCenter));
                        if (driftFromStart > kDriftCancelThresh) {
                            cancelDoneByDrift = true;
                        }
                    }

                    bool forceDone = (stillCount >= (kStillFramesThreshold + kForceDoneExtra));

                    if (cancelDoneByDrift && !forceDone) {
                        stillCount = 0;
                        g_stillStartFrame = -1;

                        g_ballTrajectory.push_back({ g_camFrameIndex.load(), center, radius, BallTrackingState::MOVE });
                        g_lastKnownPosition = center;
                        lostCount = 0;

                        if (g_trajCsvEnabled) {
                            std::lock_guard<std::mutex> lk(g_csvMutex);
                            if (g_trajCsv.is_open()) {
                                g_trajCsv << g_camFrameIndex << ",MOVE,"
                                    << std::fixed << std::setprecision(2)
                                    << center.x << "," << center.y << "," << radius << "\n";
                                g_trajCsv.flush();
                            }
                        }
                    }
                    else {
                        g_ballTrajectory.push_back({ g_camFrameIndex.load(), cv::Point2f(-1, -1), 0.0f, BallTrackingState::DONE });
                        if (g_trajCsvEnabled) {
                            std::lock_guard<std::mutex> lk(g_csvMutex);
                            if (g_trajCsv.is_open()) {
                                g_trajCsv << g_camFrameIndex << ",DONE,-1,-1,0\n";
                                g_trajCsv.flush();
                            }
                        }
                        g_trackingState = BallTrackingState::DONE;
                    }
                }
                else {
                    g_ballTrajectory.push_back({ g_camFrameIndex.load(), center, radius, BallTrackingState::MOVE });
                    g_lastKnownPosition = center;
                    lostCount = 0;

                    if (g_trajCsvEnabled) {
                        std::lock_guard<std::mutex> lk(g_csvMutex);
                        if (g_trajCsv.is_open()) {
                            g_trajCsv << g_camFrameIndex << ",MOVE,"
                                << std::fixed << std::setprecision(2)
                                << center.x << "," << center.y << "," << radius << "\n";
                            g_trajCsv.flush();
                        }
                    }
                }
            }
            else {
                lostCount++;
                stillCount = 0;

                if (lostCount >= 15) {
                    g_ballTrajectory.push_back({ g_camFrameIndex.load(), cv::Point2f(-1, -1), 0.0f, BallTrackingState::DONE });
                    if (g_trajCsvEnabled) {
                        std::lock_guard<std::mutex> lk(g_csvMutex);
                        if (g_trajCsv.is_open()) {
                            g_trajCsv << g_camFrameIndex << ",DONE,-1,-1,0\n";
                            g_trajCsv.flush();
                        }
                    }
                    g_trackingState = BallTrackingState::DONE;
                }
            }
        }
        else {
            lostCount++;
            stillCount = 0;

            if (lostCount >= 15) {
                g_ballTrajectory.push_back({ g_camFrameIndex.load(), cv::Point2f(-1, -1), 0.0f, BallTrackingState::DONE });
                if (g_trajCsvEnabled) {
                    std::lock_guard<std::mutex> lk(g_csvMutex);
                    if (g_trajCsv.is_open()) {
                        g_trajCsv << g_camFrameIndex << ",DONE,-1,-1,0\n";
                        g_trajCsv.flush();
                    }
                }
                g_trackingState = BallTrackingState::DONE;
            }
        }
    }
    break;

    case BallTrackingState::DONE:
    {
        if (g_trajCsvEnabled) {
            std::lock_guard<std::mutex> lk(g_csvMutex);
            if (g_trajCsv.is_open()) {
                g_trajCsv.close();
                printf("CSV 파일 저장 완료: %s\n", g_trajCsvPath.c_str());
            }
            g_trajCsvEnabled = false;
        }

        g_moveTagged = false;
        g_moveTimeTag.clear();
        lostCount = 0;
        stillCount = 0;
    }
    break;
    }

    g_trackingFrameCount++;
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

// 기존 InitCamera - deprecated
extern "C" VISIONTRACKER_API bool InitCamera()
{
    printf("Warning: InitCamera() is deprecated. Use EnumerateCameras() and ConnectCameraByIndex() instead.\n");
    g_lastError.clear();

    // 카메라 열거
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (nRet != MV_OK || stDeviceList.nDeviceNum == 0) {
        g_lastError = "디바이스 나열 실패 또는 장치 없음.";
        return false;
    }

    MV_CC_DEVICE_INFO* pDeviceInfo = nullptr;

    if (!g_selectedIp.empty()) {
        for (unsigned int i = 0; i < stDeviceList.nDeviceNum; ++i) {
            MV_CC_DEVICE_INFO* pInfo = stDeviceList.pDeviceInfo[i];
            if (pInfo->nTLayerType == MV_GIGE_DEVICE) {
                const MV_GIGE_DEVICE_INFO& info = pInfo->SpecialInfo.stGigEInfo;
                std::string deviceIp = ConvertIp(info.nCurrentIp);
                if (g_selectedIp == deviceIp) {
                    pDeviceInfo = pInfo;
                    break;
                }
            }
        }
    }
    else {
        pDeviceInfo = stDeviceList.pDeviceInfo[0];
    }

    if (!pDeviceInfo) {
        g_lastError = "일치하는 디바이스 정보를 찾지 못했습니다.";
        return false;
    }

    // 이하 기존 코드와 동일...
    return ConnectCameraByIndex(0);
}

extern "C" VISIONTRACKER_API void CloseCamera()
{
    DisconnectCamera();
}

// 이하 모든 함수들은 기존과 동일
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

// 나머지 모든 함수들 (StartGrab, StopGrab, 등등)은 기존과 완전히 동일
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
    if (!g_trackingActive) return false;

    std::lock_guard<std::mutex> lock(g_frameMutex);
    if (g_latestFrame.empty() || !centerX || !centerY || !radius || !pointCount)
        return false;

    cv::Mat gray, bin;
    if (g_latestFrame.channels() == 3)
        cv::cvtColor(g_latestFrame, gray, cv::COLOR_BGR2GRAY);
    else
        gray = g_latestFrame;

    cv::Rect roiRect = g_whiteBallConfig.roi;
    if (roiRect.width == 0 || roiRect.height == 0) {
        roiRect = cv::Rect(0, 0, gray.cols, gray.rows);
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
    const bool useTracking = g_whiteBallConfig.useTracking;

    int bestIdx = -1;
    float bestDistance = FLT_MAX;
    cv::Point2f bestCenter;
    float bestRadius = 0;
    int bestPoints = 0;

    static cv::Point2f prevCenter(-1, -1);

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

        center.x += roiRect.x;
        center.y += roiRect.y;

        float distance = static_cast<float>(useTracking && prevCenter.x >= 0 ? cv::norm(center - prevCenter) : 0);

        if (useTracking) {
            if (distance < bestDistance) {
                bestIdx = (int)i;
                bestDistance = distance;
                bestCenter = center;
                bestRadius = radius;
                bestPoints = (int)contours[i].size();
            }
        }
        else {
            if ((int)contours[i].size() > bestPoints) {
                bestIdx = (int)i;
                bestCenter = center;
                bestRadius = radius;
                bestPoints = (int)contours[i].size();
            }
        }
    }

    if (bestIdx == -1) {
        UpdateBallTrackingState(cv::Point2f(), 0.0f, false);
        return false;
    }

    *centerX = static_cast<int>(bestCenter.x);
    *centerY = static_cast<int>(bestCenter.y);
    *radius = static_cast<int>(bestRadius);
    *pointCount = bestPoints;

    prevCenter = bestCenter;

    if (centerX && centerY && radius && pointCount && *pointCount > 0) {
        cv::Point2f pt((float)*centerX, (float)*centerY);
        UpdateBallTrackingState(pt, (float)*radius, true);
    }
    else {
        UpdateBallTrackingState(cv::Point2f(), 0.0f, false);
    }

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

extern "C" VISIONTRACKER_API void SetBallTrackingStateFind()
{
    g_trackingState = BallTrackingState::FIND;
    g_ballTrajectory.clear();
    g_trackingFrameCount = 0;
    g_lastKnownPosition = cv::Point2f(0, 0);
}

extern "C" VISIONTRACKER_API BallTrackingState GetBallTrackingState()
{
    return g_trackingState;
}

extern "C" VISIONTRACKER_API int GetBallTrajectoryCount()
{
    return (int)g_ballTrajectory.size();
}

extern "C" VISIONTRACKER_API bool GetBallTrajectoryAt(int index, float* x, float* y, float* r, int* frame)
{
    if (index < 0 || index >= (int)g_ballTrajectory.size())
        return false;
    const auto& entry = g_ballTrajectory[index];
    *x = entry.center.x;
    *y = entry.center.y;
    *r = entry.radius;
    *frame = entry.frameIndex;
    return true;
}

extern "C" VISIONTRACKER_API bool GetBallTrajectoryAtEx(int index, float* x, float* y, float* r, int* frame, int* state)
{
    if (index < 0 || index >= (int)g_ballTrajectory.size())
        return false;
    const auto& e = g_ballTrajectory[index];
    if (x) *x = e.center.x;
    if (y) *y = e.center.y;
    if (r) *r = e.radius;
    if (frame) *frame = e.frameIndex;
    if (state) *state = static_cast<int>(e.state);
    return true;
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
    g_hadMoveInSession = false;

    SetBallTrackingStateFind();

    {
        std::lock_guard<std::mutex> lk(g_csvMutex);

        if (g_trajCsv.is_open()) {
            g_trajCsv.close();
        }

        if (g_trajCsvPath.empty()) {
            char currentDir[MAX_PATH];
            GetCurrentDirectoryA(MAX_PATH, currentDir);
            std::string baseDir(currentDir);

            std::string dataFolder = baseDir + "\\TrajectoryData";
            CreateDirectoryA(dataFolder.c_str(), NULL);

            SYSTEMTIME st;
            GetLocalTime(&st);
            char dateFolder[64];
            sprintf_s(dateFolder, "%04d%02d%02d", st.wYear, st.wMonth, st.wDay);
            std::string fullFolder = dataFolder + "\\" + dateFolder;
            CreateDirectoryA(fullFolder.c_str(), NULL);

            std::string tag = NowTagLocal();
            g_trajCsvPath = fullFolder + "\\trajectory_start_" + tag + ".csv";

            printf("CSV 저장 폴더: %s\n", fullFolder.c_str());
        }

        g_moveTagged = false;
        g_moveTimeTag.clear();

        g_trajCsv.open(g_trajCsvPath, std::ios::out | std::ios::trunc);
        if (g_trajCsv.is_open()) {
            g_trajCsv << "frame,state,x,y,radius\n";
            g_trajCsv.flush();
            g_trajCsvEnabled = true;
            printf("CSV 파일 생성: %s\n", g_trajCsvPath.c_str());
        }
        else {
            g_trajCsvEnabled = false;
            printf("CSV 파일 생성 실패: %s\n", g_trajCsvPath.c_str());
        }
    }

    int x = 0, y = 0, r = 0, pts = 0;
    GetWhiteBallInfo(&x, &y, &r, &pts);
}

extern "C" VISIONTRACKER_API void StopTracking()
{
    g_trackingActive = false;

    {
        std::lock_guard<std::mutex> lk(g_csvMutex);
        if (g_trajCsv.is_open()) {
            g_trajCsv.flush();
            g_trajCsv.close();
            printf("추적 중지 - CSV 파일 닫기: %s\n", g_trajCsvPath.c_str());
        }
        g_trajCsvEnabled = false;
    }

    g_trackingState = BallTrackingState::IDLE;
    g_moveTagged = false;
    g_moveTimeTag.clear();
}

extern "C" VISIONTRACKER_API bool IsTrackingActive()
{
    return g_trackingActive.load();
}

extern "C" VISIONTRACKER_API void SetTrajectoryCsvPath(const char* path)
{
    std::lock_guard<std::mutex> lk(g_csvMutex);
    g_trajCsvPath = (path ? path : "");
}

extern "C" VISIONTRACKER_API const char* GetCurrentCsvPath()
{
    return g_trajCsvPath.c_str();
}