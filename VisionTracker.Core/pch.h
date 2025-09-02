// pch.h: This is a precompiled header file.
// Files listed below are compiled only once, improving build performance for future builds.
// This also affects IntelliSense performance, including code completion and many code browsing features.
// However, files listed here are ALL re-compiled if any one of them is updated between builds.
// Do not add files here that you will be updating frequently as this negates the performance advantage.

#ifndef PCH_H
#define PCH_H

// #define _CRT_SECURE_NO_WARNINGS
// #define _CRT_SECURE_NO_DEPRECATE
// #define _CRT_NONSTDC_NO_DEPRECATE

// add headers that you want to pre-compile here
#include "framework.h"

// Suppress C4819 warning for OpenCV headers
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4819)
#endif

#include <opencv2/opencv.hpp>

#ifdef _MSC_VER
#pragma warning(pop)
#endif

// Windows system headers
#include <windows.h>

// C++ standard library
#include <vector>
#include <string>
#include <memory>
#include <atomic>
#include <mutex>
#include <thread>
#include <chrono>
#include <fstream>
#include <regex>
#include <sstream>
#include <algorithm>
#include <iostream>
#include <iomanip>

// C standard library
#include <cstdio>
#include <cstring>
#include <cmath>

#endif //PCH_H