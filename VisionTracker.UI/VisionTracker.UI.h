
// VisionTracker.UI.h : main header file for the PROJECT_NAME application
//

#pragma once

#ifndef __AFXWIN_H__
	#error "include 'pch.h' before including this file for PCH"
#endif

#include "resource.h"		// main symbols


// CVisionTrackerUIApp:
// See VisionTracker.UI.cpp for the implementation of this class
//

class CVisionTrackerUIApp : public CWinApp
{
public:
	CVisionTrackerUIApp();

// Overrides
public:
	virtual BOOL InitInstance();

// Implementation

	DECLARE_MESSAGE_MAP()
};

extern CVisionTrackerUIApp theApp;
