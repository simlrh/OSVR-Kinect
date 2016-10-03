#pragma once

#include <osvr/PluginKit/PluginKit.h>
#include <osvr/PluginKit/TrackerInterfaceC.h>
#include <osvr/PluginKit/AnalogInterfaceC.h>
#include <osvr/PluginKit/ButtonInterfaceC.h>
#include <osvr/Util/EigenInterop.h>

#include <Kinect.h>
#include <NuiApi.h>

#include <thread>
#include <mutex>
#include <map>

#include <Windows.h>
#include <windowsx.h>
#include <tchar.h>
#include "resource.h"

#define _USE_MATH_DEFINES
#include <Math.h>

// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}