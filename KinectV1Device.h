#include "stdafx.h"

namespace KinectOsvr {
	class KinectV1Device {
	public:
		KinectV1Device(OSVR_PluginRegContext ctx, INuiSensor* pNuiSensor);
		~KinectV1Device();

		OSVR_ReturnCode update();
		static bool Detect(INuiSensor** ppNuiSensor);
	private:
		void ProcessBody(NUI_SKELETON_FRAME* pSkeletons);
		int addBody(int idx);
		void removeBody(int idx);
		bool firstBody(int channel);

		osvr::pluginkit::DeviceToken m_dev;
		OSVR_TrackerDeviceInterface m_tracker;
		OSVR_AnalogDeviceInterface m_analog;
		OSVR_ButtonDeviceInterface m_button;

		INuiSensor* m_pNuiSensor;
		HANDLE m_pSkeletonStreamHandle;
		HANDLE m_hNextSkeletonEvent;

		int m_channels[NUI_SKELETON_COUNT];

		bool m_firstUpdate = true;
		OSVR_PoseState m_offset;
		OSVR_PoseState m_kinectPose;

		OSVR_TimeValue m_initializeTime;
		LONGLONG m_initializeOffset;
	};
}
