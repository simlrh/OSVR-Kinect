#include "stdafx.h"

namespace KinectOsvr {
	class KinectV2Device {
	public:
		KinectV2Device(OSVR_PluginRegContext ctx, IKinectSensor* pKinectSensor);
		~KinectV2Device();

		OSVR_ReturnCode update();
		static bool Detect(IKinectSensor** ppKinectSensor);
	private:
		void ProcessBody(IBody** ppBodies);
		int addBody(int idx);
		void removeBody(int idx);
		bool firstBody(int channel);

		osvr::pluginkit::DeviceToken m_dev;
		OSVR_TrackerDeviceInterface m_tracker;
		OSVR_AnalogDeviceInterface m_analog;
		OSVR_ButtonDeviceInterface m_button;

		IKinectSensor* m_pKinectSensor;
		ICoordinateMapper*      m_pCoordinateMapper;
		IBodyFrameReader*       m_pBodyFrameReader;

		int m_channels[BODY_COUNT];

		bool m_firstUpdate = true;
		OSVR_PoseState m_offset;
		OSVR_PoseState m_kinectPose;
	};
}