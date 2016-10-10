#include "stdafx.h"
#include <Kinect.h>

namespace KinectOsvr {
	class KinectV2Device {
	public:
		KinectV2Device(OSVR_PluginRegContext ctx, IKinectSensor* pKinectSensor);
		~KinectV2Device();

		enum BodyTrackingState {
			CannotBeTracked,
			CanBeTracked,
			ShouldNotBeTracked,
			ShouldBeTracked
		};

		OSVR_ReturnCode update();
		static bool Detect(IKinectSensor** ppKinectSensor);

		BodyTrackingState *getBodyStates();
		void setTrackedBody(int i);
		void recenter();

		struct ui_thread_data
		{
			std::mutex mutex;
			KinectV2Device *kinect;
			bool end = false;
		};
		static void ui_thread(ui_thread_data& data);
		static INT_PTR CALLBACK DialogProc(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam);
	private:
		void IdentifyBodies(IBody** ppBodies, OSVR_TimeValue* timeValue);
		void ProcessBody(IBody** ppBodies, OSVR_TimeValue* timeValue);

		osvr::pluginkit::DeviceToken m_dev;
		OSVR_TrackerDeviceInterface m_tracker;
		OSVR_AnalogDeviceInterface m_analog;
		OSVR_ButtonDeviceInterface m_button;

		IKinectSensor* m_pKinectSensor;
		ICoordinateMapper*      m_pCoordinateMapper;
		IBodyFrameReader*       m_pBodyFrameReader;

		bool m_firstUpdate = true;
		OSVR_PoseState m_offset;
		OSVR_PoseState m_kinectPose;

		OSVR_TimeValue m_initializeTime;
		INT64 m_initializeOffset = 0;

		BodyTrackingState m_body_states[BODY_COUNT];
		UINT64 m_trackingId;
		int m_trackedBody;
		bool m_trackedBodyChanged;
		CameraSpacePoint m_lastTrackedPosition;
		OSVR_TimeValue m_lastTrackedTime;

		std::thread *mThread;
		ui_thread_data mThreadData;
	};
}