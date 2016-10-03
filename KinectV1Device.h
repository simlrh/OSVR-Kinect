#include "stdafx.h"

namespace KinectOsvr {
	class KinectV1Device {
	public:
		KinectV1Device(OSVR_PluginRegContext ctx, INuiSensor* pNuiSensor);
		~KinectV1Device();

		OSVR_ReturnCode update();
		static bool Detect(INuiSensor** ppNuiSensor);

		enum BodyTrackingState {
			CannotBeTracked,
			CanBeTracked,
			ShouldNotBeTracked,
			ShouldBeTracked
		};

		void toggleSeatedMode();
		BodyTrackingState *getBodyStates();
		void setTrackedBody(int i);

		struct ui_thread_data
		{
			std::mutex mutex;
			KinectV1Device *kinect;
			bool end = false;
		};
		static void ui_thread(ui_thread_data& data);
		static INT_PTR CALLBACK DialogProc(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam);
	private:


		void ProcessBody(NUI_SKELETON_FRAME* pSkeletons);
		void IdentifyBodies(NUI_SKELETON_FRAME* pSkeletons);

		osvr::pluginkit::DeviceToken m_dev;
		OSVR_TrackerDeviceInterface m_tracker;
		OSVR_AnalogDeviceInterface m_analog;
		OSVR_ButtonDeviceInterface m_button;

		INuiSensor* m_pNuiSensor;
		HANDLE m_pSkeletonStreamHandle;
		HANDLE m_hNextSkeletonEvent;

		BodyTrackingState m_body_states[NUI_SKELETON_COUNT];
		DWORD m_trackingId;
		int m_trackedBody;
		bool m_trackedBodyChanged;
		Vector4 m_lastTrackedPosition;
		LONGLONG m_lastTrackedTime;

		bool m_firstUpdate;
		OSVR_PoseState m_offset;
		OSVR_PoseState m_kinectPose;

		OSVR_TimeValue m_initializeTime;
		LONGLONG m_initializeOffset;

		std::thread *mThread;
		ui_thread_data mThreadData;

		bool m_seatedMode;
	};
}
