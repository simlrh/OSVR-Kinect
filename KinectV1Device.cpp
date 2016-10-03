#include "KinectV1Device.h"
#include "KinectMath.h"

// Generated JSON header file
#include "je_nourish_kinectv1_json.h"

#include <iostream>

namespace KinectOsvr {

	std::map<HWND, KinectV1Device*> windowMap;


	typedef HRESULT(_stdcall *NuiGetSensorCountType)(int*);
	typedef HRESULT(_stdcall *NuiCreateSensorByIndexType)(int, INuiSensor**);
	typedef HRESULT(_stdcall *NuiSkeletonCalculateBoneOrientationsType)(NUI_SKELETON_DATA*, NUI_SKELETON_BONE_ORIENTATION*);

	NuiGetSensorCountType NuiGetSensorCount;
	NuiCreateSensorByIndexType NuiCreateSensorByIndex;
	NuiSkeletonCalculateBoneOrientationsType NuiSkeletonCalculateBoneOrientations;

	KinectV1Device::KinectV1Device(OSVR_PluginRegContext ctx, INuiSensor* pNuiSensor) : m_pNuiSensor(pNuiSensor) {
		m_trackingId = m_trackedBody = -1;
		m_lastTrackedPosition.x = m_lastTrackedPosition.y = m_lastTrackedPosition.z = 0;
		m_lastTrackedTime = 0;
		m_firstUpdate = true;
		m_trackedBodyChanged = false;

		for (int i = 0; i < NUI_SKELETON_COUNT; i++) {
			m_body_states[i] = CannotBeTracked;
		}

		HRESULT hr;

		// Initialize the Kinect and specify that we'll be using skeleton
		hr = m_pNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_SKELETON);

		if (SUCCEEDED(hr))
		{
			// Create an event that will be signaled when skeleton data is available
			m_hNextSkeletonEvent = CreateEventW(NULL, TRUE, FALSE, NULL);

			// Open a skeleton stream to receive skeleton data
			hr = m_pNuiSensor->NuiSkeletonTrackingEnable(m_hNextSkeletonEvent, 0);

			m_pNuiSensor->NuiSkeletonTrackingEnable(m_hNextSkeletonEvent, 0);
		}

		mThreadData.kinect = this;
		mThread = new std::thread(KinectV1Device::ui_thread, std::ref(mThreadData));

		/// Create the initialization options
		OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);

		osvrDeviceTrackerConfigure(opts, &m_tracker);
		osvrDeviceAnalogConfigure(opts, &m_analog, 20);

		/// Create the device token with the options
		m_dev.initAsync(ctx, "KinectV1", opts);

		/// Send JSON descriptor
		m_dev.sendJsonDescriptor(je_nourish_kinectv1_json);

		/// Register update callback
		m_dev.registerUpdateCallback(this);
	};

	OSVR_ReturnCode KinectV1Device::update() {

		NUI_SKELETON_FRAME skeletonFrame = { 0 };

		HRESULT hr = m_pNuiSensor->NuiSkeletonGetNextFrame(0, &skeletonFrame);
		if (FAILED(hr))
		{
			return OSVR_RETURN_SUCCESS;
		}

		// smooth out the skeleton data
		// m_pNuiSensor->NuiTransformSmooth(&skeletonFrame, NULL);

		ProcessBody(&skeletonFrame);

		return OSVR_RETURN_SUCCESS;
	};

	void KinectV1Device::toggleSeatedMode() {
		m_seatedMode = !m_seatedMode;
		m_pNuiSensor->NuiSkeletonTrackingEnable(m_hNextSkeletonEvent, m_seatedMode ? NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT : 0);
	}

	KinectV1Device::BodyTrackingState* KinectV1Device::getBodyStates() {
		return m_body_states;
	}

	void KinectV1Device::setTrackedBody(int i)
	{
		m_trackedBody = i;
		m_trackedBodyChanged = true;
	}

	void KinectV1Device::ui_thread(ui_thread_data& data)
	{
		MSG msg;
		BOOL ret;
		HWND hDlg;
		HINSTANCE hInst;

		hInst = GetModuleHandle("je_nourish_kinect.dll");
		hDlg = CreateDialogParam(hInst, MAKEINTRESOURCE(IDD_DIALOG1), 0, DialogProc, 0);
		ShowWindow(hDlg, SW_RESTORE);
		UpdateWindow(hDlg);

		windowMap[hDlg] = data.kinect;

		KinectV1Device::BodyTrackingState previousStates[NUI_SKELETON_COUNT];
		KinectV1Device::BodyTrackingState* bodyStates = data.kinect->getBodyStates();
		for (int i = 0; i < NUI_SKELETON_COUNT; i++) {
			previousStates[i] = bodyStates[i];
		}

		do {
			ret = PeekMessage(&msg, 0, 0, 0, PM_REMOVE);

			if (ret) {
				if (!IsDialogMessage(hDlg, &msg)) {
					TranslateMessage(&msg);
					DispatchMessage(&msg);
				}
			}

			bool redraw = false;
			int bodies = 0;

			for (int i = 0; i < NUI_SKELETON_COUNT; i++) {
				if (bodyStates[i] != CannotBeTracked) {
					bodies++;
				}
				if (bodyStates[i] != previousStates[i]) {
					redraw = true;
					SendDlgItemMessage(hDlg, IDC_RADIO1 + i, WM_ENABLE, true, 0);
					EnableWindow(GetDlgItem(hDlg, IDC_RADIO1 + i), bodyStates[i] != CannotBeTracked);
					if (bodyStates[i] == ShouldBeTracked) {
						CheckRadioButton(hDlg, IDC_RADIO1, IDC_RADIO6, IDC_RADIO1 + i);
					}
					previousStates[i] = bodyStates[i];
				}
			}
			if (redraw) {
				if (bodies == 1) {
					SetDlgItemText(hDlg, IDC_STATIC1, "1 body detected.");
				}
				else {
					SetDlgItemText(hDlg, IDC_STATIC1, (std::to_string(bodies) + " bodies detected.").c_str());
				}
				UpdateWindow(hDlg);
			}

			bodyStates = data.kinect->getBodyStates();

		} while (true);

		DestroyWindow(hDlg);
	}

	INT_PTR CALLBACK KinectV1Device::DialogProc(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam)
	{
		switch (uMsg)
		{
		case WM_COMMAND:
			switch (LOWORD(wParam))
			{
			case IDC_CHECK1:
				if (BN_CLICKED == HIWORD(wParam)) {
					windowMap[hDlg]->toggleSeatedMode();
				}
				break;
			case IDC_RADIO1:
			case IDC_RADIO2:
			case IDC_RADIO3:
			case IDC_RADIO4:
			case IDC_RADIO5:
			case IDC_RADIO6:
				if (BST_CHECKED == Button_GetCheck(GetDlgItem(hDlg, LOWORD(wParam)))) {
					windowMap[hDlg]->setTrackedBody(LOWORD(wParam) - IDC_RADIO1);
				}
				break;
			}
			break;

		case WM_CLOSE:
			DestroyWindow(hDlg);
			return TRUE;

		case WM_DESTROY:
			PostQuitMessage(0);
			return TRUE;
		}

		return FALSE;
	}

	void setupOffset(OSVR_PoseState* offset, Vector4* joint, NUI_SKELETON_BONE_ORIENTATION* jointOrientation) {
		osvrVec3SetX(&(offset->translation), joint->x);
		osvrVec3SetY(&(offset->translation), joint->y);
		osvrVec3SetZ(&(offset->translation), joint->z);

		Vector4 orientation = jointOrientation->absoluteRotation.rotationQuaternion;

		osvrQuatSetX(&(offset->rotation), orientation.x);
		osvrQuatSetY(&(offset->rotation), orientation.y);
		osvrQuatSetZ(&(offset->rotation), orientation.z);
		osvrQuatSetW(&(offset->rotation), orientation.w);

		Eigen::Quaterniond q = osvr::util::fromQuat(offset->rotation);
		osvr::util::toQuat(q.inverse(), offset->rotation);
	}

	void KinectV1Device::ProcessBody(NUI_SKELETON_FRAME* pSkeletons) {

		LONGLONG timestamp = pSkeletons->liTimeStamp.QuadPart;
		if (m_initializeOffset == 0) {
			osvrTimeValueGetNow(&m_initializeTime);
			m_initializeOffset = timestamp;
		}
		timestamp = timestamp - m_initializeOffset;

		OSVR_TimeValue timeValue;
		timeValue.seconds = timestamp / 1000;
		timeValue.microseconds = (timestamp % 1000) * 1000;
		osvrTimeValueSum(&timeValue, &m_initializeTime);

		IdentifyBodies(pSkeletons);

		if (m_trackedBody >= 0)
		{
			NUI_SKELETON_DATA skeleton = pSkeletons->SkeletonData[m_trackedBody];
			m_lastTrackedPosition = skeleton.Position;
			m_lastTrackedTime = pSkeletons->liTimeStamp.QuadPart;

			if (skeleton.eTrackingState != NUI_SKELETON_TRACKED) return;

			Vector4* joints = skeleton.SkeletonPositions;
			NUI_SKELETON_BONE_ORIENTATION jointOrientations[NUI_SKELETON_POSITION_COUNT];

			HRESULT hr = NuiSkeletonCalculateBoneOrientations(&skeleton, jointOrientations);

			if (SUCCEEDED(hr)) {

				OSVR_PoseState poseState;
				OSVR_Vec3 translation;
				OSVR_Quaternion rotation;

				osvrVec3Zero(&translation);
				osvrQuatSetIdentity(&rotation);

				if (m_firstUpdate) {
					m_firstUpdate = false;

					setupOffset(&m_offset, &joints[NUI_SKELETON_POSITION_HEAD], &jointOrientations[NUI_SKELETON_POSITION_HEAD]);

					osvrVec3SetX(&(m_kinectPose.translation), -joints[NUI_SKELETON_POSITION_HEAD].x);
					osvrVec3SetY(&(m_kinectPose.translation), -joints[NUI_SKELETON_POSITION_HEAD].y);
					osvrVec3SetZ(&(m_kinectPose.translation), -joints[NUI_SKELETON_POSITION_HEAD].z);

					Eigen::Quaterniond quaternion(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()));
					osvr::util::toQuat(quaternion, m_kinectPose.rotation);
				}

				osvrDeviceTrackerSendPoseTimestamped(m_dev, m_tracker, &m_kinectPose, 21, &timeValue);

				for (int j = 0; j < NUI_SKELETON_POSITION_COUNT; ++j)
				{
					osvrVec3SetX(&translation, joints[j].x);
					osvrVec3SetY(&translation, joints[j].y);
					osvrVec3SetZ(&translation, joints[j].z);

					Vector4 orientation = jointOrientations[j].absoluteRotation.rotationQuaternion;

					osvrQuatSetX(&rotation, orientation.x);
					osvrQuatSetY(&rotation, orientation.y);
					osvrQuatSetZ(&rotation, orientation.z);
					osvrQuatSetW(&rotation, orientation.w);

					// Rotate hand orientation to something more useful for OSVR
					if (j == NUI_SKELETON_POSITION_HAND_LEFT || j == NUI_SKELETON_POSITION_HAND_RIGHT) {
						boneSpaceToWorldSpace(&rotation);
					}

					poseState.translation = translation;
					poseState.rotation = rotation;
					applyOffset(&m_offset, &poseState);
					// Send pose
					osvrDeviceTrackerSendPoseTimestamped(m_dev, m_tracker, &poseState, j, &timeValue);

					OSVR_AnalogState confidence = 0;
					switch (skeleton.eSkeletonPositionTrackingState[j]) {
					case NUI_SKELETON_POSITION_TRACKED:
						confidence = 1;
						break;
					case NUI_SKELETON_POSITION_INFERRED:
						confidence = 0.5;
						break;
					default:
					case NUI_SKELETON_POSITION_NOT_TRACKED:
						confidence = 0;
						break;
					}
					// Tracking confidence for use in smoothing plugins
					osvrDeviceAnalogSetValueTimestamped(m_dev, m_analog, confidence, j, &timeValue);
				}
			}
		}
	};

	void KinectV1Device::IdentifyBodies(NUI_SKELETON_FRAME* pSkeletons) {
		if (m_trackedBody >= 0) { // We're tracking a body
			if (m_trackedBodyChanged) {
				m_trackingId = pSkeletons->SkeletonData[m_trackedBody].dwTrackingID;
				m_trackedBodyChanged = false;
			}
			else {
				for (int i = 0; i < NUI_SKELETON_COUNT; ++i) {
					if (pSkeletons->SkeletonData[i].dwTrackingID == m_trackingId) {
						m_trackedBody = i;
						break;
					}
				}
			}

			DWORD trackingId;

			switch (pSkeletons->SkeletonData[m_trackedBody].eTrackingState) {
			case NUI_SKELETON_POSITION_ONLY: // Keep tracking same body, discount other bodies
			case NUI_SKELETON_TRACKED:
				for (int i = 0; i < NUI_SKELETON_COUNT; ++i) {
					trackingId = pSkeletons->SkeletonData[i].dwTrackingID;
					if (trackingId == m_trackingId) continue;

					switch (pSkeletons->SkeletonData[i].eTrackingState) {
					case NUI_SKELETON_NOT_TRACKED:
						m_body_states[i] = CannotBeTracked;
						break;
					case NUI_SKELETON_POSITION_ONLY:
					case NUI_SKELETON_TRACKED:
						m_body_states[i] = ShouldNotBeTracked;
						break;
					}
				}
				return;
			case NUI_SKELETON_NOT_TRACKED: // We've lost tracking
				m_body_states[m_trackedBody] = CannotBeTracked;
				m_trackedBody = -1;
				m_trackingId = -1;
				break;
			}
		}

		// Lost tracking or haven't started yet
		m_trackedBody = m_trackingId = -1;
		int candidates = 0;
		double confidence[NUI_SKELETON_COUNT];
		double timeConfidence = (pSkeletons->liTimeStamp.QuadPart - m_lastTrackedTime) / 15000.0; // If we lose tracking for a few seconds, just pick whoever's visible

		Vector4 position;
		float distanceFromLastPosition;
		float distanceConfidence;
		for (int i = 0; i < NUI_SKELETON_COUNT; ++i)
		{
			DWORD trackingId = pSkeletons->SkeletonData[i].dwTrackingID;
			NUI_SKELETON_TRACKING_STATE trackingState = pSkeletons->SkeletonData[i].eTrackingState;
			BodyTrackingState myTrackingState = m_body_states[i];

			confidence[i] = 0.0f;

			switch (trackingState) {
			case NUI_SKELETON_NOT_TRACKED:
				m_body_states[i] = CannotBeTracked;
				break;
			case NUI_SKELETON_POSITION_ONLY:
			case NUI_SKELETON_TRACKED:
				switch (myTrackingState) {
				case CannotBeTracked:
				case CanBeTracked:
					m_body_states[i] = CanBeTracked;
					candidates++;

					position = pSkeletons->SkeletonData[i].Position;
					distanceFromLastPosition = sqrt(pow(position.x - m_lastTrackedPosition.x, 2) +
						pow(position.y - m_lastTrackedPosition.y, 2) + pow(position.z - m_lastTrackedPosition.z, 2));
					distanceConfidence = 1.0f - distanceFromLastPosition / 7.0f; // Approx largest possible distance in playspace
					confidence[i] = distanceConfidence + timeConfidence;

					break;
				case ShouldNotBeTracked: // Ignore bodies we've previously ruled out
					break;
				case ShouldBeTracked: // Shouldn't be possible at this point
					break;
				}
				break;
			}
		}

		switch (candidates) {
		case 0: // No bodies found
			break;
		case 1: // Only 1 candidate (lets still wait until confidence is good enough)
		default: // Multiple possible bodies, choose based on last known position
			double bestConfidence = 0.0;
			for (int i = 0; i < NUI_SKELETON_COUNT; ++i) {
				if (m_body_states[i] == CanBeTracked) {
					if (confidence[i] > bestConfidence) {
						bestConfidence = confidence[i];
						m_trackedBody = i;
						m_trackingId = pSkeletons->SkeletonData[i].dwTrackingID;
					}
				}
			}
			if (bestConfidence > 0.75) {
				for (int i = 0; i < NUI_SKELETON_COUNT; ++i) {
					if (i == m_trackedBody) {
						m_body_states[i] = ShouldBeTracked;
					}
					else if (m_body_states[i] == CanBeTracked) {
						m_body_states[i] = ShouldNotBeTracked;
					}
				}
			}
			else {
				m_trackedBody = m_trackingId = -1;
			}
			break;
		}
	}

	bool KinectV1Device::Detect(INuiSensor** ppNuiSensor) {

		HINSTANCE hinstLib = LoadLibrary(TEXT("Kinect10.dll"));
		if (hinstLib == NULL) return false;

		NuiGetSensorCount = (NuiGetSensorCountType)GetProcAddress(hinstLib, "NuiGetSensorCount");
		NuiCreateSensorByIndex = (NuiCreateSensorByIndexType)GetProcAddress(hinstLib, "NuiCreateSensorByIndex");
		NuiSkeletonCalculateBoneOrientations = (NuiSkeletonCalculateBoneOrientationsType)GetProcAddress(hinstLib, "NuiSkeletonCalculateBoneOrientations");

		if (NuiGetSensorCount == NULL || NuiCreateSensorByIndex == NULL || NuiSkeletonCalculateBoneOrientations == NULL) return false;

		int iSensorCount = 0;
		HRESULT hr = NuiGetSensorCount(&iSensorCount);

		if (FAILED(hr)) {
			return OSVR_RETURN_SUCCESS;
		}

		bool bSensorFound = false;
		for (int i = 0; i < iSensorCount; i++) {
			hr = NuiCreateSensorByIndex(i, ppNuiSensor);
			if (FAILED(hr))
			{
				continue;
			}

			hr = (*ppNuiSensor)->NuiStatus();
			if (S_OK == hr)
			{
				bSensorFound = true;
				break;
			}

			(*ppNuiSensor)->Release();
		}

		return bSensorFound;

	};

	KinectV1Device::~KinectV1Device() {

		if (m_pNuiSensor)
		{
			m_pNuiSensor->NuiShutdown();
		}
		SafeRelease(m_pNuiSensor);
	};

};
