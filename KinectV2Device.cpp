#include "KinectV2Device.h"
#include "KinectMath.h"
#include <iostream>

// Generated JSON header file
#include "je_nourish_kinectv2_json.h"

namespace KinectOsvr {

	std::map<HWND, KinectV2Device*> windowMap2;

	KinectV2Device::KinectV2Device(OSVR_PluginRegContext ctx, IKinectSensor* pKinectSensor) : m_pKinectSensor(pKinectSensor) {

		m_trackingId = m_trackedBody = -1;
		m_lastTrackedPosition.X = m_lastTrackedPosition.X = m_lastTrackedPosition.X = 0;
		m_lastTrackedTime = { 0, 0 };
		m_firstUpdate = true;
		m_trackedBodyChanged = false;

		for (int i = 0; i < BODY_COUNT; i++) {
			m_body_states[i] = CannotBeTracked;
		}

		HRESULT hr;

		// Initialize the Kinect and get coordinate mapper and the body reader
		IBodyFrameSource* pBodyFrameSource = NULL;

		hr = m_pKinectSensor->Open();


		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
		}
		else {
			std::cout << "Failed to open sensor" << std::endl;
		}
		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
		}
		if (SUCCEEDED(hr))
		{
			hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
		}
		SafeRelease(pBodyFrameSource);

		mThreadData.kinect = this;
		mThread = new std::thread(KinectV2Device::ui_thread, std::ref(mThreadData));

		/// Create the initialization options
		OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);

		osvrDeviceTrackerConfigure(opts, &m_tracker);
		osvrDeviceAnalogConfigure(opts, &m_analog, 26);
		osvrDeviceButtonConfigure(opts, &m_button, 6);

		/// Create the device token with the options
		m_dev.initAsync(ctx, "KinectV2", opts);

		/// Send JSON descriptor
		m_dev.sendJsonDescriptor(je_nourish_kinectv2_json);

		/// Register update callback
		m_dev.registerUpdateCallback(this);
	};

	OSVR_ReturnCode KinectV2Device::update() {

		if (!m_pBodyFrameReader)
		{
			return OSVR_RETURN_SUCCESS;
		}

		IBodyFrame* pBodyFrame = NULL;

		HRESULT hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);


		if (SUCCEEDED(hr))
		{
			INT64 nTime = 0;

			hr = pBodyFrame->get_RelativeTime(&nTime);

			if (m_initializeOffset == 0) {
				osvrTimeValueGetNow(&m_initializeTime);
				m_initializeOffset = nTime;
			}
			nTime = (nTime - m_initializeOffset) / 10;

			OSVR_TimeValue timeValue;
			timeValue.seconds = nTime / 1000000;
			timeValue.microseconds = nTime % 1000000;
			osvrTimeValueSum(&timeValue, &m_initializeTime);

			IBody* ppBodies[BODY_COUNT] = { 0 };

			if (SUCCEEDED(hr))
			{
				hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
			}

			if (SUCCEEDED(hr))
			{
				ProcessBody(ppBodies, &timeValue);
			}

			for (int i = 0; i < _countof(ppBodies); ++i)
			{
				SafeRelease(ppBodies[i]);
			}
		}

		SafeRelease(pBodyFrame);

		return OSVR_RETURN_SUCCESS;
	};

	KinectV2Device::BodyTrackingState* KinectV2Device::getBodyStates() {
		return m_body_states;
	}

	void KinectV2Device::setTrackedBody(int i)
	{
		m_trackedBody = i;
		m_trackedBodyChanged = true;
	}

	void KinectV2Device::ui_thread(ui_thread_data& data)
	{
		MSG msg;
		BOOL ret;
		HWND hDlg;
		HINSTANCE hInst;

		hInst = GetModuleHandle("je_nourish_kinect.dll");
		hDlg = CreateDialogParam(hInst, MAKEINTRESOURCE(IDD_DIALOG1), 0, DialogProc, 0);
		SetWindowText(hDlg, "OSVR Kinect V2 Config");
		ShowWindow(GetDlgItem(hDlg, IDC_CHECK1), SW_HIDE);
		ShowWindow(hDlg, SW_RESTORE);
		UpdateWindow(hDlg);

		windowMap2[hDlg] = data.kinect;

		KinectV2Device::BodyTrackingState previousStates[BODY_COUNT];
		KinectV2Device::BodyTrackingState* bodyStates = data.kinect->getBodyStates();
		for (int i = 0; i < BODY_COUNT; i++) {
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
			bool foundBody = false;
			int bodies = 0;

			for (int i = 0; i < BODY_COUNT; i++) {
				if (bodyStates[i] == ShouldBeTracked) {
					foundBody = true;
				}
				if (bodyStates[i] != CannotBeTracked) {
					bodies++;
				}
				if (bodyStates[i] != previousStates[i]) {
					redraw = true;
					EnableWindow(GetDlgItem(hDlg, IDC_RADIO1 + i), bodyStates[i] != CannotBeTracked);
					if (bodyStates[i] == ShouldBeTracked) {
						CheckRadioButton(hDlg, IDC_RADIO1, IDC_RADIO6, IDC_RADIO1 + i);
					}
					previousStates[i] = bodyStates[i];
				}
			}
			if (redraw) {
				if (!foundBody) {
					CheckRadioButton(hDlg, IDC_RADIO1, IDC_RADIO6, 0);
				}
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

	INT_PTR CALLBACK KinectV2Device::DialogProc(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam)
	{
		switch (uMsg)
		{
		case WM_COMMAND:
			switch (LOWORD(wParam))
			{
			case IDC_BUTTON1:
				windowMap2[hDlg]->recenter();
				break;
			case IDC_RADIO1:
			case IDC_RADIO2:
			case IDC_RADIO3:
			case IDC_RADIO4:
			case IDC_RADIO5:
			case IDC_RADIO6:
				if (BST_CHECKED == Button_GetCheck(GetDlgItem(hDlg, LOWORD(wParam)))) {
					windowMap2[hDlg]->setTrackedBody(LOWORD(wParam) - IDC_RADIO1);
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

	void setupOffset(OSVR_PoseState* offset, Joint* joint, JointOrientation* jointOrientation) {
		osvrVec3SetX(&(offset->translation), joint->Position.X);
		osvrVec3SetY(&(offset->translation), joint->Position.Y);
		osvrVec3SetZ(&(offset->translation), joint->Position.Z);

		osvrQuatSetX(&(offset->rotation), jointOrientation->Orientation.x);
		osvrQuatSetY(&(offset->rotation), jointOrientation->Orientation.y);
		osvrQuatSetZ(&(offset->rotation), jointOrientation->Orientation.z);
		osvrQuatSetW(&(offset->rotation), jointOrientation->Orientation.w);

		Eigen::Quaterniond q = osvr::util::fromQuat(offset->rotation);
		osvr::util::toQuat(q.inverse(), offset->rotation);
	}

	void KinectV2Device::recenter()
	{
		m_firstUpdate = true;
	}

	void KinectV2Device::ProcessBody(IBody** ppBodies, OSVR_TimeValue* timeValue) {

		if (m_pCoordinateMapper)
		{
			IdentifyBodies(ppBodies, timeValue);

			if (m_trackedBody >= 0)
			{
				IBody* pBody = ppBodies[m_trackedBody];

				Joint joints[JointType_Count];
				JointOrientation jointOrientations[JointType_Count];
				HandState rightHandState = HandState_Unknown;
				HandState leftHandState = HandState_Unknown;

				pBody->get_HandRightState(&rightHandState);
				pBody->get_HandLeftState(&leftHandState);

				OSVR_ButtonState buttons[6];
				buttons[0] = rightHandState == HandState_Open;
				buttons[1] = rightHandState == HandState_Closed;
				buttons[2] = rightHandState == HandState_Lasso;
				buttons[3] = leftHandState == HandState_Open;
				buttons[4] = leftHandState == HandState_Closed;
				buttons[5] = leftHandState == HandState_Lasso;

				// Send hand gestures as button presses
				osvrDeviceButtonSetValues(m_dev, m_button, buttons, 6);

				HRESULT hr = pBody->GetJoints(_countof(joints), joints);
				HRESULT hr2 = pBody->GetJointOrientations(_countof(jointOrientations), jointOrientations);

				if (SUCCEEDED(hr) && SUCCEEDED(hr2))
				{
					OSVR_PoseState poseState;
					OSVR_Vec3 translation;
					OSVR_Quaternion rotation;

					osvrVec3Zero(&translation);
					osvrQuatSetIdentity(&rotation);

					if (m_firstUpdate) {
						m_firstUpdate = false;

						setupOffset(&m_offset, &joints[JointType_Head], &jointOrientations[JointType_Neck]);

						osvrVec3SetX(&(m_kinectPose.translation), -joints[JointType_Head].Position.X);
						osvrVec3SetY(&(m_kinectPose.translation), -joints[JointType_Head].Position.Y);
						osvrVec3SetZ(&(m_kinectPose.translation), -joints[JointType_Head].Position.Z);

						Eigen::Quaterniond quaternion(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()));
						osvr::util::toQuat(quaternion, m_kinectPose.rotation);
					}

					osvrDeviceTrackerSendPoseTimestamped(m_dev, m_tracker, &m_kinectPose, 25, timeValue);

					for (int j = 0; j < _countof(joints); ++j)
					{
						osvrVec3SetX(&translation, joints[j].Position.X);
						osvrVec3SetY(&translation, joints[j].Position.Y);
						osvrVec3SetZ(&translation, joints[j].Position.Z);

						osvrQuatSetX(&rotation, jointOrientations[j].Orientation.x);
						osvrQuatSetY(&rotation, jointOrientations[j].Orientation.y);
						osvrQuatSetZ(&rotation, jointOrientations[j].Orientation.z);
						osvrQuatSetW(&rotation, jointOrientations[j].Orientation.w);

						// Rotate hand orientation to something more useful for OSVR
						if (j == JointType_HandLeft || j == JointType_HandRight) {
							boneSpaceToWorldSpace(&rotation);
						}

						poseState.translation = translation;
						poseState.rotation = rotation;
						applyOffset(&m_offset, &poseState);
						// Send pose
						osvrDeviceTrackerSendPoseTimestamped(m_dev, m_tracker, &poseState, j, timeValue);

						OSVR_AnalogState confidence = 0;
						switch (joints[j].TrackingState) {
						case TrackingState_Tracked:
							confidence = 1;
							break;
						case TrackingState_Inferred:
							confidence = 0.5;
							break;
						default:
						case TrackingState_NotTracked:
							confidence = 0;
							break;
						}
						// Tracking confidence for use in smoothing plugins
						osvrDeviceAnalogSetValueTimestamped(m_dev, m_analog, confidence, j, timeValue);
					}
				}
			}
		}
	};

	void KinectV2Device::IdentifyBodies(IBody** ppBodies, OSVR_TimeValue* timeValue) {
		HRESULT hr;
		UINT64 trackingId;

		if (m_trackedBody >= 0) { // We're tracking a body
			if (m_trackedBodyChanged) {
				hr = ppBodies[m_trackedBody]->get_TrackingId(&m_trackingId);
				if (SUCCEEDED(hr)) {
					m_trackedBodyChanged = false;
				}
				else {
					return;
				}
			}
			else {
				for (int i = 0; i < BODY_COUNT; ++i) {
					hr = ppBodies[i]->get_TrackingId(&trackingId);
					if (SUCCEEDED(hr) && trackingId == m_trackingId) {
						m_trackedBody = i;
						break;
					}
				}
			}

			BOOLEAN isTracked;
			hr = ppBodies[m_trackedBody]->get_IsTracked(&isTracked);

			if (SUCCEEDED(hr)) {
				if (isTracked) { // Discount other bodies
					for (int i = 0; i < BODY_COUNT; ++i) {
						hr = ppBodies[i]->get_TrackingId(&trackingId);
						if (SUCCEEDED(hr) && trackingId == m_trackingId) continue;

						hr = ppBodies[i]->get_IsTracked(&isTracked);
						if (isTracked) {
							m_body_states[i] = ShouldNotBeTracked;
						}
						else {
							m_body_states[i] = CannotBeTracked;
						}
					}
					return;
				}
				else {
					m_body_states[m_trackedBody] = CannotBeTracked;
					m_trackedBody = -1;
					m_trackingId = -1;
				}
			}
		}

		// Lost tracking or haven't started yet
		m_trackedBody = m_trackingId = -1;
		int candidates = 0;
		double confidence[BODY_COUNT];
		double timeConfidence = (timeValue->seconds - m_lastTrackedTime.seconds) / 15000.0; // If we lose tracking for a few seconds, just pick whoever's visible

		Joint joints[JointType_Count];
		CameraSpacePoint position;
		float distanceFromLastPosition;
		float distanceConfidence;
		for (int i = 0; i < BODY_COUNT; ++i)
		{
			BOOLEAN trackingState;

			hr = ppBodies[i]->get_TrackingId(&trackingId);
			hr = ppBodies[i]->get_IsTracked(&trackingState);
			BodyTrackingState myTrackingState = m_body_states[i];

			confidence[i] = 0.0f;

			if (trackingState) {
				switch (myTrackingState) {
				case CannotBeTracked:
				case CanBeTracked:
					m_body_states[i] = CanBeTracked;
					candidates++;

					hr = ppBodies[i]->GetJoints(_countof(joints), joints);
					position = joints[JointType_Head].Position;

					distanceFromLastPosition = sqrt(pow(position.X - m_lastTrackedPosition.X, 2) +
						pow(position.Y - m_lastTrackedPosition.Y, 2) + pow(position.Z - m_lastTrackedPosition.Z, 2));
					distanceConfidence = 1.0f - distanceFromLastPosition / 7.0f; // Approx largest possible distance in playspace
					confidence[i] = distanceConfidence + timeConfidence;

					break;
				case ShouldNotBeTracked: // Ignore bodies we've previously ruled out
					break;
				case ShouldBeTracked: // Shouldn't be possible at this point
					break;
				}
			}
			else {
				m_body_states[i] = CannotBeTracked;
			}
		}

		switch (candidates) {
		case 0: // No bodies found
			break;
		case 1: // Only 1 candidate (lets still wait until confidence is good enough)
		default: // Multiple possible bodies, choose based on last known position
			double bestConfidence = 0.0;
			for (int i = 0; i < BODY_COUNT; ++i) {
				if (m_body_states[i] == CanBeTracked) {
					if (confidence[i] > bestConfidence) {
						bestConfidence = confidence[i];
						m_trackedBody = i;
						hr = ppBodies[i]->get_TrackingId(&m_trackingId);
					}
				}
			}
			if (bestConfidence > 0.75) {
				for (int i = 0; i < BODY_COUNT; ++i) {
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


	bool KinectV2Device::Detect(IKinectSensor** ppKinectSensor) {

		typedef HRESULT(_stdcall *GetDefaultKinectSensorType)(IKinectSensor**);
		GetDefaultKinectSensorType GetDefaultKinectSensor;

		HINSTANCE hinstLib = LoadLibrary(TEXT("Kinect20.dll"));
		if (hinstLib != NULL)
		{
			GetDefaultKinectSensor = (GetDefaultKinectSensorType)GetProcAddress(hinstLib, "GetDefaultKinectSensor");

			// If the function address is valid, call the function.

			if (GetDefaultKinectSensor != NULL)
			{
				HRESULT hr = GetDefaultKinectSensor(ppKinectSensor);
				return SUCCEEDED(hr);
			}
		}

		return false;

	};

	KinectV2Device::~KinectV2Device() {
		SafeRelease(m_pBodyFrameReader);
		SafeRelease(m_pCoordinateMapper);

		if (m_pKinectSensor)
		{
			m_pKinectSensor->Close();
		}
		SafeRelease(m_pKinectSensor);
	};

};
