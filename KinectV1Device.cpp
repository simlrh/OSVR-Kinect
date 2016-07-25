#include "KinectV1Device.h"
#include "KinectMath.h"

// Generated JSON header file
#include "je_nourish_kinectv1_json.h"

#include <iostream>

namespace KinectOsvr {

	KinectV1Device::KinectV1Device(OSVR_PluginRegContext ctx, INuiSensor* pNuiSensor) : m_pNuiSensor(pNuiSensor) {

		for (int i = 0; i < NUI_SKELETON_COUNT; i++) {
			m_channels[i] = 0;
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

			m_pNuiSensor->NuiSkeletonTrackingEnable(m_hNextSkeletonEvent, NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT);
		}

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

		for (int i = 0; i < NUI_SKELETON_COUNT; ++i)
		{
			NUI_SKELETON_DATA skeleton = pSkeletons->SkeletonData[i];
			NUI_SKELETON_TRACKING_STATE trackingState = skeleton.eTrackingState;

			if (NUI_SKELETON_TRACKED == trackingState)
			{
				// Tracked body is the one who's been visible longest
				if (firstBody(addBody(skeleton.dwTrackingID)))
				{
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

						osvrDeviceTrackerSendPose(m_dev, m_tracker, &m_kinectPose, 21);

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
							osvrDeviceTrackerSendPose(m_dev, m_tracker, &poseState, j);

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
							osvrDeviceAnalogSetValue(m_dev, m_analog, confidence, j);
						}
					}
				}
			}
			else {
				removeBody(i);
			}
		}
	};

	bool KinectV1Device::firstBody(int channel) {
		for (int i = 0; i < channel; i++) {
			if (m_channels[i] != 0) {
				return false;
			}
		}
		return true;
	}

	int KinectV1Device::addBody(int idx) {
		for (int i = 0; i < NUI_SKELETON_COUNT; i++) {
			if (m_channels[i] == idx) {
				return i;
			}
		}
		for (int i = 0; i < NUI_SKELETON_COUNT; i++) {
			if (!m_channels[i]) {
				m_channels[i] = idx;
				return i;
			}
		}
		return -1;
	}

	void KinectV1Device::removeBody(int idx) {
		for (int i = 0; i < NUI_SKELETON_COUNT; i++) {
			if (m_channels[i] == idx) {
				m_channels[i] = 0;
			}
		}
	}

	bool KinectV1Device::Detect(INuiSensor** ppNuiSensor) {
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
