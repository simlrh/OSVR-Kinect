#include <osvr/PluginKit/PluginKit.h>
#include <osvr/PluginKit/TrackerInterfaceC.h>
#include <osvr/PluginKit/AnalogInterfaceC.h>
#include <osvr/PluginKit/ButtonInterfaceC.h>
#include <osvr/Util/EigenInterop.h>

#include "KinectDevice.h"

// Generated JSON header file
#include "je_nourish_kinectv2_json.h"

#include <Kinect.h>

#include <iostream>
#define _USE_MATH_DEFINES
#include <Math.h>

namespace KinectOsvr {

	KinectDevice::KinectDevice(OSVR_PluginRegContext ctx, IKinectSensor* pKinectSensor) : m_pKinectSensor(pKinectSensor) {

		for (int i = 0; i < BODY_COUNT; i++) {
			m_channels[i] = 0;
		}

		HRESULT hr;

		// Initialize the Kinect and get coordinate mapper and the body reader
		IBodyFrameSource* pBodyFrameSource = NULL;

		hr = m_pKinectSensor->Open();

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
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

		/// Create the initialization options
		OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);

		osvrDeviceTrackerConfigure(opts, &m_tracker);
		osvrDeviceAnalogConfigure(opts, &m_analog, 25);
		osvrDeviceButtonConfigure(opts, &m_button, 6);

		/// Create the device token with the options
		m_dev.initAsync(ctx, "KinectV2", opts);

		/// Send JSON descriptor
		m_dev.sendJsonDescriptor(je_nourish_kinectv2_json);

		/// Register update callback
		m_dev.registerUpdateCallback(this);
	};

	OSVR_ReturnCode KinectDevice::update() {

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

			IBody* ppBodies[BODY_COUNT] = { 0 };

			if (SUCCEEDED(hr))
			{
				hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
			}

			if (SUCCEEDED(hr))
			{
				ProcessBody(ppBodies);
			}

			for (int i = 0; i < _countof(ppBodies); ++i)
			{
				SafeRelease(ppBodies[i]);
			}
		}

		SafeRelease(pBodyFrame);

		return OSVR_RETURN_SUCCESS;
	};

	void boneSpaceToWorldSpace(OSVR_Quaternion* q) {
		Eigen::Quaterniond quaternion = osvr::util::fromQuat(*q);

		// Rotate bone quaternion around its own x axis
		Eigen::AngleAxisd rotationAxis (M_PI/2, quaternion._transformVector(Eigen::Vector3d::UnitX()));
		Eigen::Quaterniond worldSpace = rotationAxis * quaternion;

		osvr::util::toQuat(worldSpace, (*q));
	}

	void offsetTranslation(OSVR_Vec3* translation_offset, OSVR_Vec3* translation) {
		osvrVec3SetX(translation, osvrVec3GetX(translation) - osvrVec3GetX(translation_offset));
		osvrVec3SetY(translation, osvrVec3GetY(translation) - osvrVec3GetY(translation_offset));
		osvrVec3SetZ(translation, osvrVec3GetZ(translation) - osvrVec3GetZ(translation_offset));
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

	void applyOffset(OSVR_PoseState* offset, OSVR_PoseState* poseState) {
		offsetTranslation(&(offset->translation), &(poseState->translation));
	}

	void KinectDevice::ProcessBody(IBody** ppBodies) {

		if (m_pCoordinateMapper)
		{
			for (int i = 0; i < BODY_COUNT; i++) {
				IBody* pBody = ppBodies[i];
				if (pBody)
				{

					BOOLEAN bTracked = false;
					HRESULT hr = pBody->get_IsTracked(&bTracked);

					if (!bTracked) {
						removeBody(i);
					}

					// Tracked body is the one who's been visible longest
					if (SUCCEEDED(hr) && bTracked && firstBody(addBody(i)))
					{

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

						hr = pBody->GetJoints(_countof(joints), joints);
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
							}

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
								osvrDeviceTrackerSendPose(m_dev, m_tracker, &poseState, j);

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
								osvrDeviceAnalogSetValue(m_dev, m_analog, confidence, j);
							}
						}
					}
				}
			}
		}
	};

	bool KinectDevice::firstBody(int channel) {
		for (int i = 0; i < channel; i++) {
			if (m_channels[i] != 0) {
				return false;
			}
		}
		return true;
	}

	int KinectDevice::addBody(int idx) {
		for (int i = 0; i < BODY_COUNT; i++) {
			if (m_channels[i] == idx) {
				return i;
			}
		}
		for (int i = 0; i < BODY_COUNT; i++) {
			if (!m_channels[i]) {
				m_channels[i] = idx;
				return i;
			}
		}
		return -1;
	}

	void KinectDevice::removeBody(int idx) {
		for (int i = 0; i < BODY_COUNT; i++) {
			if (m_channels[i] == idx) {
				m_channels[i] = 0;
			}
		}
	}

	bool KinectDevice::Detect(IKinectSensor** ppKinectSensor) {

		HRESULT hr = GetDefaultKinectSensor(ppKinectSensor);

		return SUCCEEDED(hr);

	};

	KinectDevice::~KinectDevice() {
		SafeRelease(m_pBodyFrameReader);
		SafeRelease(m_pCoordinateMapper);

		if (m_pKinectSensor)
		{
			m_pKinectSensor->Close();
		}
		SafeRelease(m_pKinectSensor);
	};

};
