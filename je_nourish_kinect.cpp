// Internal Includes
#include "stdafx.h"
#include "KinectV1Device.h"
#include "KinectV2Device.h"

// Standard includes
#include <iostream>

namespace KinectOsvr {
	class HardwareDetectionV1 {
	public:
		HardwareDetectionV1() : m_found(false) {}
		OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx) {

			if (!m_found) {
				INuiSensor* pNuiSensor;

				if (KinectV1Device::Detect(&pNuiSensor)) {
					m_found = true;
					osvr::pluginkit::registerObjectForDeletion(
						ctx, new KinectV1Device(ctx, pNuiSensor));
				}
			}
			return OSVR_RETURN_SUCCESS;
		}

	private:
		bool m_found;
	};
	class HardwareDetectionV2 {
	public:
		HardwareDetectionV2() : m_found(false) {}
		OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx) {

			if (!m_found) {
				IKinectSensor* pKinectSensor;
				
				if (KinectV2Device::Detect(&pKinectSensor)) {
					m_found = true;
					osvr::pluginkit::registerObjectForDeletion(
						ctx, new KinectV2Device(ctx, pKinectSensor));
				}
			}
			return OSVR_RETURN_SUCCESS;
		}

	private:
		bool m_found;
	};
}

OSVR_PLUGIN(je_nourish_kinect) {

    osvr::pluginkit::PluginContext context(ctx);

	context.registerHardwareDetectCallback(new KinectOsvr::HardwareDetectionV1());
    context.registerHardwareDetectCallback(new KinectOsvr::HardwareDetectionV2());

    return OSVR_RETURN_SUCCESS;
}
