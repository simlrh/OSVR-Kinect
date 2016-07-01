// Internal Includes
#include <osvr/PluginKit/PluginKit.h>
#include <osvr/PluginKit/AnalogInterfaceC.h>
#include <KinectDevice.h>

// Library/third-party includes
// - none

// Standard includes
#include <iostream>

namespace KinectOsvr {
	class HardwareDetection {
	public:
		HardwareDetection() : m_found(false) {}
		OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx) {

			if (!m_found) {
				IKinectSensor* pKinectSensor;
				
				if (KinectDevice::Detect(&pKinectSensor)) {
					m_found = true;
					osvr::pluginkit::registerObjectForDeletion(
						ctx, new KinectDevice(ctx, pKinectSensor));
				}
			}
			return OSVR_RETURN_SUCCESS;
		}

	private:
		bool m_found;
	};
}

OSVR_PLUGIN(je_nourish_kinectv2) {

    osvr::pluginkit::PluginContext context(ctx);

    context.registerHardwareDetectCallback(new KinectOsvr::HardwareDetection());

    return OSVR_RETURN_SUCCESS;
}
