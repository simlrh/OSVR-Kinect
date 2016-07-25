#include "KinectMath.h"

void boneSpaceToWorldSpace(OSVR_Quaternion* q) {
	Eigen::Quaterniond quaternion = osvr::util::fromQuat(*q);

	// Rotate bone quaternion around its own x axis
	Eigen::AngleAxisd rotationAxis(M_PI / 2, quaternion._transformVector(Eigen::Vector3d::UnitX()));
	Eigen::Quaterniond worldSpace = rotationAxis * quaternion;

	osvr::util::toQuat(worldSpace, (*q));
}

void offsetTranslation(OSVR_Vec3* translation_offset, OSVR_Vec3* translation) {
	osvrVec3SetX(translation, osvrVec3GetX(translation) - osvrVec3GetX(translation_offset));
	osvrVec3SetY(translation, osvrVec3GetY(translation) - osvrVec3GetY(translation_offset));
	osvrVec3SetZ(translation, osvrVec3GetZ(translation) - osvrVec3GetZ(translation_offset));
}

void applyOffset(OSVR_PoseState* offset, OSVR_PoseState* poseState) {
	offsetTranslation(&(offset->translation), &(poseState->translation));
}