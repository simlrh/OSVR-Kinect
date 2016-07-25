#include "stdafx.h"

void boneSpaceToWorldSpace(OSVR_Quaternion* q);
void offsetTranslation(OSVR_Vec3* translation_offset, OSVR_Vec3* translation);
void applyOffset(OSVR_PoseState* offset, OSVR_PoseState* poseState);