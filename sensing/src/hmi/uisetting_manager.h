#ifndef UISETTING_MANAGER_H_
#define UISETTING_MANAGER_H_

#include "utils/macros.h"



namespace phoenix {
namespace hmi {

class UISettingManager {
public:
int GetSettingMainLaneState();

int GetSettingMainLaneTxtState();

int GetSettingLaneCurbState();

int GetSettingLaneCurbTxtState();

int GetSettingMainCameraState();

int GetSettingMainCameraTxtState();

int GetSettingMainLidarState();

int GetSettingMainLidarTxtState();

int GetSettingLeftFrontRadarState();

int GetSettingLeftFrontRadarTxtState();

int GetSettingRightFrontRadarState();

int GetSettingRightFrontRadarTxtState();

int GetSettingLeftBackRadarState();

int GetSettingLeftBackRadarTxtState();

int GetSettingRightBackRadarState();

int GetSettingRightBackRadarTxtState();

int GetSettingVCLaneState();

int GetSettingVCLaneTxtState();

int GetSettingVCFrontState();

int GetSettingVCFrontTxtState();

int GetSettingVCLeftFrontState();

int GetSettingVCLeftFrontTxtState();

int GetSettingVCRightFrontState();

int GetSettingVCRightFrontTxtState();

int GetSettingVCLeftBackState();

int GetSettingVCLeftBackTxtState();

int GetSettingVCRightBackState();

int GetSettingVCRightBackTxtState();

int GetSettingFrontRadarState();

int GetSettingFrontRadarTxtState();

int GetSettingPCFusionState();

int GetSettingPCFusionTxtState();

int GetSettingADECUFusionState();

int GetSettingADECUFusionTxtState();

int GetSettingPublishFusionTopicFromAdecu();

void SetSettingMainLaneState(int st);

void SetSettingMainLaneTxtState(int st);

void SetSettingLaneCurbState(int st);

void SetSettingLaneCurbTxtState(int st);

void SetSettingMainCameraState(int st);

void SetSettingMainCameraTxtState(int st);

void SetSettingMainLidarState(int st);

void SetSettingMainLidarTxtState(int st);

void SetSettingLeftFrontRadarState(int st);

void SetSettingLeftFrontRadarTxtState(int st);

void SetSettingRightFrontRadarState(int st);

void SetSettingRightFrontRadarTxtState(int st);

void SetSettingLeftBackRadarState(int st);

void SetSettingLeftBackRadarTxtState(int st);

void SetSettingRightBackRadarState(int st);

void SetSettingRightBackRadarTxtState(int st);

void SetSettingVCLaneState(int st);

void SetSettingVCLaneTxtState(int st);

void SetSettingVCFrontState(int st);

void SetSettingVCFrontTxtState(int st);

void SetSettingVCLeftFrontState(int st);

void SetSettingVCLeftFrontTxtState(int st);

void SetSettingVCRightFrontState(int st);

void SetSettingVCRightFrontTxtState(int st);

void SetSettingVCLeftBackState(int st);

void SetSettingVCLeftBackTxtState(int st);

void SetSettingVCRightBackState(int st);

void SetSettingVCRightBackTxtState(int st);

void SetSettingFrontRadarState(int st);

void SetSettingFrontRadarTxtState(int st);

void SetSettingPCFusionState(int st);

void SetSettingPCFusionTxtState(int st);

void SetSettingADECUFusionState(int st);

void SetSettingADECUFusionTxtState(int st);

void SetSettingPublishFusionTopicFromAdecu(int st);

private:

int setting_MainLane_state_;

int setting_MainLaneTxt_state_;

int setting_LaneCurb_state_;

int setting_LaneCurbTxt_state_;

int setting_MainCamera_state_;

int setting_MainCameraTxt_state_;

int setting_MainLidar_state_;

int setting_MainLidarTxt_state_;

int setting_LeftFrontRadar_state_;

int setting_LeftFrontRadarTxt_state_;

int setting_RightFrontRadar_state_;

int setting_RightFrontRadarTxt_state_;

int setting_LeftBackRadar_state_;

int setting_LeftBackRadarTxt_state_;

int setting_RightBackRadar_state_;

int setting_RightBackRadarTxt_state_;

int setting_VCLane_state_;

int setting_VCLaneTxt_state_;

int setting_VCFront_state_;

int setting_VCFrontTxt_state_;

int setting_VCLeftFront_state_;

int setting_VCLeftFrontTxt_state_;

int setting_VCRightFront_state_;

int setting_VCRightFrontTxt_state_;

int setting_VCLeftBack_state_;

int setting_VCLeftBackTxt_state_;

int setting_VCRightBack_state_;

int setting_VCRightBackTxt_state_;

int setting_FrontRadar_state_;

int setting_FrontRadarTxt_state_;

int setting_PCFusion_state_;

int setting_PCFusionTxt_state_;

int setting_ADECUFusion_state_;

int setting_ADECUFusionTxt_state_;

int setting_PublishFusionTopicFromAdecu_;

void LoadDefaultValue();


private:
  DECLARE_SINGLETON(UISettingManager);
};


}
}


#endif