#include "uisetting_manager.h"
#include "ui_main_window.h"


namespace phoenix {
namespace hmi {

UISettingManager::UISettingManager()
{
    LoadDefaultValue();
}

/**
 * @brief 
 * 
 * Qt::CheckState::Checked:Qt::CheckState::Unchecked
 * 
 */

void UISettingManager::LoadDefaultValue()
{
    setting_MainLane_state_ = Qt::CheckState::Checked;

    setting_MainLaneTxt_state_ = Qt::CheckState::Unchecked;

    setting_LaneCurb_state_ = Qt::CheckState::Checked;

    setting_LaneCurbTxt_state_ = Qt::CheckState::Unchecked;

    setting_MainCamera_state_ = Qt::CheckState::Checked;

    setting_MainCameraTxt_state_ = Qt::CheckState::Unchecked;

    setting_MainLidar_state_ = Qt::CheckState::Checked;

    setting_MainLidarTxt_state_ = Qt::CheckState::Unchecked;

    setting_LeftFrontRadar_state_ = Qt::CheckState::Checked;

    setting_LeftFrontRadarTxt_state_ = Qt::CheckState::Unchecked;

    setting_RightFrontRadar_state_ = Qt::CheckState::Checked;

    setting_RightFrontRadarTxt_state_ = Qt::CheckState::Unchecked;

    setting_LeftBackRadar_state_ = Qt::CheckState::Checked;

    setting_LeftBackRadarTxt_state_ = Qt::CheckState::Unchecked;

    setting_RightBackRadar_state_ = Qt::CheckState::Checked;

    setting_RightBackRadarTxt_state_ = Qt::CheckState::Unchecked;

    setting_VCLane_state_ = Qt::CheckState::Unchecked;

    setting_VCLaneTxt_state_ = Qt::CheckState::Unchecked;

    setting_VCFront_state_ = Qt::CheckState::Checked;

    setting_VCFrontTxt_state_ = Qt::CheckState::Unchecked;

    setting_VCLeftFront_state_ = Qt::CheckState::Checked;

    setting_VCLeftFrontTxt_state_ = Qt::CheckState::Unchecked;

    setting_VCRightFront_state_ = Qt::CheckState::Checked;

    setting_VCRightFrontTxt_state_ = Qt::CheckState::Unchecked;

    setting_VCLeftBack_state_ = Qt::CheckState::Checked;

    setting_VCLeftBackTxt_state_ = Qt::CheckState::Unchecked;

    setting_VCRightBack_state_ = Qt::CheckState::Checked;

    setting_VCRightBackTxt_state_ = Qt::CheckState::Unchecked;

    setting_FrontRadar_state_ = Qt::CheckState::Checked;

    setting_FrontRadarTxt_state_ = Qt::CheckState::Unchecked;

    setting_PCFusion_state_ = Qt::CheckState::Checked;

    setting_PCFusionTxt_state_ = Qt::CheckState::Checked;

    setting_ADECUFusion_state_ = Qt::CheckState::Checked;

    setting_ADECUFusionTxt_state_ = Qt::CheckState::Checked;

    setting_PublishFusionTopicFromAdecu_ = 0;

}



int UISettingManager::GetSettingMainLaneState()
{
    return setting_MainLane_state_; 
}

int UISettingManager::GetSettingMainLaneTxtState()
{
    return setting_MainLaneTxt_state_; 
}

int UISettingManager::GetSettingLaneCurbState()
{
    return setting_LaneCurb_state_; 
}

int UISettingManager::GetSettingLaneCurbTxtState()
{
    return setting_LaneCurbTxt_state_; 
}

int UISettingManager::GetSettingMainCameraState()
{
    return setting_MainCamera_state_; 
}

int UISettingManager::GetSettingMainCameraTxtState()
{
    return setting_MainCameraTxt_state_; 
}

int UISettingManager::GetSettingMainLidarState()
{
    return setting_MainLidar_state_; 
}

int UISettingManager::GetSettingMainLidarTxtState()
{
    return setting_MainLidarTxt_state_; 
}

int UISettingManager::GetSettingLeftFrontRadarState()
{
    return setting_LeftFrontRadar_state_; 
}

int UISettingManager::GetSettingLeftFrontRadarTxtState()
{
    return setting_LeftFrontRadarTxt_state_; 
}

int UISettingManager::GetSettingRightFrontRadarState()
{
    return setting_RightFrontRadar_state_; 
}

int UISettingManager::GetSettingRightFrontRadarTxtState()
{
    return setting_RightFrontRadarTxt_state_; 
}

int UISettingManager::GetSettingLeftBackRadarState()
{
    return setting_LeftBackRadar_state_; 
}

int UISettingManager::GetSettingLeftBackRadarTxtState()
{
    return setting_LeftBackRadarTxt_state_; 
}

int UISettingManager::GetSettingRightBackRadarState()
{
    return setting_RightBackRadar_state_; 
}

int UISettingManager::GetSettingRightBackRadarTxtState()
{
    return setting_RightBackRadarTxt_state_; 
}

int UISettingManager::GetSettingVCLaneState()
{
    return setting_VCLane_state_; 
}

int UISettingManager::GetSettingVCLaneTxtState()
{
    return setting_VCLaneTxt_state_; 
}

int UISettingManager::GetSettingVCFrontState()
{
    return setting_VCFront_state_; 
}

int UISettingManager::GetSettingVCFrontTxtState()
{
    return setting_VCFrontTxt_state_; 
}

int UISettingManager::GetSettingVCLeftFrontState()
{
    return setting_VCLeftFront_state_; 
}

int UISettingManager::GetSettingVCLeftFrontTxtState()
{
    return setting_VCLeftFrontTxt_state_; 
}

int UISettingManager::GetSettingVCRightFrontState()
{
    return setting_VCRightFront_state_; 
}

int UISettingManager::GetSettingVCRightFrontTxtState()
{
    return setting_VCRightFrontTxt_state_; 
}

int UISettingManager::GetSettingVCLeftBackState()
{
    return setting_VCLeftBack_state_; 
}

int UISettingManager::GetSettingVCLeftBackTxtState()
{
    return setting_VCLeftBackTxt_state_; 
}

int UISettingManager::GetSettingVCRightBackState()
{
    return setting_VCRightBack_state_; 
}

int UISettingManager::GetSettingVCRightBackTxtState()
{
    return setting_VCRightBackTxt_state_; 
}

int UISettingManager::GetSettingFrontRadarState()
{
    return setting_FrontRadar_state_; 
}

int UISettingManager::GetSettingFrontRadarTxtState()
{
    return setting_FrontRadarTxt_state_; 
}

int UISettingManager::GetSettingPCFusionState()
{
    return setting_PCFusion_state_; 
}

int UISettingManager::GetSettingPCFusionTxtState()
{
    return setting_PCFusionTxt_state_; 
}

int UISettingManager::GetSettingADECUFusionState()
{
    return setting_ADECUFusion_state_; 
}

int UISettingManager::GetSettingADECUFusionTxtState()
{
    return setting_ADECUFusionTxt_state_;
}

int UISettingManager::GetSettingPublishFusionTopicFromAdecu()
{
    return setting_PublishFusionTopicFromAdecu_;
}

void UISettingManager::SetSettingMainLaneState(int st)
{
    setting_MainLane_state_ = st;
}

void UISettingManager::SetSettingMainLaneTxtState(int st)
{
    setting_MainLaneTxt_state_ = st;
}

void UISettingManager::SetSettingLaneCurbState(int st)
{
    setting_LaneCurb_state_ = st;
}

void UISettingManager::SetSettingLaneCurbTxtState(int st)
{
    setting_LaneCurbTxt_state_ = st;
}

void UISettingManager::SetSettingMainCameraState(int st)
{
    setting_MainCamera_state_ = st;
}

void UISettingManager::SetSettingMainCameraTxtState(int st)
{
    setting_MainCameraTxt_state_ = st;
}

void UISettingManager::SetSettingMainLidarState(int st)
{
    setting_MainLidar_state_ = st;
}

void UISettingManager::SetSettingMainLidarTxtState(int st)
{
    setting_MainLidarTxt_state_ = st;
}

void UISettingManager::SetSettingLeftFrontRadarState(int st)
{
    setting_LeftFrontRadar_state_ = st;
}

void UISettingManager::SetSettingLeftFrontRadarTxtState(int st)
{
    setting_LeftFrontRadarTxt_state_ = st;
}

void UISettingManager::SetSettingRightFrontRadarState(int st)
{
    setting_RightFrontRadar_state_ = st;
}

void UISettingManager::SetSettingRightFrontRadarTxtState(int st)
{
    setting_RightFrontRadarTxt_state_ = st;
}

void UISettingManager::SetSettingLeftBackRadarState(int st)
{
    setting_LeftBackRadar_state_ = st;
}

void UISettingManager::SetSettingLeftBackRadarTxtState(int st)
{
    setting_LeftBackRadarTxt_state_ = st;
}

void UISettingManager::SetSettingRightBackRadarState(int st)
{
    setting_RightBackRadar_state_ = st;
}

void UISettingManager::SetSettingRightBackRadarTxtState(int st)
{
    setting_RightBackRadarTxt_state_ = st;
}

void UISettingManager::SetSettingVCLaneState(int st)
{
    setting_VCLane_state_ = st;
}

void UISettingManager::SetSettingVCLaneTxtState(int st)
{
    setting_VCLaneTxt_state_ = st;
}

void UISettingManager::SetSettingVCFrontState(int st)
{
    setting_VCFront_state_ = st;
}

void UISettingManager::SetSettingVCFrontTxtState(int st)
{
    setting_VCFrontTxt_state_ = st;
}

void UISettingManager::SetSettingVCLeftFrontState(int st)
{
    setting_VCLeftFront_state_ = st;
}

void UISettingManager::SetSettingVCLeftFrontTxtState(int st)
{
    setting_VCLeftFrontTxt_state_ = st;
}

void UISettingManager::SetSettingVCRightFrontState(int st)
{
    setting_VCRightFront_state_ = st;
}

void UISettingManager::SetSettingVCRightFrontTxtState(int st)
{
    setting_VCRightFrontTxt_state_ = st;
}

void UISettingManager::SetSettingVCLeftBackState(int st)
{
    setting_VCLeftBack_state_ = st;
}

void UISettingManager::SetSettingVCLeftBackTxtState(int st)
{
    setting_VCLeftBackTxt_state_ = st;
}

void UISettingManager::SetSettingVCRightBackState(int st)
{
    setting_VCRightBack_state_ = st;
}

void UISettingManager::SetSettingVCRightBackTxtState(int st)
{
    setting_VCRightBackTxt_state_ = st;
}

void UISettingManager::SetSettingFrontRadarState(int st)
{
    setting_FrontRadar_state_ = st;
}

void UISettingManager::SetSettingFrontRadarTxtState(int st)
{
    setting_FrontRadarTxt_state_ = st;
}

void UISettingManager::SetSettingPCFusionState(int st)
{
    setting_PCFusion_state_ = st;
}

void UISettingManager::SetSettingPCFusionTxtState(int st)
{
    setting_PCFusionTxt_state_ = st;
}

void UISettingManager::SetSettingADECUFusionState(int st)
{
    setting_ADECUFusion_state_ = st;
}

void UISettingManager::SetSettingADECUFusionTxtState(int st)
{
    setting_ADECUFusionTxt_state_ = st;
}

void UISettingManager::SetSettingPublishFusionTopicFromAdecu(int st)
{
    setting_PublishFusionTopicFromAdecu_ = st;
}


}// end of namespace hmi
}// end of namespace phoenix