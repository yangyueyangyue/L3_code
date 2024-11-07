/*
 * File: LATC_CAL.h
 *
 * Code generated for Simulink model 'LATCtrl'.
 *
 * Model version                  : 1.265
 * Simulink Coder version         : 8.5 (R2013b) 08-Aug-2013
 * C/C++ source code generated on : Mon Oct 24 15:33:48 2022
 */

#ifndef RTW_HEADER_LATC_CAL_h_
#define RTW_HEADER_LATC_CAL_h_
#include "rtwtypes.h"

/* Volatile memory section */
/* Exported data declaration */
/* Declaration for custom storage class: Volatile */
extern volatile real_T DistErrRateLimit;
extern volatile real_T K_feedfarward;
extern volatile real_T LATC_HeadLoRL_C;
extern volatile real_T LATC_HeadUpRL_C;
extern volatile real_T LATC_Iz4EL_C;
extern volatile real_T LATC_Iz4FL_C;
//extern volatile real_T LATC_LatSpdDevIndex_C;
extern volatile real_T LATC_Lf4EL_C;
extern volatile real_T LATC_Lf4FL_C;
extern volatile real_T LATC_Lr4EL_C;
extern volatile real_T LATC_Lr4FL_C;
extern volatile real_T LATC_Q1Wgt4LQR_C;
extern volatile real_T LATC_Q2Wgt4LQR_C;
extern volatile real_T LATC_Q3Wgt4LQR_C;
extern volatile real_T LATC_Q4Wgt4LQR_C;
extern volatile real_T LATC_Sign4LQR_Cv_C;
extern volatile real_T LATC_Sign4LQR_FB_C;
extern volatile real_T LATC_YRLoRL_C;
extern volatile real_T LATC_YRUpRL_C;
extern volatile real_T LATC_agSAR_C;
extern volatile real_T LATC_agSteer4StdyLoThd_C;
extern volatile real_T LATC_agSteer4StdyUpThd_C;
extern volatile real_T LATC_agSteerOfst_C;
extern volatile real_T LATC_agStrLoLmt_C;
extern volatile real_T LATC_agStrRateLoLmt_C;
extern volatile real_T LATC_agStrRateUpLmt_C;
extern volatile real_T LATC_agStrUpLmt_C;
extern volatile real_T LATC_caFrTyre4EL_C;
extern volatile real_T LATC_caFrTyre4FL_C;
extern volatile real_T LATC_caReTyre4EL_C;
extern volatile real_T LATC_caReTyre4FL_C;
extern volatile real_T LATC_facAgRelaxK_C;
extern volatile real_T LATC_facCoeffSign4Mot_C;
extern volatile real_T LATC_facDistRelaxK_C;
extern volatile int8_T LATC_facLaneSign_C;
extern volatile real_T LATC_facLead4Full_C;
extern volatile real_T LATC_facLeadCompFB_C;
extern volatile real_T LATC_facLeadLag4Empty_C;
extern volatile real_T LATC_facLeadLag4Full_C;
extern volatile int8_T LATC_facMPUSign_C;
extern volatile real_T LATC_facStRelaxK_C;
extern volatile real_T LATC_facWgt4KalQ1_C;
extern volatile real_T LATC_facWgt4KalQ2_C;
extern volatile real_T LATC_facWgt4KalQ3_C;
extern volatile real_T LATC_facWgt4KalR1_C;
extern volatile real_T LATC_facWgt4KalR2_C;
extern volatile real_T LATC_facWgt4KalR3_C;
extern volatile real_T LATC_facYRSign4Mot_C;
extern volatile real_T LATC_facZeroIdfyFgt_C;
extern volatile uint16_T LATC_iYawRateMaxNum;
extern volatile real_T LATC_lEqWheelBase_C;
extern volatile real_T LATC_lLossDistLoLmt_C;
extern volatile real_T LATC_lLossDistUpLmt_C;
extern volatile real_T LATC_lNorDistLoLmt_C;
extern volatile real_T LATC_lNorDistUpLmt_C;
extern volatile real_T LATC_mFullLoadThd_C;
extern volatile real_T LATC_mTrctrMass4EL_C;
extern volatile real_T LATC_mTrctrMass4FL_C;
extern volatile real_T LATC_mVehMassMan_C;
extern volatile real_T LATC_numYRErrCnt_C;
extern volatile real_T LATC_para4LQR_R_C;
extern volatile real_T LATC_phiAgRelaxMax_C;
extern volatile real_T LATC_phiAgRelaxMin_C;
extern volatile real_T LATC_phiStRelaxMax_C;
extern volatile real_T LATC_phiStRelaxMin_C;
extern volatile uint8_T LATC_stADMod_C;
extern volatile uint8_T LATC_stCM_C;
extern volatile uint8_T LATC_stCR_C;
extern volatile uint8_T LATC_stVehMassEL_C;
extern volatile uint8_T LATC_swtADMod_C;
extern volatile uint8_T LATC_swtAgComp4Em_C;
extern volatile uint8_T LATC_swtAgSteerRate_C;
extern volatile uint8_T LATC_swtAgZeroSel_C;
extern volatile uint8_T LATC_swtCM_C;
extern volatile uint8_T LATC_swtCR_C;
extern volatile uint8_T LATC_swtDelyCycleMan_C;
extern volatile uint8_T LATC_swtHeadRL_C;
extern volatile uint8_T LATC_swtKalFlt_C;
extern volatile uint8_T LATC_swtLQRMan_C;
extern volatile uint8_T LATC_swtLaneCheck;
extern volatile uint8_T LATC_swtLeadLag_C;
extern volatile uint8_T LATC_swtLead_C;
extern volatile uint8_T LATC_swtMotPredt_C;
extern volatile uint8_T LATC_swtPathSel_C;
extern volatile uint8_T LATC_swtSAR_C;
extern volatile uint8_T LATC_swtTR_C;
extern volatile uint8_T LATC_swtVarDelyCycle_C;
extern volatile uint8_T LATC_swtVehMassMan_C;
extern volatile uint8_T LATC_swtYRComp4Em_C;
extern volatile uint8_T LATC_swtYRRL_C;
extern volatile uint8_T LATC_swtZeroIdfyReset_C;
extern volatile uint16_T LATC_tDelyCycleMan_C;
extern volatile uint16_T LATC_tFocastTime4LQR;
//extern volatile real_T LATC_tHeadAvrTime_C;
extern volatile real_T LATC_tPrevtime4FB_C;
extern volatile real_T LATC_tPrevtime4FF_C;
extern volatile real_T LATC_tPrevtime4LQR_C;
//extern volatile real_T LATC_tYRErrAvrTime_C;
extern volatile real_T LATC_tZeroIdfyTimeThd_C;
extern volatile real_T LATC_tqTR_C;
extern volatile real_T LATC_vMinVehSpdThd_C;
extern volatile real_T LATC_wYR4AgZero_C;
extern volatile real_T LATC_yDistRelaxMax_C;
extern volatile real_T LATC_yDistRelaxMin_C;
extern volatile real_T LCC_lPrevDistLimit;
extern volatile real_T LC_PrevDistLimitFB;
extern volatile real_T LC_PrevDistLimitFF;
extern volatile uint8_T LKA_swtAgSteerOfstSel_C;
extern volatile int32_T LQR_Calc_flag;
extern volatile uint16_T LQR_CircleTime_C;
extern volatile real_T LQR_Ele1Man_C;
extern volatile uint8_T LQR_Ele1Man_swt;
extern volatile real_T LQR_Ele2Man_C;
extern volatile uint8_T LQR_Ele2Man_swt;
extern volatile real_T LQR_Ele3Man_C;
extern volatile uint8_T LQR_Ele3Man_swt;
extern volatile real_T LQR_Ele4Man_C;
extern volatile uint8_T LQR_Ele4Man_swt;
extern volatile real_T LQR_fac4YRErrEmpty_C;
extern volatile real_T LatCvRateLmt;
extern volatile real_T LatDistRateLmt;
extern volatile real_T LatHeadRateLmt_rad;
extern volatile int32_T Prev_strategy_switch;

#endif                                 /* RTW_HEADER_LATC_CAL_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
