/*
 * File: LATC_CAL.c
 *
 * Code generated for Simulink model 'LATCtrl'.
 *
 * Model version                  : 1.265
 * Simulink Coder version         : 8.5 (R2013b) 08-Aug-2013
 * C/C++ source code generated on : Mon Oct 24 15:33:48 2022
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Infineon->TriCore
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "LATCtrl.h"

/* Exported data definition */

/* Volatile memory section */
/* Definition for custom storage class: Volatile */
volatile real_T DistErrRateLimit = 3.0;
volatile real_T K_feedfarward = 1.0;
volatile real_T LATC_HeadLoRL_C = -0.02;
volatile real_T LATC_HeadUpRL_C = 0.02;
volatile real_T LATC_Iz4EL_C = 39300.0;
volatile real_T LATC_Iz4FL_C = 80000.0;
//volatile real_T LATC_LatSpdDevIndex_C = 10.0;
volatile real_T LATC_Lf4EL_C = 1.7;
volatile real_T LATC_Lf4FL_C = 3.0;
volatile real_T LATC_Lr4EL_C = 2.3;
volatile real_T LATC_Lr4FL_C = 1.13;
volatile real_T LATC_Q1Wgt4LQR_C = 1.0;
volatile real_T LATC_Q2Wgt4LQR_C = 1.0;
volatile real_T LATC_Q3Wgt4LQR_C = 0.01;
volatile real_T LATC_Q4Wgt4LQR_C = 8.0;
volatile real_T LATC_Sign4LQR_Cv_C = 1.0;
volatile real_T LATC_Sign4LQR_FB_C = -1.0;
volatile real_T LATC_YRLoRL_C = -0.04;
volatile real_T LATC_YRUpRL_C = 0.04;
volatile real_T LATC_agSAR_C = 0.0;
volatile real_T LATC_agSteer4StdyLoThd_C = -0.5;
volatile real_T LATC_agSteer4StdyUpThd_C = 0.5;
volatile real_T LATC_agSteerOfst_C = 0.0;
volatile real_T LATC_agStrLoLmt_C = -1000.0;
volatile real_T LATC_agStrRateLoLmt_C = -50.0;
volatile real_T LATC_agStrRateUpLmt_C = 50.0;
volatile real_T LATC_agStrUpLmt_C = 1000.0;
volatile real_T LATC_caFrTyre4EL_C = 50000.0;
volatile real_T LATC_caFrTyre4FL_C = 180000.0;
volatile real_T LATC_caReTyre4EL_C = 315000.0;
volatile real_T LATC_caReTyre4FL_C = 1.25E+6;
volatile real_T LATC_facAgRelaxK_C = 1.0;
volatile real_T LATC_facCoeffSign4Mot_C = -1.0;
volatile real_T LATC_facDistRelaxK_C = 1.0;
volatile int8_T LATC_facLaneSign_C = -1;
volatile real_T LATC_facLead4Full_C = 0.45;
volatile real_T LATC_facLeadCompFB_C = 0.3;
volatile real_T LATC_facLeadLag4Empty_C = 0.6;
volatile real_T LATC_facLeadLag4Full_C = 0.95;
volatile int8_T LATC_facMPUSign_C = 1;
volatile real_T LATC_facStRelaxK_C = 1.0;
volatile real_T LATC_facWgt4KalQ1_C = 0.001;
volatile real_T LATC_facWgt4KalQ2_C = 0.003;
volatile real_T LATC_facWgt4KalQ3_C = 0.1;
volatile real_T LATC_facWgt4KalR1_C = 0.03;
volatile real_T LATC_facWgt4KalR2_C = 0.03;
volatile real_T LATC_facWgt4KalR3_C = 0.01;
volatile real_T LATC_facYRSign4Mot_C = 1.0;
volatile real_T LATC_facZeroIdfyFgt_C = 0.999;
volatile uint16_T LATC_iYawRateMaxNum = 25U;
volatile real_T LATC_lEqWheelBase_C = 4.025;
volatile real_T LATC_lLossDistLoLmt_C = 0.01;
volatile real_T LATC_lLossDistUpLmt_C = 3.5;
volatile real_T LATC_lNorDistLoLmt_C = 0.1;
volatile real_T LATC_lNorDistUpLmt_C = 3.0;
volatile real_T LATC_mFullLoadThd_C = 30000.0;
volatile real_T LATC_mTrctrMass4EL_C = 9600.0;
volatile real_T LATC_mTrctrMass4FL_C = 25000.0;
volatile real_T LATC_mVehMassMan_C = 35000.0;
volatile real_T LATC_numYRErrCnt_C = 20.0;
volatile real_T LATC_para4LQR_R_C = 200.0;
volatile real_T LATC_phiAgRelaxMax_C = 0.01;
volatile real_T LATC_phiAgRelaxMin_C = 0.005;
volatile real_T LATC_phiStRelaxMax_C = 7.0;
volatile real_T LATC_phiStRelaxMin_C = 3.0;
volatile uint8_T LATC_stADMod_C = 3U;
volatile uint8_T LATC_stCM_C = 0U;
volatile uint8_T LATC_stCR_C = 0U;
volatile uint8_T LATC_stVehMassEL_C = 1U;
volatile uint8_T LATC_swtADMod_C = 1U;
volatile uint8_T LATC_swtAgComp4Em_C = 0U;
volatile uint8_T LATC_swtAgSteerRate_C = 1U;
volatile uint8_T LATC_swtAgZeroSel_C = 1U;
volatile uint8_T LATC_swtCM_C = 0U;
volatile uint8_T LATC_swtCR_C = 0U;
volatile uint8_T LATC_swtDelyCycleMan_C = 0U;
volatile uint8_T LATC_swtHeadRL_C = 0U;
volatile uint8_T LATC_swtKalFlt_C = 0U;
volatile uint8_T LATC_swtLQRMan_C = 0U;
volatile uint8_T LATC_swtLaneCheck = 0U;
volatile uint8_T LATC_swtLeadLag_C = 1U;
volatile uint8_T LATC_swtLead_C = 1U;
volatile uint8_T LATC_swtMotPredt_C = 0U;
volatile uint8_T LATC_swtPathSel_C = 1U;
volatile uint8_T LATC_swtSAR_C = 0U;
volatile uint8_T LATC_swtTR_C = 1U;
volatile uint8_T LATC_swtVarDelyCycle_C = 0U;
volatile uint8_T LATC_swtVehMassMan_C = 0U;
volatile uint8_T LATC_swtYRComp4Em_C = 0U;
volatile uint8_T LATC_swtYRRL_C = 0U;
volatile uint8_T LATC_swtZeroIdfyReset_C = 1U;
volatile uint16_T LATC_tDelyCycleMan_C = 1U;
volatile uint16_T LATC_tFocastTime4LQR = 10U;
//volatile real_T LATC_tHeadAvrTime_C = 20.0;
volatile real_T LATC_tPrevtime4FB_C = 2.0;
volatile real_T LATC_tPrevtime4FF_C = 0.8;
volatile real_T LATC_tPrevtime4LQR_C = 0.1;
//volatile real_T LATC_tYRErrAvrTime_C = 20.0;
volatile real_T LATC_tZeroIdfyTimeThd_C = 8.0;
volatile real_T LATC_tqTR_C = 0.0;
volatile real_T LATC_vMinVehSpdThd_C = 0.5;
volatile real_T LATC_wYR4AgZero_C = 0.007;
volatile real_T LATC_yDistRelaxMax_C = 0.13;
volatile real_T LATC_yDistRelaxMin_C = 0.07;
volatile real_T LCC_lPrevDistLimit = 2.0;
volatile real_T LC_PrevDistLimitFB = 20.0;
volatile real_T LC_PrevDistLimitFF = 5.0;
volatile uint8_T LKA_swtAgSteerOfstSel_C = 1U;
volatile int32_T LQR_Calc_flag = 0;
volatile uint16_T LQR_CircleTime_C = 700U;
volatile real_T LQR_Ele1Man_C = 0.0;
volatile uint8_T LQR_Ele1Man_swt = 0U;
volatile real_T LQR_Ele2Man_C = 0.0;
volatile uint8_T LQR_Ele2Man_swt = 1U;
volatile real_T LQR_Ele3Man_C = 0.0;
volatile uint8_T LQR_Ele3Man_swt = 0U;
volatile real_T LQR_Ele4Man_C = 0.0;
volatile uint8_T LQR_Ele4Man_swt = 0U;
volatile real_T LQR_fac4YRErrEmpty_C = 1.4;
volatile real_T LatCvRateLmt = 0.001;
volatile real_T LatDistRateLmt = 0.3;
volatile real_T LatHeadRateLmt_rad = 0.03;
volatile int32_T Prev_strategy_switch = 0;

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
