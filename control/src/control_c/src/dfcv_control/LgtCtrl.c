/*
 * File: LgtCtrl.c
 *
 * Code generated for Simulink model 'LgtCtrl'.
 *
 * Model version                  : 17.381
 * Simulink Coder version         : 9.6 (R2021b) 14-May-2021
 * C/C++ source code generated on : Thu May 16 10:42:27 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Infineon->TriCore
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "LgtCtrl.h"
#include "intrp2d_fu32fl_pw.h"
#include "look1_iflf_binlcapw.h"
#include "look1_iflf_binlxpw.h"
#include "plook_u32ff_binx.h"
#include "rt_modd.h"

/* Named constants for Chart: '<S33>/StMng' */
#define LgtCtrl_IN_Active              ((uint8_T)1U)
#define LgtCtrl_IN_Cruise              ((uint8_T)1U)
#define LgtCtrl_IN_Off                 ((uint8_T)1U)
#define LgtCtrl_IN_OverTake            ((uint8_T)2U)
#define LgtCtrl_IN_Parking             ((uint8_T)2U)
#define LgtCtrl_IN_Prepare             ((uint8_T)2U)
#define LgtCtrl_IN_Starting            ((uint8_T)3U)
#define LgtCtrl_IN_Working             ((uint8_T)3U)

/* Named constants for Chart: '<S39>/CoChs_stCtlSysCfm' */
#define LgtCtrl_IN_Braking             ((uint8_T)1U)
#define LgtCtrl_IN_Driving             ((uint8_T)2U)
#define LgtCtrl_IN_Slip                ((uint8_T)3U)

/* Named constants for Chart: '<S4>/DRMC_MainState' */
#define LgtCtrl_IN_Actived             ((uint8_T)1U)
#define LgtCtrl_IN_Fault               ((uint8_T)1U)
#define LgtCtrl_IN_Finished            ((uint8_T)2U)
#define LgtCtrl_IN_KeepPressure        ((uint8_T)1U)
#define LgtCtrl_IN_Normal              ((uint8_T)2U)
#define LgtCtrl_IN_Off_n               ((uint8_T)3U)
#define LgtCtrl_IN_Prepared            ((uint8_T)3U)
#define LgtCtrl_IN_PressureBuildUp     ((uint8_T)2U)
#define LgtCtrl_IN_SpdAdjmt            ((uint8_T)3U)

/* Named constants for Chart: '<S50>/S_V' */
#define LgtCtrl_IN_ClsLp               ((uint8_T)1U)
#define LgtCtrl_IN_Finish              ((uint8_T)2U)
#define LgtCtrl_IN_Initial             ((uint8_T)3U)

/* Named constants for Chart: '<S63>/GoJudg' */
#define LgtCtrl_IN_GoEnd               ((uint8_T)1U)
#define LgtCtrl_IN_GoStrt              ((uint8_T)2U)

/* Named constants for Chart: '<S66>/PCC_CC_ChangeStateCal' */
#define LgtCtrl_IN_CC2PCC              ((uint8_T)1U)
#define LgtCtrl_IN_NoAction            ((uint8_T)2U)
#define LgtCtrl_IN_PCC2CC              ((uint8_T)3U)

/* Named constants for Chart: '<S74>/filt' */
#define LgtCtrl_IN_No                  ((uint8_T)1U)
#define LgtCtrl_IN_PCC2Slip            ((uint8_T)2U)
#define LgtCtrl_IN_Slip2PCC            ((uint8_T)3U)

/* Named constants for Chart: '<S69>/filt' */
#define LgtCtrl_IN_active              ((uint8_T)2U)
#define LgtCtrl_IN_active1             ((uint8_T)3U)

/* Named constants for Chart: '<S1>/StartingJudge' */
#define LgtCtrl_IN_NoAction_i          ((uint8_T)1U)
#define LgtCtrl_IN_Starting_e          ((uint8_T)2U)

/* Named constants for Chart: '<S10>/ShiftJudg' */
#define LgtCtrl_IN_NoShift             ((uint8_T)1U)
#define LgtCtrl_IN_Shift               ((uint8_T)2U)

/* Named constants for Chart: '<S10>/TrqLim' */
#define LgtCtrl_IN_NoLim               ((uint8_T)1U)
#define LgtCtrl_IN_TrqLim              ((uint8_T)2U)

/* Named constants for Chart: '<S120>/debounce' */
#define LgtCtrl_IN_active_g            ((uint8_T)1U)
#define LgtCtrl_IN_static              ((uint8_T)2U)
#ifndef UCHAR_MAX
#include <limits.h>
#endif

/* Skipping uchar/char check: insufficient preprocessor integer range. */

/* Skipping ushort/short check: insufficient preprocessor integer range. */

/* Skipping uint/int check: insufficient preprocessor integer range. */

/* Skipping ulong/long check: insufficient preprocessor integer range. */

/* Exported block signals */
real32_T VehDa_vEgoSpd;                /* '<S155>/Gain' */
real32_T VehDa_rBrkPedl;               /* '<S148>/Gain' */
real32_T SpdPln_aTrgAcc;               /* '<S167>/Switch' */
real32_T SpdPln_vTrgSpd;               /* '<S168>/Switch' */
real32_T SpdPln_lTrgLngErr;            /* '<S163>/Gain' */
real32_T VMC_vReq;                     /* '<S142>/A1' */
real32_T VehDa_mWght;                  /* '<S170>/MinMax1' */
real32_T VDC_frRollngRes_mp;           /* '<S102>/P4' */
real32_T VDC_frAirDrag_mp;             /* '<S101>/P' */
real32_T VDC_frSlpRes_mp;              /* '<S103>/P2' */
real32_T VehDa_rTraCurGear;            /* '<S151>/Swt2' */
real32_T VDC_facDrvTrmJ_mp;            /* '<S100>/A' */
real32_T VMC_vDif_mp;                  /* '<S128>/Swt' */
real32_T VMC_aAccnGvnrP_mp;            /* '<S126>/Swt' */
real32_T VMC_aAccnGvnrD_mp;            /* '<S123>/Switch2' */
real32_T VMC_aAccnGvnrI_mp;            /* '<S131>/Swt' */
real32_T VMC_Accnpid;                  /* '<S129>/Swt' */
real32_T VMC_aReq;                     /* '<S129>/A1' */
real32_T VDC_frAccnRes_mp;             /* '<S100>/P4' */
real32_T VehDa_aEgoAcc;                /* '<S154>/Gain' */
real32_T VMC_frpid;                    /* '<S118>/Swt' */
real32_T VDC_frCmp_mp;                 /* '<S116>/Switch' */
real32_T VDC_frDrvForce;               /* '<S104>/A' */
real32_T VDC_frEngFric_mp;             /* '<S106>/Divide' */
real32_T VDC_aReqNom;                  /* '<S104>/Switch' */
real32_T DrvAxl_trqRaw_mp;             /* '<S59>/P3' */
real32_T DrvAxl_trqReq;                /* '<S58>/Gain' */
real32_T Tra_etaGear_mp;               /* '<S84>/VDC_etaTra_C' */
real32_T Tra_trqRaw_mp;                /* '<S85>/P1' */
real32_T Tra_trqReq;                   /* '<S83>/Gain' */
real32_T Eng_prcTrqRaw_mp;             /* '<S61>/Gain' */
real32_T Eng_prcTrqRaw_mp1;            /* '<S61>/A' */
real32_T Eng_prcTrqRaw_mp2;            /* '<S61>/A2' */
real32_T Eng_prcTrqLim_mp;             /* '<S62>/Swt' */
real32_T Eng_prcTrqReq;                /* '<S60>/MSwt' */
real32_T Eng_trqReq;                   /* '<S60>/D' */
real32_T VehDa_nEngSpdEst;             /* '<S161>/MinMax' */
real32_T VehDa_nEngSpd;                /* '<S160>/Gain' */
real32_T Eng_rAccrPedlReq_mp;          /* '<S67>/Difference Inputs2' */
real32_T Eng_rAccrPedlReqRaw;          /* '<S63>/MSwt' */
real32_T Eng_trqCurrMax;               /* '<S63>/Multiport Switch1' */
real32_T Eng_rAccrPedlReqRaw1;         /* '<S80>/Difference Inputs2' */
real32_T SpdPln_pcc_tar_throttle;      /* '<S171>/Gain' */
real32_T Eng_rAccrPedlReqRaw2;         /* '<S76>/Difference Inputs2' */
real32_T Eng_rAccrPedlReq;             /* '<S73>/Difference Inputs2' */
real32_T EBS_aRaw_mp;                  /* '<S87>/Switch' */
real32_T EBS_aStp_mp;                  /* '<S88>/S' */
real32_T EBS_aReq;                     /* '<S86>/Switch2' */
real32_T VehDa_pFrontLeft;             /* '<S162>/Gain' */
real32_T VehDa_rSlop;                  /* '<S165>/Switch2' */
real32_T VMC_ratioVDiff_mp;            /* '<S133>/Swt' */
int16_T VehDa_stTraCurGear;            /* '<S14>/Data Type Conversion21' */
int16_T VehDa_prcActuTrq;              /* '<S14>/Data Type Conversion14' */
int16_T VehDa_prcDrvrDmdTrq;           /* '<S14>/Data Type Conversion16' */
int16_T VehDa_prcTrqEngNomFric;        /* '<S14>/Data Type Conversion8' */
int16_T VehDa_prcTrqEstimdLoss;        /* '<S14>/Data Type Conversion10' */
uint8_T Tra_stShiftWthSpdUp;           /* '<S10>/Data Type Conversion7' */
uint8_T VehDa_stCluSwt;                /* '<S14>/Data Type Conversion6' */
uint8_T Sys_stADMd;                    /* '<S14>/Data Type Conversion19' */
uint8_T VehDa_stTrlrCnctn;             /* '<S14>/Data Type Conversion22' */
uint8_T Tra_stTrqLimWthTCU;            /* '<S10>/Data Type Conversion9' */
uint8_T VehDa_stSrcEngCtrl;            /* '<S14>/Data Type Conversion18' */
uint8_T BhvCrdn_numBhvID;              /* '<S14>/Data Type Conversion20' */
uint8_T CoChs_stStarting;              /* '<S1>/Data Type Conversion1' */
uint8_T Eng_stVehGo_mp;                /* '<S5>/Data Type Conversion8' */
uint8_T VehDa_stSrcBrk;                /* '<S14>/Data Type Conversion12' */
uint8_T Eng_StPCC_CC_Change;           /* '<S66>/PCC_CC_ChangeStateCal' */
uint8_T CoChs_stCdn;                   /* '<S38>/ConCfm' */
int8_T SpdPln_stTarType;               /* '<S14>/Data Type Conversion26' */
boolean_T SpdPln_stReleaseThrottle;    /* '<S14>/Data Type Conversion25' */
boolean_T VMC_stAccnGvnrIntglIni_mp;   /* '<S120>/L ' */
boolean_T VMC_stAccnGvnrIntglFrz_mp;   /* '<S120>/L9' */
boolean_T CoChs_stDrvReq_mp;           /* '<S39>/R12' */
boolean_T CoChs_stBrkReq_mp;           /* '<S39>/R13' */
boolean_T CoChs_stSlipReq_mp;          /* '<S39>/R11' */

/* Exported block parameters */
real32_T ACCS_dtAccrSlpDwn_C = -2.0F;  /* Variable: ACCS_dtAccrSlpDwn_C
                                        * Referenced by: '<S22>/ACCS_dtAccrSlpDwn_C'
                                        */
real32_T ACCS_dtAccrSlpUp_C = 0.6F;    /* Variable: ACCS_dtAccrSlpUp_C
                                        * Referenced by: '<S22>/ACCS_dtAccrSlpUp_C'
                                        */
real32_T ACCS_dtDecSlpDwn_C = -2.0F;   /* Variable: ACCS_dtDecSlpDwn_C
                                        * Referenced by: '<S22>/ACCS_dtDecSlpDwn_C'
                                        */
real32_T ACCS_dtDecSlpUp_C = 1.0F;     /* Variable: ACCS_dtDecSlpUp_C
                                        * Referenced by: '<S22>/ACCS_dtDecSlpUp_C'
                                        */
real32_T ACCS_dtFrontTrgAccrSlpDwn_C = -0.5F;/* Variable: ACCS_dtFrontTrgAccrSlpDwn_C
                                              * Referenced by: '<S20>/ACCS_dtFrontTrgAccrSlpDwn_C'
                                              */
real32_T ACCS_dtFrontTrgAccrSlpUp_C = 1.0F;/* Variable: ACCS_dtFrontTrgAccrSlpUp_C
                                            * Referenced by: '<S20>/ACCS_dtFrontTrgAccrSlpUp_C'
                                            */
real32_T ACCS_dtSpSlpDwnWthTar_C = -3.0F;/* Variable: ACCS_dtSpSlpDwnWthTar_C
                                          * Referenced by: '<S21>/ACCS_dtSpSlpDwnWthTar_C'
                                          */
real32_T ACCS_dtSpSlpDwn_C = -1.0F;    /* Variable: ACCS_dtSpSlpDwn_C
                                        * Referenced by: '<S22>/ACCS_dtSpSlpDwn_C'
                                        */
real32_T ACCS_dtSpSlpUp_C = 1.0F;      /* Variable: ACCS_dtSpSlpUp_C
                                        * Referenced by: '<S22>/ACCS_dtSpSlpUp_C'
                                        */
real32_T ACCS_facDstCtlPnt_C = 1.0F;   /* Variable: ACCS_facDstCtlPnt_C
                                        * Referenced by: '<S21>/Gain1'
                                        */
real32_T ACCS_facDstToSpdCurr_C = 0.1F;/* Variable: ACCS_facDstToSpdCurr_C
                                        * Referenced by: '<S21>/ACCS_facDstToSpdCurr_C'
                                        */
real32_T ACCS_facDstToSpdNeg_C = 0.2F; /* Variable: ACCS_facDstToSpdNeg_C
                                        * Referenced by: '<S21>/ACCS_facDstToSpdNeg_C'
                                        */
real32_T ACCS_facDstToSpdPos_C = 0.45F;/* Variable: ACCS_facDstToSpdPos_C
                                        * Referenced by: '<S21>/ACCS_facDstToSpdPos_C'
                                        */
real32_T ACCS_facDstToSpd_C = 0.45F;   /* Variable: ACCS_facDstToSpd_C
                                        * Referenced by: '<S21>/ACCS_facDstToSpd_C'
                                        */
real32_T ACCS_facRelSpd2Spd_C = 0.0F;  /* Variable: ACCS_facRelSpd2Spd_C
                                        * Referenced by: '<S21>/ACCS_facRelSpd2Spd_C'
                                        */
real32_T ACCS_facSpSlpDwnCtrlPnt_C = 0.4F;/* Variable: ACCS_facSpSlpDwnCtrlPnt_C
                                           * Referenced by: '<S22>/ACCS_facSpSlpDwnCtrlPnt_C'
                                           */
real32_T ACCS_facSpSlpUpCtrlPnt_C = 0.2F;/* Variable: ACCS_facSpSlpUpCtrlPnt_C
                                          * Referenced by: '<S22>/ACCS_facSpSlpUpCtrlPnt_C'
                                          */
real32_T ACCS_lSetMin_C = 9.0F;        /* Variable: ACCS_lSetMin_C
                                        * Referenced by: '<S21>/ACCS_lSetMin_C'
                                        */
real32_T ACCS_rBrkPedlThd_C = 5.0F;    /* Variable: ACCS_rBrkPedlThd_C
                                        * Referenced by: '<S33>/ACCS_rBrkPedlThd_C'
                                        */
real32_T ACCS_tiDstSpCalReqdVal_C = 3.5F;/* Variable: ACCS_tiDstSpCalReqdVal_C
                                          * Referenced by: '<S35>/ACCS_tiDstSpCalReqdVal_C'
                                          */
real32_T ACCS_tiFollowHysNeg_C = -0.2F;/* Variable: ACCS_tiFollowHysNeg_C
                                        * Referenced by: '<S21>/ACCS_tiFollowHysNeg_C'
                                        */
real32_T ACCS_tiFollowHysPos_C = 0.2F; /* Variable: ACCS_tiFollowHysPos_C
                                        * Referenced by: '<S21>/ACCS_tiFollowHysPos_C'
                                        */
real32_T ACCS_tiTimeGapModFive_C = 1.5F;/* Variable: ACCS_tiTimeGapModFive_C
                                         * Referenced by: '<S35>/ACCS_tiTimeGapModFive_C'
                                         */
real32_T ACCS_tiTimeGapModOne_C = 3.5F;/* Variable: ACCS_tiTimeGapModOne_C
                                        * Referenced by: '<S35>/ACCS_tiTimeGapModOne_C'
                                        */
real32_T ACCS_vCruiseToStarting_C = 3.0F;/* Variable: ACCS_vCruiseToStarting_C
                                          * Referenced by: '<S33>/ACCS_vCruiseToStarting_C'
                                          */
real32_T ACCS_vDstCtlPnt_C = 100.0F;   /* Variable: ACCS_vDstCtlPnt_C
                                        * Referenced by: '<S21>/ACCS_vDstCtlPnt_C'
                                        */
real32_T ACCS_vGoMinWthTar_C = 10.0F;  /* Variable: ACCS_vGoMinWthTar_C
                                        * Referenced by: '<S33>/ACCS_vGoMinWthTar_C'
                                        */
real32_T ACCS_vGoMin_C = 0.9F;         /* Variable: ACCS_vGoMin_C
                                        * Referenced by: '<S33>/ACCS_vGoMin_C'
                                        */
real32_T ACCS_vMax_C = 80.0F;          /* Variable: ACCS_vMax_C
                                        * Referenced by:
                                        *   '<S33>/ACCS_vMax_C'
                                        *   '<S35>/ACCS_vMax_C'
                                        *   '<S36>/ACCS_vMax_C'
                                        */
real32_T ACCS_vMin_C = 0.0F;           /* Variable: ACCS_vMin_C
                                        * Referenced by:
                                        *   '<S33>/ACCS_vMin_C'
                                        *   '<S35>/ACCS_vMin_C'
                                        *   '<S36>/ACCS_vMin_C'
                                        */
real32_T ACCS_vStpThd_C = 1.0F;        /* Variable: ACCS_vStpThd_C
                                        * Referenced by: '<S33>/ACCS_vStpThd_C'
                                        */
real32_T ACCS_vTipDwn_C = 10.0F;       /* Variable: ACCS_vTipDwn_C
                                        * Referenced by: '<S36>/ACCS_vTipDwn_C'
                                        */
real32_T ACCS_vTipUp_C = 10.0F;        /* Variable: ACCS_vTipUp_C
                                        * Referenced by: '<S36>/ACCS_vTipUp_C'
                                        */
real32_T ACCS_vVSpCalReqdVal_C = 0.0F; /* Variable: ACCS_vVSpCalReqdVal_C
                                        * Referenced by: '<S35>/ACCS_vVSpCalReqdVal_C'
                                        */
real32_T AEBS_aDesFf_C = -5.0F;        /* Variable: AEBS_aDesFf_C
                                        * Referenced by: '<S16>/AEBS_aDesFf_C'
                                        */
real32_T AEBS_lSftMin_C = 5.0F;        /* Variable: AEBS_lSftMin_C
                                        * Referenced by: '<S16>/AEBS_lSftMin_C'
                                        */
real32_T AEBS_vActvdLmtLo_C = 20.0F;   /* Variable: AEBS_vActvdLmtLo_C
                                        * Referenced by: '<S16>/AEBS_vActvdLmtLo_C'
                                        */
real32_T AdpvCC_tiSpdCtrlUpdCyc_C = 0.1F;/* Variable: AdpvCC_tiSpdCtrlUpdCyc_C
                                          * Referenced by: '<S22>/AdpvCC_tiSpdCtrlUpdCyc_C'
                                          */
real32_T CC_ThroltRate_lo = -150.0F;   /* Variable: CC_ThroltRate_lo
                                        * Referenced by: '<S69>/CC_ThroltRate_lo'
                                        */
real32_T CC_ThroltRate_lo1 = -150.0F;  /* Variable: CC_ThroltRate_lo1
                                        * Referenced by: '<S69>/CC_ThroltRate_lo1'
                                        */
real32_T CC_ThroltRate_tableX[5] = { 0.0F, 10.0F, 20.0F, 30.0F, 100.0F } ;/* Variable: CC_ThroltRate_tableX
                                                                      * Referenced by:
                                                                      *   '<S66>/CC_ThroltRateUp'
                                                                      *   '<S66>/CC_ThroltRateUp1'
                                                                      *   '<S69>/CC_ThroltRateUp'
                                                                      */

real32_T CC_ThroltRate_tableY[5] = { 10.0F, 15.0F, 22.0F, 30.0F, 40.0F } ;/* Variable: CC_ThroltRate_tableY
                                                                      * Referenced by:
                                                                      *   '<S66>/CC_ThroltRateUp'
                                                                      *   '<S66>/CC_ThroltRateUp1'
                                                                      *   '<S69>/CC_ThroltRateUp'
                                                                      */

real32_T DRMC_aReqFf_C = -1.0F;        /* Variable: DRMC_aReqFf_C
                                        * Referenced by: '<S4>/DRMC_aReqFf_C'
                                        */
real32_T DRMC_facSpdCtrlKpCurr_C = 3.0F;/* Variable: DRMC_facSpdCtrlKpCurr_C
                                         * Referenced by: '<S50>/DRMC_facSpdCtrlKpCurr_C'
                                         */
real32_T DRMC_facSpdCtrlKpNegCurr_C = 3.0F;/* Variable: DRMC_facSpdCtrlKpNegCurr_C
                                            * Referenced by: '<S50>/DRMC_facSpdCtrlKpNegCurr_C'
                                            */
real32_T DRMC_facSpdCtrlKpPosCurr_C = 3.0F;/* Variable: DRMC_facSpdCtrlKpPosCurr_C
                                            * Referenced by: '<S50>/DRMC_facSpdCtrlKpPosCurr_C'
                                            */
real32_T DRMC_lReqMin_C = 3.0F;        /* Variable: DRMC_lReqMin_C
                                        * Referenced by: '<S4>/DRMC_lReqMin_C'
                                        */
real32_T DRMC_lSpdCtrlPWinNeg_C = -0.4F;/* Variable: DRMC_lSpdCtrlPWinNeg_C
                                         * Referenced by: '<S50>/DRMC_lSpdCtrlPWinNeg_C'
                                         */
real32_T DRMC_lSpdCtrlPWinPos_C = 0.4F;/* Variable: DRMC_lSpdCtrlPWinPos_C
                                        * Referenced by: '<S50>/DRMC_lSpdCtrlPWinPos_C'
                                        */
real32_T DRMC_lTrg_C = 0.1F;           /* Variable: DRMC_lTrg_C
                                        * Referenced by: '<S4>/DRMC_lTrg_C'
                                        */
real32_T DRMC_rAcclPedlPosnReq_C = 10.0F;/* Variable: DRMC_rAcclPedlPosnReq_C
                                          * Referenced by: '<S46>/DRMC_rAcclPedlPosnReq_C'
                                          */
real32_T DRMC_tiBrkResp_C = 0.0F;      /* Variable: DRMC_tiBrkResp_C
                                        * Referenced by: '<S4>/DRMC_tiBrkResp_C'
                                        */
real32_T DRMC_tiCyc_C = 0.01F;         /* Variable: DRMC_tiCyc_C
                                        * Referenced by: '<S4>/DRMC_tiCyc_C'
                                        */
real32_T DRMC_tiPBU_C = 0.45F;         /* Variable: DRMC_tiPBU_C
                                        * Referenced by: '<S4>/DRMC_tiPBU_C'
                                        */
real32_T DRMC_tiPrepMax_C = 2.0F;      /* Variable: DRMC_tiPrepMax_C
                                        * Referenced by: '<S4>/DRMC_tiPrepMax_C'
                                        */
real32_T EBS_PrkDecnSlpUp_C = 3.0F;    /* Variable: EBS_PrkDecnSlpUp_C
                                        * Referenced by: '<S86>/EBS_PrkDecnSlpUp_C'
                                        */
real32_T EBS_aCmpFixd_C = 0.4F;        /* Variable: EBS_aCmpFixd_C
                                        * Referenced by: '<S87>/EBS_aCmpFixd_C'
                                        */
real32_T EBS_aDfl_C = 0.0F;            /* Variable: EBS_aDfl_C
                                        * Referenced by: '<S86>/EBS_aDfl_C'
                                        */
real32_T EBS_aEmgyBrk_C = -2.0F;       /* Variable: EBS_aEmgyBrk_C
                                        * Referenced by: '<S86>/EBS_aEmgyBrk_C'
                                        */
real32_T EBS_aStpPlnBOne_C = -0.5F;    /* Variable: EBS_aStpPlnBOne_C
                                        * Referenced by: '<S88>/EBS_aStpPlnBOne_C'
                                        */
real32_T EBS_aStpPlnBTwo_C = -0.2F;    /* Variable: EBS_aStpPlnBTwo_C
                                        * Referenced by: '<S88>/EBS_aStpPlnBTwo_C'
                                        */
real32_T EBS_aStpTmp_C = -5.0F;        /* Variable: EBS_aStpTmp_C
                                        * Referenced by: '<S86>/EBS_aStpTmp_C'
                                        */
real32_T EBS_aStp_C = -1.0F;           /* Variable: EBS_aStp_C
                                        * Referenced by: '<S88>/EBS_aStp_C'
                                        */
real32_T EBS_aTstReq_C = 0.0F;         /* Variable: EBS_aTstReq_C
                                        * Referenced by: '<S86>/EBS_aTstReq_C'
                                        */
real32_T EBS_vStpCft_C = 2.0F;         /* Variable: EBS_vStpCft_C
                                        * Referenced by: '<S88>/EBS_vStpCft_C'
                                        */
real32_T Eng_dtAccrPedlDwn_C = -100.0F;/* Variable: Eng_dtAccrPedlDwn_C
                                        * Referenced by: '<S63>/Eng_dtAccrPedlDwn_C'
                                        */
real32_T Eng_dtAccrPedlUp_C = 30.0F;   /* Variable: Eng_dtAccrPedlUp_C
                                        * Referenced by: '<S63>/Eng_dtAccrPedlUp_C'
                                        */
real32_T Eng_facTrqLimCmpFild_C = 0.9F;/* Variable: Eng_facTrqLimCmpFild_C
                                        * Referenced by: '<S64>/Eng_facTrqLimCmpFild_C'
                                        */
real32_T Eng_facTrqLimCmp_C = 1.0F;    /* Variable: Eng_facTrqLimCmp_C
                                        * Referenced by: '<S61>/Eng_facTrqLimCmp_C'
                                        */
real32_T Eng_prcGoEnd_C = 25.0F;       /* Variable: Eng_prcGoEnd_C
                                        * Referenced by: '<S60>/Eng_prcGoEnd_C'
                                        */
real32_T Eng_prcGo_C = 30.0F;          /* Variable: Eng_prcGo_C
                                        * Referenced by: '<S60>/Eng_prcGo_C'
                                        */
real32_T Eng_prcTstReq_C = 0.0F;       /* Variable: Eng_prcTstReq_C
                                        * Referenced by: '<S60>/Eng_prcTstReq_C'
                                        */
real32_T Eng_rAccrPedlTstReq_C = 0.0F; /* Variable: Eng_rAccrPedlTstReq_C
                                        * Referenced by: '<S63>/Eng_rAccrPedlTstReq_C'
                                        */
real32_T Eng_rGoPedlCmp_C = 5.0F;      /* Variable: Eng_rGoPedlCmp_C
                                        * Referenced by: '<S63>/Eng_rGoPedlCmp_C'
                                        */
real32_T Eng_vGoThd_C = 0.5F;          /* Variable: Eng_vGoThd_C
                                        * Referenced by: '<S63>/Eng_vGoThd_C'
                                        */
real32_T Eng_vVehStaticThd_C = 0.01F;  /* Variable: Eng_vVehStaticThd_C
                                        * Referenced by: '<S63>/Eng_vVehStaticThd_C'
                                        */
real32_T EnvDetn_lCllsnTarLgt_C = 150.0F;/* Variable: EnvDetn_lCllsnTarLgt_C
                                          * Referenced by: '<S20>/EnvDetn_lCllsnTarLgt_C'
                                          */
real32_T EnvDetn_tiUpdate_C = 0.01F;   /* Variable: EnvDetn_tiUpdate_C
                                        * Referenced by: '<S20>/EnvDetn_tiUpdate_C'
                                        */
real32_T EnvDetn_vCllsnTar_C = 0.0F;   /* Variable: EnvDetn_vCllsnTar_C
                                        * Referenced by: '<S20>/EnvDetn_vCllsnTar_C'
                                        */
real32_T I1_C = 0.005F;                /* Variable: I1_C
                                        * Referenced by: '<S121>/I1_C'
                                        */
real32_T I2_C = 0.02F;                 /* Variable: I2_C
                                        * Referenced by: '<S121>/I2_C'
                                        */
real32_T I3_C = 0.02F;                 /* Variable: I3_C
                                        * Referenced by: '<S121>/I3_C'
                                        */
real32_T I4_C = 1.5F;                  /* Variable: I4_C
                                        * Referenced by: '<S121>/I4_C'
                                        */
real32_T I5_C = -1.5F;                 /* Variable: I5_C
                                        * Referenced by: '<S121>/I5_C'
                                        */
real32_T P1_C = 0.08F;                 /* Variable: P1_C
                                        * Referenced by: '<S121>/P1_C'
                                        */
real32_T P2_C = 0.15F;                 /* Variable: P2_C
                                        * Referenced by: '<S121>/P2_C'
                                        */
real32_T P3_C = 0.3F;                  /* Variable: P3_C
                                        * Referenced by: '<S121>/P3_C'
                                        */
real32_T P4_C = 1.2F;                  /* Variable: P4_C
                                        * Referenced by: '<S121>/P4_C'
                                        */
real32_T P5_C = -1.2F;                 /* Variable: P5_C
                                        * Referenced by: '<S121>/P5_C'
                                        */
real32_T PCC2CC_ThroltRate_Default = 200.0F;/* Variable: PCC2CC_ThroltRate_Default
                                             * Referenced by: '<S66>/PCC2CC_ThroltRate_Default'
                                             */
real32_T PCC_ThroltRate_lo = -150.0F;  /* Variable: PCC_ThroltRate_lo
                                        * Referenced by: '<S74>/PCC_ThroltRate_lo'
                                        */
real32_T PCC_ThroltRate_lo1 = -150.0F; /* Variable: PCC_ThroltRate_lo1
                                        * Referenced by: '<S74>/PCC_ThroltRate_lo1'
                                        */
real32_T PCC_ThroltRate_up = 40.0F;    /* Variable: PCC_ThroltRate_up
                                        * Referenced by: '<S74>/PCC_ThroltRate_up'
                                        */
real32_T Tra_aShtWthSpdUpThd_C = -0.1F;/* Variable: Tra_aShtWthSpdUpThd_C
                                        * Referenced by: '<S10>/Tra_aShtWthSpdUpThd_C'
                                        */
real32_T Tra_facCluTrqEgd_C = 0.5F;    /* Variable: Tra_facCluTrqEgd_C
                                        * Referenced by: '<S10>/Tra_facCluTrqEgd_C'
                                        */
real32_T Tra_facCluTrqRls_C = 0.5F;    /* Variable: Tra_facCluTrqRls_C
                                        * Referenced by: '<S10>/Tra_facCluTrqRls_C'
                                        */
real32_T VMC_aAccnDCmpLo_C = -0.15F;   /* Variable: VMC_aAccnDCmpLo_C
                                        * Referenced by: '<S119>/VMC_aAccnDCmpLo_C'
                                        */
real32_T VMC_aAccnDCmpUp_C = 0.15F;    /* Variable: VMC_aAccnDCmpUp_C
                                        * Referenced by: '<S119>/VMC_aAccnDCmpUp_C'
                                        */
real32_T VMC_aAccnGvnrDftlIniVal_C = 0.0F;/* Variable: VMC_aAccnGvnrDftlIniVal_C
                                           * Referenced by: '<S119>/VMC_aAccnGvnrDftlIniVal_C'
                                           */
real32_T VMC_aAccnGvnrIntglCtrlValLo_C = -0.35F;
                                      /* Variable: VMC_aAccnGvnrIntglCtrlValLo_C
                                       * Referenced by: '<S121>/VMC_aAccnGvnrIntglCtrlValLo_C'
                                       */
real32_T VMC_aAccnGvnrIntglCtrlValUp_C = 0.35F;
                                      /* Variable: VMC_aAccnGvnrIntglCtrlValUp_C
                                       * Referenced by: '<S121>/VMC_aAccnGvnrIntglCtrlValUp_C'
                                       */
real32_T VMC_aAccnGvnrIntglIniVal_C = 0.0F;/* Variable: VMC_aAccnGvnrIntglIniVal_C
                                            * Referenced by: '<S119>/VMC_aAccnGvnrIntglIniVal_C'
                                            */
real32_T VMC_aAccnGvnrOutpLo_C = -2.0F;/* Variable: VMC_aAccnGvnrOutpLo_C
                                        * Referenced by: '<S129>/VMC_aAccnGvnrOutpLo_C'
                                        */
real32_T VMC_aAccnGvnrOutpUp_C = 2.0F; /* Variable: VMC_aAccnGvnrOutpUp_C
                                        * Referenced by: '<S129>/VMC_aAccnGvnrOutpUp_C'
                                        */
real32_T VMC_dtEngFricAccr_C = 0.05F;  /* Variable: VMC_dtEngFricAccr_C
                                        * Referenced by: '<S104>/VMC_dtEngFricAccr_C'
                                        */
real32_T VMC_facAccnGvnrDftlRatio_C = 0.0F;/* Variable: VMC_facAccnGvnrDftlRatio_C
                                            * Referenced by: '<S119>/VMC_facAccnGvnrDftlRatio_C'
                                            */
real32_T VMC_facAccnGvnrKdCurr_C = 0.0F;/* Variable: VMC_facAccnGvnrKdCurr_C
                                         * Referenced by: '<S121>/VMC_facAccnGvnrKdCurr_C'
                                         */
real32_T VMC_facAccnGvnrKdNegCurr_C = 0.0F;/* Variable: VMC_facAccnGvnrKdNegCurr_C
                                            * Referenced by: '<S121>/VMC_facAccnGvnrKdNegCurr_C'
                                            */
real32_T VMC_facAccnGvnrKdPosCurr_C = 0.0F;/* Variable: VMC_facAccnGvnrKdPosCurr_C
                                            * Referenced by: '<S121>/VMC_facAccnGvnrKdPosCurr_C'
                                            */
real32_T VMC_facAccnGvnrKiCurr_C = 0.005F;/* Variable: VMC_facAccnGvnrKiCurr_C
                                           * Referenced by: '<S121>/VMC_facAccnGvnrKiCurr_C'
                                           */
real32_T VMC_facAccnGvnrKiNegCurr_C = 0.02F;/* Variable: VMC_facAccnGvnrKiNegCurr_C
                                             * Referenced by: '<S121>/VMC_facAccnGvnrKiNegCurr_C'
                                             */
real32_T VMC_facAccnGvnrKiPosCurr_C = 0.02F;/* Variable: VMC_facAccnGvnrKiPosCurr_C
                                             * Referenced by: '<S121>/VMC_facAccnGvnrKiPosCurr_C'
                                             */
real32_T VMC_facAccnGvnrKpCurr_C = 0.08F;/* Variable: VMC_facAccnGvnrKpCurr_C
                                          * Referenced by: '<S121>/VMC_facAccnGvnrKpCurr_C'
                                          */
real32_T VMC_facAccnGvnrKpNegCurrLoSpd_C = 1.0F;
                                    /* Variable: VMC_facAccnGvnrKpNegCurrLoSpd_C
                                     * Referenced by: '<S121>/VMC_facAccnGvnrKpNegCurrLoSpd_C'
                                     */
real32_T VMC_facAccnGvnrKpNegCurr_C = 0.3F;/* Variable: VMC_facAccnGvnrKpNegCurr_C
                                            * Referenced by: '<S121>/VMC_facAccnGvnrKpNegCurr_C'
                                            */
real32_T VMC_facAccnGvnrKpPosCurr_C = 0.15F;/* Variable: VMC_facAccnGvnrKpPosCurr_C
                                             * Referenced by: '<S121>/VMC_facAccnGvnrKpPosCurr_C'
                                             */
real32_T VMC_facAccnGvrnKpWthGo_C = 0.4F;/* Variable: VMC_facAccnGvrnKpWthGo_C
                                          * Referenced by: '<S121>/VMC_facAccnGvrnKpWthGo_C'
                                          */
real32_T VMC_facLocnDifFild_C = 0.0F;  /* Variable: VMC_facLocnDifFild_C
                                        * Referenced by: '<S141>/VMC_facLocnDifFild_C'
                                        */
real32_T VMC_facSpdDTfild_C = 0.93F;   /* Variable: VMC_facSpdDTfild_C
                                        * Referenced by: '<S133>/VMC_facSpdDTfild_C'
                                        */
real32_T VMC_facSpdDifFild_C = 0.9F;   /* Variable: VMC_facSpdDifFild_C
                                        * Referenced by: '<S128>/VMC_facSpdDifFild_C'
                                        */
real32_T VMC_facTrqLim_C = 2.0F;       /* Variable: VMC_facTrqLim_C
                                        * Referenced by: '<S120>/VMC_facTrqLim_C'
                                        */
real32_T VMC_facVelGvnrDftlRatio_C = 0.0F;/* Variable: VMC_facVelGvnrDftlRatio_C
                                           * Referenced by: '<S135>/VMC_facVelGvnrDftlRatio_C'
                                           */
real32_T VMC_facVelGvnrKdCurr_C = 0.0F;/* Variable: VMC_facVelGvnrKdCurr_C
                                        * Referenced by: '<S137>/VMC_facVelGvnrKdCurr_C'
                                        */
real32_T VMC_facVelGvnrKdNegCurr_C = 0.0F;/* Variable: VMC_facVelGvnrKdNegCurr_C
                                           * Referenced by: '<S137>/VMC_facVelGvnrKdNegCurr_C'
                                           */
real32_T VMC_facVelGvnrKdPosCurr_C = 0.0F;/* Variable: VMC_facVelGvnrKdPosCurr_C
                                           * Referenced by: '<S137>/VMC_facVelGvnrKdPosCurr_C'
                                           */
real32_T VMC_facVelGvnrKiCurr_C = 0.0F;/* Variable: VMC_facVelGvnrKiCurr_C
                                        * Referenced by: '<S137>/VMC_facVelGvnrKiCurr_C'
                                        */
real32_T VMC_facVelGvnrKiNegCurr_C = 0.0F;/* Variable: VMC_facVelGvnrKiNegCurr_C
                                           * Referenced by: '<S137>/VMC_facVelGvnrKiNegCurr_C'
                                           */
real32_T VMC_facVelGvnrKiPosCurr_C = 0.0F;/* Variable: VMC_facVelGvnrKiPosCurr_C
                                           * Referenced by: '<S137>/VMC_facVelGvnrKiPosCurr_C'
                                           */
real32_T VMC_facVelGvnrKpCurr_C = 0.0F;/* Variable: VMC_facVelGvnrKpCurr_C
                                        * Referenced by: '<S137>/VMC_facVelGvnrKpCurr_C'
                                        */
real32_T VMC_facVelGvnrKpNegCurr_C = 0.0F;/* Variable: VMC_facVelGvnrKpNegCurr_C
                                           * Referenced by: '<S137>/VMC_facVelGvnrKpNegCurr_C'
                                           */
real32_T VMC_facVelGvnrKpPosCurr_C = 0.0F;/* Variable: VMC_facVelGvnrKpPosCurr_C
                                           * Referenced by: '<S137>/VMC_facVelGvnrKpPosCurr_C'
                                           */
real32_T VMC_lVelGvnrDWinNeg_C = 0.0F; /* Variable: VMC_lVelGvnrDWinNeg_C
                                        * Referenced by: '<S137>/VMC_lVelGvnrDWinNeg_C'
                                        */
real32_T VMC_lVelGvnrDWinPos_C = 0.0F; /* Variable: VMC_lVelGvnrDWinPos_C
                                        * Referenced by: '<S137>/VMC_lVelGvnrDWinPos_C'
                                        */
real32_T VMC_lVelGvnrIWinNeg_C = 0.0F; /* Variable: VMC_lVelGvnrIWinNeg_C
                                        * Referenced by: '<S137>/VMC_lVelGvnrIWinNeg_C'
                                        */
real32_T VMC_lVelGvnrIWinPos_C = 0.0F; /* Variable: VMC_lVelGvnrIWinPos_C
                                        * Referenced by: '<S137>/VMC_lVelGvnrIWinPos_C'
                                        */
real32_T VMC_lVelGvnrPWinNeg_C = 0.0F; /* Variable: VMC_lVelGvnrPWinNeg_C
                                        * Referenced by: '<S137>/VMC_lVelGvnrPWinNeg_C'
                                        */
real32_T VMC_lVelGvnrPWinPos_C = 0.0F; /* Variable: VMC_lVelGvnrPWinPos_C
                                        * Referenced by: '<S137>/VMC_lVelGvnrPWinPos_C'
                                        */
real32_T VMC_tiAccnGvnrIntglUpdCyc_C = 0.01F;/* Variable: VMC_tiAccnGvnrIntglUpdCyc_C
                                              * Referenced by: '<S119>/VMC_tiAccnGvnrIntglUpdCyc_C'
                                              */
real32_T VMC_tiVelGvnrIntglUpdCyc_C = 0.01F;/* Variable: VMC_tiVelGvnrIntglUpdCyc_C
                                             * Referenced by: '<S135>/VMC_tiVelGvnrIntglUpdCyc_C'
                                             */
real32_T VMC_vAccnGvnrDWinNeg_C = -0.8F;/* Variable: VMC_vAccnGvnrDWinNeg_C
                                         * Referenced by: '<S121>/VMC_vAccnGvnrDWinNeg_C'
                                         */
real32_T VMC_vAccnGvnrDWinPos_C = 0.8F;/* Variable: VMC_vAccnGvnrDWinPos_C
                                        * Referenced by: '<S121>/VMC_vAccnGvnrDWinPos_C'
                                        */
real32_T VMC_vAccnGvnrIWinNeg_C = -1.5F;/* Variable: VMC_vAccnGvnrIWinNeg_C
                                         * Referenced by: '<S121>/VMC_vAccnGvnrIWinNeg_C'
                                         */
real32_T VMC_vAccnGvnrIWinPos_C = 1.5F;/* Variable: VMC_vAccnGvnrIWinPos_C
                                        * Referenced by: '<S121>/VMC_vAccnGvnrIWinPos_C'
                                        */
real32_T VMC_vAccnGvnrPWinNeg_C = -1.2F;/* Variable: VMC_vAccnGvnrPWinNeg_C
                                         * Referenced by: '<S121>/VMC_vAccnGvnrPWinNeg_C'
                                         */
real32_T VMC_vAccnGvnrPWinPos_C = 1.2F;/* Variable: VMC_vAccnGvnrPWinPos_C
                                        * Referenced by: '<S121>/VMC_vAccnGvnrPWinPos_C'
                                        */
real32_T VMC_vDifThd2I_C = 5.0F;       /* Variable: VMC_vDifThd2I_C
                                        * Referenced by: '<S120>/VMC_vDifThd2I_C'
                                        */
real32_T VMC_vKpDecThd_C = 8.0F;       /* Variable: VMC_vKpDecThd_C
                                        * Referenced by: '<S121>/VMC_vKpDecThd_C'
                                        */
real32_T VMC_vVelGvnrDftlIniVal_C = 0.0F;/* Variable: VMC_vVelGvnrDftlIniVal_C
                                          * Referenced by: '<S135>/VMC_vVelGvnrDftlIniVal_C'
                                          */
real32_T VMC_vVelGvnrIntglCtrlValLo_C = 0.0F;/* Variable: VMC_vVelGvnrIntglCtrlValLo_C
                                              * Referenced by: '<S137>/VMC_vVelGvnrIntglCtrlValLo_C'
                                              */
real32_T VMC_vVelGvnrIntglCtrlValUp_C = 0.0F;/* Variable: VMC_vVelGvnrIntglCtrlValUp_C
                                              * Referenced by: '<S137>/VMC_vVelGvnrIntglCtrlValUp_C'
                                              */
real32_T VMC_vVelGvnrIntglIniVal_C = 0.0F;/* Variable: VMC_vVelGvnrIntglIniVal_C
                                           * Referenced by: '<S135>/VMC_vVelGvnrIntglIniVal_C'
                                           */
real32_T VMC_vVelGvnrOutpLo_C = -3.0F; /* Variable: VMC_vVelGvnrOutpLo_C
                                        * Referenced by: '<S142>/VMC_vVelGvnrOutpLo_C'
                                        */
real32_T VMC_vVelGvnrOutpUp_C = 3.0F;  /* Variable: VMC_vVelGvnrOutpUp_C
                                        * Referenced by: '<S142>/VMC_vVelGvnrOutpUp_C'
                                        */
real32_T VehDa_facSlpFilt_C = 0.97F;   /* Variable: VehDa_facSlpFilt_C
                                        * Referenced by: '<S165>/VehDa_facSlpFilt_C'
                                        */
real32_T VehDa_mWght_C = 9000.0F;      /* Variable: VehDa_mWght_C
                                        * Referenced by: '<S170>/VehDa_mWght_C'
                                        */
real32_T VehDa_rSlop_C = 0.0F;         /* Variable: VehDa_rSlop_C
                                        * Referenced by: '<S165>/VehDa_rSlop_C'
                                        */
real32_T VehDa_rSlpCmp_C = 0.0F;       /* Variable: VehDa_rSlpCmp_C
                                        * Referenced by: '<S165>/VehDa_rSlpCmp_C'
                                        */
boolean_T Debug_swtEngSpdSlect_C = false;/* Variable: Debug_swtEngSpdSlect_C
                                          * Referenced by: '<S1>/Debug_swtEngSpdSlect_C'
                                          */
int8_T Tra_stDflNeutrGear_C = 0;       /* Variable: Tra_stDflNeutrGear_C
                                        * Referenced by: '<S85>/Tra_stDflNeutrGear_C'
                                        */
uint8_T ACCS_stACCActvdTest_C = 0U;    /* Variable: ACCS_stACCActvdTest_C
                                        * Referenced by: '<S2>/ACCS_stACCActvdTest_C'
                                        */
uint8_T ACCS_stACCActvd_C = 1U;        /* Variable: ACCS_stACCActvd_C
                                        * Referenced by: '<S21>/ACCS_stACCActvd_C'
                                        */
uint8_T ACCS_stDstSpCalReqd_C = 0U;    /* Variable: ACCS_stDstSpCalReqd_C
                                        * Referenced by: '<S35>/ACCS_stDstSpCalReqd_C'
                                        */
uint8_T ACCS_stFrontTargetSwt_C = 0U;  /* Variable: ACCS_stFrontTargetSwt_C
                                        * Referenced by: '<S22>/ACCS_stFrontTargetSwt_C'
                                        */
uint8_T ACCS_stVSpCalReqd_C = 0U;      /* Variable: ACCS_stVSpCalReqd_C
                                        * Referenced by: '<S35>/ACCS_stVSpCalReqd_C'
                                        */
uint8_T ACCS_swtDstToSpd_C = 0U;       /* Variable: ACCS_swtDstToSpd_C
                                        * Referenced by: '<S21>/ACCS_swtDstToSpd_C'
                                        */
uint8_T ACC_RXFlag = 0U;               /* Variable: ACC_RXFlag
                                        * Referenced by: '<S14>/ACC_RXFlag'
                                        */
uint8_T AEBS_swtFcnActvd_C = 0U;       /* Variable: AEBS_swtFcnActvd_C
                                        * Referenced by: '<S16>/AEBS_swtFcnActvd_C'
                                        */
uint8_T CoChs_stActvTst_C = 0U;        /* Variable: CoChs_stActvTst_C
                                        * Referenced by: '<S38>/CoChs_stActvTst_C'
                                        */
uint8_T CoChs_stClbEna_C = 0U;         /* Variable: CoChs_stClbEna_C
                                        * Referenced by: '<S38>/CoChs_stClbEna_C'
                                        */
uint8_T CoChs_stLgtCtrlEna_C = 1U;     /* Variable: CoChs_stLgtCtrlEna_C
                                        * Referenced by: '<S38>/CoChs_stLgtCtrlEna_C'
                                        */
uint8_T CoChs_swtStartingEna_C = 0U;   /* Variable: CoChs_swtStartingEna_C
                                        * Referenced by: '<S1>/CoChs_swtStartingEna_C'
                                        */
uint8_T DRMC_stClsLoop_C = 2U;         /* Variable: DRMC_stClsLoop_C
                                        * Referenced by: '<S50>/DRMC_stClsLoop_C'
                                        */
uint8_T EBS_stStpCalcSwt_C = 1U;       /* Variable: EBS_stStpCalcSwt_C
                                        * Referenced by: '<S88>/EBS_stStpCalcSwt_C'
                                        */
uint8_T EBS_swtRawAccrCacl_C = 1U;     /* Variable: EBS_swtRawAccrCacl_C
                                        * Referenced by: '<S87>/EBS_swtRawAccrCacl_C'
                                        */
uint8_T Eng_stPedlFrz_C = 1U;          /* Variable: Eng_stPedlFrz_C
                                        * Referenced by: '<S63>/Eng_stPedlFrz_C'
                                        */
uint8_T Eng_swtTrqLimCmpFild_C = 0U;   /* Variable: Eng_swtTrqLimCmpFild_C
                                        * Referenced by: '<S64>/Eng_swtTrqLimCmpFild_C'
                                        */
uint8_T EnvDetn_idCllsnTar_C = 1U;     /* Variable: EnvDetn_idCllsnTar_C
                                        * Referenced by: '<S20>/EnvDetn_idCllsnTar_C'
                                        */
uint8_T LgtCtrl_RXFlag = 0U;           /* Variable: LgtCtrl_RXFlag
                                        * Referenced by: '<S14>/LgtCtrl_RXFlag'
                                        */
uint8_T MFLv_stDstDecSwt_C = 0U;       /* Variable: MFLv_stDstDecSwt_C
                                        * Referenced by: '<S2>/MFLv_stDstDecSwt_C'
                                        */
uint8_T MFLv_stDstIncSwt_C = 0U;       /* Variable: MFLv_stDstIncSwt_C
                                        * Referenced by: '<S2>/MFLv_stDstIncSwt_C'
                                        */
uint8_T MFLv_stOffSwt_C = 0U;          /* Variable: MFLv_stOffSwt_C
                                        * Referenced by: '<S2>/MFLv_stOffSwt_C'
                                        */
uint8_T MFLv_stResuSwt_C = 0U;         /* Variable: MFLv_stResuSwt_C
                                        * Referenced by: '<S2>/MFLv_stResuSwt_C'
                                        */
uint8_T MFLv_stSpdDecSwt_C = 0U;       /* Variable: MFLv_stSpdDecSwt_C
                                        * Referenced by: '<S2>/MFLv_stSpdDecSwt_C'
                                        */
uint8_T MFLv_stSpdIncSwt_C = 0U;       /* Variable: MFLv_stSpdIncSwt_C
                                        * Referenced by: '<S2>/MFLv_stSpdIncSwt_C'
                                        */
uint8_T Sys_stADMd_C = 3U;             /* Variable: Sys_stADMd_C
                                        * Referenced by: '<S146>/Sys_stADMd_C'
                                        */
uint8_T Test_stOverflow_C = 0U;        /* Variable: Test_stOverflow_C
                                        * Referenced by: '<S165>/Test_stOverflow_C'
                                        */
uint8_T VDC_swtNomAccrCalc_C = 1U;     /* Variable: VDC_swtNomAccrCalc_C
                                        * Referenced by: '<S104>/VDC_swtNomAccrCalc_C'
                                        */
uint8_T VMC_Ienable_C = 0U;            /* Variable: VMC_Ienable_C
                                        * Referenced by: '<S121>/VMC_Ienable_C'
                                        */
uint8_T VMC_Penable_C = 0U;            /* Variable: VMC_Penable_C
                                        * Referenced by: '<S121>/VMC_Penable_C'
                                        */
uint8_T VMC_swtLocnDifFild_C = 0U;     /* Variable: VMC_swtLocnDifFild_C
                                        * Referenced by: '<S141>/VMC_swtLocnDifFild_C'
                                        */
uint8_T VMC_swtSpdDifFild_C = 0U;      /* Variable: VMC_swtSpdDifFild_C
                                        * Referenced by: '<S128>/VMC_swtSpdDifFild_C'
                                        */
uint8_T VMC_swtVelGvnrFctEna_C = 0U;   /* Variable: VMC_swtVelGvnrFctEna_C
                                        * Referenced by: '<S137>/VMC_swtVelGvnrFctEna_C'
                                        */
uint8_T VehDa_swtSlpFilt_C = 0U;       /* Variable: VehDa_swtSlpFilt_C
                                        * Referenced by: '<S165>/VehDa_swtSlpFilt_C'
                                        */
uint8_T VehDa_swtSlpSrc_C = 1U;        /* Variable: VehDa_swtSlpSrc_C
                                        * Referenced by: '<S165>/VehDa_swtSlpSrc_C'
                                        */

/* Invariant block signals (default storage) */
const ConstB_LgtCtrl_T LgtCtrl_ConstB = {
  0.0,                                 /* '<S90>/G1' */
  29.0,                                /* '<S90>/Add' */
  1.0,                                 /* '<S96>/Swt3' */
  0.0,                                 /* '<S97>/Divide2' */
  -0.001,                              /* '<S125>/P1' */
  0.001,                               /* '<S125>/P2' */
  0.8F,                                /* '<S115>/Product8' */
  -0.8F,                               /* '<S115>/Product9' */
  0U,                                  /* '<S7>/Data Type Conversion3' */
  11U,                                 /* '<S97>/Data Type Conversion12' */
  4U,                                  /* '<S97>/Data Type Conversion13' */
  12U,                                 /* '<S97>/Data Type Conversion14' */
  0U,                                  /* '<S97>/Data Type Conversion16' */
  255U,                                /* '<S97>/Data Type Conversion7' */
  255U,                                /* '<S97>/Data Type Conversion8' */
  255U,                                /* '<S97>/Data Type Conversion9' */
  0,                                   /* '<S62>/R' */
  0,                                   /* '<S117>/R3' */
  0,                                   /* '<S114>/R2' */
  0,                                   /* '<S115>/R2' */
  0,                                   /* '<S118>/R3' */
  1                                    /* '<S110>/L3' */
};

/* Constant parameters (default storage) */
const ConstP_LgtCtrl_T LgtCtrl_ConstP = {
  /* Computed Parameter: uDLookupTable_tableData
   * Referenced by: '<S103>/1-D Lookup Table'
   */
  { -1.0F, -0.999848F, -0.999391F, -0.99863F, -0.997564F, -0.996195F, -0.994522F,
    -0.992546F, -0.990268F, -0.987688F, -0.984808F, -0.981627F, -0.978148F,
    -0.97437F, -0.970296F, -0.965926F, -0.961262F, -0.956305F, -0.951057F,
    -0.945519F, -0.939693F, -0.93358F, -0.927184F, -0.920505F, -0.913545F,
    -0.906308F, -0.898794F, -0.891007F, -0.882948F, -0.87462F, -0.866025F,
    -0.857167F, -0.848048F, -0.838671F, -0.829038F, -0.819152F, -0.809017F,
    -0.798636F, -0.788011F, -0.777146F, -0.766044F, -0.75471F, -0.743145F,
    -0.731354F, -0.71934F, -0.707107F, -0.694658F, -0.681998F, -0.669131F,
    -0.656059F, -0.642788F, -0.62932F, -0.615661F, -0.601815F, -0.587785F,
    -0.573576F, -0.559193F, -0.544639F, -0.529919F, -0.515038F, -0.5F, -0.48481F,
    -0.469472F, -0.45399F, -0.438371F, -0.422618F, -0.406737F, -0.390731F,
    -0.374607F, -0.358368F, -0.34202F, -0.325568F, -0.309017F, -0.292372F,
    -0.275637F, -0.258819F, -0.241922F, -0.224951F, -0.207912F, -0.190809F,
    -0.173648F, -0.156434F, -0.139173F, -0.121869F, -0.104528F, -0.087156F,
    -0.069756F, -0.052336F, -0.034899F, -0.017452F, 0.0F, 0.017452F, 0.034899F,
    0.052336F, 0.069756F, 0.087156F, 0.104528F, 0.121869F, 0.139173F, 0.156434F,
    0.173648F, 0.190809F, 0.207912F, 0.224951F, 0.241922F, 0.258819F, 0.275637F,
    0.292372F, 0.309017F, 0.325568F, 0.34202F, 0.358368F, 0.374607F, 0.390731F,
    0.406737F, 0.422618F, 0.438371F, 0.45399F, 0.469472F, 0.48481F, 0.5F,
    0.515038F, 0.529919F, 0.544639F, 0.559193F, 0.573576F, 0.587785F, 0.601815F,
    0.615661F, 0.62932F, 0.642788F, 0.656059F, 0.669131F, 0.681998F, 0.694658F,
    0.707107F, 0.71934F, 0.731354F, 0.743145F, 0.75471F, 0.766044F, 0.777146F,
    0.788011F, 0.798636F, 0.809017F, 0.819152F, 0.829038F, 0.838671F, 0.848048F,
    0.857167F, 0.866025F, 0.87462F, 0.882948F, 0.891007F, 0.898794F, 0.906308F,
    0.913545F, 0.920505F, 0.927184F, 0.93358F, 0.939693F, 0.945519F, 0.951057F,
    0.956305F, 0.961262F, 0.965926F, 0.970296F, 0.97437F, 0.978148F, 0.981627F,
    0.984808F, 0.987688F, 0.990268F, 0.992546F, 0.994522F, 0.996195F, 0.997564F,
    0.99863F, 0.999391F, 0.999848F, 1.0F },

  /* Computed Parameter: uDLookupTable_bp01Data
   * Referenced by: '<S103>/1-D Lookup Table'
   */
  { -90.0F, -89.0F, -88.0F, -87.0F, -86.0F, -85.0F, -84.0F, -83.0F, -82.0F,
    -81.0F, -80.0F, -79.0F, -78.0F, -77.0F, -76.0F, -75.0F, -74.0F, -73.0F,
    -72.0F, -71.0F, -70.0F, -69.0F, -68.0F, -67.0F, -66.0F, -65.0F, -64.0F,
    -63.0F, -62.0F, -61.0F, -60.0F, -59.0F, -58.0F, -57.0F, -56.0F, -55.0F,
    -54.0F, -53.0F, -52.0F, -51.0F, -50.0F, -49.0F, -48.0F, -47.0F, -46.0F,
    -45.0F, -44.0F, -43.0F, -42.0F, -41.0F, -40.0F, -39.0F, -38.0F, -37.0F,
    -36.0F, -35.0F, -34.0F, -33.0F, -32.0F, -31.0F, -30.0F, -29.0F, -28.0F,
    -27.0F, -26.0F, -25.0F, -24.0F, -23.0F, -22.0F, -21.0F, -20.0F, -19.0F,
    -18.0F, -17.0F, -16.0F, -15.0F, -14.0F, -13.0F, -12.0F, -11.0F, -10.0F,
    -9.0F, -8.0F, -7.0F, -6.0F, -5.0F, -4.0F, -3.0F, -2.0F, -1.0F, 0.0F, 1.0F,
    2.0F, 3.0F, 4.0F, 5.0F, 6.0F, 7.0F, 8.0F, 9.0F, 10.0F, 11.0F, 12.0F, 13.0F,
    14.0F, 15.0F, 16.0F, 17.0F, 18.0F, 19.0F, 20.0F, 21.0F, 22.0F, 23.0F, 24.0F,
    25.0F, 26.0F, 27.0F, 28.0F, 29.0F, 30.0F, 31.0F, 32.0F, 33.0F, 34.0F, 35.0F,
    36.0F, 37.0F, 38.0F, 39.0F, 40.0F, 41.0F, 42.0F, 43.0F, 44.0F, 45.0F, 46.0F,
    47.0F, 48.0F, 49.0F, 50.0F, 51.0F, 52.0F, 53.0F, 54.0F, 55.0F, 56.0F, 57.0F,
    58.0F, 59.0F, 60.0F, 61.0F, 62.0F, 63.0F, 64.0F, 65.0F, 66.0F, 67.0F, 68.0F,
    69.0F, 70.0F, 71.0F, 72.0F, 73.0F, 74.0F, 75.0F, 76.0F, 77.0F, 78.0F, 79.0F,
    80.0F, 81.0F, 82.0F, 83.0F, 84.0F, 85.0F, 86.0F, 87.0F, 88.0F, 89.0F, 90.0F
  },

  /* Computed Parameter: H19EDci45051MAPA_tableData
   * Referenced by: '<S63>/H19E-Dci450-51-MAPA'
   */
  { -150.0F, -200.0F, -250.0F, -300.0F, -350.0F, -400.0F, -450.0F, -450.0F,
    -450.0F, -450.0F, -550.0F, -650.0F, -750.0F, -850.0F, -950.0F, -1050.0F,
    -1150.0F, -1250.0F, -1350.0F, -1450.0F, -1550.0F, -1650.0F, 150.0F, 100.0F,
    50.0F, 0.0F, -50.0F, -100.0F, -150.0F, -150.0F, -150.0F, -150.0F, -270.0F,
    -390.0F, -510.0F, -630.0F, -750.0F, -870.0F, -990.0F, -1110.0F, -1230.0F,
    -1350.0F, -1470.0F, -1590.0F, 852.0F, 752.0F, 652.0F, 552.0F, 452.0F, 352.0F,
    252.0F, 252.0F, 252.0F, 252.0F, 132.0F, 12.0F, -108.0F, -228.0F, -348.0F,
    -468.0F, -588.0F, -708.0F, -828.0F, -948.0F, -1068.0F, -1188.0F, 1254.0F,
    1154.0F, 1054.0F, 954.0F, 854.0F, 754.0F, 654.0F, 654.0F, 654.0F, 654.0F,
    534.0F, 414.0F, 294.0F, 174.0F, 54.0F, -66.0F, -186.0F, -306.0F, -426.0F,
    -546.0F, -666.0F, -786.0F, 2058.0F, 1958.0F, 1858.0F, 1758.0F, 1658.0F,
    1558.0F, 1458.0F, 1458.0F, 1458.0F, 1458.0F, 1338.0F, 1218.0F, 1098.0F,
    978.0F, 858.0F, 738.0F, 618.0F, 498.0F, 378.0F, 258.0F, 138.0F, 18.0F,
    2402.0F, 2302.0F, 2202.0F, 2102.0F, 2002.0F, 1902.0F, 1802.0F, 1802.0F,
    1802.0F, 1802.0F, 1682.0F, 1562.0F, 1442.0F, 1322.0F, 1202.0F, 1082.0F,
    962.0F, 842.0F, 722.0F, 602.0F, 482.0F, 362.0F, 2694.0F, 2594.0F, 2494.0F,
    2394.0F, 2294.0F, 2194.0F, 2094.0F, 2094.0F, 2094.0F, 2094.0F, 1974.0F,
    1854.0F, 1734.0F, 1614.0F, 1494.0F, 1374.0F, 1254.0F, 1134.0F, 1014.0F,
    894.0F, 774.0F, 654.0F, 2985.0F, 2885.0F, 2785.0F, 2685.0F, 2585.0F, 2485.0F,
    2385.0F, 2385.0F, 2385.0F, 2385.0F, 2265.0F, 2145.0F, 2025.0F, 1905.0F,
    1785.0F, 1665.0F, 1545.0F, 1425.0F, 1305.0F, 1185.0F, 1065.0F, 945.0F,
    3400.0F, 3300.0F, 3200.0F, 3100.0F, 3000.0F, 2900.0F, 2800.0F, 2800.0F,
    2800.0F, 2800.0F, 2800.0F, 2710.0F, 2620.0F, 2530.0F, 2440.0F, 2350.0F,
    2260.0F, 2170.0F, 2080.0F, 1990.0F, 1900.0F, 1810.0F, 3400.0F, 3300.0F,
    3200.0F, 3100.0F, 3000.0F, 2900.0F, 2800.0F, 2800.0F, 2800.0F, 2800.0F,
    2800.0F, 2710.0F, 2620.0F, 2530.0F, 2440.0F, 2350.0F, 2260.0F, 2170.0F,
    2080.0F, 1990.0F, 1900.0F, 1810.0F },

  /* Computed Parameter: H19EDci45051MAPA_bp01Data
   * Referenced by: '<S63>/H19E-Dci450-51-MAPA'
   */
  { 0.0F, 500.0F, 600.0F, 700.0F, 800.0F, 900.0F, 1000.0F, 1100.0F, 1200.0F,
    1300.0F, 1400.0F, 1500.0F, 1600.0F, 1700.0F, 1800.0F, 1900.0F, 2000.0F,
    2100.0F, 2200.0F, 2300.0F, 2400.0F, 2500.0F },

  /* Pooled Parameter (Expression: [0	2	10	20	40	50	60	70	95	100])
   * Referenced by:
   *   '<S63>/Thr_H19E'
   *   '<S63>/H19E-Dci450-51-MAPA'
   */
  { 0.0F, 2.0F, 10.0F, 20.0F, 40.0F, 50.0F, 60.0F, 70.0F, 95.0F, 100.0F },

  /* Computed Parameter: KAX1EQH175T3MAPFULL_tableData
   * Referenced by: '<S63>/KAX1-EQH175-T3-MAP-FULL'
   */
  { 430.0F, 430.0F, 430.0F, 510.0F, 565.0F, 620.0F, 720.0F, 800.0F, 880.0F,
    880.0F, 880.0F, 880.0F, 880.0F, 865.0F, 845.0F, 820.0F, 800.0F, 778.0F,
    750.0F, 726.0F, 695.0F, 375.0F, 215.0F, 130.0F },

  /* Computed Parameter: KAX1EQH175T3MAPFULL_bp01Data
   * Referenced by: '<S63>/KAX1-EQH175-T3-MAP-FULL'
   */
  { 0.0F, 300.0F, 400.0F, 700.0F, 800.0F, 900.0F, 1000.0F, 1100.0F, 1200.0F,
    1300.0F, 1400.0F, 1500.0F, 1600.0F, 1700.0F, 1800.0F, 1900.0F, 2000.0F,
    2100.0F, 2200.0F, 2300.0F, 2400.0F, 2500.0F, 2600.0F, 2650.0F },

  /* Computed Parameter: KAX1EQH175T3MAP_tableData
   * Referenced by: '<S63>/KAX1-EQH175-T3-MAP'
   */
  { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 49.5401F, 30.8984375F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 89.1815186F, 83.4016418F,
    102.348633F, 99.909668F, 109.758301F, 67.3535156F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 132.122498F, 123.559265F, 149.589844F,
    145.283203F, 159.602051F, 134.868164F, 93.269043F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 175.063477F, 163.716888F, 196.831055F,
    188.261719F, 206.572266F, 174.882812F, 139.890137F, 60.3451538F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 214.704895F, 200.788116F, 240.13916F,
    228.845215F, 251.152344F, 212.211914F, 186.591797F, 120.795898F, 61.1418152F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 247.731628F, 231.686554F,
    279.492188F, 268.659668F, 294.282227F, 248.62793F, 233.293457F, 181.325836F,
    122.257843F, 60.7592773F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 313.800659F,
    293.466187F, 358.220215F, 345.080566F, 376.728516F, 317.861328F, 297.827148F,
    302.121735F, 244.360962F, 182.127686F, 119.897461F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 383.153687F, 358.332214F, 436.970215F, 421.501465F, 458.691406F,
    386.208496F, 361.071777F, 365.317841F, 366.464081F, 303.496094F, 239.819336F,
    133.243896F, 0.0F, 0.0F, 0.0F, 0.0F, 462.436523F, 432.47467F, 523.564453F,
    501.899414F, 543.527832F, 456.35498F, 426.035156F, 428.93631F, 428.585815F,
    424.889526F, 359.716797F, 266.487793F, 137.535095F, 0.0F, 0.0F, 0.0F,
    483.572388F, 518.98F, 621.98F, 591.04248F, 636.044922F, 532.785645F,
    496.262207F, 497.016F, 494.292F, 487.575684F, 479.638672F, 399.707947F,
    275.093079F, 132.735168F, 0.0F, 0.0F, 483.572388F, 539.981262F, 688.117676F,
    693.325195F, 740.996094F, 618.642578F, 574.304199F, 572.223083F, 565.877686F,
    555.692139F, 545.068359F, 533.141785F, 412.765503F, 265.603271F, 143.75946F,
    0.0F, 487.541199F, 543.378F, 692.446289F, 696.115723F, 799.943848F,
    666.767578F, 617.91748F, 613.588257F, 605.048645F, 592.227783F, 580.322266F,
    566.856384F, 481.452942F, 331.871155F, 215.448303F, 59.0927124F, 493.15979F,
    548.61969F, 699.126F, 701.696777F, 863.69873F, 718.920898F, 665.022F,
    658.094788F, 647.365723F, 631.916504F, 618.310547F, 602.897766F, 550.323486F,
    398.33844F, 287.391663F, 82.7258301F, 500.755F, 555.740784F, 708.200684F,
    709.650879F, 870.412598F, 775.559082F, 716.477051F, 706.587341F, 692.828796F,
    674.307861F, 658.569336F, 641.242188F, 619.010925F, 464.606323F, 463.326263F,
    106.362915F, 510.0F, 565.0F, 720.0F, 720.0F, 880.0F, 880.0F, 880.0F, 865.0F,
    845.0F, 820.0F, 800.0F, 778.0F, 750.0F, 726.0F, 695.0F, 130.0F },

  /* Computed Parameter: KAX1EQH175T3MAP_bp01Data
   * Referenced by: '<S63>/KAX1-EQH175-T3-MAP'
   */
  { 700.0F, 800.0F, 1000.0F, 1100.0F, 1200.0F, 1400.0F, 1600.0F, 1700.0F,
    1800.0F, 1900.0F, 2000.0F, 2100.0F, 2200.0F, 2300.0F, 2400.0F, 2650.0F },

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S63>/Thr_KAX1'
   *   '<S63>/KAX1-EQH175-T3-MAP'
   */
  { 0.0F, 5.0F, 10.0F, 15.0F, 20.0F, 25.0F, 30.0F, 40.0F, 50.0F, 60.0F, 70.0F,
    80.0F, 85.0F, 90.0F, 95.0F, 100.0F },

  /* Computed Parameter: H19EDci45051MAPAFULL_tableData
   * Referenced by: '<S63>/H19E-Dci450-51-MAPA-FULL'
   */
  { 1000.0F, 1670.0F, 2100.0F, 2500.0F, 2650.0F, 2650.0F, 2650.0F, 2650.0F,
    2650.0F, 2650.0F, 2494.0F, 2348.0F, 2217.0F, 2181.0F, 0.0F },

  /* Computed Parameter: H19EDci45051MAPAFULL_bp01Data
   * Referenced by: '<S63>/H19E-Dci450-51-MAPA-FULL'
   */
  { 650.0F, 700.0F, 800.0F, 900.0F, 960.0F, 1100.0F, 1200.0F, 1300.0F, 1400.0F,
    1500.0F, 1600.0F, 1700.0F, 1800.0F, 1830.0F, 2000.0F }
};

/* Block signals (default storage) */
B_LgtCtrl_T LgtCtrl_B;

/* Block states (default storage) */
DW_LgtCtrl_T LgtCtrl_DW;
static void FeedForwardAccrCalcn_100ms_Init(void);
static void Lgt_FeedForwardAccrCalcn_100ms1(real32_T rtu_speed,
  DW_FeedForwardAccrCalcn_100ms_T *localDW);

/* Forward declaration for local functions */
static void LgtCtrl_enter_internal_Working(const real32_T *UnitDelay, const
  real32_T *Switch, real32_T *ACCS_stActvd);

/*
 * System initialize for atomic system:
 *    '<S22>/FeedForwardAccrCalcn_100ms1'
 *    '<S29>/FeedForwardAccrCalcn_100ms1'
 */
static void FeedForwardAccrCalcn_100ms_Init(void)
{
  int32_T rtb_accr_i;
  rtb_accr_i = 0;
}

/*
 * Output and update for atomic system:
 *    '<S22>/FeedForwardAccrCalcn_100ms1'
 *    '<S29>/FeedForwardAccrCalcn_100ms1'
 */
static void Lgt_FeedForwardAccrCalcn_100ms1(real32_T rtu_speed,
  DW_FeedForwardAccrCalcn_100ms_T *localDW)
{
  int32_T tmp;
  real32_T rtb_accr_az;

  /* Chart: '<S22>/FeedForwardAccrCalcn_100ms1' */
  /* Gateway: LgtCtrl/ACCS_Decision/ACCS_DesSpdCalcn/SetPointRamp/FeedForwardAccrCalcn_100ms1 */
  /* During: LgtCtrl/ACCS_Decision/ACCS_DesSpdCalcn/SetPointRamp/FeedForwardAccrCalcn_100ms1 */
  /* Entry Internal: LgtCtrl/ACCS_Decision/ACCS_DesSpdCalcn/SetPointRamp/FeedForwardAccrCalcn_100ms1 */
  /* Transition: '<S27>:2' */
  localDW->counter++;
  localDW->counter = fminf(localDW->counter, 5.0F);
  while (localDW->index < 3.0F) {
    int32_T tmp_0;

    /* Transition: '<S27>:6' */
    /* Transition: '<S27>:7' */
    if (localDW->index < 2.14748365E+9F) {
      if (localDW->index >= -2.14748365E+9F) {
        tmp = (int32_T)localDW->index;
      } else {
        tmp = MIN_int32_T;
      }
    } else {
      tmp = MAX_int32_T;
    }

    if (localDW->index + 1.0F < 2.14748365E+9F) {
      if (localDW->index + 1.0F >= -2.14748365E+9F) {
        tmp_0 = (int32_T)(localDW->index + 1.0F);
      } else {
        tmp_0 = MIN_int32_T;
      }
    } else {
      tmp_0 = MAX_int32_T;
    }

    localDW->SpeedAry[tmp] = localDW->SpeedAry[tmp_0];
    localDW->index++;

    /* Transition: '<S27>:8' */
  }

  /* Transition: '<S27>:30' */
  if (localDW->index < 2.14748365E+9F) {
    if (localDW->index >= -2.14748365E+9F) {
      tmp = (int32_T)localDW->index;
    } else {
      tmp = MIN_int32_T;
    }
  } else {
    tmp = MAX_int32_T;
  }

  localDW->SpeedAry[tmp] = rtu_speed;
  localDW->index = 0.0F;
  if (localDW->counter == 5.0F) {
    /* Transition: '<S27>:16' */
    /* Transition: '<S27>:19' */
    rtb_accr_az = (real32_T)((localDW->SpeedAry[3] - localDW->SpeedAry[0]) / 3.6
      / 0.3);

    /* Transition: '<S27>:20' */
  } else {
    /* Transition: '<S27>:21' */
    rtb_accr_az = 0.0F;
  }

  /* End of Chart: '<S22>/FeedForwardAccrCalcn_100ms1' */
}

/* Function for Chart: '<S33>/StMng' */
static void LgtCtrl_enter_internal_Working(const real32_T *UnitDelay, const
  real32_T *Switch, real32_T *ACCS_stActvd)
{
  /* Constant: '<S2>/DE_rVehAccrPedl' incorporates:
   *  Constant: '<S2>/CANMng_rEEC1ActEngTrq'
   */
  /* Entry Internal 'Working': '<S34>:29' */
  /* Transition: '<S34>:31' */
  if ((0.0 >= 0.0 + 1.0) && (0.0 > 5.0)) {
    /* Transition: '<S34>:33' */
    LgtCtrl_DW.is_Working = LgtCtrl_IN_OverTake;

    /* Entry 'OverTake': '<S34>:4' */
    *ACCS_stActvd = 3.0F;
  } else {
    /* Transition: '<S34>:32' */
    LgtCtrl_DW.is_Working = LgtCtrl_IN_Active;

    /* Constant: '<S33>/ACCS_vStpThd_C' */
    /* Entry 'Active': '<S34>:3' */
    /* Entry Internal 'Active': '<S34>:3' */
    /* Transition: '<S34>:75' */
    if ((VehDa_vEgoSpd < ACCS_vStpThd_C) && (*UnitDelay <= *Switch)) {
      /* Transition: '<S34>:85' */
      LgtCtrl_DW.is_Active = LgtCtrl_IN_Parking;

      /* Entry 'Parking': '<S34>:84' */
      *ACCS_stActvd = 11.0F;
    } else if ((VehDa_vEgoSpd < *Switch) && (*UnitDelay > *Switch)) {
      /* Transition: '<S34>:74' */
      LgtCtrl_DW.is_Active = LgtCtrl_IN_Starting;

      /* Entry 'Starting': '<S34>:81' */
      *ACCS_stActvd = 5.0F;
    } else {
      /* Transition: '<S34>:73' */
      LgtCtrl_DW.is_Active = LgtCtrl_IN_Cruise;

      /* Entry 'Cruise': '<S34>:80' */
      *ACCS_stActvd = 2.0F;
    }

    /* End of Constant: '<S33>/ACCS_vStpThd_C' */
  }

  /* End of Constant: '<S2>/DE_rVehAccrPedl' */
}

/* Model step function */
void LgtCtrl_step(void)
{
  real_T rtb_MinMax;
  real_T rtb_Sum6;
  real_T rtb_Switch;
  real_T rtb_Switch_cx;
  real_T rtb_UnitDelay_lk;
  int32_T i;
  int32_T judge;
  real32_T rtb_KAX1EQH175T3MAP[16];
  real32_T rtb_H19EDci45051MAPA[10];
  real32_T fractions[2];
  real32_T fractions_0[2];
  real32_T ACCS_stActvd;
  real32_T LgtDstRel;
  real32_T Switch;
  real32_T UnitDelay;
  real32_T rtb_A_iw;
  real32_T rtb_Add_lp;
  real32_T rtb_D1_f;
  real32_T rtb_D1_i;
  real32_T rtb_Gain;
  real32_T rtb_KAX1EQH175T3MAPFULL;
  real32_T rtb_MinMax1_p;
  real32_T rtb_MinMax2_h;
  real32_T rtb_Switch1;
  real32_T rtb_Swt1;
  real32_T rtb_Swt1_a;
  real32_T rtb_Swt1_o_idx_2;
  real32_T rtb_Swt1_o_idx_4;
  real32_T rtb_Swt2;
  real32_T rtb_Swt_lo;
  real32_T rtb_Swt_ot;
  real32_T rtb_deltafalllimit;
  real32_T rtb_deltariselimit;
  real32_T rtb_deltariselimit_b;
  real32_T rtb_sampletime;
  uint32_T bpIndices[2];
  uint32_T bpIndices_0[2];
  int16_T u;
  uint16_T rtb_Switch1_b;
  uint16_T rtb_Switch1_i;
  int8_T rtb_UnitDelay1_bt;
  uint8_T DataTypeConversion10;
  uint8_T rtb_DataTypeConversion6_g;
  uint8_T rtb_Swt2_n;
  boolean_T LogicalOperator2;
  boolean_T guard1 = false;
  boolean_T rtb_L4_c;
  boolean_T rtb_LogicalOperator3;
  boolean_T rtb_RelationalOperator1;

  /* Switch: '<S15>/Switch' incorporates:
   *  Constant: '<S15>/Constant'
   *  Constant: '<S15>/Constant2'
   *  Constant: '<S15>/Constant3'
   *  RelationalOperator: '<S15>/Relational Operator'
   *  Sum: '<S15>/Add'
   *  UnitDelay: '<S15>/Unit Delay'
   */
  if (LgtCtrl_DW.UnitDelay_DSTATE < 10.0) {
    rtb_Switch = 1.0 + LgtCtrl_DW.UnitDelay_DSTATE;
  } else {
    rtb_Switch = 1.0;
  }

  /* End of Switch: '<S15>/Switch' */

  /* RelationalOperator: '<S15>/Relational Operator1' incorporates:
   *  Constant: '<S15>/Constant3'
   */
  rtb_RelationalOperator1 = (rtb_Switch == 1.0);

  /* Gain: '<S155>/Gain' incorporates:
   *  Inport: '<Root>/VehDa_vEgoSpd_mp'
   */
  VehDa_vEgoSpd = 1.0F * VehDa_vEgoSpd_mp;

  /* Switch: '<S20>/Swt2' incorporates:
   *  Constant: '<S20>/EnvDetn_vCllsnTar_C'
   *  Constant: '<S2>/ACCS_stACCActvdTest_C'
   */
  if (ACCS_stACCActvdTest_C != 0) {
    /* Switch: '<S20>/Swt5' incorporates:
     *  Constant: '<S20>/ACCS_dtFrontTrgAccrSlpUp_C'
     *  Constant: '<S20>/EnvDetn_tiUpdate_C'
     *  Constant: '<S20>/EnvDetn_vCllsnTar_C'
     *  Gain: '<S20>/Gain1'
     *  MinMax: '<S20>/MinMax4'
     *  Product: '<S20>/P2'
     *  RelationalOperator: '<S20>/R2'
     *  RelationalOperator: '<S20>/R3'
     *  Sum: '<S20>/A4'
     *  Switch: '<S20>/Swt6'
     *  UnitDelay: '<S20>/D2'
     */
    if (EnvDetn_vCllsnTar_C > LgtCtrl_DW.D2_DSTATE) {
      rtb_Swt2 = fminf(3.6F * ACCS_dtFrontTrgAccrSlpUp_C * EnvDetn_tiUpdate_C +
                       LgtCtrl_DW.D2_DSTATE, EnvDetn_vCllsnTar_C);
    } else if (EnvDetn_vCllsnTar_C < LgtCtrl_DW.D2_DSTATE) {
      /* Switch: '<S20>/Swt6' incorporates:
       *  Constant: '<S20>/ACCS_dtFrontTrgAccrSlpDwn_C'
       *  Constant: '<S20>/EnvDetn_tiUpdate_C'
       *  Gain: '<S20>/Gain2'
       *  MinMax: '<S20>/MinMax3'
       *  Product: '<S20>/P1'
       *  Sum: '<S20>/A3'
       */
      rtb_Swt2 = fmaxf(3.6F * ACCS_dtFrontTrgAccrSlpDwn_C * EnvDetn_tiUpdate_C +
                       LgtCtrl_DW.D2_DSTATE, EnvDetn_vCllsnTar_C);
    } else {
      /* Switch: '<S20>/Swt6' */
      rtb_Swt2 = LgtCtrl_DW.D2_DSTATE;
    }

    /* End of Switch: '<S20>/Swt5' */
  } else {
    rtb_Swt2 = EnvDetn_vCllsnTar_C;
  }

  /* End of Switch: '<S20>/Swt2' */

  /* Gain: '<S20>/Gain' incorporates:
   *  Sum: '<S20>/A1'
   */
  rtb_Gain = (VehDa_vEgoSpd - rtb_Swt2) * 0.277777791F;

  /* Switch: '<S20>/Swt1' incorporates:
   *  Constant: '<S20>/Constant'
   *  Constant: '<S20>/EnvDetn_idCllsnTar_C'
   *  Constant: '<S20>/EnvDetn_tiUpdate_C'
   *  Constant: '<S2>/ACCS_stACCActvdTest_C'
   *  Inport: '<Root>/EnvDetn_idCllsnTar'
   *  Inport: '<Root>/EnvDetn_lCllsnTarLgt'
   *  Inport: '<Root>/EnvDetn_vCllsnTarRel'
   *  Product: '<S20>/P3'
   *  Sum: '<S20>/A5'
   *  Switch: '<S20>/Swt3'
   *  Switch: '<S2>/Swt1'
   *  Switch: '<S2>/Swt2'
   *  Switch: '<S2>/Swt5'
   *  UnitDelay: '<S20>/D1'
   */
  if (ACCS_stACCActvdTest_C != 0) {
    rtb_Swt1 = rtb_Gain * EnvDetn_tiUpdate_C + LgtCtrl_DW.D1_DSTATE;

    /* Sum: '<S20>/A2' incorporates:
     *  Constant: '<S20>/EnvDetn_lCllsnTarLgt_C'
     *  Constant: '<S20>/EnvDetn_tiUpdate_C'
     *  Product: '<S20>/P3'
     *  Sum: '<S20>/A5'
     *  UnitDelay: '<S20>/D1'
     */
    rtb_Swt1_a = EnvDetn_lCllsnTarLgt_C - rtb_Swt1;

    /* Saturate: '<S20>/Saturation' */
    if (rtb_Swt1_a > 500.0F) {
      rtb_Swt1_a = 500.0F;
    } else if (rtb_Swt1_a < 0.0F) {
      rtb_Swt1_a = 0.0F;
    }

    /* End of Saturate: '<S20>/Saturation' */
    rtb_Swt2_n = EnvDetn_idCllsnTar_C;
  } else {
    rtb_Swt1 = 0.0F;
    rtb_Swt1_a = EnvDetn_lCllsnTarLgt;
    rtb_Gain = EnvDetn_vCllsnTarRel;
    rtb_Swt2_n = EnvDetn_idCllsnTar;
  }

  /* End of Switch: '<S20>/Swt1' */

  /* Sum: '<S16>/Add' incorporates:
   *  Constant: '<S16>/AEBS_lSftMin_C'
   */
  rtb_MinMax2_h = rtb_Swt1_a - AEBS_lSftMin_C;

  /* Logic: '<S16>/Logical Operator3' incorporates:
   *  Constant: '<S16>/NO_TARGET_ID'
   *  Constant: '<S16>/NO_TARGET_LGT_DST'
   *  RelationalOperator: '<S16>/Relational Operator1'
   *  RelationalOperator: '<S16>/Relational Operator5'
   */
  rtb_LogicalOperator3 = ((rtb_Swt2_n != 0.0) && (rtb_Swt1_a < 250.0));

  /* Outputs for Enabled SubSystem: '<S2>/ACCS_DesSpdCalcn' incorporates:
   *  EnablePort: '<S17>/Enable'
   */
  /* Product: '<S16>/Product' incorporates:
   *  Product: '<S21>/P8'
   *  Switch: '<S21>/S2'
   *  Switch: '<S22>/S'
   *  Switch: '<S22>/Swt10'
   */
  rtb_Swt1_o_idx_2 = rtb_Gain * rtb_Gain;

  /* End of Outputs for SubSystem: '<S2>/ACCS_DesSpdCalcn' */

  /* Switch: '<S16>/Switch' incorporates:
   *  Constant: '<S16>/AEBS_aDesFf_C'
   *  Constant: '<S16>/AEBS_swtFcnActvd_C'
   *  Constant: '<S16>/AEBS_vActvdLmtLo_C'
   *  Constant: '<S16>/C1'
   *  Constant: '<S16>/C2'
   *  Constant: '<S16>/C3'
   *  Constant: '<S16>/C7'
   *  Logic: '<S16>/Logical Operator'
   *  Logic: '<S16>/Logical Operator1'
   *  Logic: '<S16>/Logical Operator2'
   *  Logic: '<S16>/Logical Operator4'
   *  Product: '<S16>/Divide'
   *  Product: '<S16>/Product'
   *  RelationalOperator: '<S16>/Relational Operator'
   *  RelationalOperator: '<S16>/Relational Operator2'
   *  RelationalOperator: '<S16>/Relational Operator3'
   *  RelationalOperator: '<S16>/Relational Operator4'
   *  UnitDelay: '<S16>/Unit Delay'
   */
  if ((VehDa_vEgoSpd >= AEBS_vActvdLmtLo_C) && (1.0 != 0.0) &&
      rtb_LogicalOperator3 && (AEBS_swtFcnActvd_C != 0) && (rtb_Gain > 0.0) &&
      (rtb_Swt1_o_idx_2 / (-2.0F) / AEBS_aDesFf_C >= rtb_MinMax2_h)) {
    rtb_LogicalOperator3 = true;
  } else {
    rtb_LogicalOperator3 = ((rtb_Gain > 0.1) && rtb_LogicalOperator3 &&
      LgtCtrl_DW.UnitDelay_DSTATE_ih);
  }

  /* End of Switch: '<S16>/Switch' */

  /* Switch: '<S16>/Switch1' incorporates:
   *  Constant: '<S16>/AEBS_aDesFf_C'
   *  Constant: '<S16>/C5'
   *  Constant: '<S16>/C6'
   *  MinMax: '<S16>/MinMax2'
   *  Product: '<S16>/Product2'
   *  Sum: '<S16>/Add1'
   *  UnitDelay: '<S16>/Unit Delay1'
   */
  if (rtb_LogicalOperator3) {
    rtb_sampletime = fmaxf(AEBS_aDesFf_C * 0.036F +
      LgtCtrl_DW.UnitDelay1_DSTATE_fs, 0.0F);
  } else {
    rtb_sampletime = VehDa_vEgoSpd;
  }

  /* End of Switch: '<S16>/Switch1' */

  /* MinMax: '<S16>/MinMax1' incorporates:
   *  Abs: '<S16>/Abs'
   *  Constant: '<S16>/AEBS_aDesFf_C'
   *  Constant: '<S16>/C2'
   *  Constant: '<S16>/C4'
   *  Gain: '<S16>/Gain'
   *  MinMax: '<S16>/MinMax'
   *  Product: '<S16>/Product1'
   *  Sqrt: '<S16>/Sqrt'
   */
  rtb_sampletime = fminf(sqrtf(fabsf(fmaxf(rtb_MinMax2_h * (-2.0F) *
    AEBS_aDesFf_C, 0.0F))) * 3.6F, rtb_sampletime);

  /* Gain: '<S148>/Gain' incorporates:
   *  Inport: '<Root>/VehDa_rBrkPedl_mp'
   */
  VehDa_rBrkPedl = 1.0F * VehDa_rBrkPedl_mp;

  /* UnitDelay: '<S2>/Unit Delay' */
  UnitDelay = LgtCtrl_B.S;

  /* S-Function (sfix_bitop): '<S146>/Bitwise Operator' incorporates:
   *  Constant: '<S14>/LgtCtrl_RXFlag'
   */
  rtb_DataTypeConversion6_g = (uint8_T)(LgtCtrl_RXFlag & ((uint8_T)1U));

  /* DataTypeConversion: '<S2>/Data Type Conversion10' incorporates:
   *  Logic: '<S146>/Logical Operator'
   */
  DataTypeConversion10 = (uint8_T)(rtb_DataTypeConversion6_g == 0);

  /* Outputs for Enabled SubSystem: '<S2>/ACCS_SpTar' incorporates:
   *  EnablePort: '<S19>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S2>/ACCS_Mod' incorporates:
   *  EnablePort: '<S18>/Enable'
   */
  if (rtb_RelationalOperator1) {
    boolean_T LogicalOperator10;
    boolean_T LogicalOperator4;
    boolean_T LogicalOperator6;
    boolean_T LogicalOperator8;

    /* Logic: '<S32>/Logical Operator' incorporates:
     *  Constant: '<S2>/MFLv_stOffSwt_C'
     *  Logic: '<S32>/Logical Operator1'
     *  UnitDelay: '<S32>/Unit Delay'
     */
    rtb_L4_c = ((MFLv_stOffSwt_C != 0) && (LgtCtrl_DW.UnitDelay_DSTATE_d0 == 0));

    /* Logic: '<S32>/Logical Operator2' incorporates:
     *  Constant: '<S2>/MFLv_stResuSwt_C'
     *  Logic: '<S32>/Logical Operator3'
     *  UnitDelay: '<S32>/Unit Delay1'
     */
    LogicalOperator2 = ((MFLv_stResuSwt_C != 0) &&
                        (LgtCtrl_DW.UnitDelay1_DSTATE_b == 0));

    /* Logic: '<S32>/Logical Operator4' incorporates:
     *  Constant: '<S2>/MFLv_stSpdIncSwt_C'
     *  Logic: '<S32>/Logical Operator5'
     *  UnitDelay: '<S32>/Unit Delay2'
     */
    LogicalOperator4 = ((MFLv_stSpdIncSwt_C != 0) &&
                        (LgtCtrl_DW.UnitDelay2_DSTATE_l == 0));

    /* Logic: '<S32>/Logical Operator6' incorporates:
     *  Constant: '<S2>/MFLv_stSpdDecSwt_C'
     *  Logic: '<S32>/Logical Operator7'
     *  UnitDelay: '<S32>/Unit Delay3'
     */
    LogicalOperator6 = ((MFLv_stSpdDecSwt_C != 0) &&
                        (LgtCtrl_DW.UnitDelay3_DSTATE_b == 0));

    /* Logic: '<S32>/Logical Operator8' incorporates:
     *  Constant: '<S2>/MFLv_stDstIncSwt_C'
     *  Logic: '<S32>/Logical Operator9'
     *  UnitDelay: '<S32>/Unit Delay4'
     */
    LogicalOperator8 = ((MFLv_stDstIncSwt_C != 0) &&
                        (LgtCtrl_DW.UnitDelay4_DSTATE_bm == 0));

    /* Logic: '<S32>/Logical Operator10' incorporates:
     *  Constant: '<S2>/MFLv_stDstDecSwt_C'
     *  Logic: '<S32>/Logical Operator11'
     *  UnitDelay: '<S32>/Unit Delay5'
     */
    LogicalOperator10 = ((MFLv_stDstDecSwt_C != 0) &&
                         (LgtCtrl_DW.UnitDelay5_DSTATE_a == 0));

    /* Switch: '<S33>/Switch' incorporates:
     *  UnitDelay: '<S2>/Unit Delay1'
     */
    if (LgtCtrl_DW.UnitDelay1_DSTATE_gm != 0) {
      /* Switch: '<S33>/Switch' incorporates:
       *  Constant: '<S33>/ACCS_vGoMinWthTar_C'
       *  Constant: '<S33>/ACCS_vGoMin_C'
       *  MinMax: '<S33>/MinMax'
       */
      Switch = fmaxf(ACCS_vGoMin_C, ACCS_vGoMinWthTar_C);
    } else {
      /* Switch: '<S33>/Switch' incorporates:
       *  Constant: '<S33>/ACCS_vGoMin_C'
       */
      Switch = ACCS_vGoMin_C;
    }

    /* End of Switch: '<S33>/Switch' */

    /* Chart: '<S33>/StMng' incorporates:
     *  Constant: '<S2>/CANMng_rEEC1ActEngTrq'
     *  Constant: '<S2>/CoAEB_stAEB'
     *  Constant: '<S2>/DE_rVehAccrPedl'
     *  Constant: '<S2>/WarnMng_st'
     *  Constant: '<S33>/ACCS_rBrkPedlThd_C'
     *  Constant: '<S33>/ACCS_vMax_C'
     *  Constant: '<S33>/ACCS_vMin_C'
     *  Constant: '<S33>/ACCS_vStpThd_C'
     *  Inport: '<Root>/Sys_stADMd_mp '
     *  UnitDelay: '<S2>/Unit Delay'
     */
    /* Gateway: LgtCtrl/ACCS_Decision/ACCS_Mod/StMng/StMng */
    /* During: LgtCtrl/ACCS_Decision/ACCS_Mod/StMng/StMng */
    if (LgtCtrl_DW.is_active_c35_LgtCtrl == 0U) {
      /* Entry: LgtCtrl/ACCS_Decision/ACCS_Mod/StMng/StMng */
      LgtCtrl_DW.is_active_c35_LgtCtrl = 1U;

      /* Entry Internal: LgtCtrl/ACCS_Decision/ACCS_Mod/StMng/StMng */
      /* Transition: '<S34>:28' */
      LgtCtrl_DW.is_c35_LgtCtrl = LgtCtrl_IN_Off;

      /* Entry 'Off': '<S34>:1' */
      ACCS_stActvd = 0.0F;
      LgtCtrl_DW.ACCS_stFirstActvd = 0.0F;
      LgtCtrl_B.ACCS_stActvdPls = 0.0F;
    } else {
      switch (LgtCtrl_DW.is_c35_LgtCtrl) {
       case LgtCtrl_IN_Off:
        ACCS_stActvd = 0.0F;

        /* During 'Off': '<S34>:1' */
        if ((DataTypeConversion10 == 0) && rtb_L4_c && (0.0 == 0.0)) {
          /* Transition: '<S34>:5' */
          LgtCtrl_DW.is_c35_LgtCtrl = LgtCtrl_IN_Prepare;

          /* Entry 'Prepare': '<S34>:2' */
          ACCS_stActvd = 1.0F;
          LgtCtrl_B.ACCS_stActvdPls = 0.0F;
        } else if ((DataTypeConversion10 == 1) && (Sys_stADMd_mp == 3)) {
          /* Transition: '<S34>:92' */
          LgtCtrl_DW.is_c35_LgtCtrl = LgtCtrl_IN_Working;

          /* Entry 'Working': '<S34>:29' */
          LgtCtrl_DW.ACCS_stFirstActvd = 1.0F;
          LgtCtrl_B.ACCS_stActvdPls = 1.0F;
          LgtCtrl_enter_internal_Working(&UnitDelay, &Switch, &ACCS_stActvd);
        }
        break;

       case LgtCtrl_IN_Prepare:
        ACCS_stActvd = 1.0F;

        /* During 'Prepare': '<S34>:2' */
        if ((DataTypeConversion10 == 0) && rtb_L4_c) {
          /* Transition: '<S34>:6' */
          LgtCtrl_DW.is_c35_LgtCtrl = LgtCtrl_IN_Off;

          /* Entry 'Off': '<S34>:1' */
          ACCS_stActvd = 0.0F;
          LgtCtrl_DW.ACCS_stFirstActvd = 0.0F;
          LgtCtrl_B.ACCS_stActvdPls = 0.0F;
        } else if ((DataTypeConversion10 == 0) && (VehDa_vEgoSpd <= ACCS_vMax_C)
                   && (VehDa_vEgoSpd >= ACCS_vMin_C) && (LogicalOperator4 ||
                    LogicalOperator6 || (LogicalOperator2 &&
                     (LgtCtrl_DW.ACCS_stFirstActvd == 1.0F)))) {
          /* Transition: '<S34>:7' */
          LgtCtrl_DW.is_c35_LgtCtrl = LgtCtrl_IN_Working;

          /* Entry 'Working': '<S34>:29' */
          LgtCtrl_DW.ACCS_stFirstActvd = 1.0F;
          LgtCtrl_B.ACCS_stActvdPls = 1.0F;
          LgtCtrl_enter_internal_Working(&UnitDelay, &Switch, &ACCS_stActvd);
        }
        break;

       default:
        /* During 'Working': '<S34>:29' */
        if ((DataTypeConversion10 == 0) && (LogicalOperator2 || (VehDa_rBrkPedl >
              ACCS_rBrkPedlThd_C))) {
          /* Transition: '<S34>:8' */
          /* Exit Internal 'Working': '<S34>:29' */
          /* Exit Internal 'Active': '<S34>:3' */
          LgtCtrl_DW.is_Active = 0;
          LgtCtrl_DW.is_Working = 0;
          LgtCtrl_DW.is_c35_LgtCtrl = LgtCtrl_IN_Prepare;

          /* Entry 'Prepare': '<S34>:2' */
          ACCS_stActvd = 1.0F;
          LgtCtrl_B.ACCS_stActvdPls = 0.0F;
        } else if (((DataTypeConversion10 == 0) && (rtb_L4_c || ((0.0 == 5.0) &&
          (0.0 == 4.0)))) || ((DataTypeConversion10 == 1) && ((Sys_stADMd_mp !=
                      3) || (VehDa_rBrkPedl > ACCS_rBrkPedlThd_C)))) {
          /* Transition: '<S34>:13' */
          /* Transition: '<S34>:94' */
          /* Exit Internal 'Working': '<S34>:29' */
          /* Exit Internal 'Active': '<S34>:3' */
          LgtCtrl_DW.is_Active = 0;
          LgtCtrl_DW.is_Working = 0;
          LgtCtrl_DW.is_c35_LgtCtrl = LgtCtrl_IN_Off;

          /* Entry 'Off': '<S34>:1' */
          ACCS_stActvd = 0.0F;
          LgtCtrl_DW.ACCS_stFirstActvd = 0.0F;
          LgtCtrl_B.ACCS_stActvdPls = 0.0F;
        } else {
          LgtCtrl_B.ACCS_stActvdPls = 0.0F;
          if (LgtCtrl_DW.is_Working == 1) {
            /* During 'Active': '<S34>:3' */
            if ((0.0 >= 0.0 + 1.0) && (0.0 > 5.0)) {
              /* Transition: '<S34>:9' */
              /* Exit Internal 'Active': '<S34>:3' */
              LgtCtrl_DW.is_Active = 0;
              LgtCtrl_DW.is_Working = LgtCtrl_IN_OverTake;

              /* Entry 'OverTake': '<S34>:4' */
              ACCS_stActvd = 3.0F;
            } else {
              switch (LgtCtrl_DW.is_Active) {
               case LgtCtrl_IN_Cruise:
                ACCS_stActvd = 2.0F;

                /* During 'Cruise': '<S34>:80' */
                if ((VehDa_vEgoSpd < ACCS_vStpThd_C) && (LgtCtrl_B.S <= Switch))
                {
                  /* Transition: '<S34>:87' */
                  LgtCtrl_DW.is_Active = LgtCtrl_IN_Parking;

                  /* Entry 'Parking': '<S34>:84' */
                  ACCS_stActvd = 11.0F;
                }
                break;

               case LgtCtrl_IN_Parking:
                ACCS_stActvd = 11.0F;

                /* During 'Parking': '<S34>:84' */
                if ((VehDa_vEgoSpd < Switch) && (LgtCtrl_B.S > Switch)) {
                  /* Transition: '<S34>:86' */
                  LgtCtrl_DW.is_Active = LgtCtrl_IN_Starting;

                  /* Entry 'Starting': '<S34>:81' */
                  ACCS_stActvd = 5.0F;
                } else if ((VehDa_vEgoSpd >= Switch) && (LgtCtrl_B.S > Switch))
                {
                  /* Transition: '<S34>:88' */
                  LgtCtrl_DW.is_Active = LgtCtrl_IN_Cruise;

                  /* Entry 'Cruise': '<S34>:80' */
                  ACCS_stActvd = 2.0F;
                }
                break;

               default:
                ACCS_stActvd = 5.0F;

                /* During 'Starting': '<S34>:81' */
                if ((VehDa_vEgoSpd >= Switch) && (LgtCtrl_B.S > Switch)) {
                  /* Transition: '<S34>:77' */
                  LgtCtrl_DW.is_Active = LgtCtrl_IN_Cruise;

                  /* Entry 'Cruise': '<S34>:80' */
                  ACCS_stActvd = 2.0F;
                }
                break;
              }
            }
          } else {
            ACCS_stActvd = 3.0F;

            /* During 'OverTake': '<S34>:4' */
            if (0.0 == 0.0) {
              /* Transition: '<S34>:10' */
              LgtCtrl_DW.is_Working = LgtCtrl_IN_Active;

              /* Entry 'Active': '<S34>:3' */
              /* Entry Internal 'Active': '<S34>:3' */
              /* Transition: '<S34>:75' */
              if ((VehDa_vEgoSpd < ACCS_vStpThd_C) && (LgtCtrl_B.S <= Switch)) {
                /* Transition: '<S34>:85' */
                LgtCtrl_DW.is_Active = LgtCtrl_IN_Parking;

                /* Entry 'Parking': '<S34>:84' */
                ACCS_stActvd = 11.0F;
              } else if ((VehDa_vEgoSpd < Switch) && (LgtCtrl_B.S > Switch)) {
                /* Transition: '<S34>:74' */
                LgtCtrl_DW.is_Active = LgtCtrl_IN_Starting;

                /* Entry 'Starting': '<S34>:81' */
                ACCS_stActvd = 5.0F;
              } else {
                /* Transition: '<S34>:73' */
                LgtCtrl_DW.is_Active = LgtCtrl_IN_Cruise;

                /* Entry 'Cruise': '<S34>:80' */
                ACCS_stActvd = 2.0F;
              }
            }
          }
        }
        break;
      }
    }

    /* End of Chart: '<S33>/StMng' */

    /* MultiPortSwitch: '<S33>/Multiport Switch' */
    switch ((int32_T)ACCS_stActvd) {
     case 0:
      /* MultiPortSwitch: '<S33>/Multiport Switch' incorporates:
       *  Constant: '<S33>/C'
       */
      LgtCtrl_B.MultiportSwitch = 15.0F;
      break;

     case 1:
      /* MultiPortSwitch: '<S33>/Multiport Switch' incorporates:
       *  Constant: '<S33>/C'
       */
      LgtCtrl_B.MultiportSwitch = 15.0F;
      break;

     case 2:
      /* MultiPortSwitch: '<S33>/Multiport Switch' incorporates:
       *  Constant: '<S33>/C1'
       */
      LgtCtrl_B.MultiportSwitch = 2.0F;
      break;

     case 3:
      /* MultiPortSwitch: '<S33>/Multiport Switch' incorporates:
       *  Constant: '<S33>/C'
       */
      LgtCtrl_B.MultiportSwitch = 15.0F;
      break;

     case 5:
      /* MultiPortSwitch: '<S33>/Multiport Switch' incorporates:
       *  Constant: '<S33>/C2'
       */
      LgtCtrl_B.MultiportSwitch = 2.0F;
      break;

     case 11:
      /* MultiPortSwitch: '<S33>/Multiport Switch' incorporates:
       *  Constant: '<S33>/C3'
       */
      LgtCtrl_B.MultiportSwitch = 11.0F;
      break;

     default:
      /* MultiPortSwitch: '<S33>/Multiport Switch' incorporates:
       *  Constant: '<S33>/C'
       */
      LgtCtrl_B.MultiportSwitch = 15.0F;
      break;
    }

    /* End of MultiPortSwitch: '<S33>/Multiport Switch' */

    /* MultiPortSwitch: '<S33>/Multiport Switch1' */
    switch ((int32_T)ACCS_stActvd) {
     case 2:
      /* MultiPortSwitch: '<S33>/Multiport Switch1' incorporates:
       *  Constant: '<S33>/c'
       */
      LgtCtrl_B.MultiportSwitch1 = 3.0F;
      break;

     case 5:
      /* MultiPortSwitch: '<S33>/Multiport Switch1' incorporates:
       *  Constant: '<S33>/c'
       */
      LgtCtrl_B.MultiportSwitch1 = 3.0F;
      break;

     case 11:
      /* MultiPortSwitch: '<S33>/Multiport Switch1' incorporates:
       *  Constant: '<S33>/c'
       */
      LgtCtrl_B.MultiportSwitch1 = 3.0F;
      break;

     default:
      /* MultiPortSwitch: '<S33>/Multiport Switch1' incorporates:
       *  Constant: '<S33>/c1'
       */
      LgtCtrl_B.MultiportSwitch1 = 0.0F;
      break;
    }

    /* End of MultiPortSwitch: '<S33>/Multiport Switch1' */

    /* Update for UnitDelay: '<S32>/Unit Delay' incorporates:
     *  Constant: '<S2>/MFLv_stOffSwt_C'
     */
    LgtCtrl_DW.UnitDelay_DSTATE_d0 = MFLv_stOffSwt_C;

    /* Update for UnitDelay: '<S32>/Unit Delay1' incorporates:
     *  Constant: '<S2>/MFLv_stResuSwt_C'
     */
    LgtCtrl_DW.UnitDelay1_DSTATE_b = MFLv_stResuSwt_C;

    /* Update for UnitDelay: '<S32>/Unit Delay2' incorporates:
     *  Constant: '<S2>/MFLv_stSpdIncSwt_C'
     */
    LgtCtrl_DW.UnitDelay2_DSTATE_l = MFLv_stSpdIncSwt_C;

    /* Update for UnitDelay: '<S32>/Unit Delay3' incorporates:
     *  Constant: '<S2>/MFLv_stSpdDecSwt_C'
     */
    LgtCtrl_DW.UnitDelay3_DSTATE_b = MFLv_stSpdDecSwt_C;

    /* Update for UnitDelay: '<S32>/Unit Delay4' incorporates:
     *  Constant: '<S2>/MFLv_stDstIncSwt_C'
     */
    LgtCtrl_DW.UnitDelay4_DSTATE_bm = MFLv_stDstIncSwt_C;

    /* Update for UnitDelay: '<S32>/Unit Delay5' incorporates:
     *  Constant: '<S2>/MFLv_stDstDecSwt_C'
     */
    LgtCtrl_DW.UnitDelay5_DSTATE_a = MFLv_stDstDecSwt_C;

    /* Chart: '<S36>/Chart' incorporates:
     *  Constant: '<S36>/ACCS_vMax_C'
     *  Constant: '<S36>/ACCS_vMin_C'
     *  Constant: '<S36>/ACCS_vTipDwn_C'
     *  Constant: '<S36>/ACCS_vTipUp_C'
     */
    /* Gateway: LgtCtrl/ACCS_Decision/ACCS_SpTar/SpTar/Chart */
    /* During: LgtCtrl/ACCS_Decision/ACCS_SpTar/SpTar/Chart */
    /* Entry Internal: LgtCtrl/ACCS_Decision/ACCS_SpTar/SpTar/Chart */
    /* Transition: '<S37>:105' */
    if (ACCS_stActvd != 0.0F) {
      /* Transition: '<S37>:5' */
      if ((ACCS_stActvd == 2.0F) || (ACCS_stActvd == 3.0F) || (ACCS_stActvd ==
           5.0F) || (ACCS_stActvd == 11.0F)) {
        /* Transition: '<S37>:80' */
        if (LgtCtrl_B.ACCS_stActvdPls == 0.0F) {
          /* Transition: '<S37>:7' */
          if (LogicalOperator4) {
            /* Transition: '<S37>:9' */
            /* Transition: '<S37>:11' */
            if (LgtCtrl_B.ACCS_vSet < 2.14748365E+9F) {
              if (LgtCtrl_B.ACCS_vSet >= -2.14748365E+9F) {
                i = (int32_T)LgtCtrl_B.ACCS_vSet;
              } else {
                i = MIN_int32_T;
              }
            } else {
              i = MAX_int32_T;
            }

            if (ACCS_vTipUp_C < 2.14748365E+9F) {
              if (ACCS_vTipUp_C >= -2.14748365E+9F) {
                judge = (int32_T)ACCS_vTipUp_C;
              } else {
                judge = MIN_int32_T;
              }
            } else {
              judge = MAX_int32_T;
            }

            LgtCtrl_B.ACCS_vSet = (LgtCtrl_B.ACCS_vSet + ACCS_vTipUp_C) -
              (real32_T)(i % judge);

            /* Transition: '<S37>:13' */
          } else {
            /* Transition: '<S37>:14' */
          }

          /* Transition: '<S37>:27' */
          if (LogicalOperator6) {
            /* Transition: '<S37>:29' */
            if (LgtCtrl_B.ACCS_vSet < 2.14748365E+9F) {
              if (LgtCtrl_B.ACCS_vSet >= -2.14748365E+9F) {
                i = (int32_T)LgtCtrl_B.ACCS_vSet;
              } else {
                i = MIN_int32_T;
              }
            } else {
              i = MAX_int32_T;
            }

            if (ACCS_vTipDwn_C < 2.14748365E+9F) {
              if (ACCS_vTipDwn_C >= -2.14748365E+9F) {
                judge = (int32_T)ACCS_vTipDwn_C;
              } else {
                judge = MIN_int32_T;
              }
            } else {
              judge = MAX_int32_T;
            }

            if (i % judge != 0) {
              /* Transition: '<S37>:128' */
              /* Transition: '<S37>:130' */
              if (LgtCtrl_B.ACCS_vSet < 2.14748365E+9F) {
                if (LgtCtrl_B.ACCS_vSet >= -2.14748365E+9F) {
                  i = (int32_T)LgtCtrl_B.ACCS_vSet;
                } else {
                  i = MIN_int32_T;
                }
              } else {
                i = MAX_int32_T;
              }

              if (ACCS_vTipDwn_C < 2.14748365E+9F) {
                if (ACCS_vTipDwn_C >= -2.14748365E+9F) {
                  judge = (int32_T)ACCS_vTipDwn_C;
                } else {
                  judge = MIN_int32_T;
                }
              } else {
                judge = MAX_int32_T;
              }

              LgtCtrl_B.ACCS_vSet -= (real32_T)(i % judge);

              /* Transition: '<S37>:131' */
            } else {
              /* Transition: '<S37>:31' */
              LgtCtrl_B.ACCS_vSet -= ACCS_vTipDwn_C;
            }

            /* Transition: '<S37>:33' */
          } else {
            /* Transition: '<S37>:34' */
          }

          /* Transition: '<S37>:16' */
          if (LgtCtrl_B.ACCS_vSet > ACCS_vMax_C) {
            /* Transition: '<S37>:18' */
            /* Transition: '<S37>:22' */
            LgtCtrl_B.ACCS_vSet = ACCS_vMax_C;

            /* Transition: '<S37>:23' */
          } else {
            /* Transition: '<S37>:21' */
          }

          /* Transition: '<S37>:37' */
          if (LgtCtrl_B.ACCS_vSet < ACCS_vMin_C) {
            /* Transition: '<S37>:39' */
            /* Transition: '<S37>:41' */
            LgtCtrl_B.ACCS_vSet = ACCS_vMin_C;

            /* Transition: '<S37>:43' */
          } else {
            /* Transition: '<S37>:44' */
          }

          /* Transition: '<S37>:46' */
          if (LogicalOperator10) {
            /* Transition: '<S37>:48' */
            if (LgtCtrl_B.ACCS_stTimeGap != 5.0F) {
              /* Transition: '<S37>:69' */
              /* Transition: '<S37>:72' */
              LgtCtrl_B.ACCS_stTimeGap++;

              /* Transition: '<S37>:71' */
            } else {
              /* Transition: '<S37>:50' */
            }

            /* Transition: '<S37>:52' */
          } else {
            /* Transition: '<S37>:53' */
          }

          /* Transition: '<S37>:56' */
          if (LogicalOperator8) {
            /* Transition: '<S37>:57' */
            if (LgtCtrl_B.ACCS_stTimeGap != 0.0F) {
              /* Transition: '<S37>:64' */
              /* Transition: '<S37>:66' */
              LgtCtrl_B.ACCS_stTimeGap--;

              /* Transition: '<S37>:67' */
            } else {
              /* Transition: '<S37>:59' */
            }

            /* Transition: '<S37>:61' */
          } else {
            /* Transition: '<S37>:62' */
          }

          /* Transition: '<S37>:126' */

          /* Transition: '<S37>:93' */
        } else if (LogicalOperator4 || LogicalOperator6) {
          /* Transition: '<S37>:95' */
          /* Transition: '<S37>:98' */
          LgtCtrl_B.ACCS_vSet = VehDa_vEgoSpd;

          /* Transition: '<S37>:99' */
        } else {
          /* Transition: '<S37>:100' */

          /* Transition: '<S37>:110' */
        }

        /* Transition: '<S37>:111' */
      } else {
        /* Transition: '<S37>:107' */
      }

      /* Transition: '<S37>:112' */
    } else {
      /* Transition: '<S37>:78' */
      LgtCtrl_B.ACCS_vSet = 0.0F;
      LgtCtrl_B.ACCS_stTimeGap = 0.0F;
    }

    /* End of Chart: '<S36>/Chart' */

    /* Switch: '<S35>/Switch2' incorporates:
     *  Constant: '<S35>/ACCS_stVSpCalReqd_C'
     *  Switch: '<S35>/Switch'
     */
    if (DataTypeConversion10 != 0) {
      /* Switch: '<S35>/Switch2' incorporates:
       *  Inport: '<Root>/ACCS_vSetFrmSys_mp'
       */
      LgtCtrl_B.Switch2 = ACCS_vSetFrmSys_mp;
    } else if (ACCS_stVSpCalReqd_C != 0) {
      /* Switch: '<S35>/Switch' incorporates:
       *  Constant: '<S35>/ACCS_vMax_C'
       *  Constant: '<S35>/ACCS_vMin_C'
       *  Constant: '<S35>/ACCS_vVSpCalReqdVal_C'
       *  MinMax: '<S35>/MinMax'
       *  MinMax: '<S35>/MinMax1'
       *  Switch: '<S35>/Switch2'
       */
      LgtCtrl_B.Switch2 = fminf(fmaxf(ACCS_vVSpCalReqdVal_C, ACCS_vMin_C),
        ACCS_vMax_C);
    } else {
      /* Switch: '<S35>/Switch2' incorporates:
       *  Switch: '<S35>/Switch'
       */
      LgtCtrl_B.Switch2 = LgtCtrl_B.ACCS_vSet;
    }

    /* End of Switch: '<S35>/Switch2' */

    /* Switch: '<S35>/Switch1' incorporates:
     *  Constant: '<S35>/ACCS_stDstSpCalReqd_C'
     */
    if (ACCS_stDstSpCalReqd_C != 0) {
      /* Switch: '<S35>/Switch1' incorporates:
       *  Constant: '<S35>/ACCS_tiDstSpCalReqdVal_C'
       *  Constant: '<S35>/ACCS_tiTimeGapModFive_C'
       *  Constant: '<S35>/ACCS_tiTimeGapModOne_C'
       *  MinMax: '<S35>/MinMax2'
       *  MinMax: '<S35>/MinMax3'
       */
      LgtCtrl_B.Switch1 = fminf(fmaxf(ACCS_tiDstSpCalReqdVal_C,
        ACCS_tiTimeGapModFive_C), ACCS_tiTimeGapModOne_C);
    } else {
      /* MultiPortSwitch: '<S36>/Multiport Switch' */
      switch ((int32_T)LgtCtrl_B.ACCS_stTimeGap) {
       case 0:
        /* Switch: '<S35>/Switch1' incorporates:
         *  Constant: '<S36>/ACCS_tiTimeGapModOne_C'
         */
        LgtCtrl_B.Switch1 = 3.5F;
        break;

       case 1:
        /* Switch: '<S35>/Switch1' incorporates:
         *  Constant: '<S36>/ACCS_tiTimeGapModTwo_C'
         */
        LgtCtrl_B.Switch1 = 3.0F;
        break;

       case 2:
        /* Switch: '<S35>/Switch1' incorporates:
         *  Constant: '<S36>/ACCS_tiTimeGapModThree_C'
         */
        LgtCtrl_B.Switch1 = 2.5F;
        break;

       case 3:
        /* Switch: '<S35>/Switch1' incorporates:
         *  Constant: '<S36>/ACCS_tiTimeGapModFour_C'
         */
        LgtCtrl_B.Switch1 = 2.0F;
        break;

       default:
        /* Switch: '<S35>/Switch1' incorporates:
         *  Constant: '<S36>/ACCS_tiTimeGapModFive_C'
         */
        LgtCtrl_B.Switch1 = 1.5F;
        break;
      }

      /* End of MultiPortSwitch: '<S36>/Multiport Switch' */
    }

    /* End of Switch: '<S35>/Switch1' */
  }

  /* End of Outputs for SubSystem: '<S2>/ACCS_Mod' */
  /* End of Outputs for SubSystem: '<S2>/ACCS_SpTar' */

  /* DataTypeConversion: '<S14>/Data Type Conversion21' incorporates:
   *  Gain: '<S152>/Gain'
   *  Inport: '<Root>/VehDa_stTraCurGear_mp'
   */
  VehDa_stTraCurGear = (int16_T)((16384 * VehDa_stTraCurGear_mp) >> 14);

  /* DataTypeConversion: '<S14>/Data Type Conversion14' incorporates:
   *  Gain: '<S156>/Gain'
   *  Inport: '<Root>/VehDa_prcActuTrq_mp'
   */
  VehDa_prcActuTrq = (int16_T)((16384 * VehDa_prcActuTrq_mp) >> 14);

  /* DataTypeConversion: '<S14>/Data Type Conversion16' incorporates:
   *  Gain: '<S153>/Gain'
   *  Inport: '<Root>/VehDa_prcDrvrDmdTrq_mp'
   */
  VehDa_prcDrvrDmdTrq = (int16_T)((16384 * VehDa_prcDrvrDmdTrq_mp) >> 14);

  /* Switch: '<S167>/Switch' incorporates:
   *  Constant: '<S14>/ACC_RXFlag'
   */
  if (ACC_RXFlag != 0) {
    /* Switch: '<S167>/Switch' incorporates:
     *  UnitDelay: '<S1>/Unit Delay3'
     */
    SpdPln_aTrgAcc = LgtCtrl_B.Swt12;
  } else {
    /* Switch: '<S167>/Switch' incorporates:
     *  Inport: '<Root>/SpdPln_aTrgAcc_mp '
     */
    SpdPln_aTrgAcc = SpdPln_aTrgAcc_mp;
  }

  /* End of Switch: '<S167>/Switch' */

  /* Chart: '<S10>/ShiftJudg' incorporates:
   *  Constant: '<S10>/Tra_aShtWthSpdUpThd_C'
   *  Constant: '<S10>/Tra_facCluTrqEgd_C'
   *  Constant: '<S10>/Tra_facCluTrqRls_C'
   *  DataTypeConversion: '<S10>/Data Type Conversion2'
   *  DataTypeConversion: '<S10>/Data Type Conversion3'
   *  DataTypeConversion: '<S10>/Data Type Conversion4'
   *  Inport: '<Root>/VehDa_stTraEgd_mp'
   *  Inport: '<Root>/VehDa_stTraSelGear_mp'
   *  Inport: '<Root>/VehDa_stTraSht_mp'
   *  UnitDelay: '<S1>/Unit Delay9'
   */
  /* Gateway: LgtCtrl/TraMonr/ShiftJudg */
  /* During: LgtCtrl/TraMonr/ShiftJudg */
  if (LgtCtrl_DW.is_active_c1_LgtCtrl == 0U) {
    /* Entry: LgtCtrl/TraMonr/ShiftJudg */
    LgtCtrl_DW.is_active_c1_LgtCtrl = 1U;

    /* Entry Internal: LgtCtrl/TraMonr/ShiftJudg */
    /* Transition: '<S98>:2' */
    LgtCtrl_DW.is_c1_LgtCtrl = LgtCtrl_IN_NoShift;

    /* DataTypeConversion: '<S10>/Data Type Conversion7' */
    /* Entry 'NoShift': '<S98>:1' */
    Tra_stShiftWthSpdUp = 0U;
  } else if (LgtCtrl_DW.is_c1_LgtCtrl == 1) {
    /* DataTypeConversion: '<S10>/Data Type Conversion7' */
    Tra_stShiftWthSpdUp = 0U;

    /* During 'NoShift': '<S98>:1' */
    if ((VehDa_stTraCurGear != 0) && (VehDa_stTraSelGear_mp !=
         VehDa_stTraCurGear) && (VehDa_prcActuTrq <= Tra_facCluTrqRls_C *
         (real32_T)VehDa_prcDrvrDmdTrq) && (VehDa_stTraSht_mp == 1) &&
        (CoChs_stCdn == 2) && (SpdPln_aTrgAcc > Tra_aShtWthSpdUpThd_C)) {
      /* Transition: '<S98>:4' */
      LgtCtrl_DW.is_c1_LgtCtrl = LgtCtrl_IN_Shift;

      /* DataTypeConversion: '<S10>/Data Type Conversion7' */
      /* Entry 'Shift': '<S98>:3' */
      Tra_stShiftWthSpdUp = 1U;
    }
  } else {
    /* DataTypeConversion: '<S10>/Data Type Conversion7' */
    Tra_stShiftWthSpdUp = 1U;

    /* During 'Shift': '<S98>:3' */
    if (((VehDa_stTraSelGear_mp == VehDa_stTraCurGear) && (VehDa_prcActuTrq >=
          Tra_facCluTrqEgd_C * (real32_T)VehDa_prcDrvrDmdTrq) &&
         (VehDa_stTraEgd_mp == 1)) || (CoChs_stCdn != 2)) {
      /* Transition: '<S98>:5' */
      LgtCtrl_DW.is_c1_LgtCtrl = LgtCtrl_IN_NoShift;

      /* DataTypeConversion: '<S10>/Data Type Conversion7' */
      /* Entry 'NoShift': '<S98>:1' */
      Tra_stShiftWthSpdUp = 0U;
    }
  }

  /* End of Chart: '<S10>/ShiftJudg' */

  /* Outputs for Enabled SubSystem: '<S2>/ACCS_DesSpdCalcn' incorporates:
   *  EnablePort: '<S17>/Enable'
   */
  if (rtb_RelationalOperator1) {
    /* Gain: '<S21>/Gain' */
    Switch = 3.6F * rtb_Gain;

    /* Sum: '<S21>/Add3' */
    ACCS_stActvd = VehDa_vEgoSpd - Switch;

    /* Sum: '<S21>/A' incorporates:
     *  Constant: '<S21>/ACCS_lSetMin_C'
     *  Constant: '<S21>/C'
     *  MinMax: '<S21>/MinMax1'
     *  Product: '<S21>/Divide'
     *  Product: '<S21>/P'
     */
    rtb_A_iw = rtb_Swt1_a - fmaxf(ACCS_stActvd / 3.6F * LgtCtrl_B.Switch1,
      ACCS_lSetMin_C);

    /* Sum: '<S21>/Add4' incorporates:
     *  Constant: '<S21>/C12'
     *  Constant: '<S21>/C5'
     */
    UnitDelay = (rtb_A_iw + 0.0F) - 2.0F;

    /* Chart: '<S21>/Dst2Spd' incorporates:
     *  Constant: '<S21>/ACCS_facDst2SpdOne_C'
     *  Constant: '<S21>/ACCS_facDst2SpdThree_C'
     *  Constant: '<S21>/ACCS_facDst2SpdTwo_C'
     *  Constant: '<S21>/ACCS_lDstCtlPntOne_C'
     *  Constant: '<S21>/ACCS_lDstCtlPntTwo_C'
     */
    /* Gateway: LgtCtrl/ACCS_Decision/ACCS_DesSpdCalcn/SetPointCaln/Dst2Spd */
    /* During: LgtCtrl/ACCS_Decision/ACCS_DesSpdCalcn/SetPointCaln/Dst2Spd */
    /* Entry Internal: LgtCtrl/ACCS_Decision/ACCS_DesSpdCalcn/SetPointCaln/Dst2Spd */
    /* Transition: '<S23>:35' */
    if (rtb_A_iw < 0.0F) {
      /* Transition: '<S23>:36' */
      /* Transition: '<S23>:38' */
      LgtDstRel = -rtb_A_iw;

      /* Transition: '<S23>:39' */
    } else {
      /* Transition: '<S23>:2' */
      LgtDstRel = rtb_A_iw;
    }

    if (LgtDstRel <= 5.0F) {
      /* Transition: '<S23>:4' */
      LgtCtrl_B.TrgtSpd = 0.45F * LgtDstRel;
      LgtCtrl_B.TrgtAccr = (real32_T)(0.45F * 0.45F * LgtDstRel / 3.6 / 3.6);

      /* Transition: '<S23>:8' */
      /* Transition: '<S23>:21' */
      /* Transition: '<S23>:56' */

      /* Transition: '<S23>:7' */
    } else if (LgtDstRel <= 50.0F) {
      /* Transition: '<S23>:13' */
      rtb_Swt1_o_idx_4 = 0.45F * 5.0F;
      LgtDstRel -= 5.0F;
      LgtCtrl_B.TrgtSpd = LgtDstRel * 0.45F + rtb_Swt1_o_idx_4;
      LgtCtrl_B.TrgtAccr = (real32_T)((0.45F * 0.45F * LgtDstRel +
        rtb_Swt1_o_idx_4 * 0.45F) / 3.6 / 3.6);

      /* Transition: '<S23>:21' */
      /* Transition: '<S23>:56' */

      /* Transition: '<S23>:55' */
    } else if (LgtDstRel > 50.0F) {
      /* Transition: '<S23>:26' */
      rtb_Swt1_o_idx_4 = (50.0F - 5.0F) * 0.45F + 0.45F * 5.0F;
      LgtDstRel -= 50.0F;
      LgtCtrl_B.TrgtSpd = LgtDstRel * 0.45F + rtb_Swt1_o_idx_4;
      LgtCtrl_B.TrgtAccr = (real32_T)((0.45F * 0.45F * LgtDstRel +
        rtb_Swt1_o_idx_4 * 0.45F) / 3.6 / 3.6);

      /* Transition: '<S23>:56' */
    } else {
      /* Transition: '<S23>:31' */
    }

    if (rtb_A_iw < 0.0F) {
      /* Transition: '<S23>:44' */
      /* Transition: '<S23>:46' */
      LgtCtrl_B.TrgtSpd = -LgtCtrl_B.TrgtSpd;
    } else {
      /* Transition: '<S23>:45' */
      LgtCtrl_B.TrgtAccr = -LgtCtrl_B.TrgtAccr;
    }

    /* End of Chart: '<S21>/Dst2Spd' */

    /* Switch: '<S21>/Switch' incorporates:
     *  Constant: '<S21>/ACCS_stACCActvd_C'
     */
    if (ACCS_stACCActvd_C != 0) {
      /* Switch: '<S21>/Switch2' incorporates:
       *  Constant: '<S2>/ACCS_stACCActvdTest_C'
       */
      if (ACCS_stACCActvdTest_C != 0) {
        /* Switch: '<S21>/Switch' incorporates:
         *  Constant: '<S21>/C1'
         *  RelationalOperator: '<S21>/L'
         */
        LgtCtrl_B.Switch_o = (real32_T)(rtb_Swt2_n != 0.0);
      } else {
        /* Switch: '<S21>/Switch' incorporates:
         *  Constant: '<S21>/C3'
         *  RelationalOperator: '<S21>/L1'
         */
        LgtCtrl_B.Switch_o = (real32_T)(rtb_Swt1_a < 250.0);
      }

      /* End of Switch: '<S21>/Switch2' */
    } else {
      /* Switch: '<S21>/Switch' incorporates:
       *  Constant: '<S21>/C2'
       */
      LgtCtrl_B.Switch_o = 0.0F;
    }

    /* End of Switch: '<S21>/Switch' */

    /* Switch: '<S21>/S' */
    if (LgtCtrl_B.Switch_o != 0.0F) {
      /* Switch: '<S21>/Switch1' incorporates:
       *  Constant: '<S21>/ACCS_facDstToSpd_C'
       *  Constant: '<S21>/ACCS_swtDstToSpd_C'
       *  Constant: '<S21>/ACCS_vDstCtlPnt_C'
       *  Constant: '<S21>/Constant6'
       *  Gain: '<S21>/Gain1'
       *  Math: '<S21>/Math Function'
       *  MinMax: '<S21>/MinMax'
       *  Product: '<S21>/Divide1'
       *  Product: '<S21>/P3'
       *  Sum: '<S21>/A1'
       *
       * About '<S21>/Math Function':
       *  Operator: exp
       */
      if (ACCS_swtDstToSpd_C != 0) {
        rtb_Swt1_a = expf(fmaxf(ACCS_vDstCtlPnt_C - VehDa_vEgoSpd, 0.0F) /
                          ACCS_vDstCtlPnt_C * ACCS_facDstCtlPnt_C) *
          ACCS_facDstToSpd_C * rtb_A_iw;
      } else {
        /* Product: '<S21>/Divide2' incorporates:
         *  Constant: '<S21>/C4'
         */
        LgtDstRel = VehDa_vEgoSpd / 3.6F;

        /* MinMax: '<S21>/MinMax3' incorporates:
         *  Constant: '<S21>/ACCS_tiFollowHysPos_C'
         *  Constant: '<S21>/C10'
         *  Product: '<S21>/P4'
         */
        rtb_Swt1_a = fmaxf(LgtDstRel * ACCS_tiFollowHysPos_C, 1.0F);

        /* MinMax: '<S21>/MinMax4' incorporates:
         *  Constant: '<S21>/ACCS_tiFollowHysNeg_C'
         *  Constant: '<S21>/C11'
         *  Product: '<S21>/P5'
         */
        LgtDstRel = fminf(LgtDstRel * ACCS_tiFollowHysNeg_C, -1.0F);

        /* Switch: '<S24>/Swt' incorporates:
         *  Constant: '<S21>/ACCS_facDstToSpdCurr_C'
         *  Logic: '<S24>/L'
         *  Logic: '<S24>/L1'
         *  Product: '<S24>/Product5'
         *  RelationalOperator: '<S24>/R'
         *  RelationalOperator: '<S24>/R1'
         *  RelationalOperator: '<S24>/R2'
         *  RelationalOperator: '<S24>/R3'
         *  Switch: '<S24>/Swt1'
         */
        if ((rtb_Swt1_a < LgtDstRel) || ((rtb_A_iw <= rtb_Swt1_a) && (rtb_A_iw >=
              LgtDstRel))) {
          rtb_Swt1_a = rtb_A_iw * ACCS_facDstToSpdCurr_C;
        } else if (rtb_A_iw > rtb_Swt1_a) {
          /* Switch: '<S24>/Swt1' incorporates:
           *  Constant: '<S21>/ACCS_facDstToSpdCurr_C'
           *  Constant: '<S21>/ACCS_facDstToSpdPos_C'
           *  Product: '<S24>/Product6'
           *  Product: '<S24>/Product8'
           *  Sum: '<S24>/Sum4'
           *  Sum: '<S24>/Sum5'
           */
          rtb_Swt1_a = (rtb_A_iw - rtb_Swt1_a) * ACCS_facDstToSpdPos_C +
            rtb_Swt1_a * ACCS_facDstToSpdCurr_C;
        } else {
          /* Switch: '<S24>/Swt1' incorporates:
           *  Constant: '<S21>/ACCS_facDstToSpdCurr_C'
           *  Constant: '<S21>/ACCS_facDstToSpdNeg_C'
           *  Product: '<S24>/Product7'
           *  Product: '<S24>/Product9'
           *  Sum: '<S24>/Sum6'
           *  Sum: '<S24>/Sum7'
           */
          rtb_Swt1_a = (rtb_A_iw - LgtDstRel) * ACCS_facDstToSpdNeg_C +
            LgtDstRel * ACCS_facDstToSpdCurr_C;
        }

        /* End of Switch: '<S24>/Swt' */
      }

      /* End of Switch: '<S21>/Switch1' */

      /* Sum: '<S21>/Add2' incorporates:
       *  Constant: '<S21>/ACCS_facRelSpd2Spd_C'
       *  Product: '<S21>/P2'
       */
      rtb_deltariselimit_b = (rtb_Swt1_a + ACCS_stActvd) + Switch *
        ACCS_facRelSpd2Spd_C;

      /* Saturate: '<S21>/Saturation' */
      if (rtb_deltariselimit_b > 100.0F) {
        rtb_deltariselimit_b = 100.0F;
      } else if (rtb_deltariselimit_b < (-2.0F)) {
        rtb_deltariselimit_b = (-2.0F);
      }

      /* End of Saturate: '<S21>/Saturation' */

      /* Switch: '<S21>/S' incorporates:
       *  MinMax: '<S21>/MinMax2'
       */
      LgtCtrl_B.S = fminf(rtb_deltariselimit_b, LgtCtrl_B.Switch2);
    } else {
      /* Switch: '<S21>/S' */
      LgtCtrl_B.S = LgtCtrl_B.Switch2;
    }

    /* End of Switch: '<S21>/S' */

    /* Sum: '<S22>/Add' incorporates:
     *  UnitDelay: '<S22>/D'
     */
    rtb_Swt1_a = LgtCtrl_B.S - LgtCtrl_B.Swt4;

    /* Chart: '<S28>/FeedForwardAccrCalcn_100ms' incorporates:
     *  Constant: '<S28>/ACCS_numAccrCalcnCnt_C'
     */
    /* Gateway: LgtCtrl/ACCS_Decision/ACCS_DesSpdCalcn/SetPointRamp/Subsystem/FeedForwardAccrCalcn_100ms */
    /* During: LgtCtrl/ACCS_Decision/ACCS_DesSpdCalcn/SetPointRamp/Subsystem/FeedForwardAccrCalcn_100ms */
    /* Entry Internal: LgtCtrl/ACCS_Decision/ACCS_DesSpdCalcn/SetPointRamp/Subsystem/FeedForwardAccrCalcn_100ms */
    /* Transition: '<S30>:2' */
    LgtCtrl_DW.counter++;
    LgtCtrl_DW.counter = fminf(LgtCtrl_DW.counter, 5.0F + 1.0F);
    while (LgtCtrl_DW.index < 5.0F) {
      /* Transition: '<S30>:6' */
      /* Transition: '<S30>:7' */
      if (LgtCtrl_DW.index < 2.14748365E+9F) {
        if (LgtCtrl_DW.index >= -2.14748365E+9F) {
          i = (int32_T)LgtCtrl_DW.index;
        } else {
          i = MIN_int32_T;
        }
      } else {
        i = MAX_int32_T;
      }

      if (LgtCtrl_DW.index + 1.0F < 2.14748365E+9F) {
        if (LgtCtrl_DW.index + 1.0F >= -2.14748365E+9F) {
          judge = (int32_T)(LgtCtrl_DW.index + 1.0F);
        } else {
          judge = MIN_int32_T;
        }
      } else {
        judge = MAX_int32_T;
      }

      LgtCtrl_DW.SpeedAry[i] = LgtCtrl_DW.SpeedAry[judge];
      LgtCtrl_DW.index++;

      /* Transition: '<S30>:8' */
    }

    /* Transition: '<S30>:30' */
    if (LgtCtrl_DW.index < 2.14748365E+9F) {
      if (LgtCtrl_DW.index >= -2.14748365E+9F) {
        i = (int32_T)LgtCtrl_DW.index;
      } else {
        i = MIN_int32_T;
      }
    } else {
      i = MAX_int32_T;
    }

    LgtCtrl_DW.SpeedAry[i] = LgtCtrl_B.S;
    LgtCtrl_DW.index = 0.0F;
    if (5.0F + 1.0F == LgtCtrl_DW.counter) {
      /* Transition: '<S30>:16' */
      /* Transition: '<S30>:19' */
      if (5.0F < 2.14748365E+9F) {
        if (5.0F >= -2.14748365E+9F) {
          i = (int32_T)5.0F;
        } else {
          i = MIN_int32_T;
        }
      } else {
        i = MAX_int32_T;
      }

      rtb_D1_i = (real32_T)((LgtCtrl_DW.SpeedAry[i] - LgtCtrl_DW.SpeedAry[0]) /
                            3.6 / 0.1 / 5.0F);

      /* Transition: '<S30>:20' */
    } else {
      /* Transition: '<S30>:21' */
      rtb_D1_i = 0.0F;
    }

    /* End of Chart: '<S28>/FeedForwardAccrCalcn_100ms' */

    /* Chart: '<S22>/FeedForwardAccrCalcn_100ms' */
    /* Gateway: LgtCtrl/ACCS_Decision/ACCS_DesSpdCalcn/SetPointRamp/FeedForwardAccrCalcn_100ms */
    /* During: LgtCtrl/ACCS_Decision/ACCS_DesSpdCalcn/SetPointRamp/FeedForwardAccrCalcn_100ms */
    /* Entry Internal: LgtCtrl/ACCS_Decision/ACCS_DesSpdCalcn/SetPointRamp/FeedForwardAccrCalcn_100ms */
    /* Transition: '<S26>:2' */
    LgtCtrl_DW.counter_d++;
    LgtCtrl_DW.counter_d = fminf(LgtCtrl_DW.counter_d, 5.0F);
    while (LgtCtrl_DW.index_f < 2.0F) {
      /* Transition: '<S26>:6' */
      /* Transition: '<S26>:7' */
      if (LgtCtrl_DW.index_f < 2.14748365E+9F) {
        if (LgtCtrl_DW.index_f >= -2.14748365E+9F) {
          i = (int32_T)LgtCtrl_DW.index_f;
        } else {
          i = MIN_int32_T;
        }
      } else {
        i = MAX_int32_T;
      }

      if (LgtCtrl_DW.index_f + 1.0F < 2.14748365E+9F) {
        if (LgtCtrl_DW.index_f + 1.0F >= -2.14748365E+9F) {
          judge = (int32_T)(LgtCtrl_DW.index_f + 1.0F);
        } else {
          judge = MIN_int32_T;
        }
      } else {
        judge = MAX_int32_T;
      }

      LgtCtrl_DW.SpeedAry_g[i] = LgtCtrl_DW.SpeedAry_g[judge];
      LgtCtrl_DW.index_f++;

      /* Transition: '<S26>:8' */
    }

    /* Transition: '<S26>:30' */
    if (LgtCtrl_DW.index_f < 2.14748365E+9F) {
      if (LgtCtrl_DW.index_f >= -2.14748365E+9F) {
        i = (int32_T)LgtCtrl_DW.index_f;
      } else {
        i = MIN_int32_T;
      }
    } else {
      i = MAX_int32_T;
    }

    LgtCtrl_DW.SpeedAry_g[i] = rtb_D1_i;
    LgtCtrl_DW.index_f = 0.0F;
    if (LgtCtrl_DW.counter_d == 5.0F) {
      /* Transition: '<S26>:16' */
      /* Transition: '<S26>:19' */
      rtb_A_iw = ((LgtCtrl_DW.SpeedAry_g[1] + LgtCtrl_DW.SpeedAry_g[2]) +
                  LgtCtrl_DW.SpeedAry_g[0]) / 3.0F;

      /* Transition: '<S26>:20' */
    } else {
      /* Transition: '<S26>:21' */
      rtb_A_iw = 0.0F;
    }

    /* End of Chart: '<S22>/FeedForwardAccrCalcn_100ms' */

    /* MultiPortSwitch: '<S22>/Multiport Switch' incorporates:
     *  Constant: '<S22>/ACCS_stFrontTargetSwt_C'
     *  Constant: '<S22>/C9'
     */
    switch (ACCS_stFrontTargetSwt_C) {
     case 0:
      rtb_D1_i = 0.0F;
      break;

     case 1:
      break;

     case 2:
      rtb_D1_i = rtb_A_iw;
      break;

     default:
      rtb_D1_i = 0.0F;
      break;
    }

    /* End of MultiPortSwitch: '<S22>/Multiport Switch' */

    /* Saturate: '<S22>/Saturation' */
    if (rtb_D1_i > 10.0F) {
      rtb_A_iw = 10.0F;
    } else if (rtb_D1_i < (-10.0F)) {
      rtb_A_iw = (-10.0F);
    } else {
      rtb_A_iw = rtb_D1_i;
    }

    /* End of Saturate: '<S22>/Saturation' */

    /* UnitDelay: '<S22>/D1' */
    rtb_D1_i = LgtCtrl_B.Swt12;

    /* Switch: '<S22>/Swt10' incorporates:
     *  Constant: '<S22>/C2'
     *  Constant: '<S22>/Constant1'
     *  RelationalOperator: '<S22>/L'
     *  RelationalOperator: '<S22>/R8'
     *  Switch: '<S22>/S'
     *  UnitDelay: '<S1>/Unit Delay'
     */
    if (CoChs_stCdn == 2.0) {
      /* MinMax: '<S22>/MinMax5' incorporates:
       *  Constant: '<S22>/ACCS_dtSpSlpUp_C'
       *  Constant: '<S22>/ACCS_facSpSlpUpCtrlPnt_C'
       *  Product: '<S22>/P5'
       *  Sum: '<S22>/A8'
       */
      rtb_Gain = fminf(rtb_Swt1_a * ACCS_facSpSlpUpCtrlPnt_C + rtb_A_iw,
                       ACCS_dtSpSlpUp_C);

      /* Switch: '<S22>/Swt5' incorporates:
       *  Constant: '<S22>/ACCS_dtAccrSlpUp_C'
       *  Constant: '<S22>/AdpvCC_tiSpdCtrlUpdCyc_C'
       *  MinMax: '<S22>/MinMax4'
       *  Product: '<S22>/P2'
       *  RelationalOperator: '<S22>/R2'
       *  RelationalOperator: '<S22>/R3'
       *  Sum: '<S22>/A4'
       *  Switch: '<S22>/Swt6'
       *  UnitDelay: '<S22>/D2'
       */
      if (rtb_Gain > LgtCtrl_B.Swt12) {
        rtb_D1_i = fminf(ACCS_dtAccrSlpUp_C * AdpvCC_tiSpdCtrlUpdCyc_C +
                         LgtCtrl_B.Swt12, rtb_Gain);
      } else if (rtb_Gain < LgtCtrl_B.Swt12) {
        /* Switch: '<S22>/Swt6' incorporates:
         *  Constant: '<S22>/ACCS_dtAccrSlpDwn_C'
         *  Constant: '<S22>/AdpvCC_tiSpdCtrlUpdCyc_C'
         *  MinMax: '<S22>/MinMax3'
         *  Product: '<S22>/P1'
         *  Sum: '<S22>/A3'
         */
        rtb_D1_i = fmaxf(ACCS_dtAccrSlpDwn_C * AdpvCC_tiSpdCtrlUpdCyc_C +
                         LgtCtrl_B.Swt12, rtb_Gain);
      }

      /* End of Switch: '<S22>/Swt5' */
    } else {
      if (LgtCtrl_B.Switch_o != 0.0) {
        /* Switch: '<S21>/S2' incorporates:
         *  Constant: '<S21>/ACCS_dtSpSlpDwnWthTar_C'
         *  Constant: '<S21>/C9'
         *  RelationalOperator: '<S21>/L3'
         *  Saturate: '<S21>/Saturation3'
         *  Switch: '<S22>/S'
         */
        if (UnitDelay < 15.0) {
          rtb_Gain = ACCS_dtSpSlpDwnWthTar_C;
        } else {
          if (UnitDelay > 200.0F) {
            /* Saturate: '<S21>/Saturation3' */
            UnitDelay = 200.0F;
          } else if (UnitDelay < 10.0F) {
            /* Saturate: '<S21>/Saturation3' */
            UnitDelay = 10.0F;
          }

          /* Switch: '<S21>/S1' incorporates:
           *  Constant: '<S21>/C6'
           *  Constant: '<S21>/C7'
           *  Constant: '<S21>/C8'
           *  RelationalOperator: '<S21>/L2'
           *  Saturate: '<S21>/Saturation3'
           */
          if (rtb_Gain > 0.0) {
            rtb_UnitDelay_lk = (-1.0);
          } else {
            rtb_UnitDelay_lk = 1.0;
          }

          /* End of Switch: '<S21>/S1' */

          /* Gain: '<S21>/Gain3' incorporates:
           *  Product: '<S21>/Divide4'
           */
          rtb_Gain = (real32_T)(rtb_Swt1_o_idx_2 / UnitDelay * rtb_UnitDelay_lk)
            * 0.5F;

          /* Saturate: '<S21>/Saturation2' */
          if (rtb_Gain > 0.0F) {
            rtb_Gain = 0.0F;
          } else if (rtb_Gain < (-3.0F)) {
            rtb_Gain = (-3.0F);
          }

          /* End of Saturate: '<S21>/Saturation2' */
        }
      } else {
        /* Switch: '<S22>/S' incorporates:
         *  Constant: '<S22>/ACCS_dtSpSlpDwn_C'
         */
        rtb_Gain = ACCS_dtSpSlpDwn_C;
      }

      /* MinMax: '<S22>/MinMax7' incorporates:
       *  Constant: '<S22>/ACCS_facSpSlpDwnCtrlPnt_C'
       *  Product: '<S22>/P6'
       *  Sum: '<S22>/A7'
       */
      rtb_Gain = fmaxf(rtb_Swt1_a * ACCS_facSpSlpDwnCtrlPnt_C + rtb_A_iw,
                       rtb_Gain);

      /* Switch: '<S22>/Swt8' incorporates:
       *  Constant: '<S22>/ACCS_dtDecSlpUp_C'
       *  Constant: '<S22>/AdpvCC_tiSpdCtrlUpdCyc_C'
       *  MinMax: '<S22>/MinMax10'
       *  Product: '<S22>/P8'
       *  RelationalOperator: '<S22>/R4'
       *  RelationalOperator: '<S22>/R5'
       *  Sum: '<S22>/A6'
       *  Switch: '<S22>/Swt9'
       *  UnitDelay: '<S22>/D1'
       */
      if (rtb_Gain > LgtCtrl_B.Swt12) {
        rtb_D1_i = fminf(ACCS_dtDecSlpUp_C * AdpvCC_tiSpdCtrlUpdCyc_C +
                         LgtCtrl_B.Swt12, rtb_Gain);
      } else if (rtb_Gain < LgtCtrl_B.Swt12) {
        /* Switch: '<S22>/Swt9' incorporates:
         *  Constant: '<S22>/ACCS_dtDecSlpDwn_C'
         *  Constant: '<S22>/AdpvCC_tiSpdCtrlUpdCyc_C'
         *  MinMax: '<S22>/MinMax9'
         *  Product: '<S22>/P7'
         *  Sum: '<S22>/A5'
         */
        rtb_D1_i = fmaxf(ACCS_dtDecSlpDwn_C * AdpvCC_tiSpdCtrlUpdCyc_C +
                         LgtCtrl_B.Swt12, rtb_Gain);
      }

      /* End of Switch: '<S22>/Swt8' */
    }

    /* Gain: '<S22>/Gain4' */
    rtb_Gain = 3.6F * rtb_D1_i;

    /* Logic: '<S22>/Logical Operator' incorporates:
     *  Constant: '<S22>/C3'
     *  Constant: '<S22>/C4'
     *  DataTypeConversion: '<S2>/Data Type Conversion7'
     *  RelationalOperator: '<S22>/L1'
     *  RelationalOperator: '<S22>/Relational Operator2'
     *  UnitDelay: '<S1>/Unit Delay'
     */
    rtb_L4_c = ((Tra_stShiftWthSpdUp == 1.0) && (CoChs_stCdn == 2.0));

    /* Logic: '<S22>/Logical Operator1' */
    rtb_RelationalOperator1 = !rtb_L4_c;

    /* Logic: '<S22>/Logical Operator3' incorporates:
     *  Constant: '<S22>/C1'
     *  Constant: '<S22>/C11'
     *  Constant: '<S22>/C5'
     *  Constant: '<S22>/C6'
     *  Logic: '<S22>/Logical Operator6'
     *  Logic: '<S22>/Logical Operator7'
     *  RelationalOperator: '<S22>/L2'
     *  RelationalOperator: '<S22>/Relational Operator1'
     *  RelationalOperator: '<S22>/Relational Operator3'
     *  RelationalOperator: '<S22>/Relational Operator4'
     *  UnitDelay: '<S1>/Unit Delay'
     */
    LogicalOperator2 = ((CoChs_stCdn != 4.0) && (CoChs_stCdn != 5.0) &&
                        (CoChs_stCdn != 6.0) && (CoChs_stCdn != 0.0));

    /* Switch: '<S22>/Swt4' incorporates:
     *  DataTypeConversion: '<S2>/Data Type Conversion12'
     *  Switch: '<S22>/Swt3'
     */
    if (rtb_LogicalOperator3) {
      /* Switch: '<S22>/Swt4' */
      LgtCtrl_B.Swt4 = rtb_sampletime;
    } else if (LogicalOperator2) {
      /* Switch: '<S22>/Swt2' incorporates:
       *  Switch: '<S22>/Swt3'
       */
      if (rtb_RelationalOperator1) {
        /* Switch: '<S22>/Swt' incorporates:
         *  RelationalOperator: '<S22>/R'
         *  RelationalOperator: '<S22>/R1'
         *  Switch: '<S22>/Swt1'
         *  UnitDelay: '<S22>/D'
         */
        if (LgtCtrl_B.S > LgtCtrl_B.Swt4) {
          /* Switch: '<S22>/Swt4' incorporates:
           *  Constant: '<S22>/AdpvCC_tiSpdCtrlUpdCyc_C'
           *  MinMax: '<S22>/MinMax2'
           *  Product: '<S22>/P4'
           *  Sum: '<S22>/A2'
           */
          LgtCtrl_B.Swt4 = fminf(rtb_Gain * AdpvCC_tiSpdCtrlUpdCyc_C +
            LgtCtrl_B.Swt4, LgtCtrl_B.S);
        } else if (LgtCtrl_B.S < LgtCtrl_B.Swt4) {
          /* Switch: '<S22>/Swt4' incorporates:
           *  Constant: '<S22>/AdpvCC_tiSpdCtrlUpdCyc_C'
           *  MinMax: '<S22>/MinMax1'
           *  Product: '<S22>/P3'
           *  Sum: '<S22>/A1'
           *  Switch: '<S22>/Swt1'
           */
          LgtCtrl_B.Swt4 = fmaxf(rtb_Gain * AdpvCC_tiSpdCtrlUpdCyc_C +
            LgtCtrl_B.Swt4, LgtCtrl_B.S);
        }

        /* End of Switch: '<S22>/Swt' */
      } else {
        /* Switch: '<S22>/Swt4' incorporates:
         *  Sum: '<S22>/Add2'
         *  UnitDelay: '<S22>/Unit Delay1'
         */
        LgtCtrl_B.Swt4 = LgtCtrl_DW.UnitDelay1_DSTATE_gz + VehDa_vEgoSpd;
      }

      /* End of Switch: '<S22>/Swt2' */
    } else {
      /* Switch: '<S22>/Swt4' incorporates:
       *  Switch: '<S22>/Swt3'
       */
      LgtCtrl_B.Swt4 = VehDa_vEgoSpd;
    }

    /* End of Switch: '<S22>/Swt4' */

    /* Chart: '<S22>/DesAccrCalcn' incorporates:
     *  Constant: '<S22>/AdpvCC_tiSpdCtrlUpdCyc_C'
     */
    /* Gateway: LgtCtrl/ACCS_Decision/ACCS_DesSpdCalcn/SetPointRamp/DesAccrCalcn */
    /* During: LgtCtrl/ACCS_Decision/ACCS_DesSpdCalcn/SetPointRamp/DesAccrCalcn */
    /* Entry Internal: LgtCtrl/ACCS_Decision/ACCS_DesSpdCalcn/SetPointRamp/DesAccrCalcn */
    /* Transition: '<S25>:36' */
    if (rtb_L4_c) {
      /* Transition: '<S25>:40' */
      /* Transition: '<S25>:39' */
      LgtCtrl_DW.counter_p = 0.0F;
    } else {
      /* Transition: '<S25>:41' */
    }

    /* Transition: '<S25>:2' */
    LgtCtrl_DW.counter_p++;
    LgtCtrl_DW.counter_p = fminf(LgtCtrl_DW.counter_p, 3.0F);
    while (LgtCtrl_DW.index_b < 2.0F) {
      /* Transition: '<S25>:6' */
      /* Transition: '<S25>:7' */
      if (LgtCtrl_DW.index_b < 2.14748365E+9F) {
        if (LgtCtrl_DW.index_b >= -2.14748365E+9F) {
          i = (int32_T)LgtCtrl_DW.index_b;
        } else {
          i = MIN_int32_T;
        }
      } else {
        i = MAX_int32_T;
      }

      if (LgtCtrl_DW.index_b + 1.0F < 2.14748365E+9F) {
        if (LgtCtrl_DW.index_b + 1.0F >= -2.14748365E+9F) {
          judge = (int32_T)(LgtCtrl_DW.index_b + 1.0F);
        } else {
          judge = MIN_int32_T;
        }
      } else {
        judge = MAX_int32_T;
      }

      LgtCtrl_DW.SpeedAry_p[i] = LgtCtrl_DW.SpeedAry_p[judge];
      LgtCtrl_DW.index_b++;

      /* Transition: '<S25>:8' */
    }

    /* Transition: '<S25>:30' */
    if (LgtCtrl_DW.index_b < 2.14748365E+9F) {
      if (LgtCtrl_DW.index_b >= -2.14748365E+9F) {
        i = (int32_T)LgtCtrl_DW.index_b;
      } else {
        i = MIN_int32_T;
      }
    } else {
      i = MAX_int32_T;
    }

    LgtCtrl_DW.SpeedAry_p[i] = LgtCtrl_B.Swt4;
    LgtCtrl_DW.index_b = 0.0F;
    if (LgtCtrl_DW.counter_p == 3.0F) {
      /* Transition: '<S25>:16' */
      /* Transition: '<S25>:19' */
      rtb_Gain = (real32_T)((LgtCtrl_DW.SpeedAry_p[2] - LgtCtrl_DW.SpeedAry_p[0])
                            / 3.6 / 2.0 / AdpvCC_tiSpdCtrlUpdCyc_C);

      /* Transition: '<S25>:20' */
    } else {
      /* Transition: '<S25>:21' */
      rtb_Gain = 0.0F;
    }

    /* End of Chart: '<S22>/DesAccrCalcn' */

    /* Chart: '<S22>/FeedForwardAccrCalcn_100ms1' */
    Lgt_FeedForwardAccrCalcn_100ms1(VehDa_vEgoSpd,
      &LgtCtrl_DW.sf_FeedForwardAccrCalcn_100ms1);

    /* Chart: '<S29>/FeedForwardAccrCalcn_100ms1' */
    Lgt_FeedForwardAccrCalcn_100ms1(VehDa_vEgoSpd,
      &LgtCtrl_DW.sf_FeedForwardAccrCalcn_100ms_b);

    /* Switch: '<S22>/Switch' */
    if (rtb_RelationalOperator1) {
      /* Update for UnitDelay: '<S22>/Unit Delay1' incorporates:
       *  Sum: '<S22>/Add1'
       */
      LgtCtrl_DW.UnitDelay1_DSTATE_gz = LgtCtrl_B.Swt4 - VehDa_vEgoSpd;
    }

    /* End of Switch: '<S22>/Switch' */

    /* Switch: '<S22>/Swt12' incorporates:
     *  DataTypeConversion: '<S2>/Data Type Conversion12'
     *  Switch: '<S22>/Swt11'
     */
    if (rtb_LogicalOperator3) {
      /* Switch: '<S22>/Swt12' incorporates:
       *  Constant: '<S16>/AEBS_aDesFf_C'
       */
      LgtCtrl_B.Swt12 = AEBS_aDesFf_C;
    } else if (LogicalOperator2) {
      /* Switch: '<S22>/Swt7' incorporates:
       *  Switch: '<S22>/Swt11'
       */
      if (rtb_RelationalOperator1) {
        /* Switch: '<S22>/Swt12' */
        LgtCtrl_B.Swt12 = rtb_Gain;
      } else {
        /* Switch: '<S22>/Swt12' incorporates:
         *  Constant: '<S22>/C8'
         */
        LgtCtrl_B.Swt12 = 0.0F;
      }

      /* End of Switch: '<S22>/Swt7' */
    } else {
      /* Switch: '<S22>/Swt12' incorporates:
       *  Constant: '<S22>/C10'
       *  Switch: '<S22>/Swt11'
       */
      LgtCtrl_B.Swt12 = 0.0F;
    }

    /* End of Switch: '<S22>/Swt12' */

    /* Switch: '<S22>/Switch1' */
    if (rtb_RelationalOperator1) {
      /* Update for UnitDelay: '<S22>/Unit Delay2' */
      LgtCtrl_DW.UnitDelay2_DSTATE_o = LgtCtrl_B.Swt12;
    }

    /* End of Switch: '<S22>/Switch1' */
  }

  /* End of Outputs for SubSystem: '<S2>/ACCS_DesSpdCalcn' */

  /* DataTypeConversion: '<S14>/Data Type Conversion6' incorporates:
   *  Gain: '<S150>/Gain'
   *  Inport: '<Root>/VehDa_stCluSwt_mp'
   */
  VehDa_stCluSwt = (uint8_T)(((uint32_T)((uint8_T)128U) * VehDa_stCluSwt_mp) >>
    7);

  /* Switch: '<S146>/Switch1' incorporates:
   *  Constant: '<S14>/ACC_RXFlag'
   *  Switch: '<S146>/Switch'
   */
  if (rtb_DataTypeConversion6_g != 0) {
    /* DataTypeConversion: '<S14>/Data Type Conversion19' incorporates:
     *  Constant: '<S146>/Sys_stADMd_C'
     */
    Sys_stADMd = Sys_stADMd_C;
  } else if (ACC_RXFlag != 0) {
    /* Switch: '<S146>/Switch' incorporates:
     *  DataTypeConversion: '<S14>/Data Type Conversion19'
     *  UnitDelay: '<S1>/Unit Delay1'
     */
    Sys_stADMd = LgtCtrl_DW.UnitDelay1_DSTATE_dg;
  } else {
    /* DataTypeConversion: '<S14>/Data Type Conversion19' incorporates:
     *  Inport: '<Root>/Sys_stADMd_mp '
     *  Switch: '<S146>/Switch'
     */
    Sys_stADMd = Sys_stADMd_mp;
  }

  /* End of Switch: '<S146>/Switch1' */

  /* Switch: '<S168>/Switch' incorporates:
   *  Constant: '<S14>/ACC_RXFlag'
   */
  if (ACC_RXFlag != 0) {
    /* Switch: '<S168>/Switch' incorporates:
     *  UnitDelay: '<S1>/Unit Delay2'
     */
    SpdPln_vTrgSpd = LgtCtrl_DW.UnitDelay2_DSTATE_i;
  } else {
    /* Switch: '<S168>/Switch' incorporates:
     *  Inport: '<Root>/SpdPln_vTrgSpd_mp '
     */
    SpdPln_vTrgSpd = SpdPln_vTrgSpd_mp;
  }

  /* End of Switch: '<S168>/Switch' */

  /* DataTypeConversion: '<S14>/Data Type Conversion25' incorporates:
   *  Inport: '<Root>/SpdPln_stReleaseThrottle_mp'
   */
  SpdPln_stReleaseThrottle = SpdPln_stReleaseThrottle_mp;

  /* Gain: '<S163>/Gain' incorporates:
   *  Inport: '<Root>/SpdPln_lTrgLngErr_mp'
   */
  SpdPln_lTrgLngErr = 1.0F * SpdPln_lTrgLngErr_mp;

  /* UnitDelay: '<S141>/U4' */
  rtb_Gain = LgtCtrl_DW.U4_DSTATE;

  /* Switch: '<S141>/Swt1' incorporates:
   *  Constant: '<S141>/C '
   *  Constant: '<S141>/VMC_swtLocnDifFild_C'
   *  UnitDelay: '<S141>/Unit Delay1'
   */
  if (LgtCtrl_DW.UnitDelay1_DSTATE != 0.0) {
    rtb_Sum6 = VMC_swtLocnDifFild_C;
  } else {
    rtb_Sum6 = 1.0;
  }

  /* End of Switch: '<S141>/Swt1' */

  /* Switch: '<S141>/Swt' incorporates:
   *  Constant: '<S141>/VMC_facLocnDifFild_C'
   *  Product: '<S141>/P5'
   *  Sum: '<S141>/Sum1'
   *  Sum: '<S141>/Sum6'
   *  UnitDelay: '<S141>/U5'
   */
  if (rtb_Sum6 != 0.0) {
    rtb_Gain = SpdPln_lTrgLngErr;
  } else {
    rtb_Gain += (LgtCtrl_DW.U5_DSTATE - rtb_Gain) * VMC_facLocnDifFild_C;
  }

  /* End of Switch: '<S141>/Swt' */

  /* Switch: '<S140>/Swt' incorporates:
   *  Constant: '<S137>/VMC_facVelGvnrKpCurr_C'
   *  Constant: '<S137>/VMC_lVelGvnrPWinNeg_C'
   *  Constant: '<S137>/VMC_lVelGvnrPWinPos_C'
   *  Logic: '<S140>/L'
   *  Logic: '<S140>/L1'
   *  Product: '<S140>/Product5'
   *  RelationalOperator: '<S140>/R'
   *  RelationalOperator: '<S140>/R1'
   *  RelationalOperator: '<S140>/R2'
   *  RelationalOperator: '<S140>/R3'
   *  Switch: '<S140>/Swt1'
   */
  if ((VMC_lVelGvnrPWinPos_C < VMC_lVelGvnrPWinNeg_C) || ((rtb_Gain <=
        VMC_lVelGvnrPWinPos_C) && (rtb_Gain >= VMC_lVelGvnrPWinNeg_C))) {
    rtb_Swt_lo = rtb_Gain * VMC_facVelGvnrKpCurr_C;
  } else if (rtb_Gain > VMC_lVelGvnrPWinPos_C) {
    /* Switch: '<S140>/Swt1' incorporates:
     *  Constant: '<S137>/VMC_facVelGvnrKpCurr_C'
     *  Constant: '<S137>/VMC_facVelGvnrKpPosCurr_C'
     *  Product: '<S140>/Product6'
     *  Product: '<S140>/Product8'
     *  Sum: '<S140>/Sum4'
     *  Sum: '<S140>/Sum5'
     */
    rtb_Swt_lo = (rtb_Gain - VMC_lVelGvnrPWinPos_C) * VMC_facVelGvnrKpPosCurr_C
      + VMC_lVelGvnrPWinPos_C * VMC_facVelGvnrKpCurr_C;
  } else {
    /* Switch: '<S140>/Swt1' incorporates:
     *  Constant: '<S137>/VMC_facVelGvnrKpCurr_C'
     *  Constant: '<S137>/VMC_facVelGvnrKpNegCurr_C'
     *  Product: '<S140>/Product7'
     *  Product: '<S140>/Product9'
     *  Sum: '<S140>/Sum6'
     *  Sum: '<S140>/Sum7'
     */
    rtb_Swt_lo = (rtb_Gain - VMC_lVelGvnrPWinNeg_C) * VMC_facVelGvnrKpNegCurr_C
      + VMC_lVelGvnrPWinNeg_C * VMC_facVelGvnrKpCurr_C;
  }

  /* End of Switch: '<S140>/Swt' */

  /* Sum: '<S138>/Sum1' incorporates:
   *  UnitDelay: '<S138>/Unit Delay1'
   */
  UnitDelay = rtb_Gain - LgtCtrl_DW.UnitDelay1_DSTATE_n;

  /* Product: '<S138>/Product' incorporates:
   *  Constant: '<S137>/VMC_facVelGvnrKdCurr_C'
   */
  rtb_Swt1_a = VMC_facVelGvnrKdCurr_C * UnitDelay;

  /* Switch: '<S138>/Swt2' incorporates:
   *  Constant: '<S135>/VMC_facVelGvnrDftlRatio_C'
   *  Constant: '<S135>/VMC_vVelGvnrDftlIniVal_C'
   *  Constant: '<S135>/VMC_vVelGvnrIntglIniVal_C'
   *  Constant: '<S136>/C2'
   *  Constant: '<S136>/PowerCtrl'
   *  Constant: '<S137>/VMC_lVelGvnrDWinNeg_C'
   *  Constant: '<S137>/VMC_lVelGvnrDWinPos_C'
   *  DataTypeConversion: '<S13>/Data Type Conversion'
   *  Logic: '<S136>/L8'
   *  Product: '<S138>/Product1'
   *  RelationalOperator: '<S136>/L6'
   *  RelationalOperator: '<S136>/L7'
   *  RelationalOperator: '<S138>/R'
   *  RelationalOperator: '<S138>/R1'
   *  Sum: '<S138>/Sum'
   *  Sum: '<S139>/Sum'
   *  Switch: '<S138>/Swt'
   *  Switch: '<S138>/Swt1'
   *  Switch: '<S139>/Switch4'
   *  UnitDelay: '<S138>/Unit Delay'
   *  UnitDelay: '<S139>/Unit Delay'
   *  UnitDelay: '<S1>/Unit Delay5'
   */
  if ((VehDa_stCluSwt != 0.0F) && (CoChs_stCdn == 2.0F)) {
    UnitDelay = VMC_vVelGvnrDftlIniVal_C;
    rtb_Swt1_a = VMC_vVelGvnrIntglIniVal_C;
  } else {
    if (UnitDelay > VMC_lVelGvnrDWinPos_C) {
      /* Switch: '<S138>/Swt' incorporates:
       *  Constant: '<S137>/VMC_facVelGvnrKdPosCurr_C'
       *  Constant: '<S137>/VMC_lVelGvnrDWinPos_C'
       *  Product: '<S138>/Product2'
       *  Sum: '<S138>/Sum2'
       *  Sum: '<S138>/Sum4'
       */
      rtb_Swt1_a += (UnitDelay - VMC_lVelGvnrDWinPos_C) *
        VMC_facVelGvnrKdPosCurr_C;
    } else if (UnitDelay < VMC_lVelGvnrDWinNeg_C) {
      /* Switch: '<S138>/Swt' incorporates:
       *  Constant: '<S137>/VMC_facVelGvnrKdNegCurr_C'
       *  Constant: '<S137>/VMC_lVelGvnrDWinNeg_C'
       *  Product: '<S138>/Product3'
       *  Sum: '<S138>/Sum3'
       *  Sum: '<S138>/Sum5'
       *  Switch: '<S138>/Swt1'
       */
      rtb_Swt1_a += (UnitDelay - VMC_lVelGvnrDWinNeg_C) *
        VMC_facVelGvnrKdNegCurr_C;
    }

    UnitDelay = VMC_facVelGvnrDftlRatio_C * LgtCtrl_DW.UnitDelay_DSTATE_as +
      rtb_Swt1_a;

    /* Switch: '<S139>/Switch3' incorporates:
     *  Constant: '<S135>/VMC_facVelGvnrDftlRatio_C'
     *  Constant: '<S135>/VMC_tiVelGvnrIntglUpdCyc_C'
     *  Constant: '<S136>/C7'
     *  Constant: '<S136>/C8'
     *  Constant: '<S136>/Enable'
     *  Constant: '<S136>/Normal'
     *  Constant: '<S137>/VMC_lVelGvnrDWinNeg_C'
     *  Constant: '<S137>/VMC_lVelGvnrIWinNeg_C'
     *  Constant: '<S137>/VMC_lVelGvnrIWinPos_C'
     *  Constant: '<S137>/VMC_swtVelGvnrFctEna_C'
     *  Constant: '<S139>/zero'
     *  DataTypeConversion: '<S13>/Data Type Conversion2'
     *  Logic: '<S136>/L '
     *  Logic: '<S136>/L14'
     *  Product: '<S138>/Product1'
     *  Product: '<S139>/Product'
     *  Product: '<S139>/Product1'
     *  RelationalOperator: '<S136>/L1'
     *  RelationalOperator: '<S136>/L12'
     *  RelationalOperator: '<S136>/L13'
     *  RelationalOperator: '<S136>/L3'
     *  RelationalOperator: '<S138>/R1'
     *  RelationalOperator: '<S139>/R'
     *  RelationalOperator: '<S139>/R1'
     *  RelationalOperator: '<S139>/R2'
     *  Sum: '<S138>/Sum'
     *  Switch: '<S138>/Swt'
     *  Switch: '<S138>/Swt1'
     *  Switch: '<S139>/Switch1'
     *  Switch: '<S139>/Switch2'
     *  Switch: '<S139>/Switch5'
     *  UnitDelay: '<S138>/Unit Delay'
     */
    if (((CoChs_stCdn != 2.0F) && (CoChs_stCdn != 3.0F)) || (Sys_stADMd != 3.0F)
        || (VMC_swtVelGvnrFctEna_C != 1.0F)) {
      rtb_Swt1_a = 0.0F;
    } else {
      if (VMC_lVelGvnrIWinPos_C < VMC_lVelGvnrIWinNeg_C) {
        /* Switch: '<S139>/Switch5' incorporates:
         *  Constant: '<S137>/VMC_facVelGvnrKiCurr_C'
         */
        rtb_Swt1_o_idx_2 = VMC_facVelGvnrKiCurr_C;
      } else if (rtb_Gain > VMC_lVelGvnrIWinPos_C) {
        /* Switch: '<S139>/Switch1' incorporates:
         *  Constant: '<S137>/VMC_facVelGvnrKiPosCurr_C'
         *  Switch: '<S139>/Switch5'
         */
        rtb_Swt1_o_idx_2 = VMC_facVelGvnrKiPosCurr_C;
      } else if (rtb_Gain < VMC_lVelGvnrIWinNeg_C) {
        /* Switch: '<S139>/Switch2' incorporates:
         *  Constant: '<S137>/VMC_facVelGvnrKiNegCurr_C'
         *  Switch: '<S139>/Switch1'
         *  Switch: '<S139>/Switch5'
         */
        rtb_Swt1_o_idx_2 = VMC_facVelGvnrKiNegCurr_C;
      } else {
        /* Switch: '<S139>/Switch5' incorporates:
         *  Constant: '<S137>/VMC_facVelGvnrKiCurr_C'
         *  Switch: '<S139>/Switch1'
         *  Switch: '<S139>/Switch2'
         */
        rtb_Swt1_o_idx_2 = VMC_facVelGvnrKiCurr_C;
      }

      rtb_Swt1_a = rtb_Gain * rtb_Swt1_o_idx_2 * VMC_tiVelGvnrIntglUpdCyc_C;
    }

    /* End of Switch: '<S139>/Switch3' */
    rtb_Swt1_a += LgtCtrl_DW.UnitDelay_DSTATE_i;
  }

  /* End of Switch: '<S138>/Swt2' */

  /* Switch: '<S143>/Swt' incorporates:
   *  Constant: '<S137>/VMC_vVelGvnrIntglCtrlValLo_C'
   *  Constant: '<S137>/VMC_vVelGvnrIntglCtrlValUp_C'
   *  MinMax: '<S143>/MinMax'
   *  MinMax: '<S143>/MinMax1'
   *  RelationalOperator: '<S143>/R3'
   */
  if (VMC_vVelGvnrIntglCtrlValUp_C >= VMC_vVelGvnrIntglCtrlValLo_C) {
    rtb_Swt1_a = fminf(fmaxf(VMC_vVelGvnrIntglCtrlValLo_C, rtb_Swt1_a),
                       VMC_vVelGvnrIntglCtrlValUp_C);
  }

  /* End of Switch: '<S143>/Swt' */

  /* Switch: '<S142>/Swt' incorporates:
   *  Constant: '<S137>/VMC_swtVelGvnrFctEna_C'
   *  Constant: '<S142>/C'
   */
  if (VMC_swtVelGvnrFctEna_C != 0) {
    /* Switch: '<S144>/Swt' incorporates:
     *  Constant: '<S142>/VMC_vVelGvnrOutpLo_C'
     *  Constant: '<S142>/VMC_vVelGvnrOutpUp_C'
     *  MinMax: '<S144>/MinMax'
     *  MinMax: '<S144>/MinMax1'
     *  RelationalOperator: '<S144>/R3'
     *  Sum: '<S142>/A'
     */
    if (VMC_vVelGvnrOutpUp_C < VMC_vVelGvnrOutpLo_C) {
      rtb_Swt1_o_idx_2 = (rtb_Swt_lo + UnitDelay) + rtb_Swt1_a;
    } else {
      rtb_Swt1_o_idx_2 = fminf(fmaxf(VMC_vVelGvnrOutpLo_C, (rtb_Swt_lo +
        UnitDelay) + rtb_Swt1_a), VMC_vVelGvnrOutpUp_C);
    }

    /* End of Switch: '<S144>/Swt' */
  } else {
    rtb_Swt1_o_idx_2 = 0.0F;
  }

  /* End of Switch: '<S142>/Swt' */

  /* Sum: '<S142>/A1' incorporates:
   *  Constant: '<S142>/MPS_TO_KPH'
   *  Product: '<S142>/Product'
   */
  VMC_vReq = SpdPln_vTrgSpd * 3.6F + rtb_Swt1_o_idx_2;

  /* Switch: '<S170>/Switch1' incorporates:
   *  Constant: '<S14>/LgtCtrl_RXFlag'
   *  Constant: '<S170>/VehDa_mWght_C'
   *  Inport: '<Root>/VehDa_mWght_mp'
   *  S-Function (sfix_bitop): '<S170>/Bitwise Operator'
   */
  if ((LgtCtrl_RXFlag & ((uint8_T)128U)) != 0) {
    rtb_Swt1_o_idx_2 = VehDa_mWght_C;
  } else {
    rtb_Swt1_o_idx_2 = VehDa_mWght_mp;
  }

  /* End of Switch: '<S170>/Switch1' */

  /* MinMax: '<S170>/MinMax1' incorporates:
   *  Constant: '<S170>/C'
   *  Constant: '<S170>/C1'
   *  MinMax: '<S170>/MinMax'
   */
  VehDa_mWght = fmaxf(fminf(rtb_Swt1_o_idx_2, 49000.0F), 2000.0F);

  /* Switch: '<S145>/Switch' incorporates:
   *  Constant: '<S145>/Constant'
   *  Constant: '<S145>/Constant2'
   *  Constant: '<S145>/Constant3'
   *  RelationalOperator: '<S145>/Relational Operator'
   *  Sum: '<S145>/Add'
   *  UnitDelay: '<S145>/Unit Delay'
   */
  if (LgtCtrl_DW.UnitDelay_DSTATE_c < 5.0) {
    rtb_Switch_cx = 1.0 + LgtCtrl_DW.UnitDelay_DSTATE_c;
  } else {
    rtb_Switch_cx = 1.0;
  }

  /* End of Switch: '<S145>/Switch' */

  /* Outputs for Enabled SubSystem: '<S14>/Slop' incorporates:
   *  EnablePort: '<S165>/Enable'
   */
  /* RelationalOperator: '<S145>/Relational Operator1' incorporates:
   *  Constant: '<S145>/Constant3'
   */
  if (rtb_Switch_cx == 1.0) {
    /* Switch: '<S165>/Swt2' incorporates:
     *  Constant: '<S165>/VehDa_swtSlpSrc_C'
     *  Gain: '<S165>/Gain'
     *  Inport: '<Root>/VehDa_agPitch_mp'
     *  Inport: '<Root>/VehDa_rSlop_mp'
     *  Trigonometry: '<S165>/Trigonometric Function'
     */
    if (VehDa_swtSlpSrc_C != 0) {
      Switch = VehDa_rSlop_mp;
    } else {
      Switch = 100.0F * tanf(VehDa_agPitch_mp);
    }

    /* End of Switch: '<S165>/Swt2' */

    /* Switch: '<S172>/Switch2' incorporates:
     *  Constant: '<S165>/Constant'
     *  Constant: '<S165>/Constant1'
     *  RelationalOperator: '<S172>/LowerRelop1'
     *  RelationalOperator: '<S172>/UpperRelop'
     *  Switch: '<S172>/Switch'
     */
    if (Switch > 25.0F) {
      Switch = 25.0F;
    } else if (Switch < (-25.0F)) {
      /* Switch: '<S172>/Switch' incorporates:
       *  Constant: '<S165>/Constant1'
       */
      Switch = (-25.0F);
    }

    /* End of Switch: '<S172>/Switch2' */

    /* Sum: '<S165>/Sum3' incorporates:
     *  Constant: '<S165>/VehDa_rSlpCmp_C'
     */
    Switch += VehDa_rSlpCmp_C;

    /* Switch: '<S165>/Swt1' incorporates:
     *  Constant: '<S165>/C '
     *  Constant: '<S165>/VehDa_swtSlpFilt_C'
     *  UnitDelay: '<S165>/Unit Delay1'
     */
    if (LgtCtrl_DW.UnitDelay1_DSTATE_ft != 0.0F) {
      rtb_Swt1_o_idx_2 = VehDa_swtSlpFilt_C;
    } else {
      rtb_Swt1_o_idx_2 = 1.0F;
    }

    /* End of Switch: '<S165>/Swt1' */

    /* Switch: '<S165>/Swt' incorporates:
     *  Constant: '<S165>/VehDa_facSlpFilt_C'
     *  Product: '<S165>/Product5'
     *  Sum: '<S165>/Sum1'
     *  Sum: '<S165>/Sum6'
     *  UnitDelay: '<S165>/Unit Delay4'
     *  UnitDelay: '<S165>/Unit Delay5'
     */
    if (rtb_Swt1_o_idx_2 != 0.0F) {
      ACCS_stActvd = Switch;
    } else {
      ACCS_stActvd = (LgtCtrl_DW.UnitDelay5_DSTATE_b -
                      LgtCtrl_DW.UnitDelay4_DSTATE_f) * VehDa_facSlpFilt_C +
        LgtCtrl_DW.UnitDelay4_DSTATE_f;
    }

    /* End of Switch: '<S165>/Swt' */

    /* Switch: '<S165>/Switch1' incorporates:
     *  Constant: '<S14>/LgtCtrl_RXFlag'
     *  Constant: '<S165>/VehDa_rSlop_C'
     *  S-Function (sfix_bitop): '<S165>/Bitwise Operator'
     */
    if ((LgtCtrl_RXFlag & ((uint8_T)8U)) != 0) {
      rtb_D1_i = VehDa_rSlop_C;
    } else {
      rtb_D1_i = ACCS_stActvd;
    }

    /* End of Switch: '<S165>/Switch1' */

    /* Switch: '<S165>/Switch2' incorporates:
     *  Abs: '<S165>/Abs'
     *  Constant: '<S165>/C1'
     *  Constant: '<S165>/Test_stOverflow_C'
     *  Logic: '<S165>/Logical Operator1'
     *  RelationalOperator: '<S165>/Relational Operator1'
     */
    if ((fabsf(rtb_D1_i) > 0.001F) || (Test_stOverflow_C != 0)) {
      /* Switch: '<S165>/Switch2' */
      VehDa_rSlop = rtb_D1_i;
    } else {
      /* Switch: '<S165>/Switch2' incorporates:
       *  Constant: '<S165>/C2'
       */
      VehDa_rSlop = 0.0F;
    }

    /* End of Switch: '<S165>/Switch2' */

    /* Switch: '<S165>/Swt4' incorporates:
     *  Constant: '<S165>/Constant2'
     *  Constant: '<S165>/Constant3'
     *  Inport: '<Root>/VehDa_stMapIsAvail_mp'
     *  Sum: '<S165>/Add'
     *  UnitDelay: '<S165>/Unit Delay2'
     */
    if (VehDa_stMapIsAvail_mp != 0) {
      rtb_Sum6 = 0.0;
    } else {
      rtb_Sum6 = 1.0 + LgtCtrl_DW.UnitDelay2_DSTATE;
    }

    /* End of Switch: '<S165>/Swt4' */

    /* MinMax: '<S165>/MinMax' incorporates:
     *  Constant: '<S165>/Constant4'
     */
    rtb_MinMax = fmin(rtb_Sum6, 200.0);

    /* Update for UnitDelay: '<S165>/Unit Delay1' incorporates:
     *  Constant: '<S165>/C1 '
     */
    LgtCtrl_DW.UnitDelay1_DSTATE_ft = 1.0F;

    /* Update for UnitDelay: '<S165>/Unit Delay4' */
    LgtCtrl_DW.UnitDelay4_DSTATE_f = Switch;

    /* Update for UnitDelay: '<S165>/Unit Delay5' */
    LgtCtrl_DW.UnitDelay5_DSTATE_b = ACCS_stActvd;

    /* Update for UnitDelay: '<S165>/Unit Delay2' */
    LgtCtrl_DW.UnitDelay2_DSTATE = rtb_MinMax;

    /* Switch: '<S165>/Swt3' incorporates:
     *  Constant: '<S165>/Constant5'
     *  Inport: '<Root>/VehDa_stMapIsAvail_mp'
     *  Logic: '<S165>/Logical Operator'
     *  RelationalOperator: '<S165>/Relational Operator'
     */
    if ((rtb_MinMax > 40.0) || (VehDa_stMapIsAvail_mp != 0)) {
      /* Update for UnitDelay: '<S165>/Unit Delay' */
      LgtCtrl_DW.UnitDelay_DSTATE_p = 0.0F;
    }

    /* End of Switch: '<S165>/Swt3' */
  }

  /* End of RelationalOperator: '<S145>/Relational Operator1' */
  /* End of Outputs for SubSystem: '<S14>/Slop' */

  /* Trigonometry: '<S102>/Trigonometric Function2' incorporates:
   *  Gain: '<S102>/Gain'
   *  Trigonometry: '<S103>/Trigonometric Function2'
   */
  Switch = atanf(0.01F * VehDa_rSlop);

  /* Product: '<S102>/P2' incorporates:
   *  Constant: '<S102>/GRAVITY_ACCN'
   *  Product: '<S103>/P1'
   */
  ACCS_stActvd = VehDa_mWght * 9.8F;

  /* Product: '<S102>/P4' incorporates:
   *  Constant: '<S102>/VDC_facRollngResCalA_C'
   *  Constant: '<S102>/VDC_facRollngResCalB_C'
   *  Constant: '<S102>/VDC_facRollngResCalCmp_C'
   *  Product: '<S102>/P1'
   *  Product: '<S102>/P2'
   *  Product: '<S102>/P3'
   *  Sum: '<S102>/Add'
   *  Trigonometry: '<S102>/Trigonometric Function'
   *  Trigonometry: '<S102>/Trigonometric Function2'
   */
  VDC_frRollngRes_mp = ((VDC_facRollngResCalCmp_C + VDC_facRollngResCalA_C) +
                        VMC_vReq * VDC_facRollngResCalB_C) * (ACCS_stActvd *
    cosf(Switch));

  /* Product: '<S101>/P' incorporates:
   *  Constant: '<S101>/C'
   *  Constant: '<S101>/VDC_arFrnt_C'
   *  Constant: '<S101>/VDC_facCD_C'
   */
  VDC_frAirDrag_mp = VDC_facCD_C * VDC_arFrnt_C * VMC_vReq * VMC_vReq *
    0.0472813249F;

  /* Product: '<S103>/Divide' incorporates:
   *  Constant: '<S103>/Constant'
   *  Gain: '<S103>/Gain1'
   */
  rtb_deltariselimit_b = 180.0F * Switch / 3.14159274F;

  /* Saturate: '<S103>/Saturation' */
  if (rtb_deltariselimit_b > 30.0F) {
    rtb_deltariselimit_b = 30.0F;
  } else if (rtb_deltariselimit_b < (-30.0F)) {
    rtb_deltariselimit_b = (-30.0F);
  }

  /* End of Saturate: '<S103>/Saturation' */

  /* Product: '<S103>/P2' incorporates:
   *  Lookup_n-D: '<S103>/1-D Lookup Table'
   */
  VDC_frSlpRes_mp = ACCS_stActvd * look1_iflf_binlxpw(rtb_deltariselimit_b,
    LgtCtrl_ConstP.uDLookupTable_bp01Data,
    LgtCtrl_ConstP.uDLookupTable_tableData, 180U);

  /* Switch: '<S151>/Swt2' incorporates:
   *  Constant: '<S151>/C1'
   *  Constant: '<S151>/C4'
   *  Constant: '<S151>/C5'
   *  Inport: '<Root>/VehDa_rTraCurGear_mp'
   *  Logic: '<S151>/L1'
   *  RelationalOperator: '<S151>/Relational Operator'
   *  RelationalOperator: '<S151>/Relational Operator1'
   *  RelationalOperator: '<S151>/Relational Operator2'
   *  Switch: '<S151>/Swt1'
   */
  if (VehDa_stTraCurGear == 0.0F) {
    /* Switch: '<S151>/Swt2' incorporates:
     *  Constant: '<S151>/C2'
     */
    VehDa_rTraCurGear = 16.688F;
  } else if ((VehDa_rTraCurGear_mp < 30.0F) && (VehDa_rTraCurGear_mp > 0.1F)) {
    /* Switch: '<S151>/Swt1' incorporates:
     *  Inport: '<Root>/VehDa_rTraCurGear_mp'
     *  Switch: '<S151>/Swt2'
     */
    VehDa_rTraCurGear = VehDa_rTraCurGear_mp;
  }

  /* End of Switch: '<S151>/Swt2' */

  /* Switch: '<S169>/Switch' incorporates:
   *  Constant: '<S173>/Constant'
   *  Constant: '<S174>/Constant'
   *  Inport: '<Root>/VehDa_stTrlrCnctn_mp'
   *  Logic: '<S169>/Logical Operator'
   *  RelationalOperator: '<S173>/Compare'
   *  RelationalOperator: '<S174>/Compare'
   */
  if ((VehDa_stTrlrCnctn_mp == ((uint8_T)0U)) || (VehDa_stTrlrCnctn_mp ==
       ((uint8_T)1U))) {
    /* DataTypeConversion: '<S14>/Data Type Conversion22' */
    VehDa_stTrlrCnctn = VehDa_stTrlrCnctn_mp;
  } else {
    /* DataTypeConversion: '<S14>/Data Type Conversion22' incorporates:
     *  Constant: '<S169>/Constant'
     */
    rtb_Sum6 = fmod(floor(0.0), 256.0);

    /* DataTypeConversion: '<S14>/Data Type Conversion22' */
    VehDa_stTrlrCnctn = (uint8_T)(rtb_Sum6 < 0.0 ? (int32_T)(uint8_T)-(int8_T)
      (uint8_T)-rtb_Sum6 : (int32_T)(uint8_T)rtb_Sum6);
  }

  /* End of Switch: '<S169>/Switch' */

  /* Switch: '<S100>/Switch' incorporates:
   *  Constant: '<S100>/VDC_numWhlWhtTrlr_C'
   *  Constant: '<S100>/VDC_numWhl_C'
   *  DataTypeConversion: '<S11>/Data Type Conversion4'
   */
  if (VehDa_stTrlrCnctn != 0) {
    rtb_Swt1_o_idx_2 = VDC_numWhlWhtTrlr_C;
  } else {
    rtb_Swt1_o_idx_2 = VDC_numWhl_C;
  }

  /* End of Switch: '<S100>/Switch' */

  /* Sum: '<S100>/A' incorporates:
   *  Constant: '<S100>/C3'
   *  Constant: '<S100>/VDC_etaDrvAxlEff_C'
   *  Constant: '<S100>/VDC_etaTra_C'
   *  Constant: '<S100>/VDC_moiDrvTrnCmp_C'
   *  Constant: '<S100>/VDC_moiFlyWhl_C'
   *  Constant: '<S100>/VDC_moiWhl_C'
   *  Constant: '<S100>/VDC_rFinalRatio_C'
   *  Constant: '<S100>/VDC_rdTire_C'
   *  Product: '<S100>/Divide1'
   *  Product: '<S100>/Divide2'
   *  Product: '<S100>/Divide3'
   *  Product: '<S100>/Divide4'
   *  Product: '<S100>/Divide5'
   *  Product: '<S100>/P1'
   *  Product: '<S100>/Product1'
   *  Product: '<S100>/Product2'
   *  Product: '<S100>/Product4'
   *  Product: '<S100>/Product5'
   *  Product: '<S100>/Product6'
   *  Sum: '<S100>/Add1'
   *  Sum: '<S100>/Add2'
   */
  VDC_facDrvTrmJ_mp = (VehDa_rTraCurGear * VehDa_rTraCurGear * (VDC_moiFlyWhl_C
    + VDC_moiDrvTrnCmp_C) * (VDC_rFinalRatio_C * VDC_rFinalRatio_C) *
                       VDC_etaDrvAxlEff_C * VDC_etaTra_C / VDC_rdTire_C /
                       VDC_rdTire_C + VDC_moiWhl_C * rtb_Swt1_o_idx_2 /
                       VDC_rdTire_C / VDC_rdTire_C) / VehDa_mWght + 1.0F;

  /* Sum: '<S128>/A' */
  Switch = VMC_vReq - VehDa_vEgoSpd;

  /* Logic: '<S120>/L14' incorporates:
   *  Constant: '<S120>/C4'
   *  Constant: '<S120>/C7'
   *  Constant: '<S120>/C8'
   *  RelationalOperator: '<S120>/L12'
   *  RelationalOperator: '<S120>/L13'
   *  RelationalOperator: '<S120>/L8'
   *  UnitDelay: '<S1>/Unit Delay5'
   */
  rtb_L4_c = ((CoChs_stCdn != 2.0F) && (CoChs_stCdn != 3.0F) && (CoChs_stCdn !=
    6.0F));

  /* RelationalOperator: '<S120>/L1' incorporates:
   *  Constant: '<S120>/Normal'
   *  DataTypeConversion: '<S12>/Data Type Conversion2'
   */
  rtb_RelationalOperator1 = (Sys_stADMd != 3.0F);

  /* Logic: '<S120>/L7' */
  LogicalOperator2 = (rtb_L4_c || rtb_RelationalOperator1);

  /* Chart: '<S120>/debounce' */
  /* Gateway: LgtCtrl/VMC_AccnGvnr/VMC_AccnGvnrIntglStMng/debounce */
  /* During: LgtCtrl/VMC_AccnGvnr/VMC_AccnGvnrIntglStMng/debounce */
  if (LgtCtrl_DW.is_active_c8_LgtCtrl == 0U) {
    /* Entry: LgtCtrl/VMC_AccnGvnr/VMC_AccnGvnrIntglStMng/debounce */
    LgtCtrl_DW.is_active_c8_LgtCtrl = 1U;

    /* Entry Internal: LgtCtrl/VMC_AccnGvnr/VMC_AccnGvnrIntglStMng/debounce */
    /* Transition: '<S134>:3' */
    LgtCtrl_B.output = LogicalOperator2;
    LgtCtrl_DW.is_c8_LgtCtrl = LgtCtrl_IN_static;
  } else if (LgtCtrl_DW.is_c8_LgtCtrl == 1) {
    /* During 'active': '<S134>:2' */
    if (LogicalOperator2 == LgtCtrl_B.output) {
      /* Transition: '<S134>:12' */
      LgtCtrl_DW.counter_dy = 0U;
      LgtCtrl_DW.is_c8_LgtCtrl = LgtCtrl_IN_static;
    } else {
      /* Transition: '<S134>:11' */
      i = LgtCtrl_DW.counter_dy + 1;
      if (LgtCtrl_DW.counter_dy + 1 > 255) {
        i = 255;
      }

      LgtCtrl_DW.counter_dy = (uint8_T)i;
      if (LgtCtrl_DW.counter_dy >= 15) {
        /* Transition: '<S134>:10' */
        LgtCtrl_B.output = LogicalOperator2;
        LgtCtrl_DW.counter_dy = 0U;
        LgtCtrl_DW.is_c8_LgtCtrl = LgtCtrl_IN_static;
      } else {
        /* Transition: '<S134>:9' */
        LgtCtrl_DW.is_c8_LgtCtrl = LgtCtrl_IN_active_g;
      }
    }

    /* During 'static': '<S134>:1' */
  } else if (LogicalOperator2 != LgtCtrl_B.output) {
    /* Transition: '<S134>:8' */
    if (LgtCtrl_DW.counter_dy >= 15) {
      /* Transition: '<S134>:10' */
      LgtCtrl_B.output = LogicalOperator2;
      LgtCtrl_DW.counter_dy = 0U;
      LgtCtrl_DW.is_c8_LgtCtrl = LgtCtrl_IN_static;
    } else {
      /* Transition: '<S134>:9' */
      LgtCtrl_DW.is_c8_LgtCtrl = LgtCtrl_IN_active_g;
    }
  }

  /* End of Chart: '<S120>/debounce' */

  /* Switch: '<S128>/Swt1' incorporates:
   *  Constant: '<S128>/C '
   *  Constant: '<S128>/VMC_swtSpdDifFild_C'
   *  Logic: '<S128>/Logical Operator'
   *  UnitDelay: '<S128>/Unit Delay1'
   */
  if (LgtCtrl_DW.UnitDelay1_DSTATE_d != 0.0) {
    rtb_Swt1_o_idx_2 = (real32_T)((VMC_swtSpdDifFild_C != 0) ||
      (LgtCtrl_B.output != 0.0F));
  } else {
    rtb_Swt1_o_idx_2 = 1.0F;
  }

  /* End of Switch: '<S128>/Swt1' */

  /* Switch: '<S128>/Swt' */
  if (rtb_Swt1_o_idx_2 != 0.0F) {
    /* Switch: '<S128>/Swt' */
    VMC_vDif_mp = Switch;
  } else {
    /* Switch: '<S128>/Swt' incorporates:
     *  Constant: '<S128>/VMC_facSpdDifFild_C'
     *  Product: '<S128>/Product5'
     *  Sum: '<S128>/Sum1'
     *  Sum: '<S128>/Sum6'
     *  UnitDelay: '<S128>/Unit Delay4'
     *  UnitDelay: '<S128>/Unit Delay5'
     */
    VMC_vDif_mp = (VMC_vDif_mp - LgtCtrl_DW.UnitDelay4_DSTATE) *
      VMC_facSpdDifFild_C + LgtCtrl_DW.UnitDelay4_DSTATE;
  }

  /* End of Switch: '<S128>/Swt' */

  /* DataTypeConversion: '<S14>/Data Type Conversion26' incorporates:
   *  Inport: '<Root>/SpdPln_stTarType_mp'
   */
  SpdPln_stTarType = SpdPln_stTarType_mp;

  /* Chart: '<S10>/TrqLim' incorporates:
   *  DataTypeConversion: '<S10>/Data Type Conversion2'
   *  Inport: '<Root>/VehDa_stTraEgd_mp'
   *  Inport: '<Root>/VehDa_stTraSelGear_mp'
   *  Inport: '<Root>/VehDa_stTraSht_mp'
   *  Inport: '<Root>/VehDa_stTraTrqLim_mp'
   *  UnitDelay: '<S1>/Unit Delay9'
   */
  /* Gateway: LgtCtrl/TraMonr/TrqLim */
  /* During: LgtCtrl/TraMonr/TrqLim */
  if (LgtCtrl_DW.is_active_c2_LgtCtrl == 0U) {
    /* Entry: LgtCtrl/TraMonr/TrqLim */
    LgtCtrl_DW.is_active_c2_LgtCtrl = 1U;

    /* Entry Internal: LgtCtrl/TraMonr/TrqLim */
    /* Transition: '<S99>:2' */
    LgtCtrl_DW.is_c2_LgtCtrl = LgtCtrl_IN_NoLim;

    /* DataTypeConversion: '<S10>/Data Type Conversion9' */
    /* Entry 'NoLim': '<S99>:1' */
    Tra_stTrqLimWthTCU = 0U;
  } else if (LgtCtrl_DW.is_c2_LgtCtrl == 1) {
    /* DataTypeConversion: '<S10>/Data Type Conversion9' */
    Tra_stTrqLimWthTCU = 0U;

    /* During 'NoLim': '<S99>:1' */
    if (((VehDa_stTraCurGear != 0) && (VehDa_stTraSelGear_mp ==
          VehDa_stTraCurGear) && (VehDa_stTraTrqLim_mp == 3) &&
         (VehDa_stTraEgd_mp == 1) && (VehDa_stTraSht_mp == 0) && (CoChs_stCdn ==
          2)) || (VehDa_vEgoSpd < 3.0F) || ((VehDa_vEgoSpd < 9.0F) &&
         (VehDa_stTraCurGear == 3))) {
      /* Transition: '<S99>:4' */
      LgtCtrl_DW.is_c2_LgtCtrl = LgtCtrl_IN_TrqLim;

      /* DataTypeConversion: '<S10>/Data Type Conversion9' */
      /* Entry 'TrqLim': '<S99>:3' */
      Tra_stTrqLimWthTCU = 1U;
    }
  } else {
    /* DataTypeConversion: '<S10>/Data Type Conversion9' */
    Tra_stTrqLimWthTCU = 1U;

    /* During 'TrqLim': '<S99>:3' */
    if (((VehDa_stTraCurGear == 0) || (VehDa_stTraSelGear_mp !=
          VehDa_stTraCurGear) || (VehDa_stTraTrqLim_mp != 3) ||
         (VehDa_stTraEgd_mp != 1) || (VehDa_stTraSht_mp != 0) || (CoChs_stCdn !=
          2)) && (VehDa_vEgoSpd >= 3.0F)) {
      /* Transition: '<S99>:5' */
      LgtCtrl_DW.is_c2_LgtCtrl = LgtCtrl_IN_NoLim;

      /* DataTypeConversion: '<S10>/Data Type Conversion9' */
      /* Entry 'NoLim': '<S99>:1' */
      Tra_stTrqLimWthTCU = 0U;
    }
  }

  /* End of Chart: '<S10>/TrqLim' */

  /* Switch: '<S121>/Swt1' incorporates:
   *  Constant: '<S121>/Constant1'
   *  Constant: '<S121>/P1_C'
   *  Constant: '<S121>/P2_C'
   *  Constant: '<S121>/P3_C'
   *  Constant: '<S121>/P4_C'
   *  Constant: '<S121>/P5_C'
   *  Constant: '<S121>/VMC_Penable_C'
   *  Constant: '<S121>/VMC_facAccnGvnrKpCurr_C'
   *  Constant: '<S121>/VMC_facAccnGvnrKpNegCurr_C'
   *  Constant: '<S121>/VMC_facAccnGvnrKpPosCurr_C'
   *  Constant: '<S121>/VMC_vAccnGvnrPWinNeg_C'
   *  Constant: '<S121>/VMC_vAccnGvnrPWinPos_C'
   *  DataTypeConversion: '<S12>/Data Type Conversion6'
   *  Logic: '<S121>/Logical Operator1'
   *  RelationalOperator: '<S121>/Relational Operator2'
   *  Switch: '<S121>/Switch'
   *  Switch: '<S121>/Switch1'
   */
  if ((SpdPln_stTarType == 1.0F) && (VMC_Penable_C != 0)) {
    ACCS_stActvd = P1_C;
    rtb_A_iw = P2_C;
    rtb_Swt1_o_idx_2 = P3_C;
    LgtDstRel = P4_C;
    rtb_Swt1_o_idx_4 = P5_C;
  } else {
    if (Tra_stTrqLimWthTCU != 0) {
      /* Switch: '<S121>/Switch' incorporates:
       *  Constant: '<S121>/VMC_facAccnGvrnKpWthGo_C'
       */
      ACCS_stActvd = VMC_facAccnGvrnKpWthGo_C;

      /* Switch: '<S121>/Switch1' incorporates:
       *  Constant: '<S121>/VMC_facAccnGvrnKpWthGo_C'
       */
      rtb_A_iw = VMC_facAccnGvrnKpWthGo_C;
    } else {
      ACCS_stActvd = VMC_facAccnGvnrKpCurr_C;
      rtb_A_iw = VMC_facAccnGvnrKpPosCurr_C;
    }

    rtb_Swt1_o_idx_2 = VMC_facAccnGvnrKpNegCurr_C;
    LgtDstRel = VMC_vAccnGvnrPWinPos_C;
    rtb_Swt1_o_idx_4 = VMC_vAccnGvnrPWinNeg_C;
  }

  /* End of Switch: '<S121>/Swt1' */

  /* Switch: '<S126>/Swt' incorporates:
   *  Logic: '<S126>/L'
   *  Logic: '<S126>/L1'
   *  RelationalOperator: '<S126>/R'
   *  RelationalOperator: '<S126>/R1'
   *  RelationalOperator: '<S126>/R2'
   *  RelationalOperator: '<S126>/R3'
   *  Switch: '<S126>/Swt1'
   */
  if ((LgtDstRel < rtb_Swt1_o_idx_4) || ((VMC_vDif_mp <= LgtDstRel) &&
       (VMC_vDif_mp >= rtb_Swt1_o_idx_4))) {
    /* Switch: '<S126>/Swt' incorporates:
     *  Product: '<S126>/Product5'
     */
    VMC_aAccnGvnrP_mp = VMC_vDif_mp * ACCS_stActvd;
  } else if (VMC_vDif_mp > LgtDstRel) {
    /* Switch: '<S126>/Swt1' incorporates:
     *  Product: '<S126>/Product6'
     *  Product: '<S126>/Product8'
     *  Sum: '<S126>/Sum4'
     *  Sum: '<S126>/Sum5'
     *  Switch: '<S126>/Swt'
     */
    VMC_aAccnGvnrP_mp = (VMC_vDif_mp - LgtDstRel) * rtb_A_iw + ACCS_stActvd *
      LgtDstRel;
  } else {
    /* Switch: '<S126>/Swt' incorporates:
     *  Product: '<S126>/Product7'
     *  Product: '<S126>/Product9'
     *  Sum: '<S126>/Sum6'
     *  Sum: '<S126>/Sum7'
     *  Switch: '<S126>/Swt1'
     */
    VMC_aAccnGvnrP_mp = (VMC_vDif_mp - rtb_Swt1_o_idx_4) * rtb_Swt1_o_idx_2 +
      ACCS_stActvd * rtb_Swt1_o_idx_4;
  }

  /* End of Switch: '<S126>/Swt' */

  /* Switch: '<S122>/Switch' incorporates:
   *  Constant: '<S122>/Constant'
   *  Constant: '<S122>/Constant2'
   *  Constant: '<S122>/Constant3'
   *  RelationalOperator: '<S122>/Relational Operator'
   *  Sum: '<S122>/Add'
   *  UnitDelay: '<S122>/Unit Delay'
   */
  if (LgtCtrl_DW.UnitDelay_DSTATE_a < 10.0) {
    rtb_MinMax = 1.0 + LgtCtrl_DW.UnitDelay_DSTATE_a;
  } else {
    rtb_MinMax = 1.0;
  }

  /* End of Switch: '<S122>/Switch' */

  /* Outputs for Enabled SubSystem: '<S119>/difference' incorporates:
   *  EnablePort: '<S130>/Enable'
   */
  /* RelationalOperator: '<S122>/Relational Operator1' incorporates:
   *  Constant: '<S122>/Constant3'
   */
  if (rtb_MinMax == 1.0) {
    /* Gain: '<S130>/Gain' incorporates:
     *  Sum: '<S130>/Add4'
     *  UnitDelay: '<S130>/Unit Delay3'
     */
    ACCS_stActvd = (VMC_vDif_mp - LgtCtrl_DW.UnitDelay3_DSTATE_g) * 10.0F;

    /* Switch: '<S133>/Swt1' incorporates:
     *  Constant: '<S133>/C '
     *  Constant: '<S133>/Eng_swtTrqLimCmpFild_C'
     *  Logic: '<S133>/Logical Operator'
     *  UnitDelay: '<S133>/Unit Delay1'
     */
    if (LgtCtrl_DW.UnitDelay1_DSTATE_g != 0.0) {
      rtb_Sum6 = ((0.0 != 0.0) || (LgtCtrl_B.output != 0.0F));
    } else {
      rtb_Sum6 = 1.0;
    }

    /* End of Switch: '<S133>/Swt1' */

    /* Switch: '<S133>/Swt' */
    if (rtb_Sum6 != 0.0) {
      /* Switch: '<S133>/Swt' */
      VMC_ratioVDiff_mp = ACCS_stActvd;
    } else {
      /* Switch: '<S133>/Swt' incorporates:
       *  Constant: '<S133>/VMC_facSpdDTfild_C'
       *  Product: '<S133>/Product5'
       *  Sum: '<S133>/Sum1'
       *  Sum: '<S133>/Sum6'
       *  UnitDelay: '<S133>/Unit Delay4'
       *  UnitDelay: '<S133>/Unit Delay5'
       */
      VMC_ratioVDiff_mp = (VMC_ratioVDiff_mp - LgtCtrl_DW.UnitDelay4_DSTATE_b) *
        VMC_facSpdDTfild_C + LgtCtrl_DW.UnitDelay4_DSTATE_b;
    }

    /* End of Switch: '<S133>/Swt' */

    /* Update for UnitDelay: '<S130>/Unit Delay3' */
    LgtCtrl_DW.UnitDelay3_DSTATE_g = VMC_vDif_mp;

    /* Update for UnitDelay: '<S133>/Unit Delay4' */
    LgtCtrl_DW.UnitDelay4_DSTATE_b = ACCS_stActvd;

    /* Update for UnitDelay: '<S133>/Unit Delay1' incorporates:
     *  Constant: '<S133>/C1 '
     */
    LgtCtrl_DW.UnitDelay1_DSTATE_g = 1.0;
  }

  /* End of RelationalOperator: '<S122>/Relational Operator1' */
  /* End of Outputs for SubSystem: '<S119>/difference' */

  /* Switch: '<S127>/Swt' incorporates:
   *  Constant: '<S121>/VMC_facAccnGvnrKdCurr_C'
   *  Constant: '<S121>/VMC_vAccnGvnrDWinNeg_C'
   *  Constant: '<S121>/VMC_vAccnGvnrDWinPos_C'
   *  Logic: '<S127>/L'
   *  Logic: '<S127>/L1'
   *  Product: '<S127>/Product5'
   *  RelationalOperator: '<S127>/R'
   *  RelationalOperator: '<S127>/R1'
   *  RelationalOperator: '<S127>/R2'
   *  RelationalOperator: '<S127>/R3'
   *  Switch: '<S127>/Swt1'
   */
  if ((VMC_vAccnGvnrDWinPos_C < VMC_vAccnGvnrDWinNeg_C) || ((VMC_ratioVDiff_mp <=
        VMC_vAccnGvnrDWinPos_C) && (VMC_ratioVDiff_mp >= VMC_vAccnGvnrDWinNeg_C)))
  {
    ACCS_stActvd = VMC_ratioVDiff_mp * VMC_facAccnGvnrKdCurr_C;
  } else if (VMC_ratioVDiff_mp > VMC_vAccnGvnrDWinPos_C) {
    /* Switch: '<S127>/Swt1' incorporates:
     *  Constant: '<S121>/VMC_facAccnGvnrKdCurr_C'
     *  Constant: '<S121>/VMC_facAccnGvnrKdPosCurr_C'
     *  Product: '<S127>/Product6'
     *  Product: '<S127>/Product8'
     *  Sum: '<S127>/Sum4'
     *  Sum: '<S127>/Sum5'
     */
    ACCS_stActvd = (VMC_ratioVDiff_mp - VMC_vAccnGvnrDWinPos_C) *
      VMC_facAccnGvnrKdPosCurr_C + VMC_vAccnGvnrDWinPos_C *
      VMC_facAccnGvnrKdCurr_C;
  } else {
    /* Switch: '<S127>/Swt1' incorporates:
     *  Constant: '<S121>/VMC_facAccnGvnrKdCurr_C'
     *  Constant: '<S121>/VMC_facAccnGvnrKdNegCurr_C'
     *  Product: '<S127>/Product7'
     *  Product: '<S127>/Product9'
     *  Sum: '<S127>/Sum6'
     *  Sum: '<S127>/Sum7'
     */
    ACCS_stActvd = (VMC_ratioVDiff_mp - VMC_vAccnGvnrDWinNeg_C) *
      VMC_facAccnGvnrKdNegCurr_C + VMC_vAccnGvnrDWinNeg_C *
      VMC_facAccnGvnrKdCurr_C;
  }

  /* End of Switch: '<S127>/Swt' */

  /* Switch: '<S123>/Switch2' incorporates:
   *  Constant: '<S119>/VMC_aAccnDCmpLo_C'
   *  Constant: '<S119>/VMC_aAccnDCmpUp_C'
   *  RelationalOperator: '<S123>/LowerRelop1'
   *  RelationalOperator: '<S123>/UpperRelop'
   *  Switch: '<S123>/Switch'
   */
  if (ACCS_stActvd > VMC_aAccnDCmpUp_C) {
    /* Switch: '<S123>/Switch2' */
    VMC_aAccnGvnrD_mp = VMC_aAccnDCmpUp_C;
  } else if (ACCS_stActvd < VMC_aAccnDCmpLo_C) {
    /* Switch: '<S123>/Switch' incorporates:
     *  Constant: '<S119>/VMC_aAccnDCmpLo_C'
     *  Switch: '<S123>/Switch2'
     */
    VMC_aAccnGvnrD_mp = VMC_aAccnDCmpLo_C;
  } else {
    /* Switch: '<S123>/Switch2' incorporates:
     *  Switch: '<S123>/Switch'
     */
    VMC_aAccnGvnrD_mp = ACCS_stActvd;
  }

  /* End of Switch: '<S123>/Switch2' */

  /* Logic: '<S120>/L ' incorporates:
   *  Abs: '<S120>/Abs1'
   *  Constant: '<S120>/C3'
   *  Constant: '<S120>/Constant'
   *  Constant: '<S120>/Enable'
   *  Constant: '<S120>/VMC_swtAccnGvrnInil_C'
   *  Constant: '<S120>/VMC_vDifThd2I_C'
   *  Constant: '<S121>/VMC_swtAccnGvnrFctEna_C'
   *  RelationalOperator: '<S120>/L3'
   *  RelationalOperator: '<S120>/Relational Operator2'
   *  RelationalOperator: '<S120>/Relational Operator4'
   *  UnitDelay: '<S12>/Unit Delay'
   */
  VMC_stAccnGvnrIntglIni_mp = (rtb_L4_c || (0.0F != 0.0F) ||
    (VMC_swtAccnGvrnInil_C != 0) || rtb_RelationalOperator1 ||
    (VMC_swtAccnGvnrFctEna_C != 1.0F) || (fabsf(LgtCtrl_DW.UnitDelay_DSTATE_d) >
    VMC_vDifThd2I_C) || (SpdPln_stTarType == 15.0));

  /* DataTypeConversion: '<S14>/Data Type Conversion18' incorporates:
   *  Gain: '<S157>/Gain'
   *  Inport: '<Root>/VehDa_stSrcEngCtrl_mp'
   */
  VehDa_stSrcEngCtrl = (uint8_T)(((uint32_T)((uint8_T)128U) *
    VehDa_stSrcEngCtrl_mp) >> 7);

  /* Sum: '<S120>/Add' incorporates:
   *  DataTypeConversion: '<S12>/Data Type Conversion3'
   *  DataTypeConversion: '<S12>/Data Type Conversion4'
   */
  u = (int16_T)(VehDa_prcDrvrDmdTrq - VehDa_prcActuTrq);

  /* Abs: '<S120>/Abs' */
  if (u < 0) {
    u = (int16_T)-u;
  }

  /* End of Abs: '<S120>/Abs' */

  /* Logic: '<S120>/L9' incorporates:
   *  Constant: '<S120>/C2'
   *  Constant: '<S120>/C5'
   *  Constant: '<S120>/PowerCtrl'
   *  Constant: '<S120>/PowerCtrl1'
   *  Constant: '<S120>/VMC_facTrqLim_C'
   *  DataTypeConversion: '<S12>/Data Type Conversion5'
   *  DataTypeConversion: '<S12>/Data Type Conversion7'
   *  Logic: '<S120>/L6'
   *  RelationalOperator: '<S120>/L10'
   *  RelationalOperator: '<S120>/L4'
   *  RelationalOperator: '<S120>/Relational Operator'
   *  RelationalOperator: '<S120>/Relational Operator1'
   *  RelationalOperator: '<S120>/Relational Operator3'
   *  UnitDelay: '<S120>/Unit Delay'
   *  UnitDelay: '<S1>/Unit Delay5'
   */
  VMC_stAccnGvnrIntglFrz_mp = ((CoChs_stCdn == 6.0F) || ((VehDa_stSrcEngCtrl ==
    3.0F) && (u > VMC_facTrqLim_C) && (CoChs_stCdn == 2.0F) &&
    (Tra_stShiftWthSpdUp != 0)) || (CoChs_stStarting == 1.0F));

  /* Switch: '<S125>/Switch4' incorporates:
   *  Constant: '<S121>/Constant2'
   *  Constant: '<S121>/VMC_Ienable_C'
   *  Logic: '<S121>/Logical Operator2'
   *  RelationalOperator: '<S121>/Relational Operator3'
   *  Sum: '<S125>/Sum'
   *  Switch: '<S121>/Swt2'
   *  Switch: '<S125>/Switch3'
   *  UnitDelay: '<S125>/Unit Delay'
   */
  if (VMC_stAccnGvnrIntglIni_mp) {
    /* Switch: '<S125>/Swt5' incorporates:
     *  Constant: '<S119>/VMC_aAccnGvnrIntglIniVal_C'
     *  MinMax: '<S125>/MinMax4'
     *  RelationalOperator: '<S125>/R3'
     *  RelationalOperator: '<S125>/R4'
     *  Sum: '<S125>/A4'
     *  Switch: '<S125>/Swt6'
     *  UnitDelay: '<S125>/D2'
     */
    if (VMC_aAccnGvnrIntglIniVal_C > VMC_aAccnGvnrI_mp) {
      rtb_UnitDelay_lk = fmin(LgtCtrl_ConstB.P2 + VMC_aAccnGvnrI_mp,
        VMC_aAccnGvnrIntglIniVal_C);
    } else if (VMC_aAccnGvnrIntglIniVal_C < VMC_aAccnGvnrI_mp) {
      /* Switch: '<S125>/Swt6' incorporates:
       *  MinMax: '<S125>/MinMax3'
       *  Sum: '<S125>/A3'
       */
      rtb_UnitDelay_lk = fmax(VMC_aAccnGvnrI_mp + LgtCtrl_ConstB.P1,
        VMC_aAccnGvnrIntglIniVal_C);
    } else {
      /* Switch: '<S125>/Swt6' */
      rtb_UnitDelay_lk = VMC_aAccnGvnrI_mp;
    }

    /* End of Switch: '<S125>/Swt5' */
  } else {
    if (VMC_stAccnGvnrIntglFrz_mp) {
      /* Switch: '<S125>/Switch3' incorporates:
       *  Constant: '<S125>/zero'
       */
      rtb_UnitDelay_lk = 0.0;
    } else {
      if ((SpdPln_stTarType == 1.0F) && (VMC_Ienable_C != 0)) {
        /* Switch: '<S121>/Swt2' incorporates:
         *  Constant: '<S121>/I1_C'
         *  Constant: '<S121>/I2_C'
         *  Constant: '<S121>/I3_C'
         *  Constant: '<S121>/I4_C'
         *  Constant: '<S121>/I5_C'
         *  Switch: '<S125>/Switch3'
         */
        ACCS_stActvd = I1_C;
        rtb_A_iw = I2_C;
        rtb_Swt1_o_idx_2 = I3_C;
        LgtDstRel = I4_C;
        rtb_Swt1_o_idx_4 = I5_C;
      } else {
        /* Switch: '<S121>/Swt2' incorporates:
         *  Constant: '<S121>/VMC_facAccnGvnrKiCurr_C'
         *  Constant: '<S121>/VMC_facAccnGvnrKiNegCurr_C'
         *  Constant: '<S121>/VMC_facAccnGvnrKiPosCurr_C'
         *  Constant: '<S121>/VMC_vAccnGvnrIWinNeg_C'
         *  Constant: '<S121>/VMC_vAccnGvnrIWinPos_C'
         *  Switch: '<S125>/Switch3'
         */
        ACCS_stActvd = VMC_facAccnGvnrKiCurr_C;
        rtb_A_iw = VMC_facAccnGvnrKiPosCurr_C;
        rtb_Swt1_o_idx_2 = VMC_facAccnGvnrKiNegCurr_C;
        LgtDstRel = VMC_vAccnGvnrIWinPos_C;
        rtb_Swt1_o_idx_4 = VMC_vAccnGvnrIWinNeg_C;
      }

      /* Switch: '<S125>/Switch5' incorporates:
       *  RelationalOperator: '<S125>/R2'
       *  Switch: '<S125>/Switch3'
       */
      if (LgtDstRel >= rtb_Swt1_o_idx_4) {
        /* Switch: '<S125>/Switch1' incorporates:
         *  RelationalOperator: '<S125>/R'
         *  RelationalOperator: '<S125>/R1'
         *  Switch: '<S125>/Switch2'
         */
        if (VMC_vDif_mp > LgtDstRel) {
          ACCS_stActvd = rtb_A_iw;
        } else if (VMC_vDif_mp < rtb_Swt1_o_idx_4) {
          /* Switch: '<S125>/Switch2' */
          ACCS_stActvd = rtb_Swt1_o_idx_2;
        }

        /* End of Switch: '<S125>/Switch1' */
      }

      /* End of Switch: '<S125>/Switch5' */

      /* Switch: '<S125>/Switch3' incorporates:
       *  Constant: '<S119>/VMC_tiAccnGvnrIntglUpdCyc_C'
       *  Product: '<S125>/Product'
       *  Product: '<S125>/Product1'
       */
      rtb_UnitDelay_lk = VMC_vDif_mp * ACCS_stActvd *
        VMC_tiAccnGvnrIntglUpdCyc_C;
    }

    /* Switch: '<S125>/Switch6' incorporates:
     *  Constant: '<S121>/Constant2'
     *  Constant: '<S121>/VMC_Ienable_C'
     *  Constant: '<S125>/zero1'
     *  Constant: '<S125>/zero2'
     *  Logic: '<S121>/Logical Operator2'
     *  RelationalOperator: '<S121>/Relational Operator3'
     *  Switch: '<S121>/Swt2'
     *  Switch: '<S125>/Switch3'
     *  UnitDelay: '<S125>/D1'
     */
    if (CoChs_stStarting != 0) {
      rtb_Sum6 = 0.001;
    } else {
      rtb_Sum6 = 0.0;
    }

    /* End of Switch: '<S125>/Switch6' */
    rtb_UnitDelay_lk = (rtb_UnitDelay_lk + VMC_aAccnGvnrI_mp) + rtb_Sum6;
  }

  /* End of Switch: '<S125>/Switch4' */

  /* Switch: '<S131>/Swt' incorporates:
   *  Constant: '<S121>/VMC_aAccnGvnrIntglCtrlValLo_C'
   *  Constant: '<S121>/VMC_aAccnGvnrIntglCtrlValUp_C'
   *  RelationalOperator: '<S131>/R3'
   */
  if (VMC_aAccnGvnrIntglCtrlValUp_C < VMC_aAccnGvnrIntglCtrlValLo_C) {
    /* Switch: '<S131>/Swt' */
    VMC_aAccnGvnrI_mp = (real32_T)rtb_UnitDelay_lk;
  } else {
    /* Switch: '<S131>/Swt' incorporates:
     *  MinMax: '<S131>/MinMax'
     *  MinMax: '<S131>/MinMax1'
     */
    VMC_aAccnGvnrI_mp = (real32_T)fmin(fmax(VMC_aAccnGvnrIntglCtrlValLo_C,
      rtb_UnitDelay_lk), VMC_aAccnGvnrIntglCtrlValUp_C);
  }

  /* End of Switch: '<S131>/Swt' */

  /* Switch: '<S129>/Swt' incorporates:
   *  Constant: '<S121>/VMC_swtAccnGvnrFctEna_C'
   */
  if (VMC_swtAccnGvnrFctEna_C != 0) {
    /* Switch: '<S132>/Swt' incorporates:
     *  Constant: '<S129>/VMC_aAccnGvnrOutpLo_C'
     *  Constant: '<S129>/VMC_aAccnGvnrOutpUp_C'
     *  RelationalOperator: '<S132>/R3'
     */
    if (VMC_aAccnGvnrOutpUp_C < VMC_aAccnGvnrOutpLo_C) {
      /* Switch: '<S129>/Swt' incorporates:
       *  Sum: '<S129>/A'
       */
      VMC_Accnpid = (VMC_aAccnGvnrP_mp + VMC_aAccnGvnrD_mp) + VMC_aAccnGvnrI_mp;
    } else {
      /* Switch: '<S129>/Swt' incorporates:
       *  MinMax: '<S132>/MinMax'
       *  MinMax: '<S132>/MinMax1'
       *  Sum: '<S129>/A'
       */
      VMC_Accnpid = fminf(fmaxf(VMC_aAccnGvnrOutpLo_C, (VMC_aAccnGvnrP_mp +
        VMC_aAccnGvnrD_mp) + VMC_aAccnGvnrI_mp), VMC_aAccnGvnrOutpUp_C);
    }

    /* End of Switch: '<S132>/Swt' */
  } else {
    /* Switch: '<S129>/Swt' incorporates:
     *  Constant: '<S129>/C'
     */
    VMC_Accnpid = 0.0F;
  }

  /* End of Switch: '<S129>/Swt' */

  /* Sum: '<S129>/A1' */
  VMC_aReq = VMC_Accnpid + SpdPln_aTrgAcc;

  /* Product: '<S100>/P4' incorporates:
   *  Product: '<S100>/P3'
   */
  VDC_frAccnRes_mp = VMC_aReq * VehDa_mWght * VDC_facDrvTrmJ_mp;

  /* Gain: '<S154>/Gain' incorporates:
   *  Inport: '<Root>/VehDa_aEgoAcc_mp'
   */
  VehDa_aEgoAcc = 1.0F * VehDa_aEgoAcc_mp;

  /* Sum: '<S112>/aDif' */
  ACCS_stActvd = VMC_aReq - VehDa_aEgoAcc;

  /* Switch: '<S112>/Swt' incorporates:
   *  Constant: '<S112>/VDC_facAccnDifFild_C'
   *  Constant: '<S112>/VDC_swtAccnDifFild_C'
   *  Product: '<S112>/Product5'
   *  Sum: '<S112>/Sum1'
   *  Sum: '<S112>/Sum6'
   *  UnitDelay: '<S112>/Unit Delay4'
   *  UnitDelay: '<S112>/Unit Delay5'
   */
  if (((uint8_T)0U) != 0) {
    rtb_A_iw = ACCS_stActvd;
  } else {
    rtb_A_iw = (LgtCtrl_DW.UnitDelay5_DSTATE_l - LgtCtrl_DW.UnitDelay4_DSTATE_e)
      * 0.99F + LgtCtrl_DW.UnitDelay4_DSTATE_e;
  }

  /* End of Switch: '<S112>/Swt' */

  /* Switch: '<S115>/Swt' incorporates:
   *  Constant: '<S111>/VDC_aFrGvnrPWinNeg_C'
   *  Constant: '<S111>/VDC_aFrGvnrPWinPos_C'
   *  Constant: '<S111>/VDC_facFrGvnrKpCurr_C'
   *  Logic: '<S115>/L'
   *  Logic: '<S115>/L1'
   *  Product: '<S115>/Product5'
   *  RelationalOperator: '<S115>/R'
   *  RelationalOperator: '<S115>/R1'
   *  RelationalOperator: '<S115>/R3'
   *  Switch: '<S115>/Swt1'
   */
  if (LgtCtrl_ConstB.R2_h || ((rtb_A_iw <= 2.0F) && (rtb_A_iw >= (-2.0F)))) {
    rtb_Swt_lo = rtb_A_iw * 0.4F;
  } else if (rtb_A_iw > 2.0F) {
    /* Switch: '<S115>/Swt1' incorporates:
     *  Constant: '<S111>/VDC_facFrGvnrKpPosCurr_C'
     *  Product: '<S115>/Product6'
     *  Sum: '<S115>/Sum4'
     *  Sum: '<S115>/Sum5'
     */
    rtb_Swt_lo = (rtb_A_iw - 2.0F) * 0.2F + LgtCtrl_ConstB.Product8;
  } else {
    /* Switch: '<S115>/Swt1' incorporates:
     *  Constant: '<S111>/VDC_facFrGvnrKpNegCurr_C'
     *  Product: '<S115>/Product7'
     *  Sum: '<S115>/Sum6'
     *  Sum: '<S115>/Sum7'
     */
    rtb_Swt_lo = (rtb_A_iw - (-2.0F)) * 0.4F + LgtCtrl_ConstB.Product9;
  }

  /* End of Switch: '<S115>/Swt' */

  /* Sum: '<S113>/Sum1' incorporates:
   *  UnitDelay: '<S113>/Unit Delay1'
   */
  LgtDstRel = rtb_A_iw - LgtCtrl_DW.UnitDelay1_DSTATE_h;

  /* Product: '<S113>/Product' incorporates:
   *  Constant: '<S111>/VDC_facFrGvnrKdCurr_C'
   */
  rtb_Swt1_o_idx_4 = 0.0F * LgtDstRel;

  /* Switch: '<S113>/Swt2' incorporates:
   *  Constant: '<S109>/VDC_facFrGvnrDftlRatio_C'
   *  Constant: '<S109>/VDC_frFrGvnrDftlIniVal_C'
   *  Constant: '<S109>/VDC_frFrGvnrIntglIniVal_C'
   *  Constant: '<S110>/C7'
   *  Constant: '<S110>/C8'
   *  Constant: '<S110>/Normal'
   *  Constant: '<S111>/VDC_aFrGvnrDWinNeg_C'
   *  Constant: '<S111>/VDC_aFrGvnrDWinPos_C'
   *  DataTypeConversion: '<S11>/Data Type Conversion2'
   *  Logic: '<S110>/L '
   *  Logic: '<S110>/L14'
   *  Product: '<S113>/Product1'
   *  RelationalOperator: '<S110>/L1'
   *  RelationalOperator: '<S110>/L12'
   *  RelationalOperator: '<S110>/L13'
   *  RelationalOperator: '<S113>/R'
   *  RelationalOperator: '<S113>/R1'
   *  Sum: '<S113>/Sum'
   *  Sum: '<S114>/Sum'
   *  Switch: '<S113>/Swt'
   *  Switch: '<S113>/Swt1'
   *  Switch: '<S114>/Switch4'
   *  UnitDelay: '<S113>/Unit Delay'
   *  UnitDelay: '<S114>/Unit Delay'
   *  UnitDelay: '<S1>/Unit Delay7'
   */
  if (((CoChs_stCdn != 2.0) && (CoChs_stCdn != 3.0)) || (Sys_stADMd != 2.0) ||
      LgtCtrl_ConstB.L3) {
    LgtDstRel = 0.0F;
    rtb_Swt1_o_idx_4 = 0.0F;
  } else {
    if (LgtDstRel > 0.0F) {
      /* Switch: '<S113>/Swt' incorporates:
       *  Constant: '<S111>/VDC_aFrGvnrDWinPos_C'
       *  Constant: '<S111>/VDC_facFrGvnrKdPosCurr_C'
       *  Product: '<S113>/Product2'
       *  Sum: '<S113>/Sum2'
       *  Sum: '<S113>/Sum4'
       */
      rtb_Swt1_o_idx_4 += (LgtDstRel - 0.0F) * 0.0F;
    } else if (LgtDstRel < 0.0F) {
      /* Switch: '<S113>/Swt' incorporates:
       *  Constant: '<S111>/VDC_aFrGvnrDWinNeg_C'
       *  Constant: '<S111>/VDC_facFrGvnrKdNegCurr_C'
       *  Product: '<S113>/Product3'
       *  Sum: '<S113>/Sum3'
       *  Sum: '<S113>/Sum5'
       *  Switch: '<S113>/Swt1'
       */
      rtb_Swt1_o_idx_4 += (LgtDstRel - 0.0F) * 0.0F;
    }

    LgtDstRel = 0.0F * LgtCtrl_DW.UnitDelay_DSTATE_af + rtb_Swt1_o_idx_4;

    /* Switch: '<S114>/Switch3' incorporates:
     *  Constant: '<S109>/VDC_facFrGvnrDftlRatio_C'
     *  Constant: '<S109>/VDC_tiDynGvnrIntglUpdCyc_C'
     *  Constant: '<S110>/C1'
     *  Constant: '<S110>/PowerCtrl'
     *  Constant: '<S111>/VDC_aFrGvnrDWinNeg_C'
     *  Constant: '<S111>/VDC_aFrGvnrIWinPos_C'
     *  Constant: '<S114>/zero'
     *  DataTypeConversion: '<S11>/Data Type Conversion'
     *  Logic: '<S110>/L5'
     *  Product: '<S113>/Product1'
     *  Product: '<S114>/Product'
     *  Product: '<S114>/Product1'
     *  RelationalOperator: '<S110>/L2'
     *  RelationalOperator: '<S110>/L4'
     *  RelationalOperator: '<S113>/R1'
     *  RelationalOperator: '<S114>/R'
     *  Sum: '<S113>/Sum'
     *  Switch: '<S113>/Swt'
     *  Switch: '<S113>/Swt1'
     *  Switch: '<S114>/Switch1'
     *  Switch: '<S114>/Switch5'
     *  UnitDelay: '<S113>/Unit Delay'
     */
    if ((VehDa_stCluSwt != 0.0) && (CoChs_stCdn == 2.0)) {
      rtb_Swt1_o_idx_4 = 0.0F;
    } else {
      if (LgtCtrl_ConstB.R2) {
        /* Switch: '<S114>/Switch5' incorporates:
         *  Constant: '<S111>/VDC_facFrGvnrKiCurr_C'
         */
        rtb_Swt1_o_idx_2 = 0.03F;
      } else if (rtb_A_iw > 2.0F) {
        /* Switch: '<S114>/Switch1' incorporates:
         *  Constant: '<S111>/VDC_facFrGvnrKiPosCurr_C'
         *  Switch: '<S114>/Switch5'
         */
        rtb_Swt1_o_idx_2 = 0.015F;
      } else {
        /* Switch: '<S114>/Switch5' incorporates:
         *  Constant: '<S111>/VDC_facFrGvnrKiNegCurr_C'
         *  Switch: '<S114>/Switch1'
         *  Switch: '<S114>/Switch2'
         */
        rtb_Swt1_o_idx_2 = 0.03F;
      }

      rtb_Swt1_o_idx_4 = rtb_A_iw * rtb_Swt1_o_idx_2 * 0.01F;
    }

    /* End of Switch: '<S114>/Switch3' */
    rtb_Swt1_o_idx_4 += LgtCtrl_DW.UnitDelay_DSTATE_io;
  }

  /* End of Switch: '<S113>/Swt2' */

  /* Switch: '<S117>/Swt' incorporates:
   *  Constant: '<S111>/VDC_frFrGvnrIntglCtrlValLo_C'
   *  Constant: '<S111>/VDC_frFrGvnrIntglCtrlValUp_C'
   *  MinMax: '<S117>/MinMax'
   *  MinMax: '<S117>/MinMax1'
   */
  if (!LgtCtrl_ConstB.R3) {
    rtb_Swt1_o_idx_4 = fminf(fmaxf(-3.0F, rtb_Swt1_o_idx_4), 2.0F);
  }

  /* End of Switch: '<S117>/Swt' */

  /* Switch: '<S118>/Swt' */
  if (LgtCtrl_ConstB.R3_b) {
    /* Switch: '<S118>/Swt' incorporates:
     *  Sum: '<S116>/TrqFeedback'
     */
    VMC_frpid = (rtb_Swt_lo + LgtDstRel) + rtb_Swt1_o_idx_4;
  } else {
    /* Switch: '<S118>/Swt' incorporates:
     *  Constant: '<S116>/VDC_frFrGvnrOutpLo_C'
     *  Constant: '<S116>/VDC_frFrGvnrOutpUp_C'
     *  MinMax: '<S118>/MinMax'
     *  MinMax: '<S118>/MinMax1'
     *  Sum: '<S116>/TrqFeedback'
     */
    VMC_frpid = fminf(fmaxf(-3.0F, (rtb_Swt_lo + LgtDstRel) + rtb_Swt1_o_idx_4),
                      3.0F);
  }

  /* End of Switch: '<S118>/Swt' */

  /* Switch: '<S116>/Switch' incorporates:
   *  Constant: '<S111>/VDC_swtFrGvnrFctEna_C'
   */
  if (0.0F != 0.0F) {
    /* Switch: '<S116>/Switch' */
    VDC_frCmp_mp = VMC_frpid;
  } else {
    /* Switch: '<S116>/Switch' incorporates:
     *  Constant: '<S116>/C'
     */
    VDC_frCmp_mp = 0.0F;
  }

  /* End of Switch: '<S116>/Switch' */

  /* Sum: '<S104>/A' */
  VDC_frDrvForce = (((VDC_frRollngRes_mp + VDC_frAirDrag_mp) + VDC_frSlpRes_mp)
                    + VDC_frAccnRes_mp) + VDC_frCmp_mp;

  /* Product: '<S107>/delta rise limit' incorporates:
   *  Constant: '<S104>/VMC_dtEngFricAccr_C'
   *  SampleTimeMath: '<S107>/sample time'
   *
   * About '<S107>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_deltariselimit = VMC_dtEngFricAccr_C * 0.01F;

  /* DataTypeConversion: '<S14>/Data Type Conversion8' incorporates:
   *  Gain: '<S159>/Gain'
   *  Inport: '<Root>/VehDa_prcTrqEngNomFric_mp'
   */
  VehDa_prcTrqEngNomFric = (int16_T)((16384 * VehDa_prcTrqEngNomFric_mp) >> 14);

  /* Product: '<S106>/Divide' incorporates:
   *  Constant: '<S106>/VDC_rFinalRatio_C'
   *  Constant: '<S106>/VDC_rdTire_C'
   *  Constant: '<S106>/VDC_trqRef_C'
   *  DataTypeConversion: '<S11>/Data Type Conversion3'
   *  Gain: '<S106>/Gain'
   */
  VDC_frEngFric_mp = (real32_T)(20972 * VehDa_prcTrqEngNomFric) * 4.76837158E-7F
    * (VehDa_rTraCurGear * VDC_rFinalRatio_C) * VDC_trqRef_C / VDC_rdTire_C;

  /* Sum: '<S107>/Difference Inputs1' incorporates:
   *  Product: '<S104>/D1'
   *  UnitDelay: '<S107>/Delay Input2'
   *
   * Block description for '<S107>/Difference Inputs1':
   *
   *  Add in CPU
   *
   * Block description for '<S107>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_D1_i = VDC_frEngFric_mp / VehDa_mWght - LgtCtrl_DW.DelayInput2_DSTATE;

  /* Product: '<S107>/delta fall limit' incorporates:
   *  Constant: '<S104>/VMC_dtEngFricAccr_C'
   *  Gain: '<S104>/Gain'
   *  SampleTimeMath: '<S107>/sample time'
   *
   * About '<S107>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_deltafalllimit = (-1.0F) * VMC_dtEngFricAccr_C * 0.01F;

  /* Switch: '<S108>/Switch2' incorporates:
   *  RelationalOperator: '<S108>/LowerRelop1'
   *  RelationalOperator: '<S108>/UpperRelop'
   *  Switch: '<S108>/Switch'
   */
  if (rtb_D1_i > rtb_deltariselimit) {
    rtb_D1_i = rtb_deltariselimit;
  } else if (rtb_D1_i < rtb_deltafalllimit) {
    /* Switch: '<S108>/Switch' */
    rtb_D1_i = rtb_deltafalllimit;
  }

  /* End of Switch: '<S108>/Switch2' */

  /* Sum: '<S107>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S107>/Delay Input2'
   *
   * Block description for '<S107>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S107>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_deltariselimit = rtb_D1_i + LgtCtrl_DW.DelayInput2_DSTATE;

  /* Switch: '<S104>/Switch' incorporates:
   *  Constant: '<S104>/VDC_swtNomAccrCalc_C'
   */
  if (VDC_swtNomAccrCalc_C != 0) {
    /* Switch: '<S104>/Switch' incorporates:
     *  Product: '<S104>/D2'
     *  Sum: '<S104>/A1'
     */
    VDC_aReqNom = VDC_frDrvForce / VehDa_mWght + rtb_deltariselimit;
  } else {
    /* Switch: '<S104>/Switch' incorporates:
     *  Product: '<S104>/D2'
     */
    VDC_aReqNom = VDC_frDrvForce / VehDa_mWght;
  }

  /* End of Switch: '<S104>/Switch' */

  /* Switch: '<S45>/Switch' incorporates:
   *  Constant: '<S39>/CoChs_aHysLo_C'
   *  Constant: '<S45>/Constant7'
   *  Logic: '<S43>/Logical Operator5'
   *  RelationalOperator: '<S39>/R2'
   *  Switch: '<S45>/Switch1'
   */
  if (VDC_aReqNom >= (-0.2)) {
    rtb_Switch1_b = ((uint16_T)0U);
  } else {
    /* Sum: '<S45>/Add' incorporates:
     *  Constant: '<S45>/Constant3'
     *  UnitDelay: '<S43>/Unit Delay1'
     */
    rtb_Switch1_b = (uint16_T)((uint32_T)((uint16_T)1U) +
      LgtCtrl_DW.UnitDelay1_DSTATE_p);

    /* MinMax: '<S45>/MinMax' incorporates:
     *  Constant: '<S45>/Constant6'
     */
    if (rtb_Switch1_b > ((uint16_T)65530U)) {
      rtb_Switch1_b = ((uint16_T)65530U);
    }

    /* End of MinMax: '<S45>/MinMax' */
  }

  /* End of Switch: '<S45>/Switch' */

  /* RelationalOperator: '<S43>/Relational Operator2' incorporates:
   *  Constant: '<S43>/K_CMB_AccPosDur'
   */
  LogicalOperator2 = (rtb_Switch1_b >= 30.0);

  /* Switch: '<S44>/Switch' incorporates:
   *  Constant: '<S39>/CoChs_aHysHi_C'
   *  Constant: '<S44>/Constant7'
   *  Logic: '<S42>/Logical Operator5'
   *  RelationalOperator: '<S39>/R1'
   *  Switch: '<S44>/Switch1'
   */
  if (VDC_aReqNom < 0.0) {
    rtb_Switch1_i = ((uint16_T)0U);
  } else {
    /* Sum: '<S44>/Add' incorporates:
     *  Constant: '<S44>/Constant3'
     *  UnitDelay: '<S42>/Unit Delay1'
     */
    rtb_Switch1_i = (uint16_T)((uint32_T)((uint16_T)1U) +
      LgtCtrl_DW.UnitDelay1_DSTATE_pb);

    /* MinMax: '<S44>/MinMax' incorporates:
     *  Constant: '<S44>/Constant6'
     */
    if (rtb_Switch1_i > ((uint16_T)65530U)) {
      rtb_Switch1_i = ((uint16_T)65530U);
    }

    /* End of MinMax: '<S44>/MinMax' */
  }

  /* End of Switch: '<S44>/Switch' */

  /* RelationalOperator: '<S42>/Relational Operator2' incorporates:
   *  Constant: '<S42>/K_CMB_AccPosDur'
   */
  rtb_RelationalOperator1 = (rtb_Switch1_i >= 30.0);

  /* Sum: '<S39>/Add3' incorporates:
   *  Sum: '<S39>/Add1'
   */
  rtb_Swt1_o_idx_2 = VehDa_vEgoSpd - SpdPln_vTrgSpd;

  /* Chart: '<S39>/CoChs_stCtlSysCfm' incorporates:
   *  Constant: '<S39>/CoChs_aHysLo_C10'
   *  Constant: '<S39>/CoChs_aHysLo_C2'
   *  Constant: '<S39>/CoChs_aHysLo_C3'
   *  Constant: '<S39>/CoChs_aHysLo_C4'
   *  Constant: '<S39>/CoChs_aHysLo_C6'
   *  Constant: '<S39>/CoChs_aHysLo_C7'
   *  Constant: '<S39>/CoChs_aHysLo_C8'
   *  Constant: '<S39>/CoChs_aHysLo_C9'
   *  Logic: '<S39>/Logical Operator2'
   *  Logic: '<S39>/Logical Operator3'
   *  Logic: '<S39>/Logical Operator6'
   *  Logic: '<S39>/Logical Operator7'
   *  RelationalOperator: '<S39>/R10'
   *  RelationalOperator: '<S39>/R4'
   *  RelationalOperator: '<S39>/R5'
   *  RelationalOperator: '<S39>/R6'
   *  RelationalOperator: '<S39>/R8'
   *  RelationalOperator: '<S39>/R9'
   *  Sum: '<S39>/Add3'
   */
  /* Gateway: LgtCtrl/CoChs/CoChs_CtlSysCfm1/CoChs_stCtlSysCfm */
  /* During: LgtCtrl/CoChs/CoChs_CtlSysCfm1/CoChs_stCtlSysCfm */
  if (LgtCtrl_DW.is_active_c13_LgtCtrl == 0U) {
    /* Entry: LgtCtrl/CoChs/CoChs_CtlSysCfm1/CoChs_stCtlSysCfm */
    LgtCtrl_DW.is_active_c13_LgtCtrl = 1U;

    /* Entry Internal: LgtCtrl/CoChs/CoChs_CtlSysCfm1/CoChs_stCtlSysCfm */
    /* Transition: '<S41>:137' */
    LgtCtrl_DW.is_c13_LgtCtrl = LgtCtrl_IN_Slip;

    /* Entry 'Slip': '<S41>:159' */
    rtb_UnitDelay_lk = 0.0;
  } else {
    switch (LgtCtrl_DW.is_c13_LgtCtrl) {
     case LgtCtrl_IN_Braking:
      /* During 'Braking': '<S41>:161' */
      if (!LogicalOperator2) {
        /* Transition: '<S41>:165' */
        LgtCtrl_DW.is_c13_LgtCtrl = LgtCtrl_IN_Slip;

        /* Entry 'Slip': '<S41>:159' */
        rtb_UnitDelay_lk = 0.0;
      } else {
        rtb_UnitDelay_lk = 2.0;
      }
      break;

     case LgtCtrl_IN_Driving:
      /* During 'Driving': '<S41>:160' */
      if (!rtb_RelationalOperator1) {
        /* Transition: '<S41>:164' */
        LgtCtrl_DW.is_c13_LgtCtrl = LgtCtrl_IN_Slip;

        /* Entry 'Slip': '<S41>:159' */
        rtb_UnitDelay_lk = 0.0;
      } else {
        rtb_UnitDelay_lk = 1.0;
      }
      break;

     default:
      /* During 'Slip': '<S41>:159' */
      if (rtb_RelationalOperator1 && ((rtb_Swt1_o_idx_2 >= 0.0) ||
           (rtb_Swt1_o_idx_2 < (-4.0)) || (1.0 == 0.0) || (SpdPln_aTrgAcc >= 0.0)))
      {
        /* Transition: '<S41>:162' */
        LgtCtrl_DW.is_c13_LgtCtrl = LgtCtrl_IN_Driving;

        /* Entry 'Driving': '<S41>:160' */
        rtb_UnitDelay_lk = 1.0;
      } else if (LogicalOperator2 && ((rtb_Swt1_o_idx_2 <= 0.0) ||
                  (rtb_Swt1_o_idx_2 > 4.0) || (1.0 == 0.0) || (SpdPln_aTrgAcc <=
        0.0))) {
        /* Transition: '<S41>:163' */
        LgtCtrl_DW.is_c13_LgtCtrl = LgtCtrl_IN_Braking;

        /* Entry 'Braking': '<S41>:161' */
        rtb_UnitDelay_lk = 2.0;
      } else {
        rtb_UnitDelay_lk = 0.0;
      }
      break;
    }
  }

  /* End of Chart: '<S39>/CoChs_stCtlSysCfm' */

  /* RelationalOperator: '<S39>/R12' incorporates:
   *  Constant: '<S39>/CoChs_aHysHi_C2'
   */
  CoChs_stDrvReq_mp = (rtb_UnitDelay_lk == 1.0);

  /* RelationalOperator: '<S39>/R13' incorporates:
   *  Constant: '<S39>/CoChs_aHysHi_C3'
   */
  CoChs_stBrkReq_mp = (rtb_UnitDelay_lk == 2.0);

  /* RelationalOperator: '<S39>/R11' incorporates:
   *  Constant: '<S39>/CoChs_aHysHi_C1'
   */
  CoChs_stSlipReq_mp = (rtb_UnitDelay_lk == 0.0);

  /* Chart: '<S38>/ConCfm' incorporates:
   *  Constant: '<S38>/CoChs_stLgtCtrlEna_C'
   */
  /* Gateway: LgtCtrl/CoChs/CoChs_ConCfm/ConCfm */
  /* During: LgtCtrl/CoChs/CoChs_ConCfm/ConCfm */
  /* Entry Internal: LgtCtrl/CoChs/CoChs_ConCfm/ConCfm */
  /* Transition: '<S40>:137' */
  guard1 = false;
  if (CoChs_stLgtCtrlEna_C == 0) {
    /* Transition: '<S40>:139' */
    /* Transition: '<S40>:141' */
    CoChs_stCdn = 8U;

    /* Transition: '<S40>:142' */
    /* Transition: '<S40>:67' */
    /* Transition: '<S40>:68' */
    /* Transition: '<S40>:133' */
    /* Transition: '<S40>:134' */
    /* Transition: '<S40>:40' */
    /* Transition: '<S40>:96' */
    /* Transition: '<S40>:103' */
    /* Transition: '<S40>:110' */
    /* Transition: '<S40>:156' */
    /* Transition: '<S40>:41' */
    /* Transition: '<S40>:42' */
    /* Transition: '<S40>:43' */

    /* Transition: '<S40>:69' */
  } else if (CoChs_stClbEna_C == 1) {
    /* Transition: '<S40>:58' */
    if (CoChs_stActvTst_C == 1) {
      /* Transition: '<S40>:55' */
      /* Transition: '<S40>:64' */
      CoChs_stCdn = 9U;

      /* Transition: '<S40>:67' */
      /* Transition: '<S40>:68' */
      /* Transition: '<S40>:133' */
      /* Transition: '<S40>:134' */
      /* Transition: '<S40>:40' */
      /* Transition: '<S40>:96' */
      /* Transition: '<S40>:103' */
      /* Transition: '<S40>:110' */
      /* Transition: '<S40>:156' */
      /* Transition: '<S40>:41' */
      /* Transition: '<S40>:42' */
      /* Transition: '<S40>:43' */

      /* Transition: '<S40>:60' */
    } else if (CoChs_stActvTst_C == 2) {
      /* Transition: '<S40>:62' */
      /* Transition: '<S40>:66' */
      CoChs_stCdn = 10U;

      /* Transition: '<S40>:68' */
      /* Transition: '<S40>:133' */
      /* Transition: '<S40>:134' */
      /* Transition: '<S40>:40' */
      /* Transition: '<S40>:96' */
      /* Transition: '<S40>:103' */
      /* Transition: '<S40>:110' */
      /* Transition: '<S40>:156' */
      /* Transition: '<S40>:41' */
      /* Transition: '<S40>:42' */
      /* Transition: '<S40>:43' */

      /* Transition: '<S40>:118' */
    } else if (CoChs_stActvTst_C == 3) {
      /* Transition: '<S40>:120' */
      /* Transition: '<S40>:122' */
      CoChs_stCdn = 11U;

      /* Transition: '<S40>:133' */
      /* Transition: '<S40>:134' */
      /* Transition: '<S40>:40' */
      /* Transition: '<S40>:96' */
      /* Transition: '<S40>:103' */
      /* Transition: '<S40>:110' */
      /* Transition: '<S40>:156' */
      /* Transition: '<S40>:41' */
      /* Transition: '<S40>:42' */
      /* Transition: '<S40>:43' */

      /* Transition: '<S40>:128' */
    } else if (CoChs_stActvTst_C == 7) {
      /* Transition: '<S40>:130' */
      /* Transition: '<S40>:132' */
      CoChs_stCdn = 7U;

      /* Transition: '<S40>:134' */
      /* Transition: '<S40>:40' */
      /* Transition: '<S40>:96' */
      /* Transition: '<S40>:103' */
      /* Transition: '<S40>:110' */
      /* Transition: '<S40>:156' */
      /* Transition: '<S40>:41' */
      /* Transition: '<S40>:42' */
      /* Transition: '<S40>:43' */
    } else {
      /* Transition: '<S40>:135' */
      /* Transition: '<S40>:12' */
      CoChs_stCdn = 0U;

      /* Transition: '<S40>:40' */
      /* Transition: '<S40>:96' */
      /* Transition: '<S40>:103' */
      /* Transition: '<S40>:110' */
      /* Transition: '<S40>:156' */
      /* Transition: '<S40>:41' */
      /* Transition: '<S40>:42' */
      /* Transition: '<S40>:43' */
    }

    /* Transition: '<S40>:73' */
  } else if ((Sys_stADMd != 3) || (BhvCrdn_numBhvID == 15) || (BhvCrdn_numBhvID ==
              0) || (BhvCrdn_numBhvID == 40)) {
    /* Transition: '<S40>:4' */
    /* Transition: '<S40>:12' */
    CoChs_stCdn = 0U;

    /* Transition: '<S40>:40' */
    /* Transition: '<S40>:96' */
    /* Transition: '<S40>:103' */
    /* Transition: '<S40>:110' */
    /* Transition: '<S40>:156' */
    /* Transition: '<S40>:41' */
    /* Transition: '<S40>:42' */
    /* Transition: '<S40>:43' */

    /* Transition: '<S40>:6' */
  } else if ((BhvCrdn_numBhvID == 8) && (LgtCtrl_DW.EmgBrkClsCtrl == 0)) {
    /* Transition: '<S40>:8' */
    /* Transition: '<S40>:14' */
    CoChs_stCdn = 1U;

    /* Transition: '<S40>:96' */
    /* Transition: '<S40>:103' */
    /* Transition: '<S40>:110' */
    /* Transition: '<S40>:156' */
    /* Transition: '<S40>:41' */
    /* Transition: '<S40>:42' */
    /* Transition: '<S40>:43' */

    /* Transition: '<S40>:91' */
  } else if (BhvCrdn_numBhvID == 5) {
    /* Transition: '<S40>:93' */
    if (CoChs_vIdl_C > 0.0F) {
      /* Transition: '<S40>:144' */
      /* Transition: '<S40>:95' */
      CoChs_stCdn = 4U;

      /* Transition: '<S40>:103' */
      /* Transition: '<S40>:110' */
      /* Transition: '<S40>:156' */
      /* Transition: '<S40>:41' */
      /* Transition: '<S40>:42' */
      /* Transition: '<S40>:43' */
    } else {
      /* Transition: '<S40>:146' */
      guard1 = true;
    }

    /* Transition: '<S40>:98' */
  } else if (BhvCrdn_numBhvID == 11) {
    /* Transition: '<S40>:100' */
    /* Transition: '<S40>:102' */
    CoChs_stCdn = 6U;

    /* Transition: '<S40>:110' */
    /* Transition: '<S40>:156' */
    /* Transition: '<S40>:41' */
    /* Transition: '<S40>:42' */
    /* Transition: '<S40>:43' */
  } else if ((BhvCrdn_numBhvID == 6) || (BhvCrdn_numBhvID == 7) ||
             ((VehDa_vEgoSpd > CoChs_vIdl_C) || (SpdPln_vTrgSpd * 3.6 >
               CoChs_vIdl_C))) {
    /* Transition: '<S40>:124' */
    /* Transition: '<S40>:24' */
    if (SpdPln_stReleaseThrottle || CoChs_stSlipReq_mp) {
      /* Transition: '<S40>:154' */
      /* Transition: '<S40>:160' */
      CoChs_stCdn = 12U;

      /* Transition: '<S40>:41' */
      /* Transition: '<S40>:42' */
      /* Transition: '<S40>:43' */
    } else {
      /* Transition: '<S40>:161' */
      guard1 = true;
    }
  } else {
    /* Transition: '<S40>:105' */
    /* Transition: '<S40>:107' */
    /* Transition: '<S40>:109' */
    CoChs_stCdn = 5U;

    /* Transition: '<S40>:156' */
    /* Transition: '<S40>:41' */
    /* Transition: '<S40>:42' */
    /* Transition: '<S40>:43' */
  }

  if (guard1) {
    if (CoChs_stDrvReq_mp) {
      /* Transition: '<S40>:28' */
      /* Transition: '<S40>:30' */
      CoChs_stCdn = 2U;

      /* Transition: '<S40>:42' */
      /* Transition: '<S40>:43' */

      /* Transition: '<S40>:166' */
    } else if (CoChs_stBrkReq_mp) {
      /* Transition: '<S40>:36' */
      /* Transition: '<S40>:34' */
      CoChs_stCdn = 3U;

      /* Transition: '<S40>:43' */
    } else {
      /* Transition: '<S40>:10' */
      /* Transition: '<S40>:113' */
    }
  }

  /* End of Chart: '<S38>/ConCfm' */

  /* Logic: '<S4>/Logical Operator' incorporates:
   *  Constant: '<S4>/Constant'
   *  Constant: '<S4>/Constant1'
   *  RelationalOperator: '<S4>/Relational Operator'
   *  RelationalOperator: '<S4>/Relational Operator1'
   *  UnitDelay: '<S4>/Unit Delay'
   */
  /* Transition: '<S40>:44' */
  LogicalOperator2 = ((LgtCtrl_B.DRMC_stMain == 3.0) || (LgtCtrl_B.DRMC_stMain ==
    4.0));

  /* Gain: '<S48>/Gain' */
  rtb_deltafalllimit = 0.277777791F * VehDa_vEgoSpd;

  /* Sum: '<S48>/A2' incorporates:
   *  Constant: '<S48>/C1'
   *  Constant: '<S4>/DRMC_aReqFf_C'
   *  Constant: '<S4>/DRMC_tiPBU_C'
   *  Product: '<S48>/D1'
   *  Product: '<S48>/P3'
   */
  rtb_D1_i = DRMC_aReqFf_C / 2.0F * DRMC_tiPBU_C + rtb_deltafalllimit;

  /* Switch: '<S48>/Switch' incorporates:
   *  Constant: '<S48>/C'
   *  Constant: '<S48>/C2'
   *  Constant: '<S4>/DRMC_aReqFf_C'
   *  Constant: '<S4>/DRMC_tiBrkResp_C'
   *  Constant: '<S4>/DRMC_tiPBU_C'
   *  Gain: '<S48>/Gain2'
   *  Product: '<S48>/D'
   *  Product: '<S48>/D2'
   *  Product: '<S48>/D3'
   *  Product: '<S48>/P'
   *  Product: '<S48>/P1'
   *  Product: '<S48>/P2'
   *  Product: '<S48>/P4'
   *  Sum: '<S48>/A'
   *  Sum: '<S48>/A1'
   *  Sum: '<S48>/A3'
   *  UnitDelay: '<S48>/Unit Delay'
   */
  if (LogicalOperator2) {
    rtb_deltafalllimit = LgtCtrl_DW.UnitDelay_DSTATE_du;
  } else {
    rtb_deltafalllimit = rtb_D1_i * rtb_D1_i / 2.0F / DRMC_aReqFf_C * (-1.0F) +
      ((DRMC_tiBrkResp_C + DRMC_tiPBU_C) * rtb_deltafalllimit + DRMC_tiPBU_C *
       DRMC_tiPBU_C * (DRMC_aReqFf_C / 6.0F));
  }

  /* End of Switch: '<S48>/Switch' */

  /* Switch: '<S147>/Switch' incorporates:
   *  Constant: '<S147>/CoChs_vIdl_C'
   *  Constant: '<S147>/Constant'
   *  Constant: '<S147>/Constant1'
   *  Constant: '<S147>/Constant2'
   *  Constant: '<S147>/Constant3'
   *  Constant: '<S14>/ACC_RXFlag'
   *  Inport: '<Root>/BhvCrdn_numBhvID_mp'
   *  Logic: '<S147>/Logical Operator'
   *  Logic: '<S147>/Logical Operator1'
   *  RelationalOperator: '<S147>/Relational Operator'
   *  RelationalOperator: '<S147>/Relational Operator1'
   *  RelationalOperator: '<S147>/Relational Operator2'
   *  RelationalOperator: '<S147>/Relational Operator3'
   *  Switch: '<S147>/Switch1'
   *  UnitDelay: '<S1>/Unit Delay4'
   */
  if (ACC_RXFlag != 0) {
    rtb_Swt1_o_idx_2 = LgtCtrl_DW.UnitDelay4_DSTATE_j;
  } else if ((CoChs_vIdl_C < 0.0) && ((BhvCrdn_numBhvID_mp == 5.0) ||
              (BhvCrdn_numBhvID_mp == 6.0) || (BhvCrdn_numBhvID_mp == 7.0))) {
    /* Switch: '<S147>/Switch1' incorporates:
     *  Constant: '<S147>/Bhv_ACC'
     */
    rtb_Swt1_o_idx_2 = 2.0F;
  } else {
    rtb_Swt1_o_idx_2 = BhvCrdn_numBhvID_mp;
  }

  /* End of Switch: '<S147>/Switch' */

  /* DataTypeConversion: '<S14>/Data Type Conversion20' */
  rtb_Swt1_o_idx_2 = fmodf(floorf(rtb_Swt1_o_idx_2), 256.0F);

  /* DataTypeConversion: '<S14>/Data Type Conversion20' */
  BhvCrdn_numBhvID = (uint8_T)(rtb_Swt1_o_idx_2 < 0.0F ? (int32_T)(uint8_T)
    -(int8_T)(uint8_T)-rtb_Swt1_o_idx_2 : (int32_T)(uint8_T)rtb_Swt1_o_idx_2);

  /* Chart: '<S4>/DRMC_MainState' incorporates:
   *  Constant: '<S4>/DRMC_lReqMin_C'
   *  Constant: '<S4>/DRMC_lTrg_C'
   *  Constant: '<S4>/DRMC_tiBrkResp_C'
   *  Constant: '<S4>/DRMC_tiCyc_C'
   *  Constant: '<S4>/DRMC_tiPBU_C'
   *  Constant: '<S4>/DRMC_tiPrepMax_C'
   *  DataTypeConversion: '<S4>/Data Type Conversion'
   *  DataTypeConversion: '<S4>/Data Type Conversion1'
   *  DataTypeConversion: '<S4>/Data Type Conversion2'
   *  Inport: '<Root>/DRMC_lCurr'
   */
  /* Gateway: LgtCtrl/DRMC/DRMC_MainState */
  /* During: LgtCtrl/DRMC/DRMC_MainState */
  if (LgtCtrl_DW.is_active_c15_LgtCtrl == 0U) {
    /* Entry: LgtCtrl/DRMC/DRMC_MainState */
    LgtCtrl_DW.is_active_c15_LgtCtrl = 1U;

    /* Entry Internal: LgtCtrl/DRMC/DRMC_MainState */
    /* Transition: '<S49>:6' */
    LgtCtrl_DW.is_c15_LgtCtrl = LgtCtrl_IN_Off_n;

    /* Entry 'Off': '<S49>:4' */
    LgtCtrl_B.DRMC_stMain = 0U;
  } else {
    switch (LgtCtrl_DW.is_c15_LgtCtrl) {
     case LgtCtrl_IN_Fault:
      LgtCtrl_B.DRMC_stMain = 25U;

      /* During 'Fault': '<S49>:5' */
      if ((Sys_stADMd != 3) || (BhvCrdn_numBhvID != 7)) {
        /* Transition: '<S49>:22' */
        LgtCtrl_DW.is_c15_LgtCtrl = LgtCtrl_IN_Off_n;

        /* Entry 'Off': '<S49>:4' */
        LgtCtrl_B.DRMC_stMain = 0U;
      }
      break;

     case LgtCtrl_IN_Normal:
      /* During 'Normal': '<S49>:16' */
      if ((Sys_stADMd != 3) || (BhvCrdn_numBhvID != 7)) {
        /* Transition: '<S49>:17' */
        /* Exit Internal 'Normal': '<S49>:16' */
        /* Exit Internal 'Actived': '<S49>:51' */
        LgtCtrl_DW.is_Actived = 0;
        LgtCtrl_DW.is_Normal = 0;
        LgtCtrl_DW.is_c15_LgtCtrl = LgtCtrl_IN_Off_n;

        /* Entry 'Off': '<S49>:4' */
        LgtCtrl_B.DRMC_stMain = 0U;
      } else {
        switch (LgtCtrl_DW.is_Normal) {
         case LgtCtrl_IN_Actived:
          /* During 'Actived': '<S49>:51' */
          switch (LgtCtrl_DW.is_Actived) {
           case LgtCtrl_IN_KeepPressure:
            LgtCtrl_B.DRMC_stMain = 4U;

            /* During 'KeepPressure': '<S49>:11' */
            if (VehDa_vEgoSpd <= 0.001) {
              /* Transition: '<S49>:15' */
              if ((DRMC_lCurr <= DRMC_lTrg_C) && (DRMC_lCurr >= -DRMC_lTrg_C)) {
                /* Transition: '<S49>:40' */
                LgtCtrl_DW.is_Actived = 0;
                LgtCtrl_DW.is_Normal = LgtCtrl_IN_Finished;

                /* Entry 'Finished': '<S49>:3' */
                LgtCtrl_B.DRMC_stMain = 5U;
              } else {
                /* Transition: '<S49>:39' */
                LgtCtrl_DW.is_Actived = 0;
                LgtCtrl_DW.is_Normal = 0;
                LgtCtrl_DW.is_c15_LgtCtrl = LgtCtrl_IN_Fault;

                /* Entry 'Fault': '<S49>:5' */
                LgtCtrl_B.DRMC_stMain = 25U;
              }
            }
            break;

           case LgtCtrl_IN_PressureBuildUp:
            LgtCtrl_B.DRMC_stMain = 3U;

            /* During 'PressureBuildUp': '<S49>:10' */
            if (LgtCtrl_DW.DRMC_ctPUB_mp * DRMC_tiCyc_C > DRMC_tiPBU_C +
                DRMC_tiBrkResp_C) {
              /* Transition: '<S49>:13' */
              LgtCtrl_DW.DRMC_ctPUB_mp = 0.0F;
              LgtCtrl_DW.is_Actived = LgtCtrl_IN_KeepPressure;

              /* Entry 'KeepPressure': '<S49>:11' */
              LgtCtrl_B.DRMC_stMain = 4U;
            } else {
              LgtCtrl_DW.DRMC_ctPUB_mp++;
            }
            break;

           default:
            LgtCtrl_B.DRMC_stMain = 2U;

            /* During 'SpdAdjmt': '<S49>:9' */
            if (DRMC_lCurr <= rtb_deltafalllimit) {
              /* Transition: '<S49>:12' */
              LgtCtrl_DW.is_Actived = LgtCtrl_IN_PressureBuildUp;

              /* Entry 'PressureBuildUp': '<S49>:10' */
              LgtCtrl_B.DRMC_stMain = 3U;
              LgtCtrl_DW.DRMC_ctPUB_mp = 0.0F;
            }
            break;
          }
          break;

         case LgtCtrl_IN_Finished:
          LgtCtrl_B.DRMC_stMain = 5U;

          /* During 'Finished': '<S49>:3' */
          break;

         default:
          /* During 'Prepared': '<S49>:1' */
          if (LgtCtrl_DW.DRMC_ctPrep_mp * DRMC_tiCyc_C >= DRMC_tiPrepMax_C) {
            /* Transition: '<S49>:42' */
            LgtCtrl_DW.DRMC_ctPrep_mp = 0.0F;
            if (VehDa_stTraCurGear == 1) {
              /* Transition: '<S49>:8' */
              LgtCtrl_DW.is_Normal = LgtCtrl_IN_Actived;
              LgtCtrl_DW.is_Actived = LgtCtrl_IN_SpdAdjmt;

              /* Entry 'SpdAdjmt': '<S49>:9' */
              LgtCtrl_B.DRMC_stMain = 2U;
            } else {
              /* Transition: '<S49>:44' */
              LgtCtrl_DW.is_Normal = 0;
              LgtCtrl_DW.is_c15_LgtCtrl = LgtCtrl_IN_Fault;

              /* Entry 'Fault': '<S49>:5' */
              LgtCtrl_B.DRMC_stMain = 25U;
            }
          } else {
            LgtCtrl_DW.DRMC_ctPrep_mp++;
          }
          break;
        }
      }
      break;

     default:
      LgtCtrl_B.DRMC_stMain = 0U;

      /* During 'Off': '<S49>:4' */
      if ((Sys_stADMd == 3) && (BhvCrdn_numBhvID == 7)) {
        /* Transition: '<S49>:7' */
        if ((VehDa_vEgoSpd > 0.5F) || (DRMC_lCurr < DRMC_lReqMin_C)) {
          /* Transition: '<S49>:19' */
          LgtCtrl_DW.is_c15_LgtCtrl = LgtCtrl_IN_Fault;

          /* Entry 'Fault': '<S49>:5' */
          LgtCtrl_B.DRMC_stMain = 25U;
        } else {
          /* Transition: '<S49>:21' */
          LgtCtrl_DW.is_c15_LgtCtrl = LgtCtrl_IN_Normal;
          LgtCtrl_DW.is_Normal = LgtCtrl_IN_Prepared;

          /* Entry 'Prepared': '<S49>:1' */
          LgtCtrl_DW.DRMC_ctPrep_mp = 0.0F;
        }
      }
      break;
    }
  }

  /* End of Chart: '<S4>/DRMC_MainState' */

  /* Switch: '<S48>/Switch1' incorporates:
   *  Gain: '<S48>/Gain1'
   *  UnitDelay: '<S48>/Unit Delay1'
   */
  if (LogicalOperator2) {
    rtb_Switch1 = LgtCtrl_DW.UnitDelay1_DSTATE_nl;
  } else {
    rtb_Switch1 = 3.6F * rtb_D1_i;
  }

  /* End of Switch: '<S48>/Switch1' */

  /* Chart: '<S50>/SpdRamp' incorporates:
   *  Constant: '<S4>/DRMC_aReqFf_C'
   *  Constant: '<S4>/DRMC_tiCyc_C'
   *  Inport: '<Root>/DRMC_lCurr'
   */
  /* Gateway: LgtCtrl/DRMC/FbCalc/SpdRamp */
  /* During: LgtCtrl/DRMC/FbCalc/SpdRamp */
  if (LgtCtrl_DW.is_active_c17_LgtCtrl == 0U) {
    /* Entry: LgtCtrl/DRMC/FbCalc/SpdRamp */
    LgtCtrl_DW.is_active_c17_LgtCtrl = 1U;

    /* Entry Internal: LgtCtrl/DRMC/FbCalc/SpdRamp */
    /* Transition: '<S54>:8' */
    LgtCtrl_DW.is_c17_LgtCtrl = LgtCtrl_IN_Initial;

    /* Entry 'Initial': '<S54>:1' */
    LgtCtrl_B.DRMC_vDes_mp1 = VehDa_vEgoSpd;
    LgtCtrl_B.DRMC_lDes_mp3 = DRMC_lCurr;
  } else {
    switch (LgtCtrl_DW.is_c17_LgtCtrl) {
     case LgtCtrl_IN_ClsLp:
      /* During 'ClsLp': '<S54>:2' */
      rtb_UnitDelay_lk = DRMC_tiCyc_C * DRMC_aReqFf_C * 3.6 +
        LgtCtrl_B.DRMC_vDes_mp1;
      if ((rtb_UnitDelay_lk <= 0.0) || (LgtCtrl_B.DRMC_stMain == 5)) {
        /* Transition: '<S54>:5' */
        LgtCtrl_DW.is_c17_LgtCtrl = LgtCtrl_IN_Finish;

        /* Entry 'Finish': '<S54>:4' */
        LgtCtrl_B.DRMC_vDes_mp1 = 0.0F;
        LgtCtrl_B.DRMC_lDes_mp3 = 0.0F;
      } else if (LgtCtrl_B.DRMC_stMain != 4) {
        /* Transition: '<S54>:6' */
        LgtCtrl_DW.is_c17_LgtCtrl = LgtCtrl_IN_Initial;

        /* Entry 'Initial': '<S54>:1' */
        LgtCtrl_B.DRMC_vDes_mp1 = VehDa_vEgoSpd;
        LgtCtrl_B.DRMC_lDes_mp3 = DRMC_lCurr;
      } else {
        LgtCtrl_B.DRMC_vDes_mp1 = (real32_T)rtb_UnitDelay_lk;
        LgtCtrl_B.DRMC_lDes_mp3 = (real32_T)(-LgtCtrl_B.DRMC_vDes_mp1 *
          LgtCtrl_B.DRMC_vDes_mp1 / 2.0F / DRMC_aReqFf_C / 3.6 / 3.6);
      }
      break;

     case LgtCtrl_IN_Finish:
      /* During 'Finish': '<S54>:4' */
      if ((LgtCtrl_B.DRMC_stMain != 5) && (LgtCtrl_B.DRMC_stMain != 4)) {
        /* Transition: '<S54>:7' */
        LgtCtrl_DW.is_c17_LgtCtrl = LgtCtrl_IN_Initial;

        /* Entry 'Initial': '<S54>:1' */
        LgtCtrl_B.DRMC_vDes_mp1 = VehDa_vEgoSpd;
        LgtCtrl_B.DRMC_lDes_mp3 = DRMC_lCurr;
      }
      break;

     default:
      /* During 'Initial': '<S54>:1' */
      if (LgtCtrl_B.DRMC_stMain == 4) {
        /* Transition: '<S54>:3' */
        LgtCtrl_DW.is_c17_LgtCtrl = LgtCtrl_IN_ClsLp;

        /* Entry 'ClsLp': '<S54>:2' */
        LgtCtrl_B.DRMC_vDes_mp1 = rtb_Switch1;
        LgtCtrl_B.DRMC_lDes_mp3 = (real32_T)(-rtb_Switch1 * rtb_Switch1 / 2.0F /
          DRMC_aReqFf_C / 3.6 / 3.6);
      } else {
        LgtCtrl_B.DRMC_vDes_mp1 = VehDa_vEgoSpd;
        LgtCtrl_B.DRMC_lDes_mp3 = DRMC_lCurr;
      }
      break;
    }
  }

  /* End of Chart: '<S50>/SpdRamp' */

  /* Chart: '<S50>/S_V' incorporates:
   *  Constant: '<S4>/DRMC_aReqFf_C'
   *  Inport: '<Root>/DRMC_lCurr'
   */
  /* Gateway: LgtCtrl/DRMC/FbCalc/S_V */
  /* During: LgtCtrl/DRMC/FbCalc/S_V */
  if (LgtCtrl_DW.is_active_c16_LgtCtrl == 0U) {
    /* Entry: LgtCtrl/DRMC/FbCalc/S_V */
    LgtCtrl_DW.is_active_c16_LgtCtrl = 1U;

    /* Entry Internal: LgtCtrl/DRMC/FbCalc/S_V */
    /* Transition: '<S53>:8' */
    LgtCtrl_DW.is_c16_LgtCtrl = LgtCtrl_IN_Initial;

    /* Entry 'Initial': '<S53>:1' */
    LgtCtrl_B.DRMC_vDes_mp2 = VehDa_vEgoSpd;
  } else {
    switch (LgtCtrl_DW.is_c16_LgtCtrl) {
     case LgtCtrl_IN_ClsLp:
      /* During 'ClsLp': '<S53>:2' */
      if ((DRMC_lCurr <= 0.0F) || (LgtCtrl_B.DRMC_stMain == 5)) {
        /* Transition: '<S53>:5' */
        LgtCtrl_DW.is_c16_LgtCtrl = LgtCtrl_IN_Finish;

        /* Entry 'Finish': '<S53>:4' */
        LgtCtrl_B.DRMC_vDes_mp2 = 0.0F;
      } else if (LgtCtrl_B.DRMC_stMain != 4) {
        /* Transition: '<S53>:6' */
        LgtCtrl_DW.is_c16_LgtCtrl = LgtCtrl_IN_Initial;

        /* Entry 'Initial': '<S53>:1' */
        LgtCtrl_B.DRMC_vDes_mp2 = VehDa_vEgoSpd;
      } else {
        LgtCtrl_B.DRMC_vDes_mp2 = (real32_T)(sqrtf(-2.0F * DRMC_aReqFf_C *
          DRMC_lCurr) * 3.6);
      }
      break;

     case LgtCtrl_IN_Finish:
      /* During 'Finish': '<S53>:4' */
      if ((LgtCtrl_B.DRMC_stMain != 5) && (LgtCtrl_B.DRMC_stMain != 4)) {
        /* Transition: '<S53>:7' */
        LgtCtrl_DW.is_c16_LgtCtrl = LgtCtrl_IN_Initial;

        /* Entry 'Initial': '<S53>:1' */
        LgtCtrl_B.DRMC_vDes_mp2 = VehDa_vEgoSpd;
      }
      break;

     default:
      /* During 'Initial': '<S53>:1' */
      if (LgtCtrl_B.DRMC_stMain == 4) {
        /* Transition: '<S53>:3' */
        LgtCtrl_DW.is_c16_LgtCtrl = LgtCtrl_IN_ClsLp;

        /* Entry 'ClsLp': '<S53>:2' */
        LgtCtrl_B.DRMC_vDes_mp2 = (real32_T)(sqrtf(-2.0F * DRMC_aReqFf_C *
          DRMC_lCurr) * 3.6);
      } else {
        LgtCtrl_B.DRMC_vDes_mp2 = VehDa_vEgoSpd;
      }
      break;
    }
  }

  /* End of Chart: '<S50>/S_V' */

  /* Chart: '<S1>/StartingJudge' incorporates:
   *  Inport: '<Root>/VehDa_stBrkReady4Rls_mp'
   *  UnitDelay: '<S1>/Unit Delay11'
   */
  /* Gateway: LgtCtrl/StartingJudge */
  /* During: LgtCtrl/StartingJudge */
  if (LgtCtrl_DW.is_active_c10_LgtCtrl == 0U) {
    /* Entry: LgtCtrl/StartingJudge */
    LgtCtrl_DW.is_active_c10_LgtCtrl = 1U;

    /* Entry Internal: LgtCtrl/StartingJudge */
    /* Transition: '<S9>:11' */
    LgtCtrl_DW.is_c10_LgtCtrl = LgtCtrl_IN_NoAction_i;

    /* Entry 'NoAction': '<S9>:1' */
    rtb_DataTypeConversion6_g = 0U;
  } else if (LgtCtrl_DW.is_c10_LgtCtrl == 1) {
    rtb_DataTypeConversion6_g = 0U;

    /* During 'NoAction': '<S9>:1' */
    if ((LgtCtrl_DW.UnitDelay11_DSTATE == 6) && (VehDa_stBrkReady4Rls_mp == 0) &&
        (CoChs_stCdn == 2) && (VehDa_vEgoSpd < 1.0F)) {
      /* Transition: '<S9>:3' */
      LgtCtrl_DW.is_c10_LgtCtrl = LgtCtrl_IN_Starting_e;

      /* Entry 'Starting': '<S9>:2' */
      rtb_DataTypeConversion6_g = 1U;
    }
  } else {
    rtb_DataTypeConversion6_g = 1U;

    /* During 'Starting': '<S9>:2' */
    if ((VehDa_stBrkReady4Rls_mp == 1) || (CoChs_stCdn != 2) || (VehDa_vEgoSpd >
         3.0F)) {
      /* Transition: '<S9>:4' */
      LgtCtrl_DW.is_c10_LgtCtrl = LgtCtrl_IN_NoAction_i;

      /* Entry 'NoAction': '<S9>:1' */
      rtb_DataTypeConversion6_g = 0U;
    }
  }

  /* End of Chart: '<S1>/StartingJudge' */

  /* DataTypeConversion: '<S1>/Data Type Conversion1' incorporates:
   *  Constant: '<S1>/CoChs_swtStartingEna_C'
   *  Logic: '<S1>/Logical Operator'
   */
  CoChs_stStarting = (uint8_T)((rtb_DataTypeConversion6_g != 0) &&
    (CoChs_swtStartingEna_C != 0));

  /* DataTypeConversion: '<S14>/Data Type Conversion10' incorporates:
   *  Gain: '<S158>/Gain'
   *  Inport: '<Root>/VehDa_prcTrqEstimdLoss_mp'
   */
  VehDa_prcTrqEstimdLoss = (int16_T)((16384 * VehDa_prcTrqEstimdLoss_mp) >> 14);

  /* Chart: '<S63>/GoJudg' incorporates:
   *  Constant: '<S63>/Eng_vGoThd_C'
   *  Constant: '<S63>/Eng_vVehStaticThd_C'
   *  DataTypeConversion: '<S5>/Data Type Conversion1'
   *  UnitDelay: '<S63>/UD1'
   */
  /* Gateway: LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/GoJudg */
  /* During: LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/GoJudg */
  if (LgtCtrl_DW.is_active_c4_LgtCtrl == 0U) {
    /* Entry: LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/GoJudg */
    LgtCtrl_DW.is_active_c4_LgtCtrl = 1U;

    /* Entry Internal: LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/GoJudg */
    /* Transition: '<S65>:12' */
    LgtCtrl_DW.is_c4_LgtCtrl = LgtCtrl_IN_GoEnd;

    /* Entry 'GoEnd': '<S65>:2' */
    rtb_DataTypeConversion6_g = 0U;
  } else if (LgtCtrl_DW.is_c4_LgtCtrl == 1) {
    rtb_DataTypeConversion6_g = 0U;

    /* During 'GoEnd': '<S65>:2' */
    if ((VehDa_vEgoSpd < Eng_vVehStaticThd_C) && (CoChs_stCdn == 2) &&
        (LgtCtrl_DW.UD1_DSTATE == 6)) {
      /* Transition: '<S65>:4' */
      LgtCtrl_DW.is_c4_LgtCtrl = LgtCtrl_IN_GoStrt;

      /* Entry 'GoStrt': '<S65>:1' */
      rtb_DataTypeConversion6_g = 1U;
    }
  } else {
    rtb_DataTypeConversion6_g = 1U;

    /* During 'GoStrt': '<S65>:1' */
    if ((VehDa_vEgoSpd >= Eng_vGoThd_C) || (CoChs_stCdn != 2)) {
      /* Transition: '<S65>:5' */
      LgtCtrl_DW.is_c4_LgtCtrl = LgtCtrl_IN_GoEnd;

      /* Entry 'GoEnd': '<S65>:2' */
      rtb_DataTypeConversion6_g = 0U;
    }
  }

  /* End of Chart: '<S63>/GoJudg' */

  /* DataTypeConversion: '<S5>/Data Type Conversion8' */
  Eng_stVehGo_mp = rtb_DataTypeConversion6_g;

  /* Product: '<S59>/P3' incorporates:
   *  Constant: '<S59>/VDC_etaDrvAxlEff_C'
   *  Constant: '<S59>/VDC_rFinalRatio_C'
   *  Constant: '<S59>/VDC_rdTire_C'
   *  Product: '<S59>/P'
   *  Product: '<S59>/P1'
   */
  DrvAxl_trqRaw_mp = VDC_frDrvForce * VDC_rdTire_C / VDC_rFinalRatio_C /
    VDC_etaDrvAxlEff_C;

  /* Gain: '<S58>/Gain' */
  DrvAxl_trqReq = 1.0F * DrvAxl_trqRaw_mp;

  /* Constant: '<S84>/VDC_etaTra_C' */
  Tra_etaGear_mp = VDC_etaTra_C;

  /* Switch: '<S85>/Swt1' incorporates:
   *  Constant: '<S85>/C3'
   *  Constant: '<S85>/Tra_stCfg_C'
   *  Constant: '<S85>/Tra_stDflNeutrGear_C'
   *  DataTypeConversion: '<S5>/Data Type Conversion'
   *  Logic: '<S85>/L1'
   *  RelationalOperator: '<S85>/R1'
   */
  if ((VehDa_stTraCurGear != Tra_stDflNeutrGear_C) && (Tra_stCfg_C != 0)) {
    rtb_Swt1_o_idx_2 = VehDa_rTraCurGear;
  } else {
    rtb_Swt1_o_idx_2 = 1.0F;
  }

  /* End of Switch: '<S85>/Swt1' */

  /* Product: '<S85>/P1' incorporates:
   *  Product: '<S85>/P'
   */
  Tra_trqRaw_mp = DrvAxl_trqReq / rtb_Swt1_o_idx_2 / Tra_etaGear_mp;

  /* Gain: '<S83>/Gain' */
  Tra_trqReq = 1.0F * Tra_trqRaw_mp;

  /* Gain: '<S61>/Gain' incorporates:
   *  Constant: '<S61>/VDC_trqRef_C'
   *  Product: '<S61>/D'
   */
  Eng_prcTrqRaw_mp = Tra_trqReq / VDC_trqRef_C * 100.0F;

  /* Switch: '<S61>/Swt' incorporates:
   *  Constant: '<S61>/C'
   *  Constant: '<S61>/Eng_stNomFricEna_C'
   *  DataTypeConversion: '<S5>/Data Type Conversion2'
   */
  if (Eng_stNomFricEna_C) {
    rtb_Swt1_o_idx_2 = VehDa_prcTrqEngNomFric;
  } else {
    rtb_Swt1_o_idx_2 = 0.0F;
  }

  /* End of Switch: '<S61>/Swt' */

  /* Switch: '<S61>/Swt1' incorporates:
   *  Constant: '<S61>/C'
   *  Constant: '<S61>/Eng_stEstimdLossEna_C'
   *  DataTypeConversion: '<S5>/Data Type Conversion3'
   */
  if (Eng_stEstimdLossEna_C) {
    rtb_MinMax2_h = VehDa_prcTrqEstimdLoss;
  } else {
    rtb_MinMax2_h = 0.0F;
  }

  /* End of Switch: '<S61>/Swt1' */

  /* Sum: '<S61>/A' */
  Eng_prcTrqRaw_mp1 = (Eng_prcTrqRaw_mp + rtb_Swt1_o_idx_2) + rtb_MinMax2_h;

  /* Switch: '<S61>/Swt2' incorporates:
   *  Constant: '<S61>/C'
   *  DataTypeConversion: '<S5>/Data Type Conversion5'
   *  DataTypeConversion: '<S5>/Data Type Conversion6'
   *  DataTypeConversion: '<S5>/Data Type Conversion7'
   *  Sum: '<S61>/A1'
   */
  if (Tra_stTrqLimWthTCU != 0) {
    rtb_Swt_lo = (int16_T)(VehDa_prcDrvrDmdTrq - VehDa_prcActuTrq);
  } else {
    rtb_Swt_lo = 0.0F;
  }

  /* End of Switch: '<S61>/Swt2' */

  /* Switch: '<S64>/Swt1' incorporates:
   *  Constant: '<S64>/C '
   *  Constant: '<S64>/Eng_swtTrqLimCmpFild_C'
   *  UnitDelay: '<S64>/Unit Delay1'
   */
  if (LgtCtrl_DW.UnitDelay1_DSTATE_f != 0.0) {
    rtb_Sum6 = Eng_swtTrqLimCmpFild_C;
  } else {
    rtb_Sum6 = 1.0;
  }

  /* End of Switch: '<S64>/Swt1' */

  /* Switch: '<S64>/Swt' incorporates:
   *  Constant: '<S64>/Eng_facTrqLimCmpFild_C'
   *  Product: '<S64>/Product5'
   *  Sum: '<S64>/Sum1'
   *  Sum: '<S64>/Sum6'
   *  UnitDelay: '<S64>/Unit Delay4'
   *  UnitDelay: '<S64>/Unit Delay5'
   */
  if (rtb_Sum6 != 0.0) {
    rtb_Swt_ot = rtb_Swt_lo;
  } else {
    rtb_Swt_ot = (LgtCtrl_DW.UnitDelay5_DSTATE_o -
                  LgtCtrl_DW.UnitDelay4_DSTATE_d) * Eng_facTrqLimCmpFild_C +
      LgtCtrl_DW.UnitDelay4_DSTATE_d;
  }

  /* End of Switch: '<S64>/Swt' */

  /* Sum: '<S61>/A2' incorporates:
   *  Constant: '<S61>/Eng_facTrqLimCmp_C'
   *  Product: '<S61>/Product'
   */
  Eng_prcTrqRaw_mp2 = rtb_Swt_ot * Eng_facTrqLimCmp_C + Eng_prcTrqRaw_mp1;

  /* Switch: '<S62>/Swt' */
  if (LgtCtrl_ConstB.R) {
    /* Switch: '<S62>/Swt' */
    Eng_prcTrqLim_mp = Eng_prcTrqRaw_mp2;
  } else {
    /* Switch: '<S62>/Swt' incorporates:
     *  Constant: '<S62>/TRQPERCENT_MAX'
     *  Constant: '<S62>/TRQPERCENT_MIN'
     *  MinMax: '<S62>/MinMax'
     *  MinMax: '<S62>/MinMax1'
     */
    Eng_prcTrqLim_mp = (real32_T)fmin(fmax(-100.0, Eng_prcTrqRaw_mp2), 100.0);
  }

  /* End of Switch: '<S62>/Swt' */

  /* MultiPortSwitch: '<S60>/MSwt' incorporates:
   *  DataTypeConversion: '<S5>/Data Type Conversion1'
   */
  switch (CoChs_stCdn) {
   case 2:
    /* MultiPortSwitch: '<S60>/MSwt' */
    Eng_prcTrqReq = Eng_prcTrqLim_mp;
    break;

   case 4:
    /* Product: '<S60>/Divide' incorporates:
     *  Constant: '<S60>/CoChs_vIdl_C'
     */
    rtb_deltariselimit_b = VehDa_vEgoSpd / CoChs_vIdl_C;

    /* Saturate: '<S60>/Saturation' */
    if (rtb_deltariselimit_b > 1.0F) {
      rtb_deltariselimit_b = 1.0F;
    } else if (rtb_deltariselimit_b < 0.0F) {
      rtb_deltariselimit_b = 0.0F;
    }

    /* End of Saturate: '<S60>/Saturation' */

    /* MultiPortSwitch: '<S60>/MSwt' incorporates:
     *  Constant: '<S60>/Eng_prcGoEnd_C'
     *  Constant: '<S60>/Eng_prcGo_C'
     *  Product: '<S60>/Product'
     *  Sum: '<S60>/Add'
     *  Sum: '<S60>/Add1'
     */
    Eng_prcTrqReq = Eng_prcGo_C - (Eng_prcGo_C - Eng_prcGoEnd_C) *
      rtb_deltariselimit_b;
    break;

   case 9:
    /* MultiPortSwitch: '<S60>/MSwt' incorporates:
     *  Constant: '<S60>/Eng_prcTstReq_C'
     */
    Eng_prcTrqReq = Eng_prcTstReq_C;
    break;

   default:
    /* MultiPortSwitch: '<S60>/MSwt' incorporates:
     *  Constant: '<S60>/C2'
     *  DataTypeConversion: '<S5>/Data Type Conversion2'
     *  Gain: '<S60>/Gain'
     *  Sum: '<S60>/Add2'
     */
    Eng_prcTrqReq = ((real32_T)VehDa_prcTrqEngNomFric + 2.0F) * (-1.0F);
    break;
  }

  /* End of MultiPortSwitch: '<S60>/MSwt' */

  /* Product: '<S60>/D' incorporates:
   *  Constant: '<S60>/C1'
   *  Constant: '<S60>/VDC_trqRef_C'
   *  Product: '<S60>/D1'
   */
  Eng_trqReq = Eng_prcTrqReq / 100.0F * VDC_trqRef_C;

  /* Sum: '<S63>/Add' incorporates:
   *  Constant: '<S63>/C4'
   *  Constant: '<S63>/C5'
   *  Constant: '<S63>/X69A_TRQ_MAX'
   *  Product: '<S63>/D3'
   *  Product: '<S63>/D4'
   */
  rtb_Add_lp = Eng_trqReq / 3400.0F * 99.0F + 1.0F;

  /* Product: '<S67>/delta rise limit' incorporates:
   *  Constant: '<S63>/Eng_dtAccrPedlUp_C'
   *  SampleTimeMath: '<S67>/sample time'
   *
   * About '<S67>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_deltariselimit_b = Eng_dtAccrPedlUp_C * 0.01F;

  /* MinMax: '<S161>/MinMax' incorporates:
   *  Constant: '<S161>/Eng_nIdl_C'
   *  Inport: '<Root>/VehDa_nOutpShaft_mp'
   *  Product: '<S161>/Product'
   */
  VehDa_nEngSpdEst = fmaxf(VehDa_nOutpShaft_mp * VehDa_rTraCurGear, Eng_nIdl_C);

  /* Gain: '<S160>/Gain' incorporates:
   *  Inport: '<Root>/VehDa_nEngSpd_mp'
   */
  VehDa_nEngSpd = 1.0F * VehDa_nEngSpd_mp;

  /* Switch: '<S1>/Switch' incorporates:
   *  Constant: '<S1>/Debug_swtEngSpdSlect_C'
   */
  if (Debug_swtEngSpdSlect_C) {
    rtb_Swt1_o_idx_2 = VehDa_nEngSpdEst;
  } else {
    rtb_Swt1_o_idx_2 = VehDa_nEngSpd;
  }

  /* End of Switch: '<S1>/Switch' */

  /* MinMax: '<S63>/MinMax2' incorporates:
   *  Constant: '<S63>/Eng_nIdl_C'
   *  Constant: '<S63>/Eng_nMax_C'
   *  MinMax: '<S63>/MinMax3'
   */
  rtb_MinMax2_h = fmaxf(fminf(rtb_Swt1_o_idx_2, Eng_nMax_C), Eng_nIdl_C);

  /* Lookup_n-D: '<S63>/H19E-Dci450-51-MAPA' incorporates:
   *  Constant: '<S63>/Thr_H19E'
   *  MinMax: '<S63>/MinMax2'
   */
  bpIndices[0U] = plook_u32ff_binx(rtb_MinMax2_h,
    LgtCtrl_ConstP.H19EDci45051MAPA_bp01Data, 21U, &rtb_Swt1_o_idx_2);
  fractions[0U] = rtb_Swt1_o_idx_2;
  for (i = 0; i < 10; i++) {
    bpIndices[1U] = plook_u32ff_binx(LgtCtrl_ConstP.pooled39[i],
      LgtCtrl_ConstP.pooled39, 9U, &rtb_Swt1_o_idx_2);
    fractions[1U] = rtb_Swt1_o_idx_2;
    rtb_H19EDci45051MAPA[i] = intrp2d_fu32fl_pw(bpIndices, fractions,
      LgtCtrl_ConstP.H19EDci45051MAPA_tableData, 22U);
  }

  /* End of Lookup_n-D: '<S63>/H19E-Dci450-51-MAPA' */

  /* Product: '<S63>/D1' incorporates:
   *  Constant: '<S63>/PRC_'
   *  Constant: '<S63>/VDC_trqRef_C'
   *  DataTypeConversion: '<S5>/Data Type Conversion2'
   *  Gain: '<S63>/Gain1'
   *  Product: '<S63>/D2'
   */
  rtb_D1_f = (real32_T)((-32768) * VehDa_prcTrqEngNomFric) * 3.05175781E-5F /
    100.0F * VDC_trqRef_C;

  /* Chart: '<S63>/ThrCalc_H19E' incorporates:
   *  Constant: '<S63>/Thr_H19E'
   */
  /* Gateway: LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/ThrCalc_H19E */
  /* During: LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/ThrCalc_H19E */
  /* Entry Internal: LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/ThrCalc_H19E */
  /* Transition: '<S70>:2' */
  i = 0;
  judge = 0;
  while ((i <= 8) && (judge == 0)) {
    /* Transition: '<S70>:5' */
    if ((Eng_trqReq < rtb_H19EDci45051MAPA[0]) || (Eng_trqReq <= rtb_D1_f)) {
      /* Transition: '<S70>:23' */
      /* Transition: '<S70>:26' */
      judge = (int32_T)1.0F;
      LgtCtrl_B.TrqExe_rEngThrReqd_a = 0.0F;

      /* Transition: '<S70>:27' */
    } else {
      /* Transition: '<S70>:9' */
    }

    /* Transition: '<S70>:29' */
    rtb_Swt1_o_idx_2 = rtb_H19EDci45051MAPA[i + 1];
    if ((judge != (int32_T)1.0F) && (Eng_trqReq >= rtb_H19EDci45051MAPA[i]) &&
        (Eng_trqReq < rtb_Swt1_o_idx_2)) {
      /* Transition: '<S70>:7' */
      /* Transition: '<S70>:11' */
      judge = (int32_T)1.0F;
      LgtCtrl_B.TrqExe_rEngThrReqd_a = (Eng_trqReq - rtb_H19EDci45051MAPA[i]) /
        (rtb_Swt1_o_idx_2 - rtb_H19EDci45051MAPA[i]) *
        (LgtCtrl_ConstP.pooled39[i + 1] - LgtCtrl_ConstP.pooled39[i]) +
        LgtCtrl_ConstP.pooled39[i];

      /* Transition: '<S70>:12' */
    } else {
      /* Transition: '<S70>:32' */
    }

    /* Transition: '<S70>:35' */
    if ((judge != (int32_T)1.0F) && (Eng_trqReq >= rtb_H19EDci45051MAPA[9])) {
      /* Transition: '<S70>:36' */
      /* Transition: '<S70>:38' */
      judge = (int32_T)1.0F;
      LgtCtrl_B.TrqExe_rEngThrReqd_a = 100.0F;

      /* Transition: '<S70>:40' */
    } else {
      /* Transition: '<S70>:41' */
    }

    /* Transition: '<S70>:13' */
    i++;
  }

  /* End of Chart: '<S63>/ThrCalc_H19E' */

  /* MinMax: '<S63>/MinMax' incorporates:
   *  Constant: '<S63>/Eng_nExtCurMin_C'
   */
  /* Transition: '<S70>:15' */
  rtb_D1_i = fmaxf(Eng_nExtCurMin_C, rtb_MinMax2_h);

  /* Lookup_n-D: '<S63>/KAX1-EQH175-T3-MAP-FULL' incorporates:
   *  Switch: '<S86>/Switch'
   */
  rtb_KAX1EQH175T3MAPFULL = look1_iflf_binlxpw(rtb_D1_i,
    LgtCtrl_ConstP.KAX1EQH175T3MAPFULL_bp01Data,
    LgtCtrl_ConstP.KAX1EQH175T3MAPFULL_tableData, 23U);

  /* MinMax: '<S63>/MinMax1' */
  rtb_MinMax1_p = fminf(Eng_trqReq, rtb_KAX1EQH175T3MAPFULL);

  /* Lookup_n-D: '<S63>/KAX1-EQH175-T3-MAP' incorporates:
   *  Constant: '<S63>/Thr_KAX1'
   *  MinMax: '<S63>/MinMax2'
   */
  bpIndices_0[0U] = plook_u32ff_binx(rtb_MinMax2_h,
    LgtCtrl_ConstP.KAX1EQH175T3MAP_bp01Data, 15U, &rtb_Swt1_o_idx_2);
  fractions_0[0U] = rtb_Swt1_o_idx_2;
  for (i = 0; i < 16; i++) {
    bpIndices_0[1U] = plook_u32ff_binx(LgtCtrl_ConstP.pooled40[i],
      LgtCtrl_ConstP.pooled40, 15U, &rtb_Swt1_o_idx_2);
    fractions_0[1U] = rtb_Swt1_o_idx_2;
    rtb_KAX1EQH175T3MAP[i] = intrp2d_fu32fl_pw(bpIndices_0, fractions_0,
      LgtCtrl_ConstP.KAX1EQH175T3MAP_tableData, 16U);
  }

  /* End of Lookup_n-D: '<S63>/KAX1-EQH175-T3-MAP' */

  /* Chart: '<S63>/ThrCalc_KAX1' incorporates:
   *  Constant: '<S63>/Thr_KAX1'
   */
  /* Gateway: LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/ThrCalc_KAX1 */
  /* During: LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/ThrCalc_KAX1 */
  /* Entry Internal: LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/ThrCalc_KAX1 */
  /* Transition: '<S71>:2' */
  i = 0;
  judge = 0;
  while ((i <= 14) && (judge == 0)) {
    /* Transition: '<S71>:5' */
    if ((rtb_MinMax1_p < rtb_KAX1EQH175T3MAP[0]) || (rtb_MinMax1_p <= rtb_D1_f))
    {
      /* Transition: '<S71>:23' */
      /* Transition: '<S71>:26' */
      judge = (int32_T)1.0F;
      LgtCtrl_B.TrqExe_rEngThrReqd = 0.0F;

      /* Transition: '<S71>:27' */
    } else {
      /* Transition: '<S71>:9' */
    }

    /* Transition: '<S71>:29' */
    rtb_Swt1_o_idx_2 = rtb_KAX1EQH175T3MAP[i + 1];
    if ((judge != (int32_T)1.0F) && (rtb_MinMax1_p >= rtb_KAX1EQH175T3MAP[i]) &&
        (rtb_MinMax1_p < rtb_Swt1_o_idx_2)) {
      /* Transition: '<S71>:7' */
      /* Transition: '<S71>:11' */
      judge = (int32_T)1.0F;
      LgtCtrl_B.TrqExe_rEngThrReqd = (rtb_MinMax1_p - rtb_KAX1EQH175T3MAP[i]) /
        (rtb_Swt1_o_idx_2 - rtb_KAX1EQH175T3MAP[i]) * (LgtCtrl_ConstP.pooled40[i
        + 1] - LgtCtrl_ConstP.pooled40[i]) + LgtCtrl_ConstP.pooled40[i];

      /* Transition: '<S71>:12' */
    } else {
      /* Transition: '<S71>:32' */
    }

    /* Transition: '<S71>:35' */
    if ((judge != (int32_T)1.0F) && (rtb_MinMax1_p >= rtb_KAX1EQH175T3MAP[15]))
    {
      /* Transition: '<S71>:36' */
      /* Transition: '<S71>:38' */
      judge = (int32_T)1.0F;
      LgtCtrl_B.TrqExe_rEngThrReqd = 100.0F;

      /* Transition: '<S71>:40' */
    } else {
      /* Transition: '<S71>:41' */
    }

    /* Transition: '<S71>:13' */
    i++;
  }

  /* End of Chart: '<S63>/ThrCalc_KAX1' */

  /* MultiPortSwitch: '<S63>/Multiport Switch' incorporates:
   *  Constant: '<S63>/Eng_idPltfrm_C'
   */
  /* Transition: '<S71>:15' */
  switch (Eng_idPltfrm_C) {
   case 1:
    rtb_Add_lp = LgtCtrl_B.TrqExe_rEngThrReqd_a;
    break;

   case 2:
    rtb_Add_lp = LgtCtrl_B.TrqExe_rEngThrReqd;
    break;

   case 3:
    /* Switch: '<S68>/Switch2' incorporates:
     *  Constant: '<S63>/C6'
     *  Constant: '<S63>/C7'
     *  RelationalOperator: '<S68>/LowerRelop1'
     *  RelationalOperator: '<S68>/UpperRelop'
     *  Switch: '<S68>/Switch'
     */
    if (rtb_Add_lp > 30.0F) {
      rtb_Add_lp = 30.0F;
    } else if (rtb_Add_lp < 0.0F) {
      /* Switch: '<S68>/Switch' incorporates:
       *  Constant: '<S63>/C6'
       */
      rtb_Add_lp = 0.0F;
    }

    /* End of Switch: '<S68>/Switch2' */
    break;

   default:
    rtb_Add_lp = LgtCtrl_B.TrqExe_rEngThrReqd_a;
    break;
  }

  /* End of MultiPortSwitch: '<S63>/Multiport Switch' */

  /* Sum: '<S67>/Difference Inputs1' incorporates:
   *  UnitDelay: '<S67>/Delay Input2'
   *
   * Block description for '<S67>/Difference Inputs1':
   *
   *  Add in CPU
   *
   * Block description for '<S67>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add_lp -= Eng_rAccrPedlReq_mp;

  /* Product: '<S67>/delta fall limit' incorporates:
   *  Constant: '<S63>/Eng_dtAccrPedlDwn_C'
   *  SampleTimeMath: '<S67>/sample time'
   *
   * About '<S67>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_Swt1_o_idx_2 = Eng_dtAccrPedlDwn_C * 0.01F;

  /* Switch: '<S79>/Switch2' incorporates:
   *  RelationalOperator: '<S79>/LowerRelop1'
   *  RelationalOperator: '<S79>/UpperRelop'
   *  Switch: '<S79>/Switch'
   */
  if (rtb_Add_lp > rtb_deltariselimit_b) {
    rtb_Add_lp = rtb_deltariselimit_b;
  } else if (rtb_Add_lp < rtb_Swt1_o_idx_2) {
    /* Switch: '<S79>/Switch' */
    rtb_Add_lp = rtb_Swt1_o_idx_2;
  }

  /* End of Switch: '<S79>/Switch2' */

  /* Sum: '<S67>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S67>/Delay Input2'
   *
   * Block description for '<S67>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S67>/Delay Input2':
   *
   *  Store in Global RAM
   */
  Eng_rAccrPedlReq_mp += rtb_Add_lp;

  /* Switch: '<S63>/Switch1' incorporates:
   *  Constant: '<S63>/Eng_rGoPedlCmp_C'
   *  Product: '<S63>/Product'
   *  Sum: '<S63>/Add1'
   *  Sum: '<S63>/Add2'
   */
  if (rtb_DataTypeConversion6_g != 0) {
    /* Product: '<S63>/Divide' incorporates:
     *  Constant: '<S63>/Eng_vGoThd_C'
     */
    rtb_deltariselimit_b = VehDa_vEgoSpd / Eng_vGoThd_C;

    /* Saturate: '<S63>/Saturation' */
    if (rtb_deltariselimit_b > 1.0F) {
      rtb_deltariselimit_b = 1.0F;
    } else if (rtb_deltariselimit_b < 0.0F) {
      rtb_deltariselimit_b = 0.0F;
    }

    /* End of Saturate: '<S63>/Saturation' */
    rtb_deltariselimit_b = (Eng_rGoPedlCmp_C - Eng_rGoPedlCmp_C *
      rtb_deltariselimit_b) + Eng_rAccrPedlReq_mp;
  } else {
    rtb_deltariselimit_b = Eng_rAccrPedlReq_mp;
  }

  /* End of Switch: '<S63>/Switch1' */

  /* Switch: '<S63>/Switch' incorporates:
   *  Constant: '<S63>/C2'
   *  Constant: '<S63>/PowerCtrl'
   *  DataTypeConversion: '<S5>/Data Type Conversion1'
   *  DataTypeConversion: '<S5>/Data Type Conversion4'
   *  Logic: '<S63>/L5'
   *  RelationalOperator: '<S63>/L2'
   *  RelationalOperator: '<S63>/L4'
   *  UnitDelay: '<S63>/UD'
   */
  if ((CoChs_stCdn == 2.0) && (VehDa_stCluSwt != 0.0)) {
    rtb_MinMax2_h = LgtCtrl_DW.UD_DSTATE;
  } else {
    rtb_MinMax2_h = rtb_deltariselimit_b;
  }

  /* End of Switch: '<S63>/Switch' */

  /* MultiPortSwitch: '<S63>/MSwt' incorporates:
   *  DataTypeConversion: '<S5>/Data Type Conversion1'
   */
  switch (CoChs_stCdn) {
   case 11:
    /* MultiPortSwitch: '<S63>/MSwt' incorporates:
     *  Constant: '<S63>/Eng_rAccrPedlTstReq_C'
     */
    Eng_rAccrPedlReqRaw = Eng_rAccrPedlTstReq_C;
    break;

   case 7:
    /* MultiPortSwitch: '<S46>/Multiport Switch' */
    switch (LgtCtrl_B.DRMC_stMain) {
     case 0:
      /* MultiPortSwitch: '<S63>/MSwt' incorporates:
       *  Constant: '<S46>/C'
       */
      Eng_rAccrPedlReqRaw = 0.0F;
      break;

     case 1:
      /* MultiPortSwitch: '<S63>/MSwt' incorporates:
       *  Constant: '<S46>/C'
       */
      Eng_rAccrPedlReqRaw = 0.0F;
      break;

     case 2:
      /* MultiPortSwitch: '<S63>/MSwt' incorporates:
       *  Constant: '<S46>/DRMC_rAcclPedlPosnReq_C'
       */
      Eng_rAccrPedlReqRaw = DRMC_rAcclPedlPosnReq_C;
      break;

     case 3:
      /* MultiPortSwitch: '<S63>/MSwt' incorporates:
       *  Constant: '<S46>/C'
       */
      Eng_rAccrPedlReqRaw = 0.0F;
      break;

     case 4:
      /* MultiPortSwitch: '<S63>/MSwt' incorporates:
       *  Constant: '<S46>/C'
       */
      Eng_rAccrPedlReqRaw = 0.0F;
      break;

     case 5:
      /* MultiPortSwitch: '<S63>/MSwt' incorporates:
       *  Constant: '<S46>/C'
       */
      Eng_rAccrPedlReqRaw = 0.0F;
      break;

     case 25:
      /* MultiPortSwitch: '<S63>/MSwt' incorporates:
       *  Constant: '<S46>/C'
       */
      Eng_rAccrPedlReqRaw = 0.0F;
      break;

     default:
      /* MultiPortSwitch: '<S63>/MSwt' incorporates:
       *  Constant: '<S46>/C'
       */
      Eng_rAccrPedlReqRaw = 0.0F;
      break;
    }

    /* End of MultiPortSwitch: '<S46>/Multiport Switch' */
    break;

   case 8:
    /* MultiPortSwitch: '<S63>/MSwt' incorporates:
     *  Inport: '<Root>/VehDa_rAccrPedl_mp'
     */
    Eng_rAccrPedlReqRaw = VehDa_rAccrPedl_mp;
    break;

   case 12:
    /* MultiPortSwitch: '<S63>/MSwt' incorporates:
     *  Constant: '<S63>/Constant'
     */
    Eng_rAccrPedlReqRaw = 0.0F;
    break;

   case 3:
    /* MultiPortSwitch: '<S63>/MSwt' incorporates:
     *  Constant: '<S63>/Constant1'
     */
    Eng_rAccrPedlReqRaw = 0.0F;
    break;

   default:
    /* Switch: '<S63>/Switch2' incorporates:
     *  Constant: '<S63>/Eng_stPedlFrz_C'
     */
    if (Eng_stPedlFrz_C != 0) {
      /* MultiPortSwitch: '<S63>/MSwt' */
      Eng_rAccrPedlReqRaw = rtb_deltariselimit_b;
    } else {
      /* MultiPortSwitch: '<S63>/MSwt' */
      Eng_rAccrPedlReqRaw = rtb_MinMax2_h;
    }

    /* End of Switch: '<S63>/Switch2' */
    break;
  }

  /* End of MultiPortSwitch: '<S63>/MSwt' */

  /* MultiPortSwitch: '<S63>/Multiport Switch1' incorporates:
   *  Constant: '<S63>/Eng_idPltfrm_C'
   */
  switch (Eng_idPltfrm_C) {
   case 1:
    /* MultiPortSwitch: '<S63>/Multiport Switch1' incorporates:
     *  Lookup_n-D: '<S63>/H19E-Dci450-51-MAPA-FULL'
     *  Switch: '<S86>/Switch'
     */
    Eng_trqCurrMax = look1_iflf_binlxpw(rtb_D1_i,
      LgtCtrl_ConstP.H19EDci45051MAPAFULL_bp01Data,
      LgtCtrl_ConstP.H19EDci45051MAPAFULL_tableData, 14U);
    break;

   case 2:
    /* MultiPortSwitch: '<S63>/Multiport Switch1' */
    Eng_trqCurrMax = rtb_KAX1EQH175T3MAPFULL;
    break;

   case 3:
    /* MultiPortSwitch: '<S63>/Multiport Switch1' incorporates:
     *  Constant: '<S63>/X69A_TRQ_MAX'
     */
    Eng_trqCurrMax = 3400.0F;
    break;

   default:
    /* MultiPortSwitch: '<S63>/Multiport Switch1' incorporates:
     *  Lookup_n-D: '<S63>/H19E-Dci450-51-MAPA-FULL'
     *  Switch: '<S86>/Switch'
     */
    Eng_trqCurrMax = look1_iflf_binlxpw(rtb_D1_i,
      LgtCtrl_ConstP.H19EDci45051MAPAFULL_bp01Data,
      LgtCtrl_ConstP.H19EDci45051MAPAFULL_tableData, 14U);
    break;
  }

  /* End of MultiPortSwitch: '<S63>/Multiport Switch1' */

  /* UnitDelay: '<S66>/Unit Delay1' */
  rtb_UnitDelay1_bt = LgtCtrl_DW.UnitDelay1_DSTATE_n1;

  /* UnitDelay: '<S69>/Unit Delay1' */
  rtb_deltariselimit_b = LgtCtrl_DW.UnitDelay1_DSTATE_k;

  /* Chart: '<S69>/filt' incorporates:
   *  DataTypeConversion: '<S5>/Data Type Conversion1'
   *  UnitDelay: '<S69>/Unit Delay'
   *  UnitDelay: '<S69>/Unit Delay2'
   */
  /* Gateway: LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/Subsystem/filt */
  /* During: LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/Subsystem/filt */
  if (LgtCtrl_DW.is_active_c12_LgtCtrl == 0U) {
    /* Entry: LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/Subsystem/filt */
    LgtCtrl_DW.is_active_c12_LgtCtrl = 1U;

    /* Entry Internal: LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/Subsystem/filt */
    /* Transition: '<S81>:161' */
    LgtCtrl_DW.is_c12_LgtCtrl = LgtCtrl_IN_No;

    /* Entry 'No': '<S81>:160' */
    rtb_DataTypeConversion6_g = 0U;
    LgtCtrl_DW.filt_count = 0U;
  } else {
    switch (LgtCtrl_DW.is_c12_LgtCtrl) {
     case LgtCtrl_IN_No:
      rtb_DataTypeConversion6_g = 0U;

      /* During 'No': '<S81>:160' */
      if ((LgtCtrl_DW.UnitDelay_DSTATE_no == 12) && (CoChs_stCdn == 2) &&
          (SpdPln_stTarType != 15)) {
        /* Transition: '<S81>:163' */
        LgtCtrl_DW.is_c12_LgtCtrl = LgtCtrl_IN_active;

        /* Entry 'active': '<S81>:162' */
        rtb_DataTypeConversion6_g = 1U;
      } else if ((LgtCtrl_DW.UnitDelay_DSTATE_no == 2) && (CoChs_stCdn == 12) &&
                 (SpdPln_stTarType != 15)) {
        /* Transition: '<S81>:186' */
        LgtCtrl_DW.is_c12_LgtCtrl = LgtCtrl_IN_active1;

        /* Entry 'active1': '<S81>:175' */
        rtb_DataTypeConversion6_g = 2U;
      }
      break;

     case LgtCtrl_IN_active:
      rtb_DataTypeConversion6_g = 1U;

      /* During 'active': '<S81>:162' */
      if ((LgtCtrl_DW.filt_count > 200) || (CoChs_stCdn != 2) || (fabsf
           (Eng_rAccrPedlReqRaw1 - rtb_deltariselimit_b) < 5.0F) ||
          (SpdPln_stTarType == 15)) {
        /* Transition: '<S81>:159' */
        LgtCtrl_DW.is_c12_LgtCtrl = LgtCtrl_IN_No;

        /* Entry 'No': '<S81>:160' */
        rtb_DataTypeConversion6_g = 0U;
        LgtCtrl_DW.filt_count = 0U;
      } else if ((LgtCtrl_DW.UnitDelay_DSTATE_no == 2) && (CoChs_stCdn == 12)) {
        /* Transition: '<S81>:188' */
        LgtCtrl_DW.filt_count = 0U;
        LgtCtrl_DW.is_c12_LgtCtrl = LgtCtrl_IN_active1;

        /* Entry 'active1': '<S81>:175' */
        rtb_DataTypeConversion6_g = 2U;
      } else {
        i = LgtCtrl_DW.filt_count + 1;
        if (LgtCtrl_DW.filt_count + 1 > 255) {
          i = 255;
        }

        LgtCtrl_DW.filt_count = (uint8_T)i;
      }
      break;

     default:
      rtb_DataTypeConversion6_g = 2U;

      /* During 'active1': '<S81>:175' */
      if ((LgtCtrl_DW.filt_count > 200) || (CoChs_stCdn != 12) || (fabsf
           (Eng_rAccrPedlReqRaw1 - rtb_deltariselimit_b) < 5.0F) ||
          (SpdPln_stTarType == 15)) {
        /* Transition: '<S81>:187' */
        LgtCtrl_DW.is_c12_LgtCtrl = LgtCtrl_IN_No;

        /* Entry 'No': '<S81>:160' */
        rtb_DataTypeConversion6_g = 0U;
        LgtCtrl_DW.filt_count = 0U;
      } else if ((LgtCtrl_DW.UnitDelay_DSTATE_no == 12) && (CoChs_stCdn == 2)) {
        /* Transition: '<S81>:189' */
        LgtCtrl_DW.filt_count = 0U;
        LgtCtrl_DW.is_c12_LgtCtrl = LgtCtrl_IN_active;

        /* Entry 'active': '<S81>:162' */
        rtb_DataTypeConversion6_g = 1U;
      } else {
        i = LgtCtrl_DW.filt_count + 1;
        if (LgtCtrl_DW.filt_count + 1 > 255) {
          i = 255;
        }

        LgtCtrl_DW.filt_count = (uint8_T)i;
      }
      break;
    }
  }

  /* End of Chart: '<S69>/filt' */

  /* Switch: '<S69>/Switch3' incorporates:
   *  Constant: '<S69>/Constant4'
   *  Constant: '<S69>/Constant6'
   *  Lookup_n-D: '<S69>/CC_ThroltRateUp'
   *  RelationalOperator: '<S69>/R2'
   *  UnitDelay: '<S69>/Unit Delay2'
   */
  if (rtb_DataTypeConversion6_g == 1.0F) {
    rtb_D1_i = look1_iflf_binlcapw(Eng_rAccrPedlReqRaw1, CC_ThroltRate_tableX,
      CC_ThroltRate_tableY, 4U);
  } else {
    rtb_D1_i = 200.0F;
  }

  /* End of Switch: '<S69>/Switch3' */

  /* Product: '<S80>/delta rise limit' incorporates:
   *  SampleTimeMath: '<S80>/sample time'
   *
   * About '<S80>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_D1_i *= 0.01F;

  /* Sum: '<S80>/Difference Inputs1' incorporates:
   *  UnitDelay: '<S80>/Delay Input2'
   *
   * Block description for '<S80>/Difference Inputs1':
   *
   *  Add in CPU
   *
   * Block description for '<S80>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_deltariselimit_b -= Eng_rAccrPedlReqRaw1;

  /* Switch: '<S69>/Switch4' incorporates:
   *  Constant: '<S69>/CC_ThroltRate_lo'
   *  Constant: '<S69>/CC_ThroltRate_lo1'
   *  Constant: '<S69>/Constant5'
   *  RelationalOperator: '<S69>/R1'
   */
  if (rtb_DataTypeConversion6_g == 2.0F) {
    rtb_KAX1EQH175T3MAPFULL = CC_ThroltRate_lo;
  } else {
    rtb_KAX1EQH175T3MAPFULL = CC_ThroltRate_lo1;
  }

  /* End of Switch: '<S69>/Switch4' */

  /* Product: '<S80>/delta fall limit' incorporates:
   *  SampleTimeMath: '<S80>/sample time'
   *
   * About '<S80>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_Swt1_o_idx_2 = rtb_KAX1EQH175T3MAPFULL * 0.01F;

  /* Switch: '<S82>/Switch2' incorporates:
   *  RelationalOperator: '<S82>/LowerRelop1'
   *  RelationalOperator: '<S82>/UpperRelop'
   *  Switch: '<S82>/Switch'
   */
  if (rtb_deltariselimit_b > rtb_D1_i) {
    rtb_deltariselimit_b = rtb_D1_i;
  } else if (rtb_deltariselimit_b < rtb_Swt1_o_idx_2) {
    /* Switch: '<S82>/Switch' */
    rtb_deltariselimit_b = rtb_Swt1_o_idx_2;
  }

  /* End of Switch: '<S82>/Switch2' */

  /* Sum: '<S80>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S80>/Delay Input2'
   *
   * Block description for '<S80>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S80>/Delay Input2':
   *
   *  Store in Global RAM
   */
  Eng_rAccrPedlReqRaw1 += rtb_deltariselimit_b;

  /* Gain: '<S171>/Gain' incorporates:
   *  Inport: '<Root>/SpdPln_pcc_tar_throttle_mp'
   */
  SpdPln_pcc_tar_throttle = 1.0F * SpdPln_pcc_tar_throttle_mp;

  /* Chart: '<S66>/PCC_CC_ChangeStateCal' incorporates:
   *  UnitDelay: '<S66>/Unit Delay2'
   */
  /* Gateway: LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/PCC_AccPedlRateSlip2/PCC_CC_ChangeStateCal */
  /* During: LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/PCC_AccPedlRateSlip2/PCC_CC_ChangeStateCal */
  if (LgtCtrl_DW.is_active_c11_LgtCtrl == 0U) {
    /* Entry: LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/PCC_AccPedlRateSlip2/PCC_CC_ChangeStateCal */
    LgtCtrl_DW.is_active_c11_LgtCtrl = 1U;

    /* Entry Internal: LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/PCC_AccPedlRateSlip2/PCC_CC_ChangeStateCal */
    /* Transition: '<S72>:161' */
    LgtCtrl_DW.is_c11_LgtCtrl = LgtCtrl_IN_NoAction;

    /* Entry 'NoAction': '<S72>:160' */
    Eng_StPCC_CC_Change = 0U;
    LgtCtrl_DW.Eng_tTimerCount = 0U;
  } else {
    switch (LgtCtrl_DW.is_c11_LgtCtrl) {
     case LgtCtrl_IN_CC2PCC:
      Eng_StPCC_CC_Change = 1U;

      /* During 'CC2PCC': '<S72>:162' */
      if ((LgtCtrl_DW.Eng_tTimerCount > 200) || (fabsf(Eng_rAccrPedlReq -
            SpdPln_pcc_tar_throttle) < 5.0F)) {
        /* Transition: '<S72>:159' */
        LgtCtrl_DW.is_c11_LgtCtrl = LgtCtrl_IN_NoAction;

        /* Entry 'NoAction': '<S72>:160' */
        Eng_StPCC_CC_Change = 0U;
        LgtCtrl_DW.Eng_tTimerCount = 0U;
      } else if ((rtb_UnitDelay1_bt == 15) && (SpdPln_stTarType != 15)) {
        /* Transition: '<S72>:166' */
        LgtCtrl_DW.Eng_tTimerCount = 0U;
        LgtCtrl_DW.is_c11_LgtCtrl = LgtCtrl_IN_PCC2CC;

        /* Entry 'PCC2CC': '<S72>:164' */
        Eng_StPCC_CC_Change = 2U;
      } else {
        i = LgtCtrl_DW.Eng_tTimerCount + 1;
        if (LgtCtrl_DW.Eng_tTimerCount + 1 > 65535) {
          i = 65535;
        }

        LgtCtrl_DW.Eng_tTimerCount = (uint16_T)i;
      }
      break;

     case LgtCtrl_IN_NoAction:
      Eng_StPCC_CC_Change = 0U;

      /* During 'NoAction': '<S72>:160' */
      if ((rtb_UnitDelay1_bt != 15) && (SpdPln_stTarType == 15)) {
        /* Transition: '<S72>:163' */
        LgtCtrl_DW.is_c11_LgtCtrl = LgtCtrl_IN_CC2PCC;

        /* Entry 'CC2PCC': '<S72>:162' */
        Eng_StPCC_CC_Change = 1U;
      } else if ((rtb_UnitDelay1_bt == 15) && (SpdPln_stTarType != 15)) {
        /* Transition: '<S72>:165' */
        LgtCtrl_DW.is_c11_LgtCtrl = LgtCtrl_IN_PCC2CC;

        /* Entry 'PCC2CC': '<S72>:164' */
        Eng_StPCC_CC_Change = 2U;
      }
      break;

     default:
      Eng_StPCC_CC_Change = 2U;

      /* During 'PCC2CC': '<S72>:164' */
      if ((rtb_UnitDelay1_bt != 15) && (SpdPln_stTarType == 15)) {
        /* Transition: '<S72>:167' */
        LgtCtrl_DW.Eng_tTimerCount = 0U;
        LgtCtrl_DW.is_c11_LgtCtrl = LgtCtrl_IN_CC2PCC;

        /* Entry 'CC2PCC': '<S72>:162' */
        Eng_StPCC_CC_Change = 1U;
      } else if ((LgtCtrl_DW.Eng_tTimerCount > 200) || (fabsf(Eng_rAccrPedlReq -
        Eng_rAccrPedlReqRaw1) < 5.0F)) {
        /* Transition: '<S72>:168' */
        LgtCtrl_DW.is_c11_LgtCtrl = LgtCtrl_IN_NoAction;

        /* Entry 'NoAction': '<S72>:160' */
        Eng_StPCC_CC_Change = 0U;
        LgtCtrl_DW.Eng_tTimerCount = 0U;
      } else {
        i = LgtCtrl_DW.Eng_tTimerCount + 1;
        if (LgtCtrl_DW.Eng_tTimerCount + 1 > 65535) {
          i = 65535;
        }

        LgtCtrl_DW.Eng_tTimerCount = (uint16_T)i;
      }
      break;
    }
  }

  /* End of Chart: '<S66>/PCC_CC_ChangeStateCal' */

  /* UnitDelay: '<S74>/Unit Delay' */
  LogicalOperator2 = LgtCtrl_DW.UnitDelay_DSTATE_ex;

  /* UnitDelay: '<S74>/Unit Delay1' */
  rtb_Swt1_o_idx_2 = LgtCtrl_DW.UnitDelay1_DSTATE_hn;

  /* Chart: '<S74>/filt' incorporates:
   *  UnitDelay: '<S74>/Unit Delay2'
   */
  /* Gateway: LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/PCC_AccPedlRateSlip2/Subsystem/filt */
  /* During: LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/PCC_AccPedlRateSlip2/Subsystem/filt */
  if (LgtCtrl_DW.is_active_c18_LgtCtrl == 0U) {
    /* Entry: LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/PCC_AccPedlRateSlip2/Subsystem/filt */
    LgtCtrl_DW.is_active_c18_LgtCtrl = 1U;

    /* Entry Internal: LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/PCC_AccPedlRateSlip2/Subsystem/filt */
    /* Transition: '<S77>:161' */
    LgtCtrl_DW.is_c18_LgtCtrl = LgtCtrl_IN_No;

    /* Entry 'No': '<S77>:160' */
    rtb_DataTypeConversion6_g = 0U;
    LgtCtrl_DW.filt_count_e = 0U;
  } else {
    switch (LgtCtrl_DW.is_c18_LgtCtrl) {
     case LgtCtrl_IN_No:
      rtb_DataTypeConversion6_g = 0U;

      /* During 'No': '<S77>:160' */
      if (LogicalOperator2 && (!SpdPln_stReleaseThrottle)) {
        /* Transition: '<S77>:163' */
        LgtCtrl_DW.is_c18_LgtCtrl = LgtCtrl_IN_Slip2PCC;

        /* Entry 'Slip2PCC': '<S77>:162' */
        rtb_DataTypeConversion6_g = 1U;
      } else if ((!LogicalOperator2) && SpdPln_stReleaseThrottle) {
        /* Transition: '<S77>:186' */
        LgtCtrl_DW.is_c18_LgtCtrl = LgtCtrl_IN_PCC2Slip;

        /* Entry 'PCC2Slip': '<S77>:175' */
        rtb_DataTypeConversion6_g = 2U;
      }
      break;

     case LgtCtrl_IN_PCC2Slip:
      rtb_DataTypeConversion6_g = 2U;

      /* During 'PCC2Slip': '<S77>:175' */
      rtb_RelationalOperator1 = !SpdPln_stReleaseThrottle;
      if ((LgtCtrl_DW.filt_count_e > 200) || rtb_RelationalOperator1 || (fabsf
           (Eng_rAccrPedlReqRaw2 - rtb_Swt1_o_idx_2) < 5.0F)) {
        /* Transition: '<S77>:187' */
        LgtCtrl_DW.is_c18_LgtCtrl = LgtCtrl_IN_No;

        /* Entry 'No': '<S77>:160' */
        rtb_DataTypeConversion6_g = 0U;
        LgtCtrl_DW.filt_count_e = 0U;
      } else if (LogicalOperator2 && rtb_RelationalOperator1) {
        /* Transition: '<S77>:189' */
        LgtCtrl_DW.filt_count_e = 0U;
        LgtCtrl_DW.is_c18_LgtCtrl = LgtCtrl_IN_Slip2PCC;

        /* Entry 'Slip2PCC': '<S77>:162' */
        rtb_DataTypeConversion6_g = 1U;
      } else {
        i = LgtCtrl_DW.filt_count_e + 1;
        if (LgtCtrl_DW.filt_count_e + 1 > 255) {
          i = 255;
        }

        LgtCtrl_DW.filt_count_e = (uint8_T)i;
      }
      break;

     default:
      rtb_DataTypeConversion6_g = 1U;

      /* During 'Slip2PCC': '<S77>:162' */
      if ((LgtCtrl_DW.filt_count_e > 200) || SpdPln_stReleaseThrottle || (fabsf
           (Eng_rAccrPedlReqRaw2 - rtb_Swt1_o_idx_2) < 5.0F)) {
        /* Transition: '<S77>:159' */
        LgtCtrl_DW.is_c18_LgtCtrl = LgtCtrl_IN_No;

        /* Entry 'No': '<S77>:160' */
        rtb_DataTypeConversion6_g = 0U;
        LgtCtrl_DW.filt_count_e = 0U;
      } else if ((!LogicalOperator2) && SpdPln_stReleaseThrottle) {
        /* Transition: '<S77>:188' */
        LgtCtrl_DW.filt_count_e = 0U;
        LgtCtrl_DW.is_c18_LgtCtrl = LgtCtrl_IN_PCC2Slip;

        /* Entry 'PCC2Slip': '<S77>:175' */
        rtb_DataTypeConversion6_g = 2U;
      } else {
        i = LgtCtrl_DW.filt_count_e + 1;
        if (LgtCtrl_DW.filt_count_e + 1 > 255) {
          i = 255;
        }

        LgtCtrl_DW.filt_count_e = (uint8_T)i;
      }
      break;
    }
  }

  /* End of Chart: '<S74>/filt' */

  /* Switch: '<S74>/Switch3' incorporates:
   *  Constant: '<S74>/Constant4'
   *  Constant: '<S74>/Constant6'
   *  Constant: '<S74>/PCC_ThroltRate_up'
   *  RelationalOperator: '<S74>/R2'
   */
  if (rtb_DataTypeConversion6_g == 1.0F) {
    rtb_KAX1EQH175T3MAPFULL = PCC_ThroltRate_up;
  } else {
    rtb_KAX1EQH175T3MAPFULL = 200.0F;
  }

  /* End of Switch: '<S74>/Switch3' */

  /* Product: '<S76>/delta rise limit' incorporates:
   *  SampleTimeMath: '<S76>/sample time'
   *
   * About '<S76>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_D1_i = rtb_KAX1EQH175T3MAPFULL * 0.01F;

  /* Sum: '<S76>/Difference Inputs1' incorporates:
   *  UnitDelay: '<S76>/Delay Input2'
   *
   * Block description for '<S76>/Difference Inputs1':
   *
   *  Add in CPU
   *
   * Block description for '<S76>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Swt1_o_idx_2 -= Eng_rAccrPedlReqRaw2;

  /* Switch: '<S74>/Switch4' incorporates:
   *  Constant: '<S74>/Constant5'
   *  Constant: '<S74>/PCC_ThroltRate_lo'
   *  Constant: '<S74>/PCC_ThroltRate_lo1'
   *  RelationalOperator: '<S74>/R1'
   */
  if (rtb_DataTypeConversion6_g == 2.0F) {
    rtb_KAX1EQH175T3MAPFULL = PCC_ThroltRate_lo;
  } else {
    rtb_KAX1EQH175T3MAPFULL = PCC_ThroltRate_lo1;
  }

  /* End of Switch: '<S74>/Switch4' */

  /* Product: '<S76>/delta fall limit' incorporates:
   *  SampleTimeMath: '<S76>/sample time'
   *
   * About '<S76>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_D1_f = rtb_KAX1EQH175T3MAPFULL * 0.01F;

  /* Switch: '<S78>/Switch2' incorporates:
   *  RelationalOperator: '<S78>/LowerRelop1'
   *  RelationalOperator: '<S78>/UpperRelop'
   *  Switch: '<S78>/Switch'
   */
  if (rtb_Swt1_o_idx_2 > rtb_D1_i) {
    rtb_Swt1_o_idx_2 = rtb_D1_i;
  } else if (rtb_Swt1_o_idx_2 < rtb_D1_f) {
    /* Switch: '<S78>/Switch' */
    rtb_Swt1_o_idx_2 = rtb_D1_f;
  }

  /* End of Switch: '<S78>/Switch2' */

  /* Sum: '<S76>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S76>/Delay Input2'
   *
   * Block description for '<S76>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S76>/Delay Input2':
   *
   *  Store in Global RAM
   */
  Eng_rAccrPedlReqRaw2 += rtb_Swt1_o_idx_2;

  /* Switch: '<S66>/Switch1' incorporates:
   *  Constant: '<S66>/Constant'
   *  RelationalOperator: '<S66>/R1'
   */
  if (SpdPln_stTarType == 15.0) {
    rtb_Swt1_o_idx_2 = Eng_rAccrPedlReqRaw2;
  } else {
    rtb_Swt1_o_idx_2 = Eng_rAccrPedlReqRaw1;
  }

  /* End of Switch: '<S66>/Switch1' */

  /* Sum: '<S73>/Difference Inputs1' incorporates:
   *  UnitDelay: '<S73>/Delay Input2'
   *
   * Block description for '<S73>/Difference Inputs1':
   *
   *  Add in CPU
   *
   * Block description for '<S73>/Delay Input2':
   *
   *  Store in Global RAM
   */
  rtb_Add_lp = rtb_Swt1_o_idx_2 - Eng_rAccrPedlReq;

  /* Switch: '<S66>/Switch2' incorporates:
   *  Constant: '<S66>/CC2PCC'
   *  Constant: '<S66>/PCC2CC'
   *  Lookup_n-D: '<S66>/CC_ThroltRateUp'
   *  RelationalOperator: '<S66>/R2'
   *  RelationalOperator: '<S66>/R3'
   *  Switch: '<S66>/Switch3'
   *  UnitDelay: '<S66>/Unit Delay2'
   */
  if (Eng_StPCC_CC_Change == 1.0) {
    rtb_D1_i = look1_iflf_binlcapw(Eng_rAccrPedlReq, CC_ThroltRate_tableX,
      CC_ThroltRate_tableY, 4U);
  } else if (Eng_StPCC_CC_Change == 2.0) {
    /* Switch: '<S66>/Switch3' incorporates:
     *  Lookup_n-D: '<S66>/CC_ThroltRateUp1'
     *  UnitDelay: '<S66>/Unit Delay2'
     */
    rtb_D1_i = look1_iflf_binlcapw(Eng_rAccrPedlReq, CC_ThroltRate_tableX,
      CC_ThroltRate_tableY, 4U);
  } else {
    /* Switch: '<S66>/Switch3' incorporates:
     *  Constant: '<S66>/PCC2CC_ThroltRate_Default'
     */
    rtb_D1_i = PCC2CC_ThroltRate_Default;
  }

  /* End of Switch: '<S66>/Switch2' */

  /* Product: '<S73>/delta rise limit' incorporates:
   *  SampleTimeMath: '<S73>/sample time'
   *
   * About '<S73>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_D1_i *= 0.01F;

  /* Product: '<S73>/delta fall limit' incorporates:
   *  Constant: '<S66>/PCC_ThroltRate_down'
   *  SampleTimeMath: '<S73>/sample time'
   *
   * About '<S73>/sample time':
   *  y = K where K = ( w * Ts )
   */
  rtb_KAX1EQH175T3MAPFULL = (-200.0F) * 0.01F;

  /* Switch: '<S75>/Switch2' incorporates:
   *  RelationalOperator: '<S75>/LowerRelop1'
   *  RelationalOperator: '<S75>/UpperRelop'
   *  Switch: '<S75>/Switch'
   */
  if (rtb_Add_lp > rtb_D1_i) {
    rtb_Add_lp = rtb_D1_i;
  } else if (rtb_Add_lp < rtb_KAX1EQH175T3MAPFULL) {
    /* Switch: '<S75>/Switch' */
    rtb_Add_lp = rtb_KAX1EQH175T3MAPFULL;
  }

  /* End of Switch: '<S75>/Switch2' */

  /* Sum: '<S73>/Difference Inputs2' incorporates:
   *  UnitDelay: '<S73>/Delay Input2'
   *
   * Block description for '<S73>/Difference Inputs2':
   *
   *  Add in CPU
   *
   * Block description for '<S73>/Delay Input2':
   *
   *  Store in Global RAM
   */
  Eng_rAccrPedlReq += rtb_Add_lp;

  /* Switch: '<S87>/Switch' incorporates:
   *  Constant: '<S87>/EBS_swtRawAccrCacl_C'
   */
  if (EBS_swtRawAccrCacl_C != 0) {
    /* Switch: '<S87>/Switch' */
    EBS_aRaw_mp = VDC_aReqNom;
  } else {
    /* Switch: '<S87>/Switch' incorporates:
     *  Constant: '<S87>/EBS_aCmpFixd_C'
     *  Sum: '<S87>/Add'
     */
    EBS_aRaw_mp = VMC_aReq + EBS_aCmpFixd_C;
  }

  /* End of Switch: '<S87>/Switch' */

  /* Switch: '<S88>/S' incorporates:
   *  Constant: '<S88>/EBS_stStpCalcSwt_C'
   */
  if (EBS_stStpCalcSwt_C != 0) {
    /* Switch: '<S88>/S' incorporates:
     *  Constant: '<S88>/EBS_aStp_C'
     */
    EBS_aStp_mp = EBS_aStp_C;
  } else {
    /* Product: '<S88>/Divide' incorporates:
     *  Constant: '<S88>/EBS_vStpCft_C'
     */
    rtb_deltariselimit_b = VehDa_vEgoSpd / EBS_vStpCft_C;

    /* Saturate: '<S88>/Saturation' */
    if (rtb_deltariselimit_b > 1.0F) {
      rtb_deltariselimit_b = 1.0F;
    } else if (rtb_deltariselimit_b < 0.0F) {
      rtb_deltariselimit_b = 0.0F;
    }

    /* End of Saturate: '<S88>/Saturation' */

    /* Switch: '<S88>/S' incorporates:
     *  Constant: '<S88>/EBS_aStpPlnBOne_C'
     *  Constant: '<S88>/EBS_aStpPlnBTwo_C'
     *  Product: '<S88>/Product'
     *  Sum: '<S88>/Add'
     *  Sum: '<S88>/Add1'
     */
    EBS_aStp_mp = (EBS_aStpPlnBOne_C - EBS_aStpPlnBTwo_C) * rtb_deltariselimit_b
      + EBS_aStpPlnBTwo_C;
  }

  /* End of Switch: '<S88>/S' */

  /* MultiPortSwitch: '<S86>/MSwt' incorporates:
   *  Constant: '<S86>/Constant'
   *  Constant: '<S86>/Constant4'
   *  Constant: '<S86>/EBS_aEmgyBrk_C'
   *  Constant: '<S86>/EBS_aTstReq_C'
   *  DataTypeConversion: '<S6>/Data Type Conversion'
   *  MinMax: '<S86>/MinMax'
   */
  switch (CoChs_stCdn) {
   case 1:
    rtb_KAX1EQH175T3MAPFULL = EBS_aEmgyBrk_C;
    break;

   case 3:
    rtb_KAX1EQH175T3MAPFULL = fminf(EBS_aRaw_mp, 3.0F);
    break;

   case 5:
    rtb_KAX1EQH175T3MAPFULL = EBS_aStp_mp;
    break;

   case 6:
    /* Switch: '<S86>/Swt1' incorporates:
     *  Constant: '<S86>/Constant7'
     *  Constant: '<S86>/Constant8'
     *  Constant: '<S86>/EBS_aStpTmp_C'
     *  Inport: '<Root>/VehDa_stEpb_mp'
     *  RelationalOperator: '<S86>/R2'
     *  RelationalOperator: '<S86>/R4'
     *  RelationalOperator: '<S86>/R5'
     *  Switch: '<S86>/Swt8'
     *  Switch: '<S86>/Swt9'
     *  UnitDelay: '<S86>/D1'
     */
    if (VehDa_stEpb_mp == 1.0F) {
      rtb_KAX1EQH175T3MAPFULL = 0.0F;
    } else if (EBS_aStpTmp_C > LgtCtrl_DW.D1_DSTATE_g) {
      /* Switch: '<S86>/Swt8' incorporates:
       *  Constant: '<S86>/CycTime'
       *  Constant: '<S86>/EBS_PrkDecnSlpUp_C'
       *  Constant: '<S86>/EBS_aStpTmp_C'
       *  MinMax: '<S86>/MinMax10'
       *  Product: '<S86>/P8'
       *  Sum: '<S86>/A6'
       *  UnitDelay: '<S86>/D1'
       */
      rtb_KAX1EQH175T3MAPFULL = fminf(EBS_PrkDecnSlpUp_C * 0.01F +
        LgtCtrl_DW.D1_DSTATE_g, EBS_aStpTmp_C);
    } else if (EBS_aStpTmp_C < LgtCtrl_DW.D1_DSTATE_g) {
      /* Switch: '<S86>/Swt9' incorporates:
       *  Constant: '<S86>/Constant3'
       *  Constant: '<S86>/CycTime'
       *  Constant: '<S86>/EBS_PrkDecnSlpUp_C'
       *  Constant: '<S86>/EBS_aStpTmp_C'
       *  MinMax: '<S86>/MinMax9'
       *  Product: '<S86>/P1'
       *  Product: '<S86>/P7'
       *  Sum: '<S86>/A5'
       *  Switch: '<S86>/Swt8'
       *  UnitDelay: '<S86>/D1'
       */
      rtb_KAX1EQH175T3MAPFULL = fmaxf(EBS_PrkDecnSlpUp_C * (-1.0F) * 0.01F +
        LgtCtrl_DW.D1_DSTATE_g, EBS_aStpTmp_C);
    } else {
      rtb_KAX1EQH175T3MAPFULL = LgtCtrl_DW.D1_DSTATE_g;
    }

    /* End of Switch: '<S86>/Swt1' */
    break;

   case 7:
    /* MultiPortSwitch: '<S47>/Multiport Switch' incorporates:
     *  Constant: '<S47>/C'
     *  Constant: '<S47>/C1'
     *  Constant: '<S4>/DRMC_aReqFf_C'
     *  Sum: '<S47>/A'
     */
    switch (LgtCtrl_B.DRMC_stMain) {
     case 0:
      rtb_KAX1EQH175T3MAPFULL = 0.0F;
      break;

     case 1:
      rtb_KAX1EQH175T3MAPFULL = (-3.0F);
      break;

     case 2:
      rtb_KAX1EQH175T3MAPFULL = 0.0F;
      break;

     case 3:
      rtb_KAX1EQH175T3MAPFULL = DRMC_aReqFf_C;
      break;

     case 4:
      /* MultiPortSwitch: '<S50>/Multiport Switch' incorporates:
       *  Constant: '<S50>/C'
       *  Constant: '<S50>/DRMC_stClsLoop_C'
       *  Gain: '<S50>/Gain'
       *  Inport: '<Root>/DRMC_lCurr'
       *  Sum: '<S50>/Add'
       *  Sum: '<S50>/Add1'
       *  Sum: '<S50>/Add2'
       */
      switch (DRMC_stClsLoop_C) {
       case 1:
        rtb_D1_i = LgtCtrl_B.DRMC_vDes_mp1 - VehDa_vEgoSpd;
        break;

       case 2:
        rtb_D1_i = LgtCtrl_B.DRMC_vDes_mp2 - VehDa_vEgoSpd;
        break;

       case 3:
        rtb_D1_i = (LgtCtrl_B.DRMC_lDes_mp3 - DRMC_lCurr) * (-1.0F);
        break;

       default:
        rtb_D1_i = 0.0F;
        break;
      }

      /* End of MultiPortSwitch: '<S50>/Multiport Switch' */

      /* Switch: '<S52>/Swt' incorporates:
       *  Constant: '<S50>/DRMC_facSpdCtrlKpCurr_C'
       *  Constant: '<S50>/DRMC_facSpdCtrlKpNegCurr_C'
       *  Constant: '<S50>/DRMC_lSpdCtrlPWinNeg_C'
       *  Constant: '<S50>/DRMC_lSpdCtrlPWinPos_C'
       *  Logic: '<S52>/L'
       *  Logic: '<S52>/L1'
       *  Product: '<S52>/Product5'
       *  Product: '<S52>/Product7'
       *  Product: '<S52>/Product9'
       *  RelationalOperator: '<S52>/R'
       *  RelationalOperator: '<S52>/R1'
       *  RelationalOperator: '<S52>/R2'
       *  RelationalOperator: '<S52>/R3'
       *  Sum: '<S52>/Sum6'
       *  Sum: '<S52>/Sum7'
       *  Switch: '<S52>/Swt1'
       */
      if ((DRMC_lSpdCtrlPWinPos_C < DRMC_lSpdCtrlPWinNeg_C) || ((rtb_D1_i <=
            DRMC_lSpdCtrlPWinPos_C) && (rtb_D1_i >= DRMC_lSpdCtrlPWinNeg_C))) {
        rtb_Swt1_o_idx_2 = rtb_D1_i * DRMC_facSpdCtrlKpCurr_C;
      } else if (rtb_D1_i > DRMC_lSpdCtrlPWinPos_C) {
        /* Switch: '<S52>/Swt1' incorporates:
         *  Constant: '<S50>/DRMC_facSpdCtrlKpCurr_C'
         *  Constant: '<S50>/DRMC_facSpdCtrlKpPosCurr_C'
         *  Product: '<S52>/Product6'
         *  Product: '<S52>/Product8'
         *  Sum: '<S52>/Sum4'
         *  Sum: '<S52>/Sum5'
         */
        rtb_Swt1_o_idx_2 = (rtb_D1_i - DRMC_lSpdCtrlPWinPos_C) *
          DRMC_facSpdCtrlKpPosCurr_C + DRMC_lSpdCtrlPWinPos_C *
          DRMC_facSpdCtrlKpCurr_C;
      } else {
        rtb_Swt1_o_idx_2 = (rtb_D1_i - DRMC_lSpdCtrlPWinNeg_C) *
          DRMC_facSpdCtrlKpNegCurr_C + DRMC_lSpdCtrlPWinNeg_C *
          DRMC_facSpdCtrlKpCurr_C;
      }

      /* End of Switch: '<S52>/Swt' */
      rtb_KAX1EQH175T3MAPFULL = DRMC_aReqFf_C + rtb_Swt1_o_idx_2;
      break;

     case 5:
      rtb_KAX1EQH175T3MAPFULL = (-3.0F);
      break;

     case 25:
      rtb_KAX1EQH175T3MAPFULL = (-3.0F);
      break;

     default:
      rtb_KAX1EQH175T3MAPFULL = 0.0F;
      break;
    }

    /* End of MultiPortSwitch: '<S47>/Multiport Switch' */
    break;

   case 10:
    rtb_KAX1EQH175T3MAPFULL = EBS_aTstReq_C;
    break;

   case 12:
    rtb_KAX1EQH175T3MAPFULL = 0.0F;
    break;

   default:
    /* Switch: '<S86>/Switch1' incorporates:
     *  Constant: '<S86>/Constant2'
     *  Constant: '<S86>/EBS_aDfl_C'
     *  RelationalOperator: '<S86>/Relational Operator1'
     */
    if (CoChs_stStarting == 1.0F) {
      /* Switch: '<S86>/Swt8' incorporates:
       *  Constant: '<S86>/CycTime'
       *  Constant: '<S86>/EBS_PrkDecnSlpUp_C'
       *  Constant: '<S86>/EBS_aStpTmp_C'
       *  MinMax: '<S86>/MinMax10'
       *  Product: '<S86>/P8'
       *  RelationalOperator: '<S86>/R4'
       *  RelationalOperator: '<S86>/R5'
       *  Sum: '<S86>/A6'
       *  Switch: '<S86>/Swt9'
       *  UnitDelay: '<S86>/D1'
       */
      if (EBS_aStpTmp_C > LgtCtrl_DW.D1_DSTATE_g) {
        rtb_KAX1EQH175T3MAPFULL = fminf(EBS_PrkDecnSlpUp_C * 0.01F +
          LgtCtrl_DW.D1_DSTATE_g, EBS_aStpTmp_C);
      } else if (EBS_aStpTmp_C < LgtCtrl_DW.D1_DSTATE_g) {
        /* Switch: '<S86>/Swt9' incorporates:
         *  Constant: '<S86>/Constant3'
         *  Constant: '<S86>/CycTime'
         *  Constant: '<S86>/EBS_PrkDecnSlpUp_C'
         *  MinMax: '<S86>/MinMax9'
         *  Product: '<S86>/P1'
         *  Product: '<S86>/P7'
         *  Sum: '<S86>/A5'
         */
        rtb_KAX1EQH175T3MAPFULL = fmaxf(EBS_PrkDecnSlpUp_C * (-1.0F) * 0.01F +
          LgtCtrl_DW.D1_DSTATE_g, EBS_aStpTmp_C);
      } else {
        rtb_KAX1EQH175T3MAPFULL = LgtCtrl_DW.D1_DSTATE_g;
      }
    } else {
      rtb_KAX1EQH175T3MAPFULL = EBS_aDfl_C;
    }

    /* End of Switch: '<S86>/Switch1' */
    break;
  }

  /* End of MultiPortSwitch: '<S86>/MSwt' */

  /* Switch: '<S86>/Switch' incorporates:
   *  Constant: '<S86>/ABE_BHV_ID'
   *  Constant: '<S86>/MIN_DECELERATION'
   *  Constant: '<S86>/MIN_DECELERATION_NORMAL'
   *  DataTypeConversion: '<S6>/Data Type Conversion1'
   *  RelationalOperator: '<S86>/Relational Operator'
   */
  if (BhvCrdn_numBhvID == 8.0) {
    rtb_D1_i = (-8.0F);
  } else {
    rtb_D1_i = (-5.0F);
  }

  /* End of Switch: '<S86>/Switch' */

  /* Switch: '<S89>/Switch2' incorporates:
   *  Constant: '<S86>/MAX_DECELERATION'
   *  RelationalOperator: '<S89>/LowerRelop1'
   *  RelationalOperator: '<S89>/UpperRelop'
   *  Switch: '<S89>/Switch'
   */
  if (rtb_KAX1EQH175T3MAPFULL > 4.0F) {
    rtb_D1_i = 4.0F;
  } else if (rtb_KAX1EQH175T3MAPFULL >= rtb_D1_i) {
    rtb_D1_i = rtb_KAX1EQH175T3MAPFULL;
  }

  /* End of Switch: '<S89>/Switch2' */

  /* Switch: '<S86>/Switch2' incorporates:
   *  Constant: '<S86>/PCC_Type1_C'
   *  Constant: '<S86>/PCC_Type_C'
   *  Logic: '<S86>/Logical Operator5'
   *  RelationalOperator: '<S86>/R1'
   *  RelationalOperator: '<S86>/R3'
   */
  if ((SpdPln_stTarType == 15.0) && (SpdPln_pcc_tar_throttle > 0.0)) {
    /* Switch: '<S86>/Switch2' incorporates:
     *  Constant: '<S86>/PCC_Type2_C'
     */
    EBS_aReq = 0.0F;
  } else {
    /* Switch: '<S86>/Switch2' */
    EBS_aReq = rtb_D1_i;
  }

  /* End of Switch: '<S86>/Switch2' */

  /* Switch: '<S93>/Swt4' incorporates:
   *  Constant: '<S93>/C4'
   *  Constant: '<S93>/C6'
   *  Constant: '<S93>/C9'
   *  RelationalOperator: '<S93>/R1'
   *  Sum: '<S93>/Add1'
   *  UnitDelay: '<S93>/U1'
   */
  if (LgtCtrl_DW.U1_DSTATE < 4.0) {
    rtb_UnitDelay_lk = 1.0 + LgtCtrl_DW.U1_DSTATE;
  } else {
    rtb_UnitDelay_lk = 0.0;
  }

  /* End of Switch: '<S93>/Swt4' */

  /* Switch: '<S96>/Swt4' incorporates:
   *  Constant: '<S96>/C4'
   *  Constant: '<S96>/C9'
   *  RelationalOperator: '<S96>/R1'
   *  Sum: '<S96>/Add1'
   *  UnitDelay: '<S96>/U1'
   */
  if (LgtCtrl_DW.U1_DSTATE_l < LgtCtrl_ConstB.Swt3) {
    rtb_Sum6 = 1.0 + LgtCtrl_DW.U1_DSTATE_l;
  } else {
    rtb_Sum6 = 0.0;
  }

  /* End of Switch: '<S96>/Swt4' */

  /* Sum: '<S124>/Sum1' incorporates:
   *  UnitDelay: '<S124>/Unit Delay1'
   */
  rtb_KAX1EQH175T3MAPFULL = VMC_vDif_mp - LgtCtrl_DW.UnitDelay1_DSTATE_c;

  /* Product: '<S124>/Product' incorporates:
   *  Constant: '<S121>/VMC_facAccnGvnrKdCurr_C'
   */
  rtb_deltariselimit_b = VMC_facAccnGvnrKdCurr_C * rtb_KAX1EQH175T3MAPFULL;

  /* Switch: '<S124>/Swt2' incorporates:
   *  Constant: '<S121>/VMC_vAccnGvnrDWinNeg_C'
   *  Constant: '<S121>/VMC_vAccnGvnrDWinPos_C'
   *  RelationalOperator: '<S124>/R'
   *  RelationalOperator: '<S124>/R1'
   *  Switch: '<S124>/Swt'
   *  Switch: '<S124>/Swt1'
   */
  if (VMC_stAccnGvnrIntglIni_mp) {
    /* Update for UnitDelay: '<S124>/Unit Delay' incorporates:
     *  Constant: '<S119>/VMC_aAccnGvnrDftlIniVal_C'
     */
    LgtCtrl_DW.UnitDelay_DSTATE_h = VMC_aAccnGvnrDftlIniVal_C;
  } else {
    if (rtb_KAX1EQH175T3MAPFULL > VMC_vAccnGvnrDWinPos_C) {
      /* Switch: '<S124>/Swt' incorporates:
       *  Constant: '<S121>/VMC_facAccnGvnrKdPosCurr_C'
       *  Constant: '<S121>/VMC_vAccnGvnrDWinPos_C'
       *  Product: '<S124>/Product2'
       *  Sum: '<S124>/Sum2'
       *  Sum: '<S124>/Sum4'
       */
      rtb_deltariselimit_b += (rtb_KAX1EQH175T3MAPFULL - VMC_vAccnGvnrDWinPos_C)
        * VMC_facAccnGvnrKdPosCurr_C;
    } else if (rtb_KAX1EQH175T3MAPFULL < VMC_vAccnGvnrDWinNeg_C) {
      /* Switch: '<S124>/Swt' incorporates:
       *  Constant: '<S121>/VMC_facAccnGvnrKdNegCurr_C'
       *  Constant: '<S121>/VMC_vAccnGvnrDWinNeg_C'
       *  Product: '<S124>/Product3'
       *  Sum: '<S124>/Sum3'
       *  Sum: '<S124>/Sum5'
       *  Switch: '<S124>/Swt1'
       */
      rtb_deltariselimit_b += (rtb_KAX1EQH175T3MAPFULL - VMC_vAccnGvnrDWinNeg_C)
        * VMC_facAccnGvnrKdNegCurr_C;
    }

    /* Update for UnitDelay: '<S124>/Unit Delay' incorporates:
     *  Constant: '<S119>/VMC_facAccnGvnrDftlRatio_C'
     *  Constant: '<S121>/VMC_vAccnGvnrDWinNeg_C'
     *  Product: '<S124>/Product1'
     *  RelationalOperator: '<S124>/R1'
     *  Sum: '<S124>/Sum'
     *  Switch: '<S124>/Swt'
     *  Switch: '<S124>/Swt1'
     */
    LgtCtrl_DW.UnitDelay_DSTATE_h = VMC_facAccnGvnrDftlRatio_C *
      LgtCtrl_DW.UnitDelay_DSTATE_h + rtb_deltariselimit_b;
  }

  /* End of Switch: '<S124>/Swt2' */

  /* DataTypeConversion: '<S14>/Data Type Conversion12' incorporates:
   *  Gain: '<S149>/Gain'
   *  Inport: '<Root>/VehDa_stSrcBrk_mp'
   */
  VehDa_stSrcBrk = (uint8_T)(((uint32_T)((uint8_T)128U) * VehDa_stSrcBrk_mp) >>
    7);

  /* Gain: '<S162>/Gain' incorporates:
   *  Inport: '<Root>/VehDa_pFrontLeft_mp'
   */
  VehDa_pFrontLeft = 1.0F * VehDa_pFrontLeft_mp;

  /* Update for UnitDelay: '<S15>/Unit Delay' */
  LgtCtrl_DW.UnitDelay_DSTATE = rtb_Switch;

  /* Update for UnitDelay: '<S20>/D2' */
  LgtCtrl_DW.D2_DSTATE = rtb_Swt2;

  /* Update for UnitDelay: '<S20>/D1' */
  LgtCtrl_DW.D1_DSTATE = rtb_Swt1;

  /* Update for UnitDelay: '<S16>/Unit Delay1' */
  LgtCtrl_DW.UnitDelay1_DSTATE_fs = rtb_sampletime;

  /* Update for UnitDelay: '<S16>/Unit Delay' */
  LgtCtrl_DW.UnitDelay_DSTATE_ih = rtb_LogicalOperator3;

  /* DataTypeConversion: '<S2>/Data Type Conversion8' */
  rtb_Swt1_o_idx_2 = fmodf(floorf(LgtCtrl_B.Switch_o), 256.0F);

  /* Update for UnitDelay: '<S2>/Unit Delay1' incorporates:
   *  DataTypeConversion: '<S2>/Data Type Conversion8'
   */
  LgtCtrl_DW.UnitDelay1_DSTATE_gm = (uint8_T)(rtb_Swt1_o_idx_2 < 0.0F ? (int32_T)
    (uint8_T)-(int8_T)(uint8_T)-rtb_Swt1_o_idx_2 : (int32_T)(uint8_T)
    rtb_Swt1_o_idx_2);

  /* DataTypeConversion: '<S2>/Data Type Conversion5' */
  rtb_Swt1_o_idx_2 = fmodf(floorf(LgtCtrl_B.MultiportSwitch1), 256.0F);

  /* Update for UnitDelay: '<S1>/Unit Delay1' incorporates:
   *  DataTypeConversion: '<S2>/Data Type Conversion5'
   */
  LgtCtrl_DW.UnitDelay1_DSTATE_dg = (uint8_T)(rtb_Swt1_o_idx_2 < 0.0F ? (int32_T)
    (uint8_T)-(int8_T)(uint8_T)-rtb_Swt1_o_idx_2 : (int32_T)(uint8_T)
    rtb_Swt1_o_idx_2);

  /* Update for UnitDelay: '<S1>/Unit Delay2' incorporates:
   *  Gain: '<S2>/Gain'
   */
  LgtCtrl_DW.UnitDelay2_DSTATE_i = 0.277777791F * LgtCtrl_B.Swt4;

  /* Update for UnitDelay: '<S141>/Unit Delay1' incorporates:
   *  Constant: '<S141>/C1 '
   */
  LgtCtrl_DW.UnitDelay1_DSTATE = 1.0;

  /* Update for UnitDelay: '<S141>/U4' */
  LgtCtrl_DW.U4_DSTATE = SpdPln_lTrgLngErr;

  /* Update for UnitDelay: '<S141>/U5' */
  LgtCtrl_DW.U5_DSTATE = rtb_Gain;

  /* Update for UnitDelay: '<S138>/Unit Delay1' */
  LgtCtrl_DW.UnitDelay1_DSTATE_n = rtb_Gain;

  /* Update for UnitDelay: '<S138>/Unit Delay' */
  LgtCtrl_DW.UnitDelay_DSTATE_as = UnitDelay;

  /* Update for UnitDelay: '<S139>/Unit Delay' */
  LgtCtrl_DW.UnitDelay_DSTATE_i = rtb_Swt1_a;

  /* Update for UnitDelay: '<S145>/Unit Delay' */
  LgtCtrl_DW.UnitDelay_DSTATE_c = rtb_Switch_cx;

  /* Update for UnitDelay: '<S128>/Unit Delay1' incorporates:
   *  Constant: '<S128>/C1 '
   */
  LgtCtrl_DW.UnitDelay1_DSTATE_d = 1.0;

  /* Update for UnitDelay: '<S128>/Unit Delay4' */
  LgtCtrl_DW.UnitDelay4_DSTATE = Switch;

  /* Update for UnitDelay: '<S122>/Unit Delay' */
  LgtCtrl_DW.UnitDelay_DSTATE_a = rtb_MinMax;

  /* Update for UnitDelay: '<S12>/Unit Delay' */
  LgtCtrl_DW.UnitDelay_DSTATE_d = VMC_vDif_mp;

  /* Update for UnitDelay: '<S112>/Unit Delay4' */
  LgtCtrl_DW.UnitDelay4_DSTATE_e = ACCS_stActvd;

  /* Update for UnitDelay: '<S112>/Unit Delay5' */
  LgtCtrl_DW.UnitDelay5_DSTATE_l = rtb_A_iw;

  /* Update for UnitDelay: '<S113>/Unit Delay1' */
  LgtCtrl_DW.UnitDelay1_DSTATE_h = rtb_A_iw;

  /* Update for UnitDelay: '<S113>/Unit Delay' */
  LgtCtrl_DW.UnitDelay_DSTATE_af = LgtDstRel;

  /* Update for UnitDelay: '<S114>/Unit Delay' */
  LgtCtrl_DW.UnitDelay_DSTATE_io = rtb_Swt1_o_idx_4;

  /* Update for UnitDelay: '<S107>/Delay Input2'
   *
   * Block description for '<S107>/Delay Input2':
   *
   *  Store in Global RAM
   */
  LgtCtrl_DW.DelayInput2_DSTATE = rtb_deltariselimit;

  /* Update for UnitDelay: '<S43>/Unit Delay1' */
  LgtCtrl_DW.UnitDelay1_DSTATE_p = rtb_Switch1_b;

  /* Update for UnitDelay: '<S42>/Unit Delay1' */
  LgtCtrl_DW.UnitDelay1_DSTATE_pb = rtb_Switch1_i;

  /* Update for UnitDelay: '<S48>/Unit Delay' */
  LgtCtrl_DW.UnitDelay_DSTATE_du = rtb_deltafalllimit;

  /* DataTypeConversion: '<S2>/Data Type Conversion4' */
  rtb_Swt1_o_idx_2 = fmodf(floorf(LgtCtrl_B.MultiportSwitch), 256.0F);

  /* Update for UnitDelay: '<S1>/Unit Delay4' incorporates:
   *  DataTypeConversion: '<S2>/Data Type Conversion4'
   */
  LgtCtrl_DW.UnitDelay4_DSTATE_j = (uint8_T)(rtb_Swt1_o_idx_2 < 0.0F ? (int32_T)
    (uint8_T)-(int8_T)(uint8_T)-rtb_Swt1_o_idx_2 : (int32_T)(uint8_T)
    rtb_Swt1_o_idx_2);

  /* Update for UnitDelay: '<S48>/Unit Delay1' */
  LgtCtrl_DW.UnitDelay1_DSTATE_nl = rtb_Switch1;

  /* Update for UnitDelay: '<S1>/Unit Delay11' */
  LgtCtrl_DW.UnitDelay11_DSTATE = CoChs_stCdn;

  /* Update for UnitDelay: '<S63>/UD1' incorporates:
   *  DataTypeConversion: '<S5>/Data Type Conversion1'
   */
  LgtCtrl_DW.UD1_DSTATE = CoChs_stCdn;

  /* Update for UnitDelay: '<S64>/Unit Delay1' incorporates:
   *  Constant: '<S64>/C1 '
   */
  LgtCtrl_DW.UnitDelay1_DSTATE_f = 1.0;

  /* Update for UnitDelay: '<S64>/Unit Delay4' */
  LgtCtrl_DW.UnitDelay4_DSTATE_d = rtb_Swt_lo;

  /* Update for UnitDelay: '<S64>/Unit Delay5' */
  LgtCtrl_DW.UnitDelay5_DSTATE_o = rtb_Swt_ot;

  /* Update for UnitDelay: '<S63>/UD' */
  LgtCtrl_DW.UD_DSTATE = rtb_MinMax2_h;

  /* Update for UnitDelay: '<S66>/Unit Delay1' */
  LgtCtrl_DW.UnitDelay1_DSTATE_n1 = SpdPln_stTarType;

  /* Update for UnitDelay: '<S69>/Unit Delay' incorporates:
   *  DataTypeConversion: '<S5>/Data Type Conversion1'
   */
  LgtCtrl_DW.UnitDelay_DSTATE_no = CoChs_stCdn;

  /* Update for UnitDelay: '<S69>/Unit Delay1' */
  LgtCtrl_DW.UnitDelay1_DSTATE_k = Eng_rAccrPedlReqRaw;

  /* Update for UnitDelay: '<S74>/Unit Delay' */
  LgtCtrl_DW.UnitDelay_DSTATE_ex = SpdPln_stReleaseThrottle;

  /* Update for UnitDelay: '<S74>/Unit Delay1' */
  LgtCtrl_DW.UnitDelay1_DSTATE_hn = SpdPln_pcc_tar_throttle;

  /* Update for UnitDelay: '<S86>/D1' incorporates:
   *  Constant: '<S86>/Constant1'
   *  MinMax: '<S86>/MinMax1'
   */
  LgtCtrl_DW.D1_DSTATE_g = fminf(rtb_D1_i, -0.5F);

  /* Switch: '<S97>/Switch' incorporates:
   *  Constant: '<S96>/C4'
   *  Constant: '<S97>/C1'
   *  Constant: '<S97>/C7'
   *  RelationalOperator: '<S96>/R2'
   */
  if (rtb_Sum6 == 0.0) {
    rtb_Switch = 1.0;
  } else {
    rtb_Switch = 0.0;
  }

  /* End of Switch: '<S97>/Switch' */

  /* Update for UnitDelay: '<S97>/Unit Delay' incorporates:
   *  Constant: '<S97>/C8'
   *  Math: '<S97>/Math'
   *  Sum: '<S97>/Sum6'
   */
  LgtCtrl_DW.UnitDelay_DSTATE_n = rt_modd(rtb_Switch +
    LgtCtrl_DW.UnitDelay_DSTATE_n, 16.0);

  /* Switch: '<S91>/Switch1' incorporates:
   *  Constant: '<S91>/C6'
   *  Constant: '<S91>/C8'
   *  Constant: '<S93>/C4'
   *  RelationalOperator: '<S93>/R2'
   */
  if (rtb_UnitDelay_lk == 0.0) {
    rtb_Switch = 1.0;
  } else {
    rtb_Switch = 0.0;
  }

  /* End of Switch: '<S91>/Switch1' */

  /* Update for UnitDelay: '<S91>/Unit Delay' incorporates:
   *  Constant: '<S91>/C9'
   *  Math: '<S91>/Math'
   *  Sum: '<S91>/Sum6'
   */
  LgtCtrl_DW.UnitDelay_DSTATE_o = rt_modd(rtb_Switch +
    LgtCtrl_DW.UnitDelay_DSTATE_o, 250.0);

  /* Update for UnitDelay: '<S93>/U1' */
  LgtCtrl_DW.U1_DSTATE = rtb_UnitDelay_lk;

  /* Update for UnitDelay: '<S96>/U1' */
  LgtCtrl_DW.U1_DSTATE_l = rtb_Sum6;

  /* Update for UnitDelay: '<S124>/Unit Delay1' */
  LgtCtrl_DW.UnitDelay1_DSTATE_c = VMC_vDif_mp;
}

/* Model initialize function */
void LgtCtrl_initialize(void)
{
  /* SystemInitialize for Enabled SubSystem: '<S2>/ACCS_DesSpdCalcn' */

  /* SystemInitialize for Chart: '<S22>/FeedForwardAccrCalcn_100ms1' */
  FeedForwardAccrCalcn_100ms_Init();

  /* SystemInitialize for Chart: '<S29>/FeedForwardAccrCalcn_100ms1' */
  FeedForwardAccrCalcn_100ms_Init();

  /* End of SystemInitialize for SubSystem: '<S2>/ACCS_DesSpdCalcn' */

  /* SystemInitialize for Chart: '<S38>/ConCfm' */
  LgtCtrl_DW.EmgBrkClsCtrl = 1U;

  /* SystemInitialize for Chart: '<S69>/filt' */
  LgtCtrl_DW.filt_count = 1U;

  /* SystemInitialize for Chart: '<S66>/PCC_CC_ChangeStateCal' */
  LgtCtrl_DW.Eng_tTimerCount = 1U;

  /* SystemInitialize for Chart: '<S74>/filt' */
  LgtCtrl_DW.filt_count_e = 1U;
}

/* Model terminate function */
void LgtCtrl_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
