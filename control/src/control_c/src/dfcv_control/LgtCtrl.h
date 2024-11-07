/*
 * File: LgtCtrl.h
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

#ifndef RTW_HEADER_LgtCtrl_h_
#define RTW_HEADER_LgtCtrl_h_
#include <math.h>
#ifndef LgtCtrl_COMMON_INCLUDES_
#define LgtCtrl_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* LgtCtrl_COMMON_INCLUDES_ */

/* Model Code Variants */

/* Macros for accessing real-time model data structure */

/* Block states (default storage) for system '<S22>/FeedForwardAccrCalcn_100ms1' */
typedef struct {
  real32_T SpeedAry[10];               /* '<S22>/FeedForwardAccrCalcn_100ms1' */
  real32_T index;                      /* '<S22>/FeedForwardAccrCalcn_100ms1' */
  real32_T counter;                    /* '<S22>/FeedForwardAccrCalcn_100ms1' */
} DW_FeedForwardAccrCalcn_100ms_T;

/* Block signals (default storage) */
typedef struct {
  real32_T output;                     /* '<S120>/debounce' */
  real32_T TrqExe_rEngThrReqd;         /* '<S63>/ThrCalc_KAX1' */
  real32_T TrqExe_rEngThrReqd_a;       /* '<S63>/ThrCalc_H19E' */
  real32_T DRMC_vDes_mp1;              /* '<S50>/SpdRamp' */
  real32_T DRMC_lDes_mp3;              /* '<S50>/SpdRamp' */
  real32_T DRMC_vDes_mp2;              /* '<S50>/S_V' */
  real32_T Switch2;                    /* '<S35>/Switch2' */
  real32_T Switch1;                    /* '<S35>/Switch1' */
  real32_T ACCS_vSet;                  /* '<S36>/Chart' */
  real32_T ACCS_stTimeGap;             /* '<S36>/Chart' */
  real32_T MultiportSwitch;            /* '<S33>/Multiport Switch' */
  real32_T MultiportSwitch1;           /* '<S33>/Multiport Switch1' */
  real32_T ACCS_stActvdPls;            /* '<S33>/StMng' */
  real32_T Switch_o;                   /* '<S21>/Switch' */
  real32_T S;                          /* '<S21>/S' */
  real32_T Swt4;                       /* '<S22>/Swt4' */
  real32_T Swt12;                      /* '<S22>/Swt12' */
  real32_T TrgtSpd;                    /* '<S21>/Dst2Spd' */
  real32_T TrgtAccr;                   /* '<S21>/Dst2Spd' */
  uint8_T DRMC_stMain;                 /* '<S4>/DRMC_MainState' */
} B_LgtCtrl_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T UnitDelay_DSTATE;             /* '<S15>/Unit Delay' */
  real_T UnitDelay1_DSTATE;            /* '<S141>/Unit Delay1' */
  real_T UnitDelay_DSTATE_c;           /* '<S145>/Unit Delay' */
  real_T UnitDelay1_DSTATE_d;          /* '<S128>/Unit Delay1' */
  real_T UnitDelay_DSTATE_a;           /* '<S122>/Unit Delay' */
  real_T UnitDelay1_DSTATE_f;          /* '<S64>/Unit Delay1' */
  real_T UnitDelay_DSTATE_n;           /* '<S97>/Unit Delay' */
  real_T UnitDelay_DSTATE_o;           /* '<S91>/Unit Delay' */
  real_T U1_DSTATE;                    /* '<S93>/U1' */
  real_T U1_DSTATE_l;                  /* '<S96>/U1' */
  real_T UnitDelay2_DSTATE;            /* '<S165>/Unit Delay2' */
  real_T UnitDelay1_DSTATE_g;          /* '<S133>/Unit Delay1' */
  real32_T D2_DSTATE;                  /* '<S20>/D2' */
  real32_T D1_DSTATE;                  /* '<S20>/D1' */
  real32_T UnitDelay1_DSTATE_fs;       /* '<S16>/Unit Delay1' */
  real32_T UnitDelay2_DSTATE_i;        /* '<S1>/Unit Delay2' */
  real32_T U4_DSTATE;                  /* '<S141>/U4' */
  real32_T U5_DSTATE;                  /* '<S141>/U5' */
  real32_T UnitDelay1_DSTATE_n;        /* '<S138>/Unit Delay1' */
  real32_T UnitDelay_DSTATE_as;        /* '<S138>/Unit Delay' */
  real32_T UnitDelay_DSTATE_i;         /* '<S139>/Unit Delay' */
  real32_T UnitDelay4_DSTATE;          /* '<S128>/Unit Delay4' */
  real32_T UnitDelay_DSTATE_d;         /* '<S12>/Unit Delay' */
  real32_T UnitDelay4_DSTATE_e;        /* '<S112>/Unit Delay4' */
  real32_T UnitDelay5_DSTATE_l;        /* '<S112>/Unit Delay5' */
  real32_T UnitDelay1_DSTATE_h;        /* '<S113>/Unit Delay1' */
  real32_T UnitDelay_DSTATE_af;        /* '<S113>/Unit Delay' */
  real32_T UnitDelay_DSTATE_io;        /* '<S114>/Unit Delay' */
  real32_T DelayInput2_DSTATE;         /* '<S107>/Delay Input2' */
  real32_T UnitDelay_DSTATE_du;        /* '<S48>/Unit Delay' */
  real32_T UnitDelay1_DSTATE_nl;       /* '<S48>/Unit Delay1' */
  real32_T UnitDelay4_DSTATE_d;        /* '<S64>/Unit Delay4' */
  real32_T UnitDelay5_DSTATE_o;        /* '<S64>/Unit Delay5' */
  real32_T UD_DSTATE;                  /* '<S63>/UD' */
  real32_T UnitDelay1_DSTATE_k;        /* '<S69>/Unit Delay1' */
  real32_T UnitDelay1_DSTATE_hn;       /* '<S74>/Unit Delay1' */
  real32_T D1_DSTATE_g;                /* '<S86>/D1' */
  real32_T UnitDelay1_DSTATE_c;        /* '<S124>/Unit Delay1' */
  real32_T UnitDelay_DSTATE_h;         /* '<S124>/Unit Delay' */
  real32_T UnitDelay1_DSTATE_ft;       /* '<S165>/Unit Delay1' */
  real32_T UnitDelay4_DSTATE_f;        /* '<S165>/Unit Delay4' */
  real32_T UnitDelay5_DSTATE_b;        /* '<S165>/Unit Delay5' */
  real32_T UnitDelay_DSTATE_p;         /* '<S165>/Unit Delay' */
  real32_T UnitDelay3_DSTATE_g;        /* '<S130>/Unit Delay3' */
  real32_T UnitDelay4_DSTATE_b;        /* '<S133>/Unit Delay4' */
  real32_T UnitDelay1_DSTATE_gz;       /* '<S22>/Unit Delay1' */
  real32_T UnitDelay2_DSTATE_o;        /* '<S22>/Unit Delay2' */
  real32_T DRMC_ctPUB_mp;              /* '<S4>/DRMC_MainState' */
  real32_T DRMC_ctPrep_mp;             /* '<S4>/DRMC_MainState' */
  real32_T ACCS_stFirstActvd;          /* '<S33>/StMng' */
  real32_T SpeedAry[10];               /* '<S28>/FeedForwardAccrCalcn_100ms' */
  real32_T index;                      /* '<S28>/FeedForwardAccrCalcn_100ms' */
  real32_T counter;                    /* '<S28>/FeedForwardAccrCalcn_100ms' */
  real32_T SpeedAry_g[10];             /* '<S22>/FeedForwardAccrCalcn_100ms' */
  real32_T index_f;                    /* '<S22>/FeedForwardAccrCalcn_100ms' */
  real32_T counter_d;                  /* '<S22>/FeedForwardAccrCalcn_100ms' */
  real32_T SpeedAry_p[10];             /* '<S22>/DesAccrCalcn' */
  real32_T index_b;                    /* '<S22>/DesAccrCalcn' */
  real32_T counter_p;                  /* '<S22>/DesAccrCalcn' */
  uint16_T UnitDelay1_DSTATE_p;        /* '<S43>/Unit Delay1' */
  uint16_T UnitDelay1_DSTATE_pb;       /* '<S42>/Unit Delay1' */
  uint16_T Eng_tTimerCount;            /* '<S66>/PCC_CC_ChangeStateCal' */
  int8_T UnitDelay1_DSTATE_n1;         /* '<S66>/Unit Delay1' */
  uint8_T UnitDelay1_DSTATE_gm;        /* '<S2>/Unit Delay1' */
  uint8_T UnitDelay1_DSTATE_dg;        /* '<S1>/Unit Delay1' */
  uint8_T UnitDelay4_DSTATE_j;         /* '<S1>/Unit Delay4' */
  uint8_T UnitDelay11_DSTATE;          /* '<S1>/Unit Delay11' */
  uint8_T UD1_DSTATE;                  /* '<S63>/UD1' */
  uint8_T UnitDelay_DSTATE_no;         /* '<S69>/Unit Delay' */
  uint8_T UnitDelay_DSTATE_d0;         /* '<S32>/Unit Delay' */
  uint8_T UnitDelay1_DSTATE_b;         /* '<S32>/Unit Delay1' */
  uint8_T UnitDelay2_DSTATE_l;         /* '<S32>/Unit Delay2' */
  uint8_T UnitDelay3_DSTATE_b;         /* '<S32>/Unit Delay3' */
  uint8_T UnitDelay4_DSTATE_bm;        /* '<S32>/Unit Delay4' */
  uint8_T UnitDelay5_DSTATE_a;         /* '<S32>/Unit Delay5' */
  boolean_T UnitDelay_DSTATE_ih;       /* '<S16>/Unit Delay' */
  boolean_T UnitDelay_DSTATE_ex;       /* '<S74>/Unit Delay' */
  uint8_T is_active_c8_LgtCtrl;        /* '<S120>/debounce' */
  uint8_T is_c8_LgtCtrl;               /* '<S120>/debounce' */
  uint8_T counter_dy;                  /* '<S120>/debounce' */
  uint8_T is_active_c2_LgtCtrl;        /* '<S10>/TrqLim' */
  uint8_T is_c2_LgtCtrl;               /* '<S10>/TrqLim' */
  uint8_T is_active_c1_LgtCtrl;        /* '<S10>/ShiftJudg' */
  uint8_T is_c1_LgtCtrl;               /* '<S10>/ShiftJudg' */
  uint8_T is_active_c10_LgtCtrl;       /* '<S1>/StartingJudge' */
  uint8_T is_c10_LgtCtrl;              /* '<S1>/StartingJudge' */
  uint8_T is_active_c12_LgtCtrl;       /* '<S69>/filt' */
  uint8_T is_c12_LgtCtrl;              /* '<S69>/filt' */
  uint8_T filt_count;                  /* '<S69>/filt' */
  uint8_T is_active_c18_LgtCtrl;       /* '<S74>/filt' */
  uint8_T is_c18_LgtCtrl;              /* '<S74>/filt' */
  uint8_T filt_count_e;                /* '<S74>/filt' */
  uint8_T is_active_c11_LgtCtrl;       /* '<S66>/PCC_CC_ChangeStateCal' */
  uint8_T is_c11_LgtCtrl;              /* '<S66>/PCC_CC_ChangeStateCal' */
  uint8_T is_active_c4_LgtCtrl;        /* '<S63>/GoJudg' */
  uint8_T is_c4_LgtCtrl;               /* '<S63>/GoJudg' */
  uint8_T is_active_c17_LgtCtrl;       /* '<S50>/SpdRamp' */
  uint8_T is_c17_LgtCtrl;              /* '<S50>/SpdRamp' */
  uint8_T is_active_c16_LgtCtrl;       /* '<S50>/S_V' */
  uint8_T is_c16_LgtCtrl;              /* '<S50>/S_V' */
  uint8_T is_active_c15_LgtCtrl;       /* '<S4>/DRMC_MainState' */
  uint8_T is_c15_LgtCtrl;              /* '<S4>/DRMC_MainState' */
  uint8_T is_Normal;                   /* '<S4>/DRMC_MainState' */
  uint8_T is_Actived;                  /* '<S4>/DRMC_MainState' */
  uint8_T is_active_c13_LgtCtrl;       /* '<S39>/CoChs_stCtlSysCfm' */
  uint8_T is_c13_LgtCtrl;              /* '<S39>/CoChs_stCtlSysCfm' */
  uint8_T EmgBrkClsCtrl;               /* '<S38>/ConCfm' */
  uint8_T is_active_c35_LgtCtrl;       /* '<S33>/StMng' */
  uint8_T is_c35_LgtCtrl;              /* '<S33>/StMng' */
  uint8_T is_Working;                  /* '<S33>/StMng' */
  uint8_T is_Active;                   /* '<S33>/StMng' */
  DW_FeedForwardAccrCalcn_100ms_T sf_FeedForwardAccrCalcn_100ms_b;/* '<S29>/FeedForwardAccrCalcn_100ms1' */
  DW_FeedForwardAccrCalcn_100ms_T sf_FeedForwardAccrCalcn_100ms1;/* '<S22>/FeedForwardAccrCalcn_100ms1' */
} DW_LgtCtrl_T;

/* Invariant block signals (default storage) */
typedef struct {
  const real_T G1;                     /* '<S90>/G1' */
  const real_T Add;                    /* '<S90>/Add' */
  const real_T Swt3;                   /* '<S96>/Swt3' */
  const real_T Divide2;                /* '<S97>/Divide2' */
  const real_T P1;                     /* '<S125>/P1' */
  const real_T P2;                     /* '<S125>/P2' */
  const real32_T Product8;             /* '<S115>/Product8' */
  const real32_T Product9;             /* '<S115>/Product9' */
  const uint8_T DataTypeConversion3;   /* '<S7>/Data Type Conversion3' */
  const uint8_T DataTypeConversion12;  /* '<S97>/Data Type Conversion12' */
  const uint8_T DataTypeConversion13;  /* '<S97>/Data Type Conversion13' */
  const uint8_T DataTypeConversion14;  /* '<S97>/Data Type Conversion14' */
  const uint8_T DataTypeConversion16;  /* '<S97>/Data Type Conversion16' */
  const uint8_T DataTypeConversion7;   /* '<S97>/Data Type Conversion7' */
  const uint8_T DataTypeConversion8;   /* '<S97>/Data Type Conversion8' */
  const uint8_T DataTypeConversion9;   /* '<S97>/Data Type Conversion9' */
  const boolean_T R;                   /* '<S62>/R' */
  const boolean_T R3;                  /* '<S117>/R3' */
  const boolean_T R2;                  /* '<S114>/R2' */
  const boolean_T R2_h;                /* '<S115>/R2' */
  const boolean_T R3_b;                /* '<S118>/R3' */
  const boolean_T L3;                  /* '<S110>/L3' */
} ConstB_LgtCtrl_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Computed Parameter: uDLookupTable_tableData
   * Referenced by: '<S103>/1-D Lookup Table'
   */
  real32_T uDLookupTable_tableData[181];

  /* Computed Parameter: uDLookupTable_bp01Data
   * Referenced by: '<S103>/1-D Lookup Table'
   */
  real32_T uDLookupTable_bp01Data[181];

  /* Computed Parameter: H19EDci45051MAPA_tableData
   * Referenced by: '<S63>/H19E-Dci450-51-MAPA'
   */
  real32_T H19EDci45051MAPA_tableData[220];

  /* Computed Parameter: H19EDci45051MAPA_bp01Data
   * Referenced by: '<S63>/H19E-Dci450-51-MAPA'
   */
  real32_T H19EDci45051MAPA_bp01Data[22];

  /* Pooled Parameter (Expression: [0	2	10	20	40	50	60	70	95	100])
   * Referenced by:
   *   '<S63>/Thr_H19E'
   *   '<S63>/H19E-Dci450-51-MAPA'
   */
  real32_T pooled39[10];

  /* Computed Parameter: KAX1EQH175T3MAPFULL_tableData
   * Referenced by: '<S63>/KAX1-EQH175-T3-MAP-FULL'
   */
  real32_T KAX1EQH175T3MAPFULL_tableData[24];

  /* Computed Parameter: KAX1EQH175T3MAPFULL_bp01Data
   * Referenced by: '<S63>/KAX1-EQH175-T3-MAP-FULL'
   */
  real32_T KAX1EQH175T3MAPFULL_bp01Data[24];

  /* Computed Parameter: KAX1EQH175T3MAP_tableData
   * Referenced by: '<S63>/KAX1-EQH175-T3-MAP'
   */
  real32_T KAX1EQH175T3MAP_tableData[256];

  /* Computed Parameter: KAX1EQH175T3MAP_bp01Data
   * Referenced by: '<S63>/KAX1-EQH175-T3-MAP'
   */
  real32_T KAX1EQH175T3MAP_bp01Data[16];

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S63>/Thr_KAX1'
   *   '<S63>/KAX1-EQH175-T3-MAP'
   */
  real32_T pooled40[16];

  /* Computed Parameter: H19EDci45051MAPAFULL_tableData
   * Referenced by: '<S63>/H19E-Dci450-51-MAPA-FULL'
   */
  real32_T H19EDci45051MAPAFULL_tableData[15];

  /* Computed Parameter: H19EDci45051MAPAFULL_bp01Data
   * Referenced by: '<S63>/H19E-Dci450-51-MAPA-FULL'
   */
  real32_T H19EDci45051MAPAFULL_bp01Data[15];
} ConstP_LgtCtrl_T;

/* Imported (extern) block signals */
extern uint8_T Sys_stADMd_mp;          /* '<Root>/Sys_stADMd_mp ' */
extern real32_T SpdPln_lTrgLngErr_mp;  /* '<Root>/SpdPln_lTrgLngErr_mp' */
extern real32_T SpdPln_vTrgSpd_mp;     /* '<Root>/SpdPln_vTrgSpd_mp ' */
extern real32_T SpdPln_aTrgAcc_mp;     /* '<Root>/SpdPln_aTrgAcc_mp ' */
extern uint8_T BhvCrdn_numBhvID_mp;    /* '<Root>/BhvCrdn_numBhvID_mp' */
extern real32_T VehDa_rSlop_mp;        /* '<Root>/VehDa_rSlop_mp' */
extern real32_T VehDa_rBrkPedl_mp;     /* '<Root>/VehDa_rBrkPedl_mp' */
extern real32_T VehDa_mWght_mp;        /* '<Root>/VehDa_mWght_mp' */
extern real32_T VehDa_vEgoSpd_mp;      /* '<Root>/VehDa_vEgoSpd_mp' */
extern real32_T VehDa_aEgoAcc_mp;      /* '<Root>/VehDa_aEgoAcc_mp' */
extern int16_T VehDa_stTraCurGear_mp;  /* '<Root>/VehDa_stTraCurGear_mp' */
extern real32_T VehDa_rTraCurGear_mp;  /* '<Root>/VehDa_rTraCurGear_mp' */
extern uint8_T VehDa_stCluSwt_mp;      /* '<Root>/VehDa_stCluSwt_mp' */
extern int16_T VehDa_prcTrqEngNomFric_mp;/* '<Root>/VehDa_prcTrqEngNomFric_mp' */
extern int16_T VehDa_prcTrqEstimdLoss_mp;/* '<Root>/VehDa_prcTrqEstimdLoss_mp' */
extern uint8_T VehDa_stSrcBrk_mp;      /* '<Root>/VehDa_stSrcBrk_mp' */
extern int16_T VehDa_prcActuTrq_mp;    /* '<Root>/VehDa_prcActuTrq_mp' */
extern int16_T VehDa_prcDrvrDmdTrq_mp; /* '<Root>/VehDa_prcDrvrDmdTrq_mp' */
extern uint8_T VehDa_stSrcEngCtrl_mp;  /* '<Root>/VehDa_stSrcEngCtrl_mp' */
extern real32_T VehDa_pFrontLeft_mp;   /* '<Root>/VehDa_pFrontLeft_mp' */
extern real32_T VehDa_nEngSpd_mp;      /* '<Root>/VehDa_nEngSpd_mp' */
extern real32_T DRMC_lCurr;            /* '<Root>/DRMC_lCurr' */
extern real32_T EnvDetn_lCllsnTarLgt;  /* '<Root>/EnvDetn_lCllsnTarLgt' */
extern real32_T EnvDetn_vCllsnTarRel;  /* '<Root>/EnvDetn_vCllsnTarRel' */
extern uint8_T EnvDetn_idCllsnTar;     /* '<Root>/EnvDetn_idCllsnTar' */
extern uint8_T VehDa_stTrlrCnctn_mp;   /* '<Root>/VehDa_stTrlrCnctn_mp' */
extern uint8_T VehDa_stTraSht_mp;      /* '<Root>/VehDa_stTraSht_mp' */
extern uint8_T VehDa_stTraEgd_mp;      /* '<Root>/VehDa_stTraEgd_mp' */
extern int16_T VehDa_stTraSelGear_mp;  /* '<Root>/VehDa_stTraSelGear_mp' */
extern uint8_T VehDa_stTraTrqLim_mp;   /* '<Root>/VehDa_stTraTrqLim_mp' */
extern real32_T VehDa_rAccrPedl_mp;    /* '<Root>/VehDa_rAccrPedl_mp' */
extern uint8_T ACCS_vSetFrmSys_mp;     /* '<Root>/ACCS_vSetFrmSys_mp' */
extern real32_T VehDa_nOutpShaft_mp;   /* '<Root>/VehDa_nOutpShaft_mp' */
extern uint8_T VehDa_stBrkReady4Rls_mp;/* '<Root>/VehDa_stBrkReady4Rls_mp' */
extern real32_T VehDa_agPitch_mp;      /* '<Root>/VehDa_agPitch_mp' */
extern real32_T VehDa_varPitch_mp;     /* '<Root>/VehDa_varPitch_mp' */
extern uint8_T VehDa_stMapIsAvail_mp;  /* '<Root>/VehDa_stMapIsAvail_mp' */
extern uint8_T VehDa_stEpb_mp;         /* '<Root>/VehDa_stEpb_mp' */
extern int8_T SpdPln_stTarType_mp;     /* '<Root>/SpdPln_stTarType_mp' */
extern boolean_T SpdPln_stReleaseThrottle_mp;
                                      /* '<Root>/SpdPln_stReleaseThrottle_mp' */
extern real32_T SpdPln_pcc_tar_throttle_mp;/* '<Root>/SpdPln_pcc_tar_throttle_mp' */

/* Imported (extern) block parameters */
extern real32_T CoChs_vIdl_C;          /* Variable: CoChs_vIdl_C
                                        * Referenced by:
                                        *   '<S38>/CoChs_vIdl_C'
                                        *   '<S147>/CoChs_vIdl_C'
                                        *   '<S60>/CoChs_vIdl_C'
                                        */
extern real32_T Eng_nExtCurMin_C;      /* Variable: Eng_nExtCurMin_C
                                        * Referenced by: '<S63>/Eng_nExtCurMin_C'
                                        */
extern real32_T Eng_nIdl_C;            /* Variable: Eng_nIdl_C
                                        * Referenced by:
                                        *   '<S161>/Eng_nIdl_C'
                                        *   '<S63>/Eng_nIdl_C'
                                        */
extern real32_T Eng_nMax_C;            /* Variable: Eng_nMax_C
                                        * Referenced by: '<S63>/Eng_nMax_C'
                                        */
extern real32_T VDC_arFrnt_C;          /* Variable: VDC_arFrnt_C
                                        * Referenced by: '<S101>/VDC_arFrnt_C'
                                        */
extern real32_T VDC_etaDrvAxlEff_C;    /* Variable: VDC_etaDrvAxlEff_C
                                        * Referenced by:
                                        *   '<S8>/VDC_etaDrvAxlEff_C'
                                        *   '<S100>/VDC_etaDrvAxlEff_C'
                                        *   '<S59>/VDC_etaDrvAxlEff_C'
                                        */
extern real32_T VDC_etaTra_C;          /* Variable: VDC_etaTra_C
                                        * Referenced by:
                                        *   '<S8>/VDC_etaTra_C'
                                        *   '<S100>/VDC_etaTra_C'
                                        *   '<S84>/VDC_etaTra_C'
                                        */
extern real32_T VDC_facCD_C;           /* Variable: VDC_facCD_C
                                        * Referenced by: '<S101>/VDC_facCD_C'
                                        */
extern real32_T VDC_facRollngResCalA_C;/* Variable: VDC_facRollngResCalA_C
                                        * Referenced by: '<S102>/VDC_facRollngResCalA_C'
                                        */
extern real32_T VDC_facRollngResCalB_C;/* Variable: VDC_facRollngResCalB_C
                                        * Referenced by: '<S102>/VDC_facRollngResCalB_C'
                                        */
extern real32_T VDC_facRollngResCalCmp_C;/* Variable: VDC_facRollngResCalCmp_C
                                          * Referenced by: '<S102>/VDC_facRollngResCalCmp_C'
                                          */
extern real32_T VDC_moiDrvTrnCmp_C;    /* Variable: VDC_moiDrvTrnCmp_C
                                        * Referenced by: '<S100>/VDC_moiDrvTrnCmp_C'
                                        */
extern real32_T VDC_moiFlyWhl_C;       /* Variable: VDC_moiFlyWhl_C
                                        * Referenced by: '<S100>/VDC_moiFlyWhl_C'
                                        */
extern real32_T VDC_moiWhl_C;          /* Variable: VDC_moiWhl_C
                                        * Referenced by: '<S100>/VDC_moiWhl_C'
                                        */
extern real32_T VDC_numWhlWhtTrlr_C;   /* Variable: VDC_numWhlWhtTrlr_C
                                        * Referenced by: '<S100>/VDC_numWhlWhtTrlr_C'
                                        */
extern real32_T VDC_numWhl_C;          /* Variable: VDC_numWhl_C
                                        * Referenced by: '<S100>/VDC_numWhl_C'
                                        */
extern real32_T VDC_rFinalRatio_C;     /* Variable: VDC_rFinalRatio_C
                                        * Referenced by:
                                        *   '<S8>/VDC_rFinalRatio_C'
                                        *   '<S100>/VDC_rFinalRatio_C'
                                        *   '<S106>/VDC_rFinalRatio_C'
                                        *   '<S59>/VDC_rFinalRatio_C'
                                        */
extern real32_T VDC_rdTire_C;          /* Variable: VDC_rdTire_C
                                        * Referenced by:
                                        *   '<S8>/VDC_rdTire_C'
                                        *   '<S100>/VDC_rdTire_C'
                                        *   '<S106>/VDC_rdTire_C'
                                        *   '<S59>/VDC_rdTire_C'
                                        */
extern real32_T VDC_trqRef_C;          /* Variable: VDC_trqRef_C
                                        * Referenced by:
                                        *   '<S106>/VDC_trqRef_C'
                                        *   '<S60>/VDC_trqRef_C'
                                        *   '<S61>/VDC_trqRef_C'
                                        *   '<S63>/VDC_trqRef_C'
                                        */
extern boolean_T Eng_stEstimdLossEna_C;/* Variable: Eng_stEstimdLossEna_C
                                        * Referenced by: '<S61>/Eng_stEstimdLossEna_C'
                                        */
extern boolean_T Eng_stNomFricEna_C;   /* Variable: Eng_stNomFricEna_C
                                        * Referenced by: '<S61>/Eng_stNomFricEna_C'
                                        */
extern uint8_T Eng_idPltfrm_C;         /* Variable: Eng_idPltfrm_C
                                        * Referenced by: '<S63>/Eng_idPltfrm_C'
                                        */
extern uint8_T MsgTx_stSrcAdr_C;       /* Variable: MsgTx_stSrcAdr_C
                                        * Referenced by: '<S97>/MsgID_LowByte'
                                        */
extern uint8_T Tra_stCfg_C;            /* Variable: Tra_stCfg_C
                                        * Referenced by: '<S85>/Tra_stCfg_C'
                                        */
extern uint8_T VMC_swtAccnGvnrFctEna_C;/* Variable: VMC_swtAccnGvnrFctEna_C
                                        * Referenced by: '<S121>/VMC_swtAccnGvnrFctEna_C'
                                        */
extern uint8_T VMC_swtAccnGvrnInil_C;  /* Variable: VMC_swtAccnGvrnInil_C
                                        * Referenced by: '<S120>/VMC_swtAccnGvrnInil_C'
                                        */

/* Block signals (default storage) */
extern B_LgtCtrl_T LgtCtrl_B;

/* Block states (default storage) */
extern DW_LgtCtrl_T LgtCtrl_DW;
extern const ConstB_LgtCtrl_T LgtCtrl_ConstB;/* constant block i/o */

/* Constant parameters (default storage) */
extern const ConstP_LgtCtrl_T LgtCtrl_ConstP;

/*
 * Exported Global Signals
 *
 * Note: Exported global signals are block signals with an exported global
 * storage class designation.  Code generation will declare the memory for
 * these signals and export their symbols.
 *
 */
extern real32_T VehDa_vEgoSpd;         /* '<S155>/Gain' */
extern real32_T VehDa_rBrkPedl;        /* '<S148>/Gain' */
extern real32_T SpdPln_aTrgAcc;        /* '<S167>/Switch' */
extern real32_T SpdPln_vTrgSpd;        /* '<S168>/Switch' */
extern real32_T SpdPln_lTrgLngErr;     /* '<S163>/Gain' */
extern real32_T VMC_vReq;              /* '<S142>/A1' */
extern real32_T VehDa_mWght;           /* '<S170>/MinMax1' */
extern real32_T VDC_frRollngRes_mp;    /* '<S102>/P4' */
extern real32_T VDC_frAirDrag_mp;      /* '<S101>/P' */
extern real32_T VDC_frSlpRes_mp;       /* '<S103>/P2' */
extern real32_T VehDa_rTraCurGear;     /* '<S151>/Swt2' */
extern real32_T VDC_facDrvTrmJ_mp;     /* '<S100>/A' */
extern real32_T VMC_vDif_mp;           /* '<S128>/Swt' */
extern real32_T VMC_aAccnGvnrP_mp;     /* '<S126>/Swt' */
extern real32_T VMC_aAccnGvnrD_mp;     /* '<S123>/Switch2' */
extern real32_T VMC_aAccnGvnrI_mp;     /* '<S131>/Swt' */
extern real32_T VMC_Accnpid;           /* '<S129>/Swt' */
extern real32_T VMC_aReq;              /* '<S129>/A1' */
extern real32_T VDC_frAccnRes_mp;      /* '<S100>/P4' */
extern real32_T VehDa_aEgoAcc;         /* '<S154>/Gain' */
extern real32_T VMC_frpid;             /* '<S118>/Swt' */
extern real32_T VDC_frCmp_mp;          /* '<S116>/Switch' */
extern real32_T VDC_frDrvForce;        /* '<S104>/A' */
extern real32_T VDC_frEngFric_mp;      /* '<S106>/Divide' */
extern real32_T VDC_aReqNom;           /* '<S104>/Switch' */
extern real32_T DrvAxl_trqRaw_mp;      /* '<S59>/P3' */
extern real32_T DrvAxl_trqReq;         /* '<S58>/Gain' */
extern real32_T Tra_etaGear_mp;        /* '<S84>/VDC_etaTra_C' */
extern real32_T Tra_trqRaw_mp;         /* '<S85>/P1' */
extern real32_T Tra_trqReq;            /* '<S83>/Gain' */
extern real32_T Eng_prcTrqRaw_mp;      /* '<S61>/Gain' */
extern real32_T Eng_prcTrqRaw_mp1;     /* '<S61>/A' */
extern real32_T Eng_prcTrqRaw_mp2;     /* '<S61>/A2' */
extern real32_T Eng_prcTrqLim_mp;      /* '<S62>/Swt' */
extern real32_T Eng_prcTrqReq;         /* '<S60>/MSwt' */
extern real32_T Eng_trqReq;            /* '<S60>/D' */
extern real32_T VehDa_nEngSpdEst;      /* '<S161>/MinMax' */
extern real32_T VehDa_nEngSpd;         /* '<S160>/Gain' */
extern real32_T Eng_rAccrPedlReq_mp;   /* '<S67>/Difference Inputs2' */
extern real32_T Eng_rAccrPedlReqRaw;   /* '<S63>/MSwt' */
extern real32_T Eng_trqCurrMax;        /* '<S63>/Multiport Switch1' */
extern real32_T Eng_rAccrPedlReqRaw1;  /* '<S80>/Difference Inputs2' */
extern real32_T SpdPln_pcc_tar_throttle;/* '<S171>/Gain' */
extern real32_T Eng_rAccrPedlReqRaw2;  /* '<S76>/Difference Inputs2' */
extern real32_T Eng_rAccrPedlReq;      /* '<S73>/Difference Inputs2' */
extern real32_T EBS_aRaw_mp;           /* '<S87>/Switch' */
extern real32_T EBS_aStp_mp;           /* '<S88>/S' */
extern real32_T EBS_aReq;              /* '<S86>/Switch2' */
extern real32_T VehDa_pFrontLeft;      /* '<S162>/Gain' */
extern real32_T VehDa_rSlop;           /* '<S165>/Switch2' */
extern real32_T VMC_ratioVDiff_mp;     /* '<S133>/Swt' */
extern int16_T VehDa_stTraCurGear;     /* '<S14>/Data Type Conversion21' */
extern int16_T VehDa_prcActuTrq;       /* '<S14>/Data Type Conversion14' */
extern int16_T VehDa_prcDrvrDmdTrq;    /* '<S14>/Data Type Conversion16' */
extern int16_T VehDa_prcTrqEngNomFric; /* '<S14>/Data Type Conversion8' */
extern int16_T VehDa_prcTrqEstimdLoss; /* '<S14>/Data Type Conversion10' */
extern uint8_T Tra_stShiftWthSpdUp;    /* '<S10>/Data Type Conversion7' */
extern uint8_T VehDa_stCluSwt;         /* '<S14>/Data Type Conversion6' */
extern uint8_T Sys_stADMd;             /* '<S14>/Data Type Conversion19' */
extern uint8_T VehDa_stTrlrCnctn;      /* '<S14>/Data Type Conversion22' */
extern uint8_T Tra_stTrqLimWthTCU;     /* '<S10>/Data Type Conversion9' */
extern uint8_T VehDa_stSrcEngCtrl;     /* '<S14>/Data Type Conversion18' */
extern uint8_T BhvCrdn_numBhvID;       /* '<S14>/Data Type Conversion20' */
extern uint8_T CoChs_stStarting;       /* '<S1>/Data Type Conversion1' */
extern uint8_T Eng_stVehGo_mp;         /* '<S5>/Data Type Conversion8' */
extern uint8_T VehDa_stSrcBrk;         /* '<S14>/Data Type Conversion12' */
extern uint8_T Eng_StPCC_CC_Change;    /* '<S66>/PCC_CC_ChangeStateCal' */
extern uint8_T CoChs_stCdn;            /* '<S38>/ConCfm' */
extern int8_T SpdPln_stTarType;        /* '<S14>/Data Type Conversion26' */
extern boolean_T SpdPln_stReleaseThrottle;/* '<S14>/Data Type Conversion25' */
extern boolean_T VMC_stAccnGvnrIntglIni_mp;/* '<S120>/L ' */
extern boolean_T VMC_stAccnGvnrIntglFrz_mp;/* '<S120>/L9' */
extern boolean_T CoChs_stDrvReq_mp;    /* '<S39>/R12' */
extern boolean_T CoChs_stBrkReq_mp;    /* '<S39>/R13' */
extern boolean_T CoChs_stSlipReq_mp;   /* '<S39>/R11' */

/*
 * Exported Global Parameters
 *
 * Note: Exported global parameters are tunable parameters with an exported
 * global storage class designation.  Code generation will declare the memory for
 * these parameters and exports their symbols.
 *
 */
extern real32_T ACCS_dtAccrSlpDwn_C;   /* Variable: ACCS_dtAccrSlpDwn_C
                                        * Referenced by: '<S22>/ACCS_dtAccrSlpDwn_C'
                                        */
extern real32_T ACCS_dtAccrSlpUp_C;    /* Variable: ACCS_dtAccrSlpUp_C
                                        * Referenced by: '<S22>/ACCS_dtAccrSlpUp_C'
                                        */
extern real32_T ACCS_dtDecSlpDwn_C;    /* Variable: ACCS_dtDecSlpDwn_C
                                        * Referenced by: '<S22>/ACCS_dtDecSlpDwn_C'
                                        */
extern real32_T ACCS_dtDecSlpUp_C;     /* Variable: ACCS_dtDecSlpUp_C
                                        * Referenced by: '<S22>/ACCS_dtDecSlpUp_C'
                                        */
extern real32_T ACCS_dtFrontTrgAccrSlpDwn_C;/* Variable: ACCS_dtFrontTrgAccrSlpDwn_C
                                             * Referenced by: '<S20>/ACCS_dtFrontTrgAccrSlpDwn_C'
                                             */
extern real32_T ACCS_dtFrontTrgAccrSlpUp_C;/* Variable: ACCS_dtFrontTrgAccrSlpUp_C
                                            * Referenced by: '<S20>/ACCS_dtFrontTrgAccrSlpUp_C'
                                            */
extern real32_T ACCS_dtSpSlpDwnWthTar_C;/* Variable: ACCS_dtSpSlpDwnWthTar_C
                                         * Referenced by: '<S21>/ACCS_dtSpSlpDwnWthTar_C'
                                         */
extern real32_T ACCS_dtSpSlpDwn_C;     /* Variable: ACCS_dtSpSlpDwn_C
                                        * Referenced by: '<S22>/ACCS_dtSpSlpDwn_C'
                                        */
extern real32_T ACCS_dtSpSlpUp_C;      /* Variable: ACCS_dtSpSlpUp_C
                                        * Referenced by: '<S22>/ACCS_dtSpSlpUp_C'
                                        */
extern real32_T ACCS_facDstCtlPnt_C;   /* Variable: ACCS_facDstCtlPnt_C
                                        * Referenced by: '<S21>/Gain1'
                                        */
extern real32_T ACCS_facDstToSpdCurr_C;/* Variable: ACCS_facDstToSpdCurr_C
                                        * Referenced by: '<S21>/ACCS_facDstToSpdCurr_C'
                                        */
extern real32_T ACCS_facDstToSpdNeg_C; /* Variable: ACCS_facDstToSpdNeg_C
                                        * Referenced by: '<S21>/ACCS_facDstToSpdNeg_C'
                                        */
extern real32_T ACCS_facDstToSpdPos_C; /* Variable: ACCS_facDstToSpdPos_C
                                        * Referenced by: '<S21>/ACCS_facDstToSpdPos_C'
                                        */
extern real32_T ACCS_facDstToSpd_C;    /* Variable: ACCS_facDstToSpd_C
                                        * Referenced by: '<S21>/ACCS_facDstToSpd_C'
                                        */
extern real32_T ACCS_facRelSpd2Spd_C;  /* Variable: ACCS_facRelSpd2Spd_C
                                        * Referenced by: '<S21>/ACCS_facRelSpd2Spd_C'
                                        */
extern real32_T ACCS_facSpSlpDwnCtrlPnt_C;/* Variable: ACCS_facSpSlpDwnCtrlPnt_C
                                           * Referenced by: '<S22>/ACCS_facSpSlpDwnCtrlPnt_C'
                                           */
extern real32_T ACCS_facSpSlpUpCtrlPnt_C;/* Variable: ACCS_facSpSlpUpCtrlPnt_C
                                          * Referenced by: '<S22>/ACCS_facSpSlpUpCtrlPnt_C'
                                          */
extern real32_T ACCS_lSetMin_C;        /* Variable: ACCS_lSetMin_C
                                        * Referenced by: '<S21>/ACCS_lSetMin_C'
                                        */
extern real32_T ACCS_rBrkPedlThd_C;    /* Variable: ACCS_rBrkPedlThd_C
                                        * Referenced by: '<S33>/ACCS_rBrkPedlThd_C'
                                        */
extern real32_T ACCS_tiDstSpCalReqdVal_C;/* Variable: ACCS_tiDstSpCalReqdVal_C
                                          * Referenced by: '<S35>/ACCS_tiDstSpCalReqdVal_C'
                                          */
extern real32_T ACCS_tiFollowHysNeg_C; /* Variable: ACCS_tiFollowHysNeg_C
                                        * Referenced by: '<S21>/ACCS_tiFollowHysNeg_C'
                                        */
extern real32_T ACCS_tiFollowHysPos_C; /* Variable: ACCS_tiFollowHysPos_C
                                        * Referenced by: '<S21>/ACCS_tiFollowHysPos_C'
                                        */
extern real32_T ACCS_tiTimeGapModFive_C;/* Variable: ACCS_tiTimeGapModFive_C
                                         * Referenced by: '<S35>/ACCS_tiTimeGapModFive_C'
                                         */
extern real32_T ACCS_tiTimeGapModOne_C;/* Variable: ACCS_tiTimeGapModOne_C
                                        * Referenced by: '<S35>/ACCS_tiTimeGapModOne_C'
                                        */
extern real32_T ACCS_vCruiseToStarting_C;/* Variable: ACCS_vCruiseToStarting_C
                                          * Referenced by: '<S33>/ACCS_vCruiseToStarting_C'
                                          */
extern real32_T ACCS_vDstCtlPnt_C;     /* Variable: ACCS_vDstCtlPnt_C
                                        * Referenced by: '<S21>/ACCS_vDstCtlPnt_C'
                                        */
extern real32_T ACCS_vGoMinWthTar_C;   /* Variable: ACCS_vGoMinWthTar_C
                                        * Referenced by: '<S33>/ACCS_vGoMinWthTar_C'
                                        */
extern real32_T ACCS_vGoMin_C;         /* Variable: ACCS_vGoMin_C
                                        * Referenced by: '<S33>/ACCS_vGoMin_C'
                                        */
extern real32_T ACCS_vMax_C;           /* Variable: ACCS_vMax_C
                                        * Referenced by:
                                        *   '<S33>/ACCS_vMax_C'
                                        *   '<S35>/ACCS_vMax_C'
                                        *   '<S36>/ACCS_vMax_C'
                                        */
extern real32_T ACCS_vMin_C;           /* Variable: ACCS_vMin_C
                                        * Referenced by:
                                        *   '<S33>/ACCS_vMin_C'
                                        *   '<S35>/ACCS_vMin_C'
                                        *   '<S36>/ACCS_vMin_C'
                                        */
extern real32_T ACCS_vStpThd_C;        /* Variable: ACCS_vStpThd_C
                                        * Referenced by: '<S33>/ACCS_vStpThd_C'
                                        */
extern real32_T ACCS_vTipDwn_C;        /* Variable: ACCS_vTipDwn_C
                                        * Referenced by: '<S36>/ACCS_vTipDwn_C'
                                        */
extern real32_T ACCS_vTipUp_C;         /* Variable: ACCS_vTipUp_C
                                        * Referenced by: '<S36>/ACCS_vTipUp_C'
                                        */
extern real32_T ACCS_vVSpCalReqdVal_C; /* Variable: ACCS_vVSpCalReqdVal_C
                                        * Referenced by: '<S35>/ACCS_vVSpCalReqdVal_C'
                                        */
extern real32_T AEBS_aDesFf_C;         /* Variable: AEBS_aDesFf_C
                                        * Referenced by: '<S16>/AEBS_aDesFf_C'
                                        */
extern real32_T AEBS_lSftMin_C;        /* Variable: AEBS_lSftMin_C
                                        * Referenced by: '<S16>/AEBS_lSftMin_C'
                                        */
extern real32_T AEBS_vActvdLmtLo_C;    /* Variable: AEBS_vActvdLmtLo_C
                                        * Referenced by: '<S16>/AEBS_vActvdLmtLo_C'
                                        */
extern real32_T AdpvCC_tiSpdCtrlUpdCyc_C;/* Variable: AdpvCC_tiSpdCtrlUpdCyc_C
                                          * Referenced by: '<S22>/AdpvCC_tiSpdCtrlUpdCyc_C'
                                          */
extern real32_T CC_ThroltRate_lo;      /* Variable: CC_ThroltRate_lo
                                        * Referenced by: '<S69>/CC_ThroltRate_lo'
                                        */
extern real32_T CC_ThroltRate_lo1;     /* Variable: CC_ThroltRate_lo1
                                        * Referenced by: '<S69>/CC_ThroltRate_lo1'
                                        */
extern real32_T CC_ThroltRate_tableX[5];/* Variable: CC_ThroltRate_tableX
                                         * Referenced by:
                                         *   '<S66>/CC_ThroltRateUp'
                                         *   '<S66>/CC_ThroltRateUp1'
                                         *   '<S69>/CC_ThroltRateUp'
                                         */
extern real32_T CC_ThroltRate_tableY[5];/* Variable: CC_ThroltRate_tableY
                                         * Referenced by:
                                         *   '<S66>/CC_ThroltRateUp'
                                         *   '<S66>/CC_ThroltRateUp1'
                                         *   '<S69>/CC_ThroltRateUp'
                                         */
extern real32_T DRMC_aReqFf_C;         /* Variable: DRMC_aReqFf_C
                                        * Referenced by: '<S4>/DRMC_aReqFf_C'
                                        */
extern real32_T DRMC_facSpdCtrlKpCurr_C;/* Variable: DRMC_facSpdCtrlKpCurr_C
                                         * Referenced by: '<S50>/DRMC_facSpdCtrlKpCurr_C'
                                         */
extern real32_T DRMC_facSpdCtrlKpNegCurr_C;/* Variable: DRMC_facSpdCtrlKpNegCurr_C
                                            * Referenced by: '<S50>/DRMC_facSpdCtrlKpNegCurr_C'
                                            */
extern real32_T DRMC_facSpdCtrlKpPosCurr_C;/* Variable: DRMC_facSpdCtrlKpPosCurr_C
                                            * Referenced by: '<S50>/DRMC_facSpdCtrlKpPosCurr_C'
                                            */
extern real32_T DRMC_lReqMin_C;        /* Variable: DRMC_lReqMin_C
                                        * Referenced by: '<S4>/DRMC_lReqMin_C'
                                        */
extern real32_T DRMC_lSpdCtrlPWinNeg_C;/* Variable: DRMC_lSpdCtrlPWinNeg_C
                                        * Referenced by: '<S50>/DRMC_lSpdCtrlPWinNeg_C'
                                        */
extern real32_T DRMC_lSpdCtrlPWinPos_C;/* Variable: DRMC_lSpdCtrlPWinPos_C
                                        * Referenced by: '<S50>/DRMC_lSpdCtrlPWinPos_C'
                                        */
extern real32_T DRMC_lTrg_C;           /* Variable: DRMC_lTrg_C
                                        * Referenced by: '<S4>/DRMC_lTrg_C'
                                        */
extern real32_T DRMC_rAcclPedlPosnReq_C;/* Variable: DRMC_rAcclPedlPosnReq_C
                                         * Referenced by: '<S46>/DRMC_rAcclPedlPosnReq_C'
                                         */
extern real32_T DRMC_tiBrkResp_C;      /* Variable: DRMC_tiBrkResp_C
                                        * Referenced by: '<S4>/DRMC_tiBrkResp_C'
                                        */
extern real32_T DRMC_tiCyc_C;          /* Variable: DRMC_tiCyc_C
                                        * Referenced by: '<S4>/DRMC_tiCyc_C'
                                        */
extern real32_T DRMC_tiPBU_C;          /* Variable: DRMC_tiPBU_C
                                        * Referenced by: '<S4>/DRMC_tiPBU_C'
                                        */
extern real32_T DRMC_tiPrepMax_C;      /* Variable: DRMC_tiPrepMax_C
                                        * Referenced by: '<S4>/DRMC_tiPrepMax_C'
                                        */
extern real32_T EBS_PrkDecnSlpUp_C;    /* Variable: EBS_PrkDecnSlpUp_C
                                        * Referenced by: '<S86>/EBS_PrkDecnSlpUp_C'
                                        */
extern real32_T EBS_aCmpFixd_C;        /* Variable: EBS_aCmpFixd_C
                                        * Referenced by: '<S87>/EBS_aCmpFixd_C'
                                        */
extern real32_T EBS_aDfl_C;            /* Variable: EBS_aDfl_C
                                        * Referenced by: '<S86>/EBS_aDfl_C'
                                        */
extern real32_T EBS_aEmgyBrk_C;        /* Variable: EBS_aEmgyBrk_C
                                        * Referenced by: '<S86>/EBS_aEmgyBrk_C'
                                        */
extern real32_T EBS_aStpPlnBOne_C;     /* Variable: EBS_aStpPlnBOne_C
                                        * Referenced by: '<S88>/EBS_aStpPlnBOne_C'
                                        */
extern real32_T EBS_aStpPlnBTwo_C;     /* Variable: EBS_aStpPlnBTwo_C
                                        * Referenced by: '<S88>/EBS_aStpPlnBTwo_C'
                                        */
extern real32_T EBS_aStpTmp_C;         /* Variable: EBS_aStpTmp_C
                                        * Referenced by: '<S86>/EBS_aStpTmp_C'
                                        */
extern real32_T EBS_aStp_C;            /* Variable: EBS_aStp_C
                                        * Referenced by: '<S88>/EBS_aStp_C'
                                        */
extern real32_T EBS_aTstReq_C;         /* Variable: EBS_aTstReq_C
                                        * Referenced by: '<S86>/EBS_aTstReq_C'
                                        */
extern real32_T EBS_vStpCft_C;         /* Variable: EBS_vStpCft_C
                                        * Referenced by: '<S88>/EBS_vStpCft_C'
                                        */
extern real32_T Eng_dtAccrPedlDwn_C;   /* Variable: Eng_dtAccrPedlDwn_C
                                        * Referenced by: '<S63>/Eng_dtAccrPedlDwn_C'
                                        */
extern real32_T Eng_dtAccrPedlUp_C;    /* Variable: Eng_dtAccrPedlUp_C
                                        * Referenced by: '<S63>/Eng_dtAccrPedlUp_C'
                                        */
extern real32_T Eng_facTrqLimCmpFild_C;/* Variable: Eng_facTrqLimCmpFild_C
                                        * Referenced by: '<S64>/Eng_facTrqLimCmpFild_C'
                                        */
extern real32_T Eng_facTrqLimCmp_C;    /* Variable: Eng_facTrqLimCmp_C
                                        * Referenced by: '<S61>/Eng_facTrqLimCmp_C'
                                        */
extern real32_T Eng_prcGoEnd_C;        /* Variable: Eng_prcGoEnd_C
                                        * Referenced by: '<S60>/Eng_prcGoEnd_C'
                                        */
extern real32_T Eng_prcGo_C;           /* Variable: Eng_prcGo_C
                                        * Referenced by: '<S60>/Eng_prcGo_C'
                                        */
extern real32_T Eng_prcTstReq_C;       /* Variable: Eng_prcTstReq_C
                                        * Referenced by: '<S60>/Eng_prcTstReq_C'
                                        */
extern real32_T Eng_rAccrPedlTstReq_C; /* Variable: Eng_rAccrPedlTstReq_C
                                        * Referenced by: '<S63>/Eng_rAccrPedlTstReq_C'
                                        */
extern real32_T Eng_rGoPedlCmp_C;      /* Variable: Eng_rGoPedlCmp_C
                                        * Referenced by: '<S63>/Eng_rGoPedlCmp_C'
                                        */
extern real32_T Eng_vGoThd_C;          /* Variable: Eng_vGoThd_C
                                        * Referenced by: '<S63>/Eng_vGoThd_C'
                                        */
extern real32_T Eng_vVehStaticThd_C;   /* Variable: Eng_vVehStaticThd_C
                                        * Referenced by: '<S63>/Eng_vVehStaticThd_C'
                                        */
extern real32_T EnvDetn_lCllsnTarLgt_C;/* Variable: EnvDetn_lCllsnTarLgt_C
                                        * Referenced by: '<S20>/EnvDetn_lCllsnTarLgt_C'
                                        */
extern real32_T EnvDetn_tiUpdate_C;    /* Variable: EnvDetn_tiUpdate_C
                                        * Referenced by: '<S20>/EnvDetn_tiUpdate_C'
                                        */
extern real32_T EnvDetn_vCllsnTar_C;   /* Variable: EnvDetn_vCllsnTar_C
                                        * Referenced by: '<S20>/EnvDetn_vCllsnTar_C'
                                        */
extern real32_T I1_C;                  /* Variable: I1_C
                                        * Referenced by: '<S121>/I1_C'
                                        */
extern real32_T I2_C;                  /* Variable: I2_C
                                        * Referenced by: '<S121>/I2_C'
                                        */
extern real32_T I3_C;                  /* Variable: I3_C
                                        * Referenced by: '<S121>/I3_C'
                                        */
extern real32_T I4_C;                  /* Variable: I4_C
                                        * Referenced by: '<S121>/I4_C'
                                        */
extern real32_T I5_C;                  /* Variable: I5_C
                                        * Referenced by: '<S121>/I5_C'
                                        */
extern real32_T P1_C;                  /* Variable: P1_C
                                        * Referenced by: '<S121>/P1_C'
                                        */
extern real32_T P2_C;                  /* Variable: P2_C
                                        * Referenced by: '<S121>/P2_C'
                                        */
extern real32_T P3_C;                  /* Variable: P3_C
                                        * Referenced by: '<S121>/P3_C'
                                        */
extern real32_T P4_C;                  /* Variable: P4_C
                                        * Referenced by: '<S121>/P4_C'
                                        */
extern real32_T P5_C;                  /* Variable: P5_C
                                        * Referenced by: '<S121>/P5_C'
                                        */
extern real32_T PCC2CC_ThroltRate_Default;/* Variable: PCC2CC_ThroltRate_Default
                                           * Referenced by: '<S66>/PCC2CC_ThroltRate_Default'
                                           */
extern real32_T PCC_ThroltRate_lo;     /* Variable: PCC_ThroltRate_lo
                                        * Referenced by: '<S74>/PCC_ThroltRate_lo'
                                        */
extern real32_T PCC_ThroltRate_lo1;    /* Variable: PCC_ThroltRate_lo1
                                        * Referenced by: '<S74>/PCC_ThroltRate_lo1'
                                        */
extern real32_T PCC_ThroltRate_up;     /* Variable: PCC_ThroltRate_up
                                        * Referenced by: '<S74>/PCC_ThroltRate_up'
                                        */
extern real32_T Tra_aShtWthSpdUpThd_C; /* Variable: Tra_aShtWthSpdUpThd_C
                                        * Referenced by: '<S10>/Tra_aShtWthSpdUpThd_C'
                                        */
extern real32_T Tra_facCluTrqEgd_C;    /* Variable: Tra_facCluTrqEgd_C
                                        * Referenced by: '<S10>/Tra_facCluTrqEgd_C'
                                        */
extern real32_T Tra_facCluTrqRls_C;    /* Variable: Tra_facCluTrqRls_C
                                        * Referenced by: '<S10>/Tra_facCluTrqRls_C'
                                        */
extern real32_T VMC_aAccnDCmpLo_C;     /* Variable: VMC_aAccnDCmpLo_C
                                        * Referenced by: '<S119>/VMC_aAccnDCmpLo_C'
                                        */
extern real32_T VMC_aAccnDCmpUp_C;     /* Variable: VMC_aAccnDCmpUp_C
                                        * Referenced by: '<S119>/VMC_aAccnDCmpUp_C'
                                        */
extern real32_T VMC_aAccnGvnrDftlIniVal_C;/* Variable: VMC_aAccnGvnrDftlIniVal_C
                                           * Referenced by: '<S119>/VMC_aAccnGvnrDftlIniVal_C'
                                           */
extern real32_T VMC_aAccnGvnrIntglCtrlValLo_C;
                                      /* Variable: VMC_aAccnGvnrIntglCtrlValLo_C
                                       * Referenced by: '<S121>/VMC_aAccnGvnrIntglCtrlValLo_C'
                                       */
extern real32_T VMC_aAccnGvnrIntglCtrlValUp_C;
                                      /* Variable: VMC_aAccnGvnrIntglCtrlValUp_C
                                       * Referenced by: '<S121>/VMC_aAccnGvnrIntglCtrlValUp_C'
                                       */
extern real32_T VMC_aAccnGvnrIntglIniVal_C;/* Variable: VMC_aAccnGvnrIntglIniVal_C
                                            * Referenced by: '<S119>/VMC_aAccnGvnrIntglIniVal_C'
                                            */
extern real32_T VMC_aAccnGvnrOutpLo_C; /* Variable: VMC_aAccnGvnrOutpLo_C
                                        * Referenced by: '<S129>/VMC_aAccnGvnrOutpLo_C'
                                        */
extern real32_T VMC_aAccnGvnrOutpUp_C; /* Variable: VMC_aAccnGvnrOutpUp_C
                                        * Referenced by: '<S129>/VMC_aAccnGvnrOutpUp_C'
                                        */
extern real32_T VMC_dtEngFricAccr_C;   /* Variable: VMC_dtEngFricAccr_C
                                        * Referenced by: '<S104>/VMC_dtEngFricAccr_C'
                                        */
extern real32_T VMC_facAccnGvnrDftlRatio_C;/* Variable: VMC_facAccnGvnrDftlRatio_C
                                            * Referenced by: '<S119>/VMC_facAccnGvnrDftlRatio_C'
                                            */
extern real32_T VMC_facAccnGvnrKdCurr_C;/* Variable: VMC_facAccnGvnrKdCurr_C
                                         * Referenced by: '<S121>/VMC_facAccnGvnrKdCurr_C'
                                         */
extern real32_T VMC_facAccnGvnrKdNegCurr_C;/* Variable: VMC_facAccnGvnrKdNegCurr_C
                                            * Referenced by: '<S121>/VMC_facAccnGvnrKdNegCurr_C'
                                            */
extern real32_T VMC_facAccnGvnrKdPosCurr_C;/* Variable: VMC_facAccnGvnrKdPosCurr_C
                                            * Referenced by: '<S121>/VMC_facAccnGvnrKdPosCurr_C'
                                            */
extern real32_T VMC_facAccnGvnrKiCurr_C;/* Variable: VMC_facAccnGvnrKiCurr_C
                                         * Referenced by: '<S121>/VMC_facAccnGvnrKiCurr_C'
                                         */
extern real32_T VMC_facAccnGvnrKiNegCurr_C;/* Variable: VMC_facAccnGvnrKiNegCurr_C
                                            * Referenced by: '<S121>/VMC_facAccnGvnrKiNegCurr_C'
                                            */
extern real32_T VMC_facAccnGvnrKiPosCurr_C;/* Variable: VMC_facAccnGvnrKiPosCurr_C
                                            * Referenced by: '<S121>/VMC_facAccnGvnrKiPosCurr_C'
                                            */
extern real32_T VMC_facAccnGvnrKpCurr_C;/* Variable: VMC_facAccnGvnrKpCurr_C
                                         * Referenced by: '<S121>/VMC_facAccnGvnrKpCurr_C'
                                         */
extern real32_T VMC_facAccnGvnrKpNegCurrLoSpd_C;
                                    /* Variable: VMC_facAccnGvnrKpNegCurrLoSpd_C
                                     * Referenced by: '<S121>/VMC_facAccnGvnrKpNegCurrLoSpd_C'
                                     */
extern real32_T VMC_facAccnGvnrKpNegCurr_C;/* Variable: VMC_facAccnGvnrKpNegCurr_C
                                            * Referenced by: '<S121>/VMC_facAccnGvnrKpNegCurr_C'
                                            */
extern real32_T VMC_facAccnGvnrKpPosCurr_C;/* Variable: VMC_facAccnGvnrKpPosCurr_C
                                            * Referenced by: '<S121>/VMC_facAccnGvnrKpPosCurr_C'
                                            */
extern real32_T VMC_facAccnGvrnKpWthGo_C;/* Variable: VMC_facAccnGvrnKpWthGo_C
                                          * Referenced by: '<S121>/VMC_facAccnGvrnKpWthGo_C'
                                          */
extern real32_T VMC_facLocnDifFild_C;  /* Variable: VMC_facLocnDifFild_C
                                        * Referenced by: '<S141>/VMC_facLocnDifFild_C'
                                        */
extern real32_T VMC_facSpdDTfild_C;    /* Variable: VMC_facSpdDTfild_C
                                        * Referenced by: '<S133>/VMC_facSpdDTfild_C'
                                        */
extern real32_T VMC_facSpdDifFild_C;   /* Variable: VMC_facSpdDifFild_C
                                        * Referenced by: '<S128>/VMC_facSpdDifFild_C'
                                        */
extern real32_T VMC_facTrqLim_C;       /* Variable: VMC_facTrqLim_C
                                        * Referenced by: '<S120>/VMC_facTrqLim_C'
                                        */
extern real32_T VMC_facVelGvnrDftlRatio_C;/* Variable: VMC_facVelGvnrDftlRatio_C
                                           * Referenced by: '<S135>/VMC_facVelGvnrDftlRatio_C'
                                           */
extern real32_T VMC_facVelGvnrKdCurr_C;/* Variable: VMC_facVelGvnrKdCurr_C
                                        * Referenced by: '<S137>/VMC_facVelGvnrKdCurr_C'
                                        */
extern real32_T VMC_facVelGvnrKdNegCurr_C;/* Variable: VMC_facVelGvnrKdNegCurr_C
                                           * Referenced by: '<S137>/VMC_facVelGvnrKdNegCurr_C'
                                           */
extern real32_T VMC_facVelGvnrKdPosCurr_C;/* Variable: VMC_facVelGvnrKdPosCurr_C
                                           * Referenced by: '<S137>/VMC_facVelGvnrKdPosCurr_C'
                                           */
extern real32_T VMC_facVelGvnrKiCurr_C;/* Variable: VMC_facVelGvnrKiCurr_C
                                        * Referenced by: '<S137>/VMC_facVelGvnrKiCurr_C'
                                        */
extern real32_T VMC_facVelGvnrKiNegCurr_C;/* Variable: VMC_facVelGvnrKiNegCurr_C
                                           * Referenced by: '<S137>/VMC_facVelGvnrKiNegCurr_C'
                                           */
extern real32_T VMC_facVelGvnrKiPosCurr_C;/* Variable: VMC_facVelGvnrKiPosCurr_C
                                           * Referenced by: '<S137>/VMC_facVelGvnrKiPosCurr_C'
                                           */
extern real32_T VMC_facVelGvnrKpCurr_C;/* Variable: VMC_facVelGvnrKpCurr_C
                                        * Referenced by: '<S137>/VMC_facVelGvnrKpCurr_C'
                                        */
extern real32_T VMC_facVelGvnrKpNegCurr_C;/* Variable: VMC_facVelGvnrKpNegCurr_C
                                           * Referenced by: '<S137>/VMC_facVelGvnrKpNegCurr_C'
                                           */
extern real32_T VMC_facVelGvnrKpPosCurr_C;/* Variable: VMC_facVelGvnrKpPosCurr_C
                                           * Referenced by: '<S137>/VMC_facVelGvnrKpPosCurr_C'
                                           */
extern real32_T VMC_lVelGvnrDWinNeg_C; /* Variable: VMC_lVelGvnrDWinNeg_C
                                        * Referenced by: '<S137>/VMC_lVelGvnrDWinNeg_C'
                                        */
extern real32_T VMC_lVelGvnrDWinPos_C; /* Variable: VMC_lVelGvnrDWinPos_C
                                        * Referenced by: '<S137>/VMC_lVelGvnrDWinPos_C'
                                        */
extern real32_T VMC_lVelGvnrIWinNeg_C; /* Variable: VMC_lVelGvnrIWinNeg_C
                                        * Referenced by: '<S137>/VMC_lVelGvnrIWinNeg_C'
                                        */
extern real32_T VMC_lVelGvnrIWinPos_C; /* Variable: VMC_lVelGvnrIWinPos_C
                                        * Referenced by: '<S137>/VMC_lVelGvnrIWinPos_C'
                                        */
extern real32_T VMC_lVelGvnrPWinNeg_C; /* Variable: VMC_lVelGvnrPWinNeg_C
                                        * Referenced by: '<S137>/VMC_lVelGvnrPWinNeg_C'
                                        */
extern real32_T VMC_lVelGvnrPWinPos_C; /* Variable: VMC_lVelGvnrPWinPos_C
                                        * Referenced by: '<S137>/VMC_lVelGvnrPWinPos_C'
                                        */
extern real32_T VMC_tiAccnGvnrIntglUpdCyc_C;/* Variable: VMC_tiAccnGvnrIntglUpdCyc_C
                                             * Referenced by: '<S119>/VMC_tiAccnGvnrIntglUpdCyc_C'
                                             */
extern real32_T VMC_tiVelGvnrIntglUpdCyc_C;/* Variable: VMC_tiVelGvnrIntglUpdCyc_C
                                            * Referenced by: '<S135>/VMC_tiVelGvnrIntglUpdCyc_C'
                                            */
extern real32_T VMC_vAccnGvnrDWinNeg_C;/* Variable: VMC_vAccnGvnrDWinNeg_C
                                        * Referenced by: '<S121>/VMC_vAccnGvnrDWinNeg_C'
                                        */
extern real32_T VMC_vAccnGvnrDWinPos_C;/* Variable: VMC_vAccnGvnrDWinPos_C
                                        * Referenced by: '<S121>/VMC_vAccnGvnrDWinPos_C'
                                        */
extern real32_T VMC_vAccnGvnrIWinNeg_C;/* Variable: VMC_vAccnGvnrIWinNeg_C
                                        * Referenced by: '<S121>/VMC_vAccnGvnrIWinNeg_C'
                                        */
extern real32_T VMC_vAccnGvnrIWinPos_C;/* Variable: VMC_vAccnGvnrIWinPos_C
                                        * Referenced by: '<S121>/VMC_vAccnGvnrIWinPos_C'
                                        */
extern real32_T VMC_vAccnGvnrPWinNeg_C;/* Variable: VMC_vAccnGvnrPWinNeg_C
                                        * Referenced by: '<S121>/VMC_vAccnGvnrPWinNeg_C'
                                        */
extern real32_T VMC_vAccnGvnrPWinPos_C;/* Variable: VMC_vAccnGvnrPWinPos_C
                                        * Referenced by: '<S121>/VMC_vAccnGvnrPWinPos_C'
                                        */
extern real32_T VMC_vDifThd2I_C;       /* Variable: VMC_vDifThd2I_C
                                        * Referenced by: '<S120>/VMC_vDifThd2I_C'
                                        */
extern real32_T VMC_vKpDecThd_C;       /* Variable: VMC_vKpDecThd_C
                                        * Referenced by: '<S121>/VMC_vKpDecThd_C'
                                        */
extern real32_T VMC_vVelGvnrDftlIniVal_C;/* Variable: VMC_vVelGvnrDftlIniVal_C
                                          * Referenced by: '<S135>/VMC_vVelGvnrDftlIniVal_C'
                                          */
extern real32_T VMC_vVelGvnrIntglCtrlValLo_C;/* Variable: VMC_vVelGvnrIntglCtrlValLo_C
                                              * Referenced by: '<S137>/VMC_vVelGvnrIntglCtrlValLo_C'
                                              */
extern real32_T VMC_vVelGvnrIntglCtrlValUp_C;/* Variable: VMC_vVelGvnrIntglCtrlValUp_C
                                              * Referenced by: '<S137>/VMC_vVelGvnrIntglCtrlValUp_C'
                                              */
extern real32_T VMC_vVelGvnrIntglIniVal_C;/* Variable: VMC_vVelGvnrIntglIniVal_C
                                           * Referenced by: '<S135>/VMC_vVelGvnrIntglIniVal_C'
                                           */
extern real32_T VMC_vVelGvnrOutpLo_C;  /* Variable: VMC_vVelGvnrOutpLo_C
                                        * Referenced by: '<S142>/VMC_vVelGvnrOutpLo_C'
                                        */
extern real32_T VMC_vVelGvnrOutpUp_C;  /* Variable: VMC_vVelGvnrOutpUp_C
                                        * Referenced by: '<S142>/VMC_vVelGvnrOutpUp_C'
                                        */
extern real32_T VehDa_facSlpFilt_C;    /* Variable: VehDa_facSlpFilt_C
                                        * Referenced by: '<S165>/VehDa_facSlpFilt_C'
                                        */
extern real32_T VehDa_mWght_C;         /* Variable: VehDa_mWght_C
                                        * Referenced by: '<S170>/VehDa_mWght_C'
                                        */
extern real32_T VehDa_rSlop_C;         /* Variable: VehDa_rSlop_C
                                        * Referenced by: '<S165>/VehDa_rSlop_C'
                                        */
extern real32_T VehDa_rSlpCmp_C;       /* Variable: VehDa_rSlpCmp_C
                                        * Referenced by: '<S165>/VehDa_rSlpCmp_C'
                                        */
extern boolean_T Debug_swtEngSpdSlect_C;/* Variable: Debug_swtEngSpdSlect_C
                                         * Referenced by: '<S1>/Debug_swtEngSpdSlect_C'
                                         */
extern int8_T Tra_stDflNeutrGear_C;    /* Variable: Tra_stDflNeutrGear_C
                                        * Referenced by: '<S85>/Tra_stDflNeutrGear_C'
                                        */
extern uint8_T ACCS_stACCActvdTest_C;  /* Variable: ACCS_stACCActvdTest_C
                                        * Referenced by: '<S2>/ACCS_stACCActvdTest_C'
                                        */
extern uint8_T ACCS_stACCActvd_C;      /* Variable: ACCS_stACCActvd_C
                                        * Referenced by: '<S21>/ACCS_stACCActvd_C'
                                        */
extern uint8_T ACCS_stDstSpCalReqd_C;  /* Variable: ACCS_stDstSpCalReqd_C
                                        * Referenced by: '<S35>/ACCS_stDstSpCalReqd_C'
                                        */
extern uint8_T ACCS_stFrontTargetSwt_C;/* Variable: ACCS_stFrontTargetSwt_C
                                        * Referenced by: '<S22>/ACCS_stFrontTargetSwt_C'
                                        */
extern uint8_T ACCS_stVSpCalReqd_C;    /* Variable: ACCS_stVSpCalReqd_C
                                        * Referenced by: '<S35>/ACCS_stVSpCalReqd_C'
                                        */
extern uint8_T ACCS_swtDstToSpd_C;     /* Variable: ACCS_swtDstToSpd_C
                                        * Referenced by: '<S21>/ACCS_swtDstToSpd_C'
                                        */
extern uint8_T ACC_RXFlag;             /* Variable: ACC_RXFlag
                                        * Referenced by: '<S14>/ACC_RXFlag'
                                        */
extern uint8_T AEBS_swtFcnActvd_C;     /* Variable: AEBS_swtFcnActvd_C
                                        * Referenced by: '<S16>/AEBS_swtFcnActvd_C'
                                        */
extern uint8_T CoChs_stActvTst_C;      /* Variable: CoChs_stActvTst_C
                                        * Referenced by: '<S38>/CoChs_stActvTst_C'
                                        */
extern uint8_T CoChs_stClbEna_C;       /* Variable: CoChs_stClbEna_C
                                        * Referenced by: '<S38>/CoChs_stClbEna_C'
                                        */
extern uint8_T CoChs_stLgtCtrlEna_C;   /* Variable: CoChs_stLgtCtrlEna_C
                                        * Referenced by: '<S38>/CoChs_stLgtCtrlEna_C'
                                        */
extern uint8_T CoChs_swtStartingEna_C; /* Variable: CoChs_swtStartingEna_C
                                        * Referenced by: '<S1>/CoChs_swtStartingEna_C'
                                        */
extern uint8_T DRMC_stClsLoop_C;       /* Variable: DRMC_stClsLoop_C
                                        * Referenced by: '<S50>/DRMC_stClsLoop_C'
                                        */
extern uint8_T EBS_stStpCalcSwt_C;     /* Variable: EBS_stStpCalcSwt_C
                                        * Referenced by: '<S88>/EBS_stStpCalcSwt_C'
                                        */
extern uint8_T EBS_swtRawAccrCacl_C;   /* Variable: EBS_swtRawAccrCacl_C
                                        * Referenced by: '<S87>/EBS_swtRawAccrCacl_C'
                                        */
extern uint8_T Eng_stPedlFrz_C;        /* Variable: Eng_stPedlFrz_C
                                        * Referenced by: '<S63>/Eng_stPedlFrz_C'
                                        */
extern uint8_T Eng_swtTrqLimCmpFild_C; /* Variable: Eng_swtTrqLimCmpFild_C
                                        * Referenced by: '<S64>/Eng_swtTrqLimCmpFild_C'
                                        */
extern uint8_T EnvDetn_idCllsnTar_C;   /* Variable: EnvDetn_idCllsnTar_C
                                        * Referenced by: '<S20>/EnvDetn_idCllsnTar_C'
                                        */
extern uint8_T LgtCtrl_RXFlag;         /* Variable: LgtCtrl_RXFlag
                                        * Referenced by: '<S14>/LgtCtrl_RXFlag'
                                        */
extern uint8_T MFLv_stDstDecSwt_C;     /* Variable: MFLv_stDstDecSwt_C
                                        * Referenced by: '<S2>/MFLv_stDstDecSwt_C'
                                        */
extern uint8_T MFLv_stDstIncSwt_C;     /* Variable: MFLv_stDstIncSwt_C
                                        * Referenced by: '<S2>/MFLv_stDstIncSwt_C'
                                        */
extern uint8_T MFLv_stOffSwt_C;        /* Variable: MFLv_stOffSwt_C
                                        * Referenced by: '<S2>/MFLv_stOffSwt_C'
                                        */
extern uint8_T MFLv_stResuSwt_C;       /* Variable: MFLv_stResuSwt_C
                                        * Referenced by: '<S2>/MFLv_stResuSwt_C'
                                        */
extern uint8_T MFLv_stSpdDecSwt_C;     /* Variable: MFLv_stSpdDecSwt_C
                                        * Referenced by: '<S2>/MFLv_stSpdDecSwt_C'
                                        */
extern uint8_T MFLv_stSpdIncSwt_C;     /* Variable: MFLv_stSpdIncSwt_C
                                        * Referenced by: '<S2>/MFLv_stSpdIncSwt_C'
                                        */
extern uint8_T Sys_stADMd_C;           /* Variable: Sys_stADMd_C
                                        * Referenced by: '<S146>/Sys_stADMd_C'
                                        */
extern uint8_T Test_stOverflow_C;      /* Variable: Test_stOverflow_C
                                        * Referenced by: '<S165>/Test_stOverflow_C'
                                        */
extern uint8_T VDC_swtNomAccrCalc_C;   /* Variable: VDC_swtNomAccrCalc_C
                                        * Referenced by: '<S104>/VDC_swtNomAccrCalc_C'
                                        */
extern uint8_T VMC_Ienable_C;          /* Variable: VMC_Ienable_C
                                        * Referenced by: '<S121>/VMC_Ienable_C'
                                        */
extern uint8_T VMC_Penable_C;          /* Variable: VMC_Penable_C
                                        * Referenced by: '<S121>/VMC_Penable_C'
                                        */
extern uint8_T VMC_swtLocnDifFild_C;   /* Variable: VMC_swtLocnDifFild_C
                                        * Referenced by: '<S141>/VMC_swtLocnDifFild_C'
                                        */
extern uint8_T VMC_swtSpdDifFild_C;    /* Variable: VMC_swtSpdDifFild_C
                                        * Referenced by: '<S128>/VMC_swtSpdDifFild_C'
                                        */
extern uint8_T VMC_swtVelGvnrFctEna_C; /* Variable: VMC_swtVelGvnrFctEna_C
                                        * Referenced by: '<S137>/VMC_swtVelGvnrFctEna_C'
                                        */
extern uint8_T VehDa_swtSlpFilt_C;     /* Variable: VehDa_swtSlpFilt_C
                                        * Referenced by: '<S165>/VehDa_swtSlpFilt_C'
                                        */
extern uint8_T VehDa_swtSlpSrc_C;      /* Variable: VehDa_swtSlpSrc_C
                                        * Referenced by: '<S165>/VehDa_swtSlpSrc_C'
                                        */

/* Model entry point functions */
extern void LgtCtrl_initialize(void);
extern void LgtCtrl_step(void);
extern void LgtCtrl_terminate(void);

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'LgtCtrl'
 * '<S1>'   : 'LgtCtrl/LgtCtrl'
 * '<S2>'   : 'LgtCtrl/LgtCtrl/ACCS_Decision'
 * '<S3>'   : 'LgtCtrl/LgtCtrl/CoChs'
 * '<S4>'   : 'LgtCtrl/LgtCtrl/DRMC'
 * '<S5>'   : 'LgtCtrl/LgtCtrl/DrvTrn'
 * '<S6>'   : 'LgtCtrl/LgtCtrl/EBS'
 * '<S7>'   : 'LgtCtrl/LgtCtrl/LgtCtrlCANMng_Tx'
 * '<S8>'   : 'LgtCtrl/LgtCtrl/MaxAccrCacln'
 * '<S9>'   : 'LgtCtrl/LgtCtrl/StartingJudge'
 * '<S10>'  : 'LgtCtrl/LgtCtrl/TraMonr'
 * '<S11>'  : 'LgtCtrl/LgtCtrl/VDC_xDyn'
 * '<S12>'  : 'LgtCtrl/LgtCtrl/VMC_AccnGvnr'
 * '<S13>'  : 'LgtCtrl/LgtCtrl/VMC_VelGvnr'
 * '<S14>'  : 'LgtCtrl/LgtCtrl/VehDa'
 * '<S15>'  : 'LgtCtrl/LgtCtrl/ACCS_Decision/100ms_Proc1'
 * '<S16>'  : 'LgtCtrl/LgtCtrl/ACCS_Decision/ACCS_AEB'
 * '<S17>'  : 'LgtCtrl/LgtCtrl/ACCS_Decision/ACCS_DesSpdCalcn'
 * '<S18>'  : 'LgtCtrl/LgtCtrl/ACCS_Decision/ACCS_Mod'
 * '<S19>'  : 'LgtCtrl/LgtCtrl/ACCS_Decision/ACCS_SpTar'
 * '<S20>'  : 'LgtCtrl/LgtCtrl/ACCS_Decision/ACCS_VrtlObj'
 * '<S21>'  : 'LgtCtrl/LgtCtrl/ACCS_Decision/ACCS_DesSpdCalcn/SetPointCaln'
 * '<S22>'  : 'LgtCtrl/LgtCtrl/ACCS_Decision/ACCS_DesSpdCalcn/SetPointRamp'
 * '<S23>'  : 'LgtCtrl/LgtCtrl/ACCS_Decision/ACCS_DesSpdCalcn/SetPointCaln/Dst2Spd'
 * '<S24>'  : 'LgtCtrl/LgtCtrl/ACCS_Decision/ACCS_DesSpdCalcn/SetPointCaln/common_p'
 * '<S25>'  : 'LgtCtrl/LgtCtrl/ACCS_Decision/ACCS_DesSpdCalcn/SetPointRamp/DesAccrCalcn'
 * '<S26>'  : 'LgtCtrl/LgtCtrl/ACCS_Decision/ACCS_DesSpdCalcn/SetPointRamp/FeedForwardAccrCalcn_100ms'
 * '<S27>'  : 'LgtCtrl/LgtCtrl/ACCS_Decision/ACCS_DesSpdCalcn/SetPointRamp/FeedForwardAccrCalcn_100ms1'
 * '<S28>'  : 'LgtCtrl/LgtCtrl/ACCS_Decision/ACCS_DesSpdCalcn/SetPointRamp/Subsystem'
 * '<S29>'  : 'LgtCtrl/LgtCtrl/ACCS_Decision/ACCS_DesSpdCalcn/SetPointRamp/Subsystem1'
 * '<S30>'  : 'LgtCtrl/LgtCtrl/ACCS_Decision/ACCS_DesSpdCalcn/SetPointRamp/Subsystem/FeedForwardAccrCalcn_100ms'
 * '<S31>'  : 'LgtCtrl/LgtCtrl/ACCS_Decision/ACCS_DesSpdCalcn/SetPointRamp/Subsystem1/FeedForwardAccrCalcn_100ms1'
 * '<S32>'  : 'LgtCtrl/LgtCtrl/ACCS_Decision/ACCS_Mod/MFLvCD'
 * '<S33>'  : 'LgtCtrl/LgtCtrl/ACCS_Decision/ACCS_Mod/StMng'
 * '<S34>'  : 'LgtCtrl/LgtCtrl/ACCS_Decision/ACCS_Mod/StMng/StMng'
 * '<S35>'  : 'LgtCtrl/LgtCtrl/ACCS_Decision/ACCS_SpTar/PrioCtrl'
 * '<S36>'  : 'LgtCtrl/LgtCtrl/ACCS_Decision/ACCS_SpTar/SpTar'
 * '<S37>'  : 'LgtCtrl/LgtCtrl/ACCS_Decision/ACCS_SpTar/SpTar/Chart'
 * '<S38>'  : 'LgtCtrl/LgtCtrl/CoChs/CoChs_ConCfm'
 * '<S39>'  : 'LgtCtrl/LgtCtrl/CoChs/CoChs_CtlSysCfm1'
 * '<S40>'  : 'LgtCtrl/LgtCtrl/CoChs/CoChs_ConCfm/ConCfm'
 * '<S41>'  : 'LgtCtrl/LgtCtrl/CoChs/CoChs_CtlSysCfm1/CoChs_stCtlSysCfm'
 * '<S42>'  : 'LgtCtrl/LgtCtrl/CoChs/CoChs_CtlSysCfm1/Delay2'
 * '<S43>'  : 'LgtCtrl/LgtCtrl/CoChs/CoChs_CtlSysCfm1/Delay3'
 * '<S44>'  : 'LgtCtrl/LgtCtrl/CoChs/CoChs_CtlSysCfm1/Delay2/DrvBrkDisenEnCount'
 * '<S45>'  : 'LgtCtrl/LgtCtrl/CoChs/CoChs_CtlSysCfm1/Delay3/DrvBrkDisenEnCount'
 * '<S46>'  : 'LgtCtrl/LgtCtrl/DRMC/AcclPedlPosnCacl'
 * '<S47>'  : 'LgtCtrl/LgtCtrl/DRMC/BrkCacl'
 * '<S48>'  : 'LgtCtrl/LgtCtrl/DRMC/BrkPntCalc'
 * '<S49>'  : 'LgtCtrl/LgtCtrl/DRMC/DRMC_MainState'
 * '<S50>'  : 'LgtCtrl/LgtCtrl/DRMC/FbCalc'
 * '<S51>'  : 'LgtCtrl/LgtCtrl/DRMC/GearGovern'
 * '<S52>'  : 'LgtCtrl/LgtCtrl/DRMC/FbCalc/DRMC_GvnrP'
 * '<S53>'  : 'LgtCtrl/LgtCtrl/DRMC/FbCalc/S_V'
 * '<S54>'  : 'LgtCtrl/LgtCtrl/DRMC/FbCalc/SpdRamp'
 * '<S55>'  : 'LgtCtrl/LgtCtrl/DrvTrn/DrvAxl'
 * '<S56>'  : 'LgtCtrl/LgtCtrl/DrvTrn/Eng'
 * '<S57>'  : 'LgtCtrl/LgtCtrl/DrvTrn/Tra'
 * '<S58>'  : 'LgtCtrl/LgtCtrl/DrvTrn/DrvAxl/CoDA'
 * '<S59>'  : 'LgtCtrl/LgtCtrl/DrvTrn/DrvAxl/DrvAxl_TrqCalc'
 * '<S60>'  : 'LgtCtrl/LgtCtrl/DrvTrn/Eng/CoEng'
 * '<S61>'  : 'LgtCtrl/LgtCtrl/DrvTrn/Eng/Eng_TrqCalc'
 * '<S62>'  : 'LgtCtrl/LgtCtrl/DrvTrn/Eng/Eng_TrqLim'
 * '<S63>'  : 'LgtCtrl/LgtCtrl/DrvTrn/Eng/Eng_TrqToThr'
 * '<S64>'  : 'LgtCtrl/LgtCtrl/DrvTrn/Eng/Eng_TrqCalc/TrqLimCmpFilt'
 * '<S65>'  : 'LgtCtrl/LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/GoJudg'
 * '<S66>'  : 'LgtCtrl/LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/PCC_AccPedlRateSlip2'
 * '<S67>'  : 'LgtCtrl/LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/Rate Limiter Dynamic'
 * '<S68>'  : 'LgtCtrl/LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/Saturation Dynamic'
 * '<S69>'  : 'LgtCtrl/LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/Subsystem'
 * '<S70>'  : 'LgtCtrl/LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/ThrCalc_H19E'
 * '<S71>'  : 'LgtCtrl/LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/ThrCalc_KAX1'
 * '<S72>'  : 'LgtCtrl/LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/PCC_AccPedlRateSlip2/PCC_CC_ChangeStateCal'
 * '<S73>'  : 'LgtCtrl/LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/PCC_AccPedlRateSlip2/Rate Limiter Dynamic'
 * '<S74>'  : 'LgtCtrl/LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/PCC_AccPedlRateSlip2/Subsystem'
 * '<S75>'  : 'LgtCtrl/LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/PCC_AccPedlRateSlip2/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S76>'  : 'LgtCtrl/LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/PCC_AccPedlRateSlip2/Subsystem/Rate Limiter Dynamic1'
 * '<S77>'  : 'LgtCtrl/LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/PCC_AccPedlRateSlip2/Subsystem/filt'
 * '<S78>'  : 'LgtCtrl/LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/PCC_AccPedlRateSlip2/Subsystem/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S79>'  : 'LgtCtrl/LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S80>'  : 'LgtCtrl/LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/Subsystem/Rate Limiter Dynamic1'
 * '<S81>'  : 'LgtCtrl/LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/Subsystem/filt'
 * '<S82>'  : 'LgtCtrl/LgtCtrl/DrvTrn/Eng/Eng_TrqToThr/Subsystem/Rate Limiter Dynamic1/Saturation Dynamic'
 * '<S83>'  : 'LgtCtrl/LgtCtrl/DrvTrn/Tra/CoTra'
 * '<S84>'  : 'LgtCtrl/LgtCtrl/DrvTrn/Tra/Tra_EffCalc'
 * '<S85>'  : 'LgtCtrl/LgtCtrl/DrvTrn/Tra/Tra_TrqCalc'
 * '<S86>'  : 'LgtCtrl/LgtCtrl/EBS/CoEBS'
 * '<S87>'  : 'LgtCtrl/LgtCtrl/EBS/EBS_DecnCalc'
 * '<S88>'  : 'LgtCtrl/LgtCtrl/EBS/EBS_StpDecnCalc'
 * '<S89>'  : 'LgtCtrl/LgtCtrl/EBS/CoEBS/Saturation Dynamic'
 * '<S90>'  : 'LgtCtrl/LgtCtrl/LgtCtrlCANMng_Tx/Command_ADCU'
 * '<S91>'  : 'LgtCtrl/LgtCtrl/LgtCtrlCANMng_Tx/EEC2_ADCU'
 * '<S92>'  : 'LgtCtrl/LgtCtrl/LgtCtrlCANMng_Tx/XBR_ADCU'
 * '<S93>'  : 'LgtCtrl/LgtCtrl/LgtCtrlCANMng_Tx/EEC2_ADCU/Enable'
 * '<S94>'  : 'LgtCtrl/LgtCtrl/LgtCtrlCANMng_Tx/XBR_ADCU/MsgTx'
 * '<S95>'  : 'LgtCtrl/LgtCtrl/LgtCtrlCANMng_Tx/XBR_ADCU/StMng'
 * '<S96>'  : 'LgtCtrl/LgtCtrl/LgtCtrlCANMng_Tx/XBR_ADCU/MsgTx/Enable'
 * '<S97>'  : 'LgtCtrl/LgtCtrl/LgtCtrlCANMng_Tx/XBR_ADCU/MsgTx/MsgTx'
 * '<S98>'  : 'LgtCtrl/LgtCtrl/TraMonr/ShiftJudg'
 * '<S99>'  : 'LgtCtrl/LgtCtrl/TraMonr/TrqLim'
 * '<S100>' : 'LgtCtrl/LgtCtrl/VDC_xDyn/VDC_AccnResCal'
 * '<S101>' : 'LgtCtrl/LgtCtrl/VDC_xDyn/VDC_AirDragCal'
 * '<S102>' : 'LgtCtrl/LgtCtrl/VDC_xDyn/VDC_RollngResCal'
 * '<S103>' : 'LgtCtrl/LgtCtrl/VDC_xDyn/VDC_SlpResCal'
 * '<S104>' : 'LgtCtrl/LgtCtrl/VDC_xDyn/VDC_xDrvForceCal'
 * '<S105>' : 'LgtCtrl/LgtCtrl/VDC_xDyn/VDC_xDynGvnr'
 * '<S106>' : 'LgtCtrl/LgtCtrl/VDC_xDyn/VDC_xEngFricForceCal'
 * '<S107>' : 'LgtCtrl/LgtCtrl/VDC_xDyn/VDC_xDrvForceCal/Rate Limiter Dynamic'
 * '<S108>' : 'LgtCtrl/LgtCtrl/VDC_xDyn/VDC_xDrvForceCal/Rate Limiter Dynamic/Saturation Dynamic'
 * '<S109>' : 'LgtCtrl/LgtCtrl/VDC_xDyn/VDC_xDynGvnr/VDC_FrGvn'
 * '<S110>' : 'LgtCtrl/LgtCtrl/VDC_xDyn/VDC_xDynGvnr/VDC_xDynGvnrIntglStMng'
 * '<S111>' : 'LgtCtrl/LgtCtrl/VDC_xDyn/VDC_xDynGvnr/VDC_xDynGvnrPrmCfm'
 * '<S112>' : 'LgtCtrl/LgtCtrl/VDC_xDyn/VDC_xDynGvnr/VDC_FrGvn/Subsystem'
 * '<S113>' : 'LgtCtrl/LgtCtrl/VDC_xDyn/VDC_xDynGvnr/VDC_FrGvn/VDC_xDynGvnrD'
 * '<S114>' : 'LgtCtrl/LgtCtrl/VDC_xDyn/VDC_xDynGvnr/VDC_FrGvn/VDC_xDynGvnrI'
 * '<S115>' : 'LgtCtrl/LgtCtrl/VDC_xDyn/VDC_xDynGvnr/VDC_FrGvn/VDC_xDynGvnrP'
 * '<S116>' : 'LgtCtrl/LgtCtrl/VDC_xDyn/VDC_xDynGvnr/VDC_FrGvn/VDC_xDynGvnrSum'
 * '<S117>' : 'LgtCtrl/LgtCtrl/VDC_xDyn/VDC_xDynGvnr/VDC_FrGvn/VDC_xDynGvnrI/Dynamic_Saturation'
 * '<S118>' : 'LgtCtrl/LgtCtrl/VDC_xDyn/VDC_xDynGvnr/VDC_FrGvn/VDC_xDynGvnrSum/Dynamic_Saturation'
 * '<S119>' : 'LgtCtrl/LgtCtrl/VMC_AccnGvnr/VMC_AccnGvnr'
 * '<S120>' : 'LgtCtrl/LgtCtrl/VMC_AccnGvnr/VMC_AccnGvnrIntglStMng'
 * '<S121>' : 'LgtCtrl/LgtCtrl/VMC_AccnGvnr/VMC_AccnGvnrPrmCfm'
 * '<S122>' : 'LgtCtrl/LgtCtrl/VMC_AccnGvnr/VMC_AccnGvnr/100ms_Proc1'
 * '<S123>' : 'LgtCtrl/LgtCtrl/VMC_AccnGvnr/VMC_AccnGvnr/Saturation Dynamic'
 * '<S124>' : 'LgtCtrl/LgtCtrl/VMC_AccnGvnr/VMC_AccnGvnr/VMC_AccnGvnrD'
 * '<S125>' : 'LgtCtrl/LgtCtrl/VMC_AccnGvnr/VMC_AccnGvnr/VMC_AccnGvnrI'
 * '<S126>' : 'LgtCtrl/LgtCtrl/VMC_AccnGvnr/VMC_AccnGvnr/VMC_AccnGvnrP'
 * '<S127>' : 'LgtCtrl/LgtCtrl/VMC_AccnGvnr/VMC_AccnGvnr/VMC_AccnGvnrP1'
 * '<S128>' : 'LgtCtrl/LgtCtrl/VMC_AccnGvnr/VMC_AccnGvnr/VMC_AccnGvnrPT1'
 * '<S129>' : 'LgtCtrl/LgtCtrl/VMC_AccnGvnr/VMC_AccnGvnr/VMC_AccnGvnrSum'
 * '<S130>' : 'LgtCtrl/LgtCtrl/VMC_AccnGvnr/VMC_AccnGvnr/difference'
 * '<S131>' : 'LgtCtrl/LgtCtrl/VMC_AccnGvnr/VMC_AccnGvnr/VMC_AccnGvnrI/Dynamic_Saturation'
 * '<S132>' : 'LgtCtrl/LgtCtrl/VMC_AccnGvnr/VMC_AccnGvnr/VMC_AccnGvnrSum/Dynamic_Saturation'
 * '<S133>' : 'LgtCtrl/LgtCtrl/VMC_AccnGvnr/VMC_AccnGvnr/difference/Filt'
 * '<S134>' : 'LgtCtrl/LgtCtrl/VMC_AccnGvnr/VMC_AccnGvnrIntglStMng/debounce'
 * '<S135>' : 'LgtCtrl/LgtCtrl/VMC_VelGvnr/VMC_VelGvnr'
 * '<S136>' : 'LgtCtrl/LgtCtrl/VMC_VelGvnr/VMC_VelGvnrIntglMng'
 * '<S137>' : 'LgtCtrl/LgtCtrl/VMC_VelGvnr/VMC_VelGvnrPrmCfm'
 * '<S138>' : 'LgtCtrl/LgtCtrl/VMC_VelGvnr/VMC_VelGvnr/VMC_VelGvnrD'
 * '<S139>' : 'LgtCtrl/LgtCtrl/VMC_VelGvnr/VMC_VelGvnr/VMC_VelGvnrI'
 * '<S140>' : 'LgtCtrl/LgtCtrl/VMC_VelGvnr/VMC_VelGvnr/VMC_VelGvnrP'
 * '<S141>' : 'LgtCtrl/LgtCtrl/VMC_VelGvnr/VMC_VelGvnr/VMC_VelGvnrPT1'
 * '<S142>' : 'LgtCtrl/LgtCtrl/VMC_VelGvnr/VMC_VelGvnr/VMC_VelGvnrSum'
 * '<S143>' : 'LgtCtrl/LgtCtrl/VMC_VelGvnr/VMC_VelGvnr/VMC_VelGvnrI/Dynamic_Saturation'
 * '<S144>' : 'LgtCtrl/LgtCtrl/VMC_VelGvnr/VMC_VelGvnr/VMC_VelGvnrSum/Dynamic_Saturation'
 * '<S145>' : 'LgtCtrl/LgtCtrl/VehDa/50ms_Proc'
 * '<S146>' : 'LgtCtrl/LgtCtrl/VehDa/ADMd'
 * '<S147>' : 'LgtCtrl/LgtCtrl/VehDa/BhvID'
 * '<S148>' : 'LgtCtrl/LgtCtrl/VehDa/BrkPedlPosn'
 * '<S149>' : 'LgtCtrl/LgtCtrl/VehDa/BrkSrcAddr'
 * '<S150>' : 'LgtCtrl/LgtCtrl/VehDa/CluSwt'
 * '<S151>' : 'LgtCtrl/LgtCtrl/VehDa/CurrGearRatio'
 * '<S152>' : 'LgtCtrl/LgtCtrl/VehDa/CurrGearSt'
 * '<S153>' : 'LgtCtrl/LgtCtrl/VehDa/DrvrDmdTrq'
 * '<S154>' : 'LgtCtrl/LgtCtrl/VehDa/EgoAccr'
 * '<S155>' : 'LgtCtrl/LgtCtrl/VehDa/EgoSpd'
 * '<S156>' : 'LgtCtrl/LgtCtrl/VehDa/EngActuTrq'
 * '<S157>' : 'LgtCtrl/LgtCtrl/VehDa/EngCtrlSrcAddr'
 * '<S158>' : 'LgtCtrl/LgtCtrl/VehDa/EngEstimdLossTrq'
 * '<S159>' : 'LgtCtrl/LgtCtrl/VehDa/EngNomFricTrq'
 * '<S160>' : 'LgtCtrl/LgtCtrl/VehDa/EngSpd'
 * '<S161>' : 'LgtCtrl/LgtCtrl/VehDa/EngSpdEst'
 * '<S162>' : 'LgtCtrl/LgtCtrl/VehDa/FrontLeftBrkPress'
 * '<S163>' : 'LgtCtrl/LgtCtrl/VehDa/LngErr'
 * '<S164>' : 'LgtCtrl/LgtCtrl/VehDa/ReleaseThrottle'
 * '<S165>' : 'LgtCtrl/LgtCtrl/VehDa/Slop'
 * '<S166>' : 'LgtCtrl/LgtCtrl/VehDa/TarType'
 * '<S167>' : 'LgtCtrl/LgtCtrl/VehDa/TrgAcc'
 * '<S168>' : 'LgtCtrl/LgtCtrl/VehDa/TrgSpd'
 * '<S169>' : 'LgtCtrl/LgtCtrl/VehDa/TrlrCnctn'
 * '<S170>' : 'LgtCtrl/LgtCtrl/VehDa/VehWght'
 * '<S171>' : 'LgtCtrl/LgtCtrl/VehDa/tar_throttle'
 * '<S172>' : 'LgtCtrl/LgtCtrl/VehDa/Slop/Saturation Dynamic'
 * '<S173>' : 'LgtCtrl/LgtCtrl/VehDa/TrlrCnctn/Compare To Constant'
 * '<S174>' : 'LgtCtrl/LgtCtrl/VehDa/TrlrCnctn/Compare To Constant1'
 */
#endif                                 /* RTW_HEADER_LgtCtrl_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
