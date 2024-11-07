/*
 * File: LATCtrl.c
 *
 * Code generated for Simulink model 'LATCtrl'.
 *
 * Model version                  : 1.268
 * Simulink Coder version         : 8.5 (R2013b) 08-Aug-2013
 * C/C++ source code generated on : Wed Oct 26 14:06:36 2022
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Infineon->TriCore
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "LATCtrl.h"

/* Named constants for Chart: '<S24>/Chart' */
#define LATCtrl_IN_Avrg                ((uint8_T)1U)
#define LATCtrl_IN_Frozen              ((uint8_T)2U)
#ifndef UCHAR_MAX
#include <limits.h>
#endif

#if 0

/* Skip this size verification because of preprocessor limitation */
#if ( UCHAR_MAX != (0xFFU) ) || ( SCHAR_MAX != (0x7F) )
#error "Code was generated for compiler with different sized uchar/char. Consider adjusting Emulation Hardware word size settings on the Hardware Implementation pane to match your compiler word sizes as defined in the compiler's limits.h header file. Alternatively, you can select 'None' for Emulation Hardware and select the 'Enable portable word sizes' option for ERT based targets, which will disable the preprocessor word size checks."
#endif
#endif

#if 0

/* Skip this size verification because of preprocessor limitation */
#if ( USHRT_MAX != (0xFFFFU) ) || ( SHRT_MAX != (0x7FFF) )
#error "Code was generated for compiler with different sized ushort/short. Consider adjusting Emulation Hardware word size settings on the Hardware Implementation pane to match your compiler word sizes as defined in the compilers limits.h header file. Alternatively, you can select 'None' for Emulation Hardware and select the 'Enable portable word sizes' option for ERT based targets, this will disable the preprocessor word size checks."
#endif
#endif

#if 0

/* Skip this size verification because of preprocessor limitation */
#if ( UINT_MAX != (0xFFFFFFFFU) ) || ( INT_MAX != (0x7FFFFFFF) )
#error "Code was generated for compiler with different sized uint/int. Consider adjusting Emulation Hardware word size settings on the Hardware Implementation pane to match your compiler word sizes as defined in the compilers limits.h header file. Alternatively, you can select 'None' for Emulation Hardware and select the 'Enable portable word sizes' option for ERT based targets, this will disable the preprocessor word size checks."
#endif
#endif

#if 0

/* Skip this size verification because of preprocessor limitation */
#if ( ULONG_MAX != (0xFFFFFFFFU) ) || ( LONG_MAX != (0x7FFFFFFF) )
#error "Code was generated for compiler with different sized ulong/long. Consider adjusting Emulation Hardware word size settings on the Hardware Implementation pane to match your compiler word sizes as defined in the compilers limits.h header file. Alternatively, you can select 'None' for Emulation Hardware and select the 'Enable portable word sizes' option for ERT based targets, this will disable the preprocessor word size checks."
#endif
#endif

#if 0

/* Skip this size verification because of preprocessor limitation */
#if ( ULLONG_MAX != (0xFFFFFFFFFFFFFFFFULL) ) || ( LLONG_MAX != (0x7FFFFFFFFFFFFFFFLL) )
#error "Code was generated for compiler with different sized ulong_long/long_long. Consider adjusting Emulation Hardware word size settings on the Hardware Implementation pane to match your compiler word sizes as defined in the compilers limits.h header file. Alternatively, you can select 'None' for Emulation Hardware and select the 'Enable portable word sizes' option for ERT based targets, this will disable the preprocessor word size checks."
#endif
#endif

/* Invariant block signals (auto storage) */
/* Imported (extern) block signals */
 real_T DE_aLatAcc;              /* '<Root>/DE_aLatAcc' */
 real_T VMP_lLatPosn4TgtPath;    /* '<Root>/VMP_lLatPosn4TgtPath ' */
 real_T VMP_agHeadAngle4TgtPath; /* '<Root>/VMP_agHeadAngle4TgtPath' */
 real_T VMP_cCv4TgtPath;         /* '<Root>/VMP_cCv4TgtPath' */
 real_T DE_wVehYawRate;          /* '<Root>/DE_wVehYawRate' */
 real_T DE_vVehSpdKph;           /* '<Root>/DE_vVehSpdKph' */
 uint8_T CoST_stADMoEna;         /* '<Root>/CoST_stADMoEna' */
 real_T DE_phiSteerAngle;        /* '<Root>/DE_phiSteerAngle' */
 uint8_T COEHPS_LKADemdRsp;      /* '<Root>/COEHPS_LKADemdRsp' */
 real_T VMP_lLaneDist0;          /* '<Root>/VMP_lLaneDist0' */
 real_T VMP_Lane0Cv1;            /* '<Root>/VMP_Lane0Cv1' */
 real_T VMP_Lane0Cv2;            /* '<Root>/VMP_Lane0Cv2' */
 real_T VMP_Lane0Cv3;            /* '<Root>/VMP_Lane0Cv3' */
 real_T VMP_lLaneDist1;          /* '<Root>/VMP_lLaneDist1' */
 real_T VMP_Lane1Cv1;            /* '<Root>/VMP_Lane1Cv1' */
 real_T VMP_Lane1Cv2;            /* '<Root>/VMP_Lane1Cv2' */
 real_T VMP_Lane1Cv3;            /* '<Root>/VMP_Lane1Cv3' */
 real_T VMP_cTgtPathCv0;         /* '<Root>/VMP_cTgtPathCv0' */
 real_T VMP_cTgtPathCv1;         /* '<Root>/VMP_cTgtPathCv1' */
 real_T VMP_cTgtPathCv2;         /* '<Root>/VMP_cTgtPathCv2' */
 real_T VMP_cTgtPathCv3;         /* '<Root>/VMP_cTgtPathCv3' */
 real_T VMP_cTgtPathCv4;         /* '<Root>/VMP_cTgtPathCv4' */
 real_T VMP_cTgtPathCv5;         /* '<Root>/VMP_cTgtPathCv5' */
 real_T VMP_lLgtDist2PathZero;   /* '<Root>/VMP_lLgtDist2PathZero' */
 uint8_T VMP_stLane0Prob;        /* '<Root>/VMP_stLane0Prob' */
 uint8_T VMP_stLane1Prob;        /* '<Root>/VMP_stLane1Prob' */
 real_T DE_mVehMass;             /* '<Root>/DE_mVehMass' */
 uint8_T DE_stTrlr;              /* '<Root>/DE_stTrlr' */
 int32_T ADCU_tTiDely4MCU2RCAR;  /* '<Root>/ADCU_tTiDely4MCU2RCAR' */
 real_T DE_aLonAcc;              /* '<Root>/DE_aLonAcc' */
 real_T DE_aVertAcc;             /* '<Root>/DE_aVertAcc' */
 int32_T VMP_LC_Flag;            /* '<Root>/VMP_LC_Flag' */

/* ConstVolatile memory section */
const volatile ConstB_LATCtrl_T LATCtrl_ConstB = {
  { 0.0, 0.0 }
  ,                                    /* '<S39>/Matrix Concatenate3' */

  { 0.0, 1.0 }
  ,                                    /* '<S38>/Matrix Concatenate4' */

  { 0.0, 0.01 }
  ,                                    /* '<S38>/Matrix Concatenate5' */
  10000.0
  ,                                    /* '<S3>/Product3' */

  { 1.0, 0.0, 0.0 }
  ,                                    /* 'synthesized block' */

  { 1.0, 0.0, 0.0 }
  ,                                    /* '<S36>/Math Function' */

  { 0.0, 0.01, 1.0 }
  ,                                    /* 'synthesized block' */

  { 0.0, 0.01, 1.0 }
  ,                                    /* '<S36>/Math Function2' */
  0.0
  ,                                    /* '<S19>/Constant2' */
  1.0999999999999999
  ,                                    /* '<S56>/Add' */
  9600.0
  ,                                    /* '<Root>/Data Type Conversion5' */
  0.0
  ,                                    /* '<S56>/Add1' */
  15400.0
  ,                                    /* '<S56>/Add2' */
  7.142857142857142E-5
  ,                                    /* '<S56>/Divide2' */
  0.0
  ,                                    /* '<S56>/Product' */
  1.7
  ,                                    /* '<S56>/Add3' */
  1.7
  ,                                    /* '<S56>/MinMax' */
  1.7
  ,                                    /* '<S56>/MinMax1' */
  3.1799999999999997
  ,                                    /* '<S56>/Add4' */
  0.0
  ,                                    /* '<Root>/Data Type Conversion4' */
  0.0
  ,                                    /* '<S55>/Switch3' */
  0.0
  ,                                    /* '<S55>/Divide2' */
  1114825U
  ,                                    /* '<S62>/Gain' */
  1
  ,                                    /* '<S87>/Relational Operator1' */
  1
  ,                                    /* '<S87>/Relational Operator2' */
  1
  /* '<S87>/Logical Operator3' */
};

/* Constant parameters (auto storage) */
/* ConstVolatile memory section */
const volatile ConstP_LATCtrl_T LATCtrl_ConstP = {
  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S32>/Constant'
   *   '<S32>/Constant1'
   *   '<S48>/delay1'
   */
  { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 },

  /* Expression: [0.082196282, 0.082196282, 0.082196282, 0.082196282, 0.082196282, 0.082196282, 0.082196282, 0.082196282, 0.082196282, 0.082196282, 0.082196282, 0.082196282, 0.082196282, 0.082196282, 0.082196282, 0.082196282, 0.087889661, 0.093530592, 0.099120413, 0.104653454, 0.110121819, 0.115517882, 0.120835327, 0.126069351, 0.131216511, 0.136274443, 0.14124159, 0.146116992, 0.150900137, 0.155590861, 0.160189294, 0.164695814, 0.16911103, 0.173435754, 0.177670989, 0.181816416, 0.18587400, 0.189846967, 0.193735991, 0.197542979, 0.201269663, 0.204917848, 0.20848907, 0.211985205, 0.215408833, 0.218760866, 0.222043894, 0.225259477, 0.228409496, 0.231496164, 0.234520591, 0.237485289, 0.240391651, 0.243241386, 0.246036165, 0.248777611, 0.251467304, 0.254107162, 0.256697909, 0.259241755, 0.261739326, 0.264192748, 0.266602553, 0.268970784, 0.271297881, 0.27358581, 0.275835318, 0.278047108, 0.280223036, 0.282363729, 0.284469775, 0.286542931, 0.288583715, 0.290593015, 0.292571284, 0.294520155, 0.296440021, 0.29833165, 0.300195785, 0.30203314, 0.303844407, 0.305629845, 0.30739091, 0.309127817, 0.310841163, 0.312531529, 0.314199474, 0.315845536, 0.317470237, 0.319074082, 0.320657557, 0.322221133, 0.323765265, 0.325290393, 0.326796944, 0.328285328, 0.329755945, 0.33120918, 0.332645407, 0.334064986, 0.33546827, 0.336855596, 0.338227293, 0.33958368, 0.340925066, 0.342251751, 0.343564024, 0.344862168, 0.346146457, 0.347417155, 0.348674520]
   * Referenced by: '<S23>/K4_Full'
   */
  { 0.082196282, 0.082196282, 0.082196282, 0.082196282, 0.082196282, 0.082196282,
    0.082196282, 0.082196282, 0.082196282, 0.082196282, 0.082196282, 0.082196282,
    0.082196282, 0.082196282, 0.082196282, 0.082196282, 0.087889661, 0.093530592,
    0.099120413, 0.104653454, 0.110121819, 0.115517882, 0.120835327, 0.126069351,
    0.131216511, 0.136274443, 0.14124159, 0.146116992, 0.150900137, 0.155590861,
    0.160189294, 0.164695814, 0.16911103, 0.173435754, 0.177670989, 0.181816416,
    0.185874, 0.189846967, 0.193735991, 0.197542979, 0.201269663, 0.204917848,
    0.20848907, 0.211985205, 0.215408833, 0.218760866, 0.222043894, 0.225259477,
    0.228409496, 0.231496164, 0.234520591, 0.237485289, 0.240391651, 0.243241386,
    0.246036165, 0.248777611, 0.251467304, 0.254107162, 0.256697909, 0.259241755,
    0.261739326, 0.264192748, 0.266602553, 0.268970784, 0.271297881, 0.27358581,
    0.275835318, 0.278047108, 0.280223036, 0.282363729, 0.284469775, 0.286542931,
    0.288583715, 0.290593015, 0.292571284, 0.294520155, 0.296440021, 0.29833165,
    0.300195785, 0.30203314, 0.303844407, 0.305629845, 0.30739091, 0.309127817,
    0.310841163, 0.312531529, 0.314199474, 0.315845536, 0.317470237, 0.319074082,
    0.320657557, 0.322221133, 0.323765265, 0.325290393, 0.326796944, 0.328285328,
    0.329755945, 0.33120918, 0.332645407, 0.334064986, 0.33546827, 0.336855596,
    0.338227293, 0.33958368, 0.340925066, 0.342251751, 0.343564024, 0.344862168,
    0.346146457, 0.347417155, 0.34867452 },

  /* Pooled Parameter (Expression: [0.0 1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0 10.0 11.0 12.0 13.0 14.0 15.0 16.0 17.0 18.0 19.0 20.0 21.0 22.0 23.0 24.0 25.0 26.0 27.0 28.0 29.0 30.0 31.0 32.0 33.0 34.0 35.0 36.0 37.0 38.0 39.0 40.0 41.0 42.0 43.0 44.0 45.0 46.0 47.0 48.0 49.0 50.0 51.0 52.0 53.0 54.0 55.0 56.0 57.0 58.0 59.0 60.0 61.0 62.0 63.0 64.0 65.0 66.0 67.0 68.0 69.0 70.0 71.0 72.0 73.0 74.0 75.0 76.0 77.0 78.0 79.0 80.0 81.0 82.0 83.0 84.0 85.0 86.0 87.0 88.0 89.0 90.0 91.0 92.0 93.0 94.0 95.0 96.0 97.0 98.0 99.0 100.0 101.0 102.0 103.0 104.0 105.0 106.0 107.0 108.0 109.0 110.0 ])
   * Referenced by:
   *   '<S23>/K4_Full'
   *   '<S23>/K4_empty'
   */
  { 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0,
    14.0, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0,
    27.0, 28.0, 29.0, 30.0, 31.0, 32.0, 33.0, 34.0, 35.0, 36.0, 37.0, 38.0, 39.0,
    40.0, 41.0, 42.0, 43.0, 44.0, 45.0, 46.0, 47.0, 48.0, 49.0, 50.0, 51.0, 52.0,
    53.0, 54.0, 55.0, 56.0, 57.0, 58.0, 59.0, 60.0, 61.0, 62.0, 63.0, 64.0, 65.0,
    66.0, 67.0, 68.0, 69.0, 70.0, 71.0, 72.0, 73.0, 74.0, 75.0, 76.0, 77.0, 78.0,
    79.0, 80.0, 81.0, 82.0, 83.0, 84.0, 85.0, 86.0, 87.0, 88.0, 89.0, 90.0, 91.0,
    92.0, 93.0, 94.0, 95.0, 96.0, 97.0, 98.0, 99.0, 100.0, 101.0, 102.0, 103.0,
    104.0, 105.0, 106.0, 107.0, 108.0, 109.0, 110.0 },

  /* Expression:  [0.650920442, 0.650920442, 0.650920442, 0.650920442, 0.650920442, 0.650920442, 0.669450899, 0.686237303, 0.701379243, 0.71504542, 0.727487675, 0.738899785, 0.749410198, 0.759152679, 0.768238252, 0.776757585, 0.784794648, 0.79242282, 0.799704856, 0.806694927, 0.813437233, 0.819970915, 0.826326117, 0.832528772, 0.838600216, 0.844557947, 0.850417575, 0.856188161, 0.861881769, 0.867504746, 0.873063247, 0.87856219, 0.884005482, 0.889396198, 0.894736732, 0.900028921, 0.905274153, 0.908745406]
   * Referenced by: '<S23>/K3_Full'
   */
  { 0.650920442, 0.650920442, 0.650920442, 0.650920442, 0.650920442, 0.650920442,
    0.669450899, 0.686237303, 0.701379243, 0.71504542, 0.727487675, 0.738899785,
    0.749410198, 0.759152679, 0.768238252, 0.776757585, 0.784794648, 0.79242282,
    0.799704856, 0.806694927, 0.813437233, 0.819970915, 0.826326117, 0.832528772,
    0.838600216, 0.844557947, 0.850417575, 0.856188161, 0.861881769, 0.867504746,
    0.873063247, 0.87856219, 0.884005482, 0.889396198, 0.894736732, 0.900028921,
    0.905274153, 0.908745406 },

  /* Pooled Parameter (Expression: [ 0.0 3.0 6.0 9.0 12.0 15.0 18.0 21.0 24.0 27.0 30.0 33.0 36.0 39.0 42.0 45.0 48.0 51.0 54.0 57.0 60.0 63.0 66.0 69.0 72.0 75.0 78.0 81.0 84.0 87.0 90.0 93.0 96.0 99.0 102.0 105.0 108.0 110.0 ])
   * Referenced by:
   *   '<S23>/K3_Full'
   *   '<S23>/K3_empty'
   */
  { 0.0, 3.0, 6.0, 9.0, 12.0, 15.0, 18.0, 21.0, 24.0, 27.0, 30.0, 33.0, 36.0,
    39.0, 42.0, 45.0, 48.0, 51.0, 54.0, 57.0, 60.0, 63.0, 66.0, 69.0, 72.0, 75.0,
    78.0, 81.0, 84.0, 87.0, 90.0, 93.0, 96.0, 99.0, 102.0, 105.0, 108.0, 110.0 },

  /* Expression: [0.08 0.08]
   * Referenced by: '<S23>/K2_Full'
   */
  { 0.08, 0.08 },

  /* Pooled Parameter (Expression: [0.0 110.0])
   * Referenced by:
   *   '<S23>/K2_Full'
   *   '<S23>/K2_empty'
   */
  { 0.0, 110.0 },

  /* Expression: [0.053548497, 0.053548497, 0.053548497, 0.053548497, 0.054150466, 0.054371952, 0.054382439, 0.054345531, 0.054300855, 0.054260114, 0.054223619, 0.054191084, 0.054161948, 0.054135659, 0.054111646, 0.054089816, 0.054069912, 0.054051254, 0.054034114, 0.054018107, 0.054003103, 0.053988992, 0.053975681]
   * Referenced by: '<S23>/K1_Full'
   */
  { 0.053548497, 0.053548497, 0.053548497, 0.053548497, 0.054150466, 0.054371952,
    0.054382439, 0.054345531, 0.054300855, 0.054260114, 0.054223619, 0.054191084,
    0.054161948, 0.054135659, 0.054111646, 0.054089816, 0.054069912, 0.054051254,
    0.054034114, 0.054018107, 0.054003103, 0.053988992, 0.053975681 },

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S23>/K1_Full'
   *   '<S23>/K1_empty'
   */
  { 0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 60.0,
    65.0, 70.0, 75.0, 80.0, 85.0, 90.0, 95.0, 100.0, 105.0, 110.0 },

  /* Expression: [0.144605051, 0.144605051, 0.144605051, 0.144605051, 0.144605051, 0.144605051, 0.144605051, 0.144605051, 0.144605051, 0.144605051, 0.144605051, 0.144605051, 0.144605051, 0.144605051, 0.144605051, 0.144605051, 0.151739085, 0.158450373, 0.164776967, 0.170747921, 0.176386951, 0.181714645, 0.186749713, 0.191509656, 0.196011083, 0.200269834, 0.204301026, 0.20811904, 0.211737511, 0.215169315, 0.218426567, 0.221520633, 0.224462142, 0.227261017, 0.229926504, 0.232467204, 0.234891117, 0.23720567, 0.239417765, 0.241533805, 0.243559739, 0.245501087, 0.247362979, 0.249150179, 0.250867115, 0.252517906, 0.254106383, 0.255636115, 0.257110423, 0.258532407, 0.259904956, 0.261230767, 0.26251236, 0.263752089, 0.264952159, 0.266114631, 0.267241437, 0.268334388, 0.269395182, 0.270425414, 0.271426582, 0.272400094, 0.273347273, 0.274269367, 0.275167549, 0.276042926, 0.276896542, 0.27772938, 0.278542371, 0.279336394, 0.280112279, 0.280870811, 0.281612734, 0.282338752, 0.283049534, 0.28374571, 0.284427882, 0.285096619, 0.285752462, 0.286395925, 0.287027496, 0.28764764, 0.28825680, 0.288855395, 0.289443828, 0.290022479, 0.290591713, 0.291151878, 0.291703304, 0.292246307, 0.292781191, 0.293308244, 0.293827741, 0.294339945, 0.294845111, 0.295343478, 0.295835277, 0.296320731, 0.296800052, 0.297273441, 0.297741095, 0.298203199, 0.298659934, 0.299111471, 0.299557974, 0.299999604, 0.300436512, 0.300868844, 0.301296741, 0.301720339, 0.302139768]
   * Referenced by: '<S23>/K4_empty'
   */
  { 0.144605051, 0.144605051, 0.144605051, 0.144605051, 0.144605051, 0.144605051,
    0.144605051, 0.144605051, 0.144605051, 0.144605051, 0.144605051, 0.144605051,
    0.144605051, 0.144605051, 0.144605051, 0.144605051, 0.151739085, 0.158450373,
    0.164776967, 0.170747921, 0.176386951, 0.181714645, 0.186749713, 0.191509656,
    0.196011083, 0.200269834, 0.204301026, 0.20811904, 0.211737511, 0.215169315,
    0.218426567, 0.221520633, 0.224462142, 0.227261017, 0.229926504, 0.232467204,
    0.234891117, 0.23720567, 0.239417765, 0.241533805, 0.243559739, 0.245501087,
    0.247362979, 0.249150179, 0.250867115, 0.252517906, 0.254106383, 0.255636115,
    0.257110423, 0.258532407, 0.259904956, 0.261230767, 0.26251236, 0.263752089,
    0.264952159, 0.266114631, 0.267241437, 0.268334388, 0.269395182, 0.270425414,
    0.271426582, 0.272400094, 0.273347273, 0.274269367, 0.275167549, 0.276042926,
    0.276896542, 0.27772938, 0.278542371, 0.279336394, 0.280112279, 0.280870811,
    0.281612734, 0.282338752, 0.283049534, 0.28374571, 0.284427882, 0.285096619,
    0.285752462, 0.286395925, 0.287027496, 0.28764764, 0.2882568, 0.288855395,
    0.289443828, 0.290022479, 0.290591713, 0.291151878, 0.291703304, 0.292246307,
    0.292781191, 0.293308244, 0.293827741, 0.294339945, 0.294845111, 0.295343478,
    0.295835277, 0.296320731, 0.296800052, 0.297273441, 0.297741095, 0.298203199,
    0.298659934, 0.299111471, 0.299557974, 0.299999604, 0.300436512, 0.300868844,
    0.301296741, 0.301720339, 0.302139768 },

  /* Expression: [0.541699374, 0.541699374, 0.541699374, 0.541699374, 0.541699374, 0.541699374, 0.535491033, 0.526724323, 0.516884455, 0.506572706, 0.496166533, 0.48592248, 0.476006434, 0.466517173, 0.457506531, 0.448994961, 0.440982696, 0.433457413, 0.426399374, 0.419784806, 0.413588105, 0.407783243, 0.402344639, 0.397247694, 0.39246908, 0.387986898, 0.383780716, 0.379831567, 0.376121893, 0.372635477, 0.36935736, 0.36627375, 0.36337194, 0.360640216, 0.358067781, 0.355644674, 0.353361704, 0.351913358]
   * Referenced by: '<S23>/K3_empty'
   */
  { 0.541699374, 0.541699374, 0.541699374, 0.541699374, 0.541699374, 0.541699374,
    0.535491033, 0.526724323, 0.516884455, 0.506572706, 0.496166533, 0.48592248,
    0.476006434, 0.466517173, 0.457506531, 0.448994961, 0.440982696, 0.433457413,
    0.426399374, 0.419784806, 0.413588105, 0.407783243, 0.402344639, 0.397247694,
    0.39246908, 0.387986898, 0.383780716, 0.379831567, 0.376121893, 0.372635477,
    0.36935736, 0.36627375, 0.36337194, 0.360640216, 0.358067781, 0.355644674,
    0.353361704, 0.351913358 },

  /* Expression: [0.1 0.1]
   * Referenced by: '<S23>/K2_empty'
   */
  { 0.1, 0.1 },

  /* Expression:  [0.057723142, 0.057723142, 0.057723142, 0.057723142, 0.058009028, 0.058321582, 0.058504600, 0.058602221, 0.058654894, 0.058684368, 0.058701492, 0.058711750, 0.058718023, 0.058721887, 0.058724244, 0.058725628, 0.058726367, 0.058726669, 0.058726669, 0.058726457, 0.058726093, 0.058725620, 0.058725067]
   * Referenced by: '<S23>/K1_empty'
   */
  { 0.057723142, 0.057723142, 0.057723142, 0.057723142, 0.058009028, 0.058321582,
    0.0585046, 0.058602221, 0.058654894, 0.058684368, 0.058701492, 0.05871175,
    0.058718023, 0.058721887, 0.058724244, 0.058725628, 0.058726367, 0.058726669,
    0.058726669, 0.058726457, 0.058726093, 0.05872562, 0.058725067 },

  /* Expression: [1 1 2 2.5 2.5]
   * Referenced by: '<S73>/1-D Lookup Table4'
   */
  { 1.0, 1.0, 2.0, 2.5, 2.5 },

  /* Pooled Parameter (Expression: [0 0.004 0.008 0.015 0.02])
   * Referenced by:
   *   '<S73>/1-D Lookup Table1'
   *   '<S73>/1-D Lookup Table3'
   *   '<S73>/1-D Lookup Table4'
   */
  { 0.0, 0.004, 0.008, 0.015, 0.02 },

  /* Expression: [1 1 0.8 0.5 0.5]
   * Referenced by: '<S73>/1-D Lookup Table1'
   */
  { 1.0, 1.0, 0.8, 0.5, 0.5 },

  /* Expression: [1.1 1.1 0.5 0.4 0.4]
   * Referenced by: '<S73>/1-D Lookup Table3'
   */
  { 1.1, 1.1, 0.5, 0.4, 0.4 },

  /* Pooled Parameter (Expression: [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1])
   * Referenced by:
   *   '<S67>/Constant2'
   *   '<S68>/Constant2'
   */
  { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    1.0 },

  /* Pooled Parameter (Expression: [0 1;1 0;0 1;0 1;1 0;1 0;0 0;0 0])
   * Referenced by:
   *   '<S14>/Logic'
   *   '<S15>/Logic'
   *   '<S85>/Logic'
   */
  { 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0 }
};

/* Block signals (auto storage) */
B_LATCtrl_T LATCtrl_B;

/* Block states (auto storage) */
DW_LATCtrl_T LATCtrl_DW;

/* Forward declaration for local functions */
static void LATCtrl_inner_default_Avrg(void);

/* Function for Chart: '<S24>/Chart' */
static void LATCtrl_inner_default_Avrg(void)
{
  /* Constant: '<S24>/Constant3' */
  /* Transition: '<S90>:2' */
  if ((LATC_stSteerAvrgEnb == 1) && (LATC_stSteerAvrgActv == 1) &&
      (LATCtrl_B.I_Cnt < 15.0)) {
    /* Transition: '<S90>:4' */
    LATCtrl_DW.I_avrg++;

    /* Transition: '<S90>:7' */
    LATCtrl_DW.Avrg2Store = ((LATCtrl_DW.I_avrg - 1.0) * LATCtrl_DW.Avrg2Store +
      LATC_phiSteerAngle) / LATCtrl_DW.I_avrg;

    /* Constant: '<S24>/Constant' */
    if (!(LATCtrl_DW.I_avrg < 100.0)) {
      /* Transition: '<S90>:56' */
      LATCtrl_B.Avrg = (LATCtrl_B.I_Cnt * LATCtrl_B.Avrg + LATCtrl_DW.Avrg2Store)
        / (LATCtrl_B.I_Cnt + 1.0);

      /* Transition: '<S90>:60' */
      LATCtrl_B.I_Cnt++;
      LATCtrl_DW.I_avrg = 0.0;
      LATCtrl_DW.Avrg2Store = 0.0;
    } else {
      /* Transition: '<S90>:36' */
      /* Transition: '<S90>:63' */
    }

    /* End of Constant: '<S24>/Constant' */
    /* Transition: '<S90>:61' */
  } else {
    if ((LATCtrl_B.I_Cnt != 15.0) && (LATC_stSteerAvrgEnb != 1)) {
      /* Transition: '<S90>:6' */
      LATCtrl_DW.I_avrg = 0.0;
      LATCtrl_DW.Avrg2Store = 0.0;
      LATCtrl_B.I_Cnt = 0.0;
      LATCtrl_B.Avrg = 0.0;
    }
  }

  /* End of Constant: '<S24>/Constant3' */
}

/* Model step function */
void LATCtrl_step(void)
{
  uint16_T i;
  real_T Pk[9];
  real_T Y[9];
  real_T A[9];
  real_T B[9];
  int32_T r;
  int32_T r_0;
  real_T maxval;
  real_T a;
  int32_T rtemp;
  uint16_T n;
  real_T P_lt[16];
  real_T y[4];
  real_T e_y[16];
  real_T u[3];
  real_T v[3];
  real_T v_0[3];
  int32_T i_0;
  real_T tmp[9];
  real_T tmp_0[3];
  real_T tmp_1[4];
  real_T tmp_2[4];
  real_T tmp_3[16];
  real_T tmp_4[4];
  real_T b_idx;
  boolean_T unnamed_idx;
  real_T tmp_5;
  uint32_T qY;
  uint32_T tmp_6;
  real_T u_0;
  real_T u_1;
  real_T v_1;

  /* UnitDelay: '<S39>/Unit Delay1' */
  LATCtrl_B.UnitDelay1 = LATCtrl_DW.UnitDelay1_DSTATE;

  /* UnitDelay: '<S39>/Unit Delay' */
  LATCtrl_B.UnitDelay_a = LATCtrl_DW.UnitDelay_DSTATE_i;

  /* Sum: '<S39>/Sum6' incorporates:
   *  Constant: '<S39>/C2'
   */
  LATCtrl_B.Sum6_g = (uint16_T)(LATCtrl_B.UnitDelay_a + (uint32_T)((uint8_T)1U));

  /* Math: '<S39>/Math Function' incorporates:
   *  Constant: '<S39>/C3'
   */
  i = LATCtrl_B.Sum6_g;
  n = LATC_iYawRateMaxNum;
  if (n == 0) {
    LATCtrl_B.MathFunction_g = i;
  } else {
    LATCtrl_B.MathFunction_g = (uint16_T)(i % n);
  }

  /* End of Math: '<S39>/Math Function' */

  /* DataTypeConversion: '<S39>/Data Type Conversion6' */
  LATCtrl_B.DataTypeConversion6_g = (uint8_T)LATCtrl_B.MathFunction_g;
  for (i_0 = 0; i_0 < 25; i_0++) {
    /* DataStoreRead: '<S39>/Data Store Read' */
    LATCtrl_B.DataStoreRead[i_0] = LATCtrl_DW.LATC_YawRateStore[i_0];

    /* Assignment: '<S39>/Assignment' */
    LATCtrl_B.Assignment[i_0] = LATCtrl_B.DataStoreRead[i_0];
  }

  /* Assignment: '<S39>/Assignment' */
  LATCtrl_B.Assignment[LATCtrl_B.DataTypeConversion6_g] = LATCtrl_B.UnitDelay1;

  /* DataStoreWrite: '<S39>/Data Store Write' */
  memcpy(&LATCtrl_DW.LATC_YawRateStore[0], &LATCtrl_B.Assignment[0], 25U *
         sizeof(real_T));

  /* SignalConversion: '<S38>/ConcatBufferAtMatrix Concatenate1In1' incorporates:
   *  Constant: '<S38>/Constant5'
   */
  LATCtrl_B.MatrixConcatenate1[0] = 1.0;

  /* DataTypeConversion: '<Root>/Data Type Conversion10' incorporates:
   *  Inport: '<Root>/DE_vVehSpdKph'
   */
  LATC_vVehSpdKph = DE_vVehSpdKph;

  /* Product: '<S5>/Divide2' incorporates:
   *  Constant: '<S5>/KPH2MPS'
   */
  LATCtrl_B.Divide2 = LATC_vVehSpdKph / 3.6;

  /* MinMax: '<S5>/MinMax' incorporates:
   *  Constant: '<S5>/Constant2'
   */
  u_0 = LATCtrl_B.Divide2;
  maxval = LATC_vMinVehSpdThd_C;
  if (u_0 >= maxval) {
    maxval = u_0;
  }

  LATCtrl_B.MinMax = maxval;

  /* End of MinMax: '<S5>/MinMax' */

  /* Product: '<S38>/Product3' incorporates:
   *  Constant: '<S31>/Constant11'
   */
  LATCtrl_B.MatrixConcatenate1[1] = LATCtrl_B.MinMax * 0.01;

  /* Concatenate: '<S38>/Matrix Concatenate3' */
  LATCtrl_B.MatrixConcatenate3[0] = LATCtrl_B.MatrixConcatenate1[0];
  LATCtrl_B.MatrixConcatenate3[2] = LATCtrl_B.MatrixConcatenate1[1];
  LATCtrl_B.MatrixConcatenate3[1] = LATCtrl_ConstB.MatrixConcatenate4[0];
  LATCtrl_B.MatrixConcatenate3[3] = LATCtrl_ConstB.MatrixConcatenate4[1];

  /* DataTypeConversion: '<S3>/Data Type Conversion8' incorporates:
   *  Inport: '<Root>/ADCU_tTiDely4MCU2RCAR'
   */
  LATC_tTiDely4MCU2RCAR = (real_T)ADCU_tTiDely4MCU2RCAR;

  /* Switch: '<S3>/Switch1' incorporates:
   *  Constant: '<S3>/TR_swt2'
   *  Constant: '<S3>/TR_swt3'
   */
  if (LATC_swtDelyCycleMan_C != 0) {
    LATC_tDelyLenCycle = LATC_tDelyCycleMan_C;
  } else {
    /* Product: '<S3>/Divide' */
    LATCtrl_B.Divide_hg = LATC_tTiDely4MCU2RCAR / LATCtrl_ConstB.Product3;

    /* DataTypeConversion: '<S3>/Data Type Conversion' */
    u_1 = LATCtrl_B.Divide_hg;
    v_1 = fabs(u_1);
    if (v_1 < 4.503599627370496E+15) {
      if (v_1 >= 0.5) {
        u_1 = floor(u_1 + 0.5);
      } else {
        u_1 = 0.0;
      }
    }

    tmp_5 = fmod(u_1, 65536.0);
    LATCtrl_B.DataTypeConversion_e = (uint16_T)(tmp_5 < 0.0 ? (int32_T)(uint16_T)
      -(int16_T)(uint16_T)-tmp_5 : (int32_T)(uint16_T)tmp_5);

    /* End of DataTypeConversion: '<S3>/Data Type Conversion' */

    /* Saturate: '<S3>/Saturation' */
    i = LATCtrl_B.DataTypeConversion_e;
    n = ((uint16_T)30U);
    if (i <= n) {
      LATCtrl_B.Saturation_k = i;
    } else {
      LATCtrl_B.Saturation_k = n;
    }

    /* End of Saturate: '<S3>/Saturation' */
    LATC_tDelyLenCycle = LATCtrl_B.Saturation_k;
  }

  /* End of Switch: '<S3>/Switch1' */

  /* Switch: '<S39>/Switch1' incorporates:
   *  Constant: '<S39>/C3'
   *  Constant: '<S5>/LATC_swtVarDelyCycle_C'
   */
  if (LATC_swtVarDelyCycle_C != 0) {
    LATCtrl_B.Switch1_dz = LATC_tDelyLenCycle;
  } else {
    LATCtrl_B.Switch1_dz = LATC_iYawRateMaxNum;
  }

  /* End of Switch: '<S39>/Switch1' */

  /* MATLAB Function: '<S39>/Predict' incorporates:
   *  Constant: '<S39>/C3'
   */
  /* MATLAB Function 'LKA_SteerCtrl/K_Filt/LATC_ParamPredict/LATC_MotPredict/Predict': '<S40>:1' */
  /* '<S40>:1:3' x=x0; */
  LATCtrl_B.x_m[0] = LATCtrl_ConstB.MatrixConcatenate3[0];
  LATCtrl_B.x_m[1] = LATCtrl_ConstB.MatrixConcatenate3[1];

  /* '<S40>:1:4' i=uint16(0); */
  /* '<S40>:1:5' while ((i <Dely_Len)) */
  for (i = 0U; i < LATCtrl_B.Switch1_dz; i++) {
    /* '<S40>:1:6' n=mod(index+i+1+(Max_num-Dely_Len),Max_num); */
    tmp_6 = (uint32_T)LATCtrl_B.MathFunction_g + i;
    if (tmp_6 > 65535U) {
      tmp_6 = 65535U;
    }

    tmp_6++;
    if (tmp_6 > 65535U) {
      tmp_6 = 65535U;
    }

    i_0 = LATC_iYawRateMaxNum;
    qY = (uint32_T)i_0 - LATCtrl_B.Switch1_dz;
    if (qY > (uint32_T)i_0) {
      qY = 0U;
    }

    i_0 = (int32_T)qY;
    tmp_6 += i_0;
    if (tmp_6 > 65535U) {
      tmp_6 = 65535U;
    }

    n = (uint16_T)tmp_6;
    if (LATC_iYawRateMaxNum != 0) {
      i_0 = LATC_iYawRateMaxNum;
      n = (uint16_T)((uint32_T)n - (uint16_T)((uint16_T)((uint32_T)i_0 ==
        (uint32_T)0 ? MAX_uint32_T : (uint32_T)n / i_0) * (uint32_T)
        LATC_iYawRateMaxNum));
    }

    /* '<S40>:1:7' x_p=A*x+B*u(n+1); */
    /* '<S40>:1:8' x=x_p; */
    tmp_6 = n + 1U;
    if (tmp_6 > 65535U) {
      tmp_6 = 65535U;
    }

    tmp_5 = LATCtrl_B.Assignment[(int32_T)tmp_6 - 1];
    u_1 = LATCtrl_B.MatrixConcatenate3[0] * LATCtrl_B.x_m[0];
    u_1 += LATCtrl_B.MatrixConcatenate3[2] * LATCtrl_B.x_m[1];
    v_1 = LATCtrl_ConstB.MatrixConcatenate5[0] * tmp_5 + u_1;
    u_1 = LATCtrl_B.MatrixConcatenate3[1] * LATCtrl_B.x_m[0];
    u_1 += LATCtrl_B.MatrixConcatenate3[3] * LATCtrl_B.x_m[1];
    u_1 += LATCtrl_ConstB.MatrixConcatenate5[1] * tmp_5;
    LATCtrl_B.x_m[0] = v_1;
    LATCtrl_B.x_m[1] = u_1;

    /* '<S40>:1:9' i=i+1; */
  }

  /* End of MATLAB Function: '<S39>/Predict' */

  /* UnitDelay: '<S92>/D' */
  LATCtrl_B.D = LATCtrl_DW.D_DSTATE;

  /* DataTypeConversion: '<S11>/Data Type Conversion7' incorporates:
   *  Inport: '<Root>/VMP_cTgtPathCv0'
   */
  LATC_cTgtPathCv0 = VMP_cTgtPathCv0;

  /* DataTypeConversion: '<S13>/Data Type Conversion18' incorporates:
   *  Inport: '<Root>/VMP_lLaneDist1'
   */
  LATC_lLaneDist1 = VMP_lLaneDist1;

  /* Delay: '<S12>/Delay' */
  LATCtrl_B.Delay = LATCtrl_DW.Delay_DSTATE[0];

  /* UnitDelay: '<S12>/Unit Delay' */
  LATCtrl_B.UnitDelay = LATCtrl_DW.UnitDelay_DSTATE;

  /* Abs: '<S12>/Abs' */
  LATCtrl_B.Abs = fabs(LATCtrl_B.UnitDelay);

  /* RelationalOperator: '<S12>/RO1' incorporates:
   *  Constant: '<S12>/Constant6'
   */
  LATCtrl_B.RO1 = (LATCtrl_B.Abs <= LATC_lNorDistUpLmt_C);

  /* RelationalOperator: '<S12>/RO2' incorporates:
   *  Constant: '<S12>/Constant1'
   */
  LATCtrl_B.RO2 = (LATCtrl_B.Abs > LATC_lNorDistLoLmt_C);

  /* Logic: '<S12>/LO1' */
  LATCtrl_B.LO1 = (LATCtrl_B.RO1 && LATCtrl_B.RO2);

  /* DataTypeConversion: '<S13>/Data Type Conversion13' incorporates:
   *  Inport: '<Root>/VMP_lLaneDist0'
   */
  LATC_lLaneDist0 = VMP_lLaneDist0;

  /* Abs: '<S12>/Abs2' */
  LATCtrl_B.Abs2 = fabs(LATC_lLaneDist0);

  /* RelationalOperator: '<S12>/RO3' incorporates:
   *  Constant: '<S12>/Constant3'
   */
  LATCtrl_B.RO3 = (LATCtrl_B.Abs2 >= LATC_lLossDistUpLmt_C);

  /* RelationalOperator: '<S12>/RO4' incorporates:
   *  Constant: '<S12>/Constant2'
   */
  LATCtrl_B.RO4 = (LATCtrl_B.Abs2 < LATC_lLossDistLoLmt_C);

  /* Logic: '<S12>/LO2' */
  LATCtrl_B.LO2 = (LATCtrl_B.RO3 || LATCtrl_B.RO4);

  /* Logic: '<S12>/LO3' */
  LATCtrl_B.LO3 = (LATCtrl_B.LO1 && LATCtrl_B.LO2);

  /* Abs: '<S12>/Abs1' */
  LATCtrl_B.Abs1 = fabs(LATC_lLaneDist0);

  /* RelationalOperator: '<S12>/RO5' incorporates:
   *  Constant: '<S12>/Constant5'
   */
  LATCtrl_B.RO5 = (LATCtrl_B.Abs1 <= LATC_lNorDistUpLmt_C);

  /* RelationalOperator: '<S12>/RO6' incorporates:
   *  Constant: '<S12>/Constant4'
   */
  LATCtrl_B.RO6 = (LATCtrl_B.Abs1 > LATC_lNorDistLoLmt_C);

  /* Logic: '<S12>/LO4' */
  LATCtrl_B.LO4 = (LATCtrl_B.RO5 && LATCtrl_B.RO6);

  /* Memory: '<S14>/Memory' */
  LATCtrl_B.Memory = LATCtrl_DW.Memory_PreviousInput;

  /* CombinatorialLogic: '<S14>/Logic' */
  unnamed_idx = LATCtrl_B.LO3;
  i_0 = unnamed_idx;
  unnamed_idx = LATCtrl_B.LO4;
  i_0 = (int32_T)(((uint32_T)i_0 << 1) + unnamed_idx);
  unnamed_idx = LATCtrl_B.Memory;
  i_0 = (int32_T)(((uint32_T)i_0 << 1) + unnamed_idx);
  LATCtrl_B.Logic[0U] = LATCtrl_ConstP.pooled47[(uint32_T)i_0];
  LATCtrl_B.Logic[1U] = LATCtrl_ConstP.pooled47[i_0 + 8U];

  /* Switch: '<S12>/Switch9' incorporates:
   *  Constant: '<S12>/Constant15'
   */
  if (LATC_swtLaneCheck > ((uint8_T)0U)) {
    /* RelationalOperator: '<S12>/RO17' incorporates:
     *  Constant: '<S12>/LaneAvailSt'
     *  Inport: '<Root>/VMP_stLane0Prob'
     */
    LATCtrl_B.RO17 = ((real_T)VMP_stLane0Prob < 4.0);
    LATC_stLeftLaneLoss = LATCtrl_B.RO17;
  } else {
    LATC_stLeftLaneLoss = LATCtrl_B.Logic[0];
  }

  /* End of Switch: '<S12>/Switch9' */

  /* UnitDelay: '<S12>/Unit Delay1' */
  LATCtrl_B.UnitDelay1_b = LATCtrl_DW.UnitDelay1_DSTATE_j;

  /* Abs: '<S12>/Abs4' */
  LATCtrl_B.Abs4 = fabs(LATCtrl_B.UnitDelay1_b);

  /* RelationalOperator: '<S12>/RO9' incorporates:
   *  Constant: '<S12>/Constant8'
   */
  LATCtrl_B.RO9 = (LATCtrl_B.Abs4 <= LATC_lNorDistUpLmt_C);

  /* RelationalOperator: '<S12>/RO10' incorporates:
   *  Constant: '<S12>/Constant7'
   */
  LATCtrl_B.RO10 = (LATCtrl_B.Abs4 > LATC_lNorDistLoLmt_C);

  /* Logic: '<S12>/LO7' */
  LATCtrl_B.LO7 = (LATCtrl_B.RO9 && LATCtrl_B.RO10);

  /* Abs: '<S12>/Abs6' */
  LATCtrl_B.Abs6 = fabs(LATC_lLaneDist1);

  /* RelationalOperator: '<S12>/RO11' incorporates:
   *  Constant: '<S12>/Constant11'
   */
  LATCtrl_B.RO11 = (LATCtrl_B.Abs6 >= LATC_lLossDistUpLmt_C);

  /* RelationalOperator: '<S12>/RO12' incorporates:
   *  Constant: '<S12>/Constant10'
   */
  LATCtrl_B.RO12 = (LATCtrl_B.Abs6 < LATC_lLossDistLoLmt_C);

  /* Logic: '<S12>/LO8' */
  LATCtrl_B.LO8 = (LATCtrl_B.RO11 || LATCtrl_B.RO12);

  /* Logic: '<S12>/LO9' */
  LATCtrl_B.LO9 = (LATCtrl_B.LO7 && LATCtrl_B.LO8);

  /* Abs: '<S12>/Abs5' */
  LATCtrl_B.Abs5 = fabs(LATC_lLaneDist1);

  /* RelationalOperator: '<S12>/RO13' incorporates:
   *  Constant: '<S12>/Constant12'
   */
  LATCtrl_B.RO13 = (LATCtrl_B.Abs5 <= LATC_lNorDistUpLmt_C);

  /* RelationalOperator: '<S12>/RO14' incorporates:
   *  Constant: '<S12>/Constant13'
   */
  LATCtrl_B.RO14 = (LATCtrl_B.Abs5 > LATC_lNorDistLoLmt_C);

  /* Logic: '<S12>/LO10' */
  LATCtrl_B.LO10 = (LATCtrl_B.RO13 && LATCtrl_B.RO14);

  /* Memory: '<S15>/Memory' */
  LATCtrl_B.Memory_b = LATCtrl_DW.Memory_PreviousInput_j;

  /* CombinatorialLogic: '<S15>/Logic' */
  unnamed_idx = LATCtrl_B.LO9;
  i_0 = unnamed_idx;
  unnamed_idx = LATCtrl_B.LO10;
  i_0 = (int32_T)(((uint32_T)i_0 << 1) + unnamed_idx);
  unnamed_idx = LATCtrl_B.Memory_b;
  i_0 = (int32_T)(((uint32_T)i_0 << 1) + unnamed_idx);
  LATCtrl_B.Logic_n[0U] = LATCtrl_ConstP.pooled47[(uint32_T)i_0];
  LATCtrl_B.Logic_n[1U] = LATCtrl_ConstP.pooled47[i_0 + 8U];

  /* Switch: '<S12>/Switch10' incorporates:
   *  Constant: '<S12>/Constant15'
   */
  if (LATC_swtLaneCheck > ((uint8_T)0U)) {
    /* RelationalOperator: '<S12>/RO19' incorporates:
     *  Constant: '<S12>/LaneAvailSt2'
     *  Inport: '<Root>/VMP_stLane1Prob'
     */
    LATCtrl_B.RO19 = ((real_T)VMP_stLane1Prob < 4.0);
    LATC_stRgtLaneLoss = LATCtrl_B.RO19;
  } else {
    LATC_stRgtLaneLoss = LATCtrl_B.Logic_n[0];
  }

  /* End of Switch: '<S12>/Switch10' */

  /* Logic: '<S12>/LO13' */
  LATCtrl_B.LO13 = (LATC_stLeftLaneLoss || LATC_stRgtLaneLoss);

  /* UnitDelay: '<S12>/Unit Delay6' */
  LATCtrl_B.UnitDelay6_g = LATCtrl_DW.UnitDelay6_DSTATE_o;

  /* Logic: '<S12>/LO15' */
  LATCtrl_B.LO15 = !LATCtrl_B.UnitDelay6_g;

  /* Logic: '<S12>/LO14' */
  LATCtrl_B.LO14 = (LATCtrl_B.LO13 && LATCtrl_B.LO15);

  /* RelationalOperator: '<S12>/RO21' incorporates:
   *  Constant: '<S12>/Constant16'
   */
  LATCtrl_B.RO21 = (LATCtrl_B.Delay <= 4.5);

  /* RelationalOperator: '<S12>/RO22' incorporates:
   *  Constant: '<S12>/Constant19'
   */
  LATCtrl_B.RO22 = (LATCtrl_B.Delay > 2.3);

  /* Logic: '<S12>/LO17' */
  LATCtrl_B.LO17 = (LATCtrl_B.RO21 && LATCtrl_B.RO22);

  /* Logic: '<S12>/LO16' */
  LATCtrl_B.LO16 = (LATCtrl_B.LO14 && LATCtrl_B.LO17);

  /* UnitDelay: '<S12>/Unit Delay2' */
  LATCtrl_B.UnitDelay2 = LATCtrl_DW.UnitDelay2_DSTATE;

  /* Switch: '<S12>/Switch2' */
  if (LATCtrl_B.LO16) {
    LATC_lLaneWidth4Loss = LATCtrl_B.Delay;
  } else {
    LATC_lLaneWidth4Loss = LATCtrl_B.UnitDelay2;
  }

  /* End of Switch: '<S12>/Switch2' */

  /* Switch: '<S12>/Switch11' incorporates:
   *  Constant: '<S12>/Constant15'
   */
  if (LATC_swtLaneCheck > ((uint8_T)0U)) {
    /* RelationalOperator: '<S12>/RO18' incorporates:
     *  Constant: '<S12>/LaneAvailSt1'
     *  Inport: '<Root>/VMP_stLane1Prob'
     */
    LATCtrl_B.RO18 = ((real_T)VMP_stLane1Prob >= 4.0);
    LATC_stRgtLaneAvail = LATCtrl_B.RO18;
  } else {
    /* Abs: '<S12>/Abs3' */
    LATCtrl_B.Abs3_j = fabs(LATC_lLaneDist1);

    /* RelationalOperator: '<S12>/RO8' incorporates:
     *  Constant: '<S12>/Constant17'
     */
    LATCtrl_B.RO8 = (LATCtrl_B.Abs3_j > LATC_lNorDistLoLmt_C);

    /* RelationalOperator: '<S12>/RO7' incorporates:
     *  Constant: '<S12>/Constant18'
     */
    LATCtrl_B.RO7 = (LATCtrl_B.Abs3_j <= LATC_lNorDistUpLmt_C);

    /* Logic: '<S12>/LO6' */
    LATCtrl_B.LO6 = (LATCtrl_B.RO7 && LATCtrl_B.RO8);
    LATC_stRgtLaneAvail = LATCtrl_B.LO6;
  }

  /* End of Switch: '<S12>/Switch11' */

  /* Logic: '<S12>/LO5' */
  LATCtrl_B.LO5 = (LATC_stRgtLaneAvail && LATC_stLeftLaneLoss);

  /* Switch: '<S12>/Switch' */
  if (LATCtrl_B.LO5) {
    /* Sum: '<S12>/Add1' */
    LATCtrl_B.Add1_j = LATC_lLaneDist1 + LATC_lLaneWidth4Loss;
    LATC_facLeftLaneA0 = LATCtrl_B.Add1_j;
  } else {
    LATC_facLeftLaneA0 = LATC_lLaneDist0;
  }

  /* End of Switch: '<S12>/Switch' */

  /* Switch: '<S12>/Switch12' incorporates:
   *  Constant: '<S12>/Constant15'
   */
  if (LATC_swtLaneCheck > ((uint8_T)0U)) {
    /* RelationalOperator: '<S12>/RO20' incorporates:
     *  Constant: '<S12>/LaneAvailSt3'
     *  Inport: '<Root>/VMP_stLane0Prob'
     */
    LATCtrl_B.RO20 = ((real_T)VMP_stLane0Prob >= 4.0);
    LATC_stLeftLaneAvail = LATCtrl_B.RO20;
  } else {
    /* Abs: '<S12>/Abs7' */
    LATCtrl_B.Abs7 = fabs(LATC_lLaneDist0);

    /* RelationalOperator: '<S12>/RO16' incorporates:
     *  Constant: '<S12>/Constant9'
     */
    LATCtrl_B.RO16 = (LATCtrl_B.Abs7 > LATC_lNorDistLoLmt_C);

    /* RelationalOperator: '<S12>/RO15' incorporates:
     *  Constant: '<S12>/Constant14'
     */
    LATCtrl_B.RO15 = (LATCtrl_B.Abs7 <= LATC_lNorDistUpLmt_C);

    /* Logic: '<S12>/LO12' */
    LATCtrl_B.LO12 = (LATCtrl_B.RO15 && LATCtrl_B.RO16);
    LATC_stLeftLaneAvail = LATCtrl_B.LO12;
  }

  /* End of Switch: '<S12>/Switch12' */

  /* Logic: '<S12>/LO11' */
  LATCtrl_B.LO11 = (LATC_stLeftLaneAvail && LATC_stRgtLaneLoss);

  /* Switch: '<S12>/Switch1' */
  if (LATCtrl_B.LO11) {
    /* Sum: '<S12>/Subtract' */
    LATCtrl_B.Subtract_e = LATC_lLaneDist0 - LATC_lLaneWidth4Loss;
    LATC_facRgtLaneA0 = LATCtrl_B.Subtract_e;
  } else {
    LATC_facRgtLaneA0 = LATC_lLaneDist1;
  }

  /* End of Switch: '<S12>/Switch1' */

  /* Switch: '<S16>/Switch2' incorporates:
   *  Constant: '<S5>/LATC_swtPathSel'
   */
  if (LATC_swtPathSel_C != 0) {
    LCC_facCntrLaneA0 = LATC_cTgtPathCv0;
  } else {
    /* Sum: '<S16>/Add6' */
    LATCtrl_B.Add6_h = LATC_facLeftLaneA0 + LATC_facRgtLaneA0;

    /* Product: '<S16>/Divide' incorporates:
     *  Constant: '<S16>/Constant5'
     */
    LATCtrl_B.Divide_ha = LATCtrl_B.Add6_h / 2.0;
    LCC_facCntrLaneA0 = LATCtrl_B.Divide_ha;
  }

  /* End of Switch: '<S16>/Switch2' */

  /* DataTypeConversion: '<S11>/Data Type Conversion12' incorporates:
   *  Inport: '<Root>/VMP_cTgtPathCv1'
   */
  LATC_cTgtPathCv1 = VMP_cTgtPathCv1;

  /* DataTypeConversion: '<S13>/Data Type Conversion19' incorporates:
   *  Inport: '<Root>/VMP_Lane1Cv1'
   */
  LATC_Lane1Cv1 = VMP_Lane1Cv1;

  /* DataTypeConversion: '<S13>/Data Type Conversion15' incorporates:
   *  Inport: '<Root>/VMP_Lane0Cv1'
   */
  LATC_Lane0Cv1 = VMP_Lane0Cv1;

  /* Switch: '<S12>/Switch3' */
  if (LATCtrl_B.LO5) {
    LATC_facLeftLaneA1 = LATC_Lane1Cv1;
  } else {
    LATC_facLeftLaneA1 = LATC_Lane0Cv1;
  }

  /* End of Switch: '<S12>/Switch3' */

  /* Switch: '<S12>/Switch6' */
  if (LATCtrl_B.LO11) {
    LATC_facRgtLaneA1 = LATC_Lane0Cv1;
  } else {
    LATC_facRgtLaneA1 = LATC_Lane1Cv1;
  }

  /* End of Switch: '<S12>/Switch6' */

  /* Switch: '<S16>/Switch1' incorporates:
   *  Constant: '<S5>/LATC_swtPathSel'
   */
  if (LATC_swtPathSel_C != 0) {
    LCC_facCntrLaneA1 = LATC_cTgtPathCv1;
  } else {
    /* Sum: '<S16>/Add1' */
    LATCtrl_B.Add1_cc = LATC_facLeftLaneA1 + LATC_facRgtLaneA1;

    /* Product: '<S16>/Divide1' incorporates:
     *  Constant: '<S16>/Constant1'
     */
    LATCtrl_B.Divide1_o = LATCtrl_B.Add1_cc / 2.0;
    LCC_facCntrLaneA1 = LATCtrl_B.Divide1_o;
  }

  /* End of Switch: '<S16>/Switch1' */

  /* Product: '<S20>/Product3' incorporates:
   *  Constant: '<S20>/PrevTime4LQR'
   */
  LATCtrl_B.Product3 = LATCtrl_B.MinMax * LATC_tPrevtime4FB_C;

  /* Saturate: '<S20>/Saturation' */
  u_0 = LATCtrl_B.Product3;
  v_1 = 0.0;
  u_1 = LC_PrevDistLimitFB;
  if (u_0 >= u_1) {
    LATCtrl_B.Saturation = u_1;
  } else if (u_0 <= v_1) {
    LATCtrl_B.Saturation = v_1;
  } else {
    LATCtrl_B.Saturation = u_0;
  }

  /* End of Saturate: '<S20>/Saturation' */

  /* Product: '<S50>/Product1' */
  LATCtrl_B.Product1 = LCC_facCntrLaneA1 * LATCtrl_B.Saturation;

  /* DataTypeConversion: '<S11>/Data Type Conversion22' incorporates:
   *  Inport: '<Root>/VMP_cTgtPathCv2'
   */
  LATC_cTgtPathCv2 = VMP_cTgtPathCv2;

  /* DataTypeConversion: '<S13>/Data Type Conversion20' incorporates:
   *  Inport: '<Root>/VMP_Lane1Cv2'
   */
  LATC_Lane1Cv2 = VMP_Lane1Cv2;

  /* DataTypeConversion: '<S13>/Data Type Conversion16' incorporates:
   *  Inport: '<Root>/VMP_Lane0Cv2'
   */
  LATC_Lane0Cv2 = VMP_Lane0Cv2;

  /* Switch: '<S12>/Switch4' */
  if (LATCtrl_B.LO5) {
    LATC_facLeftLaneA2 = LATC_Lane1Cv2;
  } else {
    LATC_facLeftLaneA2 = LATC_Lane0Cv2;
  }

  /* End of Switch: '<S12>/Switch4' */

  /* Switch: '<S12>/Switch7' */
  if (LATCtrl_B.LO11) {
    LATC_facRgtLaneA2 = LATC_Lane0Cv2;
  } else {
    LATC_facRgtLaneA2 = LATC_Lane1Cv2;
  }

  /* End of Switch: '<S12>/Switch7' */

  /* Switch: '<S16>/Switch3' incorporates:
   *  Constant: '<S5>/LATC_swtPathSel'
   */
  if (LATC_swtPathSel_C != 0) {
    LCC_facCntrLaneA2 = LATC_cTgtPathCv2;
  } else {
    /* Sum: '<S16>/Add2' */
    LATCtrl_B.Add2_c = LATC_facLeftLaneA2 + LATC_facRgtLaneA2;

    /* Product: '<S16>/Divide2' incorporates:
     *  Constant: '<S16>/Constant2'
     */
    LATCtrl_B.Divide2_o = LATCtrl_B.Add2_c / 2.0;
    LCC_facCntrLaneA2 = LATCtrl_B.Divide2_o;
  }

  /* End of Switch: '<S16>/Switch3' */

  /* Math: '<S50>/Math Function' incorporates:
   *  Constant: '<S50>/Constant1'
   */
  maxval = LATCtrl_B.Saturation;
  a = 2.0;
  if ((maxval < 0.0) && (a > floor(a))) {
    u_0 = -maxval;
    u_1 = pow(u_0, a);
    LATCtrl_B.MathFunction = -u_1;
  } else {
    LATCtrl_B.MathFunction = pow(maxval, a);
  }

  /* End of Math: '<S50>/Math Function' */

  /* Product: '<S50>/Product2' */
  LATCtrl_B.Product2 = LCC_facCntrLaneA2 * LATCtrl_B.MathFunction;

  /* DataTypeConversion: '<S11>/Data Type Conversion23' incorporates:
   *  Inport: '<Root>/VMP_cTgtPathCv3'
   */
  LATC_cTgtPathCv3 = VMP_cTgtPathCv3;

  /* DataTypeConversion: '<S13>/Data Type Conversion21' incorporates:
   *  Inport: '<Root>/VMP_Lane1Cv3'
   */
  LATC_Lane1Cv3 = VMP_Lane1Cv3;

  /* DataTypeConversion: '<S13>/Data Type Conversion17' incorporates:
   *  Inport: '<Root>/VMP_Lane0Cv3'
   */
  LATC_Lane0Cv3 = VMP_Lane0Cv3;

  /* Switch: '<S12>/Switch5' */
  if (LATCtrl_B.LO5) {
    LATC_facLeftLaneA3 = LATC_Lane1Cv3;
  } else {
    LATC_facLeftLaneA3 = LATC_Lane0Cv3;
  }

  /* End of Switch: '<S12>/Switch5' */

  /* Switch: '<S12>/Switch8' */
  if (LATCtrl_B.LO11) {
    LATC_facRgtLaneA3 = LATC_Lane0Cv3;
  } else {
    LATC_facRgtLaneA3 = LATC_Lane1Cv3;
  }

  /* End of Switch: '<S12>/Switch8' */

  /* Switch: '<S16>/Switch4' incorporates:
   *  Constant: '<S5>/LATC_swtPathSel'
   */
  if (LATC_swtPathSel_C != 0) {
    LCC_facCntrLaneA3 = LATC_cTgtPathCv3;
  } else {
    /* Sum: '<S16>/Add4' */
    LATCtrl_B.Add4_b = LATC_facLeftLaneA3 + LATC_facRgtLaneA3;

    /* Product: '<S16>/Divide3' incorporates:
     *  Constant: '<S16>/Constant3'
     */
    LATCtrl_B.Divide3_m = LATCtrl_B.Add4_b / 2.0;
    LCC_facCntrLaneA3 = LATCtrl_B.Divide3_m;
  }

  /* End of Switch: '<S16>/Switch4' */

  /* Math: '<S50>/Math Function1' incorporates:
   *  Constant: '<S50>/Constant2'
   */
  maxval = LATCtrl_B.Saturation;
  a = 3.0;
  if ((maxval < 0.0) && (a > floor(a))) {
    u_0 = -maxval;
    u_1 = pow(u_0, a);
    LATCtrl_B.MathFunction1 = -u_1;
  } else {
    LATCtrl_B.MathFunction1 = pow(maxval, a);
  }

  /* End of Math: '<S50>/Math Function1' */

  /* Product: '<S50>/Product3' */
  LATCtrl_B.Product3_j = LCC_facCntrLaneA3 * LATCtrl_B.MathFunction1;

  /* DataTypeConversion: '<S11>/Data Type Conversion24' incorporates:
   *  Inport: '<Root>/VMP_cTgtPathCv4'
   */
  LATC_cTgtPathCv4 = VMP_cTgtPathCv4;

  /* Switch: '<S16>/Switch5' incorporates:
   *  Constant: '<S16>/LaneCv4'
   *  Constant: '<S5>/LATC_swtPathSel'
   */
  if (LATC_swtPathSel_C != 0) {
    LCC_facCntrLaneA4 = LATC_cTgtPathCv4;
  } else {
    LCC_facCntrLaneA4 = 0.0;
  }

  /* End of Switch: '<S16>/Switch5' */

  /* Math: '<S50>/Math Function2' incorporates:
   *  Constant: '<S50>/Constant3'
   */
  maxval = LATCtrl_B.Saturation;
  a = 4.0;
  if ((maxval < 0.0) && (a > floor(a))) {
    u_0 = -maxval;
    u_1 = pow(u_0, a);
    LATCtrl_B.MathFunction2 = -u_1;
  } else {
    LATCtrl_B.MathFunction2 = pow(maxval, a);
  }

  /* End of Math: '<S50>/Math Function2' */

  /* Product: '<S50>/Product4' */
  LATCtrl_B.Product4 = LCC_facCntrLaneA4 * LATCtrl_B.MathFunction2;

  /* DataTypeConversion: '<S11>/Data Type Conversion25' incorporates:
   *  Inport: '<Root>/VMP_cTgtPathCv5'
   */
  LATC_cTgtPathCv5 = VMP_cTgtPathCv5;

  /* Switch: '<S16>/Switch6' incorporates:
   *  Constant: '<S16>/LaneCv5'
   *  Constant: '<S5>/LATC_swtPathSel'
   */
  if (LATC_swtPathSel_C != 0) {
    LCC_facCntrLaneA5 = LATC_cTgtPathCv5;
  } else {
    LCC_facCntrLaneA5 = 0.0;
  }

  /* End of Switch: '<S16>/Switch6' */

  /* Math: '<S50>/Math Function3' incorporates:
   *  Constant: '<S50>/Constant4'
   */
  maxval = LATCtrl_B.Saturation;
  a = 5.0;
  if ((maxval < 0.0) && (a > floor(a))) {
    u_0 = -maxval;
    u_1 = pow(u_0, a);
    LATCtrl_B.MathFunction3 = -u_1;
  } else {
    LATCtrl_B.MathFunction3 = pow(maxval, a);
  }

  /* End of Math: '<S50>/Math Function3' */

  /* Product: '<S50>/Product5' */
  LATCtrl_B.Product5 = LCC_facCntrLaneA5 * LATCtrl_B.MathFunction3;

  /* Sum: '<S50>/Add3' */
  LC_lPrevLatDist4FB = ((((LCC_facCntrLaneA0 + LATCtrl_B.Product1) +
    LATCtrl_B.Product2) + LATCtrl_B.Product3_j) + LATCtrl_B.Product4) +
    LATCtrl_B.Product5;

  /* DataTypeConversion: '<Root>/Data Type Conversion' incorporates:
   *  Inport: '<Root>/VMP_LC_Flag'
   */
  LATCtrl_B.DataTypeConversion = VMP_LC_Flag;

  /* Switch: '<S28>/Switch1' incorporates:
   *  Constant: '<S28>/PrevTime4LQR'
   *  Constant: '<S5>/LATC_swtVarDelyCycle_C'
   */
  if (LATC_swtVarDelyCycle_C != 0) {
    /* Product: '<S5>/Product4' incorporates:
     *  Constant: '<S5>/CT_S'
     */
    LATCtrl_B.Product4_a = (real_T)LATC_tDelyLenCycle * 0.01;

    /* Sum: '<S28>/Add1' incorporates:
     *  Constant: '<S28>/PrevTime4LQR'
     */
    LATCtrl_B.Add1_c = LATC_tPrevtime4LQR_C + LATCtrl_B.Product4_a;
    LATCtrl_B.Switch1 = LATCtrl_B.Add1_c;
  } else {
    LATCtrl_B.Switch1 = LATC_tPrevtime4LQR_C;
  }

  /* End of Switch: '<S28>/Switch1' */

  /* Product: '<S28>/Product3' */
  LATCtrl_B.Product3_h = LATCtrl_B.MinMax * LATCtrl_B.Switch1;

  /* Saturate: '<S28>/LCC_lPrevDistLimit' */
  u_0 = LATCtrl_B.Product3_h;
  v_1 = 0.0;
  u_1 = LCC_lPrevDistLimit;
  if (u_0 >= u_1) {
    LATCtrl_B.LCC_lPrevDistLimit_p = u_1;
  } else if (u_0 <= v_1) {
    LATCtrl_B.LCC_lPrevDistLimit_p = v_1;
  } else {
    LATCtrl_B.LCC_lPrevDistLimit_p = u_0;
  }

  /* End of Saturate: '<S28>/LCC_lPrevDistLimit' */

  /* DataTypeConversion: '<S11>/Data Type Conversion26' incorporates:
   *  Inport: '<Root>/VMP_lLgtDist2PathZero'
   */
  LATC_lLgtDist2PathZero = VMP_lLgtDist2PathZero;

  /* Switch: '<S28>/Switch2' incorporates:
   *  Constant: '<S28>/LngDistOfst'
   *  Constant: '<S5>/LATC_swtPathSel'
   */
  if (LATC_swtPathSel_C != 0) {
    LATCtrl_B.Switch2 = LATC_lLgtDist2PathZero;
  } else {
    LATCtrl_B.Switch2 = 0.0;
  }

  /* End of Switch: '<S28>/Switch2' */

  /* Sum: '<S28>/Add7' */
  LCC_lPrevDist4LQR = LATCtrl_B.LCC_lPrevDistLimit_p + LATCtrl_B.Switch2;

  /* Product: '<S25>/Product1' */
  LATCtrl_B.Product1_i = LCC_facCntrLaneA1 * LCC_lPrevDist4LQR;

  /* Math: '<S25>/Math Function' incorporates:
   *  Constant: '<S25>/Constant1'
   */
  maxval = LCC_lPrevDist4LQR;
  a = 2.0;
  if ((maxval < 0.0) && (a > floor(a))) {
    u_0 = -maxval;
    u_1 = pow(u_0, a);
    LATCtrl_B.MathFunction_j = -u_1;
  } else {
    LATCtrl_B.MathFunction_j = pow(maxval, a);
  }

  /* End of Math: '<S25>/Math Function' */

  /* Product: '<S25>/Product2' */
  LATCtrl_B.Product2_o = LCC_facCntrLaneA2 * LATCtrl_B.MathFunction_j;

  /* Math: '<S25>/Math Function1' incorporates:
   *  Constant: '<S25>/Constant2'
   */
  maxval = LCC_lPrevDist4LQR;
  a = 3.0;
  if ((maxval < 0.0) && (a > floor(a))) {
    u_0 = -maxval;
    u_1 = pow(u_0, a);
    LATCtrl_B.MathFunction1_n = -u_1;
  } else {
    LATCtrl_B.MathFunction1_n = pow(maxval, a);
  }

  /* End of Math: '<S25>/Math Function1' */

  /* Product: '<S25>/Product3' */
  LATCtrl_B.Product3_f = LCC_facCntrLaneA3 * LATCtrl_B.MathFunction1_n;

  /* Math: '<S25>/Math Function2' incorporates:
   *  Constant: '<S25>/Constant3'
   */
  maxval = LCC_lPrevDist4LQR;
  a = 4.0;
  if ((maxval < 0.0) && (a > floor(a))) {
    u_0 = -maxval;
    u_1 = pow(u_0, a);
    LATCtrl_B.MathFunction2_e = -u_1;
  } else {
    LATCtrl_B.MathFunction2_e = pow(maxval, a);
  }

  /* End of Math: '<S25>/Math Function2' */

  /* Product: '<S25>/Product4' */
  LATCtrl_B.Product4_h = LCC_facCntrLaneA4 * LATCtrl_B.MathFunction2_e;

  /* Math: '<S25>/Math Function3' incorporates:
   *  Constant: '<S25>/Constant4'
   */
  maxval = LCC_lPrevDist4LQR;
  a = 5.0;
  if ((maxval < 0.0) && (a > floor(a))) {
    u_0 = -maxval;
    u_1 = pow(u_0, a);
    LATCtrl_B.MathFunction3_l = -u_1;
  } else {
    LATCtrl_B.MathFunction3_l = pow(maxval, a);
  }

  /* End of Math: '<S25>/Math Function3' */

  /* Product: '<S25>/Product5' */
  LATCtrl_B.Product5_b = LCC_facCntrLaneA5 * LATCtrl_B.MathFunction3_l;

  /* Sum: '<S25>/Add3' */
  LCC_lPrevLatDist4LQR = ((((LCC_facCntrLaneA0 + LATCtrl_B.Product1_i) +
    LATCtrl_B.Product2_o) + LATCtrl_B.Product3_f) + LATCtrl_B.Product4_h) +
    LATCtrl_B.Product5_b;

  /* Switch: '<S25>/Switch' incorporates:
   *  Constant: '<S25>/Prev_strategy_switch'
   */
  if (Prev_strategy_switch != 0) {
    /* Switch: '<S25>/Switch3' */
    if (LATCtrl_B.DataTypeConversion != 0.0) {
      LATCtrl_B.Switch3_nn = LC_lPrevLatDist4FB;
    } else {
      LATCtrl_B.Switch3_nn = LCC_lPrevLatDist4LQR;
    }

    /* End of Switch: '<S25>/Switch3' */
    LATCtrl_B.Switch = LATCtrl_B.Switch3_nn;
  } else {
    LATCtrl_B.Switch = LCC_lPrevLatDist4LQR;
  }

  /* End of Switch: '<S25>/Switch' */

  /* Saturate: '<S25>/Saturation' */
  u_0 = LATCtrl_B.Switch;
  v_1 = (-0.3);
  u_1 = 0.3;
  if (u_0 >= u_1) {
    LATCtrl_B.Saturation_j = u_1;
  } else if (u_0 <= v_1) {
    LATCtrl_B.Saturation_j = v_1;
  } else {
    LATCtrl_B.Saturation_j = u_0;
  }

  /* End of Saturate: '<S25>/Saturation' */

  /* RelationalOperator: '<S92>/R' */
  LATCtrl_B.R = (LATCtrl_B.Saturation_j > LATCtrl_B.D);

  /* Switch: '<S92>/Swt' */
  if (LATCtrl_B.R) {
    /* Product: '<S92>/P1' incorporates:
     *  Constant: '<S25>/Constant'
     *  Constant: '<S25>/LatDistRateLmt'
     */
    LATCtrl_B.P1_o = LatDistRateLmt * 0.01;

    /* Sum: '<S92>/A1' */
    LATCtrl_B.A1_j = LATCtrl_B.P1_o + LATCtrl_B.D;

    /* MinMax: '<S92>/MinMax2' */
    u_0 = LATCtrl_B.A1_j;
    maxval = LATCtrl_B.Saturation_j;
    if (u_0 <= maxval) {
      maxval = u_0;
    }

    LATCtrl_B.MinMax2_b = maxval;

    /* End of MinMax: '<S92>/MinMax2' */
    LatDist4LQR = LATCtrl_B.MinMax2_b;
  } else {
    /* RelationalOperator: '<S92>/R1' */
    LATCtrl_B.R1_p = (LATCtrl_B.Saturation_j < LATCtrl_B.D);

    /* Switch: '<S92>/Swt1' */
    if (LATCtrl_B.R1_p) {
      /* Gain: '<S25>/Gain' incorporates:
       *  Constant: '<S25>/LatDistRateLmt'
       */
      LATCtrl_B.Gain_n = (-1.0) * LatDistRateLmt;

      /* Product: '<S92>/P' incorporates:
       *  Constant: '<S25>/Constant'
       */
      LATCtrl_B.P_k = LATCtrl_B.Gain_n * 0.01;

      /* Sum: '<S92>/A' */
      LATCtrl_B.A_g = LATCtrl_B.D + LATCtrl_B.P_k;

      /* MinMax: '<S92>/MinMax1' */
      u_0 = LATCtrl_B.A_g;
      maxval = LATCtrl_B.Saturation_j;
      if (u_0 >= maxval) {
        maxval = u_0;
      }

      LATCtrl_B.MinMax1_f3 = maxval;

      /* End of MinMax: '<S92>/MinMax1' */
      LATCtrl_B.Swt1_k = LATCtrl_B.MinMax1_f3;
    } else {
      LATCtrl_B.Swt1_k = LATCtrl_B.D;
    }

    /* End of Switch: '<S92>/Swt1' */
    LatDist4LQR = LATCtrl_B.Swt1_k;
  }

  /* End of Switch: '<S92>/Swt' */

  /* Product: '<S17>/Product3' incorporates:
   *  Constant: '<S17>/LATC_facLaneSign'
   */
  LATCtrl_B.Product3_b = LatDist4LQR * LATC_facCoeffSign4Mot_C;

  /* UnitDelay: '<S93>/D' */
  LATCtrl_B.D_a = LATCtrl_DW.D_DSTATE_i;

  /* Math: '<S51>/Math Function2' incorporates:
   *  Constant: '<S51>/Constant6'
   */
  maxval = LATCtrl_B.Saturation;
  a = 4.0;
  if ((maxval < 0.0) && (a > floor(a))) {
    u_0 = -maxval;
    u_1 = pow(u_0, a);
    LATCtrl_B.MathFunction2_k = -u_1;
  } else {
    LATCtrl_B.MathFunction2_k = pow(maxval, a);
  }

  /* End of Math: '<S51>/Math Function2' */

  /* Product: '<S51>/Product7' */
  LATCtrl_B.Product7 = LCC_facCntrLaneA5 * LATCtrl_B.MathFunction2_k;

  /* Product: '<S51>/Product8' incorporates:
   *  Constant: '<S51>/Constant7'
   */
  LATCtrl_B.Product8 = LATCtrl_B.Product7 * 5.0;

  /* Math: '<S51>/Math Function1' incorporates:
   *  Constant: '<S51>/Constant4'
   */
  maxval = LATCtrl_B.Saturation;
  a = 3.0;
  if ((maxval < 0.0) && (a > floor(a))) {
    u_0 = -maxval;
    u_1 = pow(u_0, a);
    LATCtrl_B.MathFunction1_a = -u_1;
  } else {
    LATCtrl_B.MathFunction1_a = pow(maxval, a);
  }

  /* End of Math: '<S51>/Math Function1' */

  /* Product: '<S51>/Product5' */
  LATCtrl_B.Product5_g = LCC_facCntrLaneA4 * LATCtrl_B.MathFunction1_a;

  /* Product: '<S51>/Product6' incorporates:
   *  Constant: '<S51>/Constant5'
   */
  LATCtrl_B.Product6 = LATCtrl_B.Product5_g * 4.0;

  /* Math: '<S51>/Math Function' incorporates:
   *  Constant: '<S51>/Constant1'
   */
  maxval = LATCtrl_B.Saturation;
  a = 2.0;
  if ((maxval < 0.0) && (a > floor(a))) {
    u_0 = -maxval;
    u_1 = pow(u_0, a);
    LATCtrl_B.MathFunction_p = -u_1;
  } else {
    LATCtrl_B.MathFunction_p = pow(maxval, a);
  }

  /* End of Math: '<S51>/Math Function' */

  /* Product: '<S51>/Product2' */
  LATCtrl_B.Product2_p = LCC_facCntrLaneA3 * LATCtrl_B.MathFunction_p;

  /* Product: '<S51>/Product4' incorporates:
   *  Constant: '<S51>/Constant3'
   */
  LATCtrl_B.Product4_i = LATCtrl_B.Product2_p * 3.0;

  /* Product: '<S51>/Product1' */
  LATCtrl_B.Product1_d = LCC_facCntrLaneA2 * LATCtrl_B.Saturation;

  /* Product: '<S51>/Product3' incorporates:
   *  Constant: '<S51>/Constant2'
   */
  LATCtrl_B.Product3_o = LATCtrl_B.Product1_d * 2.0;

  /* Sum: '<S51>/Add3' */
  LC_lPrevHeading4FB = (((LATCtrl_B.Product8 + LATCtrl_B.Product6) +
    LATCtrl_B.Product4_i) + LATCtrl_B.Product3_o) + LCC_facCntrLaneA1;

  /* Math: '<S26>/Math Function2' incorporates:
   *  Constant: '<S26>/Constant6'
   */
  maxval = LCC_lPrevDist4LQR;
  a = 4.0;
  if ((maxval < 0.0) && (a > floor(a))) {
    u_0 = -maxval;
    u_1 = pow(u_0, a);
    LATCtrl_B.MathFunction2_p = -u_1;
  } else {
    LATCtrl_B.MathFunction2_p = pow(maxval, a);
  }

  /* End of Math: '<S26>/Math Function2' */

  /* Product: '<S26>/Product7' */
  LATCtrl_B.Product7_d = LCC_facCntrLaneA5 * LATCtrl_B.MathFunction2_p;

  /* Product: '<S26>/Product8' incorporates:
   *  Constant: '<S26>/Constant7'
   */
  LATCtrl_B.Product8_l = LATCtrl_B.Product7_d * 5.0;

  /* Math: '<S26>/Math Function1' incorporates:
   *  Constant: '<S26>/Constant4'
   */
  maxval = LCC_lPrevDist4LQR;
  a = 3.0;
  if ((maxval < 0.0) && (a > floor(a))) {
    u_0 = -maxval;
    u_1 = pow(u_0, a);
    LATCtrl_B.MathFunction1_m = -u_1;
  } else {
    LATCtrl_B.MathFunction1_m = pow(maxval, a);
  }

  /* End of Math: '<S26>/Math Function1' */

  /* Product: '<S26>/Product5' */
  LATCtrl_B.Product5_f = LCC_facCntrLaneA4 * LATCtrl_B.MathFunction1_m;

  /* Product: '<S26>/Product6' incorporates:
   *  Constant: '<S26>/Constant5'
   */
  LATCtrl_B.Product6_o = LATCtrl_B.Product5_f * 4.0;

  /* Math: '<S26>/Math Function' incorporates:
   *  Constant: '<S26>/Constant1'
   */
  maxval = LCC_lPrevDist4LQR;
  a = 2.0;
  if ((maxval < 0.0) && (a > floor(a))) {
    u_0 = -maxval;
    u_1 = pow(u_0, a);
    LATCtrl_B.MathFunction_h = -u_1;
  } else {
    LATCtrl_B.MathFunction_h = pow(maxval, a);
  }

  /* End of Math: '<S26>/Math Function' */

  /* Product: '<S26>/Product2' */
  LATCtrl_B.Product2_b = LCC_facCntrLaneA3 * LATCtrl_B.MathFunction_h;

  /* Product: '<S26>/Product4' incorporates:
   *  Constant: '<S26>/Constant3'
   */
  LATCtrl_B.Product4_b = LATCtrl_B.Product2_b * 3.0;

  /* Product: '<S26>/Product1' */
  LATCtrl_B.Product1_m = LCC_facCntrLaneA2 * LCC_lPrevDist4LQR;

  /* Product: '<S26>/Product3' incorporates:
   *  Constant: '<S26>/Constant2'
   */
  LATCtrl_B.Product3_i = LATCtrl_B.Product1_m * 2.0;

  /* Sum: '<S26>/Add3' */
  LCC_lPrevHeading4LQR = (((LATCtrl_B.Product8_l + LATCtrl_B.Product6_o) +
    LATCtrl_B.Product4_b) + LATCtrl_B.Product3_i) + LCC_facCntrLaneA1;

  /* Switch: '<S26>/Switch' incorporates:
   *  Constant: '<S26>/Prev_strategy_switch'
   */
  if (Prev_strategy_switch != 0) {
    /* Switch: '<S26>/Switch3' */
    if (LATCtrl_B.DataTypeConversion != 0.0) {
      LATCtrl_B.Switch3_m = LC_lPrevHeading4FB;
    } else {
      LATCtrl_B.Switch3_m = LCC_lPrevHeading4LQR;
    }

    /* End of Switch: '<S26>/Switch3' */
    LATCtrl_B.Switch_o = LATCtrl_B.Switch3_m;
  } else {
    LATCtrl_B.Switch_o = LCC_lPrevHeading4LQR;
  }

  /* End of Switch: '<S26>/Switch' */

  /* Saturate: '<S26>/Saturation' */
  u_0 = LATCtrl_B.Switch_o;
  v_1 = (-0.03);
  u_1 = 0.03;
  if (u_0 >= u_1) {
    LATCtrl_B.Saturation_f = u_1;
  } else if (u_0 <= v_1) {
    LATCtrl_B.Saturation_f = v_1;
  } else {
    LATCtrl_B.Saturation_f = u_0;
  }

  /* End of Saturate: '<S26>/Saturation' */

  /* RelationalOperator: '<S93>/R' */
  LATCtrl_B.R_l = (LATCtrl_B.Saturation_f > LATCtrl_B.D_a);

  /* Switch: '<S93>/Swt' */
  if (LATCtrl_B.R_l) {
    /* Product: '<S93>/P1' incorporates:
     *  Constant: '<S26>/Constant'
     *  Constant: '<S26>/LatHeadRateLmt_rad'
     */
    LATCtrl_B.P1_i = LatHeadRateLmt_rad * 0.01;

    /* Sum: '<S93>/A1' */
    LATCtrl_B.A1_f = LATCtrl_B.P1_i + LATCtrl_B.D_a;

    /* MinMax: '<S93>/MinMax2' */
    u_0 = LATCtrl_B.A1_f;
    maxval = LATCtrl_B.Saturation_f;
    if (u_0 <= maxval) {
      maxval = u_0;
    }

    LATCtrl_B.MinMax2_j = maxval;

    /* End of MinMax: '<S93>/MinMax2' */
    LatHeading4LQR = LATCtrl_B.MinMax2_j;
  } else {
    /* RelationalOperator: '<S93>/R1' */
    LATCtrl_B.R1_j = (LATCtrl_B.Saturation_f < LATCtrl_B.D_a);

    /* Switch: '<S93>/Swt1' */
    if (LATCtrl_B.R1_j) {
      /* Gain: '<S26>/Gain' incorporates:
       *  Constant: '<S26>/LatHeadRateLmt_rad'
       */
      LATCtrl_B.Gain_j = (-1.0) * LatHeadRateLmt_rad;

      /* Product: '<S93>/P' incorporates:
       *  Constant: '<S26>/Constant'
       */
      LATCtrl_B.P_c = LATCtrl_B.Gain_j * 0.01;

      /* Sum: '<S93>/A' */
      LATCtrl_B.A_f = LATCtrl_B.D_a + LATCtrl_B.P_c;

      /* MinMax: '<S93>/MinMax1' */
      u_0 = LATCtrl_B.A_f;
      maxval = LATCtrl_B.Saturation_f;
      if (u_0 >= maxval) {
        maxval = u_0;
      }

      LATCtrl_B.MinMax1_f = maxval;

      /* End of MinMax: '<S93>/MinMax1' */
      LATCtrl_B.Swt1_p = LATCtrl_B.MinMax1_f;
    } else {
      LATCtrl_B.Swt1_p = LATCtrl_B.D_a;
    }

    /* End of Switch: '<S93>/Swt1' */
    LatHeading4LQR = LATCtrl_B.Swt1_p;
  }

  /* End of Switch: '<S93>/Swt' */

  /* Product: '<S17>/Product1' incorporates:
   *  Constant: '<S17>/LATC_facLaneSign'
   */
  LATCtrl_B.Product1_o = LatHeading4LQR * LATC_facCoeffSign4Mot_C;

  /* DataTypeConversion: '<Root>/Data Type Conversion9' incorporates:
   *  Inport: '<Root>/DE_wVehYawRate'
   */
  LATC_wVehYawRate = DE_wVehYawRate;

  /* Switch: '<S57>/Switch3' incorporates:
   *  Constant: '<S57>/LKA_ManYawRateSt'
   *  Constant: '<S57>/LKA_ManYawRate_C'
   */
  if (FALSE) {
    LATCtrl_B.Switch3 = 0.0;
  } else {
    LATCtrl_B.Switch3 = LATC_wVehYawRate;
  }

  /* End of Switch: '<S57>/Switch3' */

  /* Sum: '<S57>/Add3' incorporates:
   *  Constant: '<S57>/LKA_wVehYawRateOfst_C'
   */
  LATCtrl_B.Add3 = LATCtrl_B.Switch3 + 0.0;

  /* Product: '<S17>/Product6' incorporates:
   *  Constant: '<S17>/LATC_facLaneSign1'
   */
  LATCtrl_B.Product6_j = LATCtrl_B.Add3 * LATC_facYRSign4Mot_C;

  /* Switch: '<S37>/Switch' incorporates:
   *  Constant: '<S37>/LATC_swtKalFlt_C1'
   */
  if (LATC_swtMotPredt_C != 0) {
    /* SignalConversion: '<S37>/ConcatBufferAtVector ConcatenateIn3' */
    LATCtrl_B.VectorConcatenate_o[2] = LATCtrl_B.Product6_j;

    /* SignalConversion: '<S37>/ConcatBufferAtVector ConcatenateIn2' */
    LATCtrl_B.VectorConcatenate_o[1] = LATCtrl_B.Product1_o;

    /* SignalConversion: '<S37>/ConcatBufferAtVector ConcatenateIn1' */
    LATCtrl_B.VectorConcatenate_o[0] = LATCtrl_B.Product3_b;
    LATCtrl_B.Switch_c[0] = LATCtrl_B.VectorConcatenate_o[0];
    LATCtrl_B.Switch_c[1] = LATCtrl_B.VectorConcatenate_o[1];
    LATCtrl_B.Switch_c[2] = LATCtrl_B.VectorConcatenate_o[2];
  } else {
    /* SignalConversion: '<S37>/ConcatBufferAtVector Concatenate1In3' */
    LATCtrl_B.VectorConcatenate1_f[2] = LATCtrl_B.Product6_j;

    /* Trigonometry: '<S17>/T_atan' */
    LATCtrl_B.T_atan = atan(LATCtrl_B.x_m[1]);

    /* Sum: '<S37>/Add1' */
    LATCtrl_B.VectorConcatenate1_f[1] = LATCtrl_B.Product1_o + LATCtrl_B.T_atan;

    /* Sum: '<S37>/Add' */
    LATCtrl_B.VectorConcatenate1_f[0] = LATCtrl_B.Product3_b + LATCtrl_B.x_m[0];
    LATCtrl_B.Switch_c[0] = LATCtrl_B.VectorConcatenate1_f[0];
    LATCtrl_B.Switch_c[1] = LATCtrl_B.VectorConcatenate1_f[1];
    LATCtrl_B.Switch_c[2] = LATCtrl_B.VectorConcatenate1_f[2];
  }

  /* End of Switch: '<S37>/Switch' */

  /* Delay: '<S30>/Variable Integer Delay3' */
  if (LATCtrl_DW.icLoad != 0) {
    LATCtrl_DW.VariableIntegerDelay3_DSTATE[0] = LATCtrl_B.Switch_c[0];
    LATCtrl_DW.VariableIntegerDelay3_DSTATE[1] = LATCtrl_B.Switch_c[1];
    LATCtrl_DW.VariableIntegerDelay3_DSTATE[2] = LATCtrl_B.Switch_c[2];
  }

  LATCtrl_B.VariableIntegerDelay3[0] = LATCtrl_DW.VariableIntegerDelay3_DSTATE[0];
  LATCtrl_B.VariableIntegerDelay3[1] = LATCtrl_DW.VariableIntegerDelay3_DSTATE[1];
  LATCtrl_B.VariableIntegerDelay3[2] = LATCtrl_DW.VariableIntegerDelay3_DSTATE[2];

  /* End of Delay: '<S30>/Variable Integer Delay3' */

  /* Switch: '<Root>/Switch' incorporates:
   *  Constant: '<Root>/TR_swt'
   *  Constant: '<Root>/TR_swt1'
   */
  if (LATC_swtADMod_C > ((uint8_T)0U)) {
    /* DataTypeConversion: '<Root>/Data Type Conversion11' incorporates:
     *  Inport: '<Root>/CoST_stADMoEna'
     */
    LATCtrl_B.DataTypeConversion11 = CoST_stADMoEna;
    LATC_stOperMod = LATCtrl_B.DataTypeConversion11;
  } else {
    LATC_stOperMod = LATC_stADMod_C;
  }

  /* End of Switch: '<Root>/Switch' */

  /* RelationalOperator: '<S30>/ROL1' incorporates:
   *  Constant: '<S30>/C6'
   */
  LATCtrl_B.ROL1 = ((real_T)LATC_stOperMod == 3.0);

  /* UnitDelay: '<S30>/Unit Delay' */
  LATCtrl_B.UnitDelay_e = LATCtrl_DW.UnitDelay_DSTATE_b;

  /* Logic: '<S30>/LOA' */
  LATCtrl_B.LOA = (LATCtrl_B.ROL1 && (LATCtrl_B.UnitDelay_e != 0.0));

  /* Switch: '<S30>/Switch' */
  if (LATCtrl_B.LOA) {
    LATCtrl_B.Switch_n[0] = LATCtrl_B.VariableIntegerDelay3[0];
    LATCtrl_B.Switch_n[1] = LATCtrl_B.VariableIntegerDelay3[1];
    LATCtrl_B.Switch_n[2] = LATCtrl_B.VariableIntegerDelay3[2];
  } else {
    LATCtrl_B.Switch_n[0] = LATCtrl_B.Switch_c[0];
    LATCtrl_B.Switch_n[1] = LATCtrl_B.Switch_c[1];
    LATCtrl_B.Switch_n[2] = LATCtrl_B.Switch_c[2];
  }

  /* End of Switch: '<S30>/Switch' */

  /* Trigonometry: '<S35>/T_sin' */
  LATCtrl_B.T_sin = sin(LATCtrl_B.Switch_n[1]);

  /* Product: '<S35>/Product' */
  LATCtrl_B.Product = LATCtrl_B.MinMax * LATCtrl_B.T_sin;

  /* Product: '<S35>/Product1' incorporates:
   *  Constant: '<S30>/CycleTime4Kalman'
   */
  LATCtrl_B.Product1_g = LATCtrl_B.Product * 0.01;

  /* Sum: '<S35>/Add' */
  LATCtrl_B.VectorConcatenate[0] = LATCtrl_B.Switch_n[0] + LATCtrl_B.Product1_g;

  /* Product: '<S20>/Product1' incorporates:
   *  Constant: '<S20>/PrevTime4LQR1'
   */
  LATCtrl_B.Product1_mp = LATCtrl_B.MinMax * LATC_tPrevtime4FF_C;

  /* Saturate: '<S20>/Saturation1' */
  u_0 = LATCtrl_B.Product1_mp;
  v_1 = 0.0;
  u_1 = LC_PrevDistLimitFF;
  if (u_0 >= u_1) {
    LATCtrl_B.Saturation1 = u_1;
  } else if (u_0 <= v_1) {
    LATCtrl_B.Saturation1 = v_1;
  } else {
    LATCtrl_B.Saturation1 = u_0;
  }

  /* End of Saturate: '<S20>/Saturation1' */

  /* Math: '<S52>/Math Function1' incorporates:
   *  Constant: '<S52>/Constant5'
   */
  maxval = LATCtrl_B.Saturation1;
  a = 3.0;
  if ((maxval < 0.0) && (a > floor(a))) {
    u_0 = -maxval;
    u_1 = pow(u_0, a);
    LATCtrl_B.MathFunction1_g = -u_1;
  } else {
    LATCtrl_B.MathFunction1_g = pow(maxval, a);
  }

  /* End of Math: '<S52>/Math Function1' */

  /* Product: '<S52>/Product6' */
  LATCtrl_B.Product6_h = LCC_facCntrLaneA5 * LATCtrl_B.MathFunction1_g;

  /* Product: '<S52>/Product7' incorporates:
   *  Constant: '<S52>/Constant6'
   */
  LATCtrl_B.Product7_h = LATCtrl_B.Product6_h * 20.0;

  /* Math: '<S52>/Math Function' incorporates:
   *  Constant: '<S52>/Constant1'
   */
  maxval = LATCtrl_B.Saturation1;
  a = 2.0;
  if ((maxval < 0.0) && (a > floor(a))) {
    u_0 = -maxval;
    u_1 = pow(u_0, a);
    LATCtrl_B.MathFunction_hi = -u_1;
  } else {
    LATCtrl_B.MathFunction_hi = pow(maxval, a);
  }

  /* End of Math: '<S52>/Math Function' */

  /* Product: '<S52>/Product1' */
  LATCtrl_B.Product1_c = LCC_facCntrLaneA4 * LATCtrl_B.MathFunction_hi;

  /* Product: '<S52>/Product5' incorporates:
   *  Constant: '<S52>/Constant4'
   */
  LATCtrl_B.Product5_a = LATCtrl_B.Product1_c * 12.0;

  /* Product: '<S52>/Product2' */
  LATCtrl_B.Product2_l = LCC_facCntrLaneA3 * LATCtrl_B.Saturation1;

  /* Product: '<S52>/Product4' incorporates:
   *  Constant: '<S52>/Constant3'
   */
  LATCtrl_B.Product4_p = LATCtrl_B.Product2_l * 6.0;

  /* Product: '<S52>/Product3' incorporates:
   *  Constant: '<S52>/Constant2'
   */
  LATCtrl_B.Product3_iz = LCC_facCntrLaneA2 * 2.0;

  /* Sum: '<S52>/Add3' */
  LC_lPrevCv4FF = ((LATCtrl_B.Product7_h + LATCtrl_B.Product5_a) +
                   LATCtrl_B.Product4_p) + LATCtrl_B.Product3_iz;

  /* UnitDelay: '<S94>/D' */
  LATCtrl_B.D_l = LATCtrl_DW.D_DSTATE_p;

  /* Math: '<S27>/Math Function1' incorporates:
   *  Constant: '<S27>/Constant5'
   */
  maxval = LCC_lPrevDist4LQR;
  a = 3.0;
  if ((maxval < 0.0) && (a > floor(a))) {
    u_0 = -maxval;
    u_1 = pow(u_0, a);
    LATCtrl_B.MathFunction1_j = -u_1;
  } else {
    LATCtrl_B.MathFunction1_j = pow(maxval, a);
  }

  /* End of Math: '<S27>/Math Function1' */

  /* Product: '<S27>/Product6' */
  LATCtrl_B.Product6_a = LCC_facCntrLaneA5 * LATCtrl_B.MathFunction1_j;

  /* Product: '<S27>/Product7' incorporates:
   *  Constant: '<S27>/Constant6'
   */
  LATCtrl_B.Product7_b = LATCtrl_B.Product6_a * 20.0;

  /* Math: '<S27>/Math Function' incorporates:
   *  Constant: '<S27>/Constant1'
   */
  maxval = LCC_lPrevDist4LQR;
  a = 2.0;
  if ((maxval < 0.0) && (a > floor(a))) {
    u_0 = -maxval;
    u_1 = pow(u_0, a);
    LATCtrl_B.MathFunction_pq = -u_1;
  } else {
    LATCtrl_B.MathFunction_pq = pow(maxval, a);
  }

  /* End of Math: '<S27>/Math Function' */

  /* Product: '<S27>/Product1' */
  LATCtrl_B.Product1_cu = LCC_facCntrLaneA4 * LATCtrl_B.MathFunction_pq;

  /* Product: '<S27>/Product5' incorporates:
   *  Constant: '<S27>/Constant4'
   */
  LATCtrl_B.Product5_m = LATCtrl_B.Product1_cu * 12.0;

  /* Product: '<S27>/Product2' */
  LATCtrl_B.Product2_c = LCC_facCntrLaneA3 * LCC_lPrevDist4LQR;

  /* Product: '<S27>/Product4' incorporates:
   *  Constant: '<S27>/Constant3'
   */
  LATCtrl_B.Product4_b3 = LATCtrl_B.Product2_c * 6.0;

  /* Product: '<S27>/Product3' incorporates:
   *  Constant: '<S27>/Constant2'
   */
  LATCtrl_B.Product3_c = LCC_facCntrLaneA2 * 2.0;

  /* Sum: '<S27>/Add1' */
  LATCtrl_B.Add1 = ((LATCtrl_B.Product7_b + LATCtrl_B.Product5_m) +
                    LATCtrl_B.Product4_b3) + LATCtrl_B.Product3_c;

  /* RelationalOperator: '<S94>/R' */
  LATCtrl_B.R_k = (LATCtrl_B.Add1 > LATCtrl_B.D_l);

  /* Switch: '<S94>/Swt' */
  if (LATCtrl_B.R_k) {
    /* Product: '<S94>/P1' incorporates:
     *  Constant: '<S27>/Constant'
     *  Constant: '<S27>/LatCvRateLmt'
     */
    LATCtrl_B.P1 = LatCvRateLmt * 0.01;

    /* Sum: '<S94>/A1' */
    LATCtrl_B.A1 = LATCtrl_B.P1 + LATCtrl_B.D_l;

    /* MinMax: '<S94>/MinMax2' */
    u_0 = LATCtrl_B.A1;
    maxval = LATCtrl_B.Add1;
    if (u_0 <= maxval) {
      maxval = u_0;
    }

    LATCtrl_B.MinMax2 = maxval;

    /* End of MinMax: '<S94>/MinMax2' */
    LCC_lPrevCv4LQR = LATCtrl_B.MinMax2;
  } else {
    /* RelationalOperator: '<S94>/R1' */
    LATCtrl_B.R1_m = (LATCtrl_B.Add1 < LATCtrl_B.D_l);

    /* Switch: '<S94>/Swt1' */
    if (LATCtrl_B.R1_m) {
      /* Gain: '<S27>/Gain' incorporates:
       *  Constant: '<S27>/LatCvRateLmt'
       */
      LATCtrl_B.Gain_h = (-1.0) * LatCvRateLmt;

      /* Product: '<S94>/P' incorporates:
       *  Constant: '<S27>/Constant'
       */
      LATCtrl_B.P = LATCtrl_B.Gain_h * 0.01;

      /* Sum: '<S94>/A' */
      LATCtrl_B.A = LATCtrl_B.D_l + LATCtrl_B.P;

      /* MinMax: '<S94>/MinMax1' */
      u_0 = LATCtrl_B.A;
      maxval = LATCtrl_B.Add1;
      if (u_0 >= maxval) {
        maxval = u_0;
      }

      LATCtrl_B.MinMax1 = maxval;

      /* End of MinMax: '<S94>/MinMax1' */
      LATCtrl_B.Swt1 = LATCtrl_B.MinMax1;
    } else {
      LATCtrl_B.Swt1 = LATCtrl_B.D_l;
    }

    /* End of Switch: '<S94>/Swt1' */
    LCC_lPrevCv4LQR = LATCtrl_B.Swt1;
  }

  /* End of Switch: '<S94>/Swt' */

  /* Switch: '<S27>/Switch' incorporates:
   *  Constant: '<S27>/Prev_strategy_switch'
   */
  if (Prev_strategy_switch != 0) {
    /* Switch: '<S27>/Switch3' */
    if (LATCtrl_B.DataTypeConversion != 0.0) {
      LATCtrl_B.Switch3_b = LC_lPrevCv4FF;
    } else {
      LATCtrl_B.Switch3_b = LCC_lPrevCv4LQR;
    }

    /* End of Switch: '<S27>/Switch3' */
    LATCtrl_B.Switch_o3 = LATCtrl_B.Switch3_b;
  } else {
    LATCtrl_B.Switch_o3 = LCC_lPrevCv4LQR;
  }

  /* End of Switch: '<S27>/Switch' */

  /* Saturate: '<S27>/Saturation' */
  u_0 = LATCtrl_B.Switch_o3;
  v_1 = (-0.02);
  u_1 = 0.02;
  if (u_0 >= u_1) {
    LatCv4LQR = u_1;
  } else if (u_0 <= v_1) {
    LatCv4LQR = v_1;
  } else {
    LatCv4LQR = u_0;
  }

  /* End of Saturate: '<S27>/Saturation' */

  /* Product: '<S17>/Product2' incorporates:
   *  Constant: '<S17>/LATC_facLaneSign'
   */
  LATCtrl_B.Product2_ca = LatCv4LQR * LATC_facCoeffSign4Mot_C;

  /* Product: '<S35>/Product3' */
  LATCtrl_B.Product3_ju = LATCtrl_B.MinMax * LATCtrl_B.Product2_ca;

  /* Sum: '<S35>/Add2' */
  LATCtrl_B.Add2 = LATCtrl_B.Switch_n[2] + LATCtrl_B.Product3_ju;

  /* Product: '<S35>/Product2' incorporates:
   *  Constant: '<S30>/CycleTime4Kalman'
   */
  LATCtrl_B.Product2_c5 = LATCtrl_B.Add2 * 0.01;

  /* Sum: '<S35>/Add1' */
  LATCtrl_B.VectorConcatenate[1] = LATCtrl_B.Switch_n[1] + LATCtrl_B.Product2_c5;

  /* SignalConversion: '<S35>/ConcatBufferAtVector ConcatenateIn3' */
  LATCtrl_B.VectorConcatenate[2] = LATCtrl_B.Switch_n[2];

  /* Trigonometry: '<S36>/T_cos' */
  LATCtrl_B.T_cos = cos(LATCtrl_B.Switch_n[1]);

  /* Product: '<S36>/Product' */
  LATCtrl_B.Product_h = LATCtrl_B.MinMax * LATCtrl_B.T_cos;

  /* Product: '<S36>/Product1' incorporates:
   *  Constant: '<S30>/CycleTime4Kalman'
   */
  LATCtrl_B.Product1_c3 = LATCtrl_B.Product_h * 0.01;

  /* SignalConversion: '<S36>/TmpSignal ConversionAtMath Function1Inport1' incorporates:
   *  Constant: '<S36>/Constant3'
   *  Constant: '<S36>/Constant4'
   */
  LATCtrl_B.TmpSignalConversionAtMathFuncti[0] = LATCtrl_B.Product1_c3;
  LATCtrl_B.TmpSignalConversionAtMathFuncti[1] = 1.0;
  LATCtrl_B.TmpSignalConversionAtMathFuncti[2] = 0.0;

  /* Math: '<S36>/Math Function1' */
  LATCtrl_B.MathFunction1_d[0] = LATCtrl_B.TmpSignalConversionAtMathFuncti[0];
  LATCtrl_B.MathFunction1_d[1] = LATCtrl_B.TmpSignalConversionAtMathFuncti[1];
  LATCtrl_B.MathFunction1_d[2] = LATCtrl_B.TmpSignalConversionAtMathFuncti[2];

  /* Concatenate: '<S36>/Vector Concatenate' */
  LATCtrl_B.VectorConcatenate_d[0] = LATCtrl_ConstB.MathFunction[0];
  LATCtrl_B.VectorConcatenate_d[3] = LATCtrl_ConstB.MathFunction[1];
  LATCtrl_B.VectorConcatenate_d[6] = LATCtrl_ConstB.MathFunction[2];
  LATCtrl_B.VectorConcatenate_d[1] = LATCtrl_B.MathFunction1_d[0];
  LATCtrl_B.VectorConcatenate_d[4] = LATCtrl_B.MathFunction1_d[1];
  LATCtrl_B.VectorConcatenate_d[7] = LATCtrl_B.MathFunction1_d[2];
  LATCtrl_B.VectorConcatenate_d[2] = LATCtrl_ConstB.MathFunction2[0];
  LATCtrl_B.VectorConcatenate_d[5] = LATCtrl_ConstB.MathFunction2[1];
  LATCtrl_B.VectorConcatenate_d[8] = LATCtrl_ConstB.MathFunction2[2];

  /* UnitDelay: '<S33>/Unit Delay1' */
  LATCtrl_B.UnitDelay1_p = LATCtrl_DW.UnitDelay1_DSTATE_i;

  /* Sum: '<S33>/Sum6' incorporates:
   *  Constant: '<S33>/C3'
   */
  LATCtrl_B.Sum6 = 1.0 + LATCtrl_B.UnitDelay1_p;

  /* Sum: '<S33>/Add2' incorporates:
   *  Constant: '<S33>/ExC1_C3'
   */
  LATCtrl_B.Add2_e = (real_T)((uint8_T)0U) + LATCtrl_B.Sum6;

  /* Math: '<S33>/Math Function' incorporates:
   *  Constant: '<S33>/C4'
   */
  LATCtrl_B.MathFunction_i = rt_modd(LATCtrl_B.Add2_e, 5.0);

  /* RelationalOperator: '<S33>/ROL' incorporates:
   *  Constant: '<S33>/C5'
   */
  LATCtrl_B.ROL = (LATCtrl_B.MathFunction_i <= 1.5);

  /* RelationalOperator: '<S33>/ROL1' incorporates:
   *  Constant: '<S33>/C6'
   */
  LATCtrl_B.ROL1_p = (LATCtrl_B.MathFunction_i >= 0.5);

  /* Logic: '<S33>/LOA' */
  LATCtrl_B.LOA_k = (LATCtrl_B.ROL && LATCtrl_B.ROL1_p);

  /* Switch: '<S30>/Switch2' */
  if (LATCtrl_B.LOA_k) {
    LATCtrl_B.Switch2_d[0] = LATCtrl_B.Switch_c[0];
    LATCtrl_B.Switch2_d[1] = LATCtrl_B.Switch_c[1];
    LATCtrl_B.Switch2_d[2] = LATCtrl_B.Switch_c[2];
  } else {
    LATCtrl_B.Switch2_d[0] = LATCtrl_B.VectorConcatenate[0];
    LATCtrl_B.Switch2_d[1] = LATCtrl_B.VectorConcatenate[1];
    LATCtrl_B.Switch2_d[2] = LATCtrl_B.VectorConcatenate[2];
  }

  /* End of Switch: '<S30>/Switch2' */

  /* SignalConversion: '<S32>/TmpSignal ConversionAtMath FunctionInport1' incorporates:
   *  Constant: '<S32>/Constant2'
   *  Constant: '<S32>/Constant3'
   *  Constant: '<S32>/Constant4'
   */
  LATCtrl_B.TmpSignalConversionAtMathFunc_b[0] = LATC_facWgt4KalQ1_C;
  LATCtrl_B.TmpSignalConversionAtMathFunc_b[1] = 0.0;
  LATCtrl_B.TmpSignalConversionAtMathFunc_b[2] = 0.0;

  /* Math: '<S32>/Math Function' */
  LATCtrl_B.MathFunction_f[0] = LATCtrl_B.TmpSignalConversionAtMathFunc_b[0];
  LATCtrl_B.MathFunction_f[1] = LATCtrl_B.TmpSignalConversionAtMathFunc_b[1];
  LATCtrl_B.MathFunction_f[2] = LATCtrl_B.TmpSignalConversionAtMathFunc_b[2];

  /* SignalConversion: '<S32>/TmpSignal ConversionAtMath Function1Inport1' incorporates:
   *  Constant: '<S32>/Constant5'
   *  Constant: '<S32>/Constant6'
   *  Constant: '<S32>/Constant7'
   */
  LATCtrl_B.TmpSignalConversionAtMathFunc_d[0] = 0.0;
  LATCtrl_B.TmpSignalConversionAtMathFunc_d[1] = LATC_facWgt4KalQ2_C;
  LATCtrl_B.TmpSignalConversionAtMathFunc_d[2] = 0.0;

  /* Math: '<S32>/Math Function1' */
  LATCtrl_B.MathFunction1_jb[0] = LATCtrl_B.TmpSignalConversionAtMathFunc_d[0];
  LATCtrl_B.MathFunction1_jb[1] = LATCtrl_B.TmpSignalConversionAtMathFunc_d[1];
  LATCtrl_B.MathFunction1_jb[2] = LATCtrl_B.TmpSignalConversionAtMathFunc_d[2];

  /* SignalConversion: '<S32>/TmpSignal ConversionAtMath Function2Inport1' incorporates:
   *  Constant: '<S32>/Constant10'
   *  Constant: '<S32>/Constant8'
   *  Constant: '<S32>/Constant9'
   */
  LATCtrl_B.TmpSignalConversionAtMathFunc_j[0] = 0.0;
  LATCtrl_B.TmpSignalConversionAtMathFunc_j[1] = 0.0;
  LATCtrl_B.TmpSignalConversionAtMathFunc_j[2] = LATC_facWgt4KalQ3_C;

  /* Math: '<S32>/Math Function2' */
  LATCtrl_B.MathFunction2_d[0] = LATCtrl_B.TmpSignalConversionAtMathFunc_j[0];
  LATCtrl_B.MathFunction2_d[1] = LATCtrl_B.TmpSignalConversionAtMathFunc_j[1];
  LATCtrl_B.MathFunction2_d[2] = LATCtrl_B.TmpSignalConversionAtMathFunc_j[2];

  /* Concatenate: '<S32>/Vector Concatenate' */
  LATCtrl_B.VectorConcatenate_p[0] = LATCtrl_B.MathFunction_f[0];
  LATCtrl_B.VectorConcatenate_p[3] = LATCtrl_B.MathFunction_f[1];
  LATCtrl_B.VectorConcatenate_p[6] = LATCtrl_B.MathFunction_f[2];
  LATCtrl_B.VectorConcatenate_p[1] = LATCtrl_B.MathFunction1_jb[0];
  LATCtrl_B.VectorConcatenate_p[4] = LATCtrl_B.MathFunction1_jb[1];
  LATCtrl_B.VectorConcatenate_p[7] = LATCtrl_B.MathFunction1_jb[2];
  LATCtrl_B.VectorConcatenate_p[2] = LATCtrl_B.MathFunction2_d[0];
  LATCtrl_B.VectorConcatenate_p[5] = LATCtrl_B.MathFunction2_d[1];
  LATCtrl_B.VectorConcatenate_p[8] = LATCtrl_B.MathFunction2_d[2];

  /* SignalConversion: '<S32>/TmpSignal ConversionAtMath Function3Inport1' incorporates:
   *  Constant: '<S32>/Constant12'
   *  Constant: '<S32>/Constant13'
   *  Constant: '<S32>/Constant14'
   */
  LATCtrl_B.TmpSignalConversionAtMathFunc_o[0] = LATC_facWgt4KalR1_C;
  LATCtrl_B.TmpSignalConversionAtMathFunc_o[1] = 0.0;
  LATCtrl_B.TmpSignalConversionAtMathFunc_o[2] = 0.0;

  /* Math: '<S32>/Math Function3' */
  LATCtrl_B.MathFunction3_a[0] = LATCtrl_B.TmpSignalConversionAtMathFunc_o[0];
  LATCtrl_B.MathFunction3_a[1] = LATCtrl_B.TmpSignalConversionAtMathFunc_o[1];
  LATCtrl_B.MathFunction3_a[2] = LATCtrl_B.TmpSignalConversionAtMathFunc_o[2];

  /* SignalConversion: '<S32>/TmpSignal ConversionAtMath Function4Inport1' incorporates:
   *  Constant: '<S32>/Constant15'
   *  Constant: '<S32>/Constant16'
   *  Constant: '<S32>/Constant17'
   */
  LATCtrl_B.TmpSignalConversionAtMathFunc_k[0] = 0.0;
  LATCtrl_B.TmpSignalConversionAtMathFunc_k[1] = LATC_facWgt4KalR2_C;
  LATCtrl_B.TmpSignalConversionAtMathFunc_k[2] = 0.0;

  /* Math: '<S32>/Math Function4' */
  LATCtrl_B.MathFunction4[0] = LATCtrl_B.TmpSignalConversionAtMathFunc_k[0];
  LATCtrl_B.MathFunction4[1] = LATCtrl_B.TmpSignalConversionAtMathFunc_k[1];
  LATCtrl_B.MathFunction4[2] = LATCtrl_B.TmpSignalConversionAtMathFunc_k[2];

  /* SignalConversion: '<S32>/TmpSignal ConversionAtMath Function5Inport1' incorporates:
   *  Constant: '<S32>/Constant11'
   *  Constant: '<S32>/Constant18'
   *  Constant: '<S32>/Constant19'
   */
  LATCtrl_B.TmpSignalConversionAtMathFun_ou[0] = 0.0;
  LATCtrl_B.TmpSignalConversionAtMathFun_ou[1] = 0.0;
  LATCtrl_B.TmpSignalConversionAtMathFun_ou[2] = LATC_facWgt4KalR3_C;

  /* Math: '<S32>/Math Function5' */
  LATCtrl_B.MathFunction5[0] = LATCtrl_B.TmpSignalConversionAtMathFun_ou[0];
  LATCtrl_B.MathFunction5[1] = LATCtrl_B.TmpSignalConversionAtMathFun_ou[1];
  LATCtrl_B.MathFunction5[2] = LATCtrl_B.TmpSignalConversionAtMathFun_ou[2];

  /* Concatenate: '<S32>/Vector Concatenate1' */
  LATCtrl_B.VectorConcatenate1[0] = LATCtrl_B.MathFunction3_a[0];
  LATCtrl_B.VectorConcatenate1[3] = LATCtrl_B.MathFunction3_a[1];
  LATCtrl_B.VectorConcatenate1[6] = LATCtrl_B.MathFunction3_a[2];
  LATCtrl_B.VectorConcatenate1[1] = LATCtrl_B.MathFunction4[0];
  LATCtrl_B.VectorConcatenate1[4] = LATCtrl_B.MathFunction4[1];
  LATCtrl_B.VectorConcatenate1[7] = LATCtrl_B.MathFunction4[2];
  LATCtrl_B.VectorConcatenate1[2] = LATCtrl_B.MathFunction5[0];
  LATCtrl_B.VectorConcatenate1[5] = LATCtrl_B.MathFunction5[1];
  LATCtrl_B.VectorConcatenate1[8] = LATCtrl_B.MathFunction5[2];
  for (i_0 = 0; i_0 < 9; i_0++) {
    /* Delay: '<S30>/Variable Integer Delay' */
    if (LATCtrl_DW.icLoad_p != 0) {
      LATCtrl_DW.VariableIntegerDelay_DSTATE[i_0] =
        LATCtrl_B.VectorConcatenate_p[i_0];
    }

    LATCtrl_B.VariableIntegerDelay[i_0] =
      LATCtrl_DW.VariableIntegerDelay_DSTATE[i_0];

    /* End of Delay: '<S30>/Variable Integer Delay' */

    /* Switch: '<S30>/Switch1' */
    if (LATCtrl_B.LOA) {
      LATCtrl_B.Switch1_f[i_0] = LATCtrl_B.VariableIntegerDelay[i_0];
    } else {
      LATCtrl_B.Switch1_f[i_0] = LATCtrl_B.VectorConcatenate_p[i_0];
    }

    /* End of Switch: '<S30>/Switch1' */
  }

  /* MATLAB Function: '<S30>/LQR_solver' incorporates:
   *  Constant: '<S32>/Constant'
   *  Constant: '<S32>/Constant1'
   */
  /* MATLAB Function 'LKA_SteerCtrl/K_Filt/Kalman/LQR_solver': '<S34>:1' */
  /* %1. ״̬����x(k+1)=A*x(k)+b*u(k+1) */
  /* %x_k2 =A*x+ B*u; */
  /* %2. ����״̬���Э�������sw�ǹ�������Э���� */
  /* '<S34>:1:7' Pk1_ = F*Pk1*F' + Q; */
  for (i_0 = 0; i_0 < 3; i_0++) {
    for (r = 0; r < 3; r++) {
      tmp[i_0 + 3 * r] = 0.0;
      tmp[i_0 + 3 * r] += LATCtrl_B.Switch1_f[3 * r] *
        LATCtrl_B.VectorConcatenate_d[i_0];
      tmp[i_0 + 3 * r] += LATCtrl_B.Switch1_f[3 * r + 1] *
        LATCtrl_B.VectorConcatenate_d[i_0 + 3];
      tmp[i_0 + 3 * r] += LATCtrl_B.Switch1_f[3 * r + 2] *
        LATCtrl_B.VectorConcatenate_d[i_0 + 6];
    }
  }

  for (i_0 = 0; i_0 < 3; i_0++) {
    for (r = 0; r < 3; r++) {
      tmp_5 = tmp[i_0] * LATCtrl_B.VectorConcatenate_d[r];
      tmp_5 += tmp[i_0 + 3] * LATCtrl_B.VectorConcatenate_d[r + 3];
      tmp_5 += tmp[i_0 + 6] * LATCtrl_B.VectorConcatenate_d[r + 6];
      Pk[i_0 + 3 * r] = LATCtrl_B.VectorConcatenate_p[3 * r + i_0] + tmp_5;
    }
  }

  /* %3. ���㿨�������棬szΪ��������Э���� */
  /* '<S34>:1:11' K = Pk1_*C' / (C*Pk1_*C' + R); */
  for (i_0 = 0; i_0 < 3; i_0++) {
    for (r = 0; r < 3; r++) {
      tmp[i_0 + 3 * r] = 0.0;
      tmp[i_0 + 3 * r] += Pk[3 * r] * LATCtrl_ConstP.pooled7[i_0];
      tmp[i_0 + 3 * r] += Pk[3 * r + 1] * LATCtrl_ConstP.pooled7[i_0 + 3];
      tmp[i_0 + 3 * r] += Pk[3 * r + 2] * LATCtrl_ConstP.pooled7[i_0 + 6];
    }
  }

  for (i_0 = 0; i_0 < 3; i_0++) {
    for (r = 0; r < 3; r++) {
      tmp_5 = tmp[r] * LATCtrl_ConstP.pooled7[i_0];
      tmp_5 += tmp[r + 3] * LATCtrl_ConstP.pooled7[i_0 + 3];
      tmp_5 += tmp[r + 6] * LATCtrl_ConstP.pooled7[i_0 + 6];
      A[i_0 + 3 * r] = LATCtrl_B.VectorConcatenate1[3 * i_0 + r] + tmp_5;
    }
  }

  for (i_0 = 0; i_0 < 3; i_0++) {
    for (r = 0; r < 3; r++) {
      B[i_0 + 3 * r] = 0.0;
      B[i_0 + 3 * r] += Pk[r] * LATCtrl_ConstP.pooled7[i_0];
      B[i_0 + 3 * r] += Pk[r + 3] * LATCtrl_ConstP.pooled7[i_0 + 3];
      B[i_0 + 3 * r] += Pk[r + 6] * LATCtrl_ConstP.pooled7[i_0 + 6];
    }
  }

  i_0 = 0;
  r = 1;
  r_0 = 2;
  maxval = fabs(A[0]);
  a = fabs(A[1]);
  if (a > maxval) {
    maxval = a;
    i_0 = 1;
    r = 0;
  }

  if (fabs(A[2]) > maxval) {
    i_0 = 2;
    r = 1;
    r_0 = 0;
  }

  A[r] /= A[i_0];
  A[r_0] /= A[i_0];
  A[3 + r] -= A[3 + i_0] * A[r];
  A[3 + r_0] -= A[3 + i_0] * A[r_0];
  A[6 + r] -= A[6 + i_0] * A[r];
  A[6 + r_0] -= A[6 + i_0] * A[r_0];
  if (fabs(A[3 + r_0]) > fabs(A[3 + r])) {
    rtemp = r;
    r = r_0;
    r_0 = rtemp;
  }

  A[3 + r_0] /= A[3 + r];
  A[6 + r_0] -= A[3 + r_0] * A[6 + r];
  Y[0] = B[i_0];
  Y[1] = B[r] - Y[0] * A[r];
  Y[2] = (B[r_0] - Y[0] * A[r_0]) - A[3 + r_0] * Y[1];
  Y[2] /= A[6 + r_0];
  Y[0] -= A[6 + i_0] * Y[2];
  Y[1] -= A[6 + r] * Y[2];
  Y[1] /= A[3 + r];
  Y[0] -= A[3 + i_0] * Y[1];
  Y[0] /= A[i_0];
  Y[3] = B[i_0 + 3];
  Y[4] = B[r + 3] - Y[3] * A[r];
  Y[5] = (B[r_0 + 3] - Y[3] * A[r_0]) - A[3 + r_0] * Y[4];
  Y[5] /= A[6 + r_0];
  Y[3] -= A[6 + i_0] * Y[5];
  Y[4] -= A[6 + r] * Y[5];
  Y[4] /= A[3 + r];
  Y[3] -= A[3 + i_0] * Y[4];
  Y[3] /= A[i_0];
  Y[6] = B[i_0 + 6];
  Y[7] = B[r + 6] - Y[6] * A[r];
  Y[8] = (B[r_0 + 6] - Y[6] * A[r_0]) - A[3 + r_0] * Y[7];
  Y[8] /= A[6 + r_0];
  Y[6] -= A[6 + i_0] * Y[8];
  Y[7] -= A[6 + r] * Y[8];
  Y[7] /= A[3 + r];
  Y[6] -= A[3 + i_0] * Y[7];
  Y[6] /= A[i_0];
  for (i_0 = 0; i_0 < 3; i_0++) {
    LATCtrl_B.K_i[3 * i_0] = Y[i_0];
    LATCtrl_B.K_i[1 + 3 * i_0] = Y[i_0 + 3];
    LATCtrl_B.K_i[2 + 3 * i_0] = Y[i_0 + 6];
  }

  /* %4. �õ�ǰ����ֵ����״̬����ֵ������K+1ʱ�̵�״̬ */
  /* '<S34>:1:14' xhat = x + K*(y - C*x); */
  /* %5. �������Э���� */
  /* '<S34>:1:17' Pk2 = (I - K*C) * Pk1_ * (I - K*C)' + K*R*K'; */
  for (i_0 = 0; i_0 < 3; i_0++) {
    tmp_5 = LATCtrl_ConstP.pooled7[i_0] * LATCtrl_B.VectorConcatenate[0];
    tmp_5 += LATCtrl_ConstP.pooled7[i_0 + 3] * LATCtrl_B.VectorConcatenate[1];
    tmp_5 += LATCtrl_ConstP.pooled7[i_0 + 6] * LATCtrl_B.VectorConcatenate[2];
    tmp_0[i_0] = LATCtrl_B.Switch2_d[i_0] - tmp_5;
  }

  for (i_0 = 0; i_0 < 3; i_0++) {
    tmp_5 = LATCtrl_B.K_i[i_0] * tmp_0[0];
    tmp_5 += LATCtrl_B.K_i[i_0 + 3] * tmp_0[1];
    tmp_5 += LATCtrl_B.K_i[i_0 + 6] * tmp_0[2];
    LATCtrl_B.xhat[i_0] = LATCtrl_B.VectorConcatenate[i_0] + tmp_5;
  }

  for (i_0 = 0; i_0 < 3; i_0++) {
    for (r = 0; r < 3; r++) {
      tmp_5 = LATCtrl_ConstP.pooled7[3 * r] * LATCtrl_B.K_i[i_0];
      tmp_5 += LATCtrl_ConstP.pooled7[3 * r + 1] * LATCtrl_B.K_i[i_0 + 3];
      tmp_5 += LATCtrl_ConstP.pooled7[3 * r + 2] * LATCtrl_B.K_i[i_0 + 6];
      tmp[i_0 + 3 * r] = LATCtrl_ConstP.pooled7[3 * r + i_0] - tmp_5;
    }
  }

  for (i_0 = 0; i_0 < 3; i_0++) {
    for (r = 0; r < 3; r++) {
      B[i_0 + 3 * r] = 0.0;
      B[i_0 + 3 * r] += Pk[3 * r] * tmp[i_0];
      B[i_0 + 3 * r] += Pk[3 * r + 1] * tmp[i_0 + 3];
      B[i_0 + 3 * r] += Pk[3 * r + 2] * tmp[i_0 + 6];
    }
  }

  for (i_0 = 0; i_0 < 3; i_0++) {
    for (r = 0; r < 3; r++) {
      tmp_5 = LATCtrl_ConstP.pooled7[3 * i_0] * LATCtrl_B.K_i[r];
      tmp_5 += LATCtrl_ConstP.pooled7[3 * i_0 + 1] * LATCtrl_B.K_i[r + 3];
      tmp_5 += LATCtrl_ConstP.pooled7[3 * i_0 + 2] * LATCtrl_B.K_i[r + 6];
      tmp[i_0 + 3 * r] = LATCtrl_ConstP.pooled7[3 * i_0 + r] - tmp_5;
    }
  }

  for (i_0 = 0; i_0 < 3; i_0++) {
    for (r = 0; r < 3; r++) {
      A[i_0 + 3 * r] = 0.0;
      A[i_0 + 3 * r] += LATCtrl_B.VectorConcatenate1[3 * r] * LATCtrl_B.K_i[i_0];
      A[i_0 + 3 * r] += LATCtrl_B.VectorConcatenate1[3 * r + 1] *
        LATCtrl_B.K_i[i_0 + 3];
      A[i_0 + 3 * r] += LATCtrl_B.VectorConcatenate1[3 * r + 2] *
        LATCtrl_B.K_i[i_0 + 6];
    }
  }

  for (i_0 = 0; i_0 < 3; i_0++) {
    for (r = 0; r < 3; r++) {
      Y[i_0 + 3 * r] = 0.0;
      Y[i_0 + 3 * r] += tmp[3 * r] * B[i_0];
      Y[i_0 + 3 * r] += tmp[3 * r + 1] * B[i_0 + 3];
      Y[i_0 + 3 * r] += tmp[3 * r + 2] * B[i_0 + 6];
    }
  }

  for (i_0 = 0; i_0 < 3; i_0++) {
    for (r = 0; r < 3; r++) {
      tmp[i_0 + 3 * r] = 0.0;
      tmp[i_0 + 3 * r] += A[i_0] * LATCtrl_B.K_i[r];
      tmp[i_0 + 3 * r] += A[i_0 + 3] * LATCtrl_B.K_i[r + 3];
      tmp[i_0 + 3 * r] += A[i_0 + 6] * LATCtrl_B.K_i[r + 6];
    }
  }

  for (i_0 = 0; i_0 < 3; i_0++) {
    LATCtrl_B.Pk2[3 * i_0] = Y[3 * i_0] + tmp[3 * i_0];
    LATCtrl_B.Pk2[1 + 3 * i_0] = Y[3 * i_0 + 1] + tmp[3 * i_0 + 1];
    LATCtrl_B.Pk2[2 + 3 * i_0] = Y[3 * i_0 + 2] + tmp[3 * i_0 + 2];
  }

  /* End of MATLAB Function: '<S30>/LQR_solver' */

  /* DataTypeConversion: '<S30>/Data Type Conversion' */
  LATC_facC0Hat = LATCtrl_B.xhat[0];

  /* Switch: '<S29>/Switch' incorporates:
   *  Constant: '<S29>/LATC_swtKalFlt_C'
   */
  if (LATC_swtKalFlt_C != 0) {
    LATC_facC0Filt = LATC_facC0Hat;
  } else {
    LATC_facC0Filt = LATCtrl_B.Product3_b;
  }

  /* End of Switch: '<S29>/Switch' */

  /* DataTypeConversion: '<S30>/Data Type Conversion1' */
  LATC_facC1Hat = LATCtrl_B.xhat[1];

  /* Switch: '<S29>/Switch1' incorporates:
   *  Constant: '<S29>/LATC_swtKalFlt_C'
   */
  if (LATC_swtKalFlt_C != 0) {
    LATC_facC1Filt = LATC_facC1Hat;
  } else {
    LATC_facC1Filt = LATCtrl_B.Product1_o;
  }

  /* End of Switch: '<S29>/Switch1' */

  /* Product: '<S17>/Product4' incorporates:
   *  Constant: '<S17>/LATC_facLaneSign'
   */
  LATCtrl_B.Product4_l = LATC_facC1Filt * LATC_facCoeffSign4Mot_C;

  /* Product: '<S17>/Product5' incorporates:
   *  Constant: '<S17>/LATC_facLaneSign'
   */
  LATCtrl_B.Product5_c = LATC_facC0Filt * LATC_facCoeffSign4Mot_C;

  /* DataStoreRead: '<S84>/Data Store Read' */
  memcpy(&LATCtrl_B.DataStoreRead_d[0], &LATCtrl_DW.LATC_YawRateArry[0], 20U *
         sizeof(real_T));

  /* UnitDelay: '<S78>/D' */
  LATCtrl_B.D_k = LATCtrl_DW.D_DSTATE_l;

  /* DataTypeConversion: '<Root>/Data Type Conversion14' incorporates:
   *  Inport: '<Root>/COEHPS_LKADemdRsp'
   */
  LATCtrl_B.DataTypeConversion14 = COEHPS_LKADemdRsp;

  /* RelationalOperator: '<S23>/R' incorporates:
   *  Constant: '<S23>/LKA_ACTVCTRL'
   */
  LATCtrl_B.R_f = (LATCtrl_B.DataTypeConversion14 == 1.0);

  /* Logic: '<S23>/LA1' incorporates:
   *  Constant: '<S23>/LATC_swtYRRL_C'
   */
  LATCtrl_B.LA1 = (LATCtrl_B.R_f && (LATC_swtYRRL_C != 0));

  /* Switch: '<S78>/Switch14' */
  if (LATCtrl_B.LA1) {
    /* RelationalOperator: '<S78>/R' */
    LATCtrl_B.R_j = (LATCtrl_B.Add3 > LATCtrl_B.D_k);

    /* Switch: '<S78>/Swt' */
    if (LATCtrl_B.R_j) {
      /* Product: '<S78>/P1' incorporates:
       *  Constant: '<S23>/Constant33'
       *  Constant: '<S23>/LATC_YRUpRL_C'
       */
      LATCtrl_B.P1_m = LATC_YRUpRL_C * 0.01;

      /* Sum: '<S78>/A1' */
      LATCtrl_B.A1_h = LATCtrl_B.P1_m + LATCtrl_B.D_k;

      /* MinMax: '<S78>/MinMax2' */
      u_0 = LATCtrl_B.A1_h;
      maxval = LATCtrl_B.Add3;
      if (u_0 <= maxval) {
        maxval = u_0;
      }

      LATCtrl_B.MinMax2_m = maxval;

      /* End of MinMax: '<S78>/MinMax2' */
      LATCtrl_B.Swt = LATCtrl_B.MinMax2_m;
    } else {
      /* RelationalOperator: '<S78>/R1' */
      LATCtrl_B.R1_h = (LATCtrl_B.Add3 < LATCtrl_B.D_k);

      /* Switch: '<S78>/Swt1' */
      if (LATCtrl_B.R1_h) {
        /* Product: '<S78>/P' incorporates:
         *  Constant: '<S23>/Constant33'
         *  Constant: '<S23>/LATC_YRLoRL_C'
         */
        LATCtrl_B.P_d = LATC_YRLoRL_C * 0.01;

        /* Sum: '<S78>/A' */
        LATCtrl_B.A_d = LATCtrl_B.D_k + LATCtrl_B.P_d;

        /* MinMax: '<S78>/MinMax1' */
        u_0 = LATCtrl_B.A_d;
        maxval = LATCtrl_B.Add3;
        if (u_0 >= maxval) {
          maxval = u_0;
        }

        LATCtrl_B.MinMax1_e = maxval;

        /* End of MinMax: '<S78>/MinMax1' */
        LATCtrl_B.Swt1_a = LATCtrl_B.MinMax1_e;
      } else {
        LATCtrl_B.Swt1_a = LATCtrl_B.D_k;
      }

      /* End of Switch: '<S78>/Swt1' */
      LATCtrl_B.Swt = LATCtrl_B.Swt1_a;
    }

    /* End of Switch: '<S78>/Swt' */
    LATCtrl_B.Switch14 = LATCtrl_B.Swt;
  } else {
    LATCtrl_B.Switch14 = LATCtrl_B.Add3;
  }

  /* End of Switch: '<S78>/Switch14' */

  /* Switch: '<S23>/Switch6' incorporates:
   *  Constant: '<S5>/LATC_facLaneSign'
   *  Constant: '<S5>/LATC_facMPUSign_C'
   *  Constant: '<S5>/LATC_swtPathSel'
   */
  if (LATC_swtPathSel_C != 0) {
    LATCtrl_B.Switch6 = (real_T)LATC_facMPUSign_C;
  } else {
    LATCtrl_B.Switch6 = (real_T)LATC_facLaneSign_C;
  }

  /* End of Switch: '<S23>/Switch6' */

  /* Product: '<S23>/Product12' */
  LATCtrl_B.Product12 = LatCv4LQR * LATCtrl_B.Switch6;

  /* Product: '<S23>/Product7' incorporates:
   *  Constant: '<S23>/Constant6'
   */
  LATCtrl_B.Product7_hr = LATCtrl_B.Product12 * LATC_Sign4LQR_Cv_C;

  /* Gain: '<S23>/Gain1' */
  LATCtrl_B.Gain1 = 0.27777777777777779 * LATCtrl_B.MinMax;

  /* Product: '<S23>/Product6' */
  LKA_wVehRPS_Ref = LATCtrl_B.Product7_hr * LATCtrl_B.Gain1;

  /* Sum: '<S23>/Subtract' */
  LATC_wYRErr4LQR = LATCtrl_B.Switch14 - LKA_wVehRPS_Ref;

  /* Switch: '<S84>/Switch1' incorporates:
   *  Constant: '<S84>/C5'
   */
  if (LATCtrl_B.R_f) {
    LATCtrl_B.Switch1_d = LATC_wYRErr4LQR;
  } else {
    LATCtrl_B.Switch1_d = 0.0;
  }

  /* End of Switch: '<S84>/Switch1' */

  /* UnitDelay: '<S84>/Unit Delay' */
  LATCtrl_B.UnitDelay_p = LATCtrl_DW.UnitDelay_DSTATE_p;

  /* Switch: '<S84>/Switch13' incorporates:
   *  Constant: '<S84>/C2'
   */
  LATCtrl_B.Switch13 = 1.0;

  /* Sum: '<S84>/Sum6' */
  LATCtrl_B.Sum6_h = LATCtrl_B.UnitDelay_p + LATCtrl_B.Switch13;

  /* Math: '<S84>/Math Function' incorporates:
   *  Constant: '<S73>/LATC_numYRErrCnt_C'
   */
  LATCtrl_B.MathFunction_iq = rt_modd(LATCtrl_B.Sum6_h, LATC_numYRErrCnt_C);

  /* DataTypeConversion: '<S84>/Data Type Conversion6' */
  tmp_5 = fmod(floor(LATCtrl_B.MathFunction_iq), 256.0);
  LATCtrl_B.DataTypeConversion6_j = (uint8_T)(tmp_5 < 0.0 ? (int32_T)(uint8_T)
    -(int8_T)(uint8_T)-tmp_5 : (int32_T)(uint8_T)tmp_5);

  /* Assignment: '<S84>/Assignment' */
  memcpy(&LATCtrl_B.Assignment_f[0], &LATCtrl_B.DataStoreRead_d[0], 20U * sizeof
         (real_T));
  LATCtrl_B.Assignment_f[LATCtrl_B.DataTypeConversion6_j] = LATCtrl_B.Switch1_d;

  /* Product: '<S84>/Product' */
  tmp_5 = 0.0;
  for (i_0 = 0; i_0 < 20; i_0++) {
    /* DataStoreWrite: '<S84>/Data Store Write' */
    LATCtrl_DW.LATC_YawRateArry[i_0] = LATCtrl_B.Assignment_f[i_0];

    /* Math: '<S84>/Math Function1' */
    LATCtrl_B.MathFunction1_b[i_0] = LATCtrl_B.Assignment_f[i_0];
    u_0 = LATCtrl_B.MathFunction1_b[i_0];
    v_1 = LATCtrl_B.Assignment_f[i_0];
    tmp_5 += u_0 * v_1;
  }

  LATCtrl_B.Product_c = tmp_5;

  /* End of Product: '<S84>/Product' */

  /* RelationalOperator: '<S84>/R' incorporates:
   *  Constant: '<S84>/LATC_wSKOnThd_C'
   */
  LATCtrl_B.R_h = (LATCtrl_B.Product_c > 0.008);

  /* RelationalOperator: '<S84>/R2' incorporates:
   *  Constant: '<S84>/C3'
   */
  LATCtrl_B.R2 = (LATCtrl_B.R_f == 1.0);

  /* Logic: '<S84>/A' */
  LATCtrl_B.A_g1 = (LATCtrl_B.R_h && LATCtrl_B.R2);

  /* RelationalOperator: '<S84>/R1' incorporates:
   *  Constant: '<S84>/LATC_wSKOffThd_C'
   */
  LATCtrl_B.R1 = (LATCtrl_B.Product_c < 0.004);

  /* RelationalOperator: '<S84>/R3' incorporates:
   *  Constant: '<S84>/C4'
   */
  LATCtrl_B.R3 = (LATCtrl_B.R_f == 0.0);

  /* Logic: '<S84>/A1' */
  LATCtrl_B.A1_b = (LATCtrl_B.R1 || LATCtrl_B.R3);

  /* Memory: '<S85>/Memory' */
  LATCtrl_B.Memory_j = LATCtrl_DW.Memory_PreviousInput_d;

  /* CombinatorialLogic: '<S85>/Logic' */
  unnamed_idx = LATCtrl_B.A_g1;
  i_0 = unnamed_idx;
  unnamed_idx = LATCtrl_B.A1_b;
  i_0 = (int32_T)(((uint32_T)i_0 << 1) + unnamed_idx);
  unnamed_idx = LATCtrl_B.Memory_j;
  i_0 = (int32_T)(((uint32_T)i_0 << 1) + unnamed_idx);
  LATCtrl_B.Logic_a[0U] = LATCtrl_ConstP.pooled47[(uint32_T)i_0];
  LATCtrl_B.Logic_a[1U] = LATCtrl_ConstP.pooled47[i_0 + 8U];

  /* Abs: '<S73>/Abs' */
  LATCtrl_B.Abs_f = fabs(LATCtrl_B.Product_c);

  /* Lookup_n-D: '<S73>/1-D Lookup Table3' */
  LATCtrl_B.DLookupTable3 = look1_binlxpw(LATCtrl_B.Abs_f, (const real_T*)
    LATCtrl_ConstP.pooled13, (const real_T*)LATCtrl_ConstP.DLookupTable3_tableDa,
    4U);

  /* UnitDelay: '<S77>/D' */
  LATCtrl_B.D_k5 = LATCtrl_DW.D_DSTATE_k;

  /* Logic: '<S23>/LA' incorporates:
   *  Constant: '<S23>/LATC_swtHeadRL_C'
   */
  LATCtrl_B.LA = ((LATC_swtHeadRL_C != 0) && LATCtrl_B.R_f);

  /* Switch: '<S77>/Switch14' */
  if (LATCtrl_B.LA) {
    /* RelationalOperator: '<S77>/R' */
    LATCtrl_B.R_i = (LATCtrl_B.Product4_l > LATCtrl_B.D_k5);

    /* Switch: '<S77>/Swt' */
    if (LATCtrl_B.R_i) {
      /* Product: '<S77>/P1' incorporates:
       *  Constant: '<S23>/Constant33'
       *  Constant: '<S23>/LATC_HeadUpRL_C'
       */
      LATCtrl_B.P1_k = LATC_HeadUpRL_C * 0.01;

      /* Sum: '<S77>/A1' */
      LATCtrl_B.A1_c = LATCtrl_B.P1_k + LATCtrl_B.D_k5;

      /* MinMax: '<S77>/MinMax2' */
      u_0 = LATCtrl_B.A1_c;
      maxval = LATCtrl_B.Product4_l;
      if (u_0 <= maxval) {
        maxval = u_0;
      }

      LATCtrl_B.MinMax2_i = maxval;

      /* End of MinMax: '<S77>/MinMax2' */
      LATCtrl_B.Swt_g = LATCtrl_B.MinMax2_i;
    } else {
      /* RelationalOperator: '<S77>/R1' */
      LATCtrl_B.R1_o = (LATCtrl_B.Product4_l < LATCtrl_B.D_k5);

      /* Switch: '<S77>/Swt1' */
      if (LATCtrl_B.R1_o) {
        /* Product: '<S77>/P' incorporates:
         *  Constant: '<S23>/Constant33'
         *  Constant: '<S23>/LATC_HeadLoRL_C'
         */
        LATCtrl_B.P_j = LATC_HeadLoRL_C * 0.01;

        /* Sum: '<S77>/A' */
        LATCtrl_B.A_dv = LATCtrl_B.D_k5 + LATCtrl_B.P_j;

        /* MinMax: '<S77>/MinMax1' */
        u_0 = LATCtrl_B.A_dv;
        maxval = LATCtrl_B.Product4_l;
        if (u_0 >= maxval) {
          maxval = u_0;
        }

        LATCtrl_B.MinMax1_k = maxval;

        /* End of MinMax: '<S77>/MinMax1' */
        LATCtrl_B.Swt1_j = LATCtrl_B.MinMax1_k;
      } else {
        LATCtrl_B.Swt1_j = LATCtrl_B.D_k5;
      }

      /* End of Switch: '<S77>/Swt1' */
      LATCtrl_B.Swt_g = LATCtrl_B.Swt1_j;
    }

    /* End of Switch: '<S77>/Swt' */
    LATCtrl_B.Switch14_k = LATCtrl_B.Swt_g;
  } else {
    LATCtrl_B.Switch14_k = LATCtrl_B.Product4_l;
  }

  /* End of Switch: '<S77>/Switch14' */

  /* Product: '<S23>/Product11' */
  LATCtrl_B.Product11 = LATCtrl_B.Switch14_k * LATCtrl_B.Switch6;

  /* Sum: '<S23>/Subtract2' incorporates:
   *  Constant: '<S23>/LATC_lLQRTgtDist1'
   */
  LATCtrl_B.Subtract2 = 0.0 - LATCtrl_B.Product11;

  /* Abs: '<S80>/Abs' */
  LATCtrl_B.Abs_p = fabs(LATCtrl_B.Subtract2);

  /* RelationalOperator: '<S80>/RO1' incorporates:
   *  Constant: '<S23>/Constant30'
   */
  LATCtrl_B.RO1_e = (LATCtrl_B.Abs_p > LATC_phiAgRelaxMax_C);

  /* Product: '<S80>/Product1' incorporates:
   *  Constant: '<S23>/Constant31'
   *  Constant: '<S23>/Constant32'
   */
  LATCtrl_B.Product1_b = LATC_facAgRelaxK_C * LATC_phiAgRelaxMin_C;

  /* Sum: '<S80>/Subtract' incorporates:
   *  Constant: '<S23>/Constant30'
   */
  LATCtrl_B.Subtract = LATC_phiAgRelaxMax_C - LATCtrl_B.Product1_b;

  /* Sum: '<S80>/Subtract1' incorporates:
   *  Constant: '<S23>/Constant30'
   *  Constant: '<S23>/Constant32'
   */
  LATCtrl_B.Subtract1 = LATC_phiAgRelaxMax_C - LATC_phiAgRelaxMin_C;

  /* Product: '<S80>/Divide' */
  LATCtrl_B.Divide = LATCtrl_B.Subtract / LATCtrl_B.Subtract1;

  /* Switch: '<S80>/Switch1' */
  if (LATCtrl_B.RO1_e) {
    LATCtrl_B.Switch1_f3 = LATCtrl_B.Subtract2;
  } else {
    /* Abs: '<S80>/Abs1' */
    LATCtrl_B.Abs1_g = fabs(LATCtrl_B.Subtract2);

    /* RelationalOperator: '<S80>/RO2' incorporates:
     *  Constant: '<S23>/Constant32'
     */
    LATCtrl_B.RO2_l = (LATCtrl_B.Abs1_g < LATC_phiAgRelaxMin_C);

    /* Switch: '<S80>/Switch2' */
    if (LATCtrl_B.RO2_l) {
      /* Product: '<S80>/Product2' incorporates:
       *  Constant: '<S23>/Constant31'
       */
      LATCtrl_B.Product2_oi = LATCtrl_B.Subtract2 * LATC_facAgRelaxK_C;
      LATCtrl_B.Switch2_g = LATCtrl_B.Product2_oi;
    } else {
      /* RelationalOperator: '<S80>/RO3' incorporates:
       *  Constant: '<S23>/Constant32'
       */
      LATCtrl_B.RO3_j = (LATCtrl_B.Subtract2 > LATC_phiAgRelaxMin_C);

      /* Switch: '<S80>/Switch3' */
      if (LATCtrl_B.RO3_j) {
        /* Sum: '<S80>/Subtract2' incorporates:
         *  Constant: '<S23>/Constant32'
         */
        LATCtrl_B.Subtract2_gu = LATCtrl_B.Subtract2 - LATC_phiAgRelaxMin_C;

        /* Product: '<S80>/Product4' */
        LATCtrl_B.Product4_hy = LATCtrl_B.Divide * LATCtrl_B.Subtract2_gu;

        /* Product: '<S80>/Product3' incorporates:
         *  Constant: '<S23>/Constant31'
         *  Constant: '<S23>/Constant32'
         */
        LATCtrl_B.Product3_k = LATC_facAgRelaxK_C * LATC_phiAgRelaxMin_C;

        /* Sum: '<S80>/Add1' */
        LATCtrl_B.Add1_c4 = LATCtrl_B.Product3_k + LATCtrl_B.Product4_hy;
        LATCtrl_B.Switch3_fw = LATCtrl_B.Add1_c4;
      } else {
        /* Sum: '<S80>/Add3' incorporates:
         *  Constant: '<S23>/Constant30'
         */
        LATCtrl_B.Add3_i = LATCtrl_B.Subtract2 + LATC_phiAgRelaxMax_C;

        /* Product: '<S80>/Product6' */
        LATCtrl_B.Product6_ht = LATCtrl_B.Divide * LATCtrl_B.Add3_i;

        /* Product: '<S80>/Product5' incorporates:
         *  Constant: '<S23>/Constant30'
         *  Constant: '<S80>/Fac_Sign'
         */
        LATCtrl_B.Product5_j = LATC_phiAgRelaxMax_C * (-1.0);

        /* Sum: '<S80>/Add2' */
        LATCtrl_B.Add2_g = LATCtrl_B.Product5_j + LATCtrl_B.Product6_ht;
        LATCtrl_B.Switch3_fw = LATCtrl_B.Add2_g;
      }

      /* End of Switch: '<S80>/Switch3' */
      LATCtrl_B.Switch2_g = LATCtrl_B.Switch3_fw;
    }

    /* End of Switch: '<S80>/Switch2' */
    LATCtrl_B.Switch1_f3 = LATCtrl_B.Switch2_g;
  }

  /* End of Switch: '<S80>/Switch1' */

  /* Delay: '<S72>/Delay1' */
  LATCtrl_B.Delay1 = LATCtrl_DW.Delay1_DSTATE;

  /* Delay: '<S72>/Delay2' */
  LATCtrl_B.Delay2 = LATCtrl_DW.Delay2_DSTATE[0];

  /* Delay: '<S72>/Delay3' */
  LATCtrl_B.Delay3 = LATCtrl_DW.Delay3_DSTATE[0];

  /* Delay: '<S72>/Delay4' */
  LATCtrl_B.Delay4 = LATCtrl_DW.Delay4_DSTATE[0];

  /* Delay: '<S72>/Delay5' */
  LATCtrl_B.Delay5 = LATCtrl_DW.Delay5_DSTATE[0];

  /* Delay: '<S72>/Delay6' */
  LATCtrl_B.Delay6 = LATCtrl_DW.Delay6_DSTATE[0];

  /* Delay: '<S72>/Delay7' */
  LATCtrl_B.Delay7 = LATCtrl_DW.Delay7_DSTATE[0];

  /* Delay: '<S72>/Delay8' */
  LATCtrl_B.Delay8 = LATCtrl_DW.Delay8_DSTATE[0];

  /* Delay: '<S72>/Delay9' */
  LATCtrl_B.Delay9 = LATCtrl_DW.Delay9_DSTATE[0];

  /* Sum: '<S72>/Sum' */
  LATCtrl_B.Sum = ((((((((LATCtrl_B.Switch1_f3 + LATCtrl_B.Delay1) +
    LATCtrl_B.Delay2) + LATCtrl_B.Delay3) + LATCtrl_B.Delay4) + LATCtrl_B.Delay5)
                     + LATCtrl_B.Delay6) + LATCtrl_B.Delay7) + LATCtrl_B.Delay8)
    + LATCtrl_B.Delay9;

  /* Gain: '<S72>/Gain' */
  LATC_agHeadFlt4LQR = 0.1 * LATCtrl_B.Sum;

  /* Trigonometry: '<S23>/Trigonometric Function' */
  LATCtrl_B.TrigonometricFunction = sin(LATCtrl_B.Switch1_f3);

  /* Product: '<S23>/Product9' */
  LATCtrl_B.Product9 = LATCtrl_B.TrigonometricFunction * LATCtrl_B.Gain1;

  /* Delay: '<S23>/Delay1' incorporates:
   *  Constant: '<S23>/C3'
   */
  if (LATC_tFocastTime4LQR <= 0) {
    LATCtrl_B.Delay1_i = LATCtrl_B.Switch14;
  } else {
    if (LATC_tFocastTime4LQR > 100) {
      i = 100U;
    } else {
      i = LATC_tFocastTime4LQR;
    }

    i = (uint16_T)(100U - i);
    LATCtrl_B.Delay1_i = LATCtrl_DW.Delay1_DSTATE_c[i];
  }

  /* End of Delay: '<S23>/Delay1' */

  /* SignalConversion: '<S68>/ConcatBufferAtMatrix ConcatenateIn1' incorporates:
   *  Constant: '<S68>/Constant1'
   *  Constant: '<S68>/Constant6'
   *  Constant: '<S68>/Constant7'
   *  Constant: '<S68>/Constant8'
   */
  LATCtrl_B.MatrixConcatenate[0] = 0.0;
  LATCtrl_B.MatrixConcatenate[1] = 0.0;
  LATCtrl_B.MatrixConcatenate[2] = 0.0;
  LATCtrl_B.MatrixConcatenate[3] = 0.0;

  /* DataTypeConversion: '<S6>/Data Type Conversion1' incorporates:
   *  Inport: '<Root>/DE_mVehMass'
   */
  LATC_mMass4EBS = DE_mVehMass;

  /* Switch: '<S6>/Switch1' incorporates:
   *  Constant: '<S6>/TR_swt2'
   *  Constant: '<S6>/TR_swt3'
   */
  if (LATC_swtVehMassMan_C != 0) {
    LATC_mVehMass = LATC_mVehMassMan_C;
  } else {
    /* RelationalOperator: '<S6>/Relational Operator5' incorporates:
     *  Constant: '<S6>/PrevTime4LQR'
     */
    LATCtrl_B.RelationalOperator5_p = (LATC_mMass4EBS <= 120000.0);

    /* Switch: '<S6>/Switch2' incorporates:
     *  Inport: '<Root>/DE_stTrlr'
     *  Switch: '<S6>/Switch3'
     */
    if (LATCtrl_B.RelationalOperator5_p) {
      LATCtrl_B.Switch2_im = LATC_mMass4EBS;
    } else {
      if (DE_stTrlr != 0) {
        /* Switch: '<S6>/Switch3' incorporates:
         *  Constant: '<S6>/LngDistOfst1'
         */
        LATCtrl_B.Switch3_n = 35000.0;
      } else {
        /* Switch: '<S6>/Switch3' incorporates:
         *  Constant: '<S6>/LngDistOfst2'
         */
        LATCtrl_B.Switch3_n = 10000.0;
      }

      LATCtrl_B.Switch2_im = LATCtrl_B.Switch3_n;
    }

    /* End of Switch: '<S6>/Switch2' */
    LATC_mVehMass = LATCtrl_B.Switch2_im;
  }

  /* End of Switch: '<S6>/Switch1' */

  /* RelationalOperator: '<S75>/Relational Operator5' incorporates:
   *  Constant: '<S75>/Constant1'
   */
  LATCtrl_B.RelationalOperator5 = (LATC_mVehMass <= LATC_mFullLoadThd_C);

  /* Logic: '<S75>/LOA' incorporates:
   *  Constant: '<S75>/Constant10'
   */
  LATCtrl_B.LOA_m = ((LATC_stVehMassEL_C != 0) && LATCtrl_B.RelationalOperator5);

  /* Switch: '<S75>/Switch1' incorporates:
   *  Constant: '<S75>/Constant2'
   *  Constant: '<S75>/Constant4'
   */
  if (LATCtrl_B.LOA_m) {
    LATCtrl_B.Switch1_h = LATC_caFrTyre4EL_C;
  } else {
    LATCtrl_B.Switch1_h = LATC_caFrTyre4FL_C;
  }

  /* End of Switch: '<S75>/Switch1' */

  /* Switch: '<S75>/Switch2' incorporates:
   *  Constant: '<S75>/Constant3'
   *  Constant: '<S75>/Constant5'
   */
  if (LATCtrl_B.LOA_m) {
    LATCtrl_B.Switch2_i = LATC_caReTyre4EL_C;
  } else {
    LATCtrl_B.Switch2_i = LATC_caReTyre4FL_C;
  }

  /* End of Switch: '<S75>/Switch2' */

  /* Sum: '<S68>/Add1' */
  LATCtrl_B.Add1_m = LATCtrl_B.Switch1_h + LATCtrl_B.Switch2_i;

  /* Switch: '<S75>/Switch3' incorporates:
   *  Constant: '<S75>/Constant32'
   *  Constant: '<S75>/Constant6'
   */
  if (LATCtrl_B.LOA_m) {
    LATCtrl_B.Switch3_h = LATC_mTrctrMass4EL_C;
  } else {
    LATCtrl_B.Switch3_h = LATC_mTrctrMass4FL_C;
  }

  /* End of Switch: '<S75>/Switch3' */

  /* Product: '<S68>/Product1' */
  LATCtrl_B.Product1_cf = LATCtrl_B.Switch3_h * LATCtrl_B.Gain1;

  /* Product: '<S68>/Divide' */
  LATCtrl_B.Divide_g = LATCtrl_B.Add1_m / LATCtrl_B.Product1_cf;

  /* Product: '<S68>/Product9' incorporates:
   *  Constant: '<S68>/Fac_Sign'
   */
  LATCtrl_B.Product9_i = LATCtrl_B.Divide_g * (-1.0);

  /* Switch: '<S75>/Switch5' incorporates:
   *  Constant: '<S75>/Constant30'
   *  Constant: '<S75>/Constant8'
   */
  if (LATCtrl_B.LOA_m) {
    LATCtrl_B.Switch5 = LATC_Lf4EL_C;
  } else {
    LATCtrl_B.Switch5 = LATC_Lf4FL_C;
  }

  /* End of Switch: '<S75>/Switch5' */

  /* Product: '<S68>/Product10' */
  LATCtrl_B.Product10 = LATCtrl_B.Switch1_h * LATCtrl_B.Switch5;

  /* Switch: '<S75>/Switch' incorporates:
   *  Constant: '<S75>/Constant31'
   *  Constant: '<S75>/Constant9'
   */
  if (LATCtrl_B.LOA_m) {
    LATCtrl_B.Switch_ou = LATC_Lr4EL_C;
  } else {
    LATCtrl_B.Switch_ou = LATC_Lr4FL_C;
  }

  /* End of Switch: '<S75>/Switch' */

  /* Product: '<S68>/Product11' */
  LATCtrl_B.Product11_h = LATCtrl_B.Switch2_i * LATCtrl_B.Switch_ou;

  /* Sum: '<S68>/Subtract1' */
  LATCtrl_B.Subtract1_n = LATCtrl_B.Product10 - LATCtrl_B.Product11_h;

  /* Switch: '<S75>/Switch4' incorporates:
   *  Constant: '<S75>/Constant29'
   *  Constant: '<S75>/Constant7'
   */
  if (LATCtrl_B.LOA_m) {
    LATCtrl_B.Switch4 = LATC_Iz4EL_C;
  } else {
    LATCtrl_B.Switch4 = LATC_Iz4FL_C;
  }

  /* End of Switch: '<S75>/Switch4' */

  /* Product: '<S68>/Product12' */
  LATCtrl_B.Product12_l = LATCtrl_B.Switch4 * LATCtrl_B.Gain1;

  /* Product: '<S68>/Divide1' */
  LATCtrl_B.Divide1 = LATCtrl_B.Subtract1_n / LATCtrl_B.Product12_l;

  /* Product: '<S68>/Product13' incorporates:
   *  Constant: '<S68>/Fac_Sign1'
   */
  LATCtrl_B.Product13 = LATCtrl_B.Divide1 * (-1.0);

  /* SignalConversion: '<S68>/ConcatBufferAtMatrix ConcatenateIn2' incorporates:
   *  Constant: '<S68>/Constant10'
   *  Constant: '<S68>/Constant9'
   */
  LATCtrl_B.MatrixConcatenate[4] = 1.0;
  LATCtrl_B.MatrixConcatenate[5] = LATCtrl_B.Product9_i;
  LATCtrl_B.MatrixConcatenate[6] = 0.0;
  LATCtrl_B.MatrixConcatenate[7] = LATCtrl_B.Product13;

  /* SignalConversion: '<S68>/ConcatBufferAtMatrix ConcatenateIn3' incorporates:
   *  Constant: '<S68>/Constant11'
   *  Constant: '<S68>/Constant12'
   *  Constant: '<S68>/Constant3'
   *  Constant: '<S68>/Constant4'
   */
  LATCtrl_B.MatrixConcatenate[8] = 0.0;
  LATCtrl_B.MatrixConcatenate[9] = 0.0;
  LATCtrl_B.MatrixConcatenate[10] = 0.0;
  LATCtrl_B.MatrixConcatenate[11] = 0.0;

  /* Product: '<S68>/Product16' */
  LATCtrl_B.Product16 = LATCtrl_B.Switch1_h * LATCtrl_B.Switch5;

  /* Product: '<S68>/Product17' */
  LATCtrl_B.Product17 = LATCtrl_B.Switch2_i * LATCtrl_B.Switch_ou;

  /* Sum: '<S68>/Subtract3' */
  LATCtrl_B.Subtract3 = LATCtrl_B.Product16 - LATCtrl_B.Product17;

  /* Product: '<S68>/Product18' */
  LATCtrl_B.Product18 = LATCtrl_B.Switch3_h * LATCtrl_B.Gain1;

  /* Product: '<S68>/Divide4' */
  LATCtrl_B.Divide4 = LATCtrl_B.Subtract3 / LATCtrl_B.Product18;

  /* Sum: '<S68>/Add5' */
  LATCtrl_B.Add5 = LATCtrl_B.Gain1 + LATCtrl_B.Divide4;

  /* Product: '<S68>/Product19' incorporates:
   *  Constant: '<S68>/Fac_Sign2'
   */
  LATCtrl_B.Product19 = LATCtrl_B.Add5 * (-1.0);

  /* Product: '<S68>/Product20' */
  LATCtrl_B.Product20 = LATCtrl_B.Switch1_h * LATCtrl_B.Switch5 *
    LATCtrl_B.Switch5;

  /* Product: '<S68>/Product21' */
  LATCtrl_B.Product21 = LATCtrl_B.Switch2_i * LATCtrl_B.Switch_ou *
    LATCtrl_B.Switch_ou;

  /* Sum: '<S68>/Add3' */
  LATCtrl_B.Add3_p = LATCtrl_B.Product20 + LATCtrl_B.Product21;

  /* Product: '<S68>/Product22' */
  LATCtrl_B.Product22 = LATCtrl_B.Switch4 * LATCtrl_B.Gain1;

  /* Product: '<S68>/Divide5' */
  LATCtrl_B.Divide5 = LATCtrl_B.Add3_p / LATCtrl_B.Product22;

  /* Product: '<S68>/Product23' incorporates:
   *  Constant: '<S68>/Fac_Sign3'
   */
  LATCtrl_B.Product23 = LATCtrl_B.Divide5 * (-1.0);

  /* SignalConversion: '<S68>/ConcatBufferAtMatrix ConcatenateIn4' incorporates:
   *  Constant: '<S68>/Constant13'
   *  Constant: '<S68>/Constant14'
   */
  LATCtrl_B.MatrixConcatenate[12] = 0.0;
  LATCtrl_B.MatrixConcatenate[13] = LATCtrl_B.Product19;
  LATCtrl_B.MatrixConcatenate[14] = 1.0;
  LATCtrl_B.MatrixConcatenate[15] = LATCtrl_B.Product23;
  for (i_0 = 0; i_0 < 16; i_0++) {
    /* Product: '<S68>/Product2' incorporates:
     *  Constant: '<S23>/Constant42'
     */
    LATCtrl_B.Product2_i[i_0] = LATCtrl_B.MatrixConcatenate[i_0] * 0.01;

    /* Sum: '<S68>/Add4' incorporates:
     *  Constant: '<S68>/Constant2'
     */
    LATCtrl_B.Add4[i_0] = LATCtrl_B.Product2_i[i_0] +
      LATCtrl_ConstP.pooled24[i_0];
  }

  /* SignalConversion: '<S68>/ConcatBufferAtMatrix Concatenate2In1' incorporates:
   *  Constant: '<S68>/Constant15'
   */
  LATCtrl_B.MatrixConcatenate2[0] = 0.0;

  /* Product: '<S68>/Divide6' */
  LATCtrl_B.MatrixConcatenate2[1] = LATCtrl_B.Switch1_h / LATCtrl_B.Switch3_h;

  /* SignalConversion: '<S68>/ConcatBufferAtMatrix Concatenate2In3' incorporates:
   *  Constant: '<S68>/Constant16'
   */
  LATCtrl_B.MatrixConcatenate2[2] = 0.0;

  /* Product: '<S68>/Product24' */
  LATCtrl_B.Product24 = LATCtrl_B.Switch1_h * LATCtrl_B.Switch5;

  /* Product: '<S68>/Divide7' */
  LATCtrl_B.MatrixConcatenate2[3] = LATCtrl_B.Product24 / LATCtrl_B.Switch4;

  /* Product: '<S68>/Product26' incorporates:
   *  Constant: '<S23>/Constant42'
   */
  LATCtrl_B.Product26[0] = LATCtrl_B.MatrixConcatenate2[0] * 0.01;
  LATCtrl_B.Product26[1] = LATCtrl_B.MatrixConcatenate2[1] * 0.01;
  LATCtrl_B.Product26[2] = LATCtrl_B.MatrixConcatenate2[2] * 0.01;
  LATCtrl_B.Product26[3] = LATCtrl_B.MatrixConcatenate2[3] * 0.01;

  /* Math: '<S68>/Math Function' */
  LATCtrl_B.MathFunction_m[0] = LATCtrl_B.Product26[0];
  LATCtrl_B.MathFunction_m[1] = LATCtrl_B.Product26[1];
  LATCtrl_B.MathFunction_m[2] = LATCtrl_B.Product26[2];
  LATCtrl_B.MathFunction_m[3] = LATCtrl_B.Product26[3];

  /* DataStoreRead: '<S74>/Data Store Read' */
  memcpy(&LATCtrl_B.DataStoreRead_o[0], &LATCtrl_DW.LATC_YawRateStore_i[0], 10U *
         sizeof(real_T));

  /* DataTypeConversion: '<Root>/Data Type Conversion8' incorporates:
   *  Inport: '<Root>/DE_phiSteerAngle'
   */
  LATC_phiSteerAngle = DE_phiSteerAngle;

  /* Switch: '<S53>/Switch3' incorporates:
   *  Constant: '<S53>/LKA_ManStYawRateSt'
   *  Constant: '<S53>/LKA_ManStYawRate_C'
   */
  if (FALSE) {
    LATCtrl_B.Switch3_e = 0.0;
  } else {
    LATCtrl_B.Switch3_e = LATC_phiSteerAngle;
  }

  /* End of Switch: '<S53>/Switch3' */

  /* RelationalOperator: '<S88>/Relational Operator1' incorporates:
   *  Constant: '<S88>/Constant1'
   */
  LATCtrl_B.RelationalOperator1 = (LATC_vVehSpdKph <= 130.0);

  /* RelationalOperator: '<S88>/Relational Operator2' incorporates:
   *  Constant: '<S88>/Constant4'
   */
  LATCtrl_B.RelationalOperator2 = (LATC_vVehSpdKph >= 20.0);

  /* Logic: '<S88>/Logical Operator' */
  LATC_stSteerAvrgEnb = (LATCtrl_B.RelationalOperator1 &&
    LATCtrl_B.RelationalOperator2);

  /* Sum: '<S5>/Add5' */
  LATCtrl_B.Add5_m = LATC_facLeftLaneA0 + LATC_facRgtLaneA0;

  /* Product: '<S5>/Divide4' incorporates:
   *  Constant: '<S5>/FAC_6'
   */
  LATCtrl_B.Divide4_e = LATCtrl_B.Add5_m / 2.0;

  /* Product: '<S5>/Product3' incorporates:
   *  Constant: '<S5>/LATC_facLaneSign'
   */
  LATC_lDist4Cmr = LATCtrl_B.Divide4_e * (real_T)LATC_facLaneSign_C;

  /* Sum: '<S5>/Add7' */
  LATCtrl_B.Add7 = LATC_facLeftLaneA2 + LATC_facRgtLaneA2;

  /* Product: '<S5>/Product1' incorporates:
   *  Constant: '<S5>/LATC_facLaneSign'
   */
  LATC_cCv4Cmr = LATCtrl_B.Add7 * (real_T)LATC_facLaneSign_C;

  /* Switch: '<S87>/Switch1' incorporates:
   *  Constant: '<S87>/Constant4'
   */
  if (LATC_swtAgZeroSel_C != 0) {
    /* Abs: '<S87>/Abs1' */
    LATCtrl_B.Abs1_b = fabs(LATC_cCv4Cmr);

    /* RelationalOperator: '<S87>/Relational Operator6' incorporates:
     *  Constant: '<S87>/Constant8'
     */
    LATCtrl_B.RelationalOperator6 = (LATCtrl_B.Abs1_b <= 0.0002);

    /* Abs: '<S87>/Abs' */
    LATCtrl_B.Abs_pt = fabs(LATC_lDist4Cmr);

    /* RelationalOperator: '<S87>/Relational Operator5' incorporates:
     *  Constant: '<S87>/Constant7'
     */
    LATCtrl_B.RelationalOperator5_j = (LATCtrl_B.Abs_pt <= 0.3);

    /* Logic: '<S87>/Logical Operator1' */
    LATCtrl_B.LogicalOperator1_m = (LATCtrl_B.RelationalOperator5_j &&
      LATCtrl_B.RelationalOperator6);

    /* Abs: '<S87>/Abs4' */
    LATCtrl_B.Abs4_k = fabs(LATC_facRgtLaneA0);

    /* RelationalOperator: '<S87>/Relational Operator8' incorporates:
     *  Constant: '<S87>/Constant6'
     */
    LATCtrl_B.RelationalOperator8 = (LATCtrl_B.Abs4_k >= 1.0);

    /* Abs: '<S87>/Abs3' */
    LATCtrl_B.Abs3 = fabs(LATC_facLeftLaneA0);

    /* RelationalOperator: '<S87>/Relational Operator7' incorporates:
     *  Constant: '<S87>/Constant6'
     */
    LATCtrl_B.RelationalOperator7 = (LATCtrl_B.Abs3 >= 1.0);

    /* Logic: '<S87>/Logical Operator4' */
    LATCtrl_B.LogicalOperator4 = (LATCtrl_B.RelationalOperator7 &&
      LATCtrl_B.RelationalOperator8);

    /* Logic: '<S87>/Logical Operator' */
    LATCtrl_B.LogicalOperator_a = (LATCtrl_B.LogicalOperator4 &&
      LATCtrl_B.LogicalOperator1_m && LATCtrl_ConstB.LogicalOperator3);
    LATCtrl_B.Switch1_hs = LATCtrl_B.LogicalOperator_a;
  } else {
    /* Abs: '<S87>/Abs2' */
    LATCtrl_B.Abs2_o = fabs(LATCtrl_B.Add3);

    /* RelationalOperator: '<S87>/Relational Operator3' incorporates:
     *  Constant: '<S87>/Constant3'
     */
    LATCtrl_B.RelationalOperator3 = (LATCtrl_B.Abs2_o <= LATC_wYR4AgZero_C);
    LATCtrl_B.Switch1_hs = LATCtrl_B.RelationalOperator3;
  }

  /* End of Switch: '<S87>/Switch1' */

  /* RelationalOperator: '<S87>/Relational Operator4' incorporates:
   *  Constant: '<S87>/Constant5'
   */
  LATCtrl_B.RelationalOperator4 = (LATC_stOperMod != ((uint8_T)3U));

  /* Logic: '<S87>/Logical Operator2' */
  LATC_stSteerAvrgActv = (LATCtrl_B.Switch1_hs && LATCtrl_B.RelationalOperator4);

  /* Chart: '<S24>/Chart' incorporates:
   *  Constant: '<S24>/Constant3'
   */
  /* Gateway: LKA_SteerCtrl/LKA_SteerSelfLearn/Chart */
  /* During: LKA_SteerCtrl/LKA_SteerSelfLearn/Chart */
  if (LATCtrl_DW.is_active_c3_LATCtrl == 0U) {
    /* Entry: LKA_SteerCtrl/LKA_SteerSelfLearn/Chart */
    LATCtrl_DW.is_active_c3_LATCtrl = 1U;

    /* Entry Internal: LKA_SteerCtrl/LKA_SteerSelfLearn/Chart */
    /* Transition: '<S90>:30' */
    LATCtrl_DW.I_avrg = 0.0;
    LATCtrl_DW.Avrg2Store = 0.0;
    LATCtrl_B.I_Cnt = 0.0;
    LATCtrl_B.Avrg = 0.0;
    LATCtrl_B.I_Cnt_Frz = 0.0;
    LATCtrl_B.I_avrg_Frz = 0.0;
    LATCtrl_DW.is_c3_LATCtrl = LATCtrl_IN_Avrg;

    /* Entry Internal 'Avrg': '<S90>:29' */
    LATCtrl_inner_default_Avrg();
  } else if (LATCtrl_DW.is_c3_LATCtrl == LATCtrl_IN_Avrg) {
    /* During 'Avrg': '<S90>:29' */
    if ((LATCtrl_B.I_Cnt < 15.0) && (LATC_stSteerAvrgActv == 0) &&
        (LATC_stSteerAvrgEnb == 1)) {
      /* Transition: '<S90>:93' */
      LATCtrl_B.I_Cnt_Frz = LATCtrl_B.I_Cnt;
      LATCtrl_B.I_avrg_Frz = LATCtrl_DW.I_avrg;
      LATCtrl_B.Avrg_Frz = LATCtrl_B.Avrg;
      LATCtrl_DW.is_c3_LATCtrl = LATCtrl_IN_Frozen;

      /* Entry Internal 'Frozen': '<S90>:87' */
      /* Transition: '<S90>:89' */
      /* Transition: '<S90>:92' */
    } else {
      LATCtrl_inner_default_Avrg();
    }
  } else {
    /* During 'Frozen': '<S90>:87' */
    if ((LATC_stSteerAvrgActv == 1) || (LATC_stSteerAvrgEnb == 0)) {
      /* Transition: '<S90>:94' */
      LATCtrl_B.I_Cnt_Frz = 0.0;
      LATCtrl_B.I_avrg_Frz = 0.0;
      LATCtrl_B.Avrg_Frz = 0.0;
      LATCtrl_DW.is_c3_LATCtrl = LATCtrl_IN_Avrg;

      /* Entry Internal 'Avrg': '<S90>:29' */
      LATCtrl_inner_default_Avrg();
    } else {
      /* Transition: '<S90>:90' */
      /* Transition: '<S90>:92' */
    }
  }

  /* End of Chart: '<S24>/Chart' */

  /* RelationalOperator: '<S89>/Relational Operator1' incorporates:
   *  Constant: '<S24>/Constant3'
   */
  LATC_stSteerAvrgComplt = (LATCtrl_B.I_Cnt == 15.0);

  /* UnitDelay: '<S91>/Unit Delay' */
  LATCtrl_B.UnitDelay_od = LATCtrl_DW.UnitDelay_DSTATE_cs;

  /* Logic: '<S91>/Logical Operator3' */
  LATCtrl_B.LogicalOperator3 = !LATCtrl_B.UnitDelay_od;

  /* Logic: '<S91>/Logical Operator2' */
  LATCtrl_B.LogicalOperator2 = (LATC_stSteerAvrgComplt &&
    LATCtrl_B.LogicalOperator3);

  /* UnitDelay: '<S89>/Unit Delay' */
  LATCtrl_B.UnitDelay_l = LATCtrl_DW.UnitDelay_DSTATE_m;

  /* Switch: '<S89>/Switch1' */
  if (LATCtrl_B.LogicalOperator2) {
    LATC_agSteerZeroOfst = LATCtrl_B.Avrg;
  } else {
    LATC_agSteerZeroOfst = LATCtrl_B.UnitDelay_l;
  }

  /* End of Switch: '<S89>/Switch1' */

  /* Switch: '<S53>/Switch2' incorporates:
   *  Constant: '<S19>/Constant'
   *  Constant: '<S53>/LKA_swtAgSteerOfstSel_C'
   *  Constant: '<S53>/SA_Ofst_C'
   *  Switch: '<S53>/Switch1'
   */
  if (LKA_swtAgSteerOfstSel_C != 0) {
    LATCtrl_B.Switch2_p = LATC_agSteerOfst_C;
  } else {
    if (0.0 != 0.0) {
      /* Switch: '<S53>/Switch1' incorporates:
       *  Constant: '<S19>/Constant1'
       */
      LATCtrl_B.Switch1_g = 0.0;
    } else {
      /* Switch: '<S53>/Switch1' */
      LATCtrl_B.Switch1_g = LATC_agSteerZeroOfst;
    }

    LATCtrl_B.Switch2_p = LATCtrl_B.Switch1_g;
  }

  /* End of Switch: '<S53>/Switch2' */

  /* Sum: '<S53>/Add3' */
  LATCtrl_B.Add3_n = LATCtrl_B.Switch3_e - LATCtrl_B.Switch2_p;

  /* Product: '<S53>/Divide2' incorporates:
   *  Constant: '<S53>/RAD2DEG'
   */
  LATCtrl_B.Divide2_e = LATCtrl_B.Add3_n / 57.3;

  /* Product: '<S53>/Divide1' incorporates:
   *  Constant: '<S53>/RAD2DEG1'
   */
  LATCtrl_B.Divide1_i = LATCtrl_B.Divide2_e / 19.3;

  /* UnitDelay: '<S74>/Unit Delay' */
  LATCtrl_B.UnitDelay_o = LATCtrl_DW.UnitDelay_DSTATE_c;

  /* Sum: '<S74>/Sum6' incorporates:
   *  Constant: '<S74>/C2'
   */
  LATCtrl_B.Sum6_j = (uint16_T)(LATCtrl_B.UnitDelay_o + (uint32_T)((uint8_T)1U));

  /* Math: '<S74>/Math Function' incorporates:
   *  Constant: '<S74>/C3'
   */
  i = LATCtrl_B.Sum6_j;
  n = LATC_tFocastTime4LQR;
  if (n == 0) {
    LATCtrl_B.MathFunction_mz = i;
  } else {
    LATCtrl_B.MathFunction_mz = (uint16_T)(i % n);
  }

  /* End of Math: '<S74>/Math Function' */

  /* DataTypeConversion: '<S74>/Data Type Conversion6' */
  LATCtrl_B.DataTypeConversion6_n = (uint8_T)LATCtrl_B.MathFunction_mz;

  /* Assignment: '<S74>/Assignment' */
  memcpy(&LATCtrl_B.Assignment_h[0], &LATCtrl_B.DataStoreRead_o[0], 10U * sizeof
         (real_T));
  LATCtrl_B.Assignment_h[LATCtrl_B.DataTypeConversion6_n] = LATCtrl_B.Divide1_i;

  /* Switch: '<S74>/Switch1' incorporates:
   *  Constant: '<S5>/LATC_swtVarDelyCycle_C'
   *  Constant: '<S74>/C3'
   */
  if (LATC_swtVarDelyCycle_C != 0) {
    LATCtrl_B.Switch1_lu = LATC_tDelyLenCycle;
  } else {
    LATCtrl_B.Switch1_lu = LATC_tFocastTime4LQR;
  }

  /* End of Switch: '<S74>/Switch1' */

  /* SignalConversion: '<S86>/TmpSignal ConversionAt SFunction Inport1' incorporates:
   *  Constant: '<S23>/Constant34'
   *  Constant: '<S23>/Constant35'
   *  MATLAB Function: '<S74>/Predict'
   */
  LATCtrl_B.TmpSignalConversionAtSFunctionI[0] = 0.0;
  LATCtrl_B.TmpSignalConversionAtSFunctionI[1] = LATCtrl_B.Product9;
  LATCtrl_B.TmpSignalConversionAtSFunctionI[2] = 0.0;
  LATCtrl_B.TmpSignalConversionAtSFunctionI[3] = LATCtrl_B.Delay1_i;

  /* MATLAB Function: '<S74>/Predict' incorporates:
   *  Constant: '<S74>/C3'
   */
  /* MATLAB Function 'LKA_SteerCtrl/LKA_SteerCalc4LQR/LATC_MotPredict/Predict': '<S86>:1' */
  /* '<S86>:1:3' x=x0; */
  LATCtrl_B.x[0] = LATCtrl_B.TmpSignalConversionAtSFunctionI[0];
  LATCtrl_B.x[1] = LATCtrl_B.TmpSignalConversionAtSFunctionI[1];
  LATCtrl_B.x[2] = LATCtrl_B.TmpSignalConversionAtSFunctionI[2];
  LATCtrl_B.x[3] = LATCtrl_B.TmpSignalConversionAtSFunctionI[3];

  /* '<S86>:1:4' i=uint16(0); */
  /* '<S86>:1:5' while ((i <Dely_Len)) */
  for (i = 0U; i < LATCtrl_B.Switch1_lu; i++) {
    /* '<S86>:1:6' n=mod(index+i+1+(Max_num-Dely_Len),Max_num); */
    tmp_6 = (uint32_T)LATCtrl_B.MathFunction_mz + i;
    if (tmp_6 > 65535U) {
      tmp_6 = 65535U;
    }

    tmp_6++;
    if (tmp_6 > 65535U) {
      tmp_6 = 65535U;
    }

    i_0 = LATC_tFocastTime4LQR;
    qY = (uint32_T)i_0 - LATCtrl_B.Switch1_lu;
    if (qY > (uint32_T)i_0) {
      qY = 0U;
    }

    i_0 = (int32_T)qY;
    tmp_6 += i_0;
    if (tmp_6 > 65535U) {
      tmp_6 = 65535U;
    }

    n = (uint16_T)tmp_6;
    if (LATC_tFocastTime4LQR != 0) {
      i_0 = LATC_tFocastTime4LQR;
      n = (uint16_T)((uint32_T)n - (uint16_T)((uint16_T)((uint32_T)i_0 ==
        (uint32_T)0 ? MAX_uint32_T : (uint32_T)n / i_0) * (uint32_T)
        LATC_tFocastTime4LQR));
    }

    /* '<S86>:1:7' x_p=A*x+B*u(n+1); */
    /* '<S86>:1:8' x=x_p; */
    tmp_6 = n + 1U;
    if (tmp_6 > 65535U) {
      tmp_6 = 65535U;
    }

    tmp_5 = LATCtrl_B.Assignment_h[(int32_T)tmp_6 - 1];
    for (i_0 = 0; i_0 < 4; i_0++) {
      u_1 = LATCtrl_B.Add4[i_0] * LATCtrl_B.x[0];
      u_1 += LATCtrl_B.Add4[i_0 + 4] * LATCtrl_B.x[1];
      u_1 += LATCtrl_B.Add4[i_0 + 8] * LATCtrl_B.x[2];
      u_1 += LATCtrl_B.Add4[i_0 + 12] * LATCtrl_B.x[3];
      tmp_1[i_0] = LATCtrl_B.MathFunction_m[i_0] * tmp_5 + u_1;
    }

    LATCtrl_B.x[0] = tmp_1[0];
    LATCtrl_B.x[1] = tmp_1[1];
    LATCtrl_B.x[2] = tmp_1[2];
    LATCtrl_B.x[3] = tmp_1[3];

    /* '<S86>:1:9' i=i+1; */
  }

  /* Switch: '<S23>/Switch5' incorporates:
   *  Constant: '<S23>/TR_swt'
   *  Constant: '<S23>/TR_swt1'
   */
  if (LATC_swtLQRMan_C != 0) {
    LATCtrl_B.Switch5_d[0] = 0.0;
    LATCtrl_B.Switch5_d[1] = 0.0;
    LATCtrl_B.Switch5_d[2] = 0.0;
    LATCtrl_B.Switch5_d[3] = 0.0;
  } else {
    LATCtrl_B.Switch5_d[0] = LATCtrl_B.x[0];
    LATCtrl_B.Switch5_d[1] = LATCtrl_B.x[1];
    LATCtrl_B.Switch5_d[2] = LATCtrl_B.x[2];
    LATCtrl_B.Switch5_d[3] = LATCtrl_B.x[3];
  }

  /* End of Switch: '<S23>/Switch5' */

  /* Switch: '<S73>/Switch3' incorporates:
   *  Constant: '<S73>/LQR_Ele3Man_C'
   *  Constant: '<S73>/LQR_Ele3Man_swt'
   */
  if (LQR_Ele3Man_swt != 0) {
    LATCtrl_B.Switch3_d = LQR_Ele3Man_C;
  } else {
    /* Sum: '<S73>/Sum2' */
    LATCtrl_B.Sum2 = LATC_agHeadFlt4LQR + LATCtrl_B.Switch5_d[2];
    LATCtrl_B.Switch3_d = LATCtrl_B.Sum2;
  }

  /* End of Switch: '<S73>/Switch3' */

  /* Delay: '<S82>/Delay1' */
  LATCtrl_B.Delay1_n = LATCtrl_DW.Delay1_DSTATE_m;

  /* Delay: '<S82>/Delay2' */
  LATCtrl_B.Delay2_o = LATCtrl_DW.Delay2_DSTATE_g[0];

  /* Delay: '<S82>/Delay3' */
  LATCtrl_B.Delay3_j = LATCtrl_DW.Delay3_DSTATE_f[0];

  /* Delay: '<S82>/Delay4' */
  LATCtrl_B.Delay4_j = LATCtrl_DW.Delay4_DSTATE_n[0];

  /* Delay: '<S82>/Delay5' */
  LATCtrl_B.Delay5_k = LATCtrl_DW.Delay5_DSTATE_c[0];

  /* Delay: '<S82>/Delay6' */
  LATCtrl_B.Delay6_a = LATCtrl_DW.Delay6_DSTATE_k[0];

  /* Delay: '<S82>/Delay7' */
  LATCtrl_B.Delay7_p = LATCtrl_DW.Delay7_DSTATE_l[0];

  /* Delay: '<S82>/Delay8' */
  LATCtrl_B.Delay8_l = LATCtrl_DW.Delay8_DSTATE_m[0];

  /* Delay: '<S82>/Delay9' */
  LATCtrl_B.Delay9_j = LATCtrl_DW.Delay9_DSTATE_h[0];

  /* Sum: '<S82>/Sum' */
  LATCtrl_B.Sum_l = ((((((((LATC_wYRErr4LQR + LATCtrl_B.Delay1_n) +
    LATCtrl_B.Delay2_o) + LATCtrl_B.Delay3_j) + LATCtrl_B.Delay4_j) +
                        LATCtrl_B.Delay5_k) + LATCtrl_B.Delay6_a) +
                      LATCtrl_B.Delay7_p) + LATCtrl_B.Delay8_l) +
    LATCtrl_B.Delay9_j;

  /* Gain: '<S82>/Gain' */
  LATC_wYRErrFlt4LQR = 0.1 * LATCtrl_B.Sum_l;

  /* Switch: '<S73>/Switch4' incorporates:
   *  Constant: '<S73>/LQR_Ele4Man_C'
   *  Constant: '<S73>/LQR_Ele4Man_swt'
   */
  if (LQR_Ele4Man_swt != 0) {
    LATCtrl_B.Switch4_b = LQR_Ele4Man_C;
  } else {
    LATCtrl_B.Switch4_b = LATC_wYRErrFlt4LQR;
  }

  /* End of Switch: '<S73>/Switch4' */

  /* Product: '<S73>/Product13' incorporates:
   *  Constant: '<S73>/LATC_fac4YRErrEmpty_C'
   */
  LATCtrl_B.Product13_e = LATCtrl_B.Switch4_b * LQR_fac4YRErrEmpty_C;

  /* Product: '<S23>/Product10' */
  LATCtrl_B.Product10_c = LATCtrl_B.Product5_c * LATCtrl_B.Switch6;

  /* Sum: '<S23>/Subtract1' incorporates:
   *  Constant: '<S23>/LATC_lLQRTgtDist'
   */
  LATCtrl_B.Subtract1_m = 0.0 - LATCtrl_B.Product10_c;

  /* Abs: '<S79>/Abs' */
  LATCtrl_B.Abs_k = fabs(LATCtrl_B.Subtract1_m);

  /* RelationalOperator: '<S79>/RO1' incorporates:
   *  Constant: '<S23>/Constant2'
   */
  LATCtrl_B.RO1_b = (LATCtrl_B.Abs_k > LATC_yDistRelaxMax_C);

  /* Product: '<S79>/Product1' incorporates:
   *  Constant: '<S23>/Constant29'
   *  Constant: '<S23>/Constant3'
   */
  LATCtrl_B.Product1_n = LATC_facDistRelaxK_C * LATC_yDistRelaxMin_C;

  /* Sum: '<S79>/Subtract' incorporates:
   *  Constant: '<S23>/Constant2'
   */
  LATCtrl_B.Subtract_n = LATC_yDistRelaxMax_C - LATCtrl_B.Product1_n;

  /* Sum: '<S79>/Subtract1' incorporates:
   *  Constant: '<S23>/Constant2'
   *  Constant: '<S23>/Constant3'
   */
  LATCtrl_B.Subtract1_g = LATC_yDistRelaxMax_C - LATC_yDistRelaxMin_C;

  /* Product: '<S79>/Divide' */
  LATCtrl_B.Divide_p = LATCtrl_B.Subtract_n / LATCtrl_B.Subtract1_g;

  /* Switch: '<S79>/Switch1' */
  if (LATCtrl_B.RO1_b) {
    LATCtrl_B.Switch1_dv = LATCtrl_B.Subtract1_m;
  } else {
    /* Abs: '<S79>/Abs1' */
    LATCtrl_B.Abs1_k = fabs(LATCtrl_B.Subtract1_m);

    /* RelationalOperator: '<S79>/RO2' incorporates:
     *  Constant: '<S23>/Constant3'
     */
    LATCtrl_B.RO2_a = (LATCtrl_B.Abs1_k < LATC_yDistRelaxMin_C);

    /* Switch: '<S79>/Switch2' */
    if (LATCtrl_B.RO2_a) {
      /* Product: '<S79>/Product2' incorporates:
       *  Constant: '<S23>/Constant29'
       */
      LATCtrl_B.Product2_ci = LATCtrl_B.Subtract1_m * LATC_facDistRelaxK_C;
      LATCtrl_B.Switch2_pb = LATCtrl_B.Product2_ci;
    } else {
      /* RelationalOperator: '<S79>/RO3' incorporates:
       *  Constant: '<S23>/Constant3'
       */
      LATCtrl_B.RO3_a = (LATCtrl_B.Subtract1_m > LATC_yDistRelaxMin_C);

      /* Switch: '<S79>/Switch3' */
      if (LATCtrl_B.RO3_a) {
        /* Sum: '<S79>/Subtract2' incorporates:
         *  Constant: '<S23>/Constant3'
         */
        LATCtrl_B.Subtract2_f = LATCtrl_B.Subtract1_m - LATC_yDistRelaxMin_C;

        /* Product: '<S79>/Product4' */
        LATCtrl_B.Product4_ah = LATCtrl_B.Divide_p * LATCtrl_B.Subtract2_f;

        /* Product: '<S79>/Product3' incorporates:
         *  Constant: '<S23>/Constant29'
         *  Constant: '<S23>/Constant3'
         */
        LATCtrl_B.Product3_e = LATC_facDistRelaxK_C * LATC_yDistRelaxMin_C;

        /* Sum: '<S79>/Add1' */
        LATCtrl_B.Add1_d = LATCtrl_B.Product3_e + LATCtrl_B.Product4_ah;
        LATCtrl_B.Switch3_np = LATCtrl_B.Add1_d;
      } else {
        /* Sum: '<S79>/Add3' incorporates:
         *  Constant: '<S23>/Constant2'
         */
        LATCtrl_B.Add3_et = LATCtrl_B.Subtract1_m + LATC_yDistRelaxMax_C;

        /* Product: '<S79>/Product6' */
        LATCtrl_B.Product6_i = LATCtrl_B.Divide_p * LATCtrl_B.Add3_et;

        /* Product: '<S79>/Product5' incorporates:
         *  Constant: '<S23>/Constant2'
         *  Constant: '<S79>/Fac_Sign'
         */
        LATCtrl_B.Product5_cj = LATC_yDistRelaxMax_C * (-1.0);

        /* Sum: '<S79>/Add2' */
        LATCtrl_B.Add2_l = LATCtrl_B.Product5_cj + LATCtrl_B.Product6_i;
        LATCtrl_B.Switch3_np = LATCtrl_B.Add2_l;
      }

      /* End of Switch: '<S79>/Switch3' */
      LATCtrl_B.Switch2_pb = LATCtrl_B.Switch3_np;
    }

    /* End of Switch: '<S79>/Switch2' */
    LATCtrl_B.Switch1_dv = LATCtrl_B.Switch2_pb;
  }

  /* End of Switch: '<S79>/Switch1' */

  /* Delay: '<S69>/Delay1' */
  LATCtrl_B.Delay1_m = LATCtrl_DW.Delay1_DSTATE_f;

  /* Delay: '<S69>/Delay2' */
  LATCtrl_B.Delay2_i = LATCtrl_DW.Delay2_DSTATE_b[0];

  /* Delay: '<S69>/Delay3' */
  LATCtrl_B.Delay3_f = LATCtrl_DW.Delay3_DSTATE_d[0];

  /* Delay: '<S69>/Delay4' */
  LATCtrl_B.Delay4_p = LATCtrl_DW.Delay4_DSTATE_f[0];

  /* Delay: '<S69>/Delay5' */
  LATCtrl_B.Delay5_j = LATCtrl_DW.Delay5_DSTATE_cr[0];

  /* Delay: '<S69>/Delay6' */
  LATCtrl_B.Delay6_c = LATCtrl_DW.Delay6_DSTATE_i[0];

  /* Delay: '<S69>/Delay7' */
  LATCtrl_B.Delay7_n = LATCtrl_DW.Delay7_DSTATE_c[0];

  /* Delay: '<S69>/Delay8' */
  LATCtrl_B.Delay8_c = LATCtrl_DW.Delay8_DSTATE_f[0];

  /* Delay: '<S69>/Delay9' */
  LATCtrl_B.Delay9_m = LATCtrl_DW.Delay9_DSTATE_m[0];

  /* Sum: '<S69>/Sum' */
  LATCtrl_B.Sum_b = ((((((((LATCtrl_B.Switch1_dv + LATCtrl_B.Delay1_m) +
    LATCtrl_B.Delay2_i) + LATCtrl_B.Delay3_f) + LATCtrl_B.Delay4_p) +
                        LATCtrl_B.Delay5_j) + LATCtrl_B.Delay6_c) +
                      LATCtrl_B.Delay7_n) + LATCtrl_B.Delay8_c) +
    LATCtrl_B.Delay9_m;

  /* Gain: '<S69>/Gain' */
  LATC_vDistErrFlt4LQR = 0.1 * LATCtrl_B.Sum_b;

  /* Delay: '<S69>/Delay10' */
  LATCtrl_B.Delay10 = LATCtrl_DW.Delay10_DSTATE;

  /* Delay: '<S69>/Delay11' */
  LATCtrl_B.Delay11 = LATCtrl_DW.Delay11_DSTATE[0];

  /* Delay: '<S69>/Delay12' */
  LATCtrl_B.Delay12 = LATCtrl_DW.Delay12_DSTATE[0];

  /* Delay: '<S69>/Delay13' */
  LATCtrl_B.Delay13 = LATCtrl_DW.Delay13_DSTATE[0];

  /* Delay: '<S69>/Delay14' */
  LATCtrl_B.Delay14 = LATCtrl_DW.Delay14_DSTATE[0];

  /* Delay: '<S69>/Delay15' */
  LATCtrl_B.Delay15 = LATCtrl_DW.Delay15_DSTATE[0];

  /* Delay: '<S69>/Delay16' */
  LATCtrl_B.Delay16 = LATCtrl_DW.Delay16_DSTATE[0];

  /* Delay: '<S69>/Delay17' */
  LATCtrl_B.Delay17 = LATCtrl_DW.Delay17_DSTATE[0];

  /* Delay: '<S69>/Delay18' */
  LATCtrl_B.Delay18 = LATCtrl_DW.Delay18_DSTATE[0];

  /* Sum: '<S69>/Sum1' */
  LATCtrl_B.Sum1 = ((((((((LATC_vDistErrFlt4LQR + LATCtrl_B.Delay10) +
    LATCtrl_B.Delay11) + LATCtrl_B.Delay12) + LATCtrl_B.Delay13) +
                       LATCtrl_B.Delay14) + LATCtrl_B.Delay15) +
                     LATCtrl_B.Delay16) + LATCtrl_B.Delay17) + LATCtrl_B.Delay18;

  /* Gain: '<S69>/Gain1' */
  LATCtrl_B.Gain1_g = 0.1 * LATCtrl_B.Sum1;

  /* Saturate: '<S69>/Saturation1' */
  maxval = -DistErrRateLimit;
  u_0 = LATCtrl_B.Gain1_g;
  u_1 = DistErrRateLimit;
  if (u_0 >= u_1) {
    LATCtrl_B.Saturation1_l = u_1;
  } else if (u_0 <= maxval) {
    LATCtrl_B.Saturation1_l = maxval;
  } else {
    LATCtrl_B.Saturation1_l = u_0;
  }

  /* End of Saturate: '<S69>/Saturation1' */

  /* UnitDelay: '<S69>/Unit Delay5' */
  LATCtrl_B.UnitDelay5 = LATCtrl_DW.UnitDelay5_DSTATE;

  /* UnitDelay: '<S69>/Unit Delay6' */
  LATCtrl_B.UnitDelay6 = LATCtrl_DW.UnitDelay6_DSTATE;

  /* MATLAB Function: '<S69>/MATLAB Function' incorporates:
   *  Constant: '<S23>/CT'
   *  Constant: '<S23>/CT2'
   *  Constant: '<S23>/Q'
   */
  /* MATLAB Function 'LKA_SteerCtrl/LKA_SteerCalc4LQR/DistErrfilter/MATLAB Function': '<S83>:1' */
  /* '<S83>:1:3' V_pre = SignalOut+a*T; */
  maxval = LATCtrl_B.Saturation1_l * 0.01 + LATCtrl_B.UnitDelay5;

  /* '<S83>:1:4' covOut = covf + Q; */
  a = LATCtrl_B.UnitDelay6 + 0.1;

  /* '<S83>:1:5' K = covOut/(covOut+R); */
  /* '<S83>:1:6' y = V_pre + K*(SignalIn - V_pre); */
  LATC_vLatSpdFlt4LQR = a / (a + 1.0) * (LATC_vDistErrFlt4LQR - maxval) + maxval;
  LATCtrl_B.covOut = a;

  /* Switch: '<S73>/Switch1' incorporates:
   *  Constant: '<S73>/LQR_Ele1Man_C'
   *  Constant: '<S73>/LQR_Ele1Man_swt'
   */
  if (LQR_Ele1Man_swt != 0) {
    LATCtrl_B.Switch1_l = LQR_Ele1Man_C;
  } else {
    /* Sum: '<S73>/Sum6' */
    LATCtrl_B.Sum6_e = LATC_vDistErrFlt4LQR + LATCtrl_B.Switch5_d[0];
    printf("LATCtrl_B.Switch5_d[0]= %f",LATCtrl_B.Switch5_d[0]);
    LATCtrl_B.Switch1_l = LATCtrl_B.Sum6_e;
  }

  /* End of Switch: '<S73>/Switch1' */

  /* Switch: '<S73>/Switch2' incorporates:
   *  Constant: '<S73>/LQR_Ele2Man_C'
   *  Constant: '<S73>/LQR_Ele2Man_swt'
   */
  if (LQR_Ele2Man_swt != 0) {
    LATCtrl_B.Switch2_m = LQR_Ele2Man_C;
  } else {
    /* Sum: '<S73>/Sum1' */
    LATCtrl_B.Sum1_p = LATC_vLatSpdFlt4LQR + LATCtrl_B.Switch5_d[1];
    LATCtrl_B.Switch2_m = LATCtrl_B.Sum1_p;
  }

  /* End of Switch: '<S73>/Switch2' */

  /* Switch: '<S73>/Switch5' */
  LATCtrl_B.Switch5_c = LATCtrl_B.Switch1_l;

  /* Switch: '<S73>/Switch6' */
  LATCtrl_B.Switch6_j = LATCtrl_B.Switch2_m;

  /* Switch: '<S73>/Switch7' */
  if (LATCtrl_B.LOA_m) {
    /* Switch: '<S73>/Switch9' incorporates:
     *  Constant: '<S73>/LATC_swtAgComp4Em_C'
     */
    if (LATC_swtAgComp4Em_C != 0) {
      /* Product: '<S73>/Product3' */
      LATCtrl_B.Product3_n = LATCtrl_B.DLookupTable3 * LATCtrl_B.Switch3_d;
      LATCtrl_B.Switch9 = LATCtrl_B.Product3_n;
    } else {
      LATCtrl_B.Switch9 = LATCtrl_B.Switch3_d;
    }

    /* End of Switch: '<S73>/Switch9' */
    LATCtrl_B.Switch7 = LATCtrl_B.Switch9;
  } else {
    /* Product: '<S73>/Product1' */
    LATCtrl_B.Product1_oe = LATCtrl_B.Switch3_d * LATCtrl_B.DLookupTable3;
    LATCtrl_B.Switch7 = LATCtrl_B.Product1_oe;
  }

  /* End of Switch: '<S73>/Switch7' */

  /* Switch: '<S73>/Switch8' */
  if (LATCtrl_B.LOA_m) {
    /* Switch: '<S73>/Switch10' incorporates:
     *  Constant: '<S73>/LATC_swtYRComp4Em_C'
     */
    if (LATC_swtYRComp4Em_C != 0) {
      /* Lookup_n-D: '<S73>/1-D Lookup Table1' */
      LATCtrl_B.DLookupTable1 = look1_binlxpw(LATCtrl_B.Abs_f, (const real_T*)
        LATCtrl_ConstP.pooled13, (const real_T*)
        LATCtrl_ConstP.DLookupTable1_tableDa, 4U);

      /* Product: '<S73>/Product4' */
      LATCtrl_B.Product4_o = LATCtrl_B.Product13_e * LATCtrl_B.DLookupTable1;
      LATCtrl_B.Switch10_h = LATCtrl_B.Product4_o;
    } else {
      LATCtrl_B.Switch10_h = LATCtrl_B.Product13_e;
    }

    /* End of Switch: '<S73>/Switch10' */
    LATCtrl_B.Switch8 = LATCtrl_B.Switch10_h;
  } else {
    /* Lookup_n-D: '<S73>/1-D Lookup Table4' */
    LATCtrl_B.DLookupTable4 = look1_binlxpw(LATCtrl_B.Abs_f, (const real_T*)
      LATCtrl_ConstP.pooled13, (const real_T*)
      LATCtrl_ConstP.DLookupTable4_tableDa, 4U);

    /* Product: '<S73>/Product2' */
    LATCtrl_B.Product2_il = LATCtrl_B.Switch4_b * LATCtrl_B.DLookupTable4;
    LATCtrl_B.Switch8 = LATCtrl_B.Product2_il;
  }

  /* End of Switch: '<S73>/Switch8' */

  /* DataStoreWrite: '<S74>/Data Store Write' */
  memcpy(&LATCtrl_DW.LATC_YawRateStore_i[0], &LATCtrl_B.Assignment_h[0], 10U *
         sizeof(real_T));

  /* SignalConversion: '<S23>/ConcatBufferAtMatrix Concatenate2In1' incorporates:
   *  Constant: '<S23>/Constant11'
   *  Constant: '<S23>/Constant19'
   *  Constant: '<S23>/Constant20'
   *  Constant: '<S23>/Constant24'
   */
  LATCtrl_B.MatrixConcatenate2_o[0] = LATC_Q1Wgt4LQR_C;
  LATCtrl_B.MatrixConcatenate2_o[1] = 0.0;
  LATCtrl_B.MatrixConcatenate2_o[2] = 0.0;
  LATCtrl_B.MatrixConcatenate2_o[3] = 0.0;

  /* SignalConversion: '<S23>/ConcatBufferAtMatrix Concatenate2In2' incorporates:
   *  Constant: '<S23>/Constant21'
   *  Constant: '<S23>/Constant22'
   *  Constant: '<S23>/Constant23'
   *  Constant: '<S23>/Constant25'
   */
  LATCtrl_B.MatrixConcatenate2_o[4] = 0.0;
  LATCtrl_B.MatrixConcatenate2_o[5] = LATC_Q2Wgt4LQR_C;
  LATCtrl_B.MatrixConcatenate2_o[6] = 0.0;
  LATCtrl_B.MatrixConcatenate2_o[7] = 0.0;

  /* SignalConversion: '<S23>/ConcatBufferAtMatrix Concatenate2In3' incorporates:
   *  Constant: '<S23>/Constant12'
   *  Constant: '<S23>/Constant13'
   *  Constant: '<S23>/Constant14'
   *  Constant: '<S23>/Constant26'
   */
  LATCtrl_B.MatrixConcatenate2_o[8] = 0.0;
  LATCtrl_B.MatrixConcatenate2_o[9] = 0.0;
  LATCtrl_B.MatrixConcatenate2_o[10] = LATC_Q3Wgt4LQR_C;
  LATCtrl_B.MatrixConcatenate2_o[11] = 0.0;

  /* SignalConversion: '<S23>/ConcatBufferAtMatrix Concatenate2In4' incorporates:
   *  Constant: '<S23>/Constant15'
   *  Constant: '<S23>/Constant16'
   *  Constant: '<S23>/Constant17'
   *  Constant: '<S23>/Constant18'
   */
  LATCtrl_B.MatrixConcatenate2_o[12] = 0.0;
  LATCtrl_B.MatrixConcatenate2_o[13] = 0.0;
  LATCtrl_B.MatrixConcatenate2_o[14] = 0.0;
  LATCtrl_B.MatrixConcatenate2_o[15] = LATC_Q4Wgt4LQR_C;

  /* SignalConversion: '<S67>/ConcatBufferAtMatrix ConcatenateIn1' incorporates:
   *  Constant: '<S67>/Constant1'
   *  Constant: '<S67>/Constant6'
   *  Constant: '<S67>/Constant7'
   *  Constant: '<S67>/Constant8'
   */
  LATCtrl_B.MatrixConcatenate_h[0] = 0.0;
  LATCtrl_B.MatrixConcatenate_h[1] = 0.0;
  LATCtrl_B.MatrixConcatenate_h[2] = 0.0;
  LATCtrl_B.MatrixConcatenate_h[3] = 0.0;

  /* Sum: '<S67>/Add1' */
  LATCtrl_B.Add1_e = LATCtrl_B.Switch1_h + LATCtrl_B.Switch2_i;

  /* Product: '<S67>/Product1' */
  LATCtrl_B.Product1_cy = LATCtrl_B.Switch3_h * LATCtrl_B.Gain1;

  /* Product: '<S67>/Divide' */
  LATCtrl_B.Divide_h = LATCtrl_B.Add1_e / LATCtrl_B.Product1_cy;

  /* Product: '<S67>/Product9' incorporates:
   *  Constant: '<S67>/Fac_Sign'
   */
  LATCtrl_B.Product9_h = LATCtrl_B.Divide_h * (-1.0);

  /* Product: '<S67>/Product10' */
  LATCtrl_B.Product10_e = LATCtrl_B.Switch1_h * LATCtrl_B.Switch5;

  /* Product: '<S67>/Product11' */
  LATCtrl_B.Product11_l = LATCtrl_B.Switch2_i * LATCtrl_B.Switch_ou;

  /* Sum: '<S67>/Subtract1' */
  LATCtrl_B.Subtract1_i = LATCtrl_B.Product10_e - LATCtrl_B.Product11_l;

  /* Product: '<S67>/Product12' */
  LATCtrl_B.Product12_n = LATCtrl_B.Switch4 * LATCtrl_B.Gain1;

  /* Product: '<S67>/Divide1' */
  LATCtrl_B.Divide1_e = LATCtrl_B.Subtract1_i / LATCtrl_B.Product12_n;

  /* Product: '<S67>/Product13' incorporates:
   *  Constant: '<S67>/Fac_Sign1'
   */
  LATCtrl_B.Product13_e3 = LATCtrl_B.Divide1_e * (-1.0);

  /* SignalConversion: '<S67>/ConcatBufferAtMatrix ConcatenateIn2' incorporates:
   *  Constant: '<S67>/Constant10'
   *  Constant: '<S67>/Constant9'
   */
  LATCtrl_B.MatrixConcatenate_h[4] = 1.0;
  LATCtrl_B.MatrixConcatenate_h[5] = LATCtrl_B.Product9_h;
  LATCtrl_B.MatrixConcatenate_h[6] = 0.0;
  LATCtrl_B.MatrixConcatenate_h[7] = LATCtrl_B.Product13_e3;

  /* Sum: '<S67>/Add2' */
  LATCtrl_B.Add2_b = LATCtrl_B.Switch1_h + LATCtrl_B.Switch2_i;

  /* Product: '<S67>/Divide2' */
  LATCtrl_B.Divide2_e2 = LATCtrl_B.Add2_b / LATCtrl_B.Switch3_h;

  /* Product: '<S67>/Product14' */
  LATCtrl_B.Product14 = LATCtrl_B.Switch1_h * LATCtrl_B.Switch5;

  /* Product: '<S67>/Product15' */
  LATCtrl_B.Product15 = LATCtrl_B.Switch2_i * LATCtrl_B.Switch_ou;

  /* Sum: '<S67>/Subtract2' */
  LATCtrl_B.Subtract2_g = LATCtrl_B.Product14 - LATCtrl_B.Product15;

  /* Product: '<S67>/Divide3' */
  LATCtrl_B.Divide3 = LATCtrl_B.Subtract2_g / LATCtrl_B.Switch4;

  /* SignalConversion: '<S67>/ConcatBufferAtMatrix ConcatenateIn3' incorporates:
   *  Constant: '<S67>/Constant11'
   *  Constant: '<S67>/Constant12'
   */
  LATCtrl_B.MatrixConcatenate_h[8] = 0.0;
  LATCtrl_B.MatrixConcatenate_h[9] = LATCtrl_B.Divide2_e2;
  LATCtrl_B.MatrixConcatenate_h[10] = 0.0;
  LATCtrl_B.MatrixConcatenate_h[11] = LATCtrl_B.Divide3;

  /* Product: '<S67>/Product16' */
  LATCtrl_B.Product16_e = LATCtrl_B.Switch1_h * LATCtrl_B.Switch5;

  /* Product: '<S67>/Product17' */
  LATCtrl_B.Product17_f = LATCtrl_B.Switch2_i * LATCtrl_B.Switch_ou;

  /* Sum: '<S67>/Subtract3' */
  LATCtrl_B.Subtract3_m = LATCtrl_B.Product16_e - LATCtrl_B.Product17_f;

  /* Product: '<S67>/Product18' */
  LATCtrl_B.Product18_o = LATCtrl_B.Switch3_h * LATCtrl_B.Gain1;

  /* Product: '<S67>/Divide4' */
  LATCtrl_B.Divide4_c = LATCtrl_B.Subtract3_m / LATCtrl_B.Product18_o;

  /* Product: '<S67>/Product19' incorporates:
   *  Constant: '<S67>/Fac_Sign2'
   */
  LATCtrl_B.Product19_d = LATCtrl_B.Divide4_c * (-1.0);

  /* Product: '<S67>/Product20' */
  LATCtrl_B.Product20_a = LATCtrl_B.Switch1_h * LATCtrl_B.Switch5 *
    LATCtrl_B.Switch5;

  /* Product: '<S67>/Product21' */
  LATCtrl_B.Product21_j = LATCtrl_B.Switch2_i * LATCtrl_B.Switch_ou *
    LATCtrl_B.Switch_ou;

  /* Sum: '<S67>/Add3' */
  LATCtrl_B.Add3_l = LATCtrl_B.Product20_a + LATCtrl_B.Product21_j;

  /* Product: '<S67>/Product22' */
  LATCtrl_B.Product22_k = LATCtrl_B.Switch4 * LATCtrl_B.Gain1;

  /* Product: '<S67>/Divide5' */
  LATCtrl_B.Divide5_o = LATCtrl_B.Add3_l / LATCtrl_B.Product22_k;

  /* Product: '<S67>/Product23' incorporates:
   *  Constant: '<S67>/Fac_Sign3'
   */
  LATCtrl_B.Product23_c = LATCtrl_B.Divide5_o * (-1.0);

  /* SignalConversion: '<S67>/ConcatBufferAtMatrix ConcatenateIn4' incorporates:
   *  Constant: '<S67>/Constant13'
   *  Constant: '<S67>/Constant14'
   */
  LATCtrl_B.MatrixConcatenate_h[12] = 0.0;
  LATCtrl_B.MatrixConcatenate_h[13] = LATCtrl_B.Product19_d;
  LATCtrl_B.MatrixConcatenate_h[14] = 1.0;
  LATCtrl_B.MatrixConcatenate_h[15] = LATCtrl_B.Product23_c;

  /* SignalConversion: '<S67>/ConcatBufferAtMatrix Concatenate2In1' incorporates:
   *  Constant: '<S67>/Constant15'
   */
  LATCtrl_B.MatrixConcatenate2_l[0] = 0.0;

  /* Product: '<S67>/Divide6' */
  LATCtrl_B.MatrixConcatenate2_l[1] = LATCtrl_B.Switch1_h / LATCtrl_B.Switch3_h;

  /* SignalConversion: '<S67>/ConcatBufferAtMatrix Concatenate2In3' incorporates:
   *  Constant: '<S67>/Constant16'
   */
  LATCtrl_B.MatrixConcatenate2_l[2] = 0.0;

  /* Product: '<S67>/Product24' */
  LATCtrl_B.Product24_c = LATCtrl_B.Switch1_h * LATCtrl_B.Switch5;

  /* Product: '<S67>/Divide7' */
  LATCtrl_B.MatrixConcatenate2_l[3] = LATCtrl_B.Product24_c / LATCtrl_B.Switch4;

  /* Product: '<S67>/Product26' incorporates:
   *  Constant: '<S75>/Constant33'
   */
  LATCtrl_B.Product26_g[0] = LATCtrl_B.MatrixConcatenate2_l[0] * 0.01;
  LATCtrl_B.Product26_g[1] = LATCtrl_B.MatrixConcatenate2_l[1] * 0.01;
  LATCtrl_B.Product26_g[2] = LATCtrl_B.MatrixConcatenate2_l[2] * 0.01;
  LATCtrl_B.Product26_g[3] = LATCtrl_B.MatrixConcatenate2_l[3] * 0.01;

  /* Math: '<S67>/Math Function' */
  LATCtrl_B.MathFunction_n[0] = LATCtrl_B.Product26_g[0];
  LATCtrl_B.MathFunction_n[1] = LATCtrl_B.Product26_g[1];
  LATCtrl_B.MathFunction_n[2] = LATCtrl_B.Product26_g[2];
  LATCtrl_B.MathFunction_n[3] = LATCtrl_B.Product26_g[3];

  /* MATLAB Function 'LKA_SteerCtrl/LKA_SteerCalc4LQR/LQR_solver': '<S76>:1' */
  /* '<S76>:1:3' P=P_z; */
  for (i_0 = 0; i_0 < 16; i_0++) {
    /* Product: '<S67>/Product2' incorporates:
     *  Constant: '<S75>/Constant33'
     */
    LATCtrl_B.Product2_p2[i_0] = LATCtrl_B.MatrixConcatenate_h[i_0] * 0.01;

    /* Sum: '<S67>/Add4' incorporates:
     *  Constant: '<S67>/Constant2'
     */
    LATCtrl_B.Add4_k[i_0] = LATCtrl_B.Product2_p2[i_0] +
      LATCtrl_ConstP.pooled24[i_0];

    /* Delay: '<S23>/Variable Integer Delay3' */
    if (LATCtrl_DW.icLoad_c != 0) {
      LATCtrl_DW.VariableIntegerDelay3_DSTATE_m[i_0] =
        LATCtrl_B.MatrixConcatenate2_o[i_0];
    }

    LATCtrl_B.VariableIntegerDelay3_i[i_0] =
      LATCtrl_DW.VariableIntegerDelay3_DSTATE_m[i_0];

    /* End of Delay: '<S23>/Variable Integer Delay3' */

    /* MATLAB Function: '<S23>/LQR_solver' */
    LATCtrl_B.P_a[i_0] = LATCtrl_B.VariableIntegerDelay3_i[i_0];
  }

  /* MATLAB Function: '<S23>/LQR_solver' incorporates:
   *  Constant: '<S23>/Constant1'
   *  Constant: '<S23>/Constant27'
   *  Constant: '<S23>/Constant28'
   */
  /* '<S76>:1:4' i=uint16(0); */
  i = 0U;

  /* '<S76>:1:5' diff=50; */
  a = 50.0;

  /* '<S76>:1:6' while ((i <Max_num&&diff >= Max_thd)) */
  while ((i < LQR_CircleTime_C) && (a >= 0.01)) {
    /* '<S76>:1:7' P_lt=A'*P*A-A'*P*B/((R+B'*P*B))*(B'*P*A)+Q; */
    for (i_0 = 0; i_0 < 4; i_0++) {
      u_1 = LATCtrl_B.P_a[i_0 << 2] * LATCtrl_B.MathFunction_n[0];
      u_1 += LATCtrl_B.P_a[(i_0 << 2) + 1] * LATCtrl_B.MathFunction_n[1];
      u_1 += LATCtrl_B.P_a[(i_0 << 2) + 2] * LATCtrl_B.MathFunction_n[2];
      u_1 += LATCtrl_B.P_a[(i_0 << 2) + 3] * LATCtrl_B.MathFunction_n[3];
      y[i_0] = u_1;
    }

    u_1 = LATCtrl_B.MathFunction_n[0];
    v_1 = LATCtrl_B.MathFunction_n[1];
    u_0 = LATCtrl_B.MathFunction_n[2];
    b_idx = LATCtrl_B.MathFunction_n[3];
    maxval = y[0] * u_1;
    maxval += y[1] * v_1;
    maxval += y[2] * u_0;
    maxval += y[3] * b_idx;
    maxval = LATC_para4LQR_R_C + maxval;
    for (i_0 = 0; i_0 < 4; i_0++) {
      for (r = 0; r < 4; r++) {
        e_y[i_0 + (r << 2)] = 0.0;
        e_y[i_0 + (r << 2)] += LATCtrl_B.Add4_k[i_0 << 2] * LATCtrl_B.P_a[r << 2];
        e_y[i_0 + (r << 2)] += LATCtrl_B.Add4_k[(i_0 << 2) + 1] * LATCtrl_B.P_a
          [(r << 2) + 1];
        e_y[i_0 + (r << 2)] += LATCtrl_B.Add4_k[(i_0 << 2) + 2] * LATCtrl_B.P_a
          [(r << 2) + 2];
        e_y[i_0 + (r << 2)] += LATCtrl_B.Add4_k[(i_0 << 2) + 3] * LATCtrl_B.P_a
          [(r << 2) + 3];
      }
    }

    for (i_0 = 0; i_0 < 4; i_0++) {
      for (r = 0; r < 4; r++) {
        tmp_3[i_0 + (r << 2)] = 0.0;
        tmp_3[i_0 + (r << 2)] += LATCtrl_B.Add4_k[i_0 << 2] * LATCtrl_B.P_a[r <<
          2];
        tmp_3[i_0 + (r << 2)] += LATCtrl_B.Add4_k[(i_0 << 2) + 1] *
          LATCtrl_B.P_a[(r << 2) + 1];
        tmp_3[i_0 + (r << 2)] += LATCtrl_B.Add4_k[(i_0 << 2) + 2] *
          LATCtrl_B.P_a[(r << 2) + 2];
        tmp_3[i_0 + (r << 2)] += LATCtrl_B.Add4_k[(i_0 << 2) + 3] *
          LATCtrl_B.P_a[(r << 2) + 3];
      }
    }

    for (i_0 = 0; i_0 < 4; i_0++) {
      tmp_5 = LATCtrl_B.P_a[i_0 << 2] * LATCtrl_B.MathFunction_n[0];
      tmp_5 += LATCtrl_B.P_a[(i_0 << 2) + 1] * LATCtrl_B.MathFunction_n[1];
      tmp_5 += LATCtrl_B.P_a[(i_0 << 2) + 2] * LATCtrl_B.MathFunction_n[2];
      tmp_5 += LATCtrl_B.P_a[(i_0 << 2) + 3] * LATCtrl_B.MathFunction_n[3];
      tmp_1[i_0] = tmp_5;
    }

    for (i_0 = 0; i_0 < 4; i_0++) {
      tmp_5 = tmp_3[i_0] * LATCtrl_B.MathFunction_n[0];
      tmp_5 += tmp_3[i_0 + 4] * LATCtrl_B.MathFunction_n[1];
      tmp_5 += tmp_3[i_0 + 8] * LATCtrl_B.MathFunction_n[2];
      tmp_5 += tmp_3[i_0 + 12] * LATCtrl_B.MathFunction_n[3];
      tmp_4[i_0] = tmp_5 / maxval;
    }

    for (i_0 = 0; i_0 < 4; i_0++) {
      tmp_5 = LATCtrl_B.Add4_k[i_0 << 2] * tmp_1[0];
      tmp_5 += LATCtrl_B.Add4_k[(i_0 << 2) + 1] * tmp_1[1];
      tmp_5 += LATCtrl_B.Add4_k[(i_0 << 2) + 2] * tmp_1[2];
      tmp_5 += LATCtrl_B.Add4_k[(i_0 << 2) + 3] * tmp_1[3];
      tmp_2[i_0] = tmp_5;
    }

    for (i_0 = 0; i_0 < 4; i_0++) {
      for (r = 0; r < 4; r++) {
        tmp_3[i_0 + (r << 2)] = 0.0;
        tmp_3[i_0 + (r << 2)] += LATCtrl_B.Add4_k[r << 2] * e_y[i_0];
        tmp_3[i_0 + (r << 2)] += LATCtrl_B.Add4_k[(r << 2) + 1] * e_y[i_0 + 4];
        tmp_3[i_0 + (r << 2)] += LATCtrl_B.Add4_k[(r << 2) + 2] * e_y[i_0 + 8];
        tmp_3[i_0 + (r << 2)] += LATCtrl_B.Add4_k[(r << 2) + 3] * e_y[i_0 + 12];
      }
    }

    for (i_0 = 0; i_0 < 4; i_0++) {
      e_y[i_0] = tmp_4[i_0] * tmp_2[0];
      e_y[i_0 + 4] = tmp_4[i_0] * tmp_2[1];
      e_y[i_0 + 8] = tmp_4[i_0] * tmp_2[2];
      e_y[i_0 + 12] = tmp_4[i_0] * tmp_2[3];
    }

    for (i_0 = 0; i_0 < 4; i_0++) {
      P_lt[i_0 << 2] = (tmp_3[i_0 << 2] - e_y[i_0 << 2]) +
        LATCtrl_B.MatrixConcatenate2_o[i_0 << 2];
      P_lt[1 + (i_0 << 2)] = (tmp_3[(i_0 << 2) + 1] - e_y[(i_0 << 2) + 1]) +
        LATCtrl_B.MatrixConcatenate2_o[(i_0 << 2) + 1];
      P_lt[2 + (i_0 << 2)] = (tmp_3[(i_0 << 2) + 2] - e_y[(i_0 << 2) + 2]) +
        LATCtrl_B.MatrixConcatenate2_o[(i_0 << 2) + 2];
      P_lt[3 + (i_0 << 2)] = (tmp_3[(i_0 << 2) + 3] - e_y[(i_0 << 2) + 3]) +
        LATCtrl_B.MatrixConcatenate2_o[(i_0 << 2) + 3];
    }

    /* '<S76>:1:8' diff=max(max(abs(P_lt-P))); */
    for (i_0 = 0; i_0 < 16; i_0++) {
      LATCtrl_B.P_a[i_0] = P_lt[i_0] - LATCtrl_B.P_a[i_0];
      e_y[i_0] = fabs(LATCtrl_B.P_a[i_0]);
    }

    maxval = e_y[0];
    for (r_0 = 1; r_0 + 1 < 5; r_0++) {
      if (e_y[r_0] > maxval) {
        maxval = e_y[r_0];
      }
    }

    y[0] = maxval;
    maxval = e_y[4];
    for (r_0 = 5; r_0 + 1 < 9; r_0++) {
      if (e_y[r_0] > maxval) {
        maxval = e_y[r_0];
      }
    }

    y[1] = maxval;
    maxval = e_y[8];
    for (r_0 = 9; r_0 + 1 < 13; r_0++) {
      if (e_y[r_0] > maxval) {
        maxval = e_y[r_0];
      }
    }

    y[2] = maxval;
    maxval = e_y[12];
    for (r_0 = 13; r_0 + 1 < 17; r_0++) {
      if (e_y[r_0] > maxval) {
        maxval = e_y[r_0];
      }
    }

    y[3] = maxval;
    a = y[0];
    if (y[1] > a) {
      a = y[1];
    }

    if (y[2] > a) {
      a = y[2];
    }

    if (y[3] > a) {
      a = y[3];
    }

    /* '<S76>:1:9' P=P_lt; */
    memcpy(&LATCtrl_B.P_a[0], &P_lt[0], sizeof(real_T) << 4U);

    /* '<S76>:1:10' i=i+1; */
    i++;
  }

  /* '<S76>:1:12' num=i; */
  /* '<S76>:1:13' K= inv(R+B'*P*B)*(B'*P*A); */
  for (i_0 = 0; i_0 < 4; i_0++) {
    u_1 = LATCtrl_B.P_a[i_0 << 2] * LATCtrl_B.MathFunction_n[0];
    u_1 += LATCtrl_B.P_a[(i_0 << 2) + 1] * LATCtrl_B.MathFunction_n[1];
    u_1 += LATCtrl_B.P_a[(i_0 << 2) + 2] * LATCtrl_B.MathFunction_n[2];
    u_1 += LATCtrl_B.P_a[(i_0 << 2) + 3] * LATCtrl_B.MathFunction_n[3];
    y[i_0] = u_1;
  }

  u_1 = LATCtrl_B.MathFunction_n[0];
  v_1 = LATCtrl_B.MathFunction_n[1];
  u_0 = LATCtrl_B.MathFunction_n[2];
  b_idx = LATCtrl_B.MathFunction_n[3];
  maxval = y[0] * u_1;
  maxval += y[1] * v_1;
  maxval += y[2] * u_0;
  maxval += y[3] * b_idx;
  maxval = 1.0 / (LATC_para4LQR_R_C + maxval);
  for (i_0 = 0; i_0 < 4; i_0++) {
    tmp_5 = LATCtrl_B.P_a[i_0 << 2] * LATCtrl_B.MathFunction_n[0];
    tmp_5 += LATCtrl_B.P_a[(i_0 << 2) + 1] * LATCtrl_B.MathFunction_n[1];
    tmp_5 += LATCtrl_B.P_a[(i_0 << 2) + 2] * LATCtrl_B.MathFunction_n[2];
    tmp_5 += LATCtrl_B.P_a[(i_0 << 2) + 3] * LATCtrl_B.MathFunction_n[3];
    tmp_1[i_0] = tmp_5;
  }

  for (i_0 = 0; i_0 < 4; i_0++) {
    tmp_5 = LATCtrl_B.Add4_k[i_0 << 2] * tmp_1[0];
    tmp_5 += LATCtrl_B.Add4_k[(i_0 << 2) + 1] * tmp_1[1];
    tmp_5 += LATCtrl_B.Add4_k[(i_0 << 2) + 2] * tmp_1[2];
    tmp_5 += LATCtrl_B.Add4_k[(i_0 << 2) + 3] * tmp_1[3];
    tmp_2[i_0] = tmp_5;
  }

  LATCtrl_B.K[0] = maxval * tmp_2[0];
  LATCtrl_B.K[1] = maxval * tmp_2[1];
  LATCtrl_B.K[2] = maxval * tmp_2[2];
  LATCtrl_B.K[3] = maxval * tmp_2[3];
  LATCtrl_B.diff = a;
  LQR_IntNum = i;
  LATCtrl_B.i = i;

  /* Gain: '<S23>/Gain' */
  LATCtrl_B.Gain = 3.6 * LATCtrl_B.Gain1;

  /* Saturate: '<S23>/Saturation' */
  u_0 = LATCtrl_B.Gain;
  v_1 = 0.0;
  u_1 = 110.0;
  if (u_0 >= u_1) {
    LATCtrl_B.Saturation_m = u_1;
  } else if (u_0 <= v_1) {
    LATCtrl_B.Saturation_m = v_1;
  } else {
    LATCtrl_B.Saturation_m = u_0;
  }

  /* End of Saturate: '<S23>/Saturation' */

  /* Switch: '<S23>/Switch' */
  if (LATCtrl_B.LOA_m) {
    /* Lookup_n-D: '<S23>/K4_empty' */
    LATCtrl_B.MatrixConcatenate1_k[3] = look1_binlxpw(LATCtrl_B.Saturation_m, (
      const real_T*)LATCtrl_ConstP.pooled9, (const real_T*)
      LATCtrl_ConstP.K4_empty_tableData, 110U);

    /* Lookup_n-D: '<S23>/K3_empty' */
    LATCtrl_B.MatrixConcatenate1_k[2] = look1_binlxpw(LATCtrl_B.Saturation_m, (
      const real_T*)LATCtrl_ConstP.pooled10, (const real_T*)
      LATCtrl_ConstP.K3_empty_tableData, 37U);

    /* Lookup_n-D: '<S23>/K2_empty' */
    LATCtrl_B.MatrixConcatenate1_k[1] = look1_binlxpw(LATCtrl_B.Saturation_m, (
      const real_T*)LATCtrl_ConstP.pooled11, (const real_T*)
      LATCtrl_ConstP.K2_empty_tableData, 1U);

    /* Lookup_n-D: '<S23>/K1_empty' */
    LATCtrl_B.MatrixConcatenate1_k[0] = look1_binlxpw(LATCtrl_B.Saturation_m, (
      const real_T*)LATCtrl_ConstP.pooled12, (const real_T*)
      LATCtrl_ConstP.K1_empty_tableData, 22U);
    LATCtrl_B.Switch_d[0] = LATCtrl_B.MatrixConcatenate1_k[0];
    LATCtrl_B.Switch_d[1] = LATCtrl_B.MatrixConcatenate1_k[1];
    LATCtrl_B.Switch_d[2] = LATCtrl_B.MatrixConcatenate1_k[2];
    LATCtrl_B.Switch_d[3] = LATCtrl_B.MatrixConcatenate1_k[3];
  } else {
    /* Lookup_n-D: '<S23>/K4_Full' */
    LATCtrl_B.MatrixConcatenate3_m[3] = look1_binlxpw(LATCtrl_B.Saturation_m, (
      const real_T*)LATCtrl_ConstP.pooled9, (const real_T*)
      LATCtrl_ConstP.K4_Full_tableData, 110U);

    /* Lookup_n-D: '<S23>/K3_Full' */
    LATCtrl_B.MatrixConcatenate3_m[2] = look1_binlxpw(LATCtrl_B.Saturation_m, (
      const real_T*)LATCtrl_ConstP.pooled10, (const real_T*)
      LATCtrl_ConstP.K3_Full_tableData, 37U);

    /* Lookup_n-D: '<S23>/K2_Full' */
    LATCtrl_B.MatrixConcatenate3_m[1] = look1_binlxpw(LATCtrl_B.Saturation_m, (
      const real_T*)LATCtrl_ConstP.pooled11, (const real_T*)
      LATCtrl_ConstP.K2_Full_tableData, 1U);

    /* Lookup_n-D: '<S23>/K1_Full' */
    LATCtrl_B.MatrixConcatenate3_m[0] = look1_binlxpw(LATCtrl_B.Saturation_m, (
      const real_T*)LATCtrl_ConstP.pooled12, (const real_T*)
      LATCtrl_ConstP.K1_Full_tableData, 22U);
    LATCtrl_B.Switch_d[0] = LATCtrl_B.MatrixConcatenate3_m[0];
    LATCtrl_B.Switch_d[1] = LATCtrl_B.MatrixConcatenate3_m[1];
    LATCtrl_B.Switch_d[2] = LATCtrl_B.MatrixConcatenate3_m[2];
    LATCtrl_B.Switch_d[3] = LATCtrl_B.MatrixConcatenate3_m[3];
  }

  /* End of Switch: '<S23>/Switch' */

  /* Switch: '<S23>/Switch1' incorporates:
   *  Constant: '<S23>/LQR_Calc_flag'
   */
  if (LQR_Calc_flag > 0) {
    LATCtrl_B.Switch1_b[0] = LATCtrl_B.Switch_d[0];
    LATCtrl_B.Switch1_b[1] = LATCtrl_B.Switch_d[1];
    LATCtrl_B.Switch1_b[2] = LATCtrl_B.Switch_d[2];
    LATCtrl_B.Switch1_b[3] = LATCtrl_B.Switch_d[3];
  } else {
    LATCtrl_B.Switch1_b[0] = LATCtrl_B.K[0];
    LATCtrl_B.Switch1_b[1] = LATCtrl_B.K[1];
    LATCtrl_B.Switch1_b[2] = LATCtrl_B.K[2];
    LATCtrl_B.Switch1_b[3] = LATCtrl_B.K[3];
  }

  /* End of Switch: '<S23>/Switch1' */

  /* Product: '<S23>/Product8' incorporates:
   *  Constant: '<S23>/Constant5'
   */
  LATCtrl_B.Product8_n[0] = LATCtrl_B.Switch1_b[0] * LATC_Sign4LQR_FB_C;
  LATCtrl_B.Product8_n[1] = LATCtrl_B.Switch1_b[1] * LATC_Sign4LQR_FB_C;
  LATCtrl_B.Product8_n[2] = LATCtrl_B.Switch1_b[2] * LATC_Sign4LQR_FB_C;
  LATCtrl_B.Product8_n[3] = LATCtrl_B.Switch1_b[3] * LATC_Sign4LQR_FB_C;

  /* SignalConversion: '<S23>/TmpSignal ConversionAtProductInport2' */
  LATCtrl_B.TmpSignalConversionAtProductInp[0] = LATCtrl_B.Switch5_c;
  LATCtrl_B.TmpSignalConversionAtProductInp[1] = LATCtrl_B.Switch6_j;
  LATCtrl_B.TmpSignalConversionAtProductInp[2] = LATCtrl_B.Switch7;
  LATCtrl_B.TmpSignalConversionAtProductInp[3] = LATCtrl_B.Switch8;

  /* Product: '<S23>/Product' */
  y[0] = LATCtrl_B.Product8_n[0];
  y[1] = LATCtrl_B.Product8_n[1];
  y[2] = LATCtrl_B.Product8_n[2];
  y[3] = LATCtrl_B.Product8_n[3];
  u_1 = LATCtrl_B.TmpSignalConversionAtProductInp[0];
  v_1 = LATCtrl_B.TmpSignalConversionAtProductInp[1];
  u_0 = LATCtrl_B.TmpSignalConversionAtProductInp[2];
  b_idx = LATCtrl_B.TmpSignalConversionAtProductInp[3];
  tmp_5 = y[0] * u_1;
  tmp_5 += y[1] * v_1;
  tmp_5 += y[2] * u_0;
  tmp_5 += y[3] * b_idx;
  LATC_agFrTyre4LQR = tmp_5;

  /* DiscreteTransferFcn: '<S70>/Discrete Transfer Fcn' */
  maxval = LATC_agFrTyre4LQR;
  maxval -= (-0.987) * LATCtrl_DW.DiscreteTransferFcn_states;
  maxval /= 1.0;
  LATCtrl_DW.DiscreteTransferFcn_tmp = maxval;
  maxval = 4.0 * LATCtrl_DW.DiscreteTransferFcn_tmp;
  maxval = (-3.987) * LATCtrl_DW.DiscreteTransferFcn_states + maxval;
  LATCtrl_B.DiscreteTransferFcn = maxval;

  /* Switch: '<S70>/Switch10' incorporates:
   *  Constant: '<S70>/Intg_Ini'
   */
  if (LATCtrl_B.R_f) {
    /* Product: '<S70>/Product13' incorporates:
     *  Constant: '<S70>/LATC_facLeadLag4Empty_C'
     */
    LATCtrl_B.Product13_h = LATCtrl_B.DiscreteTransferFcn *
      LATC_facLeadLag4Empty_C;
    LATCtrl_B.Switch10 = LATCtrl_B.Product13_h;
  } else {
    LATCtrl_B.Switch10 = 0.0;
  }

  /* End of Switch: '<S70>/Switch10' */

  /* DiscreteTransferFcn: '<S70>/Discrete Transfer Fcn2' */
  maxval = LATCtrl_B.Switch10;
  maxval -= (-39.99) * LATCtrl_DW.DiscreteTransferFcn2_states;
  maxval /= 40.0;
  LATCtrl_DW.DiscreteTransferFcn2_tmp = maxval;
  maxval = 20.0 * LATCtrl_DW.DiscreteTransferFcn2_tmp;
  maxval = (-19.99) * LATCtrl_DW.DiscreteTransferFcn2_states + maxval;
  LATCtrl_B.DiscreteTransferFcn2 = maxval;

  /* DiscreteTransferFcn: '<S71>/Discrete Transfer Fcn' */
  maxval = LATC_agFrTyre4LQR;
  maxval -= (-0.96) * LATCtrl_DW.DiscreteTransferFcn_states_j;
  maxval /= 1.0;
  LATCtrl_DW.DiscreteTransferFcn_tmp_b = maxval;
  maxval = 5.0 * LATCtrl_DW.DiscreteTransferFcn_tmp_b;
  maxval = (-4.96) * LATCtrl_DW.DiscreteTransferFcn_states_j + maxval;
  LATCtrl_B.DiscreteTransferFcn_p = maxval;

  /* Switch: '<S71>/Switch10' incorporates:
   *  Constant: '<S71>/Intg_Ini'
   */
  if (LATCtrl_B.R_f) {
    /* Product: '<S71>/Product13' incorporates:
     *  Constant: '<S71>/LATC_facLeadLag4Full_C'
     */
    LATCtrl_B.Product13_a = LATCtrl_B.DiscreteTransferFcn_p *
      LATC_facLeadLag4Full_C;
    LATCtrl_B.Switch10_l = LATCtrl_B.Product13_a;
  } else {
    LATCtrl_B.Switch10_l = 0.0;
  }

  /* End of Switch: '<S71>/Switch10' */

  /* DiscreteTransferFcn: '<S71>/Discrete Transfer Fcn2' */
  maxval = LATCtrl_B.Switch10_l;
  maxval -= (-39.99) * LATCtrl_DW.DiscreteTransferFcn2_states_l;
  maxval /= 40.0;
  LATCtrl_DW.DiscreteTransferFcn2_tmp_h = maxval;
  maxval = 20.0 * LATCtrl_DW.DiscreteTransferFcn2_tmp_h;
  maxval = (-19.99) * LATCtrl_DW.DiscreteTransferFcn2_states_l + maxval;
  LATCtrl_B.DiscreteTransferFcn2_d = maxval;

  /* Product: '<S23>/Product1' incorporates:
   *  Constant: '<S23>/Constant4'
   */
  LATC_agFrTyreFF = LATC_lEqWheelBase_C * LATCtrl_B.Product7_hr;

  /* Product: '<S81>/Product2' */
  LATC_agFrTyreFBEle1 = LATCtrl_B.Product8_n[0] * LATCtrl_B.Switch5_c;
  printf("LATCtrl_B.Product8_n[0] = %f \n", LATCtrl_B.Product8_n[0]);
  printf("LATCtrl_B.Switch5_c = %d \n", LATCtrl_B.Switch5_c);

  /* Product: '<S81>/Product3' */
  LATC_agFrTyreFBEle2 = LATCtrl_B.Product8_n[1] * LATCtrl_B.Switch6_j;

  /* Product: '<S81>/Product4' */
  LATC_agFrTyreFBEle3 = LATCtrl_B.Product8_n[2] * LATCtrl_B.Switch7;

  /* Product: '<S81>/Product5' */
  LATC_agFrTyreFBEle4 = LATCtrl_B.Product8_n[3] * LATCtrl_B.Switch8;

  /* Sum: '<S81>/Add' */
  LATC_agFrTyreFB4Math = ((LATC_agFrTyreFBEle1 + LATC_agFrTyreFBEle2) +
    LATC_agFrTyreFBEle3) + LATC_agFrTyreFBEle4;

  /* DataTypeConversion: '<S81>/Data Type Conversion1' */
  LATC_eleK1 = LATCtrl_B.Product8_n[0];

  /* DataTypeConversion: '<S81>/Data Type Conversion2' */
  LATC_eleK2 = LATCtrl_B.Product8_n[1];

  /* DataTypeConversion: '<S81>/Data Type Conversion3' */
  LATC_eleK3 = LATCtrl_B.Product8_n[2];

  /* DataTypeConversion: '<S81>/Data Type Conversion5' */
  LATC_eleK4 = LATCtrl_B.Product8_n[3];

  /* Switch: '<S23>/Switch7' incorporates:
   *  Switch: '<S71>/Switch9'
   */
  if (LATCtrl_B.LOA_m) {
    /* Switch: '<S70>/Switch1' incorporates:
     *  Constant: '<S70>/LATC_swtLeadLag_C'
     */
    if (LATC_swtLeadLag_C != 0) {
      /* Logic: '<S70>/LA' incorporates:
       *  Constant: '<S70>/LATC_swtLead_C'
       */
      LATCtrl_B.LA_i = (LATCtrl_B.R_f && (LATC_swtLead_C != 0));

      /* Switch: '<S70>/Switch9' */
      if (LATCtrl_B.LA_i) {
        LATCtrl_B.Switch9_l = LATCtrl_B.DiscreteTransferFcn2;
      } else {
        /* Product: '<S70>/Product14' incorporates:
         *  Constant: '<S70>/LATC_facLeadCompFB_C'
         */
        LATCtrl_B.Product14_b = LATCtrl_B.DiscreteTransferFcn *
          LATC_facLeadCompFB_C;
        LATCtrl_B.Switch9_l = LATCtrl_B.Product14_b;
      }

      /* End of Switch: '<S70>/Switch9' */
      LATCtrl_B.Switch1_c = LATCtrl_B.Switch9_l;
    } else {
      LATCtrl_B.Switch1_c = LATC_agFrTyre4LQR;
    }

    /* End of Switch: '<S70>/Switch1' */
    LATC_agFrTyreFB = LATCtrl_B.Switch1_c;
  } else {
    if (LATCtrl_B.R_f) {
      /* Switch: '<S71>/Switch9' */
      LATCtrl_B.Switch9_b = LATCtrl_B.DiscreteTransferFcn2_d;
    } else {
      /* Product: '<S71>/Product14' incorporates:
       *  Constant: '<S71>/LATC_facLead4Full_C'
       *  Switch: '<S71>/Switch9'
       */
      LATCtrl_B.Product14_m = LATCtrl_B.DiscreteTransferFcn_p *
        LATC_facLead4Full_C;

      /* Switch: '<S71>/Switch9' */
      LATCtrl_B.Switch9_b = LATCtrl_B.Product14_m;
    }

    LATC_agFrTyreFB = LATCtrl_B.Switch9_b;
  }

  /* End of Switch: '<S23>/Switch7' */

  /* Trigonometry: '<S5>/Trigonometric Function2' */
  LATCtrl_B.TrigonometricFunction2 = atan(LATC_facLeftLaneA1);

  /* Trigonometry: '<S5>/Trigonometric Function3' */
  LATCtrl_B.TrigonometricFunction3 = atan(LATC_facRgtLaneA1);

  /* Sum: '<S5>/Add6' */
  LATCtrl_B.Add6 = LATCtrl_B.TrigonometricFunction2 +
    LATCtrl_B.TrigonometricFunction3;

  /* Product: '<S5>/Divide5' incorporates:
   *  Constant: '<S5>/FAC_8'
   */
  LATCtrl_B.Divide5_d = LATCtrl_B.Add6 / 2.0;

  /* Product: '<S53>/Divide3' incorporates:
   *  Constant: '<S53>/RAD2DEG'
   */
  LATCtrl_B.MatrixConcatenate_hc[1] = LATCtrl_B.Switch3_e / 57.3;

  /* RelationalOperator: '<S19>/Relational Operator1' incorporates:
   *  Constant: '<S19>/LATC_facZeroIdfyFgt_C1'
   */
  LATCtrl_B.RelationalOperator1_p = (LATCtrl_B.MinMax >= 1.0);

  /* Outputs for Enabled SubSystem: '<S19>/LATC_ZeroPosition' incorporates:
   *  EnablePort: '<S47>/Enable'
   */
  if (LATCtrl_B.RelationalOperator1_p) {
    if (!LATCtrl_DW.LATC_ZeroPosition_MODE) {
      /* InitializeConditions for Delay: '<S47>/Delay' */
      LATCtrl_DW.Delay_DSTATE_o = 0.0;

      /* InitializeConditions for UnitDelay: '<S48>/delay' */
      LATCtrl_DW.delay_DSTATE[0] = 0.0;
      LATCtrl_DW.delay_DSTATE[1] = 0.02;
      LATCtrl_DW.delay_DSTATE[2] = 0.0;

      /* InitializeConditions for UnitDelay: '<S48>/delay1' */
      memcpy(&LATCtrl_DW.delay1_DSTATE[0], (const real_T*)LATCtrl_ConstP.pooled7,
             9U * sizeof(real_T));
      LATCtrl_DW.LATC_ZeroPosition_MODE = TRUE;
    }

    /* SignalConversion: '<S47>/ConcatBufferAtMatrix ConcatenateIn3' incorporates:
     *  Constant: '<S47>/Constant1'
     */
    LATCtrl_B.MatrixConcatenate_hc[2] = (-1.0);

    /* Delay: '<S47>/Delay' */
    LATCtrl_B.Delay_n = LATCtrl_DW.Delay_DSTATE_o;

    /* UnitDelay: '<S48>/delay' */
    LATCtrl_B.delay[0] = LATCtrl_DW.delay_DSTATE[0];
    LATCtrl_B.delay[1] = LATCtrl_DW.delay_DSTATE[1];
    LATCtrl_B.delay[2] = LATCtrl_DW.delay_DSTATE[2];

    /* Product: '<S47>/Divide3' incorporates:
     *  Constant: '<S47>/Constant2'
     */
    LATCtrl_B.MatrixConcatenate_hc[0] = (-1.0) * LATCtrl_B.Add3 /
      LATCtrl_B.MinMax;

    /* Math: '<S47>/Math Function2' */
    LATCtrl_B.MathFunction2_o[0] = LATCtrl_B.MatrixConcatenate_hc[0];
    LATCtrl_B.MathFunction2_o[1] = LATCtrl_B.MatrixConcatenate_hc[1];
    LATCtrl_B.MathFunction2_o[2] = LATCtrl_B.MatrixConcatenate_hc[2];
    for (i_0 = 0; i_0 < 9; i_0++) {
      /* UnitDelay: '<S48>/delay1' */
      LATCtrl_B.delay1[i_0] = LATCtrl_DW.delay1_DSTATE[i_0];

      /* Product: '<S48>/Divide' */
      Pk[i_0] = LATCtrl_B.delay1[i_0];
    }

    /* Product: '<S48>/Divide' */
    u[0] = LATCtrl_B.MathFunction2_o[0];
    u[1] = LATCtrl_B.MathFunction2_o[1];
    u[2] = LATCtrl_B.MathFunction2_o[2];
    for (i_0 = 0; i_0 < 3; i_0++) {
      LATCtrl_B.Divide_j[i_0] = 0.0;
      LATCtrl_B.Divide_j[i_0] += Pk[i_0] * u[0];
      LATCtrl_B.Divide_j[i_0] += Pk[i_0 + 3] * u[1];
      LATCtrl_B.Divide_j[i_0] += Pk[i_0 + 6] * u[2];
    }

    /* Math: '<S48>/Math Function' */
    LATCtrl_B.MathFunction_a[0] = LATCtrl_B.MathFunction2_o[0];
    LATCtrl_B.MathFunction_a[1] = LATCtrl_B.MathFunction2_o[1];
    LATCtrl_B.MathFunction_a[2] = LATCtrl_B.MathFunction2_o[2];

    /* Product: '<S48>/Divide3' */
    memcpy(&Pk[0], &LATCtrl_B.delay1[0], 9U * sizeof(real_T));
    u[0] = LATCtrl_B.MathFunction2_o[0];
    u[1] = LATCtrl_B.MathFunction2_o[1];
    u[2] = LATCtrl_B.MathFunction2_o[2];
    for (i_0 = 0; i_0 < 3; i_0++) {
      u_1 = Pk[i_0] * u[0];
      u_1 += Pk[i_0 + 3] * u[1];
      u_1 += Pk[i_0 + 6] * u[2];
      v[i_0] = u_1;
    }

    v_1 = LATCtrl_B.MathFunction_a[0];
    maxval = LATCtrl_B.MathFunction_a[1];
    a = LATCtrl_B.MathFunction_a[2];
    tmp_5 = v_1 * v[0];
    tmp_5 += maxval * v[1];
    tmp_5 += a * v[2];
    LATCtrl_B.Divide3_i = tmp_5;

    /* End of Product: '<S48>/Divide3' */

    /* Sum: '<S48>/Sum1' incorporates:
     *  Constant: '<S19>/LATC_facZeroIdfyFgt_C'
     */
    LATCtrl_B.Sum1_n = LATC_facZeroIdfyFgt_C + LATCtrl_B.Divide3_i;

    /* Product: '<S48>/Divide2' */
    u[0] = LATCtrl_B.Divide_j[0];
    u[1] = LATCtrl_B.Divide_j[1];
    u[2] = LATCtrl_B.Divide_j[2];
    a = LATCtrl_B.Sum1_n;
    LATCtrl_B.Divide2_m[0] = u[0] / a;
    LATCtrl_B.Divide2_m[1] = u[1] / a;
    LATCtrl_B.Divide2_m[2] = u[2] / a;

    /* Math: '<S48>/Math Function2' */
    LATCtrl_B.MathFunction2_kx[0] = LATCtrl_B.MathFunction2_o[0];
    LATCtrl_B.MathFunction2_kx[1] = LATCtrl_B.MathFunction2_o[1];
    LATCtrl_B.MathFunction2_kx[2] = LATCtrl_B.MathFunction2_o[2];

    /* Product: '<S48>/Product1' */
    v_1 = LATCtrl_B.MathFunction2_kx[0];
    maxval = LATCtrl_B.MathFunction2_kx[1];
    a = LATCtrl_B.MathFunction2_kx[2];
    u_1 = LATCtrl_B.delay[0];
    u_0 = LATCtrl_B.delay[1];
    b_idx = LATCtrl_B.delay[2];
    tmp_5 = v_1 * u_1;
    tmp_5 += maxval * u_0;
    tmp_5 += a * b_idx;
    LATCtrl_B.Product1_j = tmp_5;

    /* Sum: '<S47>/Sum' */
    LATCtrl_B.Sum_m = LATCtrl_B.Add3 - LATCtrl_B.Delay_n;

    /* Product: '<S47>/Product1' incorporates:
     *  Constant: '<S47>/Constant'
     */
    LATCtrl_B.Product1_ir = LATCtrl_B.Sum_m * 100.0;

    /* Sum: '<S48>/Sum' */
    LATCtrl_B.Sum_o = LATCtrl_B.Product1_ir - LATCtrl_B.Product1_j;

    /* Product: '<S48>/Divide5' */
    u[0] = LATCtrl_B.Divide2_m[0];
    u[1] = LATCtrl_B.Divide2_m[1];
    u[2] = LATCtrl_B.Divide2_m[2];
    a = LATCtrl_B.Sum_o;
    v_1 = u[0];
    v_1 *= a;
    u[0] = v_1;
    v_1 = u[1];
    v_1 *= a;
    u[1] = v_1;
    v_1 = u[2];
    v_1 *= a;
    u[2] = v_1;
    LATCtrl_B.Divide5_j[0] = u[0];
    LATCtrl_B.Divide5_j[1] = u[1];
    LATCtrl_B.Divide5_j[2] = u[2];

    /* Sum: '<S48>/Sum2' */
    LATCtrl_B.Sum2_n[0] = LATCtrl_B.delay[0] + LATCtrl_B.Divide5_j[0];
    LATCtrl_B.Sum2_n[1] = LATCtrl_B.delay[1] + LATCtrl_B.Divide5_j[1];
    LATCtrl_B.Sum2_n[2] = LATCtrl_B.delay[2] + LATCtrl_B.Divide5_j[2];

    /* RelationalOperator: '<S49>/LowerRelop1' incorporates:
     *  Constant: '<S48>/max1'
     */
    LATCtrl_B.LowerRelop1 = (LATCtrl_B.Sum2_n[1] > 1.0);

    /* RelationalOperator: '<S49>/UpperRelop' incorporates:
     *  Constant: '<S48>/min1'
     */
    LATCtrl_B.UpperRelop = (LATCtrl_B.Sum2_n[1] < 0.01);

    /* Switch: '<S49>/Switch' incorporates:
     *  Constant: '<S48>/min1'
     */
    if (LATCtrl_B.UpperRelop) {
      LATCtrl_B.Switch_n3 = 0.01;
    } else {
      LATCtrl_B.Switch_n3 = LATCtrl_B.Sum2_n[1];
    }

    /* End of Switch: '<S49>/Switch' */

    /* Switch: '<S49>/Switch2' incorporates:
     *  Constant: '<S48>/max1'
     */
    if (LATCtrl_B.LowerRelop1) {
      LATCtrl_B.Switch2_h = 1.0;
    } else {
      LATCtrl_B.Switch2_h = LATCtrl_B.Switch_n3;
    }

    /* End of Switch: '<S49>/Switch2' */

    /* Product: '<S47>/Divide2' */
    LATCtrl_B.Divide2_a = 1.0 / LATCtrl_B.Switch2_h * LATCtrl_B.Sum2_n[2];

    /* Product: '<S47>/Product' incorporates:
     *  Constant: '<S47>/Constant4'
     */
    LATCtrl_B.Product_j = LATCtrl_B.Divide2_a * 57.3;

    /* Product: '<S48>/Divide6' incorporates:
     *  Constant: '<S19>/LATC_facZeroIdfyFgt_C'
     *  Constant: '<S48>/Constant2'
     */
    LATCtrl_B.Divide6 = 1.0 / LATC_facZeroIdfyFgt_C;

    /* Math: '<S48>/Math Function1' */
    LATCtrl_B.MathFunction1_h[0] = LATCtrl_B.MathFunction2_o[0];
    LATCtrl_B.MathFunction1_h[1] = LATCtrl_B.MathFunction2_o[1];
    LATCtrl_B.MathFunction1_h[2] = LATCtrl_B.MathFunction2_o[2];

    /* Product: '<S48>/Divide7' */
    v_1 = LATCtrl_B.MathFunction1_h[0];
    maxval = LATCtrl_B.MathFunction1_h[1];
    a = LATCtrl_B.MathFunction1_h[2];
    memcpy(&Pk[0], &LATCtrl_B.delay1[0], 9U * sizeof(real_T));
    for (i_0 = 0; i_0 < 3; i_0++) {
      u_1 = Pk[3 * i_0] * v_1;
      u_1 += Pk[3 * i_0 + 1] * maxval;
      u_1 += Pk[3 * i_0 + 2] * a;
      v_0[i_0] = u_1;
    }

    u[0] = LATCtrl_B.Divide2_m[0];
    u[1] = LATCtrl_B.Divide2_m[1];
    u[2] = LATCtrl_B.Divide2_m[2];
    for (i_0 = 0; i_0 < 3; i_0++) {
      LATCtrl_B.Divide7[i_0] = u[i_0] * v_0[0];
      LATCtrl_B.Divide7[i_0 + 3] = u[i_0] * v_0[1];
      LATCtrl_B.Divide7[i_0 + 6] = u[i_0] * v_0[2];
    }

    /* End of Product: '<S48>/Divide7' */

    /* Update for Delay: '<S47>/Delay' */
    LATCtrl_DW.Delay_DSTATE_o = LATCtrl_B.Add3;

    /* Update for UnitDelay: '<S48>/delay' */
    LATCtrl_DW.delay_DSTATE[0] = LATCtrl_B.Sum2_n[0];
    LATCtrl_DW.delay_DSTATE[1] = LATCtrl_B.Switch2_h;
    LATCtrl_DW.delay_DSTATE[2] = LATCtrl_B.Sum2_n[2];
    for (i_0 = 0; i_0 < 9; i_0++) {
      /* Sum: '<S48>/Sum3' */
      LATCtrl_B.Sum3[i_0] = LATCtrl_B.delay1[i_0] - LATCtrl_B.Divide7[i_0];

      /* Product: '<S48>/Divide9' */
      LATCtrl_B.Divide9[i_0] = LATCtrl_B.Sum3[i_0] * LATCtrl_B.Divide6;

      /* Update for UnitDelay: '<S48>/delay1' */
      LATCtrl_DW.delay1_DSTATE[i_0] = LATCtrl_B.Divide9[i_0];
    }
  } else {
    if (LATCtrl_DW.LATC_ZeroPosition_MODE) {
      LATCtrl_DW.LATC_ZeroPosition_MODE = FALSE;
    }
  }

  /* End of Outputs for SubSystem: '<S19>/LATC_ZeroPosition' */

  /* UnitDelay: '<S19>/Unit Delay' */
  LATCtrl_B.UnitDelay_a1 = LATCtrl_DW.UnitDelay_DSTATE_e;

  /* Logic: '<S19>/Logical Operator' */
  LATCtrl_B.LogicalOperator = !(LATCtrl_B.UnitDelay_a1 != 0);

  /* DataTypeConversion: '<S19>/Data Type Conversion' */
  LATCtrl_B.DataTypeConversion_p = LATCtrl_B.LogicalOperator;

  /* Constant: '<S19>/LATC_tZeroIdfyTimeThd_C' */
  LATCtrl_B.LATC_tZeroIdfyTimeThd_C_i = LATC_tZeroIdfyTimeThd_C;

  /* DataTypeConversion: '<Root>/Data Type Conversion2' incorporates:
   *  Inport: '<Root>/VMP_agHeadAngle4TgtPath'
   */
  LATC_agHead4TgtPath = VMP_agHeadAngle4TgtPath;

  /* Switch: '<S54>/Switch3' incorporates:
   *  Constant: '<S54>/LKA_ManStYawRateSt'
   *  Constant: '<S54>/LKA_ManStYawRate_C'
   */
  if (FALSE) {
    LATCtrl_B.Switch3_f = 0.0;
  } else {
    LATCtrl_B.Switch3_f = LATC_agHead4TgtPath;
  }

  /* End of Switch: '<S54>/Switch3' */

  /* Sum: '<S54>/Add3' incorporates:
   *  Constant: '<S54>/LKA_phiVehHeadOfst_C'
   */
  LATCtrl_B.Add3_e = LATCtrl_B.Switch3_f + 0.0;

  /* Trigonometry: '<S56>/Trigonometric Function' */
  LATCtrl_B.TrigonometricFunction_b = sin(LATC_agHead4TgtPath);

  /* Product: '<S56>/Product1' */
  LATCtrl_B.Product1_g0 = LATCtrl_ConstB.Add4 *
    LATCtrl_B.TrigonometricFunction_b;

  /* DataTypeConversion: '<Root>/Data Type Conversion1' incorporates:
   *  Inport: '<Root>/VMP_lLatPosn4TgtPath '
   */
  LATC_lLatPosn4TgtPath = VMP_lLatPosn4TgtPath;

  /* Sum: '<S56>/Add5' */
  LATCtrl_B.Add5_mi = LATC_lLatPosn4TgtPath - LATCtrl_B.Product1_g0;

  /* UnitDelay: '<S64>/D' */
  LATCtrl_B.D_n = LATCtrl_DW.D_DSTATE_a;

  /* Product: '<S62>/Product4' incorporates:
   *  Constant: '<S62>/K_feedfarward'
   */
  LATCtrl_B.Product4_m = K_feedfarward * LATC_agFrTyreFF;

  /* Sum: '<S62>/Add2' */
  LATCtrl_B.Add2_ep = LATCtrl_B.Product4_m + LATC_agFrTyreFB;

  /* Product: '<S62>/Product1' incorporates:
   *  Constant: '<S62>/LKA_AgSteerRatio'
   */
  LATC_agTarSteerRad = LATCtrl_B.Add2_ep * (real_T)((uint16_T)19U);

  /* Product: '<S62>/Product' incorporates:
   *  Constant: '<S62>/SAR_swt1'
   */
  LATC_agTarSteerDeg = LATC_agTarSteerRad * 57.3;

  /* Abs: '<S66>/Abs' */
  LATCtrl_B.Abs_h = fabs(LATC_agTarSteerDeg);

  /* RelationalOperator: '<S66>/RO1' incorporates:
   *  Constant: '<S59>/Up_Thd1'
   */
  LATCtrl_B.RO1_a = (LATCtrl_B.Abs_h > LATC_phiStRelaxMax_C);

  /* Product: '<S66>/Product1' incorporates:
   *  Constant: '<S59>/Up_Thd2'
   *  Constant: '<S59>/Up_Thd3'
   */
  LATCtrl_B.Product1_gn = LATC_facStRelaxK_C * LATC_phiStRelaxMin_C;

  /* Sum: '<S66>/Subtract' incorporates:
   *  Constant: '<S59>/Up_Thd1'
   */
  LATCtrl_B.Subtract_o = LATC_phiStRelaxMax_C - LATCtrl_B.Product1_gn;

  /* Sum: '<S66>/Subtract1' incorporates:
   *  Constant: '<S59>/Up_Thd1'
   *  Constant: '<S59>/Up_Thd2'
   */
  LATCtrl_B.Subtract1_d = LATC_phiStRelaxMax_C - LATC_phiStRelaxMin_C;

  /* Product: '<S66>/Divide' */
  LATCtrl_B.Divide_l = LATCtrl_B.Subtract_o / LATCtrl_B.Subtract1_d;

  /* Switch: '<S66>/Switch1' */
  if (LATCtrl_B.RO1_a) {
    LATCtrl_B.Switch1_j = LATC_agTarSteerDeg;
  } else {
    /* Abs: '<S66>/Abs1' */
    LATCtrl_B.Abs1_e = fabs(LATC_agTarSteerDeg);

    /* RelationalOperator: '<S66>/RO2' incorporates:
     *  Constant: '<S59>/Up_Thd2'
     */
    LATCtrl_B.RO2_az = (LATCtrl_B.Abs1_e < LATC_phiStRelaxMin_C);

    /* Switch: '<S66>/Switch2' */
    if (LATCtrl_B.RO2_az) {
      /* Product: '<S66>/Product2' incorporates:
       *  Constant: '<S59>/Up_Thd3'
       */
      LATCtrl_B.Product2_i5 = LATC_agTarSteerDeg * LATC_facStRelaxK_C;
      LATCtrl_B.Switch2_e = LATCtrl_B.Product2_i5;
    } else {
      /* RelationalOperator: '<S66>/RO3' incorporates:
       *  Constant: '<S59>/Up_Thd2'
       */
      LATCtrl_B.RO3_g = (LATC_agTarSteerDeg > LATC_phiStRelaxMin_C);

      /* Switch: '<S66>/Switch3' */
      if (LATCtrl_B.RO3_g) {
        /* Sum: '<S66>/Subtract2' incorporates:
         *  Constant: '<S59>/Up_Thd2'
         */
        LATCtrl_B.Subtract2_n = LATC_agTarSteerDeg - LATC_phiStRelaxMin_C;

        /* Product: '<S66>/Product4' */
        LATCtrl_B.Product4_f = LATCtrl_B.Divide_l * LATCtrl_B.Subtract2_n;

        /* Product: '<S66>/Product3' incorporates:
         *  Constant: '<S59>/Up_Thd2'
         *  Constant: '<S59>/Up_Thd3'
         */
        LATCtrl_B.Product3_nx = LATC_facStRelaxK_C * LATC_phiStRelaxMin_C;

        /* Sum: '<S66>/Add1' */
        LATCtrl_B.Add1_o = LATCtrl_B.Product3_nx + LATCtrl_B.Product4_f;
        LATCtrl_B.Switch3_i = LATCtrl_B.Add1_o;
      } else {
        /* Sum: '<S66>/Add3' incorporates:
         *  Constant: '<S59>/Up_Thd1'
         */
        LATCtrl_B.Add3_h = LATC_agTarSteerDeg + LATC_phiStRelaxMax_C;

        /* Product: '<S66>/Product6' */
        LATCtrl_B.Product6_m = LATCtrl_B.Divide_l * LATCtrl_B.Add3_h;

        /* Product: '<S66>/Product5' incorporates:
         *  Constant: '<S59>/Up_Thd1'
         *  Constant: '<S66>/Fac_Sign'
         */
        LATCtrl_B.Product5_h = LATC_phiStRelaxMax_C * (-1.0);

        /* Sum: '<S66>/Add2' */
        LATCtrl_B.Add2_n = LATCtrl_B.Product5_h + LATCtrl_B.Product6_m;
        LATCtrl_B.Switch3_i = LATCtrl_B.Add2_n;
      }

      /* End of Switch: '<S66>/Switch3' */
      LATCtrl_B.Switch2_e = LATCtrl_B.Switch3_i;
    }

    /* End of Switch: '<S66>/Switch2' */
    LATCtrl_B.Switch1_j = LATCtrl_B.Switch2_e;
  }

  /* End of Switch: '<S66>/Switch1' */

  /* Delay: '<S64>/Delay' */
  LATCtrl_B.Delay_i = LATCtrl_DW.Delay_DSTATE_n[0];

  /* Switch: '<S64>/Swt3' */
  if (LATCtrl_B.Delay_i) {
    /* RelationalOperator: '<S64>/R' */
    LATCtrl_B.R_d = (LATCtrl_B.Switch1_j > LATCtrl_B.D_n);

    /* Switch: '<S64>/Swt' */
    if (LATCtrl_B.R_d) {
      /* Product: '<S64>/P1' incorporates:
       *  Constant: '<S59>/CT'
       *  Constant: '<S59>/LKA_ManAgSteerOfstSel1'
       */
      LATCtrl_B.P1_l = LATC_agStrRateUpLmt_C * 0.01;

      /* Sum: '<S64>/A1' */
      LATCtrl_B.A1_m = LATCtrl_B.P1_l + LATCtrl_B.D_n;

      /* MinMax: '<S64>/MinMax2' */
      u_0 = LATCtrl_B.A1_m;
      maxval = LATCtrl_B.Switch1_j;
      if (u_0 <= maxval) {
        maxval = u_0;
      }

      LATCtrl_B.MinMax2_k = maxval;

      /* End of MinMax: '<S64>/MinMax2' */
      LATCtrl_B.Swt_i = LATCtrl_B.MinMax2_k;
    } else {
      /* RelationalOperator: '<S64>/R1' */
      LATCtrl_B.R1_i = (LATCtrl_B.Switch1_j < LATCtrl_B.D_n);

      /* Switch: '<S64>/Swt1' */
      if (LATCtrl_B.R1_i) {
        /* Product: '<S64>/P' incorporates:
         *  Constant: '<S59>/CT'
         *  Constant: '<S59>/LKA_ManAgSteerOfstSel2'
         */
        LATCtrl_B.P_dp = LATC_agStrRateLoLmt_C * 0.01;

        /* Sum: '<S64>/A' */
        LATCtrl_B.A_ds = LATCtrl_B.D_n + LATCtrl_B.P_dp;

        /* MinMax: '<S64>/MinMax1' */
        u_0 = LATCtrl_B.A_ds;
        maxval = LATCtrl_B.Switch1_j;
        if (u_0 >= maxval) {
          maxval = u_0;
        }

        LATCtrl_B.MinMax1_m = maxval;

        /* End of MinMax: '<S64>/MinMax1' */
        LATCtrl_B.Swt1_d = LATCtrl_B.MinMax1_m;
      } else {
        LATCtrl_B.Swt1_d = LATCtrl_B.D_n;
      }

      /* End of Switch: '<S64>/Swt1' */
      LATCtrl_B.Swt_i = LATCtrl_B.Swt1_d;
    }

    /* End of Switch: '<S64>/Swt' */
    LATC_agTarSteer4RL = LATCtrl_B.Swt_i;
  } else {
    LATC_agTarSteer4RL = LATCtrl_B.Switch1_j;
  }

  /* End of Switch: '<S64>/Swt3' */

  /* MinMax: '<S64>/MinMax4' incorporates:
   *  Constant: '<S59>/LKA_ManAgSteerOfstSel3'
   */
  u_0 = LATC_agStrUpLmt_C;
  maxval = LATC_agTarSteer4RL;
  if (u_0 <= maxval) {
    maxval = u_0;
  }

  LATCtrl_B.MinMax4 = maxval;

  /* End of MinMax: '<S64>/MinMax4' */

  /* MinMax: '<S64>/MinMax3' incorporates:
   *  Constant: '<S59>/LKA_ManAgSteerOfstSel4'
   */
  u_0 = LATCtrl_B.MinMax4;
  maxval = LATC_agStrLoLmt_C;
  if (u_0 >= maxval) {
    maxval = u_0;
  }

  LATCtrl_B.MinMax3 = maxval;

  /* End of MinMax: '<S64>/MinMax3' */

  /* Switch: '<S64>/Swt2' incorporates:
   *  Constant: '<S64>/LATC_swtAgSteerRate_C'
   */
  if (LATC_swtAgSteerRate_C != 0) {
    /* Signum: '<S64>/Sign' */
    u_1 = LATCtrl_B.MinMax3;
    if (u_1 < 0.0) {
      LATCtrl_B.Sign = -1.0;
    } else if (u_1 > 0.0) {
      LATCtrl_B.Sign = 1.0;
    } else {
      LATCtrl_B.Sign = u_1;
    }

    /* End of Signum: '<S64>/Sign' */

    /* Abs: '<S64>/Abs' */
    LATCtrl_B.Abs_l = fabs(LATCtrl_B.MinMax3);

    /* Switch: '<S64>/Switch' incorporates:
     *  Constant: '<S64>/Constant'
     *  Switch: '<S64>/Switch1'
     */
    if (LATC_vVehSpdKph >= 60.0) {
      LATCtrl_B.Switch_dp = 50.0;
    } else {
      if (LATC_vVehSpdKph >= 40.0) {
        /* Switch: '<S64>/Switch1' incorporates:
         *  Constant: '<S64>/Constant1'
         */
        LATCtrl_B.Switch1_ck = 100.0;
      } else {
        /* Switch: '<S64>/Switch1' incorporates:
         *  Constant: '<S64>/Constant2'
         */
        LATCtrl_B.Switch1_ck = 300.0;
      }

      LATCtrl_B.Switch_dp = LATCtrl_B.Switch1_ck;
    }

    /* End of Switch: '<S64>/Switch' */

    /* MinMax: '<S64>/MinMax5' */
    u_0 = LATCtrl_B.Switch_dp;
    maxval = LATCtrl_B.Abs_l;
    if (u_0 <= maxval) {
      maxval = u_0;
    }

    LATCtrl_B.MinMax5 = maxval;

    /* End of MinMax: '<S64>/MinMax5' */

    /* Product: '<S64>/P2' */
    LATCtrl_B.P2 = LATCtrl_B.MinMax5 * LATCtrl_B.Sign;
    LATC_agTarSteer4RLS = LATCtrl_B.P2;
  } else {
    LATC_agTarSteer4RLS = LATCtrl_B.Switch1_j;
  }

  /* End of Switch: '<S64>/Swt2' */

  /* Switch: '<S63>/Switch2' incorporates:
   *  Constant: '<S19>/Constant'
   *  Constant: '<S59>/LKA_swtAgSteerOfstSel_C'
   *  Constant: '<S59>/SA_Ofst_C'
   *  Switch: '<S63>/Switch1'
   */
  if (LKA_swtAgSteerOfstSel_C != 0) {
    LATCtrl_B.Switch2_k = LATC_agSteerOfst_C;
  } else {
    if (0.0 != 0.0) {
      /* Switch: '<S63>/Switch1' incorporates:
       *  Constant: '<S19>/Constant1'
       */
      LATCtrl_B.Switch1_i = 0.0;
    } else {
      /* Switch: '<S63>/Switch1' */
      LATCtrl_B.Switch1_i = LATC_agSteerZeroOfst;
    }

    LATCtrl_B.Switch2_k = LATCtrl_B.Switch1_i;
  }

  /* End of Switch: '<S63>/Switch2' */

  /* Sum: '<S63>/Add1' */
  LATC_agTarSteerReq = LATC_agTarSteer4RLS + LATCtrl_B.Switch2_k;

  /* RelationalOperator: '<S60>/R' incorporates:
   *  Constant: '<S60>/LKA_ACTVCTRL'
   */
  LATCtrl_B.R_c = (LATCtrl_B.DataTypeConversion14 == 1.0);

  /* UnitDelay: '<S60>/D2' */
  LATCtrl_B.D2 = LATCtrl_DW.D2_DSTATE;

  /* Logic: '<S60>/Logical Operator2' */
  LATCtrl_B.LogicalOperator2_f = !LATCtrl_B.D2;

  /* Logic: '<S60>/Logical Operator1' */
  LATCtrl_B.LogicalOperator1 = (LATCtrl_B.R_c && LATCtrl_B.LogicalOperator2_f);

  /* UnitDelay: '<S58>/D1' */
  LATCtrl_B.D1 = LATCtrl_DW.D1_DSTATE;

  /* Switch: '<S58>/Switch' */
  if (LATCtrl_B.LogicalOperator1) {
    LATCtrl_B.Switch_h = LATC_phiSteerAngle;
  } else {
    LATCtrl_B.Switch_h = LATCtrl_B.D1;
  }

  /* End of Switch: '<S58>/Switch' */

  /* Sum: '<S58>/Add' */
  LATCtrl_B.Add = LATC_agTarSteerReq - LATCtrl_B.Switch_h;

  /* UnitDelay: '<S61>/D1' */
  LATCtrl_B.D1_j = LATCtrl_DW.D1_DSTATE_i;

  /* Switch: '<S61>/Switch' incorporates:
   *  Constant: '<S61>/LKA_TimeInit'
   */
  if (LATCtrl_B.LogicalOperator1) {
    LATCtrl_B.Switch_p = 0.0;
  } else {
    /* Sum: '<S61>/Add' incorporates:
     *  Constant: '<S61>/LKA_CycleTime_s'
     */
    LATCtrl_B.Add_p = 0.01 + LATCtrl_B.D1_j;
    LATCtrl_B.Switch_p = LATCtrl_B.Add_p;
  }

  /* End of Switch: '<S61>/Switch' */

  /* MinMax: '<S61>/MinMax' incorporates:
   *  Constant: '<S58>/LKA_ACRampTime'
   */
  u_0 = LATCtrl_B.Switch_p;
  maxval = 2.0;
  if (u_0 <= maxval) {
    maxval = u_0;
  }

  LATCtrl_B.MinMax_b = maxval;

  /* End of MinMax: '<S61>/MinMax' */

  /* Product: '<S58>/Divide' incorporates:
   *  Constant: '<S58>/LKA_ACRampTime'
   *  Constant: '<S58>/LKA_FullPercent'
   */
  LATCtrl_B.Divide_pk = LATCtrl_B.MinMax_b * 100.0 / 2.0;

  /* Saturate: '<S58>/Saturation' */
  u_0 = LATCtrl_B.Divide_pk;
  v_1 = 0.0;
  u_1 = 100.0;
  if (u_0 >= u_1) {
    LATCtrl_B.Saturation_c = u_1;
  } else if (u_0 <= v_1) {
    LATCtrl_B.Saturation_c = v_1;
  } else {
    LATCtrl_B.Saturation_c = u_0;
  }

  /* End of Saturate: '<S58>/Saturation' */

  /* Product: '<S58>/Divide1' incorporates:
   *  Constant: '<S58>/LKA_FullPercent1'
   */
  LATCtrl_B.Divide1_m = LATCtrl_B.Add * LATCtrl_B.Saturation_c / 100.0;

  /* Sum: '<S58>/Add1' */
  LATC_agTarSteerReqd = LATCtrl_B.Switch_h + LATCtrl_B.Divide1_m;

  /* Product: '<S62>/Product2' */
  LATC_agFFSteerDeg = (real_T)LATCtrl_ConstB.Gain * 0.0009765625 *
    LATCtrl_B.Product4_m;

  /* Product: '<S62>/Product3' */
  LATC_agFBSteerDeg = (real_T)LATCtrl_ConstB.Gain * 0.0009765625 *
    LATC_agFrTyreFB;

  /* RelationalOperator: '<S64>/R2' incorporates:
   *  Constant: '<S64>/LKA_ACTVCTRL'
   */
  LATCtrl_B.R2_e = ((real_T)LATC_stOperMod == 3.0);

  /* RelationalOperator: '<S65>/R' incorporates:
   *  Constant: '<S59>/Up_Thd'
   */
  LATCtrl_B.R_hs = (LATC_agTarSteerDeg > LATC_agSteer4StdyUpThd_C);

  /* Switch: '<S65>/Swt' */
  if (LATCtrl_B.R_hs) {
    /* Sum: '<S65>/A1' incorporates:
     *  Constant: '<S59>/Up_Thd'
     */
    LATCtrl_B.A1_k = LATC_agTarSteerDeg - LATC_agSteer4StdyUpThd_C;
    LATC_agTarSteer4Post = LATCtrl_B.A1_k;
  } else {
    /* RelationalOperator: '<S65>/R1' incorporates:
     *  Constant: '<S59>/Lo_Thd'
     */
    LATCtrl_B.R1_jl = (LATC_agTarSteerDeg < LATC_agSteer4StdyLoThd_C);

    /* Switch: '<S65>/Swt1' incorporates:
     *  Constant: '<S65>/Zr_Ag'
     */
    if (LATCtrl_B.R1_jl) {
      /* Sum: '<S65>/A2' incorporates:
       *  Constant: '<S59>/Lo_Thd'
       */
      LATCtrl_B.A2 = LATC_agTarSteerDeg - LATC_agSteer4StdyLoThd_C;
      LATCtrl_B.Swt1_f = LATCtrl_B.A2;
    } else {
      LATCtrl_B.Swt1_f = 0.0;
    }

    /* End of Switch: '<S65>/Swt1' */
    LATC_agTarSteer4Post = LATCtrl_B.Swt1_f;
  }

  /* End of Switch: '<S65>/Swt' */

  /* Product: '<S5>/Product2' incorporates:
   *  Constant: '<S5>/LATC_facLaneSign'
   */
  LATC_agHead4Cmr = LATCtrl_B.Divide5_d * (real_T)LATC_facLaneSign_C;

  /* DataTypeConversion: '<Root>/Data Type Conversion6' incorporates:
   *  Inport: '<Root>/DE_aLatAcc'
   */
  LATCtrl_B.DataTypeConversion6 = DE_aLatAcc;

  /* Sum: '<Root>/Add7' incorporates:
   *  Constant: '<Root>/LATC_aLatAccOfst_C'
   */
  LATC_aLatAcc = LATCtrl_B.DataTypeConversion6 + 0.0;

  /* RelationalOperator: '<S2>/Relational Operator' incorporates:
   *  Constant: '<S2>/OperMod'
   */
  LATCtrl_B.RelationalOperator = ((real_T)LATC_stOperMod == 3.0);

  /* Outputs for Atomic SubSystem: '<Root>/CAN_HPS' */
  /* Switch: '<S1>/Switch' incorporates:
   *  Constant: '<S1>/SAR_C'
   *  Constant: '<S1>/SAR_swt'
   */
  if (LATC_swtSAR_C != 0) {
    LATCtrl_B.Switch_j = LATC_agSAR_C;
  } else {
    LATCtrl_B.Switch_j = LATC_agTarSteerReqd;
  }

  /* End of Switch: '<S1>/Switch' */

  /* DataTypeConversion: '<S1>/Data Type Conversion1' */
  VMC_phiTarAgSteer = LATCtrl_B.Switch_j;

  /* Switch: '<S1>/Switch1' incorporates:
   *  Constant: '<Root>/Tq'
   *  Constant: '<S1>/TR_C'
   *  Constant: '<S1>/TR_swt'
   */
  if (LATC_swtTR_C != 0) {
    LATCtrl_B.Switch1_a = LATC_tqTR_C;
  } else {
    LATCtrl_B.Switch1_a = 0.0;
  }

  /* End of Switch: '<S1>/Switch1' */

  /* DataTypeConversion: '<S1>/Data Type Conversion2' */
  LATCtrl_B.DataTypeConversion2 = LATCtrl_B.Switch1_a;

  /* Switch: '<S1>/Switch2' incorporates:
   *  Constant: '<S1>/CR_C'
   *  Constant: '<S1>/CR_swt'
   *  Switch: '<S2>/Switch'
   */
  if (LATC_swtCR_C != 0) {
    LATCtrl_B.Switch2_mw = (real_T)LATC_stCR_C;
  } else {
    if (LATCtrl_B.RelationalOperator) {
      /* Switch: '<S2>/Switch' incorporates:
       *  Constant: '<S2>/OperMod1'
       */
      LATCtrl_B.Switch_jw = 1.0;
    } else {
      /* Switch: '<S2>/Switch' incorporates:
       *  Constant: '<S2>/OperMod2'
       */
      LATCtrl_B.Switch_jw = 0.0;
    }

    LATCtrl_B.Switch2_mw = LATCtrl_B.Switch_jw;
  }

  /* End of Switch: '<S1>/Switch2' */

  /* DataTypeConversion: '<S1>/Data Type Conversion3' */
  tmp_5 = fmod(floor(LATCtrl_B.Switch2_mw), 256.0);
  VMC_CtrlReqd = (uint8_T)(tmp_5 < 0.0 ? (int32_T)(uint8_T)-(int8_T)(uint8_T)
    -tmp_5 : (int32_T)(uint8_T)tmp_5);

  /* Switch: '<S1>/Switch3' incorporates:
   *  Constant: '<S1>/CM_C'
   *  Constant: '<S1>/CM_swt'
   *  Switch: '<S2>/Switch1'
   */
  if (LATC_swtCM_C != 0) {
    LATCtrl_B.Switch3_d5 = (real_T)LATC_stCM_C;
  } else {
    if (LATCtrl_B.RelationalOperator) {
      /* Switch: '<S2>/Switch1' incorporates:
       *  Constant: '<S2>/OperMod4'
       */
      LATCtrl_B.Switch1_g5 = 3.0;
    } else {
      /* Switch: '<S2>/Switch1' incorporates:
       *  Constant: '<S2>/OperMod5'
       */
      LATCtrl_B.Switch1_g5 = 0.0;
    }

    LATCtrl_B.Switch3_d5 = LATCtrl_B.Switch1_g5;
  }

  /* End of Switch: '<S1>/Switch3' */

  /* DataTypeConversion: '<S1>/Data Type Conversion4' */
  tmp_5 = fmod(floor(LATCtrl_B.Switch3_d5), 256.0);
  VMC_CtrlMode = (uint8_T)(tmp_5 < 0.0 ? (int32_T)(uint8_T)-(int8_T)(uint8_T)
    -tmp_5 : (int32_T)(uint8_T)tmp_5);

  /* Outputs for Atomic SubSystem: '<S1>/Exc_10' */
  /* DataTypeConversion: '<S7>/Data Type Conversion13' */
  tmp_5 = fmod(floor(LATCtrl_B.DataTypeConversion2 / 0.01), 65536.0);
  LATCtrl_B.DataTypeConversion13 = (int16_T)(tmp_5 < 0.0 ? (int32_T)(int16_T)
    -(int16_T)(uint16_T)-tmp_5 : (int32_T)(int16_T)(uint16_T)tmp_5);

  /* DataTypeConversion: '<S7>/Data Type Conversion14' */
  tmp_5 = fmod(floor(VMC_phiTarAgSteer / 0.1), 4.294967296E+9);
  LATCtrl_B.DataTypeConversion14_f = tmp_5 < 0.0 ? -(int32_T)(uint32_T)-tmp_5 :
    (int32_T)(uint32_T)tmp_5;

  /* UnitDelay: '<S8>/Unit Delay' */
  LATCtrl_B.UnitDelay_n = LATCtrl_DW.UnitDelay_DSTATE_j;

  /* Sum: '<S8>/Sum6' incorporates:
   *  Constant: '<S8>/C2'
   */
  LATCtrl_B.Sum6_d = (uint8_T)((uint32_T)((uint8_T)1U) + LATCtrl_B.UnitDelay_n);

  /* Sum: '<S8>/Add2' incorporates:
   *  Constant: '<S8>/ExC1_C3'
   */
  LATCtrl_B.Add2_o = (uint8_T)((uint32_T)((uint8_T)0U) + LATCtrl_B.Sum6_d);

  /* S-Function (sfix_bitop): '<S8>/Bitwise Operator' */
  LATCtrl_B.BitwiseOperator = (uint8_T)(LATCtrl_B.Add2_o & ((uint8_T)255U));

  /* DataTypeConversion: '<S8>/Data Type Conversion6' */
  LATCtrl_B.lkas6 = LATCtrl_B.BitwiseOperator;

  /* DataTypeConversion: '<S7>/Data Type Conversion2' */
  ExC_MessageCntr = (real_T)LATCtrl_B.lkas6;

  /* DataTypeConversion: '<S8>/Data Type Conversion7' */
  LATCtrl_B.DataTypeConversion7_d = LATCtrl_B.DataTypeConversion13;

  /* Product: '<S8>/Product' incorporates:
   *  Constant: '<S8>/Exc1_trqReqdFactor_C'
   */
  tmp_5 = fmod(floor((real_T)LATCtrl_B.DataTypeConversion7_d * 0.01 * 100.0),
               65536.0);
  LATCtrl_B.Product_g = (int16_T)(tmp_5 < 0.0 ? (int32_T)(int16_T)-(int16_T)
    (uint16_T)-tmp_5 : (int32_T)(int16_T)(uint16_T)tmp_5);

  /* DataTypeConversion: '<S8>/Data Type Conversion9' */
  LATCtrl_B.DataTypeConversion9 = (uint16_T)LATCtrl_B.Product_g;

  /* S-Function (sfix_bitop): '<S8>/Bitwise Operator1' */
  LATCtrl_B.Low = (uint16_T)(LATCtrl_B.DataTypeConversion9 & ((uint16_T)255U));

  /* DataTypeConversion: '<S8>/Data Type Conversion' */
  LATCtrl_B.lkas0 = (uint8_T)LATCtrl_B.Low;

  /* S-Function (sfix_bitop): '<S8>/Bitwise Operator2' */
  LATCtrl_B.Low_i = (uint16_T)(LATCtrl_B.DataTypeConversion9 & ((uint16_T)65280U));

  /* ArithShift: '<S8>/Shift Arithmetic' */
  LATCtrl_B.High = (uint16_T)(LATCtrl_B.Low_i >> 8);

  /* DataTypeConversion: '<S8>/Data Type Conversion1' */
  LATCtrl_B.lkas1 = (uint8_T)LATCtrl_B.High;

  /* DataTypeConversion: '<S8>/Data Type Conversion14' */
  LATCtrl_B.DataTypeConversion14_b = LATCtrl_B.DataTypeConversion14_f;

  /* Product: '<S8>/Product2' incorporates:
   *  Constant: '<S8>/Exc1_trqReqdFactor_C2'
   */
  u_1 = (real_T)LATCtrl_B.DataTypeConversion14_b * 0.1 * 10.0;
  v_1 = fabs(u_1);
  if (v_1 < 4.503599627370496E+15) {
    if (v_1 >= 0.5) {
      u_1 = floor(u_1 + 0.5);
    } else {
      u_1 = 0.0;
    }
  }

  tmp_5 = fmod(u_1, 65536.0);
  LATCtrl_B.Product2_f = (int16_T)(tmp_5 < 0.0 ? (int32_T)(int16_T)-(int16_T)
    (uint16_T)-tmp_5 : (int32_T)(int16_T)(uint16_T)tmp_5);

  /* End of Product: '<S8>/Product2' */

  /* DataTypeConversion: '<S8>/Data Type Conversion8' */
  LATCtrl_B.DataTypeConversion8 = (uint16_T)LATCtrl_B.Product2_f;

  /* S-Function (sfix_bitop): '<S8>/Bitwise Operator3' */
  LATCtrl_B.Low_k = (uint16_T)(LATCtrl_B.DataTypeConversion8 & ((uint16_T)255U));

  /* DataTypeConversion: '<S8>/Data Type Conversion2' */
  LATCtrl_B.lkas2 = (uint8_T)LATCtrl_B.Low_k;

  /* S-Function (sfix_bitop): '<S8>/Bitwise Operator4' */
  LATCtrl_B.Low_d = (uint16_T)(LATCtrl_B.DataTypeConversion8 & ((uint16_T)65280U));

  /* ArithShift: '<S8>/Shift Arithmetic1' */
  LATCtrl_B.High_p = (uint16_T)(LATCtrl_B.Low_d >> 8);

  /* DataTypeConversion: '<S8>/Data Type Conversion3' */
  LATCtrl_B.lkas3 = (uint8_T)LATCtrl_B.High_p;

  /* DataTypeConversion: '<S8>/Data Type Conversion4' */
  LATCtrl_B.lkas4 = VMC_CtrlReqd;

  /* DataTypeConversion: '<S8>/Data Type Conversion5' */
  LATCtrl_B.lkas5 = VMC_CtrlMode;

  /* Sum: '<S9>/Add2' */
  LATCtrl_B.Add2_p = (uint8_T)(((((((uint32_T)LATCtrl_B.lkas0 + LATCtrl_B.lkas1)
    + LATCtrl_B.lkas2) + LATCtrl_B.lkas3) + LATCtrl_B.lkas4) + LATCtrl_B.lkas5)
    + LATCtrl_B.lkas6);

  /* S-Function (sfix_bitop): '<S9>/Bitwise Operator' */
  LATCtrl_B.BitwiseOperator_a = (uint8_T)(LATCtrl_B.Add2_p & ((uint8_T)255U));

  /* DataTypeConversion: '<S9>/Data Type Conversion6' */
  LATCtrl_B.DataTypeConversion6_f = LATCtrl_B.BitwiseOperator_a;

  /* DataTypeConversion: '<S7>/Data Type Conversion3' */
  ExC_CheckSum = (real_T)LATCtrl_B.DataTypeConversion6_f;

  /* BusCreator: '<S8>/Bus Creator1' */
  LKAS_0413.lkas0 = LATCtrl_B.lkas0;
  LKAS_0413.lkas1 = LATCtrl_B.lkas1;
  LKAS_0413.lkas2 = LATCtrl_B.lkas2;
  LKAS_0413.lkas3 = LATCtrl_B.lkas3;
  LKAS_0413.lkas4 = LATCtrl_B.lkas4;
  LKAS_0413.lkas5 = LATCtrl_B.lkas5;
  LKAS_0413.lkas6 = LATCtrl_B.lkas6;
  LKAS_0413.lkas7 = LATCtrl_B.DataTypeConversion6_f;

  /* Update for UnitDelay: '<S8>/Unit Delay' */
  LATCtrl_DW.UnitDelay_DSTATE_j = LATCtrl_B.BitwiseOperator;

  /* End of Outputs for SubSystem: '<S1>/Exc_10' */

  /* BusCreator: '<S1>/Bus Creator1' incorporates:
   *  Constant: '<S1>/IG_State'
   *  Constant: '<S1>/ZeroForMSG'
   */
  EHPS_0412.ehps0 = ((uint8_T)1U);
  EHPS_0412.ehps1 = ((uint8_T)0U);
  EHPS_0412.ehps2 = ((uint8_T)0U);
  EHPS_0412.ehps3 = ((uint8_T)0U);
  EHPS_0412.ehps4 = ((uint8_T)0U);
  EHPS_0412.ehps5 = ((uint8_T)0U);
  EHPS_0412.ehps6 = ((uint8_T)0U);
  EHPS_0412.ehps7 = ((uint8_T)0U);

  /* End of Outputs for SubSystem: '<Root>/CAN_HPS' */

  /* DataTypeConversion: '<Root>/Data Type Conversion3' incorporates:
   *  Inport: '<Root>/VMP_cCv4TgtPath'
   */
  LATC_cCv4TgtPath = VMP_cCv4TgtPath;

  /* Abs: '<S12>/Abs8' */
  LATCtrl_B.Abs8 = fabs(LATC_lLaneDist0);

  /* Abs: '<S12>/Abs9' */
  LATCtrl_B.Abs9 = fabs(LATC_lLaneDist1);

  /* Sum: '<S12>/Add' */
  LATC_lLaneWidthRaw = LATCtrl_B.Abs8 + LATCtrl_B.Abs9;

  /* DataTypeConversion: '<Root>/Data Type Conversion7' incorporates:
   *  Inport: '<Root>/DE_aLonAcc'
   */
  LATCtrl_B.DataTypeConversion7 = DE_aLonAcc;

  /* Sum: '<Root>/Add1' incorporates:
   *  Constant: '<Root>/LATC_aLonAccOfst_C'
   */
  LATC_aLonAcc = LATCtrl_B.DataTypeConversion7 + 0.0;

  /* DataTypeConversion: '<Root>/Data Type Conversion12' incorporates:
   *  Inport: '<Root>/DE_aVertAcc'
   */
  LATCtrl_B.DataTypeConversion12 = DE_aVertAcc;

  /* Sum: '<Root>/Add2' incorporates:
   *  Constant: '<Root>/LATC_aVertAccOfst_C'
   */
  LATC_aVertAcc = LATCtrl_B.DataTypeConversion12 + 0.0;

  /* Update for UnitDelay: '<S39>/Unit Delay1' */
  LATCtrl_DW.UnitDelay1_DSTATE = LATCtrl_B.Product6_j;

  /* Update for UnitDelay: '<S39>/Unit Delay' */
  LATCtrl_DW.UnitDelay_DSTATE_i = LATCtrl_B.MathFunction_g;

  /* Update for UnitDelay: '<S92>/D' */
  LATCtrl_DW.D_DSTATE = LatDist4LQR;

  /* Update for Delay: '<S12>/Delay' */
  for (rtemp = 0; rtemp < 19; rtemp++) {
    LATCtrl_DW.Delay_DSTATE[rtemp] = LATCtrl_DW.Delay_DSTATE[rtemp + 1];
  }

  LATCtrl_DW.Delay_DSTATE[19] = LATC_lLaneWidthRaw;

  /* End of Update for Delay: '<S12>/Delay' */

  /* Update for UnitDelay: '<S12>/Unit Delay' */
  LATCtrl_DW.UnitDelay_DSTATE = LATC_lLaneDist0;

  /* Update for Memory: '<S14>/Memory' */
  LATCtrl_DW.Memory_PreviousInput = LATCtrl_B.Logic[0];

  /* Update for UnitDelay: '<S12>/Unit Delay1' */
  LATCtrl_DW.UnitDelay1_DSTATE_j = LATC_lLaneDist1;

  /* Update for Memory: '<S15>/Memory' */
  LATCtrl_DW.Memory_PreviousInput_j = LATCtrl_B.Logic_n[0];

  /* Update for UnitDelay: '<S12>/Unit Delay6' */
  LATCtrl_DW.UnitDelay6_DSTATE_o = LATCtrl_B.LO13;

  /* Update for UnitDelay: '<S12>/Unit Delay2' */
  LATCtrl_DW.UnitDelay2_DSTATE = LATC_lLaneWidth4Loss;

  /* Update for UnitDelay: '<S93>/D' */
  LATCtrl_DW.D_DSTATE_i = LatHeading4LQR;

  /* Update for Delay: '<S30>/Variable Integer Delay3' */
  LATCtrl_DW.icLoad = 0U;
  LATCtrl_DW.VariableIntegerDelay3_DSTATE[0] = LATCtrl_B.xhat[0];
  LATCtrl_DW.VariableIntegerDelay3_DSTATE[1] = LATCtrl_B.xhat[1];
  LATCtrl_DW.VariableIntegerDelay3_DSTATE[2] = LATCtrl_B.xhat[2];

  /* Update for UnitDelay: '<S30>/Unit Delay' incorporates:
   *  Constant: '<S30>/CycleTime4Kalman1'
   */
  LATCtrl_DW.UnitDelay_DSTATE_b = 1.0;

  /* Update for UnitDelay: '<S94>/D' */
  LATCtrl_DW.D_DSTATE_p = LCC_lPrevCv4LQR;

  /* Update for UnitDelay: '<S33>/Unit Delay1' */
  LATCtrl_DW.UnitDelay1_DSTATE_i = LATCtrl_B.MathFunction_i;

  /* Update for Delay: '<S30>/Variable Integer Delay' */
  LATCtrl_DW.icLoad_p = 0U;
  memcpy(&LATCtrl_DW.VariableIntegerDelay_DSTATE[0], &LATCtrl_B.Pk2[0], 9U *
         sizeof(real_T));

  /* Update for UnitDelay: '<S78>/D' */
  LATCtrl_DW.D_DSTATE_l = LATCtrl_B.Switch14;

  /* Update for UnitDelay: '<S84>/Unit Delay' */
  LATCtrl_DW.UnitDelay_DSTATE_p = LATCtrl_B.MathFunction_iq;

  /* Update for Memory: '<S85>/Memory' */
  LATCtrl_DW.Memory_PreviousInput_d = LATCtrl_B.Logic_a[0];

  /* Update for UnitDelay: '<S77>/D' */
  LATCtrl_DW.D_DSTATE_k = LATCtrl_B.Switch14_k;

  /* Update for Delay: '<S72>/Delay1' */
  LATCtrl_DW.Delay1_DSTATE = LATCtrl_B.Switch1_f3;

  /* Update for Delay: '<S72>/Delay2' */
  LATCtrl_DW.Delay2_DSTATE[0] = LATCtrl_DW.Delay2_DSTATE[1];
  LATCtrl_DW.Delay2_DSTATE[1] = LATCtrl_B.Switch1_f3;

  /* Update for Delay: '<S72>/Delay3' */
  LATCtrl_DW.Delay3_DSTATE[0] = LATCtrl_DW.Delay3_DSTATE[1];
  LATCtrl_DW.Delay3_DSTATE[1] = LATCtrl_DW.Delay3_DSTATE[2];
  LATCtrl_DW.Delay3_DSTATE[2] = LATCtrl_B.Switch1_f3;

  /* Update for Delay: '<S72>/Delay4' */
  LATCtrl_DW.Delay4_DSTATE[0] = LATCtrl_DW.Delay4_DSTATE[1];
  LATCtrl_DW.Delay4_DSTATE[1] = LATCtrl_DW.Delay4_DSTATE[2];
  LATCtrl_DW.Delay4_DSTATE[2] = LATCtrl_DW.Delay4_DSTATE[3];
  LATCtrl_DW.Delay4_DSTATE[3] = LATCtrl_B.Switch1_f3;

  /* Update for Delay: '<S72>/Delay5' */
  LATCtrl_DW.Delay5_DSTATE[0] = LATCtrl_DW.Delay5_DSTATE[1];
  LATCtrl_DW.Delay5_DSTATE[1] = LATCtrl_DW.Delay5_DSTATE[2];
  LATCtrl_DW.Delay5_DSTATE[2] = LATCtrl_DW.Delay5_DSTATE[3];
  LATCtrl_DW.Delay5_DSTATE[3] = LATCtrl_DW.Delay5_DSTATE[4];
  LATCtrl_DW.Delay5_DSTATE[4] = LATCtrl_B.Switch1_f3;

  /* Update for Delay: '<S72>/Delay6' */
  for (rtemp = 0; rtemp < 5; rtemp++) {
    LATCtrl_DW.Delay6_DSTATE[rtemp] = LATCtrl_DW.Delay6_DSTATE[rtemp + 1];
  }

  LATCtrl_DW.Delay6_DSTATE[5] = LATCtrl_B.Switch1_f3;

  /* End of Update for Delay: '<S72>/Delay6' */

  /* Update for Delay: '<S72>/Delay7' */
  for (rtemp = 0; rtemp < 6; rtemp++) {
    LATCtrl_DW.Delay7_DSTATE[rtemp] = LATCtrl_DW.Delay7_DSTATE[rtemp + 1];
  }

  LATCtrl_DW.Delay7_DSTATE[6] = LATCtrl_B.Switch1_f3;

  /* End of Update for Delay: '<S72>/Delay7' */

  /* Update for Delay: '<S72>/Delay8' */
  for (rtemp = 0; rtemp < 7; rtemp++) {
    LATCtrl_DW.Delay8_DSTATE[rtemp] = LATCtrl_DW.Delay8_DSTATE[rtemp + 1];
  }

  LATCtrl_DW.Delay8_DSTATE[7] = LATCtrl_B.Switch1_f3;

  /* End of Update for Delay: '<S72>/Delay8' */

  /* Update for Delay: '<S72>/Delay9' */
  for (rtemp = 0; rtemp < 8; rtemp++) {
    LATCtrl_DW.Delay9_DSTATE[rtemp] = LATCtrl_DW.Delay9_DSTATE[rtemp + 1];
  }

  LATCtrl_DW.Delay9_DSTATE[8] = LATCtrl_B.Switch1_f3;

  /* End of Update for Delay: '<S72>/Delay9' */

  /* Update for Delay: '<S23>/Delay1' */
  for (rtemp = 0; rtemp < 99; rtemp++) {
    LATCtrl_DW.Delay1_DSTATE_c[rtemp] = LATCtrl_DW.Delay1_DSTATE_c[rtemp + 1];
  }

  LATCtrl_DW.Delay1_DSTATE_c[99] = LATCtrl_B.Switch14;

  /* End of Update for Delay: '<S23>/Delay1' */

  /* Update for UnitDelay: '<S91>/Unit Delay' */
  LATCtrl_DW.UnitDelay_DSTATE_cs = LATC_stSteerAvrgComplt;

  /* Update for UnitDelay: '<S89>/Unit Delay' */
  LATCtrl_DW.UnitDelay_DSTATE_m = LATC_agSteerZeroOfst;

  /* Update for UnitDelay: '<S74>/Unit Delay' */
  LATCtrl_DW.UnitDelay_DSTATE_c = LATCtrl_B.MathFunction_mz;

  /* Update for Delay: '<S82>/Delay1' */
  LATCtrl_DW.Delay1_DSTATE_m = LATC_wYRErr4LQR;

  /* Update for Delay: '<S82>/Delay2' */
  LATCtrl_DW.Delay2_DSTATE_g[0] = LATCtrl_DW.Delay2_DSTATE_g[1];
  LATCtrl_DW.Delay2_DSTATE_g[1] = LATC_wYRErr4LQR;

  /* Update for Delay: '<S82>/Delay3' */
  LATCtrl_DW.Delay3_DSTATE_f[0] = LATCtrl_DW.Delay3_DSTATE_f[1];
  LATCtrl_DW.Delay3_DSTATE_f[1] = LATCtrl_DW.Delay3_DSTATE_f[2];
  LATCtrl_DW.Delay3_DSTATE_f[2] = LATC_wYRErr4LQR;

  /* Update for Delay: '<S82>/Delay4' */
  LATCtrl_DW.Delay4_DSTATE_n[0] = LATCtrl_DW.Delay4_DSTATE_n[1];
  LATCtrl_DW.Delay4_DSTATE_n[1] = LATCtrl_DW.Delay4_DSTATE_n[2];
  LATCtrl_DW.Delay4_DSTATE_n[2] = LATCtrl_DW.Delay4_DSTATE_n[3];
  LATCtrl_DW.Delay4_DSTATE_n[3] = LATC_wYRErr4LQR;

  /* Update for Delay: '<S82>/Delay5' */
  LATCtrl_DW.Delay5_DSTATE_c[0] = LATCtrl_DW.Delay5_DSTATE_c[1];
  LATCtrl_DW.Delay5_DSTATE_c[1] = LATCtrl_DW.Delay5_DSTATE_c[2];
  LATCtrl_DW.Delay5_DSTATE_c[2] = LATCtrl_DW.Delay5_DSTATE_c[3];
  LATCtrl_DW.Delay5_DSTATE_c[3] = LATCtrl_DW.Delay5_DSTATE_c[4];
  LATCtrl_DW.Delay5_DSTATE_c[4] = LATC_wYRErr4LQR;

  /* Update for Delay: '<S82>/Delay6' */
  for (rtemp = 0; rtemp < 5; rtemp++) {
    LATCtrl_DW.Delay6_DSTATE_k[rtemp] = LATCtrl_DW.Delay6_DSTATE_k[rtemp + 1];
  }

  LATCtrl_DW.Delay6_DSTATE_k[5] = LATC_wYRErr4LQR;

  /* End of Update for Delay: '<S82>/Delay6' */

  /* Update for Delay: '<S82>/Delay7' */
  for (rtemp = 0; rtemp < 6; rtemp++) {
    LATCtrl_DW.Delay7_DSTATE_l[rtemp] = LATCtrl_DW.Delay7_DSTATE_l[rtemp + 1];
  }

  LATCtrl_DW.Delay7_DSTATE_l[6] = LATC_wYRErr4LQR;

  /* End of Update for Delay: '<S82>/Delay7' */

  /* Update for Delay: '<S82>/Delay8' */
  for (rtemp = 0; rtemp < 7; rtemp++) {
    LATCtrl_DW.Delay8_DSTATE_m[rtemp] = LATCtrl_DW.Delay8_DSTATE_m[rtemp + 1];
  }

  LATCtrl_DW.Delay8_DSTATE_m[7] = LATC_wYRErr4LQR;

  /* End of Update for Delay: '<S82>/Delay8' */

  /* Update for Delay: '<S82>/Delay9' */
  for (rtemp = 0; rtemp < 8; rtemp++) {
    LATCtrl_DW.Delay9_DSTATE_h[rtemp] = LATCtrl_DW.Delay9_DSTATE_h[rtemp + 1];
  }

  LATCtrl_DW.Delay9_DSTATE_h[8] = LATC_wYRErr4LQR;

  /* End of Update for Delay: '<S82>/Delay9' */

  /* Update for Delay: '<S69>/Delay1' */
  LATCtrl_DW.Delay1_DSTATE_f = LATCtrl_B.Switch1_dv;

  /* Update for Delay: '<S69>/Delay2' */
  LATCtrl_DW.Delay2_DSTATE_b[0] = LATCtrl_DW.Delay2_DSTATE_b[1];
  LATCtrl_DW.Delay2_DSTATE_b[1] = LATCtrl_B.Switch1_dv;

  /* Update for Delay: '<S69>/Delay3' */
  LATCtrl_DW.Delay3_DSTATE_d[0] = LATCtrl_DW.Delay3_DSTATE_d[1];
  LATCtrl_DW.Delay3_DSTATE_d[1] = LATCtrl_DW.Delay3_DSTATE_d[2];
  LATCtrl_DW.Delay3_DSTATE_d[2] = LATCtrl_B.Switch1_dv;

  /* Update for Delay: '<S69>/Delay4' */
  LATCtrl_DW.Delay4_DSTATE_f[0] = LATCtrl_DW.Delay4_DSTATE_f[1];
  LATCtrl_DW.Delay4_DSTATE_f[1] = LATCtrl_DW.Delay4_DSTATE_f[2];
  LATCtrl_DW.Delay4_DSTATE_f[2] = LATCtrl_DW.Delay4_DSTATE_f[3];
  LATCtrl_DW.Delay4_DSTATE_f[3] = LATCtrl_B.Switch1_dv;

  /* Update for Delay: '<S69>/Delay5' */
  LATCtrl_DW.Delay5_DSTATE_cr[0] = LATCtrl_DW.Delay5_DSTATE_cr[1];
  LATCtrl_DW.Delay5_DSTATE_cr[1] = LATCtrl_DW.Delay5_DSTATE_cr[2];
  LATCtrl_DW.Delay5_DSTATE_cr[2] = LATCtrl_DW.Delay5_DSTATE_cr[3];
  LATCtrl_DW.Delay5_DSTATE_cr[3] = LATCtrl_DW.Delay5_DSTATE_cr[4];
  LATCtrl_DW.Delay5_DSTATE_cr[4] = LATCtrl_B.Switch1_dv;

  /* Update for Delay: '<S69>/Delay6' */
  for (rtemp = 0; rtemp < 5; rtemp++) {
    LATCtrl_DW.Delay6_DSTATE_i[rtemp] = LATCtrl_DW.Delay6_DSTATE_i[rtemp + 1];
  }

  LATCtrl_DW.Delay6_DSTATE_i[5] = LATCtrl_B.Switch1_dv;

  /* End of Update for Delay: '<S69>/Delay6' */

  /* Update for Delay: '<S69>/Delay7' */
  for (rtemp = 0; rtemp < 6; rtemp++) {
    LATCtrl_DW.Delay7_DSTATE_c[rtemp] = LATCtrl_DW.Delay7_DSTATE_c[rtemp + 1];
  }

  LATCtrl_DW.Delay7_DSTATE_c[6] = LATCtrl_B.Switch1_dv;

  /* End of Update for Delay: '<S69>/Delay7' */

  /* Update for Delay: '<S69>/Delay8' */
  for (rtemp = 0; rtemp < 7; rtemp++) {
    LATCtrl_DW.Delay8_DSTATE_f[rtemp] = LATCtrl_DW.Delay8_DSTATE_f[rtemp + 1];
  }

  LATCtrl_DW.Delay8_DSTATE_f[7] = LATCtrl_B.Switch1_dv;

  /* End of Update for Delay: '<S69>/Delay8' */

  /* Update for Delay: '<S69>/Delay9' */
  for (rtemp = 0; rtemp < 8; rtemp++) {
    LATCtrl_DW.Delay9_DSTATE_m[rtemp] = LATCtrl_DW.Delay9_DSTATE_m[rtemp + 1];
  }

  LATCtrl_DW.Delay9_DSTATE_m[8] = LATCtrl_B.Switch1_dv;

  /* End of Update for Delay: '<S69>/Delay9' */

  /* Update for Delay: '<S69>/Delay10' */
  LATCtrl_DW.Delay10_DSTATE = LATC_vDistErrFlt4LQR;

  /* Update for Delay: '<S69>/Delay11' */
  LATCtrl_DW.Delay11_DSTATE[0] = LATCtrl_DW.Delay11_DSTATE[1];
  LATCtrl_DW.Delay11_DSTATE[1] = LATC_vDistErrFlt4LQR;

  /* Update for Delay: '<S69>/Delay12' */
  LATCtrl_DW.Delay12_DSTATE[0] = LATCtrl_DW.Delay12_DSTATE[1];
  LATCtrl_DW.Delay12_DSTATE[1] = LATCtrl_DW.Delay12_DSTATE[2];
  LATCtrl_DW.Delay12_DSTATE[2] = LATC_vDistErrFlt4LQR;

  /* Update for Delay: '<S69>/Delay13' */
  LATCtrl_DW.Delay13_DSTATE[0] = LATCtrl_DW.Delay13_DSTATE[1];
  LATCtrl_DW.Delay13_DSTATE[1] = LATCtrl_DW.Delay13_DSTATE[2];
  LATCtrl_DW.Delay13_DSTATE[2] = LATCtrl_DW.Delay13_DSTATE[3];
  LATCtrl_DW.Delay13_DSTATE[3] = LATC_vDistErrFlt4LQR;

  /* Update for Delay: '<S69>/Delay14' */
  LATCtrl_DW.Delay14_DSTATE[0] = LATCtrl_DW.Delay14_DSTATE[1];
  LATCtrl_DW.Delay14_DSTATE[1] = LATCtrl_DW.Delay14_DSTATE[2];
  LATCtrl_DW.Delay14_DSTATE[2] = LATCtrl_DW.Delay14_DSTATE[3];
  LATCtrl_DW.Delay14_DSTATE[3] = LATCtrl_DW.Delay14_DSTATE[4];
  LATCtrl_DW.Delay14_DSTATE[4] = LATC_vDistErrFlt4LQR;

  /* Update for Delay: '<S69>/Delay15' */
  for (rtemp = 0; rtemp < 5; rtemp++) {
    LATCtrl_DW.Delay15_DSTATE[rtemp] = LATCtrl_DW.Delay15_DSTATE[rtemp + 1];
  }

  LATCtrl_DW.Delay15_DSTATE[5] = LATC_vDistErrFlt4LQR;

  /* End of Update for Delay: '<S69>/Delay15' */

  /* Update for Delay: '<S69>/Delay16' */
  for (rtemp = 0; rtemp < 6; rtemp++) {
    LATCtrl_DW.Delay16_DSTATE[rtemp] = LATCtrl_DW.Delay16_DSTATE[rtemp + 1];
  }

  LATCtrl_DW.Delay16_DSTATE[6] = LATC_vDistErrFlt4LQR;

  /* End of Update for Delay: '<S69>/Delay16' */

  /* Update for Delay: '<S69>/Delay17' */
  for (rtemp = 0; rtemp < 7; rtemp++) {
    LATCtrl_DW.Delay17_DSTATE[rtemp] = LATCtrl_DW.Delay17_DSTATE[rtemp + 1];
  }

  LATCtrl_DW.Delay17_DSTATE[7] = LATC_vDistErrFlt4LQR;

  /* End of Update for Delay: '<S69>/Delay17' */

  /* Update for Delay: '<S69>/Delay18' */
  for (rtemp = 0; rtemp < 8; rtemp++) {
    LATCtrl_DW.Delay18_DSTATE[rtemp] = LATCtrl_DW.Delay18_DSTATE[rtemp + 1];
  }

  LATCtrl_DW.Delay18_DSTATE[8] = LATC_vDistErrFlt4LQR;

  /* End of Update for Delay: '<S69>/Delay18' */

  /* Update for UnitDelay: '<S69>/Unit Delay5' */
  LATCtrl_DW.UnitDelay5_DSTATE = LATC_vLatSpdFlt4LQR;

  /* Update for UnitDelay: '<S69>/Unit Delay6' */
  LATCtrl_DW.UnitDelay6_DSTATE = LATCtrl_B.covOut;

  /* Update for Delay: '<S23>/Variable Integer Delay3' */
  LATCtrl_DW.icLoad_c = 0U;
  memcpy(&LATCtrl_DW.VariableIntegerDelay3_DSTATE_m[0], &LATCtrl_B.P_a[0],
         sizeof(real_T) << 4U);

  /* Update for DiscreteTransferFcn: '<S70>/Discrete Transfer Fcn' */
  LATCtrl_DW.DiscreteTransferFcn_states = LATCtrl_DW.DiscreteTransferFcn_tmp;

  /* Update for DiscreteTransferFcn: '<S70>/Discrete Transfer Fcn2' */
  LATCtrl_DW.DiscreteTransferFcn2_states = LATCtrl_DW.DiscreteTransferFcn2_tmp;

  /* Update for DiscreteTransferFcn: '<S71>/Discrete Transfer Fcn' */
  LATCtrl_DW.DiscreteTransferFcn_states_j = LATCtrl_DW.DiscreteTransferFcn_tmp_b;

  /* Update for DiscreteTransferFcn: '<S71>/Discrete Transfer Fcn2' */
  LATCtrl_DW.DiscreteTransferFcn2_states_l =
    LATCtrl_DW.DiscreteTransferFcn2_tmp_h;

  /* Update for UnitDelay: '<S19>/Unit Delay' incorporates:
   *  Constant: '<S19>/LATC_swtZeroIdfyReset_C'
   */
  LATCtrl_DW.UnitDelay_DSTATE_e = LATC_swtZeroIdfyReset_C;

  /* Update for UnitDelay: '<S64>/D' */
  LATCtrl_DW.D_DSTATE_a = LATCtrl_B.MinMax3;

  /* Update for Delay: '<S64>/Delay' */
  for (rtemp = 0; rtemp < 9; rtemp++) {
    LATCtrl_DW.Delay_DSTATE_n[rtemp] = LATCtrl_DW.Delay_DSTATE_n[rtemp + 1];
  }

  LATCtrl_DW.Delay_DSTATE_n[9] = LATCtrl_B.R2_e;

  /* End of Update for Delay: '<S64>/Delay' */

  /* Update for UnitDelay: '<S60>/D2' */
  LATCtrl_DW.D2_DSTATE = LATCtrl_B.R_c;

  /* Update for UnitDelay: '<S58>/D1' */
  LATCtrl_DW.D1_DSTATE = LATCtrl_B.Switch_h;

  /* Update for UnitDelay: '<S61>/D1' */
  LATCtrl_DW.D1_DSTATE_i = LATCtrl_B.MinMax_b;
}

/* Model initialize function */
void LATCtrl_initialize(void)
{
  /* InitializeConditions for Enabled SubSystem: '<S19>/LATC_ZeroPosition' */
  /* InitializeConditions for Delay: '<S47>/Delay' */
  LATCtrl_DW.Delay_DSTATE_o = 0.0;

  /* InitializeConditions for UnitDelay: '<S48>/delay' */
  LATCtrl_DW.delay_DSTATE[0] = 0.0;
  LATCtrl_DW.delay_DSTATE[1] = 0.02;
  LATCtrl_DW.delay_DSTATE[2] = 0.0;

  /* InitializeConditions for UnitDelay: '<S48>/delay1' */
  memcpy(&LATCtrl_DW.delay1_DSTATE[0], (const real_T*)LATCtrl_ConstP.pooled7, 9U
         * sizeof(real_T));

  /* End of InitializeConditions for SubSystem: '<S19>/LATC_ZeroPosition' */

  /* InitializeConditions for Delay: '<S30>/Variable Integer Delay3' */
  LATCtrl_DW.icLoad = 1U;

  /* InitializeConditions for Delay: '<S30>/Variable Integer Delay' */
  LATCtrl_DW.icLoad_p = 1U;

  /* InitializeConditions for Delay: '<S23>/Variable Integer Delay3' */
  LATCtrl_DW.icLoad_c = 1U;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
