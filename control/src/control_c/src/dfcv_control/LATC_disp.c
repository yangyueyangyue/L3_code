/*
 * File: LATC_disp.c
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

/* Definition for custom storage class: ExportToFile */
BUS_EHPS_0412 EHPS_0412;
real_T ExC_CheckSum;
real_T ExC_MessageCntr;
real_T LATC_Lane0Cv1;
real_T LATC_Lane0Cv2;
real_T LATC_Lane0Cv3;
real_T LATC_Lane1Cv1;
real_T LATC_Lane1Cv2;
real_T LATC_Lane1Cv3;
real_T LATC_aLatAcc;
real_T LATC_aLonAcc;
real_T LATC_aVertAcc;
real_T LATC_agFBSteerDeg;
real_T LATC_agFFSteerDeg;
real_T LATC_agFrTyre4LQR;
real_T LATC_agFrTyreFB;
real_T LATC_agFrTyreFB4Math;
real_T LATC_agFrTyreFBEle1;
real_T LATC_agFrTyreFBEle2;
real_T LATC_agFrTyreFBEle3;
real_T LATC_agFrTyreFBEle4;
real_T LATC_agFrTyreFF;
real_T LATC_agHead4Cmr;
real_T LATC_agHead4TgtPath;
real_T LATC_agHeadFlt4LQR;
real_T LATC_agSteerZeroOfst;
real_T LATC_agTarSteer4Post;
real_T LATC_agTarSteer4RL;
real_T LATC_agTarSteer4RLS;
real_T LATC_agTarSteerDeg;
real_T LATC_agTarSteerRad;
real_T LATC_agTarSteerReq;
real_T LATC_agTarSteerReqd;
real_T LATC_cCv4Cmr;
real_T LATC_cCv4TgtPath;
real_T LATC_cTgtPathCv0;
real_T LATC_cTgtPathCv1;
real_T LATC_cTgtPathCv2;
real_T LATC_cTgtPathCv3;
real_T LATC_cTgtPathCv4;
real_T LATC_cTgtPathCv5;
real_T LATC_eleK1;
real_T LATC_eleK2;
real_T LATC_eleK3;
real_T LATC_eleK4;
real_T LATC_facC0Filt;
real_T LATC_facC0Hat;
real_T LATC_facC1Filt;
real_T LATC_facC1Hat;
real_T LATC_facLeftLaneA0;
real_T LATC_facLeftLaneA1;
real_T LATC_facLeftLaneA2;
real_T LATC_facLeftLaneA3;
real_T LATC_facRgtLaneA0;
real_T LATC_facRgtLaneA1;
real_T LATC_facRgtLaneA2;
real_T LATC_facRgtLaneA3;
real_T LATC_lDist4Cmr;
real_T LATC_lLaneDist0;
real_T LATC_lLaneDist1;
real_T LATC_lLaneWidth4Loss;
real_T LATC_lLaneWidthRaw;
real_T LATC_lLatPosn4TgtPath;
real_T LATC_lLgtDist2PathZero;
real_T LATC_mMass4EBS;
real_T LATC_mVehMass;
real_T LATC_phiSteerAngle;
boolean_T LATC_stLeftLaneAvail;
boolean_T LATC_stLeftLaneLoss;
uint8_T LATC_stOperMod;
boolean_T LATC_stRgtLaneAvail;
boolean_T LATC_stRgtLaneLoss;
boolean_T LATC_stSteerAvrgActv;
boolean_T LATC_stSteerAvrgComplt;
boolean_T LATC_stSteerAvrgEnb;
uint16_T LATC_tDelyLenCycle;
real_T LATC_tTiDely4MCU2RCAR;
real_T LATC_vDistErrFlt4LQR;
real_T LATC_vLatSpdFlt4LQR;
real_T LATC_vVehSpdKph;
real_T LATC_wVehYawRate;
real_T LATC_wYRErr4LQR;
real_T LATC_wYRErrFlt4LQR;
real_T LCC_facCntrLaneA0;
real_T LCC_facCntrLaneA1;
real_T LCC_facCntrLaneA2;
real_T LCC_facCntrLaneA3;
real_T LCC_facCntrLaneA4;
real_T LCC_facCntrLaneA5;
real_T LCC_lPrevCv4LQR;
real_T LCC_lPrevDist4LQR;
real_T LCC_lPrevHeading4LQR;
real_T LCC_lPrevLatDist4LQR;
real_T LC_lPrevCv4FF;
real_T LC_lPrevHeading4FB;
real_T LC_lPrevLatDist4FB;
BUS_LKAS_0413 LKAS_0413;
real_T LKA_wVehRPS_Ref;
uint16_T LQR_IntNum;
real_T LatCv4LQR;
real_T LatDist4LQR;
real_T LatHeading4LQR;
uint8_T VMC_CtrlMode;
uint8_T VMC_CtrlReqd;
real_T VMC_phiTarAgSteer;

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
