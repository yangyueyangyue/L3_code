/*
 * File: LATC_disp.h
 *
 * Code generated for Simulink model 'LATCtrl'.
 *
 * Model version                  : 1.265
 * Simulink Coder version         : 8.5 (R2013b) 08-Aug-2013
 * C/C++ source code generated on : Mon Oct 24 15:33:48 2022
 */

#ifndef RTW_HEADER_LATC_disp_h_
#define RTW_HEADER_LATC_disp_h_
#include "LACC_BUS.h"
#include "rtwtypes.h"

/* Exported data declaration */
/* Declaration for custom storage class: ExportToFile */
extern BUS_EHPS_0412 EHPS_0412;
extern real_T ExC_CheckSum;
extern real_T ExC_MessageCntr;
extern real_T LATC_Lane0Cv1;
extern real_T LATC_Lane0Cv2;
extern real_T LATC_Lane0Cv3;
extern real_T LATC_Lane1Cv1;
extern real_T LATC_Lane1Cv2;
extern real_T LATC_Lane1Cv3;
extern real_T LATC_aLatAcc;
extern real_T LATC_aLonAcc;
extern real_T LATC_aVertAcc;
extern real_T LATC_agFBSteerDeg;
extern real_T LATC_agFFSteerDeg;
extern real_T LATC_agFrTyre4LQR;
extern real_T LATC_agFrTyreFB;
extern real_T LATC_agFrTyreFB4Math;
extern real_T LATC_agFrTyreFBEle1;
extern real_T LATC_agFrTyreFBEle2;
extern real_T LATC_agFrTyreFBEle3;
extern real_T LATC_agFrTyreFBEle4;
extern real_T LATC_agFrTyreFF;
extern real_T LATC_agHead4Cmr;
extern real_T LATC_agHead4TgtPath;
extern real_T LATC_agHeadFlt4LQR;
extern real_T LATC_agSteerZeroOfst;
extern real_T LATC_agTarSteer4Post;
extern real_T LATC_agTarSteer4RL;
extern real_T LATC_agTarSteer4RLS;
extern real_T LATC_agTarSteerDeg;
extern real_T LATC_agTarSteerRad;
extern real_T LATC_agTarSteerReq;
extern real_T LATC_agTarSteerReqd;
extern real_T LATC_cCv4Cmr;
extern real_T LATC_cCv4TgtPath;
extern real_T LATC_cTgtPathCv0;
extern real_T LATC_cTgtPathCv1;
extern real_T LATC_cTgtPathCv2;
extern real_T LATC_cTgtPathCv3;
extern real_T LATC_cTgtPathCv4;
extern real_T LATC_cTgtPathCv5;
extern real_T LATC_eleK1;
extern real_T LATC_eleK2;
extern real_T LATC_eleK3;
extern real_T LATC_eleK4;
extern real_T LATC_facC0Filt;
extern real_T LATC_facC0Hat;
extern real_T LATC_facC1Filt;
extern real_T LATC_facC1Hat;
extern real_T LATC_facLeftLaneA0;
extern real_T LATC_facLeftLaneA1;
extern real_T LATC_facLeftLaneA2;
extern real_T LATC_facLeftLaneA3;
extern real_T LATC_facRgtLaneA0;
extern real_T LATC_facRgtLaneA1;
extern real_T LATC_facRgtLaneA2;
extern real_T LATC_facRgtLaneA3;
extern real_T LATC_lDist4Cmr;
extern real_T LATC_lLaneDist0;
extern real_T LATC_lLaneDist1;
extern real_T LATC_lLaneWidth4Loss;
extern real_T LATC_lLaneWidthRaw;
extern real_T LATC_lLatPosn4TgtPath;
extern real_T LATC_lLgtDist2PathZero;
extern real_T LATC_mMass4EBS;
extern real_T LATC_mVehMass;
extern real_T LATC_phiSteerAngle;
extern boolean_T LATC_stLeftLaneAvail;
extern boolean_T LATC_stLeftLaneLoss;
extern uint8_T LATC_stOperMod;
extern boolean_T LATC_stRgtLaneAvail;
extern boolean_T LATC_stRgtLaneLoss;
extern boolean_T LATC_stSteerAvrgActv;
extern boolean_T LATC_stSteerAvrgComplt;
extern boolean_T LATC_stSteerAvrgEnb;
extern uint16_T LATC_tDelyLenCycle;
extern real_T LATC_tTiDely4MCU2RCAR;
extern real_T LATC_vDistErrFlt4LQR;
extern real_T LATC_vLatSpdFlt4LQR;
extern real_T LATC_vVehSpdKph;
extern real_T LATC_wVehYawRate;
extern real_T LATC_wYRErr4LQR;
extern real_T LATC_wYRErrFlt4LQR;
extern real_T LCC_facCntrLaneA0;
extern real_T LCC_facCntrLaneA1;
extern real_T LCC_facCntrLaneA2;
extern real_T LCC_facCntrLaneA3;
extern real_T LCC_facCntrLaneA4;
extern real_T LCC_facCntrLaneA5;
extern real_T LCC_lPrevCv4LQR;
extern real_T LCC_lPrevDist4LQR;
extern real_T LCC_lPrevHeading4LQR;
extern real_T LCC_lPrevLatDist4LQR;
extern real_T LC_lPrevCv4FF;
extern real_T LC_lPrevHeading4FB;
extern real_T LC_lPrevLatDist4FB;
extern BUS_LKAS_0413 LKAS_0413;
extern real_T LKA_wVehRPS_Ref;
extern uint16_T LQR_IntNum;
extern real_T LatCv4LQR;
extern real_T LatDist4LQR;
extern real_T LatHeading4LQR;
extern uint8_T VMC_CtrlMode;
extern uint8_T VMC_CtrlReqd;
extern real_T VMC_phiTarAgSteer;

#endif                                 /* RTW_HEADER_LATC_disp_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
