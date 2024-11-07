#ifndef _DFCV_VEHCILE_DEFINITION_C_
#define _DFCV_VEHCILE_DEFINITION_C_


#include "Vehicle_Definiton.h"

uint8_T Sys_stADMd_mp = 3;
real32_T SpdPln_lTrgLngErr_mp = 0;
real32_T SpdPln_vTrgSpd_mp = 0;
real32_T SpdPln_aTrgAcc_mp = 0;
uint8_T BhvCrdn_numBhvID_mp = 2;
real32_T VehDa_rSlop_mp = 0;
real32_T VehDa_rBrkPedl_mp = 0;
real32_T VehDa_mWght_mp = 10000;
real32_T VehDa_vEgoSpd_mp = 0;
real32_T VehDa_aEgoAcc_mp = 0;
int16_T VehDa_stTraCurGear_mp = 8;
real32_T VehDa_rTraCurGear_mp = 1;
uint8_T VehDa_stCluSwt_mp = 0;
int16_T VehDa_prcTrqEngNomFric_mp = 0;
int16_T VehDa_prcTrqEstimdLoss_mp = 0;
uint8_T VehDa_stSrcBrk_mp = 0;
int16_T VehDa_prcActuTrq_mp = 0;
int16_T VehDa_prcDrvrDmdTrq_mp = 0;
uint8_T VehDa_stSrcEngCtrl_mp = 0;
real32_T VehDa_pFrontLeft_mp = 0;
real32_T VehDa_nEngSpd_mp = 1000;
real32_T DRMC_lCurr = 20;
real32_T EnvDetn_lCllsnTarLgt = 0;
real32_T EnvDetn_vCllsnTarRel = 0;
uint8_T EnvDetn_idCllsnTar = 0;
uint8_T VehDa_stTrlrCnctn_mp = 0;
uint8_T VehDa_stTraSht_mp = 0;
uint8_T VehDa_stTraEgd_mp = 0;
int16_T VehDa_stTraSelGear_mp = 0;
uint8_T VehDa_stTraTrqLim_mp = 0;
real32_T VehDa_rAccrPedl_mp = 0;


real32_T VehDa_nOutpShaft_mp = 0;
uint8_T VehDa_stBrkReady4Rls_mp = 0;
real32_T VehDa_agPitch_mp = 0;
real32_T VehDa_varPitch_mp = 0;
uint8_T VehDa_stMapIsAvail_mp = 0;
uint8_T ACCS_vSetFrmSys_mp = 0;
uint8_T ADCU_set_speed_C = 70;
uint8_T VehDa_stEpb_mp = 0;
int8_T SpdPln_stTarType_mp = 0;
boolean_T SpdPln_stReleaseThrottle_mp = 0;
real32_T SpdPln_pcc_tar_throttle_mp = 0;

#if(VEHICLE_PLATFORM_DFCV == VEHICLE_PLATFORM_X69A)

real32_T VDC_facRollngResCalA_C = 0.0059;//���賣����

real32_T VDC_facRollngResCalB_C = 2.56E-5;//���������

real32_T VDC_facRollngResCalCmp_C = 0;//���貹����

real32_T VDC_facCD_C = 0.56;//����ϵ��

real32_T VDC_arFrnt_C = 2;//ӭ�����

real32_T VDC_moiWhl_C = 15.21;//������̥ת������

real32_T VDC_numWhl_C = 12.0;//��̥����

real32_T VDC_moiDrvTrnCmp_C = 0.0;//����ϵͳת����������

real32_T VDC_moiFlyWhl_C = 1.16;//����ת������

real32_T VDC_etaDrvAxlEff_C = 0.9;//׼����������Ч��

real32_T VDC_etaTra_C = 1.0;//�����䴫��Ч��

real32_T VDC_rFinalRatio_C = 8.881;//��������������

real32_T VDC_rdTire_C = 0.5223;//��̥�����뾶

real32_T VDC_trqRef_C = 3200.0;//�����ο�Ť��

real32_T CoChs_vIdl_C = 0.5; //Ĭ����͵����г�����

boolean_T Tra_stCfg_C = 0;//�Ƿ����ñ�����

real32_T Eng_nMax_C = 6000;//������/������ת��

real32_T Eng_nIdl_C = 0;//������/�������ת��

boolean_T Eng_stNomFricEna_C = 0;//������Ħ��Ť�ذٷֱȼ��㿪��

boolean_T Eng_stEstimdLossEna_C = 0;//������������ʧŤ�ذٷֱȼ��㿪��

#endif //(VEHICLE_PLATFORM = VEHICLE_PLATFORM_X69A)

#if(VEHICLE_PLATFORM_DFCV == VEHICLE_PLATFORM_KAX1)

real32_T VDC_facRollngResCalA_C = 0.01;

real32_T VDC_facRollngResCalB_C = 5E-5;

real32_T VDC_facRollngResCalCmp_C = 0.05;

real32_T VDC_facCD_C = 0.6;

real32_T VDC_arFrnt_C = 3.5;

real32_T VDC_moiWhl_C = 12.2;

real32_T VDC_numWhl_C = 6;

real32_T VDC_moiDrvTrnCmp_C = 0.0;

real32_T VDC_moiFlyWhl_C = 0.824;

real32_T VDC_etaDrvAxlEff_C = 0.95;

real32_T VDC_etaTra_C = 0.96;

real32_T VDC_rFinalRatio_C = 5.92;

real32_T VDC_rdTire_C = 0.494;

real32_T VDC_trqRef_C = 1000;

real32_T CoChs_vIdl_C = 6;

boolean_T Tra_stCfg_C = 1;

real32_T Eng_nMax_C = 2650;

real32_T Eng_nIdl_C = 650;

boolean_T Eng_stNomFricEna_C = 1;

boolean_T Eng_stEstimdLossEna_C = 0;

#endif //(VEHICLE_PLATFORM = VEHICLE_PLATFORM_KAX1)

#if(VEHICLE_PLATFORM_DFCV == VEHICLE_PLATFORM_H19E)

real32_T VDC_facRollngResCalA_C = 0;

real32_T VDC_facRollngResCalB_C = 0;

real32_T VDC_facRollngResCalCmp_C = 0.0078;

real32_T VDC_facCD_C = 0.56;

real32_T VDC_arFrnt_C = 6.5;

real32_T VDC_moiWhl_C = 15.21;

real32_T VDC_numWhl_C = 10;

real32_T VDC_moiDrvTrnCmp_C = 0.0;

real32_T VDC_moiFlyWhl_C = 1.16;

real32_T VDC_etaDrvAxlEff_C = 0.97;

real32_T VDC_etaTra_C = 0.98;

real32_T VDC_rFinalRatio_C = 2.69;

real32_T VDC_rdTire_C = 0.53;

real32_T VDC_trqRef_C = 3316;

real32_T CoChs_vIdl_C = -1;

boolean_T Tra_stCfg_C = 1;

real32_T Eng_nMax_C = 2000;

real32_T Eng_nIdl_C = 550;

boolean_T Eng_stNomFricEna_C = 0;

boolean_T Eng_stEstimdLossEna_C = 0;

real32_T VDC_numWhlWhtTrlr_C = 22;// 带挂轮胎个数

real32_T Eng_nExtCurMin_C = 1000;//外特性查表最低转速

uint8_T Eng_idPltfrm_C = 1;//车辆平台ID

uint8_T MsgTx_stSrcAdr_C = 158; //XBR源地址

uint8_T VMC_swtAccnGvnrFctEna_C = 1;//车速闭环功能使能

uint8_T VMC_swtAccnGvrnInil_C = 0;//车速闭环积分功能使能 

#endif //(VEHICLE_PLATFORM = VEHICLE_PLATFORM_H19E)

#endif//_DFCV_VEHCILE_DEFINITION_C_