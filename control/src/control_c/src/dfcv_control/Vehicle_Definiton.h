#ifndef _DFCV_VEHCILE_DEFINITION_H_
#define _DFCV_VEHCILE_DEFINITION_H_

#define VEHICLE_PLATFORM_H19E (1)
#define VEHICLE_PLATFORM_KAX1 (2)
#define VEHICLE_PLATFORM_X69A (3)

#define VEHICLE_PLATFORM_DFCV VEHICLE_PLATFORM_H19E

#ifndef VEHICLE_PLATFORM_DFCV
#define VEHICLE_PLATFORM_DFCV VEHICLE_PLATFORM_H19E
#endif

#include "rtwtypes.h"


extern uint8_T Sys_stADMd_mp;
extern real32_T SpdPln_lTrgLngErr_mp;
extern real32_T SpdPln_vTrgSpd_mp;
extern real32_T SpdPln_aTrgAcc_mp;
extern uint8_T BhvCrdn_numBhvID_mp;
extern real32_T VehDa_rSlop_mp;
extern real32_T VehDa_rBrkPedl_mp;
extern real32_T VehDa_mWght_mp;
extern real32_T VehDa_vEgoSpd_mp;
extern real32_T VehDa_aEgoAcc_mp;
extern int16_T VehDa_stTraCurGear_mp;
extern real32_T VehDa_rTraCurGear_mp;
extern uint8_T VehDa_stCluSwt_mp;
extern int16_T VehDa_prcTrqEngNomFric_mp;
extern int16_T VehDa_prcTrqEstimdLoss_mp;
extern uint8_T VehDa_stSrcBrk_mp;
extern int16_T VehDa_prcActuTrq_mp;
extern int16_T VehDa_prcDrvrDmdTrq_mp;
extern uint8_T VehDa_stSrcEngCtrl_mp;
extern real32_T VehDa_pFrontLeft_mp;
extern real32_T VehDa_nEngSpd_mp;
extern real32_T DRMC_lCurr;
extern real32_T EnvDetn_lCllsnTarLgt;
extern real32_T EnvDetn_vCllsnTarRel;
extern uint8_T EnvDetn_idCllsnTar;
extern uint8_T VehDa_stTrlrCnctn_mp;
extern uint8_T VehDa_stTraSht_mp;
extern uint8_T VehDa_stTraEgd_mp;
extern int16_T VehDa_stTraSelGear_mp;
extern uint8_T VehDa_stTraTrqLim_mp;
extern real32_T VehDa_rAccrPedl_mp;

extern real32_T VehDa_nOutpShaft_mp;
extern uint8_T VehDa_stBrkReady4Rls_mp;
extern real32_T VehDa_agPitch_mp;
extern real32_T VehDa_varPitch_mp;
extern uint8_T VehDa_stMapIsAvail_mp;
extern uint8_T VehDa_stEpb_mp;       
extern int8_T SpdPln_stTarType_mp;   
extern boolean_T SpdPln_stReleaseThrottle_mp;
extern uint8_T ACCS_vSetFrmSys_mp;
extern real32_T SpdPln_pcc_tar_throttle_mp;

extern uint8_T ADCU_set_speed_C;
extern real32_T VDC_facRollngResCalA_C;
extern real32_T VDC_facRollngResCalB_C;
extern real32_T VDC_facRollngResCalCmp_C;
extern real32_T VDC_facCD_C;
extern real32_T VDC_arFrnt_C;
extern real32_T VDC_moiWhl_C;
extern real32_T VDC_numWhl_C;
extern real32_T VDC_numWhlWhtTrlr_C;
extern real32_T VDC_moiDrvTrnCmp_C;
extern real32_T VDC_moiFlyWhl_C;
extern real32_T VDC_etaDrvAxlEff_C;
extern real32_T VDC_etaTra_C;
extern real32_T VDC_rFinalRatio_C;
extern real32_T VDC_rdTire_C;
extern real32_T VDC_trqRef_C;
extern real32_T CoChs_vIdl_C;
extern boolean_T Tra_stCfg_C;
extern real32_T Eng_nMax_C;
extern real32_T Eng_nIdl_C;
extern boolean_T Eng_stNomFricEna_C;
extern boolean_T Eng_stEstimdLossEna_C;
extern real32_T Eng_nExtCurMin_C;
extern uint8_T Eng_idPltfrm_C;
extern uint8_T MsgTx_stSrcAdr_C;
extern uint8_T VMC_swtAccnGvnrFctEna_C;
extern uint8_T VMC_swtAccnGvrnInil_C;

#endif