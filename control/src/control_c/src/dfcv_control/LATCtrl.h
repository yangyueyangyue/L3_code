/*
 * File: LATCtrl.h
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

#ifndef RTW_HEADER_LATCtrl_h_
#define RTW_HEADER_LATCtrl_h_
#ifndef LATCtrl_COMMON_INCLUDES_
# define LATCtrl_COMMON_INCLUDES_
#include <float.h>
#include <math.h>
#include <string.h>
#include "rtwtypes.h"
#include "rtw_shared_utils.h"
#endif                                 /* LATCtrl_COMMON_INCLUDES_ */

#include "LATCtrl_types.h"

/* Includes for objects with custom storage classes. */
#include "LATC_disp.h"
#include "LATC_CAL.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((void*) 0)
#endif

/* Block signals (auto storage) */
typedef struct {
  real_T DataStoreRead[25];            /* '<S39>/Data Store Read' */
  real_T UnitDelay1;                   /* '<S39>/Unit Delay1' */
  real_T Assignment[25];               /* '<S39>/Assignment' */
  real_T Divide2;                      /* '<S5>/Divide2' */
  real_T MinMax;                       /* '<S5>/MinMax' */
  real_T MatrixConcatenate1[2];        /* '<S38>/Matrix Concatenate1' */
  real_T MatrixConcatenate3[4];        /* '<S38>/Matrix Concatenate3' */
  real_T D;                            /* '<S92>/D' */
  real_T Delay;                        /* '<S12>/Delay' */
  real_T UnitDelay;                    /* '<S12>/Unit Delay' */
  real_T Abs;                          /* '<S12>/Abs' */
  real_T Abs2;                         /* '<S12>/Abs2' */
  real_T Abs1;                         /* '<S12>/Abs1' */
  real_T UnitDelay1_b;                 /* '<S12>/Unit Delay1' */
  real_T Abs4;                         /* '<S12>/Abs4' */
  real_T Abs6;                         /* '<S12>/Abs6' */
  real_T Abs5;                         /* '<S12>/Abs5' */
  real_T UnitDelay2;                   /* '<S12>/Unit Delay2' */
  real_T Product3;                     /* '<S20>/Product3' */
  real_T Saturation;                   /* '<S20>/Saturation' */
  real_T Product1;                     /* '<S50>/Product1' */
  real_T MathFunction;                 /* '<S50>/Math Function' */
  real_T Product2;                     /* '<S50>/Product2' */
  real_T MathFunction1;                /* '<S50>/Math Function1' */
  real_T Product3_j;                   /* '<S50>/Product3' */
  real_T MathFunction2;                /* '<S50>/Math Function2' */
  real_T Product4;                     /* '<S50>/Product4' */
  real_T MathFunction3;                /* '<S50>/Math Function3' */
  real_T Product5;                     /* '<S50>/Product5' */
  real_T DataTypeConversion;           /* '<Root>/Data Type Conversion' */
  real_T Switch1;                      /* '<S28>/Switch1' */
  real_T Product3_h;                   /* '<S28>/Product3' */
  real_T LCC_lPrevDistLimit_p;         /* '<S28>/LCC_lPrevDistLimit' */
  real_T Switch2;                      /* '<S28>/Switch2' */
  real_T Product1_i;                   /* '<S25>/Product1' */
  real_T MathFunction_j;               /* '<S25>/Math Function' */
  real_T Product2_o;                   /* '<S25>/Product2' */
  real_T MathFunction1_n;              /* '<S25>/Math Function1' */
  real_T Product3_f;                   /* '<S25>/Product3' */
  real_T MathFunction2_e;              /* '<S25>/Math Function2' */
  real_T Product4_h;                   /* '<S25>/Product4' */
  real_T MathFunction3_l;              /* '<S25>/Math Function3' */
  real_T Product5_b;                   /* '<S25>/Product5' */
  real_T Switch;                       /* '<S25>/Switch' */
  real_T Saturation_j;                 /* '<S25>/Saturation' */
  real_T Product3_b;                   /* '<S17>/Product3' */
  real_T D_a;                          /* '<S93>/D' */
  real_T MathFunction2_k;              /* '<S51>/Math Function2' */
  real_T Product7;                     /* '<S51>/Product7' */
  real_T Product8;                     /* '<S51>/Product8' */
  real_T MathFunction1_a;              /* '<S51>/Math Function1' */
  real_T Product5_g;                   /* '<S51>/Product5' */
  real_T Product6;                     /* '<S51>/Product6' */
  real_T MathFunction_p;               /* '<S51>/Math Function' */
  real_T Product2_p;                   /* '<S51>/Product2' */
  real_T Product4_i;                   /* '<S51>/Product4' */
  real_T Product1_d;                   /* '<S51>/Product1' */
  real_T Product3_o;                   /* '<S51>/Product3' */
  real_T MathFunction2_p;              /* '<S26>/Math Function2' */
  real_T Product7_d;                   /* '<S26>/Product7' */
  real_T Product8_l;                   /* '<S26>/Product8' */
  real_T MathFunction1_m;              /* '<S26>/Math Function1' */
  real_T Product5_f;                   /* '<S26>/Product5' */
  real_T Product6_o;                   /* '<S26>/Product6' */
  real_T MathFunction_h;               /* '<S26>/Math Function' */
  real_T Product2_b;                   /* '<S26>/Product2' */
  real_T Product4_b;                   /* '<S26>/Product4' */
  real_T Product1_m;                   /* '<S26>/Product1' */
  real_T Product3_i;                   /* '<S26>/Product3' */
  real_T Switch_o;                     /* '<S26>/Switch' */
  real_T Saturation_f;                 /* '<S26>/Saturation' */
  real_T Product1_o;                   /* '<S17>/Product1' */
  real_T Switch3;                      /* '<S57>/Switch3' */
  real_T Add3;                         /* '<S57>/Add3' */
  real_T Product6_j;                   /* '<S17>/Product6' */
  real_T Switch_c[3];                  /* '<S37>/Switch' */
  real_T VariableIntegerDelay3[3];     /* '<S30>/Variable Integer Delay3' */
  real_T UnitDelay_e;                  /* '<S30>/Unit Delay' */
  real_T Switch_n[3];                  /* '<S30>/Switch' */
  real_T T_sin;                        /* '<S35>/T_sin' */
  real_T Product;                      /* '<S35>/Product' */
  real_T Product1_g;                   /* '<S35>/Product1' */
  real_T Product1_mp;                  /* '<S20>/Product1' */
  real_T Saturation1;                  /* '<S20>/Saturation1' */
  real_T MathFunction1_g;              /* '<S52>/Math Function1' */
  real_T Product6_h;                   /* '<S52>/Product6' */
  real_T Product7_h;                   /* '<S52>/Product7' */
  real_T MathFunction_hi;              /* '<S52>/Math Function' */
  real_T Product1_c;                   /* '<S52>/Product1' */
  real_T Product5_a;                   /* '<S52>/Product5' */
  real_T Product2_l;                   /* '<S52>/Product2' */
  real_T Product4_p;                   /* '<S52>/Product4' */
  real_T Product3_iz;                  /* '<S52>/Product3' */
  real_T D_l;                          /* '<S94>/D' */
  real_T MathFunction1_j;              /* '<S27>/Math Function1' */
  real_T Product6_a;                   /* '<S27>/Product6' */
  real_T Product7_b;                   /* '<S27>/Product7' */
  real_T MathFunction_pq;              /* '<S27>/Math Function' */
  real_T Product1_cu;                  /* '<S27>/Product1' */
  real_T Product5_m;                   /* '<S27>/Product5' */
  real_T Product2_c;                   /* '<S27>/Product2' */
  real_T Product4_b3;                  /* '<S27>/Product4' */
  real_T Product3_c;                   /* '<S27>/Product3' */
  real_T Add1;                         /* '<S27>/Add1' */
  real_T Switch_o3;                    /* '<S27>/Switch' */
  real_T Product2_ca;                  /* '<S17>/Product2' */
  real_T Product3_ju;                  /* '<S35>/Product3' */
  real_T Add2;                         /* '<S35>/Add2' */
  real_T Product2_c5;                  /* '<S35>/Product2' */
  real_T VectorConcatenate[3];         /* '<S35>/Vector Concatenate' */
  real_T T_cos;                        /* '<S36>/T_cos' */
  real_T Product_h;                    /* '<S36>/Product' */
  real_T Product1_c3;                  /* '<S36>/Product1' */
  real_T TmpSignalConversionAtMathFuncti[3];
  real_T MathFunction1_d[3];           /* '<S36>/Math Function1' */
  real_T VectorConcatenate_d[9];       /* '<S36>/Vector Concatenate' */
  real_T UnitDelay1_p;                 /* '<S33>/Unit Delay1' */
  real_T Sum6;                         /* '<S33>/Sum6' */
  real_T Add2_e;                       /* '<S33>/Add2' */
  real_T MathFunction_i;               /* '<S33>/Math Function' */
  real_T Switch2_d[3];                 /* '<S30>/Switch2' */
  real_T TmpSignalConversionAtMathFunc_b[3];
  real_T MathFunction_f[3];            /* '<S32>/Math Function' */
  real_T TmpSignalConversionAtMathFunc_d[3];
  real_T MathFunction1_jb[3];          /* '<S32>/Math Function1' */
  real_T TmpSignalConversionAtMathFunc_j[3];
  real_T MathFunction2_d[3];           /* '<S32>/Math Function2' */
  real_T VectorConcatenate_p[9];       /* '<S32>/Vector Concatenate' */
  real_T TmpSignalConversionAtMathFunc_o[3];
  real_T MathFunction3_a[3];           /* '<S32>/Math Function3' */
  real_T TmpSignalConversionAtMathFunc_k[3];
  real_T MathFunction4[3];             /* '<S32>/Math Function4' */
  real_T TmpSignalConversionAtMathFun_ou[3];
  real_T MathFunction5[3];             /* '<S32>/Math Function5' */
  real_T VectorConcatenate1[9];        /* '<S32>/Vector Concatenate1' */
  real_T VariableIntegerDelay[9];      /* '<S30>/Variable Integer Delay' */
  real_T Switch1_f[9];                 /* '<S30>/Switch1' */
  real_T Product4_l;                   /* '<S17>/Product4' */
  real_T Product5_c;                   /* '<S17>/Product5' */
  real_T DataStoreRead_d[20];          /* '<S84>/Data Store Read' */
  real_T D_k;                          /* '<S78>/D' */
  real_T DataTypeConversion14;         /* '<Root>/Data Type Conversion14' */
  real_T Switch14;                     /* '<S78>/Switch14' */
  real_T Switch6;                      /* '<S23>/Switch6' */
  real_T Product12;                    /* '<S23>/Product12' */
  real_T Product7_hr;                  /* '<S23>/Product7' */
  real_T Gain1;                        /* '<S23>/Gain1' */
  real_T Switch1_d;                    /* '<S84>/Switch1' */
  real_T UnitDelay_p;                  /* '<S84>/Unit Delay' */
  real_T Switch13;                     /* '<S84>/Switch13' */
  real_T Sum6_h;                       /* '<S84>/Sum6' */
  real_T MathFunction_iq;              /* '<S84>/Math Function' */
  real_T Assignment_f[20];             /* '<S84>/Assignment' */
  real_T MathFunction1_b[20];          /* '<S84>/Math Function1' */
  real_T Product_c;                    /* '<S84>/Product' */
  real_T Abs_f;                        /* '<S73>/Abs' */
  real_T DLookupTable3;                /* '<S73>/1-D Lookup Table3' */
  real_T D_k5;                         /* '<S77>/D' */
  real_T Switch14_k;                   /* '<S77>/Switch14' */
  real_T Product11;                    /* '<S23>/Product11' */
  real_T Subtract2;                    /* '<S23>/Subtract2' */
  real_T Abs_p;                        /* '<S80>/Abs' */
  real_T Product1_b;                   /* '<S80>/Product1' */
  real_T Subtract;                     /* '<S80>/Subtract' */
  real_T Subtract1;                    /* '<S80>/Subtract1' */
  real_T Divide;                       /* '<S80>/Divide' */
  real_T Switch1_f3;                   /* '<S80>/Switch1' */
  real_T Delay1;                       /* '<S72>/Delay1' */
  real_T Delay2;                       /* '<S72>/Delay2' */
  real_T Delay3;                       /* '<S72>/Delay3' */
  real_T Delay4;                       /* '<S72>/Delay4' */
  real_T Delay5;                       /* '<S72>/Delay5' */
  real_T Delay6;                       /* '<S72>/Delay6' */
  real_T Delay7;                       /* '<S72>/Delay7' */
  real_T Delay8;                       /* '<S72>/Delay8' */
  real_T Delay9;                       /* '<S72>/Delay9' */
  real_T Sum;                          /* '<S72>/Sum' */
  real_T TrigonometricFunction;        /* '<S23>/Trigonometric Function' */
  real_T Product9;                     /* '<S23>/Product9' */
  real_T Delay1_i;                     /* '<S23>/Delay1' */
  real_T Switch1_h;                    /* '<S75>/Switch1' */
  real_T Switch2_i;                    /* '<S75>/Switch2' */
  real_T Add1_m;                       /* '<S68>/Add1' */
  real_T Switch3_h;                    /* '<S75>/Switch3' */
  real_T Product1_cf;                  /* '<S68>/Product1' */
  real_T Divide_g;                     /* '<S68>/Divide' */
  real_T Product9_i;                   /* '<S68>/Product9' */
  real_T Switch5;                      /* '<S75>/Switch5' */
  real_T Product10;                    /* '<S68>/Product10' */
  real_T Switch_ou;                    /* '<S75>/Switch' */
  real_T Product11_h;                  /* '<S68>/Product11' */
  real_T Subtract1_n;                  /* '<S68>/Subtract1' */
  real_T Switch4;                      /* '<S75>/Switch4' */
  real_T Product12_l;                  /* '<S68>/Product12' */
  real_T Divide1;                      /* '<S68>/Divide1' */
  real_T Product13;                    /* '<S68>/Product13' */
  real_T Product16;                    /* '<S68>/Product16' */
  real_T Product17;                    /* '<S68>/Product17' */
  real_T Subtract3;                    /* '<S68>/Subtract3' */
  real_T Product18;                    /* '<S68>/Product18' */
  real_T Divide4;                      /* '<S68>/Divide4' */
  real_T Add5;                         /* '<S68>/Add5' */
  real_T Product19;                    /* '<S68>/Product19' */
  real_T Product20;                    /* '<S68>/Product20' */
  real_T Product21;                    /* '<S68>/Product21' */
  real_T Add3_p;                       /* '<S68>/Add3' */
  real_T Product22;                    /* '<S68>/Product22' */
  real_T Divide5;                      /* '<S68>/Divide5' */
  real_T Product23;                    /* '<S68>/Product23' */
  real_T MatrixConcatenate[16];        /* '<S68>/Matrix Concatenate' */
  real_T Product2_i[16];               /* '<S68>/Product2' */
  real_T Add4[16];                     /* '<S68>/Add4' */
  real_T Product24;                    /* '<S68>/Product24' */
  real_T MatrixConcatenate2[4];        /* '<S68>/Matrix Concatenate2' */
  real_T Product26[4];                 /* '<S68>/Product26' */
  real_T MathFunction_m[4];            /* '<S68>/Math Function' */
  real_T DataStoreRead_o[10];          /* '<S74>/Data Store Read' */
  real_T Switch3_e;                    /* '<S53>/Switch3' */
  real_T Add5_m;                       /* '<S5>/Add5' */
  real_T Divide4_e;                    /* '<S5>/Divide4' */
  real_T Add7;                         /* '<S5>/Add7' */
  real_T UnitDelay_l;                  /* '<S89>/Unit Delay' */
  real_T Switch2_p;                    /* '<S53>/Switch2' */
  real_T Add3_n;                       /* '<S53>/Add3' */
  real_T Divide2_e;                    /* '<S53>/Divide2' */
  real_T Divide1_i;                    /* '<S53>/Divide1' */
  real_T Assignment_h[10];             /* '<S74>/Assignment' */
  real_T Switch5_d[4];                 /* '<S23>/Switch5' */
  real_T Switch3_d;                    /* '<S73>/Switch3' */
  real_T Delay1_n;                     /* '<S82>/Delay1' */
  real_T Delay2_o;                     /* '<S82>/Delay2' */
  real_T Delay3_j;                     /* '<S82>/Delay3' */
  real_T Delay4_j;                     /* '<S82>/Delay4' */
  real_T Delay5_k;                     /* '<S82>/Delay5' */
  real_T Delay6_a;                     /* '<S82>/Delay6' */
  real_T Delay7_p;                     /* '<S82>/Delay7' */
  real_T Delay8_l;                     /* '<S82>/Delay8' */
  real_T Delay9_j;                     /* '<S82>/Delay9' */
  real_T Sum_l;                        /* '<S82>/Sum' */
  real_T Switch4_b;                    /* '<S73>/Switch4' */
  real_T Product13_e;                  /* '<S73>/Product13' */
  real_T Product10_c;                  /* '<S23>/Product10' */
  real_T Subtract1_m;                  /* '<S23>/Subtract1' */
  real_T Abs_k;                        /* '<S79>/Abs' */
  real_T Product1_n;                   /* '<S79>/Product1' */
  real_T Subtract_n;                   /* '<S79>/Subtract' */
  real_T Subtract1_g;                  /* '<S79>/Subtract1' */
  real_T Divide_p;                     /* '<S79>/Divide' */
  real_T Switch1_dv;                   /* '<S79>/Switch1' */
  real_T Delay1_m;                     /* '<S69>/Delay1' */
  real_T Delay2_i;                     /* '<S69>/Delay2' */
  real_T Delay3_f;                     /* '<S69>/Delay3' */
  real_T Delay4_p;                     /* '<S69>/Delay4' */
  real_T Delay5_j;                     /* '<S69>/Delay5' */
  real_T Delay6_c;                     /* '<S69>/Delay6' */
  real_T Delay7_n;                     /* '<S69>/Delay7' */
  real_T Delay8_c;                     /* '<S69>/Delay8' */
  real_T Delay9_m;                     /* '<S69>/Delay9' */
  real_T Sum_b;                        /* '<S69>/Sum' */
  real_T Delay10;                      /* '<S69>/Delay10' */
  real_T Delay11;                      /* '<S69>/Delay11' */
  real_T Delay12;                      /* '<S69>/Delay12' */
  real_T Delay13;                      /* '<S69>/Delay13' */
  real_T Delay14;                      /* '<S69>/Delay14' */
  real_T Delay15;                      /* '<S69>/Delay15' */
  real_T Delay16;                      /* '<S69>/Delay16' */
  real_T Delay17;                      /* '<S69>/Delay17' */
  real_T Delay18;                      /* '<S69>/Delay18' */
  real_T Sum1;                         /* '<S69>/Sum1' */
  real_T Gain1_g;                      /* '<S69>/Gain1' */
  real_T Saturation1_l;                /* '<S69>/Saturation1' */
  real_T UnitDelay5;                   /* '<S69>/Unit Delay5' */
  real_T UnitDelay6;                   /* '<S69>/Unit Delay6' */
  real_T Switch1_l;                    /* '<S73>/Switch1' */
  real_T Switch2_m;                    /* '<S73>/Switch2' */
  real_T Switch5_c;                    /* '<S73>/Switch5' */
  real_T Switch6_j;                    /* '<S73>/Switch6' */
  real_T Switch7;                      /* '<S73>/Switch7' */
  real_T Switch8;                      /* '<S73>/Switch8' */
  real_T MatrixConcatenate2_o[16];     /* '<S23>/Matrix Concatenate2' */
  real_T Add1_e;                       /* '<S67>/Add1' */
  real_T Product1_cy;                  /* '<S67>/Product1' */
  real_T Divide_h;                     /* '<S67>/Divide' */
  real_T Product9_h;                   /* '<S67>/Product9' */
  real_T Product10_e;                  /* '<S67>/Product10' */
  real_T Product11_l;                  /* '<S67>/Product11' */
  real_T Subtract1_i;                  /* '<S67>/Subtract1' */
  real_T Product12_n;                  /* '<S67>/Product12' */
  real_T Divide1_e;                    /* '<S67>/Divide1' */
  real_T Product13_e3;                 /* '<S67>/Product13' */
  real_T Add2_b;                       /* '<S67>/Add2' */
  real_T Divide2_e2;                   /* '<S67>/Divide2' */
  real_T Product14;                    /* '<S67>/Product14' */
  real_T Product15;                    /* '<S67>/Product15' */
  real_T Subtract2_g;                  /* '<S67>/Subtract2' */
  real_T Divide3;                      /* '<S67>/Divide3' */
  real_T Product16_e;                  /* '<S67>/Product16' */
  real_T Product17_f;                  /* '<S67>/Product17' */
  real_T Subtract3_m;                  /* '<S67>/Subtract3' */
  real_T Product18_o;                  /* '<S67>/Product18' */
  real_T Divide4_c;                    /* '<S67>/Divide4' */
  real_T Product19_d;                  /* '<S67>/Product19' */
  real_T Product20_a;                  /* '<S67>/Product20' */
  real_T Product21_j;                  /* '<S67>/Product21' */
  real_T Add3_l;                       /* '<S67>/Add3' */
  real_T Product22_k;                  /* '<S67>/Product22' */
  real_T Divide5_o;                    /* '<S67>/Divide5' */
  real_T Product23_c;                  /* '<S67>/Product23' */
  real_T MatrixConcatenate_h[16];      /* '<S67>/Matrix Concatenate' */
  real_T Product2_p2[16];              /* '<S67>/Product2' */
  real_T Add4_k[16];                   /* '<S67>/Add4' */
  real_T Product24_c;                  /* '<S67>/Product24' */
  real_T MatrixConcatenate2_l[4];      /* '<S67>/Matrix Concatenate2' */
  real_T Product26_g[4];               /* '<S67>/Product26' */
  real_T MathFunction_n[4];            /* '<S67>/Math Function' */
  real_T VariableIntegerDelay3_i[16];  /* '<S23>/Variable Integer Delay3' */
  real_T Gain;                         /* '<S23>/Gain' */
  real_T Saturation_m;                 /* '<S23>/Saturation' */
  real_T Switch_d[4];                  /* '<S23>/Switch' */
  real_T Switch1_b[4];                 /* '<S23>/Switch1' */
  real_T Product8_n[4];                /* '<S23>/Product8' */
  real_T TmpSignalConversionAtProductInp[4];/* '<S23>/LATC_ErrComp' */
  real_T DiscreteTransferFcn;          /* '<S70>/Discrete Transfer Fcn' */
  real_T Switch10;                     /* '<S70>/Switch10' */
  real_T DiscreteTransferFcn2;         /* '<S70>/Discrete Transfer Fcn2' */
  real_T DiscreteTransferFcn_p;        /* '<S71>/Discrete Transfer Fcn' */
  real_T Switch10_l;                   /* '<S71>/Switch10' */
  real_T DiscreteTransferFcn2_d;       /* '<S71>/Discrete Transfer Fcn2' */
  real_T TrigonometricFunction2;       /* '<S5>/Trigonometric Function2' */
  real_T TrigonometricFunction3;       /* '<S5>/Trigonometric Function3' */
  real_T Add6;                         /* '<S5>/Add6' */
  real_T Divide5_d;                    /* '<S5>/Divide5' */
  real_T LATC_tZeroIdfyTimeThd_C_i;    /* '<S19>/LATC_tZeroIdfyTimeThd_C' */
  real_T Switch3_f;                    /* '<S54>/Switch3' */
  real_T Add3_e;                       /* '<S54>/Add3' */
  real_T TrigonometricFunction_b;      /* '<S56>/Trigonometric Function' */
  real_T Product1_g0;                  /* '<S56>/Product1' */
  real_T Add5_mi;                      /* '<S56>/Add5' */
  real_T D_n;                          /* '<S64>/D' */
  real_T Product4_m;                   /* '<S62>/Product4' */
  real_T Add2_ep;                      /* '<S62>/Add2' */
  real_T Abs_h;                        /* '<S66>/Abs' */
  real_T Product1_gn;                  /* '<S66>/Product1' */
  real_T Subtract_o;                   /* '<S66>/Subtract' */
  real_T Subtract1_d;                  /* '<S66>/Subtract1' */
  real_T Divide_l;                     /* '<S66>/Divide' */
  real_T Switch1_j;                    /* '<S66>/Switch1' */
  real_T MinMax4;                      /* '<S64>/MinMax4' */
  real_T MinMax3;                      /* '<S64>/MinMax3' */
  real_T Switch2_k;                    /* '<S63>/Switch2' */
  real_T D1;                           /* '<S58>/D1' */
  real_T Switch_h;                     /* '<S58>/Switch' */
  real_T Add;                          /* '<S58>/Add' */
  real_T D1_j;                         /* '<S61>/D1' */
  real_T Switch_p;                     /* '<S61>/Switch' */
  real_T MinMax_b;                     /* '<S61>/MinMax' */
  real_T Divide_pk;                    /* '<S58>/Divide' */
  real_T Saturation_c;                 /* '<S58>/Saturation' */
  real_T Divide1_m;                    /* '<S58>/Divide1' */
  real_T DataTypeConversion6;          /* '<Root>/Data Type Conversion6' */
  real_T Abs8;                         /* '<S12>/Abs8' */
  real_T Abs9;                         /* '<S12>/Abs9' */
  real_T DataTypeConversion7;          /* '<Root>/Data Type Conversion7' */
  real_T DataTypeConversion12;         /* '<Root>/Data Type Conversion12' */
  real_T Switch2_im;                   /* '<S6>/Switch2' */
  real_T Switch3_n;                    /* '<S6>/Switch3' */
  real_T Product4_a;                   /* '<S5>/Product4' */
  real_T Add1_c;                       /* '<S28>/Add1' */
  real_T P1;                           /* '<S94>/P1' */
  real_T A1;                           /* '<S94>/A1' */
  real_T MinMax2;                      /* '<S94>/MinMax2' */
  real_T Swt1;                         /* '<S94>/Swt1' */
  real_T Gain_h;                       /* '<S27>/Gain' */
  real_T P;                            /* '<S94>/P' */
  real_T A;                            /* '<S94>/A' */
  real_T MinMax1;                      /* '<S94>/MinMax1' */
  real_T Switch3_b;                    /* '<S27>/Switch3' */
  real_T P1_i;                         /* '<S93>/P1' */
  real_T A1_f;                         /* '<S93>/A1' */
  real_T MinMax2_j;                    /* '<S93>/MinMax2' */
  real_T Swt1_p;                       /* '<S93>/Swt1' */
  real_T Gain_j;                       /* '<S26>/Gain' */
  real_T P_c;                          /* '<S93>/P' */
  real_T A_f;                          /* '<S93>/A' */
  real_T MinMax1_f;                    /* '<S93>/MinMax1' */
  real_T Switch3_m;                    /* '<S26>/Switch3' */
  real_T P1_o;                         /* '<S92>/P1' */
  real_T A1_j;                         /* '<S92>/A1' */
  real_T MinMax2_b;                    /* '<S92>/MinMax2' */
  real_T Swt1_k;                       /* '<S92>/Swt1' */
  real_T Gain_n;                       /* '<S25>/Gain' */
  real_T P_k;                          /* '<S92>/P' */
  real_T A_g;                          /* '<S92>/A' */
  real_T MinMax1_f3;                   /* '<S92>/MinMax1' */
  real_T Switch3_nn;                   /* '<S25>/Switch3' */
  real_T Avrg;                         /* '<S24>/Chart' */
  real_T I_Cnt;                        /* '<S24>/Chart' */
  real_T I_Cnt_Frz;                    /* '<S24>/Chart' */
  real_T I_avrg_Frz;                   /* '<S24>/Chart' */
  real_T Avrg_Frz;                     /* '<S24>/Chart' */
  real_T Abs1_b;                       /* '<S87>/Abs1' */
  real_T Abs_pt;                       /* '<S87>/Abs' */
  real_T Abs4_k;                       /* '<S87>/Abs4' */
  real_T Abs3;                         /* '<S87>/Abs3' */
  real_T Abs2_o;                       /* '<S87>/Abs2' */
  real_T Abs1_g;                       /* '<S80>/Abs1' */
  real_T Switch2_g;                    /* '<S80>/Switch2' */
  real_T Product2_oi;                  /* '<S80>/Product2' */
  real_T Switch3_fw;                   /* '<S80>/Switch3' */
  real_T Subtract2_gu;                 /* '<S80>/Subtract2' */
  real_T Product4_hy;                  /* '<S80>/Product4' */
  real_T Product3_k;                   /* '<S80>/Product3' */
  real_T Add1_c4;                      /* '<S80>/Add1' */
  real_T Add3_i;                       /* '<S80>/Add3' */
  real_T Product6_ht;                  /* '<S80>/Product6' */
  real_T Product5_j;                   /* '<S80>/Product5' */
  real_T Add2_g;                       /* '<S80>/Add2' */
  real_T Abs1_k;                       /* '<S79>/Abs1' */
  real_T Switch2_pb;                   /* '<S79>/Switch2' */
  real_T Product2_ci;                  /* '<S79>/Product2' */
  real_T Switch3_np;                   /* '<S79>/Switch3' */
  real_T Subtract2_f;                  /* '<S79>/Subtract2' */
  real_T Product4_ah;                  /* '<S79>/Product4' */
  real_T Product3_e;                   /* '<S79>/Product3' */
  real_T Add1_d;                       /* '<S79>/Add1' */
  real_T Add3_et;                      /* '<S79>/Add3' */
  real_T Product6_i;                   /* '<S79>/Product6' */
  real_T Product5_cj;                  /* '<S79>/Product5' */
  real_T Add2_l;                       /* '<S79>/Add2' */
  real_T Swt;                          /* '<S78>/Swt' */
  real_T P1_m;                         /* '<S78>/P1' */
  real_T A1_h;                         /* '<S78>/A1' */
  real_T MinMax2_m;                    /* '<S78>/MinMax2' */
  real_T Swt1_a;                       /* '<S78>/Swt1' */
  real_T P_d;                          /* '<S78>/P' */
  real_T A_d;                          /* '<S78>/A' */
  real_T MinMax1_e;                    /* '<S78>/MinMax1' */
  real_T Swt_g;                        /* '<S77>/Swt' */
  real_T P1_k;                         /* '<S77>/P1' */
  real_T A1_c;                         /* '<S77>/A1' */
  real_T MinMax2_i;                    /* '<S77>/MinMax2' */
  real_T Swt1_j;                       /* '<S77>/Swt1' */
  real_T P_j;                          /* '<S77>/P' */
  real_T A_dv;                         /* '<S77>/A' */
  real_T MinMax1_k;                    /* '<S77>/MinMax1' */
  real_T K[4];                         /* '<S23>/LQR_solver' */
  real_T diff;                         /* '<S23>/LQR_solver' */
  real_T P_a[16];                      /* '<S23>/LQR_solver' */
  real_T TmpSignalConversionAtSFunctionI[4];/* '<S74>/Predict' */
  real_T x[4];                         /* '<S74>/Predict' */
  real_T Sum6_e;                       /* '<S73>/Sum6' */
  real_T Sum1_p;                       /* '<S73>/Sum1' */
  real_T Sum2;                         /* '<S73>/Sum2' */
  real_T Switch9;                      /* '<S73>/Switch9' */
  real_T Product3_n;                   /* '<S73>/Product3' */
  real_T Product1_oe;                  /* '<S73>/Product1' */
  real_T Switch10_h;                   /* '<S73>/Switch10' */
  real_T DLookupTable1;                /* '<S73>/1-D Lookup Table1' */
  real_T Product4_o;                   /* '<S73>/Product4' */
  real_T DLookupTable4;                /* '<S73>/1-D Lookup Table4' */
  real_T Product2_il;                  /* '<S73>/Product2' */
  real_T Product13_a;                  /* '<S71>/Product13' */
  real_T Product13_h;                  /* '<S70>/Product13' */
  real_T covOut;                       /* '<S69>/MATLAB Function' */
  real_T MatrixConcatenate1_k[4];      /* '<S23>/Matrix Concatenate1' */
  real_T MatrixConcatenate3_m[4];      /* '<S23>/Matrix Concatenate3' */
  real_T Switch1_c;                    /* '<S70>/Switch1' */
  real_T Switch9_l;                    /* '<S70>/Switch9' */
  real_T Product14_b;                  /* '<S70>/Product14' */
  real_T Switch9_b;                    /* '<S71>/Switch9' */
  real_T Product14_m;                  /* '<S71>/Product14' */
  real_T Abs1_e;                       /* '<S66>/Abs1' */
  real_T Switch2_e;                    /* '<S66>/Switch2' */
  real_T Product2_i5;                  /* '<S66>/Product2' */
  real_T Switch3_i;                    /* '<S66>/Switch3' */
  real_T Subtract2_n;                  /* '<S66>/Subtract2' */
  real_T Product4_f;                   /* '<S66>/Product4' */
  real_T Product3_nx;                  /* '<S66>/Product3' */
  real_T Add1_o;                       /* '<S66>/Add1' */
  real_T Add3_h;                       /* '<S66>/Add3' */
  real_T Product6_m;                   /* '<S66>/Product6' */
  real_T Product5_h;                   /* '<S66>/Product5' */
  real_T Add2_n;                       /* '<S66>/Add2' */
  real_T A1_k;                         /* '<S65>/A1' */
  real_T Swt1_f;                       /* '<S65>/Swt1' */
  real_T A2;                           /* '<S65>/A2' */
  real_T Sign;                         /* '<S64>/Sign' */
  real_T Abs_l;                        /* '<S64>/Abs' */
  real_T Switch_dp;                    /* '<S64>/Switch' */
  real_T MinMax5;                      /* '<S64>/MinMax5' */
  real_T P2;                           /* '<S64>/P2' */
  real_T Switch1_ck;                   /* '<S64>/Switch1' */
  real_T Swt_i;                        /* '<S64>/Swt' */
  real_T P1_l;                         /* '<S64>/P1' */
  real_T A1_m;                         /* '<S64>/A1' */
  real_T MinMax2_k;                    /* '<S64>/MinMax2' */
  real_T Swt1_d;                       /* '<S64>/Swt1' */
  real_T P_dp;                         /* '<S64>/P' */
  real_T A_ds;                         /* '<S64>/A' */
  real_T MinMax1_m;                    /* '<S64>/MinMax1' */
  real_T Switch1_i;                    /* '<S63>/Switch1' */
  real_T Add_p;                        /* '<S61>/Add' */
  real_T Switch1_g;                    /* '<S53>/Switch1' */
  real_T Delay_n;                      /* '<S47>/Delay' */
  real_T delay[3];                     /* '<S48>/delay' */
  real_T delay1[9];                    /* '<S48>/delay1' */
  real_T MatrixConcatenate_hc[3];      /* '<S47>/Matrix Concatenate' */
  real_T MathFunction2_o[3];           /* '<S47>/Math Function2' */
  real_T Divide_j[3];                  /* '<S48>/Divide' */
  real_T MathFunction_a[3];            /* '<S48>/Math Function' */
  real_T Divide3_i;                    /* '<S48>/Divide3' */
  real_T Sum1_n;                       /* '<S48>/Sum1' */
  real_T Divide2_m[3];                 /* '<S48>/Divide2' */
  real_T MathFunction2_kx[3];          /* '<S48>/Math Function2' */
  real_T Product1_j;                   /* '<S48>/Product1' */
  real_T Sum_m;                        /* '<S47>/Sum' */
  real_T Product1_ir;                  /* '<S47>/Product1' */
  real_T Sum_o;                        /* '<S48>/Sum' */
  real_T Divide5_j[3];                 /* '<S48>/Divide5' */
  real_T Sum2_n[3];                    /* '<S48>/Sum2' */
  real_T Switch_n3;                    /* '<S49>/Switch' */
  real_T Switch2_h;                    /* '<S49>/Switch2' */
  real_T Divide2_a;                    /* '<S47>/Divide2' */
  real_T Product_j;                    /* '<S47>/Product' */
  real_T Divide6;                      /* '<S48>/Divide6' */
  real_T MathFunction1_h[3];           /* '<S48>/Math Function1' */
  real_T Divide7[9];                   /* '<S48>/Divide7' */
  real_T Sum3[9];                      /* '<S48>/Sum3' */
  real_T Divide9[9];                   /* '<S48>/Divide9' */
  real_T x_m[2];                       /* '<S39>/Predict' */
  real_T VectorConcatenate_o[3];       /* '<S37>/Vector Concatenate' */
  real_T T_atan;                       /* '<S17>/T_atan' */
  real_T VectorConcatenate1_f[3];      /* '<S37>/Vector Concatenate1' */
  real_T xhat[3];                      /* '<S30>/LQR_solver' */
  real_T K_i[9];                       /* '<S30>/LQR_solver' */
  real_T Pk2[9];                       /* '<S30>/LQR_solver' */
  real_T Add1_cc;                      /* '<S16>/Add1' */
  real_T Divide1_o;                    /* '<S16>/Divide1' */
  real_T Add6_h;                       /* '<S16>/Add6' */
  real_T Divide_ha;                    /* '<S16>/Divide' */
  real_T Add2_c;                       /* '<S16>/Add2' */
  real_T Divide2_o;                    /* '<S16>/Divide2' */
  real_T Add4_b;                       /* '<S16>/Add4' */
  real_T Divide3_m;                    /* '<S16>/Divide3' */
  real_T Add1_j;                       /* '<S12>/Add1' */
  real_T Subtract_e;                   /* '<S12>/Subtract' */
  real_T Abs3_j;                       /* '<S12>/Abs3' */
  real_T Abs7;                         /* '<S12>/Abs7' */
  real_T Divide_hg;                    /* '<S3>/Divide' */
  real_T Switch_j;                     /* '<S1>/Switch' */
  real_T Switch1_a;                    /* '<S1>/Switch1' */
  real_T DataTypeConversion2;          /* '<S1>/Data Type Conversion2' */
  real_T Switch2_mw;                   /* '<S1>/Switch2' */
  real_T Switch3_d5;                   /* '<S1>/Switch3' */
  real_T Switch_jw;                    /* '<S2>/Switch' */
  real_T Switch1_g5;                   /* '<S2>/Switch1' */
  int32_T DataTypeConversion14_f;      /* '<S7>/Data Type Conversion14' */
  int32_T DataTypeConversion14_b;      /* '<S8>/Data Type Conversion14' */
  int16_T Product_g;                   /* '<S8>/Product' */
  int16_T Product2_f;                  /* '<S8>/Product2' */
  uint16_T UnitDelay_a;                /* '<S39>/Unit Delay' */
  uint16_T Sum6_g;                     /* '<S39>/Sum6' */
  uint16_T MathFunction_g;             /* '<S39>/Math Function' */
  uint16_T Switch1_dz;                 /* '<S39>/Switch1' */
  uint16_T UnitDelay_o;                /* '<S74>/Unit Delay' */
  uint16_T Sum6_j;                     /* '<S74>/Sum6' */
  uint16_T MathFunction_mz;            /* '<S74>/Math Function' */
  uint16_T Switch1_lu;                 /* '<S74>/Switch1' */
  uint16_T i;                          /* '<S23>/LQR_solver' */
  uint16_T DataTypeConversion_e;       /* '<S3>/Data Type Conversion' */
  uint16_T Saturation_k;               /* '<S3>/Saturation' */
  uint16_T DataTypeConversion9;        /* '<S8>/Data Type Conversion9' */
  uint16_T Low;                        /* '<S8>/Bitwise Operator1' */
  uint16_T Low_i;                      /* '<S8>/Bitwise Operator2' */
  uint16_T High;                       /* '<S8>/Shift Arithmetic' */
  uint16_T DataTypeConversion8;        /* '<S8>/Data Type Conversion8' */
  uint16_T Low_k;                      /* '<S8>/Bitwise Operator3' */
  uint16_T Low_d;                      /* '<S8>/Bitwise Operator4' */
  uint16_T High_p;                     /* '<S8>/Shift Arithmetic1' */
  int16_T DataTypeConversion13;        /* '<S7>/Data Type Conversion13' */
  int16_T DataTypeConversion7_d;       /* '<S8>/Data Type Conversion7' */
  uint8_T DataTypeConversion6_g;       /* '<S39>/Data Type Conversion6' */
  uint8_T DataTypeConversion6_j;       /* '<S84>/Data Type Conversion6' */
  uint8_T DataTypeConversion6_n;       /* '<S74>/Data Type Conversion6' */
  uint8_T UnitDelay_a1;                /* '<S19>/Unit Delay' */
  uint8_T DataTypeConversion_p;        /* '<S19>/Data Type Conversion' */
  uint8_T UnitDelay_n;                 /* '<S8>/Unit Delay' */
  uint8_T Sum6_d;                      /* '<S8>/Sum6' */
  uint8_T Add2_o;                      /* '<S8>/Add2' */
  uint8_T BitwiseOperator;             /* '<S8>/Bitwise Operator' */
  uint8_T lkas6;                       /* '<S8>/Data Type Conversion6' */
  uint8_T lkas0;                       /* '<S8>/Data Type Conversion' */
  uint8_T lkas1;                       /* '<S8>/Data Type Conversion1' */
  uint8_T lkas2;                       /* '<S8>/Data Type Conversion2' */
  uint8_T lkas3;                       /* '<S8>/Data Type Conversion3' */
  uint8_T lkas4;                       /* '<S8>/Data Type Conversion4' */
  uint8_T lkas5;                       /* '<S8>/Data Type Conversion5' */
  uint8_T Add2_p;                      /* '<S9>/Add2' */
  uint8_T BitwiseOperator_a;           /* '<S9>/Bitwise Operator' */
  uint8_T DataTypeConversion6_f;       /* '<S9>/Data Type Conversion6' */
  uint8_T DataTypeConversion11;        /* '<Root>/Data Type Conversion11' */
  boolean_T RO1;                       /* '<S12>/RO1' */
  boolean_T RO2;                       /* '<S12>/RO2' */
  boolean_T LO1;                       /* '<S12>/LO1' */
  boolean_T RO3;                       /* '<S12>/RO3' */
  boolean_T RO4;                       /* '<S12>/RO4' */
  boolean_T LO2;                       /* '<S12>/LO2' */
  boolean_T LO3;                       /* '<S12>/LO3' */
  boolean_T RO5;                       /* '<S12>/RO5' */
  boolean_T RO6;                       /* '<S12>/RO6' */
  boolean_T LO4;                       /* '<S12>/LO4' */
  boolean_T Memory;                    /* '<S14>/Memory' */
  boolean_T Logic[2];                  /* '<S14>/Logic' */
  boolean_T RO9;                       /* '<S12>/RO9' */
  boolean_T RO10;                      /* '<S12>/RO10' */
  boolean_T LO7;                       /* '<S12>/LO7' */
  boolean_T RO11;                      /* '<S12>/RO11' */
  boolean_T RO12;                      /* '<S12>/RO12' */
  boolean_T LO8;                       /* '<S12>/LO8' */
  boolean_T LO9;                       /* '<S12>/LO9' */
  boolean_T RO13;                      /* '<S12>/RO13' */
  boolean_T RO14;                      /* '<S12>/RO14' */
  boolean_T LO10;                      /* '<S12>/LO10' */
  boolean_T Memory_b;                  /* '<S15>/Memory' */
  boolean_T Logic_n[2];                /* '<S15>/Logic' */
  boolean_T LO13;                      /* '<S12>/LO13' */
  boolean_T UnitDelay6_g;              /* '<S12>/Unit Delay6' */
  boolean_T LO15;                      /* '<S12>/LO15' */
  boolean_T LO14;                      /* '<S12>/LO14' */
  boolean_T RO21;                      /* '<S12>/RO21' */
  boolean_T RO22;                      /* '<S12>/RO22' */
  boolean_T LO17;                      /* '<S12>/LO17' */
  boolean_T LO16;                      /* '<S12>/LO16' */
  boolean_T LO5;                       /* '<S12>/LO5' */
  boolean_T LO11;                      /* '<S12>/LO11' */
  boolean_T R;                         /* '<S92>/R' */
  boolean_T R_l;                       /* '<S93>/R' */
  boolean_T ROL1;                      /* '<S30>/ROL1' */
  boolean_T LOA;                       /* '<S30>/LOA' */
  boolean_T R_k;                       /* '<S94>/R' */
  boolean_T ROL;                       /* '<S33>/ROL' */
  boolean_T ROL1_p;                    /* '<S33>/ROL1' */
  boolean_T LOA_k;                     /* '<S33>/LOA' */
  boolean_T R_f;                       /* '<S23>/R' */
  boolean_T LA1;                       /* '<S23>/LA1' */
  boolean_T R_h;                       /* '<S84>/R' */
  boolean_T R2;                        /* '<S84>/R2' */
  boolean_T A_g1;                      /* '<S84>/A' */
  boolean_T R1;                        /* '<S84>/R1' */
  boolean_T R3;                        /* '<S84>/R3' */
  boolean_T A1_b;                      /* '<S84>/A1' */
  boolean_T Memory_j;                  /* '<S85>/Memory' */
  boolean_T Logic_a[2];                /* '<S85>/Logic' */
  boolean_T LA;                        /* '<S23>/LA' */
  boolean_T RO1_e;                     /* '<S80>/RO1' */
  boolean_T RelationalOperator5;       /* '<S75>/Relational Operator5' */
  boolean_T LOA_m;                     /* '<S75>/LOA' */
  boolean_T RelationalOperator1;       /* '<S88>/Relational Operator1' */
  boolean_T RelationalOperator2;       /* '<S88>/Relational Operator2' */
  boolean_T Switch1_hs;                /* '<S87>/Switch1' */
  boolean_T RelationalOperator4;       /* '<S87>/Relational Operator4' */
  boolean_T UnitDelay_od;              /* '<S91>/Unit Delay' */
  boolean_T LogicalOperator3;          /* '<S91>/Logical Operator3' */
  boolean_T LogicalOperator2;          /* '<S91>/Logical Operator2' */
  boolean_T RO1_b;                     /* '<S79>/RO1' */
  boolean_T RelationalOperator1_p;     /* '<S19>/Relational Operator1' */
  boolean_T LogicalOperator;           /* '<S19>/Logical Operator' */
  boolean_T RO1_a;                     /* '<S66>/RO1' */
  boolean_T Delay_i;                   /* '<S64>/Delay' */
  boolean_T R_c;                       /* '<S60>/R' */
  boolean_T D2;                        /* '<S60>/D2' */
  boolean_T LogicalOperator2_f;        /* '<S60>/Logical Operator2' */
  boolean_T LogicalOperator1;          /* '<S60>/Logical Operator1' */
  boolean_T R2_e;                      /* '<S64>/R2' */
  boolean_T R_hs;                      /* '<S65>/R' */
  boolean_T RelationalOperator;        /* '<S2>/Relational Operator' */
  boolean_T RelationalOperator5_p;     /* '<S6>/Relational Operator5' */
  boolean_T R1_m;                      /* '<S94>/R1' */
  boolean_T R1_j;                      /* '<S93>/R1' */
  boolean_T R1_p;                      /* '<S92>/R1' */
  boolean_T RelationalOperator6;       /* '<S87>/Relational Operator6' */
  boolean_T RelationalOperator5_j;     /* '<S87>/Relational Operator5' */
  boolean_T LogicalOperator1_m;        /* '<S87>/Logical Operator1' */
  boolean_T RelationalOperator8;       /* '<S87>/Relational Operator8' */
  boolean_T RelationalOperator7;       /* '<S87>/Relational Operator7' */
  boolean_T LogicalOperator4;          /* '<S87>/Logical Operator4' */
  boolean_T LogicalOperator_a;         /* '<S87>/Logical Operator' */
  boolean_T RelationalOperator3;       /* '<S87>/Relational Operator3' */
  boolean_T RO2_l;                     /* '<S80>/RO2' */
  boolean_T RO3_j;                     /* '<S80>/RO3' */
  boolean_T RO2_a;                     /* '<S79>/RO2' */
  boolean_T RO3_a;                     /* '<S79>/RO3' */
  boolean_T R_j;                       /* '<S78>/R' */
  boolean_T R1_h;                      /* '<S78>/R1' */
  boolean_T R_i;                       /* '<S77>/R' */
  boolean_T R1_o;                      /* '<S77>/R1' */
  boolean_T LA_i;                      /* '<S70>/LA' */
  boolean_T RO2_az;                    /* '<S66>/RO2' */
  boolean_T RO3_g;                     /* '<S66>/RO3' */
  boolean_T R1_jl;                     /* '<S65>/R1' */
  boolean_T R_d;                       /* '<S64>/R' */
  boolean_T R1_i;                      /* '<S64>/R1' */
  boolean_T LowerRelop1;               /* '<S49>/LowerRelop1' */
  boolean_T UpperRelop;                /* '<S49>/UpperRelop' */
  boolean_T RO19;                      /* '<S12>/RO19' */
  boolean_T RO18;                      /* '<S12>/RO18' */
  boolean_T RO8;                       /* '<S12>/RO8' */
  boolean_T RO7;                       /* '<S12>/RO7' */
  boolean_T LO6;                       /* '<S12>/LO6' */
  boolean_T RO20;                      /* '<S12>/RO20' */
  boolean_T RO16;                      /* '<S12>/RO16' */
  boolean_T RO15;                      /* '<S12>/RO15' */
  boolean_T LO12;                      /* '<S12>/LO12' */
  boolean_T RO17;                      /* '<S12>/RO17' */
} B_LATCtrl_T;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real_T UnitDelay1_DSTATE;            /* '<S39>/Unit Delay1' */
  real_T D_DSTATE;                     /* '<S92>/D' */
  real_T Delay_DSTATE[20];             /* '<S12>/Delay' */
  real_T UnitDelay_DSTATE;             /* '<S12>/Unit Delay' */
  real_T UnitDelay1_DSTATE_j;          /* '<S12>/Unit Delay1' */
  real_T UnitDelay2_DSTATE;            /* '<S12>/Unit Delay2' */
  real_T D_DSTATE_i;                   /* '<S93>/D' */
  real_T VariableIntegerDelay3_DSTATE[3];/* '<S30>/Variable Integer Delay3' */
  real_T UnitDelay_DSTATE_b;           /* '<S30>/Unit Delay' */
  real_T D_DSTATE_p;                   /* '<S94>/D' */
  real_T UnitDelay1_DSTATE_i;          /* '<S33>/Unit Delay1' */
  real_T VariableIntegerDelay_DSTATE[9];/* '<S30>/Variable Integer Delay' */
  real_T D_DSTATE_l;                   /* '<S78>/D' */
  real_T UnitDelay_DSTATE_p;           /* '<S84>/Unit Delay' */
  real_T D_DSTATE_k;                   /* '<S77>/D' */
  real_T Delay1_DSTATE;                /* '<S72>/Delay1' */
  real_T Delay2_DSTATE[2];             /* '<S72>/Delay2' */
  real_T Delay3_DSTATE[3];             /* '<S72>/Delay3' */
  real_T Delay4_DSTATE[4];             /* '<S72>/Delay4' */
  real_T Delay5_DSTATE[5];             /* '<S72>/Delay5' */
  real_T Delay6_DSTATE[6];             /* '<S72>/Delay6' */
  real_T Delay7_DSTATE[7];             /* '<S72>/Delay7' */
  real_T Delay8_DSTATE[8];             /* '<S72>/Delay8' */
  real_T Delay9_DSTATE[9];             /* '<S72>/Delay9' */
  real_T Delay1_DSTATE_c[100];         /* '<S23>/Delay1' */
  real_T UnitDelay_DSTATE_m;           /* '<S89>/Unit Delay' */
  real_T Delay1_DSTATE_m;              /* '<S82>/Delay1' */
  real_T Delay2_DSTATE_g[2];           /* '<S82>/Delay2' */
  real_T Delay3_DSTATE_f[3];           /* '<S82>/Delay3' */
  real_T Delay4_DSTATE_n[4];           /* '<S82>/Delay4' */
  real_T Delay5_DSTATE_c[5];           /* '<S82>/Delay5' */
  real_T Delay6_DSTATE_k[6];           /* '<S82>/Delay6' */
  real_T Delay7_DSTATE_l[7];           /* '<S82>/Delay7' */
  real_T Delay8_DSTATE_m[8];           /* '<S82>/Delay8' */
  real_T Delay9_DSTATE_h[9];           /* '<S82>/Delay9' */
  real_T Delay1_DSTATE_f;              /* '<S69>/Delay1' */
  real_T Delay2_DSTATE_b[2];           /* '<S69>/Delay2' */
  real_T Delay3_DSTATE_d[3];           /* '<S69>/Delay3' */
  real_T Delay4_DSTATE_f[4];           /* '<S69>/Delay4' */
  real_T Delay5_DSTATE_cr[5];          /* '<S69>/Delay5' */
  real_T Delay6_DSTATE_i[6];           /* '<S69>/Delay6' */
  real_T Delay7_DSTATE_c[7];           /* '<S69>/Delay7' */
  real_T Delay8_DSTATE_f[8];           /* '<S69>/Delay8' */
  real_T Delay9_DSTATE_m[9];           /* '<S69>/Delay9' */
  real_T Delay10_DSTATE;               /* '<S69>/Delay10' */
  real_T Delay11_DSTATE[2];            /* '<S69>/Delay11' */
  real_T Delay12_DSTATE[3];            /* '<S69>/Delay12' */
  real_T Delay13_DSTATE[4];            /* '<S69>/Delay13' */
  real_T Delay14_DSTATE[5];            /* '<S69>/Delay14' */
  real_T Delay15_DSTATE[6];            /* '<S69>/Delay15' */
  real_T Delay16_DSTATE[7];            /* '<S69>/Delay16' */
  real_T Delay17_DSTATE[8];            /* '<S69>/Delay17' */
  real_T Delay18_DSTATE[9];            /* '<S69>/Delay18' */
  real_T UnitDelay5_DSTATE;            /* '<S69>/Unit Delay5' */
  real_T UnitDelay6_DSTATE;            /* '<S69>/Unit Delay6' */
  real_T VariableIntegerDelay3_DSTATE_m[16];/* '<S23>/Variable Integer Delay3' */
  real_T DiscreteTransferFcn_states;   /* '<S70>/Discrete Transfer Fcn' */
  real_T DiscreteTransferFcn2_states;  /* '<S70>/Discrete Transfer Fcn2' */
  real_T DiscreteTransferFcn_states_j; /* '<S71>/Discrete Transfer Fcn' */
  real_T DiscreteTransferFcn2_states_l;/* '<S71>/Discrete Transfer Fcn2' */
  real_T D_DSTATE_a;                   /* '<S64>/D' */
  real_T D1_DSTATE;                    /* '<S58>/D1' */
  real_T D1_DSTATE_i;                  /* '<S61>/D1' */
  real_T Delay_DSTATE_o;               /* '<S47>/Delay' */
  real_T delay_DSTATE[3];              /* '<S48>/delay' */
  real_T delay1_DSTATE[9];             /* '<S48>/delay1' */
  real_T LATC_YawRateStore[25];        /* '<S39>/Data Store Memory' */
  real_T Add2_DWORK1;                  /* '<S33>/Add2' */
  real_T LATC_YawRateArry[20];         /* '<S84>/Data Store Memory' */
  real_T LATC_YawRateStore_i[10];      /* '<S74>/Data Store Memory' */
  real_T DiscreteTransferFcn_tmp;      /* '<S70>/Discrete Transfer Fcn' */
  real_T DiscreteTransferFcn2_tmp;     /* '<S70>/Discrete Transfer Fcn2' */
  real_T DiscreteTransferFcn_tmp_b;    /* '<S71>/Discrete Transfer Fcn' */
  real_T DiscreteTransferFcn2_tmp_h;   /* '<S71>/Discrete Transfer Fcn2' */
  real_T I_avrg;                       /* '<S24>/Chart' */
  real_T Avrg2Store;                   /* '<S24>/Chart' */
  real_T Divide3_DWORK1[3];            /* '<S48>/Divide3' */
  real_T Divide2_DWORK3;               /* '<S48>/Divide2' */
  real_T Divide2_DWORK4;               /* '<S48>/Divide2' */
  real_T Divide2_DWORK5;               /* '<S48>/Divide2' */
  struct {
    void *LoggedData;
  } Scope1_PWORK;                      /* '<S23>/Scope1' */

  int32_T Divide2_DWORK2;              /* '<S48>/Divide2' */
  uint32_T Sum6_DWORK1;                /* '<S39>/Sum6' */
  uint16_T UnitDelay_DSTATE_i;         /* '<S39>/Unit Delay' */
  uint16_T UnitDelay_DSTATE_c;         /* '<S74>/Unit Delay' */
  uint8_T UnitDelay_DSTATE_e;          /* '<S19>/Unit Delay' */
  uint8_T UnitDelay_DSTATE_j;          /* '<S8>/Unit Delay' */
  boolean_T UnitDelay6_DSTATE_o;       /* '<S12>/Unit Delay6' */
  boolean_T UnitDelay_DSTATE_cs;       /* '<S91>/Unit Delay' */
  boolean_T Delay_DSTATE_n[10];        /* '<S64>/Delay' */
  boolean_T D2_DSTATE;                 /* '<S60>/D2' */
  uint8_T icLoad;                      /* '<S30>/Variable Integer Delay3' */
  uint8_T icLoad_p;                    /* '<S30>/Variable Integer Delay' */
  uint8_T icLoad_c;                    /* '<S23>/Variable Integer Delay3' */
  uint8_T is_active_c3_LATCtrl;        /* '<S24>/Chart' */
  uint8_T is_c3_LATCtrl;               /* '<S24>/Chart' */
  boolean_T Memory_PreviousInput;      /* '<S14>/Memory' */
  boolean_T Memory_PreviousInput_j;    /* '<S15>/Memory' */
  boolean_T Memory_PreviousInput_d;    /* '<S85>/Memory' */
  boolean_T LATC_ZeroPosition_MODE;    /* '<S19>/LATC_ZeroPosition' */
} DW_LATCtrl_T;

/* Invariant block signals (auto storage) */
typedef struct {
  const real_T MatrixConcatenate3[2];  /* '<S39>/Matrix Concatenate3' */
  const real_T MatrixConcatenate4[2];  /* '<S38>/Matrix Concatenate4' */
  const real_T MatrixConcatenate5[2];  /* '<S38>/Matrix Concatenate5' */
  const real_T Product3;               /* '<S3>/Product3' */
  const real_T TmpSignalConversionAtMathFu[3];
  const real_T MathFunction[3];        /* '<S36>/Math Function' */
  const real_T TmpSignalConversionAtMath_j[3];
  const real_T MathFunction2[3];       /* '<S36>/Math Function2' */
  const real_T LATC_numStZeroCntrr;    /* '<S19>/Constant2' */
  const real_T Add;                    /* '<S56>/Add' */
  const real_T DataTypeConversion5;    /* '<Root>/Data Type Conversion5' */
  const real_T Add1;                   /* '<S56>/Add1' */
  const real_T Add2;                   /* '<S56>/Add2' */
  const real_T Divide2;                /* '<S56>/Divide2' */
  const real_T Product;                /* '<S56>/Product' */
  const real_T Add3;                   /* '<S56>/Add3' */
  const real_T MinMax;                 /* '<S56>/MinMax' */
  const real_T MinMax1;                /* '<S56>/MinMax1' */
  const real_T Add4;                   /* '<S56>/Add4' */
  const real_T DataTypeConversion4;    /* '<Root>/Data Type Conversion4' */
  const real_T Switch3;                /* '<S55>/Switch3' */
  const real_T Divide2_f;              /* '<S55>/Divide2' */
  const uint32_T Gain;                 /* '<S62>/Gain' */
  const boolean_T RelationalOperator1; /* '<S87>/Relational Operator1' */
  const boolean_T RelationalOperator2; /* '<S87>/Relational Operator2' */
  const boolean_T LogicalOperator3;    /* '<S87>/Logical Operator3' */
} ConstB_LATCtrl_T;

/* Constant parameters (auto storage) */
typedef struct {
  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S32>/Constant'
   *   '<S32>/Constant1'
   *   '<S48>/delay1'
   */
  real_T pooled7[9];

  /* Expression: [0.0821962819, 0.0821962819, 0.0821962819, 0.0821962819, 0.0821962819, 0.0821962819, 0.0821962819, 0.0821962819, 0.0821962819, 0.0821962819, 0.0821962819, 0.0821962819, 0.0821962819, 0.0821962819, 0.0821962819, 0.0821962819, 0.0878896606, 0.0935305920, 0.0991204128, 0.104653454, 0.1101218185, 0.1155178824, 0.1208353266, 0.1260693506, 0.1312165111, 0.1362744432, 0.1412415903, 0.1461169924, 0.1509001367, 0.1555908611, 0.1601892935, 0.1646958141, 0.1691110297, 0.173435754, 0.1776709894, 0.1818164161, 0.1858739999, 0.1898469667, 0.193735991, 0.1975429794, 0.2012696632, 0.204917848, 0.208489070, 0.2119852054, 0.2154088325, 0.2187608661, 0.2220438939, 0.2252594774, 0.2284094959, 0.2314961638, 0.2345205909, 0.2374852893, 0.2403916507, 0.2432413864, 0.246036165, 0.2487776111, 0.2514673041, 0.2541071622, 0.2566979092, 0.2592417546, 0.2617393259, 0.2641927481, 0.2666025529, 0.2689707844, 0.2712978809, 0.2735858104, 0.275835318, 0.2780471077, 0.2802230355, 0.2823637285, 0.2844697753, 0.2865429312, 0.2885837153, 0.2905930153, 0.2925712837, 0.2945201547, 0.2964400207, 0.2983316503, 0.300195785, 0.3020331404, 0.3038444069, 0.3056298452, 0.3073909103, 0.3091278166, 0.3108411634, 0.3125315294, 0.3141994735, 0.3158455355, 0.3174702369, 0.3190740816, 0.3206575565, 0.3222211325, 0.3237652647, 0.3252903932, 0.3267969437, 0.3282853281, 0.329755945, 0.331209180, 0.3326454066, 0.3340649864, 0.3354682696, 0.3368555955, 0.3382272927, 0.339583680, 0.3409250662, 0.3422517508, 0.3435640243, 0.3448621684, 0.3461464567, 0.3474171546, 0.3486745196]
   * Referenced by: '<S23>/K4_Full'
   */
  real_T K4_Full_tableData[111];

  /* Pooled Parameter (Expression: [0.0 1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0 10.0 11.0 12.0 13.0 14.0 15.0 16.0 17.0 18.0 19.0 20.0 21.0 22.0 23.0 24.0 25.0 26.0 27.0 28.0 29.0 30.0 31.0 32.0 33.0 34.0 35.0 36.0 37.0 38.0 39.0 40.0 41.0 42.0 43.0 44.0 45.0 46.0 47.0 48.0 49.0 50.0 51.0 52.0 53.0 54.0 55.0 56.0 57.0 58.0 59.0 60.0 61.0 62.0 63.0 64.0 65.0 66.0 67.0 68.0 69.0 70.0 71.0 72.0 73.0 74.0 75.0 76.0 77.0 78.0 79.0 80.0 81.0 82.0 83.0 84.0 85.0 86.0 87.0 88.0 89.0 90.0 91.0 92.0 93.0 94.0 95.0 96.0 97.0 98.0 99.0 100.0 101.0 102.0 103.0 104.0 105.0 106.0 107.0 108.0 109.0 110.0 ])
   * Referenced by:
   *   '<S23>/K4_Full'
   *   '<S23>/K4_empty'
   */
  real_T pooled9[111];

  /* Expression: [0.6509204418, 0.6509204418, 0.6509204418, 0.6509204418, 0.6509204418, 0.6509204418, 0.6694508985, 0.6862373033, 0.701379243, 0.7150454203, 0.7274876752, 0.7388997852, 0.7494101983, 0.7591526793, 0.7682382522, 0.7767575845, 0.7847946479, 0.7924228196, 0.7997048556, 0.8066949268, 0.8134372333, 0.8199709145, 0.8263261168, 0.8325287719, 0.838600216, 0.8445579473, 0.8504175748, 0.8561881605, 0.8618817689, 0.8675047464, 0.8730632466, 0.8785621898, 0.8840054823, 0.8893961984, 0.8947367319, 0.900028921, 0.9052741533, 0.9087454056]
   * Referenced by: '<S23>/K3_Full'
   */
  real_T K3_Full_tableData[38];

  /* Pooled Parameter (Expression: [ 0.0 3.0 6.0 9.0 12.0 15.0 18.0 21.0 24.0 27.0 30.0 33.0 36.0 39.0 42.0 45.0 48.0 51.0 54.0 57.0 60.0 63.0 66.0 69.0 72.0 75.0 78.0 81.0 84.0 87.0 90.0 93.0 96.0 99.0 102.0 105.0 108.0 110.0 ])
   * Referenced by:
   *   '<S23>/K3_Full'
   *   '<S23>/K3_empty'
   */
  real_T pooled10[38];

  /* Expression: [0.08 0.08]
   * Referenced by: '<S23>/K2_Full'
   */
  real_T K2_Full_tableData[2];

  /* Pooled Parameter (Expression: [0.0 110.0])
   * Referenced by:
   *   '<S23>/K2_Full'
   *   '<S23>/K2_empty'
   */
  real_T pooled11[2];

  /* Expression: [0.0535484967, 0.0535484967, 0.0535484967, 0.0535484967, 0.0541504661, 0.0543719523, 0.0543824385, 0.0543455306, 0.0543008546, 0.0542601140, 0.0542236192, 0.0541910844, 0.0541619481, 0.0541356594, 0.0541116458, 0.0540898161, 0.0540699123, 0.0540512537, 0.0540341137, 0.0540181068, 0.0540031027, 0.0539889916, 0.0539756806]
   * Referenced by: '<S23>/K1_Full'
   */
  real_T K1_Full_tableData[23];

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S23>/K1_Full'
   *   '<S23>/K1_empty'
   */
  real_T pooled12[23];

  /* Expression: [0.1446050509, 0.1446050509, 0.1446050509, 0.1446050509, 0.1446050509, 0.1446050509, 0.1446050509, 0.1446050509, 0.1446050509, 0.1446050509, 0.1446050509, 0.1446050509, 0.1446050509, 0.1446050509, 0.1446050509, 0.1446050509, 0.1517390847, 0.158450373, 0.1647769669, 0.1707479211, 0.1763869514, 0.1817146446, 0.1867497127, 0.1915096563, 0.1960110826, 0.2002698344, 0.2043010263, 0.2081190402, 0.2117375109, 0.2151693145, 0.218426567, 0.2215206327, 0.2244621421, 0.2272610173, 0.2299265038, 0.2324672044, 0.2348911167, 0.2372056704, 0.2394177647, 0.2415338052, 0.2435597387, 0.2455010872, 0.2473629789, 0.2491501785, 0.2508671147, 0.2525179056, 0.2541063831, 0.2556361147, 0.2571104234, 0.2585324073, 0.259904956, 0.2612307669, 0.2625123595, 0.2637520892, 0.2649521589, 0.2661146308, 0.2672414368, 0.2683343876, 0.2693951818, 0.270425414, 0.2714265819, 0.2724000935, 0.2733472729, 0.2742693668, 0.275167549, 0.2760429261, 0.2768965415, 0.277729380, 0.2785423714, 0.2793363941, 0.2801122786, 0.2808708107, 0.2816127338, 0.2823387524, 0.2830495335, 0.2837457099, 0.2844278818, 0.2850966188, 0.2857524618, 0.2863959246, 0.2870274959, 0.2876476403, 0.2882567998, 0.2888553953, 0.2894438277, 0.2900224789, 0.2905917131, 0.2911518775, 0.2917033035, 0.2922463074, 0.2927811913, 0.2933082437, 0.2938277405, 0.2943399453, 0.2948451106, 0.2953434776, 0.2958352774, 0.2963207314, 0.2968000516, 0.2972734411, 0.2977410947, 0.2982031992, 0.2986599338, 0.2991114705, 0.2995579744, 0.299999604, 0.3004365118, 0.300868844, 0.3012967413, 0.3017203391, 0.3021397675]
   * Referenced by: '<S23>/K4_empty'
   */
  real_T K4_empty_tableData[111];

  /* Expression: [0.5416993741, 0.5416993741, 0.5416993741, 0.5416993741, 0.5416993741, 0.5416993741, 0.5354910333, 0.5267243234, 0.5168844545, 0.5065727064, 0.4961665326, 0.4859224797, 0.4760064341, 0.4665171731, 0.4575065305, 0.4489949605, 0.4409826955, 0.4334574132, 0.4263993737, 0.4197848055, 0.4135881053, 0.407783243, 0.4023446393, 0.3972476935, 0.3924690804, 0.3879868979, 0.3837807164, 0.3798315672, 0.3761218931, 0.3726354773, 0.3693573597, 0.3662737501, 0.3633719397, 0.3606402158, 0.3580677806, 0.3556446743, 0.3533617044, 0.3519133581]
   * Referenced by: '<S23>/K3_empty'
   */
  real_T K3_empty_tableData[38];

  /* Expression: [0.1 0.1]
   * Referenced by: '<S23>/K2_empty'
   */
  real_T K2_empty_tableData[2];

  /* Expression:  [0.0577231420, 0.0577231420, 0.0577231420, 0.0577231420, 0.0580090282, 0.0583215817, 0.0585045995, 0.0586022205, 0.0586548942, 0.0586843682, 0.0587014919, 0.0587117503, 0.0587180227, 0.0587218868, 0.0587242436, 0.0587256276, 0.0587263669, 0.0587266691, 0.0587266691, 0.0587264567, 0.0587260928, 0.0587256196, 0.0587250667]
   * Referenced by: '<S23>/K1_empty'
   */
  real_T K1_empty_tableData[23];

  /* Expression: [1 1 2 2.5 2.5]
   * Referenced by: '<S73>/1-D Lookup Table4'
   */
  real_T DLookupTable4_tableDa[5];

  /* Pooled Parameter (Expression: [0 0.004 0.008 0.015 0.02])
   * Referenced by:
   *   '<S73>/1-D Lookup Table1'
   *   '<S73>/1-D Lookup Table3'
   *   '<S73>/1-D Lookup Table4'
   */
  real_T pooled13[5];

  /* Expression: [1 1 0.8 0.5 0.5]
   * Referenced by: '<S73>/1-D Lookup Table1'
   */
  real_T DLookupTable1_tableDa[5];

  /* Expression: [1.1 1.1 0.5 0.4 0.4]
   * Referenced by: '<S73>/1-D Lookup Table3'
   */
  real_T DLookupTable3_tableDa[5];

  /* Pooled Parameter (Expression: [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1])
   * Referenced by:
   *   '<S67>/Constant2'
   *   '<S68>/Constant2'
   */
  real_T pooled24[16];

  /* Pooled Parameter (Expression: [0 1;1 0;0 1;0 1;1 0;1 0;0 0;0 0])
   * Referenced by:
   *   '<S14>/Logic'
   *   '<S15>/Logic'
   *   '<S85>/Logic'
   */
  boolean_T pooled47[16];
} ConstP_LATCtrl_T;

/* Imported (extern) block signals */
extern real_T DE_aLatAcc;              /* '<Root>/DE_aLatAcc' */
extern real_T VMP_lLatPosn4TgtPath;    /* '<Root>/VMP_lLatPosn4TgtPath ' */
extern real_T VMP_agHeadAngle4TgtPath; /* '<Root>/VMP_agHeadAngle4TgtPath' */
extern real_T VMP_cCv4TgtPath;         /* '<Root>/VMP_cCv4TgtPath' */
extern real_T DE_wVehYawRate;          /* '<Root>/DE_wVehYawRate' */
extern real_T DE_vVehSpdKph;           /* '<Root>/DE_vVehSpdKph' */
extern uint8_T CoST_stADMoEna;         /* '<Root>/CoST_stADMoEna' */
extern real_T DE_phiSteerAngle;        /* '<Root>/DE_phiSteerAngle' */
extern uint8_T COEHPS_LKADemdRsp;      /* '<Root>/COEHPS_LKADemdRsp' */
extern real_T VMP_lLaneDist0;          /* '<Root>/VMP_lLaneDist0' */
extern real_T VMP_Lane0Cv1;            /* '<Root>/VMP_Lane0Cv1' */
extern real_T VMP_Lane0Cv2;            /* '<Root>/VMP_Lane0Cv2' */
extern real_T VMP_Lane0Cv3;            /* '<Root>/VMP_Lane0Cv3' */
extern real_T VMP_lLaneDist1;          /* '<Root>/VMP_lLaneDist1' */
extern real_T VMP_Lane1Cv1;            /* '<Root>/VMP_Lane1Cv1' */
extern real_T VMP_Lane1Cv2;            /* '<Root>/VMP_Lane1Cv2' */
extern real_T VMP_Lane1Cv3;            /* '<Root>/VMP_Lane1Cv3' */
extern real_T VMP_cTgtPathCv0;         /* '<Root>/VMP_cTgtPathCv0' */
extern real_T VMP_cTgtPathCv1;         /* '<Root>/VMP_cTgtPathCv1' */
extern real_T VMP_cTgtPathCv2;         /* '<Root>/VMP_cTgtPathCv2' */
extern real_T VMP_cTgtPathCv3;         /* '<Root>/VMP_cTgtPathCv3' */
extern real_T VMP_cTgtPathCv4;         /* '<Root>/VMP_cTgtPathCv4' */
extern real_T VMP_cTgtPathCv5;         /* '<Root>/VMP_cTgtPathCv5' */
extern real_T VMP_lLgtDist2PathZero;   /* '<Root>/VMP_lLgtDist2PathZero' */
extern uint8_T VMP_stLane0Prob;        /* '<Root>/VMP_stLane0Prob' */
extern uint8_T VMP_stLane1Prob;        /* '<Root>/VMP_stLane1Prob' */
extern real_T DE_mVehMass;             /* '<Root>/DE_mVehMass' */
extern uint8_T DE_stTrlr;              /* '<Root>/DE_stTrlr' */
extern int32_T ADCU_tTiDely4MCU2RCAR;  /* '<Root>/ADCU_tTiDely4MCU2RCAR' */
extern real_T DE_aLonAcc;              /* '<Root>/DE_aLonAcc' */
extern real_T DE_aVertAcc;             /* '<Root>/DE_aVertAcc' */
extern int32_T VMP_LC_Flag;            /* '<Root>/VMP_LC_Flag' */

/* Block signals (auto storage) */
extern B_LATCtrl_T LATCtrl_B;

/* Block states (auto storage) */
extern DW_LATCtrl_T LATCtrl_DW;

/* ConstVolatile memory section */
extern const volatile ConstB_LATCtrl_T LATCtrl_ConstB;/* constant block i/o */

/* Constant parameters (auto storage) */
/* ConstVolatile memory section */
extern const volatile ConstP_LATCtrl_T LATCtrl_ConstP;

/* Model entry point functions */
extern void LATCtrl_initialize(void);
extern void LATCtrl_step(void);

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
 * '<Root>' : 'LATCtrl'
 * '<S1>'   : 'LATCtrl/CAN_HPS'
 * '<S2>'   : 'LATCtrl/CoEHPS'
 * '<S3>'   : 'LATCtrl/LATC_DelyCycleCalc'
 * '<S4>'   : 'LATCtrl/LATC_IP'
 * '<S5>'   : 'LATCtrl/LKA_SteerCtrl'
 * '<S6>'   : 'LATCtrl/PrevDist2Lane4LQR4'
 * '<S7>'   : 'LATCtrl/CAN_HPS/Exc_10'
 * '<S8>'   : 'LATCtrl/CAN_HPS/Exc_10/ExC1'
 * '<S9>'   : 'LATCtrl/CAN_HPS/Exc_10/ExC1/CRC_Check'
 * '<S10>'  : 'LATCtrl/LATC_IP/LATC_LaneIP'
 * '<S11>'  : 'LATCtrl/LATC_IP/LATC_MpuIP'
 * '<S12>'  : 'LATCtrl/LATC_IP/LATC_LaneIP/IP_LaneSignalCheck'
 * '<S13>'  : 'LATCtrl/LATC_IP/LATC_LaneIP/IP_LaneSignalInput'
 * '<S14>'  : 'LATCtrl/LATC_IP/LATC_LaneIP/IP_LaneSignalCheck/S-R Flip-Flop'
 * '<S15>'  : 'LATCtrl/LATC_IP/LATC_LaneIP/IP_LaneSignalCheck/S-R Flip-Flop1'
 * '<S16>'  : 'LATCtrl/LKA_SteerCtrl/Fac4CLCalc'
 * '<S17>'  : 'LATCtrl/LKA_SteerCtrl/K_Filt'
 * '<S18>'  : 'LATCtrl/LKA_SteerCtrl/K_Filt1'
 * '<S19>'  : 'LATCtrl/LKA_SteerCtrl/LATC_ZeroIdfy'
 * '<S20>'  : 'LATCtrl/LKA_SteerCtrl/LC_Module4LQR'
 * '<S21>'  : 'LATCtrl/LKA_SteerCtrl/LKA_ImportCalc'
 * '<S22>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerAgCtrl'
 * '<S23>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerCalc4LQR'
 * '<S24>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerSelfLearn'
 * '<S25>'  : 'LATCtrl/LKA_SteerCtrl/PrevDist2Lane4LQR1'
 * '<S26>'  : 'LATCtrl/LKA_SteerCtrl/PrevDist2Lane4LQR2'
 * '<S27>'  : 'LATCtrl/LKA_SteerCtrl/PrevDist2Lane4LQR3'
 * '<S28>'  : 'LATCtrl/LKA_SteerCtrl/PrevDist2Lane4LQR4'
 * '<S29>'  : 'LATCtrl/LKA_SteerCtrl/K_Filt/KalOut'
 * '<S30>'  : 'LATCtrl/LKA_SteerCtrl/K_Filt/Kalman'
 * '<S31>'  : 'LATCtrl/LKA_SteerCtrl/K_Filt/LATC_ParamPredict'
 * '<S32>'  : 'LATCtrl/LKA_SteerCtrl/K_Filt/Kalman/ICQR'
 * '<S33>'  : 'LATCtrl/LKA_SteerCtrl/K_Filt/Kalman/K_Enb'
 * '<S34>'  : 'LATCtrl/LKA_SteerCtrl/K_Filt/Kalman/LQR_solver'
 * '<S35>'  : 'LATCtrl/LKA_SteerCtrl/K_Filt/Kalman/PrevState'
 * '<S36>'  : 'LATCtrl/LKA_SteerCtrl/K_Filt/Kalman/YacobMatrix'
 * '<S37>'  : 'LATCtrl/LKA_SteerCtrl/K_Filt/Kalman/measurement'
 * '<S38>'  : 'LATCtrl/LKA_SteerCtrl/K_Filt/LATC_ParamPredict/LATC_MotABMatrx'
 * '<S39>'  : 'LATCtrl/LKA_SteerCtrl/K_Filt/LATC_ParamPredict/LATC_MotPredict'
 * '<S40>'  : 'LATCtrl/LKA_SteerCtrl/K_Filt/LATC_ParamPredict/LATC_MotPredict/Predict'
 * '<S41>'  : 'LATCtrl/LKA_SteerCtrl/K_Filt1/PrevDist2Lane4LQR1'
 * '<S42>'  : 'LATCtrl/LKA_SteerCtrl/K_Filt1/PrevDist2Lane4LQR2'
 * '<S43>'  : 'LATCtrl/LKA_SteerCtrl/K_Filt1/PrevDist2Lane4LQR3'
 * '<S44>'  : 'LATCtrl/LKA_SteerCtrl/K_Filt1/PrevDist2Lane4LQR4'
 * '<S45>'  : 'LATCtrl/LKA_SteerCtrl/K_Filt1/PrevDist2Lane4LQR5'
 * '<S46>'  : 'LATCtrl/LKA_SteerCtrl/K_Filt1/PrevDist2Lane4LQR6'
 * '<S47>'  : 'LATCtrl/LKA_SteerCtrl/LATC_ZeroIdfy/LATC_ZeroPosition'
 * '<S48>'  : 'LATCtrl/LKA_SteerCtrl/LATC_ZeroIdfy/LATC_ZeroPosition/RLS'
 * '<S49>'  : 'LATCtrl/LKA_SteerCtrl/LATC_ZeroIdfy/LATC_ZeroPosition/RLS/Saturation Dynamic1'
 * '<S50>'  : 'LATCtrl/LKA_SteerCtrl/LC_Module4LQR/PrevDist2Lane4LC'
 * '<S51>'  : 'LATCtrl/LKA_SteerCtrl/LC_Module4LQR/PrevDist2Lane4LQR2'
 * '<S52>'  : 'LATCtrl/LKA_SteerCtrl/LC_Module4LQR/PrevDist2Lane4LQR3'
 * '<S53>'  : 'LATCtrl/LKA_SteerCtrl/LKA_ImportCalc/LKA_StAngleTrans'
 * '<S54>'  : 'LATCtrl/LKA_SteerCtrl/LKA_ImportCalc/LKA_StAngleTrans1'
 * '<S55>'  : 'LATCtrl/LKA_SteerCtrl/LKA_ImportCalc/LKA_StYawRateTrans'
 * '<S56>'  : 'LATCtrl/LKA_SteerCtrl/LKA_ImportCalc/LKA_VehCentr2LaneCalc'
 * '<S57>'  : 'LATCtrl/LKA_SteerCtrl/LKA_ImportCalc/LKA_YawRateTrans'
 * '<S58>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerAgCtrl/LKA_AgSteerFlatCalc'
 * '<S59>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerAgCtrl/LKA_AgSteerPost'
 * '<S60>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerAgCtrl/LKA_YRFlatCalc'
 * '<S61>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerAgCtrl/LKA_AgSteerFlatCalc/LKA_ACAccTimeCalc'
 * '<S62>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerAgCtrl/LKA_AgSteerPost/LATC_AGSteerRawCalc'
 * '<S63>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerAgCtrl/LKA_AgSteerPost/LATC_AgSteerComp'
 * '<S64>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerAgCtrl/LKA_AgSteerPost/LKA_AgSteerRateLmt'
 * '<S65>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerAgCtrl/LKA_AgSteerPost/LKA_AgSteerRatePOST4LQR'
 * '<S66>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerAgCtrl/LKA_AgSteerPost/SignalRelax'
 * '<S67>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerCalc4LQR/ABMatCalc'
 * '<S68>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerCalc4LQR/ABMatCalc1'
 * '<S69>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerCalc4LQR/DistErrfilter'
 * '<S70>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerCalc4LQR/EmptyComp'
 * '<S71>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerCalc4LQR/FullComp'
 * '<S72>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerCalc4LQR/HeadErrfilter'
 * '<S73>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerCalc4LQR/LATC_ErrComp'
 * '<S74>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerCalc4LQR/LATC_MotPredict'
 * '<S75>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerCalc4LQR/LQR_ParamCalc'
 * '<S76>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerCalc4LQR/LQR_solver'
 * '<S77>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerCalc4LQR/RateLmt'
 * '<S78>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerCalc4LQR/RateLmt1'
 * '<S79>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerCalc4LQR/SignalRelax'
 * '<S80>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerCalc4LQR/SignalRelax1'
 * '<S81>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerCalc4LQR/Subsystem1'
 * '<S82>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerCalc4LQR/YawErrfilter'
 * '<S83>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerCalc4LQR/DistErrfilter/MATLAB Function'
 * '<S84>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerCalc4LQR/LATC_ErrComp/LKA_YawRateFCCalc'
 * '<S85>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerCalc4LQR/LATC_ErrComp/LKA_YawRateFCCalc/S-R Flip-Flop'
 * '<S86>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerCalc4LQR/LATC_MotPredict/Predict'
 * '<S87>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerSelfLearn/AvrgActvCalc'
 * '<S88>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerSelfLearn/AvrgEnbCalc'
 * '<S89>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerSelfLearn/Avrg_Post'
 * '<S90>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerSelfLearn/Chart'
 * '<S91>'  : 'LATCtrl/LKA_SteerCtrl/LKA_SteerSelfLearn/Avrg_Post/Subsystem1'
 * '<S92>'  : 'LATCtrl/LKA_SteerCtrl/PrevDist2Lane4LQR1/Subsystem'
 * '<S93>'  : 'LATCtrl/LKA_SteerCtrl/PrevDist2Lane4LQR2/Subsystem'
 * '<S94>'  : 'LATCtrl/LKA_SteerCtrl/PrevDist2Lane4LQR3/Subsystem'
 */
#endif                                 /* RTW_HEADER_LATCtrl_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
