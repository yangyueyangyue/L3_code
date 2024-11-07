/** @copyright Copyright (c) 2018-2021 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       longitudinal_control_df_d17.c
 * @brief      纵向控制器(DF_D17)
 * @details    实现纵向控制器(DF_D17)
 *
 * @author     pengc
 * @date       2021.07.28
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

/******************************************************************************/
/* 头文件                                                                      */
/******************************************************************************/
#include "df_d17_b2/longitudinal_control_df_d17_b2_c.h"

#include "utils/log_c.h"
#include "utils/com_utils_c.h"
#include "utils/com_clock_c.h"
#include "utils/linear_interpolation_c.h"
#include "math/math_utils_c.h"


/******************************************************************************/
/* 全局及静态变量                                                               */
/******************************************************************************/

// 根据当前车速，限制最大油门量(Key table & gain table)
static const Float32_t s_key_tab_lmt_acc_by_spd[LON_CTL_PID_LERP_TABLE_SIZE] = {
    0.0F/3.6F,  10.0F/3.6F,  20.0F/3.6F,  30.0F/3.6F,  40.0F/3.6F,
   50.0F/3.6F,  60.0F/3.6F,  70.0F/3.6F,  80.0F/3.6F,  90.0F/3.6F,
  100.0F/3.6F, 110.0F/3.6F, 120.0F/3.6F, 130.0F/3.6F, 140.0F/3.6F,
  150.0F/3.6F, 160.0F/3.6F, 170.0F/3.6F, 180.0F/3.6F, 190.0F/3.6F,
  200.0F/3.6F, 210.0F/3.6F, 220.0F/3.6F, 230.0F/3.6F, 240.0F/3.6F
};
// 最大油门量
static const Float32_t s_value_tab_lmt_acc_by_spd[LON_CTL_PID_LERP_TABLE_SIZE] = {
   30.0F,  40.0F,  80.0F,  80.0F,  80.0F,
   80.0F,  80.0F,  80.0F,  80.0F,  90.0F,
  100.0F, 100.0F, 100.0F, 100.0F, 100.0F,
  100.0F, 100.0F, 100.0F, 100.0F, 100.0F,
  100.0F, 100.0F, 100.0F, 100.0F, 100.0F
};


// 通过重量修正速度误差补偿值(Key table & i-gain table)
static const Float32_t s_key_tab_crt_spd_err_feed_by_weight[LON_CTL_PID_LERP_TABLE_SIZE] = {
      0.0F, 10000.0F, 12000.0F, 14000.0F, 16000.0F,
  18000.0F, 20000.0F, 22000.0F, 24000.0F, 26000.0F,
  28000.0F, 30000.0F, 32000.0F, 34000.0F, 36000.0F,
  38000.0F, 40000.0F, 42000.0F, 44000.0F, 46000.0F,
  48000.0F, 50000.0F, 52000.0F, 56000.0F, 60000.0F
};
// I
static const Float32_t s_i_tab_crt_spd_err_feed_by_weight[LON_CTL_PID_LERP_TABLE_SIZE] = {
  0.500F, 0.500F, 0.525F, 0.550F, 0.575F,
  0.600F, 0.625F, 0.650F, 0.675F, 0.700F,
  0.725F, 0.750F, 0.775F, 0.800F, 0.825F,
  0.850F, 0.875F, 0.900F, 0.925F, 0.950F,
  0.975F, 1.000F, 1.000F, 1.000F, 1.000F
};


// 设置速度误差的等级（从负大到正大排列等级）
static const Float32_t s_tab_spd_err_level[LON_CTL_PID_SPD_ERR_FEED_TABLE_SIZE-1] = {
/*        0             1            2            3             4            5            6            7             8           */
/*   0           1            2            3             4            5            6             7            8            9     */
/* 负大3  ] [   负大2   ] [   负大1   ] [   负中3   ] [   负中2   ] [   负中1   ] [   负小3   ] [   负小2   ] [   负小1   ] [   负零    */
      -50.0F/3.6F, -35.0F/3.6F, -25.0F/3.6F, -15.0F/3.6F, -10.0F/3.6F, -5.00F/3.6F, -2.00F/3.6F, -1.00F/3.6F, -0.50F/3.6F,

/*        9            */
/*              10     */
/* 负零   ] [   正零    */
       0.0F/3.6F,

/*        10            11          12           13           14           15           16            17            18           */
/*              11           12           13           14            15           16           17            18            19    */
/* 正零   ] [   正小1   ] [   正小2   ] [   正小3   ] [   正中1   ] [   正中2   ] [   正中3   ] [   正大1   ] [   正大2   ] [   正大3   */
      +0.50F/3.6F, +1.00F/3.6F, +2.00F/3.6F, +5.00F/3.6F, +10.0F/3.6F, +15.0F/3.6F, +25.0F/3.6F, +35.0F/3.6F, +50.0F/3.6F
};

// 设置加速度的等级（从负大到正大排列等级）
static const Float32_t s_tab_spd_chg_level[LON_CTL_PID_SPD_ERR_FEED_TABLE_SIZE-1] = {
/*        0             1            2            3             4            5            6            7             8           */
/*   0           1            2            3             4            5            6             7            8            9     */
/* 负大3  ] [   负大2   ] [   负大1   ] [   负中3   ] [   负中2   ] [   负中1   ] [   负小3   ] [   负小2   ] [   负小1   ] [   负零    */
      -8.00F/3.6F, -7.00F/3.6F, -6.00F/3.6F, -5.00F/3.6F, -4.00F/3.6F, -3.00F/3.6F, -2.00F/3.6F, -1.00F/3.6F, -0.50F/3.6F,

/*        9            */
/*              10     */
/* 负零   ] [   正零    */
       0.0F/3.6F,

/*        10            11          12           13           14           15           16            17            18           */
/*              11           12           13           14            15           16           17            18            19    */
/* 正零   ] [   正小1   ] [   正小2   ] [   正小3   ] [   正中1   ] [   正中2   ] [   正中3   ] [   正大1   ] [   正大2   ] [   正大3   */
      +0.50F/3.6F, +1.00F/3.6F, +2.00F/3.6F, +3.00F/3.6F, +4.00F/3.6F, +5.00F/3.6F, +6.00F/3.6F, +7.00F/3.6F, +8.00F/3.6F
};

// 速度误差误差补偿系数表
// I
static const Float32_t s_i_gain_tab_spd_err_feed[LON_CTL_PID_SPD_ERR_FEED_TABLE_SIZE*LON_CTL_PID_SPD_ERR_FEED_TABLE_SIZE] = {
/* 减速度高     0     1     2     3     4     5     6     7     8     9      10    11    12    13    14    15    16    17    18    19  加速度高  */
/*            -9    -8    -7    -6    -5    -4    -3    -2    -1    -0      +0    +1    +2    +3    +4    +5    +6    +7    +8    +9          */
/* 未达到目标速度 */
/* 0*//*-9*/ +0.16,+0.16,+0.16,+0.16,+0.16,+0.16,+0.16,+0.16,+0.16,+0.16,  +0.16,+0.16,+0.16,+0.16,+0.16,+0.10,+0.06,+0.02,+0.00,-0.06,
/* 1*//*-8*/ +0.16,+0.16,+0.16,+0.16,+0.16,+0.16,+0.16,+0.16,+0.16,+0.16,  +0.16,+0.16,+0.16,+0.16,+0.10,+0.10,+0.04,+0.00,-0.06,-0.20,
/* 2*//*-7*/ +0.16,+0.16,+0.16,+0.16,+0.16,+0.16,+0.16,+0.16,+0.16,+0.16,  +0.16,+0.10,+0.10,+0.10,+0.00,-0.02,-0.06,-0.16,-0.20,-0.20,
/* 3*//*-6*/ +0.40,+0.40,+0.40,+0.40,+0.40,+0.40,+0.40,+0.40,+0.40,+0.40,  +0.20,+0.16,+0.10,+0.00,-0.06,-0.16,-0.20,-0.30,-0.40,-0.40,
/* 4*//*-5*/ +0.60,+0.60,+0.60,+0.60,+0.60,+0.60,+0.60,+0.60,+0.60,+0.40,  +0.40,+0.20,+0.00,-0.20,-0.40,-0.40,-0.40,-0.60,-0.60,-0.80,
/* 5*//*-4*/ +0.80,+0.80,+0.80,+0.80,+0.80,+0.80,+0.80,+0.80,+0.80,+0.60,  +0.20,+0.00,-0.16,-0.20,-0.40,-0.60,-0.80,-1.00,-1.20,-1.40,
/* 6*//*-3*/ +1.00,+1.00,+1.00,+1.00,+1.00,+1.00,+1.00,+1.00,+1.00,+0.60,  +0.30,+0.00,-0.60,-0.80,-1.00,-1.60,-1.80,-1.80,-1.80,-1.80,
/* 7*//*-2*/ +1.00,+1.00,+1.00,+1.00,+1.00,+1.00,+1.00,+1.00,+1.00,+0.80,  +0.00,-1.20,-1.60,-1.80,-2.00,-2.20,-2.40,-2.40,-2.40,-2.40,
/* 8*//*-1*/ +1.00,+1.00,+1.00,+1.00,+1.00,+1.00,+1.00,+1.00,+1.00,+0.80,  +0.00,-0.80,-0.80,-0.80,-1.00,-1.20,-1.40,-1.40,-1.40,-1.40,
/* 9*//*-0*/ +1.00,+1.00,+1.00,+1.00,+1.00,+1.00,+1.00,+1.00,+1.00,+0.80,  +0.00,-0.80,-0.80,-0.80,-1.00,-1.20,-1.40,-1.40,-1.40,-1.40,
/* 已达到目标速度 */
/*10*//*+0*/ +1.00,+1.00,+1.00,+1.00,+1.00,+1.00,+1.00,+1.00,+0.40,+0.00,  -0.60,-0.60,-0.80,-0.80,-1.00,-1.20,-1.40,-1.40,-1.40,-1.40,
/*11*//*+1*/ +1.00,+1.00,+1.00,+1.00,+1.00,+1.00,+1.00,+1.00,+0.40,+0.00,  -0.60,-0.60,-0.80,-0.80,-1.00,-1.20,-1.40,-1.40,-1.40,-1.40,
/*12*//*+2*/ +1.00,+1.00,+1.00,+1.00,+1.00,+1.00,+1.00,+1.00,+0.40,+0.00,  -0.60,-0.60,-0.80,-0.80,-1.00,-1.20,-1.40,-1.40,-1.40,-1.40,
/*13*//*+3*/ +1.00,+1.00,+1.00,+0.80,+0.60,+0.60,+0.40,+0.40,+0.00,-0.80,  -0.80,-0.80,-0.80,-0.80,-0.80,-0.80,-0.80,-1.00,-1.20,-1.40,
/*14*//*+4*/ +0.80,+0.80,+0.80,+0.80,+0.60,+0.60,+0.40,+0.40,+0.00,-0.60,  -0.60,-0.60,-0.60,-0.60,-0.60,-0.60,-0.80,-1.00,-1.20,-1.40,
/*15*//*+5*/ +0.60,+0.60,+0.60,+0.60,+0.40,+0.40,+0.20,+0.00,-0.40,-0.40,  -0.40,-0.40,-0.60,-0.60,-0.60,-0.60,-0.80,-1.00,-1.20,-1.40,
/*16*//*+6*/ +0.40,+0.40,+0.40,+0.40,+0.40,+0.10,+0.00,-0.40,-0.60,-0.60,  -0.60,-0.60,-0.60,-0.60,-0.60,-0.60,-0.60,-1.00,-1.20,-1.40,
/*17*//*+7*/ +0.00,+0.00,+0.00,+0.00,+0.00,-0.40,-0.40,-0.40,-0.40,-0.40,  -0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,
/*18*//*+8*/ +0.00,+0.00,+0.00,+0.00,+0.00,-0.40,-0.40,-0.40,-0.40,-0.40,  -0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,
/*19*//*+9*/ +0.00,+0.00,+0.00,+0.00,+0.00,-0.40,-0.40,-0.40,-0.40,-0.40,  -0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,
/* 超过目标速度  */
};


/******************************************************************************/
/* 内部函数                                                                    */
/******************************************************************************/

/*
* @brief 计算加速量
* @param[in] instance 成员变量
* @param[in] cur_v 当前车速(m/s)
* @param[in] cur_a 当前加速度(m/s^2)
* @param[in] tar_v 目标速度(m/s)
* @param[in] tar_a 目标加速度(m/s^2)
* @return 目标加速量(0 ~ 100%)
*
* @par Change Log:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
* </table>
*/
static Float32_t CalcAccValue(
    LonCtlDfD17B2Instance_t* const instance,
    const LonCtlDataSource_t* data_source,
    Float32_t cur_v, Float32_t cur_a, Float32_t tar_v, Float32_t tar_a) {
  // 根据当前车速，限制最大油门量
  Int32_t key_idx_max_acc = 0;
  Float32_t key_ratio_max_acc = 0.0F;
  Float32_t max_acc_value = 0.0F;
  // 根据重量修调补偿量
  // Float32_t weight = 10000.0F; // 空载(D17-B2)
  // Float32_t weight = 50000.0F; // 满载(D17-B2)
  Float32_t weight = data_source->chassis.gross_weight;
  Float32_t key_ratio_crt_by_wgt = 0.0F;
  Int32_t key_idx_crt_by_wgt = 0.0F;
  Float32_t ki_crt_by_wgt = 0.0F;
  // PID 控制器
  FuzzyPID_t* pid_controller = &(instance->acc.pid_controller);
  // 误差及其变化率的等级索引
  Int32_t err_lv_idx = 0;
  Int32_t spd_chg_lv_idx = 0;
  // 误差及其变化率
  Float32_t spd_err = cur_v - tar_v;
  Float32_t spd_chg = cur_a;
  // 补偿值
  Float32_t kp = 0.0F;
  Float32_t ki = 0.0F;
  Float32_t kd = 0.0F;
  Float32_t i_value = 0.0F;
  // 油门量
  Float32_t acc = 0;

  // 根据当前车速，限制最大油门量
  key_idx_max_acc = Phoenix_Common_LerpInOrderedTable_f(
        instance->param.key_tab_lmt_acc_by_spd,
        LON_CTL_PID_LERP_TABLE_SIZE,
        cur_v,
        &key_ratio_max_acc);
  max_acc_value = Phoenix_Common_Lerp_f(
        instance->param.value_tab_lmt_acc_by_spd[key_idx_max_acc],
        instance->param.value_tab_lmt_acc_by_spd[key_idx_max_acc+1],
        key_ratio_max_acc);

  // 根据重量修调补偿量
  key_idx_crt_by_wgt = Phoenix_Common_LerpInOrderedTable_f(
        instance->param.key_tab_crt_spd_err_feed_by_weight,
        LON_CTL_PID_LERP_TABLE_SIZE,
        weight,
        &key_ratio_crt_by_wgt);
  ki_crt_by_wgt = Phoenix_Common_Lerp_f(
        instance->param.i_tab_crt_spd_err_feed_by_weight[key_idx_crt_by_wgt],
        instance->param.i_tab_crt_spd_err_feed_by_weight[key_idx_crt_by_wgt+1],
        key_ratio_crt_by_wgt);

  // 更新当前误差等级
  Phoenix_Com_FuzzyPID_UpdateErrLevel_f(pid_controller, spd_err, spd_chg);
  err_lv_idx = pid_controller->err_level_idx_;
  spd_chg_lv_idx = pid_controller->err_spd_level_idx_;

  // 计算当前PID增益
  ki = Phoenix_Com_FuzzyPID_GetGain_f(pid_controller, 1);

  // 计算油门量
  if ((phoenix_com_abs_f(spd_err) < 1.0F/3.6F) &&
      (phoenix_com_abs_f(spd_chg) < 0.5F/3.6F)) {
    acc = instance->acc.value_i;
  } else {
    instance->acc.errs[1] = instance->acc.errs[0];
    instance->acc.errs[0] = spd_err;

    i_value = 0;
    if (spd_err < 0.0F) {
      // 未达到目标速度
      instance->acc.value_p = -kp * spd_err;
      i_value = -ki * ki_crt_by_wgt * spd_err;
    } else {
      // 超过了目标速度
      instance->acc.value_p = kp * spd_err;
      i_value = ki * ki_crt_by_wgt * spd_err;
    }
    if (i_value > 2.0F) {
      i_value = 2.0F;
    }

    instance->acc.value_i += i_value;
    instance->acc.value_d = kd * (instance->acc.errs[0] - instance->acc.errs[1]);

    if (instance->acc.value_i > max_acc_value) {
      instance->acc.value_i = max_acc_value;
    }
    if (instance->acc.value_i > 90.0f) {
      instance->acc.value_i = 90.0f;
    } else if (instance->acc.value_i < 0.0f) {
      instance->acc.value_i = 0.0f;
    }

    acc = instance->acc.value_p + instance->acc.value_i + instance->acc.value_d;
  }

#if 1
  printf("@@@@@ Spd_err_Feed: \n"
         "      w=%0.1f, w_ratio=%0.2f"
         ", tar_v=%0.1f, cur_v=%0.1f, cur_a=%0.2f, spd_err=%0.1f"
         ", err_lv=%d, acc_lv=%d, ki=%0.4f, i=%0.4f"
         ", i_value=%0.1f, acc=%0.1f\n",
         weight, ki_crt_by_wgt,
         tar_v*3.6F, cur_v*3.6F, cur_a, spd_err*3.6F,
         err_lv_idx, spd_chg_lv_idx, ki, i_value,
         instance->acc.value_i, acc);
#endif

  if (acc > 100.0f) {
    acc = 100.0f;
  } else if (acc < 0.0f) {
    acc = 0.0f;
  }

  return (acc);
}

/*
* @brief 计算制动量
* @param[in] instance 成员变量
* @param[in] cur_v 当前车速(m/s)
* @param[in] cur_a 当前加速度(m/s^2)
* @param[in] tar_v 目标速度(m/s)
* @param[in] tar_a 目标加速度(m/s^2)
* @return 目标制动量(0 ~ 100%)
*
* @par Change Log:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
* </table>
*/
static Float32_t CalcBrakeValue(
    LonCtlDfD17B2Instance_t* const instance,
    const LonCtlDataSource_t* data_source,
    Float32_t cur_v, Float32_t cur_a, Float32_t tar_v, Float32_t tar_a) {
  Float32_t max_brake = 10.0F;
  Float32_t brake = 0.0F;

  if (tar_a > -0.2F) {
    Float32_t e = cur_v - tar_v;

    if (phoenix_com_abs_f(e) < (2.0F/3.6F)) {
      brake = instance->brake.value_i;
    } else {
      instance->brake.errs[1] = instance->brake.errs[0];
      instance->brake.errs[0] = e;

      instance->brake.value_p = 2.0F * instance->brake.errs[0];
      instance->brake.value_i += 0.1F * instance->brake.errs[0];
      instance->brake.value_d =
          0.1F * (instance->acc.errs[0] - instance->acc.errs[1]);

      if (instance->brake.value_i > 20.0F) {
        instance->brake.value_i = 20.0F;
      } else if (instance->brake.value_i < 0.0F) {
        instance->brake.value_i = 0.0F;
      }

      brake =
          instance->brake.value_p +
          instance->brake.value_i +
          instance->brake.value_d;
    }

    if (brake > 20.0F) {
      brake = 20.0F;
    } else if (brake < 0.0F) {
      brake = 0.0F;
    }
  } else {
    instance->brake.errs[0] = 0.0F;
    instance->brake.errs[1] = 0.0F;
    instance->brake.value_i = 0.0F;

    if (tar_a < -8.0F) {
      tar_a = -8.0F;
    }

    brake = -(tar_a / max_brake)*100.0F;
  }

  if ((tar_v < 0.5F/3.6F) &&
      (cur_v < 1.0F/3.6F)) {
    brake = brake > 10.0F ? brake : 10.0F;
  }

  if (brake > 100.0F) {
    brake = 100.0F;
  } else if (brake < 0.0F) {
    brake = 0.0F;
  }

  return (brake);
}


/******************************************************************************/
/* 外部函数                                                                    */
/******************************************************************************/

/*
 * @brief 初始化纵向控制器
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_LonCtl_DfD17B2_Initialize(
    LonCtlDfD17B2Instance_t* const instance) {
  Int32_t ret = 0;

  phoenix_com_memset(&instance->acc, 0, sizeof(instance->acc));
  phoenix_com_memset(&instance->brake, 0, sizeof(instance->brake));

  // 根据当前车速，限制最大油门量(Key table & value table)
  phoenix_com_memcpy(
        instance->param.key_tab_lmt_acc_by_spd,
        s_key_tab_lmt_acc_by_spd,
        sizeof(instance->param.key_tab_lmt_acc_by_spd));
  // 最大油门量
  phoenix_com_memcpy(
        instance->param.value_tab_lmt_acc_by_spd,
        s_value_tab_lmt_acc_by_spd,
        sizeof(instance->param.value_tab_lmt_acc_by_spd));

  // 通过重量修正速度误差补偿值(Key table & i-gain table)
  phoenix_com_memcpy(
        instance->param.key_tab_crt_spd_err_feed_by_weight,
        s_key_tab_crt_spd_err_feed_by_weight,
        sizeof(instance->param.key_tab_crt_spd_err_feed_by_weight));
  // I
  phoenix_com_memcpy(
        instance->param.i_tab_crt_spd_err_feed_by_weight,
        s_i_tab_crt_spd_err_feed_by_weight,
        sizeof(instance->param.i_tab_crt_spd_err_feed_by_weight));

  // 速度误差补偿Fuzzy Table
  // 速度误差等级表
  Phoenix_Com_Matrix_Init(
        &(instance->param.tab_spd_err_level),
        LON_CTL_PID_SPD_ERR_FEED_TABLE_SIZE-1, 1,
        instance->param.tab_buf_spd_err_level);
  Phoenix_Com_Matrix_InputFromArray_f(
        &(instance->param.tab_spd_err_level),
        LON_CTL_PID_SPD_ERR_FEED_TABLE_SIZE-1,
        s_tab_spd_err_level);
  // 速度变化率等级表
  Phoenix_Com_Matrix_Init(
        &(instance->param.tab_spd_chg_level),
        LON_CTL_PID_SPD_ERR_FEED_TABLE_SIZE-1, 1,
        instance->param.tab_buf_spd_chg_level);
  Phoenix_Com_Matrix_InputFromArray_f(
        &(instance->param.tab_spd_chg_level),
        LON_CTL_PID_SPD_ERR_FEED_TABLE_SIZE-1,
        s_tab_spd_chg_level);
  // 速度误差补偿系数表
  // I
  Phoenix_Com_Matrix_Init(
        &(instance->param.i_gain_tab_spd_err_feed),
        LON_CTL_PID_SPD_ERR_FEED_TABLE_SIZE, LON_CTL_PID_SPD_ERR_FEED_TABLE_SIZE,
        instance->param.i_gain_tab_buf_spd_err_feed);
  Phoenix_Com_Matrix_InputFromArray_f(
        &(instance->param.i_gain_tab_spd_err_feed),
        LON_CTL_PID_SPD_ERR_FEED_TABLE_SIZE*LON_CTL_PID_SPD_ERR_FEED_TABLE_SIZE,
        s_i_gain_tab_spd_err_feed);

  // 初始化PID控制器
  ret = Phoenix_Com_FuzzyPID_Init(
        &(instance->acc.pid_controller),
        &(instance->param.tab_spd_err_level),
        &(instance->param.tab_spd_chg_level));
  if (ret < 0) {
    LOG_ERR_C("Failed to initialize longitudinal error pid controller.");
    COM_CHECK_C(0 == ret);
  }
  ret = Phoenix_Com_FuzzyPID_SetGainTable(
        &(instance->acc.pid_controller), 1,
        &(instance->param.i_gain_tab_spd_err_feed));
  if (ret < 0) {
    LOG_ERR_C("Failed to set I gain table of longitudinal error pid controller.");
    COM_CHECK_C(0 == ret);
  }
}

/*
 * @brief 重置纵向控制器内部状态(加速模块)
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_LonCtl_DfD17B2_ResetAccValue(
    LonCtlDfD17B2Instance_t* const instance, Float32_t value) {
  instance->acc.value_i = value;
}

/*
 * @brief 重置纵向控制器内部状态(制动模块)
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_LonCtl_DfD17B2_ResetBrakeValue(
    LonCtlDfD17B2Instance_t* const instance, Float32_t value) {
  instance->brake.value_i = value;
}

/*
 * @brief 计算纵向控制量
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
Int32_t Phoenix_LonCtl_DfD17B2_CalcLonCtlValue(
    LonCtlDfD17B2Instance_t* const instance,
    const LonCtlDataSource_t* data_source,
    Float32_t* acc_value,
    Float32_t* brake_value) {
  Float32_t cur_v = data_source->cur_v;
  Float32_t cur_a = data_source->cur_a;
  Float32_t tar_v = data_source->tar_v;
  Float32_t tar_a = data_source->tar_a;
  Float32_t diff_v = 0.0F;

  *acc_value = 0;
  *brake_value = 0;

  if (tar_v > 100.0F/3.6F) {
    tar_v = 100.0F/3.6F;
  } else if (tar_v < 0.0F) {
    tar_v = 0.0F;
  }

  if (cur_v > 200.0F/3.6F) {
    cur_v = 200.0F/3.6F;
  } else if (cur_v < 0.0F) {
    cur_v = 0.0F;
  }

  if (tar_a > 5.0F) {
    tar_a = 5.0F;
  } else if (tar_a < -5.0F) {
    tar_a = -5.0F;
  }

  if (cur_a > 20.0F) {
    cur_a = 20.0F;
  } else if (cur_a < -20.0F) {
    cur_a = -20.0F;
  }

  diff_v = cur_v - tar_v;

  // 计算刹车量
  if (/* 当前车速大于目标车速，上层模块要求减速 */
      ((diff_v > 2.0F/3.6F) && (tar_a < -0.5F)) ||
      /* 当前车速大于目标车速，且车速仍在增加 */
      ((diff_v > 3.0F/3.6F) && (cur_a > 0.6F)) ||
      /* 当前车速大于目标车速，且车速仍在增加*/
      ((diff_v > 5.0F/3.6F) && (cur_a > -0.2F)) ||
      /* 目标车速为0，当前车速大于3km/h */
      ((tar_v < 0.5F/3.6F) && (cur_v < 3.0F/3.6F)) ||
      /* 目标车速为0, 上层模块要求制动 */
      ((tar_v < 0.5F/3.6F) && (tar_a < -0.5F))) {
    *brake_value = CalcBrakeValue(instance, data_source,
                                  cur_v, cur_a, tar_v, tar_a);
    *acc_value = 0.0F;
    Phoenix_LonCtl_DfD17B2_ResetAccValue(instance, 0.0F);
  } else {
    // 计算油门量
    *acc_value = CalcAccValue(instance, data_source,
                              cur_v, cur_a, tar_v, tar_a);
    *brake_value = 0;
    Phoenix_LonCtl_DfD17B2_ResetBrakeValue(instance, 0.0F);
  }

  return (0);
}

