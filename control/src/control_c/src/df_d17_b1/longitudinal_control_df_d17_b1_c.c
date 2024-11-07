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
#include "df_d17_b1/longitudinal_control_df_d17_b1_c.h"

#include "utils/com_utils_c.h"
#include "utils/com_clock_c.h"
#include "math/math_utils_c.h"


/******************************************************************************/
/* 全局及静态变量                                                               */
/******************************************************************************/

// 加速量积分补偿系数表
static Float32_t s_fuzzy_table_acc_ki[16][20] = {
           /* 0    1    2    3    4    5    6    7    8    9   10   11   12   13   14   15   16   17   18   19*/
           /*-9   -8   -7   -6   -5   -4   -3   -2   -1   -0   +0   +1   +2   +3   +4   +5   +6   +7   +8   +9*/
/* 0*//*-9*/{+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.5f,+0.3f,+0.2f,+0.1f,+0.0f},
/* 1*//*-8*/{+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.5f,+0.3f,+0.1f,+0.0f,-0.3f},
/* 2*//*-7*/{+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.5f,+0.5f,+0.2f,+0.0f,-0.3f,-1.0f},
/* 3*//*-6*/{+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.5f,+0.3f,+0.0f,-0.2f,-0.5f,-1.0f},
/* 4*//*-5*/{+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.8f,+0.5f,+0.5f,+0.5f,+0.0f,-0.1f,-0.3f,-0.8f,-1.0f,-1.0f},
/* 5*//*-4*/{+2.0f,+2.0f,+2.0f,+2.0f,+2.0f,+2.0f,+2.0f,+2.0f,+2.0f,+2.0f,+1.0f,+0.8f,+0.5f,+0.0f,-0.3f,-0.8f,-1.0f,-1.5f,-2.0f,-2.0f},
/* 6*//*-3*/{+3.0f,+3.0f,+3.0f,+3.0f,+3.0f,+3.0f,+3.0f,+3.0f,+3.0f,+2.0f,+2.0f,+1.0f,+0.0f,-1.0f,-2.0f,-2.0f,-2.0f,-3.0f,-3.0f,-4.0f},
/* 7*//*-2*/{+4.0f,+4.0f,+4.0f,+4.0f,+4.0f,+4.0f,+4.0f,+4.0f,+4.0f,+3.0f,+1.0f,+0.0f,-0.8f,-1.0f,-2.0f,-3.0f,-4.0f,-5.0f,-6.0f,-7.0f},
/* 8*//*-1*/{+5.0f,+5.0f,+5.0f,+5.0f,+5.0f,+5.0f,+5.0f,+5.0f,+5.0f,+3.0f,+1.5f,+0.0f,-1.5f,-2.0f,-2.0f,-3.0f,-4.0f,-5.0f,-6.0f,-7.0f},
/* 9*//*-0*/{+5.0f,+5.0f,+5.0f,+5.0f,+5.0f,+5.0f,+5.0f,+5.0f,+5.0f,+3.0f,+0.0f,-4.0f,-4.0f,-4.0f,-5.0f,-6.0f,-7.0f,-7.0f,-7.0f,-7.0f},
/*10*//*+0*/{+5.0f,+5.0f,+5.0f,+5.0f,+5.0f,+5.0f,+5.0f,+5.0f,+2.0f,+0.0f,-3.0f,-3.0f,-4.0f,-4.0f,-5.0f,-6.0f,-7.0f,-7.0f,-7.0f,-7.0f},
/*11*//*+1*/{+5.0f,+5.0f,+5.0f,+4.0f,+3.0f,+3.0f,+2.0f,+2.0f,+0.0f,-4.0f,-4.0f,-3.0f,-3.0f,-3.0f,-3.0f,-3.0f,-4.0f,-5.0f,-6.0f,-7.0f},
/*12*//*+2*/{+4.0f,+4.0f,+4.0f,+4.0f,+3.0f,+3.0f,+2.0f,+2.0f,+0.0f,-3.0f,-3.0f,-2.0f,-3.0f,-3.0f,-3.0f,-3.0f,-4.0f,-5.0f,-6.0f,-7.0f},
/*13*//*+3*/{+3.0f,+3.0f,+3.0f,+3.0f,+2.0f,+2.0f,+1.0f,+0.0f,-2.0f,-2.0f,-2.0f,-2.0f,-3.0f,-3.0f,-3.0f,-3.0f,-4.0f,-5.0f,-6.0f,-7.0f},
/*14*//*+4*/{+2.0f,+2.0f,+2.0f,+2.0f,+2.0f,+0.5f,+0.0f,-2.0f,-3.0f,-3.0f,-3.0f,-3.0f,-3.0f,-3.0f,-3.0f,-3.0f,-4.0f,-5.0f,-6.0f,-7.0f},
/*15*//*+5*/{+0.0f,+0.0f,+0.0f,+0.0f,+0.0f,-2.0f,-2.0f,-2.0f,-2.0f,-2.0f,-2.0f,-2.0f,-2.0f,-2.0f,-2.0f,-2.0f,-2.0f,-2.0f,-2.0f,-2.0f},
};


/******************************************************************************/
/* 内部函数                                                                    */
/******************************************************************************/

/*
 * @brief 计算速度误差模糊等级
 * @param[in] e 速度误差
 * @param[out] level 模糊等级
 * @param[out] index 模糊等级的索引
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
static void CalcFuzzyLevelAccE(Float32_t e, Float32_t* level, Int32_t* index) {
  *level = 0;
  *index = 0;

  if (e < -60.0F/3.6F) {
    *level = -9;
    *index = 0;
  } else if ((-60.0F/3.6F <= e) && (e < -50.0F/3.6F)) {
    *level = -8;
    *index = 1;
  } else if ((-50.0F/3.6F <= e) && (e < -40.0F/3.6F)) {
    *level = -7;
    *index = 2;
  } else if ((-40.0F/3.6F <= e) && (e < -30.0F/3.6F)) {
    *level = -6;
    *index = 3;
  } else if ((-30.0F/3.6F <= e) && (e < -20.0F/3.6F)) {
    *level = -5;
    *index = 4;
  } else if ((-20.0F/3.6F <= e) && (e < -15.0F/3.6F)) {
    *level = -4;
    *index = 5;
  } else if ((-15.0F/3.6F <= e) && (e < -10.0F/3.6F)) {
    *level = -3;
    *index = 6;
  } else if ((-10.0F/3.6F <= e) && (e < -5.0F/3.6F)) {
    *level = -2;
    *index = 7;
  } else if ((-5.0F/3.6F <= e) && (e < -2.0F/3.6F)) {
    *level = -1;
    *index = 8;
  } else if ((-2.0F/3.6F <= e) && (e <= 0.0F)) {
    *level = -0.5;   // -0
    *index = 9;
  } else if ((0.0F < e) && (e <= 2.0F/3.6F)) {
    *level = 0.5;    // +0
    *index = 10;
  } else if ((2.0F/3.6F < e) && (e <= 5.0F/3.6F)) {
    *level = 1;
    *index = 11;
  } else if ((5.0F/3.6F < e) && (e <= 10.0F/3.6F)) {
    *level = 2;
    *index = 12;
  } else if ((10.0F/3.6F < e) && (e <= 15.0F/3.6F)) {
    *level = 3;
    *index = 13;
  } else if ((15.0F/3.6F < e) && (e <= 20.0F/3.6F)) {
    *level = 4;
    *index = 14;
  } else if (20.0F/3.6F < e) {
    *level = 5;
    *index = 15;
  } else {
    *level = 0;
    *index = 0;
  }
}

/*
 * @brief 计算加速度模糊等级
 * @param[in] ec 加速度
 * @param[out] level 模糊等级
 * @param[out] index 模糊等级的索引
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/07/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
static void CalcFuzzyLevelAccEC(
    Float32_t ec, Float32_t* level, Int32_t* index) {
  *level = 0;
  *index = 0;

  if (ec < -8.0F/3.6F) {
    *level = -9;
    *index = 0;
  } else if ((-8.0F/3.6F <= ec) && (ec < -7.0F/3.6F)) {
    *level = -8;
    *index = 1;
  } else if ((-7.0F/3.6F <= ec) && (ec < -6.0F/3.6F)) {
    *level = -7;
    *index = 2;
  } else if ((-6.0F/3.6F <= ec) && (ec < -5.0F/3.6F)) {
    *level = -6;
    *index = 3;
  } else if ((-5.0F/3.6F <= ec) && (ec < -4.0F/3.6F)) {
    *level = -5;
    *index = 4;
  } else if ((-4.0F/3.6F <= ec) && (ec < -3.0F/3.6F)) {
    *level = -4;
    *index = 5;
  } else if ((-3.0F/3.6F <= ec) && (ec < -2.0F/3.6F)) {
    *level = -3;
    *index = 6;
  } else if ((-2.0F/3.6F <= ec) && (ec < -1.0F/3.6F)) {
    *level = -2;
    *index = 7;
  } else if ((-1.0F/3.6F <= ec) && (ec < -0.5F/3.6F)) {
    *level = -1;
    *index = 8;
  } else if ((-0.5F/3.6F <= ec) && (ec <= 0.0F)) {
    *level = -0.5;   // -0
    *index = 9;
  } else if ((0.0F < ec) && (ec <= 0.5F/3.6F)) {
    *level = 0.5;    // +0
    *index = 10;
  } else if ((0.5F/3.6F < ec) && (ec <= 1.0F/3.6F)) {
    *level = 1;
    *index = 11;
  } else if ((1.0F/3.6F < ec) && (ec <= 2.0F/3.6F)) {
    *level = 2;
    *index = 12;
  } else if ((2.0F/3.6F < ec) && (ec <= 3.0F/3.6F)) {
    *level = 3;
    *index = 13;
  } else if ((3.0F/3.6F < ec) && (ec <= 4.0F/3.6F)) {
    *level = 4;
    *index = 14;
  } else if ((4.0F/3.6F < ec) && (ec <= 5.0F/3.6F)) {
    *level = 5;
    *index = 15;
  } else if ((5.0F/3.6F < ec) && (ec <= 6.0F/3.6F)) {
    *level = 6;
    *index = 16;
  } else if ((6.0F/3.6F < ec) && (ec <= 7.0F/3.6F)) {
    *level = 7;
    *index = 17;
  } else if ((7.0F/3.6F < ec) && (ec <= 8.0F/3.6F)) {
    *level = 8;
    *index = 18;
  } else if (8.0F/3.6F < ec) {
    *level = 9;
    *index = 19;
  } else {
    *level = 0;
    *index = 0;
  }
}

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
    LonCtlDfD17B1Instance_t* const instance,
    Float32_t cur_v, Float32_t cur_a, Float32_t tar_v, Float32_t tar_a) {
  Float32_t acc = 0;
  Float32_t e = cur_v - tar_v;
  Float32_t ec = cur_a;
  Float32_t level_e = 0;
  Int32_t index_e = 0;
  Float32_t level_ec = 0;
  Int32_t index_ec = 0;
  Float32_t level_ki = 0.0F;
  Float32_t Kp = 0.0F;
  Float32_t Ki = 0.0F;
  Float32_t Kd = 0.0F;
  Float32_t value_i = 0.0F;
  Float32_t param_max_acc_value = 80;

  // 计算模糊等级
  CalcFuzzyLevelAccE(e, &level_e, &index_e);
  CalcFuzzyLevelAccEC(ec, &level_ec, &index_ec);

  // 计算模糊输出
  level_ki = s_fuzzy_table_acc_ki[index_e][index_ec];

  // 获取PID系数
#if 0
  Kp = 0;
  Ki = 0.08F * level_ki;
  Kd = 0.2F;
  if (level_ki < 0) {
    Ki = 0.1F * level_ki;
  }
#else
  Kp = 0;
  Ki = 0.06F * level_ki;
  Kd = 0.2F;
  if (level_ki < 0) {
    Ki = 0.1F * level_ki;
  }
#endif

#if 0
  printf("cur_v = %f, tar_v = %f, e = %f, ec = %f\n",cur_v, tar_v, e, ec);
  printf("e = %f, level_e = %f, index_e= %f\n",e, level_e, index_e);
  printf("ec = %f, level_ec = %f, index_ec= %f\n",ec, level_ec, index_ec);
  printf("index_e = %f, index_ec = %f, level_ki= %f\n",index_e, index_ec, level_ki);
#endif

  // 计算油门量
  if ((phoenix_com_abs_f(e) < 1.0F/3.6F) &&
      (phoenix_com_abs_f(ec) < 0.5F/3.6F)) {
    acc = instance->acc.value_i;
  } else {
    instance->acc.errs[1] = instance->acc.errs[0];
    instance->acc.errs[0] = e;
    value_i = 0;
    if (instance->acc.errs[0] < 0.0f) {
      instance->acc.value_p = -Kp * instance->acc.errs[0];
      value_i = -Ki * instance->acc.errs[0];
    } else {
      instance->acc.value_p = Kp * instance->acc.errs[0];
      value_i = Ki * instance->acc.errs[0];
    }
    if (value_i > 2.0f) {
      value_i = 2.0f;
    }

    instance->acc.value_i += value_i;
    instance->acc.value_d = Kd * (instance->acc.errs[0] - instance->acc.errs[1]);

    if (cur_v < 10.0f/3.6f) {
      param_max_acc_value = 20;
    } else if (cur_v < 20.0f/3.6f) {
      param_max_acc_value = 30;
    } else if (cur_v < 30.0f/3.6f) {
      param_max_acc_value = 40;
    } else if (cur_v < 40.0f/3.6f) {
      param_max_acc_value = 50;
    } else if (cur_v < 50.0f/3.6f) {
      param_max_acc_value = 60;
    } else if (cur_v < 60.0f/3.6f) {
      param_max_acc_value = 70;
    } else if (cur_v < 70.0f/3.6f) {
      param_max_acc_value = 80;
    } else if (cur_v < 80.0f/3.6f) {
      param_max_acc_value = 80;
    } else {
      param_max_acc_value = 80;
    }

    if (instance->acc.value_i > param_max_acc_value) {
      instance->acc.value_i = param_max_acc_value;
    }
    if (instance->acc.value_i > 90.0f) {
      instance->acc.value_i = 90.0f;
    } else if (instance->acc.value_i < 0.0f) {
      instance->acc.value_i = 0.0f;
    }

    acc = instance->acc.value_p + instance->acc.value_i + instance->acc.value_d;
  }

  if (acc > 100.0f) {
    acc = 100.0f;
  } else if (acc < 0.0f) {
    acc = 0.0f;
  }
  // printf("instance->acc.value_p = %f \n",instance->acc.value_p);
  // printf("instance->acc.value_i = %f \n",instance->acc.value_i);
  // printf("instance->acc.value_d = %f \n",instance->acc.value_d);
  // printf("acc = %f \n",acc);
  // printf("=================================f \n");
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
    LonCtlDfD17B1Instance_t* const instance,
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
void Phoenix_LonCtl_DfD17B1_Initialize(
    LonCtlDfD17B1Instance_t* const instance) {
  phoenix_com_memset(&instance->acc, 0, sizeof(instance->acc));
  phoenix_com_memset(&instance->brake, 0, sizeof(instance->brake));
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
void Phoenix_LonCtl_DfD17B1_ResetAccValue(
    LonCtlDfD17B1Instance_t* const instance, Float32_t value) {
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
void Phoenix_LonCtl_DfD17B1_ResetBrakeValue(
    LonCtlDfD17B1Instance_t* const instance, Float32_t value) {
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
Int32_t Phoenix_LonCtl_DfD17B1_CalcLonCtlValue(
    LonCtlDfD17B1Instance_t* const instance,
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
    *brake_value = CalcBrakeValue(instance, cur_v, cur_a, tar_v, tar_a);
    *acc_value = 0.0F;
    Phoenix_LonCtl_DfD17B1_ResetAccValue(instance, 0.0F);
  } else {
    // 计算油门量
    *acc_value = CalcAccValue(instance, cur_v, cur_a, tar_v, tar_a);
    *brake_value = 0;
    Phoenix_LonCtl_DfD17B1_ResetBrakeValue(instance, 0.0F);
  }

  return (0);
}

