/** @copyright Copyright (c) 2018-2024 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       lateral_control_analyse.c
 * @brief      横向控制分析
 * @details    实现横向控制分析
 *
 * @author     pengc
 * @date       2024.02.28
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2024/02/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

/******************************************************************************/
/* 头文件                                                                      */
/******************************************************************************/
#include "lateral_control_analyse_c.h"

#include "utils/log_c.h"
#include "utils/com_utils_c.h"
#include "utils/com_clock_c.h"
#include "utils/linear_interpolation_c.h"
#include "utils/com_filter_c.h"
#include "math/math_utils_c.h"
#include "math/fft_c.h"
#include "vehicle_model_c.h"


/******************************************************************************/
/* 宏定义                                                                      */
/******************************************************************************/


/******************************************************************************/
/* 类型定义                                                                    */
/******************************************************************************/


/******************************************************************************/
/* 全局及静态变量                                                               */
/******************************************************************************/
// 输入输出的响应关系
static const Float32_t s_key_tab_trans_rel[LAT_CTL_ANA_TRANS_REL_ITEM_NUM] = {
  0.00F*0.017453293F,  0.50F*0.017453293F,  1.00F*0.017453293F,  1.50F*0.017453293F,  2.00F*0.017453293F,
  2.50F*0.017453293F,  3.00F*0.017453293F,  3.50F*0.017453293F,  4.00F*0.017453293F,  4.50F*0.017453293F,
  5.00F*0.017453293F,  5.50F*0.017453293F,  6.00F*0.017453293F,  6.50F*0.017453293F,  7.00F*0.017453293F,
  7.50F*0.017453293F,  8.00F*0.017453293F,  8.50F*0.017453293F,  9.00F*0.017453293F,  9.50F*0.017453293F,
  10.0F*0.017453293F
};

// 高频蛇形抑制
static const Float32_t s_key_tab_hunting_suppress_high_fre[LAT_CTL_PID_LERP_TABLE_SIZE] = {
  0.00F*0.017453293F,  0.50F*0.017453293F,  1.00F*0.017453293F,  1.50F*0.017453293F,  2.00F*0.017453293F,
  2.50F*0.017453293F,  3.00F*0.017453293F,  3.50F*0.017453293F,  4.00F*0.017453293F,  4.50F*0.017453293F,
  5.00F*0.017453293F,  5.50F*0.017453293F,  6.00F*0.017453293F,  6.50F*0.017453293F,  7.00F*0.017453293F,
  7.50F*0.017453293F,  8.00F*0.017453293F,  8.50F*0.017453293F,  9.00F*0.017453293F,  9.50F*0.017453293F,
  10.0F*0.017453293F,  11.0F*0.017453293F,  12.0F*0.017453293F,  13.0F*0.017453293F,  14.0F*0.017453293F,
};

static const Float32_t s_val_tab_hunting_suppress_high_fre[LAT_CTL_PID_LERP_TABLE_SIZE] = {
  1.00F,  0.98F,  0.95F,  0.88F,  0.85F,
  0.80F,  0.75F,  0.70F,  0.65F,  0.60F,
  0.55F,  0.50F,  0.45F,  0.40F,  0.35F,
  0.30F,  0.30F,  0.30F,  0.30F,  0.30F,
  0.30F,  0.30F,  0.30F,  0.30F,  0.30F,
};

static const Float32_t s_key_tab_hunting_suppress_d1_yaw_rate[LAT_CTL_PID_LERP_TABLE_SIZE] = {
  0.00F*0.017453293F,  0.10F*0.017453293F,  0.20F*0.017453293F,  0.30F*0.017453293F,  0.40F*0.017453293F,
  0.50F*0.017453293F,  0.60F*0.017453293F,  0.70F*0.017453293F,  0.80F*0.017453293F,  0.90F*0.017453293F,
  1.00F*0.017453293F,  1.10F*0.017453293F,  1.20F*0.017453293F,  1.30F*0.017453293F,  1.40F*0.017453293F,
  1.50F*0.017453293F,  1.60F*0.017453293F,  1.70F*0.017453293F,  1.80F*0.017453293F,  1.90F*0.017453293F,
  2.00F*0.017453293F,  2.10F*0.017453293F,  2.20F*0.017453293F,  2.30F*0.017453293F,  2.40F*0.017453293F,
};

#if 1
static const Float32_t s_val_tab_hunting_suppress_d1_yaw_rate[LAT_CTL_PID_LERP_TABLE_SIZE] = {
  1.00F,  0.90F,  0.80F,  0.70F,  0.60F,
  0.60F,  0.60F,  0.60F,  0.60F,  0.60F,
  0.60F,  0.60F,  0.60F,  0.50F,  0.50F,
  0.50F,  0.50F,  0.50F,  0.50F,  0.50F,
  0.50F,  0.50F,  0.50F,  0.50F,  0.50F,
};
#endif
#if 0
static const Float32_t s_val_tab_hunting_suppress_d1_yaw_rate[LAT_CTL_PID_LERP_TABLE_SIZE] = {
  1.00F,  0.95F,  0.90F,  0.85F,  0.80F,
  0.75F,  0.70F,  0.65F,  0.60F,  0.55F,
  0.50F,  0.50F,  0.50F,  0.50F,  0.50F,
  0.50F,  0.50F,  0.50F,  0.50F,  0.50F,
  0.50F,  0.50F,  0.50F,  0.50F,  0.50F,
};
#endif

static const Float32_t s_key_tab_hunting_suppress_curvature[LAT_CTL_PID_LERP_TABLE_SIZE] = {
  0.00F*0.017453293F,  0.10F*0.017453293F,  0.20F*0.017453293F,  0.30F*0.017453293F,  0.40F*0.017453293F,
  0.50F*0.017453293F,  0.60F*0.017453293F,  0.70F*0.017453293F,  0.80F*0.017453293F,  0.90F*0.017453293F,
  1.00F*0.017453293F,  1.10F*0.017453293F,  1.20F*0.017453293F,  1.30F*0.017453293F,  1.40F*0.017453293F,
  1.50F*0.017453293F,  1.60F*0.017453293F,  1.70F*0.017453293F,  1.80F*0.017453293F,  1.90F*0.017453293F,
  2.00F*0.017453293F,  2.10F*0.017453293F,  2.20F*0.017453293F,  2.30F*0.017453293F,  2.40F*0.017453293F,
};

static const Float32_t s_val_tab_hunting_suppress_curvature[LAT_CTL_PID_LERP_TABLE_SIZE] = {
  0.00F*0.017453293F,  0.10F*0.017453293F,  0.20F*0.017453293F,  0.30F*0.017453293F,  0.40F*0.017453293F,
  0.50F*0.017453293F,  0.60F*0.017453293F,  0.70F*0.017453293F,  0.80F*0.017453293F,  0.90F*0.017453293F,
  1.00F*0.017453293F,  1.10F*0.017453293F,  1.20F*0.017453293F,  1.30F*0.017453293F,  1.40F*0.017453293F,
  1.50F*0.017453293F,  1.60F*0.017453293F,  1.70F*0.017453293F,  1.80F*0.017453293F,  1.90F*0.017453293F,
  2.00F*0.017453293F,  2.10F*0.017453293F,  2.20F*0.017453293F,  2.30F*0.017453293F,  2.40F*0.017453293F,
};

/******************************************************************************/
/* 内部函数                                                                    */
/******************************************************************************/


/*
 * @brief 初始化定时器
 * @param[in] ins 成员变量
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2023/12/01  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
static void InitializeTimers(LatCtlAnalyseInstance_t* const ins) {
  Phoenix_Com_Timer_Init(&(ins->timers.eps_auto), 0);
}

/*
 * @brief 更新定时器
 * @param[in] ins 成员变量
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2023/12/01  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
static void UpdateTimers(LatCtlAnalyseInstance_t* const ins) {
  Phoenix_Com_Timer_UpdateWithStamp(&(ins->timers.eps_auto), ins->timestamp);
}


/*
 * @brief 计算方向盘转向不足补偿值
 * @param[in] ins 类实例
 * @param[in] yaw_rate 角速度
 * @param[in] veh_spd 车速
 * @return 方向盘转向不足补偿值 (rad)
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2024/02/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
static Float32_t CalcUndersteerFeed(
    LatCtlAnalyseInstance_t* const ins,
    Float32_t yaw_rate, Float32_t veh_spd) {
  Float32_t understeer_feed = 0.007F * 16.0F * yaw_rate * veh_spd;

  return (understeer_feed);
}


/*
 * @brief 估计转向零位
 * @param[in] ins 类实例
 * @param[in] pre_item 之前的数据
 * @param[in&out] cur_item 当前的数据
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2024/02/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
static void EstimateSteeringWheelAngleOffset(
    LatCtlAnalyseInstance_t* const ins,
    const LatCtlAnalyseStatisticDataItem_t* const pre_item,
    LatCtlAnalyseStatisticDataItem_t* cur_item) {
  Float32_t str_wel_ang_offset = cur_item->avg_exp_steady_str_wel_ang - cur_item->avg_str_wel_ang;
  if ((phoenix_com_abs_f(cur_item->yaw_rate_cha) < 1.0F*0.017453293F) && (cur_item->veh_speed > 50.0F/3.6F)) {
    static const Float32_t kStrOffsetUpdateRatio = 0.001F;
    cur_item->str_wel_ang_offset = (1.0F - kStrOffsetUpdateRatio) * pre_item->str_wel_ang_offset + 
        kStrOffsetUpdateRatio * str_wel_ang_offset;
  } else {
    cur_item->str_wel_ang_offset = pre_item->str_wel_ang_offset;
  }
  // 最大零位偏差
  if (phoenix_com_abs_f(cur_item->str_wel_ang_offset) > phoenix_com_abs_f(ins->statistic.max_str_wel_ang_offset)) {
    ins->statistic.max_str_wel_ang_offset = cur_item->str_wel_ang_offset;
  }
}

/*
 * @brief 估计转向间隙
 * @param[in] ins 类实例
 * @param[in] items_num 当前已有数据的数量，不含当前数据
 * @param[in] pre_item 之前的数据
 * @param[in&out] cur_item 当前的数据
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2024/02/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
static void EstimateSteeringWheelAngleGap(
    LatCtlAnalyseInstance_t* const ins,
    Int32_t items_num,
    const LatCtlAnalyseStatisticDataItem_t* const pre_item,
    LatCtlAnalyseStatisticDataItem_t* cur_item) {
  LatCtlAnalyseStatisticDataItem_t* item_str_for_gap = 
      Phoenix_Com_RingBuffer_GetDataByIdx(&(ins->statistic.seq_data), items_num-ins->conf.yaw_rate_res_delay_offset);
  if (Null_t == item_str_for_gap) {
    ins->est_str_gap.wnd_itg_d1_str_wel_ang = 0.0F;
    ins->est_str_gap.max_str_wel_ang_gap_in_wnd = 0.0F;
    Phoenix_Com_RingBuffer_Clear(&(ins->est_str_gap.queue.ring_buf));
    cur_item->str_wel_ang_gap = 0.0F;
  } else {
    if ((phoenix_com_abs_f(cur_item->wnd_itg_d1_yaw_rate_cha) < 0.12F*0.017453293F) && (cur_item->veh_speed > 50.0F/3.6F)) {
      Float32_t str_inc = item_str_for_gap->d1_str_wel_ang * ins->conf.sample_interval;
      Int32_t itg_num = Phoenix_Com_RingBuffer_GetSize(&(ins->est_str_gap.queue.ring_buf));
      if (itg_num < LAT_CTL_ANA_STR_GAP_WND_SIZE) {
        ins->est_str_gap.wnd_itg_d1_str_wel_ang += str_inc;
      } else {
        ins->est_str_gap.wnd_itg_d1_str_wel_ang += 
            str_inc - *((Float32_t*)Phoenix_Com_RingBuffer_GetFront(&(ins->est_str_gap.queue.ring_buf)));
      }
      Phoenix_Com_RingBuffer_PushBackOverride(&(ins->est_str_gap.queue.ring_buf), &str_inc);
      if (phoenix_com_abs_f(ins->est_str_gap.wnd_itg_d1_str_wel_ang) > ins->est_str_gap.max_str_wel_ang_gap_in_wnd) {
        ins->est_str_gap.max_str_wel_ang_gap_in_wnd = phoenix_com_abs_f(ins->est_str_gap.wnd_itg_d1_str_wel_ang);
      }
      if ((itg_num*ins->conf.sample_interval) > 2.0F) {
        cur_item->str_wel_ang_gap = 0.98F*pre_item->str_wel_ang_gap + 0.02F*ins->est_str_gap.max_str_wel_ang_gap_in_wnd;
      } else {
        cur_item->str_wel_ang_gap = pre_item->str_wel_ang_gap;
      }
    } else {
      ins->est_str_gap.wnd_itg_d1_str_wel_ang = 0.0F;
      ins->est_str_gap.max_str_wel_ang_gap_in_wnd = 0.0F;
      Phoenix_Com_RingBuffer_Clear(&(ins->est_str_gap.queue.ring_buf));
      cur_item->str_wel_ang_gap = pre_item->str_wel_ang_gap;
    }
  }
  // 最大转向间隙
  if (cur_item->str_wel_ang_gap > ins->statistic.max_str_wel_ang_gap) {
    ins->statistic.max_str_wel_ang_gap = cur_item->str_wel_ang_gap;
  }
}

/*
 * @brief FFT变换
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2023/08/31  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
static void DoFft(LatCtlAnalyseFFTInstance_t* const ins) {
  Int32_t idx = 0;

#if 0
  printf("\n**************** UpdateLatHuntingDetector (begin) *************\n");
  // Check signals
  printf("The signals are:\n");
  for (Int32_t i = 0; i < LAT_CTL_ANA_FFT_SIZE; ++i) {
    printf("%0.4f ", phoenix_com_rad2deg_f(ins->rex[i]));
  }
  printf("\n");
#endif

  // Forward FFT
  Phoenix_Common_ForwardFFT_f(LAT_CTL_ANA_FFT_SIZE, ins->rex, ins->imx);

  // Calculate polar form
  for (idx = 0; idx < LAT_CTL_ANA_FFT_SIZE/2; ++idx) {
    // 计算幅值
    ins->mag[idx] =
        phoenix_com_sqrt_f(
          ins->rex[idx]*ins->rex[idx] +
          ins->imx[idx]*ins->imx[idx]);
    if (0 == idx) {
      ins->mag[idx] /= LAT_CTL_ANA_FFT_SIZE;
    } else {
      ins->mag[idx] /= LAT_CTL_ANA_FFT_SIZE/2;
    }

#if 1
    // 计算相位
    ins->pha[idx] = phoenix_com_atan2_f(
        ins->imx[idx]*ins->imx[idx], 
        ins->rex[idx]*ins->rex[idx]);
#endif
  }

#if 0
  // Check result of Forward FFT
  printf("\nAfter Forward FFT.\n");
  printf("The rex are:\n");
  for (Int32_t i = 0; i < LAT_CTL_ANA_FFT_SIZE; ++i) {
    printf("%0.4f ", phoenix_com_rad2deg_f(ins->rex[i]));
  }
  printf("\n");
  printf("The imx are:\n");
  for (Int32_t i = 0; i < LAT_CTL_ANA_FFT_SIZE; ++i) {
    printf("%0.4f ", phoenix_com_rad2deg_f(ins->imx[i]));
  }
  printf("\n");
#endif
#if 0
  printf("The magnitude are:\n");
  for (Int32_t i = 0; i < LAT_CTL_ANA_FFT_SIZE/2; ++i) {
    printf("%0.4f ", phoenix_com_rad2deg_f(ins->mag[i]));
  }
  printf("\n");
  printf("The phase are:\n");
  for (Int32_t i = 0; i < LAT_CTL_ANA_FFT_SIZE/2; ++i) {
    printf("%0.4f ", phoenix_com_rad2deg_f(ins->pha[i]));
  }
  printf("\n");
  printf("\n**************** UpdateLatHuntingDetector (end) ***************\n");
#endif
}

/*
 * @brief 估计频域内输入输出的响应关系
 * @param[in] ins 类实例
 * @param[in&out] cur_item 当前的数据
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <ttab_trans_relh>Author    <th>Description
 * <tr><td>2024/02/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void EstimatLatCtlResInFrequencyDomain(
    LatCtlAnalyseInstance_t* const ins,
    LatCtlAnalyseStatisticDataItem_t* cur_item) {
  // 初始化估计频域内输入输出的响应关系
  phoenix_com_memset(&(ins->est_res_in_fre), 0, sizeof(ins->est_res_in_fre));

  // 已经保存的数据的数量
  Int32_t items_num = Phoenix_Com_RingBuffer_GetSize(&(ins->statistic.seq_data));
  Int32_t start_idx = items_num+1-LAT_CTL_ANA_FFT_DATA_SIZE;
  if (start_idx < 0) {
    start_idx = 0;
  } else if (start_idx > (LAT_CTL_ANA_STATISTIC_DATA_SIZE-1)) {
    start_idx = (LAT_CTL_ANA_STATISTIC_DATA_SIZE-1);
  } else {
    // nothing to do
  }

  // 读取时域中的数据
  Int32_t idx = 0;
  RingBufferIterator_t it_begin;
  RingBufferIterator_t it_end;
  Phoenix_Com_RingBuffer_SetItToBegin(&(ins->statistic.seq_data), &it_begin);
  Phoenix_Com_RingBufferIterator_Move(&it_begin, start_idx);
  Phoenix_Com_RingBuffer_SetItToEnd(&(ins->statistic.seq_data), &it_end);
  while (Phoenix_Com_RingBufferIterator_IsNotEqual(&it_begin, &it_end)) {
    LatCtlAnalyseStatisticDataItem_t* item = 
      (LatCtlAnalyseStatisticDataItem_t*)Phoenix_Com_RingBufferIterator_GetCurrent(&it_begin);

    ins->est_res_in_fre.crt_str_wel_ang.rex[idx] = item->crt_str_wel_ang;
    ins->est_res_in_fre.exp_steady_str_wel_ang.rex[idx] = item->exp_steady_str_wel_ang;
    ins->est_res_in_fre.yaw_rate_tar.rex[idx] = item->yaw_rate_tar;
    ins->est_res_in_fre.yaw_rate_cha.rex[idx] = item->yaw_rate_cha;
    
    ++idx;
    Phoenix_Com_RingBufferIterator_Increase(&it_begin);
  }
  ins->est_res_in_fre.crt_str_wel_ang.rex[idx] = cur_item->crt_str_wel_ang;
  ins->est_res_in_fre.exp_steady_str_wel_ang.rex[idx] = cur_item->exp_steady_str_wel_ang;
  ins->est_res_in_fre.yaw_rate_tar.rex[idx] = cur_item->yaw_rate_tar;
  ins->est_res_in_fre.yaw_rate_cha.rex[idx] = cur_item->yaw_rate_cha;
  ++idx;

  // 进行傅里叶变换，并提取特征频率响应
  // 修正后的方向盘角度 (rad) [减去了零位偏移]
  DoFft(&ins->est_res_in_fre.crt_str_wel_ang);
  cur_item->fd_low_mag_crt_str_wel_ang = 
      ins->est_res_in_fre.crt_str_wel_ang.mag[1] + 
      ins->est_res_in_fre.crt_str_wel_ang.mag[2] +
      ins->est_res_in_fre.crt_str_wel_ang.mag[3];
  cur_item->fd_high_mag_crt_str_wel_ang = 
      //ins->est_res_in_fre.crt_str_wel_ang.mag[4] +
      ins->est_res_in_fre.crt_str_wel_ang.mag[5] + 
      ins->est_res_in_fre.crt_str_wel_ang.mag[6] +
      ins->est_res_in_fre.crt_str_wel_ang.mag[7];
      //ins->est_res_in_fre.crt_str_wel_ang.mag[8];

  // 期望的稳态方向盘角度 (rad)
  DoFft(&ins->est_res_in_fre.exp_steady_str_wel_ang);
  cur_item->fd_low_mag_exp_steady_str_wel_ang = 
      ins->est_res_in_fre.exp_steady_str_wel_ang.mag[1] + 
      ins->est_res_in_fre.exp_steady_str_wel_ang.mag[2] +
      ins->est_res_in_fre.exp_steady_str_wel_ang.mag[3];
  cur_item->fd_high_mag_exp_steady_str_wel_ang = 
      //ins->est_res_in_fre.exp_steady_str_wel_ang.mag[4] +
      ins->est_res_in_fre.exp_steady_str_wel_ang.mag[5] + 
      ins->est_res_in_fre.exp_steady_str_wel_ang.mag[6] +
      ins->est_res_in_fre.exp_steady_str_wel_ang.mag[7];
      //ins->est_res_in_fre.exp_steady_str_wel_ang.mag[8];

#if 0
  // 横向控制的目标角速度 (rad/s)
  DoFft(&ins->est_res_in_fre.yaw_rate_tar);
  cur_item->fd_low_mag_yaw_rate_tar = 
      ins->est_res_in_fre.yaw_rate_tar.mag[1] + 
      ins->est_res_in_fre.yaw_rate_tar.mag[2] +
      ins->est_res_in_fre.yaw_rate_tar.mag[3];
  cur_item->fd_high_mag_yaw_rate_tar = 
      ins->est_res_in_fre.yaw_rate_tar.mag[4] +
      ins->est_res_in_fre.yaw_rate_tar.mag[5] + 
      ins->est_res_in_fre.yaw_rate_tar.mag[6] +
      ins->est_res_in_fre.yaw_rate_tar.mag[7] +
      ins->est_res_in_fre.yaw_rate_tar.mag[8];

  // 角速度 (rad/s) [底盘反馈的]
  DoFft(&ins->est_res_in_fre.yaw_rate_cha);
  cur_item->fd_low_mag_yaw_rate_cha = 
      ins->est_res_in_fre.yaw_rate_cha.mag[1] + 
      ins->est_res_in_fre.yaw_rate_cha.mag[2] +
      ins->est_res_in_fre.yaw_rate_cha.mag[3];
  cur_item->fd_high_mag_yaw_rate_cha = 
      ins->est_res_in_fre.yaw_rate_cha.mag[4] +
      ins->est_res_in_fre.yaw_rate_cha.mag[5] + 
      ins->est_res_in_fre.yaw_rate_cha.mag[6] +
      ins->est_res_in_fre.yaw_rate_cha.mag[7] +
      ins->est_res_in_fre.yaw_rate_cha.mag[8];
#endif
}

/*
 * @brief 估计输入输出的响应关系
 * @param[in] ins 类实例
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2024/02/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void EstimatLatCtlTransRel(
    LatCtlAnalyseInstance_t* const ins) {
  Float32_t max_x_value = s_key_tab_trans_rel[LAT_CTL_ANA_TRANS_REL_ITEM_NUM-1]+0.5F*0.017453293F;

  // 初始化估计输入输出的响应关系
  for (Int32_t i = 0; i < LAT_CTL_ANA_TRANS_REL_ITEM_NUM; ++i) {
    LatCtlAnalyseTransRelItem_t* item = &(ins->est_trans_rel.tab_trans_rel[i]);
    item->count = 0;
    item->avg_y = 0.0F;
    item->max_y = 0.0F;
  }

  // 读取数据
  RingBufferIterator_t it_begin;
  RingBufferIterator_t it_end;
  Phoenix_Com_RingBuffer_SetItToBegin(&(ins->statistic.seq_data), &it_begin);
  Phoenix_Com_RingBuffer_SetItToEnd(&(ins->statistic.seq_data), &it_end);
  while (Phoenix_Com_RingBufferIterator_IsNotEqual(&it_begin, &it_end)) {
    LatCtlAnalyseStatisticDataItem_t* data_item = 
      (LatCtlAnalyseStatisticDataItem_t*)Phoenix_Com_RingBufferIterator_GetCurrent(&it_begin);

    Float32_t x_value = data_item->fd_low_mag_exp_steady_str_wel_ang;
    if (x_value > max_x_value) {
      Phoenix_Com_RingBufferIterator_Increase(&it_begin);
      continue;
    }
    Float32_t y_value = data_item->fd_low_mag_crt_str_wel_ang;

    Float32_t key_ratio = 0.0F;
    Int32_t key_idx = Phoenix_Common_LerpInOrderedTable_f(
        s_key_tab_trans_rel, LAT_CTL_ANA_TRANS_REL_ITEM_NUM, x_value, &key_ratio);
    if (key_ratio >= 0.5F) {
      key_idx += 1;
    }
    LatCtlAnalyseTransRelItem_t* trans_item = &(ins->est_trans_rel.tab_trans_rel[key_idx]);

    Float32_t prev_count = trans_item->count;
    Float32_t prev_avg_y = trans_item->avg_y;
    trans_item->avg_y = (prev_count/(prev_count+1.0F))*prev_avg_y + (1.0F/(prev_count+1.0F))*y_value;
    if (y_value > trans_item->max_y) {
      trans_item->max_y = y_value;
    }
    trans_item->count++;

    Phoenix_Com_RingBufferIterator_Increase(&it_begin);
  }

  // 计算输入输出比率
  for (Int32_t i = 0; i < LAT_CTL_ANA_TRANS_REL_ITEM_NUM; ++i) {
    LatCtlAnalyseTransRelItem_t* trans_item = &(ins->est_trans_rel.tab_trans_rel[i]);
    if (trans_item->count > 0) {
      trans_item->total_count += trans_item->count;
      if (trans_item->total_count > 100000) {
        trans_item->total_count = 100000;
      }

      Float32_t y_value = 0.9F*trans_item->max_y + 0.1F*trans_item->avg_y;
      y_value = y_value > 0.2F*0.017453293F ? y_value : 0.2F*0.017453293F;
      Float32_t trans_ratio = trans_item->key / y_value;
      if (trans_item->trans_ratio < 0.01F) {
        trans_item->trans_ratio = trans_ratio;
      } else {
        if (trans_ratio > trans_item->trans_ratio) {
          trans_item->trans_ratio = 0.50F*trans_item->trans_ratio + 0.50F*trans_ratio;
        } else {
          trans_item->trans_ratio = 0.50F*trans_item->trans_ratio + 0.50F*trans_ratio;
        }
      }
    }
  }

  // 平滑相邻的输入输出比率
  for (Int32_t i = 0; i < LAT_CTL_ANA_TRANS_REL_ITEM_NUM; ++i) {
    LatCtlAnalyseTransRelItem_t* curr_item = &(ins->est_trans_rel.tab_trans_rel[i]);
    LatCtlAnalyseTransRelItem_t* prev_item = Null_t;
    LatCtlAnalyseTransRelItem_t* next_item = Null_t;
    if (i > 0) {
      prev_item = &(ins->est_trans_rel.tab_trans_rel[i-1]);
      if (prev_item->count < 1) {
        prev_item = Null_t;
      }
    }
    if (i < (LAT_CTL_ANA_TRANS_REL_ITEM_NUM-1)) {
      next_item = &(ins->est_trans_rel.tab_trans_rel[i+1]);
      if (next_item->count < 1) {
        next_item = Null_t;
      }
    }
    if (curr_item->count > 0) {
      if ((Null_t != prev_item) && (Null_t != next_item)) {
        curr_item->trans_ratio = 
            0.25F*(prev_item->trans_ratio + next_item->trans_ratio) + 
            0.50F*curr_item->trans_ratio;
      } else if ((Null_t != prev_item) && (Null_t == next_item)) {
        curr_item->trans_ratio = 
            0.25F*prev_item->trans_ratio + 
            0.75F*curr_item->trans_ratio;
      } else if ((Null_t == prev_item) && (Null_t != next_item)) {
        curr_item->trans_ratio = 
            0.25F*next_item->trans_ratio + 
            0.75F*curr_item->trans_ratio;
      } else {
        // nothing to do
      }
    } else {
      if ((Null_t != prev_item) && (Null_t != next_item)) {
        curr_item->trans_ratio = 
            0.30F*(prev_item->trans_ratio + next_item->trans_ratio) + 
            0.40F*curr_item->trans_ratio;
      } else if ((Null_t != prev_item) && (Null_t == next_item)) {
        curr_item->trans_ratio = 
            0.60F*prev_item->trans_ratio + 
            0.40F*curr_item->trans_ratio;
      } else if ((Null_t == prev_item) && (Null_t != next_item)) {
        curr_item->trans_ratio = 
            0.60F*next_item->trans_ratio + 
            0.40F*curr_item->trans_ratio;
      } else {
        // nothing to do
      }
    }
  }

  // 计算补偿值
  for (Int32_t i = 0; i < LAT_CTL_ANA_TRANS_REL_ITEM_NUM; ++i) {
    LatCtlAnalyseTransRelItem_t* curr_item = &(ins->est_trans_rel.tab_trans_rel[i]);

    if (curr_item->trans_ratio > 0.01F) {
      Float32_t feed_ratio = 1.0F;
      if (curr_item->trans_ratio > 0.7F) {
        feed_ratio = 0.7F / curr_item->trans_ratio;
      } else if (curr_item->trans_ratio < 0.5F) {
        feed_ratio = 0.5F / curr_item->trans_ratio;
      } else {
        feed_ratio = 1.0F;
      }
      if (feed_ratio > 1.8F) {
        feed_ratio = 1.8F;
      }
      if (feed_ratio < 0.8F) {
        feed_ratio = 0.8F;
      }
      curr_item->feed_ratio = feed_ratio;
    } else {
      curr_item->feed_ratio = 1.0F;
    }
  }
  static const Int32_t kFixedRoomIdx = 6;
  LatCtlAnalyseTransRelItem_t* fixed_item = &(ins->est_trans_rel.tab_trans_rel[kFixedRoomIdx]);
  for (Int32_t i = 0; i < kFixedRoomIdx; ++i) {
    LatCtlAnalyseTransRelItem_t* curr_item = &(ins->est_trans_rel.tab_trans_rel[i]);
    if (curr_item->count > 0) {
      if (curr_item->feed_ratio < fixed_item->feed_ratio) {
        fixed_item = curr_item;
      }
    }
  }
  for (Int32_t i = 0; i <= kFixedRoomIdx; ++i) {
    LatCtlAnalyseTransRelItem_t* curr_item = &(ins->est_trans_rel.tab_trans_rel[i]);

    curr_item->feed_ratio = fixed_item->feed_ratio;
  }

  // 更新生命值
  for (Int32_t i = 0; i < LAT_CTL_ANA_TRANS_REL_ITEM_NUM; ++i) {
    LatCtlAnalyseTransRelItem_t* curr_item = &(ins->est_trans_rel.tab_trans_rel[i]);

    if (curr_item->count < 1) {
      curr_item->life--;
      if (curr_item->life < 0) {
        curr_item->life = 0;
      }
    } else if (curr_item->count > 3) {
      curr_item->life = LAT_CTL_ANA_TRANS_REL_MAX_ITEM_LIFE;
    } else {
      curr_item->life++;
      if (curr_item->life > LAT_CTL_ANA_TRANS_REL_MAX_ITEM_LIFE) {
        curr_item->life = LAT_CTL_ANA_TRANS_REL_MAX_ITEM_LIFE;
      }
    }
  }

  // 调整各个区域的补偿值(靠近0->补偿值大，远离0->补偿值小)
  Int32_t max_feed_ratio_idx = -1;
  Float32_t max_feed_ratio = 0.0F;
  for (Int32_t i = kFixedRoomIdx; i < LAT_CTL_ANA_TRANS_REL_ITEM_NUM; ++i) {
    LatCtlAnalyseTransRelItem_t* curr_item = &(ins->est_trans_rel.tab_trans_rel[i]);

    //if (curr_item->life > 0) {
    if (curr_item->count > 0) {
      if (curr_item->feed_ratio > max_feed_ratio) {
        max_feed_ratio = curr_item->feed_ratio;
        max_feed_ratio_idx = i;
      }
    }
  }
  if (max_feed_ratio_idx >= 0) {
    LatCtlAnalyseTransRelItem_t* max_item = &(ins->est_trans_rel.tab_trans_rel[max_feed_ratio_idx]);
    for (Int32_t i = kFixedRoomIdx; i < LAT_CTL_ANA_TRANS_REL_ITEM_NUM; ++i) {
      LatCtlAnalyseTransRelItem_t* curr_item = &(ins->est_trans_rel.tab_trans_rel[i]);

      Float32_t update_ratio = 0.0F;
      if (curr_item->life > 0) {
        if (curr_item->life < max_item->life) {
          update_ratio = 0.1F;
        } else {
          update_ratio = 0.05F;
        }
      } else {
        update_ratio = 0.2F;
      }

      if (((i < max_feed_ratio_idx) && (curr_item->feed_ratio < max_item->feed_ratio)) ||
          ((i > max_feed_ratio_idx) && (curr_item->feed_ratio > max_item->feed_ratio))) {
        curr_item->trans_ratio = 
            (1.0F - update_ratio) * curr_item->trans_ratio +
            update_ratio * max_item->trans_ratio;
        curr_item->feed_ratio = 
            (1.0F - update_ratio) * curr_item->feed_ratio +
            update_ratio * max_item->feed_ratio;
      }
    }
  }

#if 0
  printf("### TransRel: \n");
  for (Int32_t i = 0; i < LAT_CTL_ANA_TRANS_REL_ITEM_NUM; ++i) {
    LatCtlAnalyseTransRelItem_t* item = &(ins->est_trans_rel.tab_trans_rel[i]);
    printf("    [%d]: key=%0.2f, life=%d, count=%d(%d), avg_y=%0.2f, max_y=%0.2f, "
          "trans_ratio=%0.3f, feed_ratio=%0.3f\n",
          i, item->key*57.29578F, item->life, item->count, item->total_count, 
          item->avg_y*57.29578F, item->max_y*57.29578F, 
          item->trans_ratio, item->feed_ratio);
  }
#endif
}

/*
 * @brief 更新蛇形抑制
 * @param[in] ins 类实例
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2024/02/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void UpdateHuntingSuppress(
    LatCtlAnalyseInstance_t* const ins, 
    const LatCtlAnalyseDataSource_t* data_source) {
  LatCtlAnalyseStatisticDataItem_t* item = 
      Phoenix_Com_RingBuffer_GetBack(&(ins->statistic.seq_data));

  if (Null_t == item) {
    ins->hunting_suppress.hunting_ratio_hig_fre = 1.0F;
    ins->hunting_suppress.hunting_ratio_d1_yaw_rate = 1.0F;
    return;
  }
  
  Float32_t mag_diff = item->fd_high_mag_crt_str_wel_ang - 3.0F*0.017453293F;
  if (mag_diff > 0.01F) {
    Float32_t key_ratio_hunting = 0.0F;
    Int32_t key_idx_hunting = Phoenix_Common_LerpInOrderedTable_f(
        s_key_tab_hunting_suppress_high_fre, LAT_CTL_PID_LERP_TABLE_SIZE,
        mag_diff, &key_ratio_hunting);
    ins->hunting_suppress.hunting_ratio_hig_fre = Phoenix_Common_Lerp_f(
        s_val_tab_hunting_suppress_high_fre[key_idx_hunting],
        s_val_tab_hunting_suppress_high_fre[key_idx_hunting+1],
        key_ratio_hunting);
#if 1
    printf("!!!!! Suppress by **high_fre: mag_diff=%0.1f, hunting_ratio_hig_fre=%0.2f\n",
        mag_diff*57.29578F , ins->hunting_suppress.hunting_ratio_hig_fre);
#endif
  } else {
    ins->hunting_suppress.hunting_ratio_hig_fre = 1.0F;
  }

  // 检测过大的角速度变化率
  //Float32_t d1_yaw_rate_diff = phoenix_com_abs_f(item->d1_yaw_rate_cha) - 0.8F*0.017453293F;
  Float32_t d1_yaw_rate_diff = phoenix_com_abs_f(data_source->chassis.yaw_rate_chg_rate) - 0.7F*0.017453293F;
  // 根据曲率变化，适当放大对角速度变化的限制
  if (d1_yaw_rate_diff > 0.0001F) {
    static const Float128_t kCurvatureThreshold = 0.0001F;
    Float32_t abs_curr_curvature = phoenix_com_abs_f(data_source->trj_info.ref_trj.curr_curvature);
    Float32_t abs_prev_curvature = phoenix_com_abs_f(data_source->trj_info.ref_trj.prev_curvature);
    Float32_t curvature_diff = 
        data_source->trj_info.ref_trj.curr_curvature - 
        data_source->trj_info.ref_trj.prev_curvature;
    Float32_t curve_threshold = 0.1F * (abs_curr_curvature > abs_prev_curvature ? abs_curr_curvature : abs_prev_curvature);
    curve_threshold = curve_threshold > kCurvatureThreshold ? curve_threshold : kCurvatureThreshold;
    Float32_t key_yaw_rate = phoenix_com_abs_f(curvature_diff*data_source->chassis.veh_speed);
    Float32_t key_ratio = 0.0F;
    Int32_t key_idx = Phoenix_Common_LerpInOrderedTable_f(
        s_key_tab_hunting_suppress_curvature, LAT_CTL_PID_LERP_TABLE_SIZE,
        key_yaw_rate, &key_ratio);
    Float32_t hunting_curvature = Phoenix_Common_Lerp_f(
        s_val_tab_hunting_suppress_curvature[key_idx],
        s_val_tab_hunting_suppress_curvature[key_idx+1],
        key_ratio) * 1.4F;
    if (curvature_diff > curve_threshold) {
      // 出右弯 or 入左弯, 允许向左较大的角速度变化率
      if (item->d1_yaw_rate_cha > 0.001F) {
#if 1
        printf("出右弯 or 入左弯, 允许向左较大的角速度变化率, \n"
               "    d1_yaw_rate_diff=%0.2f, curv_diff=%0.6f, key=%0.2f, h=%0.2f\n",
               d1_yaw_rate_diff*57.3F, curvature_diff, key_yaw_rate*57.3F, hunting_curvature*57.3F);
#endif
        d1_yaw_rate_diff -= hunting_curvature;
      }
    } else if (curvature_diff < -curve_threshold) {
      // 出左弯 or 入右弯, 允许向右较大的角速度变化率
      if (item->d1_yaw_rate_cha < -0.001F) {
#if 1
        printf("出左弯 or 入右弯, 允许向右较大的角速度变化率, \n"
               "    d1_yaw_rate_diff=%0.2f, curv_diff=%0.6f, key=%0.1f, h=%0.1f\n",
               d1_yaw_rate_diff*57.3F, curvature_diff, key_yaw_rate*57.3F, hunting_curvature*57.3F);
#endif
        d1_yaw_rate_diff -= hunting_curvature;
      }
    } else {
      // 轨迹曲率不变 (直线 or 弯道), 不允许较大的角速度变化率
      // nothing to do
    }
  }
  // 检测到过大的角速度变化率，降低控制增益
  if (d1_yaw_rate_diff > 0.0001F) {
    Float32_t key_ratio_hunting = 0.0F;
    Int32_t key_idx_hunting = Phoenix_Common_LerpInOrderedTable_f(
        s_key_tab_hunting_suppress_d1_yaw_rate, LAT_CTL_PID_LERP_TABLE_SIZE,
        d1_yaw_rate_diff, &key_ratio_hunting);
    Float32_t hunting_ratio = Phoenix_Common_Lerp_f(
        s_val_tab_hunting_suppress_d1_yaw_rate[key_idx_hunting],
        s_val_tab_hunting_suppress_d1_yaw_rate[key_idx_hunting+1],
        key_ratio_hunting);
    if (hunting_ratio < ins->hunting_suppress.hunting_ratio_d1_yaw_rate) {
      ins->hunting_suppress.hunting_ratio_d1_yaw_rate = hunting_ratio;
    }
  }

#if 0
  if (ins->hunting_suppress.hunting_ratio_d1_yaw_rate < 1.0F) {
    ins->hunting_suppress.hunting_ratio_d1_yaw_rate += 0.02F;

    if (ins->hunting_suppress.hunting_ratio_d1_yaw_rate > 1.0F) {
      ins->hunting_suppress.hunting_ratio_d1_yaw_rate = 1.0F;
    }
  }
#else
  if (ins->hunting_suppress.hunting_ratio_d1_yaw_rate < 1.0F) {
    ins->hunting_suppress.hunting_ratio_d1_yaw_rate += 0.015F;

    if (ins->hunting_suppress.hunting_ratio_d1_yaw_rate > 1.0F) {
      ins->hunting_suppress.hunting_ratio_d1_yaw_rate = 1.0F;
    }

#if 1
    printf("!!!!! Suppress by ^^d1_yaw_rate: d1_diff=%0.1f, hunting_ratio_d1_yaw_rate=%0.2f\n",
        d1_yaw_rate_diff*57.29578F , ins->hunting_suppress.hunting_ratio_d1_yaw_rate);
#endif
  }
#endif

  // 刚进入自动模式，不进行高频蛇行抑制
  if (Phoenix_Com_Timer_IsActive(&(ins->timers.eps_auto)) && item->eps_auto) {
    printf("!!! 刚进入自动模式，不进行高频蛇行抑制 !!!\n");
    ins->hunting_suppress.hunting_ratio_hig_fre = 1.0F;
    ins->hunting_suppress.hunting_ratio_d1_yaw_rate = 1.0F;
  } else {
    printf("!!! 自动模式，进行高频蛇行抑制 !!!\n");
  }

}


/******************************************************************************/
/* 外部函数                                                                    */
/******************************************************************************/

/*
 * @brief 初始化横向控制分析器
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2024/02/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_LatCtlAnalyse_Initialize(
    LatCtlAnalyseInstance_t* const ins) {
  // 初始化时间戳
  ins->timestamp = 0;

  // 初始化配置项
  // 样本间隔时间
  ins->conf.sample_interval = 0.05F;        /* 0.05s 采样间隔 */
  // 角速度响应延迟偏移量
  ins->conf.yaw_rate_res_delay_offset = 8;  /* 8* 0.05s = 0.4s 延迟时间 */

  // 初始化统计数据缓存
  Phoenix_Com_RingBuffer_Init(
      &(ins->statistic.seq_data), LAT_CTL_ANA_STATISTIC_DATA_SIZE,
      sizeof(LatCtlAnalyseStatisticDataItem_t), ins->statistic.seq_data_buf);

  // 初始化估计方向盘角度间隙缓存
  Phoenix_Com_RingBuffer_Init(
      &(ins->est_str_gap.queue.ring_buf), LAT_CTL_ANA_STR_GAP_WND_SIZE,
      sizeof(Float32_t), ins->est_str_gap.queue.data_buf);

  // 初始化估计输入输出的响应关系
  for (Int32_t i = 0; i < LAT_CTL_ANA_TRANS_REL_ITEM_NUM; ++i) {
    LatCtlAnalyseTransRelItem_t* item = &(ins->est_trans_rel.tab_trans_rel[i]);
    item->key = s_key_tab_trans_rel[i];
    item->life = 0;
    item->total_count = 0;
    item->count = 0;
    item->avg_y = 0.0F;
    item->max_y = 0.0F;
    item->trans_ratio = 0.0F;
    item->feed_ratio = 1.0F;
  }
  
  // 重置横向控制分析器
  Phoenix_LatCtlAnalyse_Reset(ins);

  // 初始化定时器
  InitializeTimers(ins);
}

/*
 * @brief 重置横向控制分析器的内部状态
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2024/02/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_LatCtlAnalyse_Reset(
    LatCtlAnalyseInstance_t* const ins) {
  // 清空统计数据
  phoenix_com_memset(ins->statistic.seq_data_buf, 0, sizeof(ins->statistic.seq_data_buf));
  Phoenix_Com_RingBuffer_Clear(&(ins->statistic.seq_data));
  // 方向盘角度零位偏移 (rad)
  // ins->statistic.avg_str_wel_ang_offset = 0.0F;
  ins->statistic.max_str_wel_ang_offset = 0.0F;
  // 方向盘角度间隙 (rad)
  // ins->statistic.avg_str_wel_ang_gap = 0.0F;
  ins->statistic.max_str_wel_ang_gap = 0.0F;

  // 初始化估计方向盘角度间隙
  ins->est_str_gap.wnd_itg_d1_str_wel_ang = 0.0F;
  ins->est_str_gap.max_str_wel_ang_gap_in_wnd = 0.0F;
  phoenix_com_memset(ins->est_str_gap.queue.data_buf, 0, sizeof(ins->est_str_gap.queue.data_buf));
  Phoenix_Com_RingBuffer_Clear(&(ins->est_str_gap.queue.ring_buf));

  // 初始化估计频域内输入输出的响应关系
  phoenix_com_memset(&(ins->est_res_in_fre), 0, sizeof(ins->est_res_in_fre));

  // 初始化蛇形抑制
  ins->hunting_suppress.hunting_ratio_hig_fre = 1.0F;
  ins->hunting_suppress.hunting_ratio_d1_yaw_rate = 1.0F;
}

/*
 * @brief 配置横向控制分析器的参数
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2024/02/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_LatCtlAnalyse_Configurate(
    LatCtlAnalyseInstance_t* const ins, 
    const LatCtlAnalyseConf_t* conf) {
  phoenix_com_memcpy(&(ins->conf), conf, sizeof(ins->conf));
}

/*
 * @brief 更新横向控制分析器
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2024/02/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_LatCtlAnalyse_Update(
    LatCtlAnalyseInstance_t* const ins,
    const LatCtlAnalyseDataSource_t* data_source) {
  // 定义常量
  static const Int32_t kD1FilterWndSize = 5;
  static const Int32_t kStrAvgWndSize = 200;
  static const Int32_t kStrItgWndSize = 10;

  // 定义局部变量
  Int64_t time_elapsed = 0;
  LatCtlAnalyseStatisticDataItem_t item;
  // 初始化统计数据项
  phoenix_com_memset(&item, 0, sizeof(item));

  // 更新定时器
  UpdateTimers(ins);

  // 检测是否超时
  time_elapsed = Phoenix_Common_CalcElapsedClockMs(
        ins->timestamp, data_source->timestamp);
  if ((time_elapsed > 200) || (time_elapsed < 0)) {
    Phoenix_LatCtlAnalyse_Reset(ins);
    LOG_ERR_C("[CTL][LatCtlAnalyse] Time out.");
  }
  // 更新时间戳
  ins->timestamp = data_source->timestamp;

  // 已经保存的数据的数量
  Int32_t items_num = Phoenix_Com_RingBuffer_GetSize(&(ins->statistic.seq_data));
  // 之前时刻保存的数据
  LatCtlAnalyseStatisticDataItem_t* pre_item = 
      Phoenix_Com_RingBuffer_GetBack(&(ins->statistic.seq_data));
  LatCtlAnalyseStatisticDataItem_t* old_item_d1 = 
      Phoenix_Com_RingBuffer_GetDataByIdx(&(ins->statistic.seq_data), items_num-kD1FilterWndSize);
  LatCtlAnalyseStatisticDataItem_t* old_item_str_avg = 
      Phoenix_Com_RingBuffer_GetDataByIdx(&(ins->statistic.seq_data), items_num-kStrAvgWndSize);
  LatCtlAnalyseStatisticDataItem_t* old_item_str_itg = 
      Phoenix_Com_RingBuffer_GetDataByIdx(&(ins->statistic.seq_data), items_num-kStrItgWndSize);

  // 检测是否刚进入自动模式
  Int8_t prev_eps_auto = 0;
  if (Null_t != pre_item) {
    prev_eps_auto = pre_item->eps_auto;
  }
  if (0 == prev_eps_auto && 0 != data_source->chassis.eps_auto) {
    // 刚进入自动模式
    Phoenix_Com_Timer_SetTimeout(&(ins->timers.eps_auto), 10*1000);
    Phoenix_Com_Timer_Restart(&(ins->timers.eps_auto));
  }

  /// 构建统计数据
  // 时间戳
  item.timestamp = data_source->timestamp;
  // 转向机是否处于自动模式
  item.eps_auto = data_source->chassis.eps_auto;
  // 车速(m/s)
  item.veh_speed = data_source->chassis.veh_speed;
  // 用于分母的车速
  Float32_t speed_for_denominator = item.veh_speed;
  if (speed_for_denominator < 10.0F/3.6F) {
    speed_for_denominator = 10.0F/3.6F;
  }

  // 方向盘角度 (rad)
  item.str_wel_ang = data_source->chassis.steering_wheel_angle;
  // 横向控制的目标角速度 (rad/s)
  item.yaw_rate_tar = data_source->ctl_info.yaw_rate_tar;
  // 角速度 (rad/s) [底盘反馈的]
  item.yaw_rate_cha = data_source->chassis.yaw_rate_cha;
  // 角速度 (rad/s) [方向盘反向计算的]
  item.yaw_rate_str = Phoenix_VehModel_CalcYawRateFromSteeringAngle(
      item.str_wel_ang, item.veh_speed);
  // 期望的稳态方向盘角度 (rad)
  item.exp_steady_str_wel_ang = Phoenix_VehModel_CalcSteeringAngleFromYawRate(
      item.yaw_rate_cha, speed_for_denominator);
  // item.exp_steady_str_wel_ang += data_source->ctl_info.understeer_feed;
  Float32_t understeer_feed = CalcUndersteerFeed(ins, item.yaw_rate_cha, item.veh_speed);
  item.exp_steady_str_wel_ang += understeer_feed;

#if 0
  printf("### str_wel_ang=%0.1f, str_from_yaw_rate=%0.1f, understeer_feed=%0.1f, exp_steady_str=%0.1f\n",
      phoenix_com_rad2deg_f(item.str_wel_ang), 
      phoenix_com_rad2deg_f(Phoenix_VehModel_CalcSteeringAngleFromYawRate(
                                item.yaw_rate_cha, speed_for_denominator)),
      phoenix_com_rad2deg_f(data_source->ctl_info.understeer_feed),
      phoenix_com_rad2deg_f(item.exp_steady_str_wel_ang)
      );
#endif

  // 首次保存数据
  if (items_num < 1) {
    // 修正后的方向盘角度 (rad) [减去了零位偏移]
    item.crt_str_wel_ang = item.str_wel_ang;
    // 保存首次数据, 并返回
    Phoenix_Com_RingBuffer_PushBackOverride(&(ins->statistic.seq_data), &item);
    return;
  }


  // 方向盘角度变化率 (rad/s)
  item.raw_d1_str_wel_ang = (item.str_wel_ang - pre_item->str_wel_ang) / ins->conf.sample_interval;
  if (Null_t == old_item_d1) {
    item.d1_str_wel_ang =  Phoenix_Com_Filter_MovingAverage_f(
        kD1FilterWndSize, items_num, pre_item->d1_str_wel_ang, item.raw_d1_str_wel_ang, 0.0F);
  } else {
    item.d1_str_wel_ang = Phoenix_Com_Filter_MovingAverage_f(
        kD1FilterWndSize, kD1FilterWndSize, pre_item->d1_str_wel_ang, item.raw_d1_str_wel_ang, old_item_d1->raw_d1_str_wel_ang);
  }
  // 期望的稳态方向盘角度变化率 (rad/s)
  item.raw_d1_exp_steady_str_wel_ang = (item.exp_steady_str_wel_ang - pre_item->exp_steady_str_wel_ang) / ins->conf.sample_interval;
  if (Null_t == old_item_d1) {
    item.d1_exp_steady_str_wel_ang = Phoenix_Com_Filter_MovingAverage_f(
        kD1FilterWndSize, items_num, pre_item->d1_exp_steady_str_wel_ang, item.raw_d1_exp_steady_str_wel_ang, 0.0F);
  } else {
    item.d1_exp_steady_str_wel_ang = Phoenix_Com_Filter_MovingAverage_f(
        kD1FilterWndSize, kD1FilterWndSize, pre_item->d1_exp_steady_str_wel_ang, 
        item.raw_d1_exp_steady_str_wel_ang, old_item_d1->raw_d1_exp_steady_str_wel_ang);
  }
  // 方向盘角度平均值 (rad) [指定的窗口内]
  if (Null_t == old_item_str_avg) {
    item.avg_str_wel_ang = Phoenix_Com_Filter_MovingAverage_f(
        kStrAvgWndSize, items_num, pre_item->avg_str_wel_ang, item.str_wel_ang, 0.0F);
  } else {
    item.avg_str_wel_ang = Phoenix_Com_Filter_MovingAverage_f(
        kStrAvgWndSize, kStrAvgWndSize, pre_item->avg_str_wel_ang, 
        item.str_wel_ang, old_item_str_avg->str_wel_ang);
  }
  // 期望的稳态方向盘角度平均值 (rad) [指定的窗口内]
  if (Null_t == old_item_str_avg) {
    item.avg_exp_steady_str_wel_ang = Phoenix_Com_Filter_MovingAverage_f(
        kStrAvgWndSize, items_num, pre_item->avg_exp_steady_str_wel_ang, item.exp_steady_str_wel_ang, 0.0F);
  } else {
    item.avg_exp_steady_str_wel_ang = Phoenix_Com_Filter_MovingAverage_f(
        kStrAvgWndSize, kStrAvgWndSize, pre_item->avg_exp_steady_str_wel_ang, 
        item.exp_steady_str_wel_ang, old_item_str_avg->exp_steady_str_wel_ang);
  }
  // 方向盘角度变化率积分值 (rad) [指定的窗口内]
  if (Null_t == old_item_str_itg) {
    item.wnd_itg_d1_str_wel_ang = pre_item->wnd_itg_d1_str_wel_ang + item.d1_str_wel_ang * ins->conf.sample_interval;
  } else {
    item.wnd_itg_d1_str_wel_ang = pre_item->wnd_itg_d1_str_wel_ang + 
        (item.d1_str_wel_ang - old_item_str_itg->d1_str_wel_ang) * ins->conf.sample_interval;
  }
  // 期望的稳态方向盘角度变化率积分值 (rad) [指定的窗口内]
  if (Null_t == old_item_str_itg) {
    item.wnd_itg_d1_exp_steady_str_wel_ang = pre_item->wnd_itg_d1_exp_steady_str_wel_ang + 
        item.d1_exp_steady_str_wel_ang * ins->conf.sample_interval;
  } else {
    item.wnd_itg_d1_exp_steady_str_wel_ang = pre_item->wnd_itg_d1_exp_steady_str_wel_ang + 
        (item.d1_exp_steady_str_wel_ang - old_item_str_itg->d1_exp_steady_str_wel_ang) * ins->conf.sample_interval;
  }


  // 角速度变化率 (rad/s^2) [底盘反馈的]
  item.raw_d1_yaw_rate_cha = (item.yaw_rate_cha - pre_item->yaw_rate_cha) / ins->conf.sample_interval;
  if (Null_t == old_item_d1) {
    item.d1_yaw_rate_cha =  Phoenix_Com_Filter_MovingAverage_f(
        kD1FilterWndSize, items_num, pre_item->d1_yaw_rate_cha, item.raw_d1_yaw_rate_cha, 0.0F);
  } else {
    item.d1_yaw_rate_cha = Phoenix_Com_Filter_MovingAverage_f(
        kD1FilterWndSize, kD1FilterWndSize, pre_item->d1_yaw_rate_cha, item.raw_d1_yaw_rate_cha, old_item_d1->raw_d1_yaw_rate_cha);
  }
  // printf("===  d1_yaw_rate=%0.2f\n", item.d1_yaw_rate_cha*57.29578F);
  // 角速度变化率 (rad/s^2) [方向盘反向计算的]
  item.raw_d1_yaw_rate_str = (item.yaw_rate_str - pre_item->yaw_rate_str) / ins->conf.sample_interval;
  if (Null_t == old_item_d1) {
    item.d1_yaw_rate_str =  Phoenix_Com_Filter_MovingAverage_f(
        kD1FilterWndSize, items_num, pre_item->d1_yaw_rate_str, item.raw_d1_yaw_rate_str, 0.0F);
  } else {
    item.d1_yaw_rate_str = Phoenix_Com_Filter_MovingAverage_f(
        kD1FilterWndSize, kD1FilterWndSize, pre_item->d1_yaw_rate_str, item.raw_d1_yaw_rate_str, old_item_d1->raw_d1_yaw_rate_str);
  }
  // 角速度变化率积分值 (rad/s) [底盘反馈的, 指定的窗口内]
  if (Null_t == old_item_str_itg) {
    item.wnd_itg_d1_yaw_rate_cha = pre_item->wnd_itg_d1_yaw_rate_cha + item.d1_yaw_rate_cha * ins->conf.sample_interval;
  } else {
    item.wnd_itg_d1_yaw_rate_cha = pre_item->wnd_itg_d1_yaw_rate_cha + 
        (item.d1_yaw_rate_cha - old_item_str_itg->d1_yaw_rate_cha) * ins->conf.sample_interval;
  }
  // 角速度变化率积分值 (rad/s) [方向盘反向计算的, 指定的窗口内]
  if (Null_t == old_item_str_itg) {
    item.wnd_itg_d1_yaw_rate_str = pre_item->wnd_itg_d1_yaw_rate_str + item.d1_yaw_rate_str * ins->conf.sample_interval;
  } else {
    item.wnd_itg_d1_yaw_rate_str = pre_item->wnd_itg_d1_yaw_rate_str + 
        (item.d1_yaw_rate_str - old_item_str_itg->d1_yaw_rate_str) * ins->conf.sample_interval;
  }


  // 估计方向盘转角零位偏移
  EstimateSteeringWheelAngleOffset(ins, pre_item, &item);
  // 修正后的方向盘角度 (rad) [减去了零位偏移]
  item.crt_str_wel_ang = item.str_wel_ang + item.str_wel_ang_offset;
  // 估计方向盘转角间隙
  EstimateSteeringWheelAngleGap(ins, items_num, pre_item, &item);

  // 估计频域内输入输出的响应关系
  EstimatLatCtlResInFrequencyDomain(ins, &item);

  // 保存数据
  Phoenix_Com_RingBuffer_PushBackOverride(&(ins->statistic.seq_data), &item);

  // 估计输入输出的响应关系
  EstimatLatCtlTransRel(ins);

  // 蛇形抑制
  UpdateHuntingSuppress(ins, data_source);
}

/*
 * @brief 获取横向控制分析的结果
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2024/02/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Phoenix_LatCtlAnalyse_GetAnaRet(
    LatCtlAnalyseInstance_t* const ins,
    LatCtlAnalyseRet_t* const ana_ret) {
  // 最新的数据
  LatCtlAnalyseStatisticDataItem_t* item = 
      Phoenix_Com_RingBuffer_GetBack(&(ins->statistic.seq_data));
  
  if (Null_t == item) {
    phoenix_com_memset(ana_ret, 0, sizeof(LatCtlAnalyseRet_t));
    return;
  }

  // 方向盘角度零位偏移 (rad)
  ana_ret->str_wel_ang_offset = item->str_wel_ang_offset;
  ana_ret->max_str_wel_ang_offset = ins->statistic.max_str_wel_ang_offset;
  // 方向盘角度间隙 (rad)
  ana_ret->str_wel_ang_gap = item->str_wel_ang_gap;
  ana_ret->max_str_wel_ang_gap = ins->statistic.max_str_wel_ang_gap;

#if 1
  // 角速度变化率 (rad/s^2) [底盘反馈的]
  ana_ret->d1_yaw_rate_cha = item->d1_yaw_rate_cha;
  // 角速度变化率 (rad/s^2) [方向盘反向计算的]
  ana_ret->d1_yaw_rate_str = item->d1_yaw_rate_str;
#else
  // 角速度变化率 (rad/s^2) [底盘反馈的]
  ana_ret->d1_yaw_rate_cha = item->d1_exp_steady_str_wel_ang;
  // 角速度变化率 (rad/s^2) [方向盘反向计算的]
  ana_ret->d1_yaw_rate_str = item->d1_str_wel_ang;
#endif

  // Frequency domain
  // 修正后的方向盘角度 (rad) [减去了零位偏移]
  ana_ret->fd_low_mag_crt_str_wel_ang = item->fd_low_mag_crt_str_wel_ang;
  ana_ret->fd_high_mag_crt_str_wel_ang = item->fd_high_mag_crt_str_wel_ang;
  // 期望的稳态方向盘角度 (rad)
  ana_ret->fd_low_mag_exp_steady_str_wel_ang = item->fd_low_mag_exp_steady_str_wel_ang;
  ana_ret->fd_high_mag_exp_steady_str_wel_ang = item->fd_high_mag_exp_steady_str_wel_ang;
  // 横向控制的目标角速度 (rad/s)
  ana_ret->fd_low_mag_yaw_rate_tar = item->fd_low_mag_yaw_rate_tar;
  ana_ret->fd_high_mag_yaw_rate_tar = item->fd_high_mag_yaw_rate_tar;
  // 角速度 (rad/s) [底盘反馈的]
  ana_ret->fd_low_mag_yaw_rate_cha = item->fd_low_mag_yaw_rate_cha;
  ana_ret->fd_high_mag_yaw_rate_cha = item->fd_high_mag_yaw_rate_cha;

  for (Int32_t i = 0; i < LAT_CTL_ANA_TRANS_REL_ITEM_NUM; ++i) {
    LatCtlAnalyseTransRelItem_t* curr_item = &(ins->est_trans_rel.tab_trans_rel[i]);
    ana_ret->fd_low_str_trans_ratio_list[i] = curr_item->trans_ratio;
    ana_ret->fd_low_str_feed_ratio_list[i] = curr_item->feed_ratio;
  }
}

/*
 * @brief 获取方向盘低频响应补偿系数
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2024/02/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
Float32_t Phoenix_LatCtlAnalyse_GetStrLowFeqResFeedRatioByYawRate(
    LatCtlAnalyseInstance_t* const ins,
    Float32_t yaw_rate, Float32_t veh_spd) {
  Float32_t speed_for_denominator = veh_spd;
  if (speed_for_denominator < 10.0F/3.6F) {
    speed_for_denominator = 10.0F/3.6F;
  }
  Float32_t feed_ratio = 1.0F;

  // 期望的稳态方向盘角度 (rad)
  Float32_t exp_steady_str_wel_ang = Phoenix_VehModel_CalcSteeringAngleFromYawRate(
      yaw_rate, speed_for_denominator);
  Float32_t understeer_feed = CalcUndersteerFeed(ins, yaw_rate, veh_spd);

#if 0
  printf(">>> from_yaw_rate=%0.2f, understeer_feed=%0.2f, exp=%0.2f\n", 
      exp_steady_str_wel_ang*57.29578F, understeer_feed*57.29578F, 
      (exp_steady_str_wel_ang+understeer_feed)*57.29578F);
#endif

  exp_steady_str_wel_ang += understeer_feed;
  
  feed_ratio = Phoenix_LatCtlAnalyse_GetStrLowFeqResFeedRatioByStrAng(
      ins, exp_steady_str_wel_ang, veh_spd);

  return (feed_ratio);
}

/*
 * @brief 获取方向盘低频响应补偿系数
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2024/02/28  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
Float32_t Phoenix_LatCtlAnalyse_GetStrLowFeqResFeedRatioByStrAng(
    LatCtlAnalyseInstance_t* const ins,
    Float32_t str_ang, Float32_t veh_spd) {
  Float32_t max_key_value = s_key_tab_trans_rel[LAT_CTL_ANA_TRANS_REL_ITEM_NUM-1]+0.5F*0.017453293F;
  Float32_t speed_for_denominator = veh_spd;
  if (speed_for_denominator < 10.0F/3.6F) {
    speed_for_denominator = 10.0F/3.6F;
  }
  Float32_t feed_ratio = 1.0F;

  if (str_ang > max_key_value) {
    return (feed_ratio);
  }

  Float32_t key_ratio = 0.0F;
  Int32_t key_idx = Phoenix_Common_LerpInOrderedTable_f(
      s_key_tab_trans_rel, LAT_CTL_ANA_TRANS_REL_ITEM_NUM, phoenix_com_abs_f(str_ang), &key_ratio);
  if (key_ratio >= 0.5F) {
    key_idx += 1;
  }
  LatCtlAnalyseTransRelItem_t* trans_item = &(ins->est_trans_rel.tab_trans_rel[key_idx]);

  feed_ratio = trans_item->feed_ratio;

#if 0
  LatCtlAnalyseStatisticDataItem_t* item = 
      Phoenix_Com_RingBuffer_GetBack(&(ins->statistic.seq_data));
  
  if (Null_t != item) {
    Float32_t mag_diff = item->fd_high_mag_crt_str_wel_ang - 3.0F*0.017453293F;
    if (mag_diff > 0.01F) {
      Float32_t key_ratio_hunting = 0.0F;
      Int32_t key_idx_hunting = Phoenix_Common_LerpInOrderedTable_f(
          s_key_tab_hunting_suppress_high_fre, LAT_CTL_PID_LERP_TABLE_SIZE,
          mag_diff, &key_ratio_hunting);
      Float32_t hunting_ratio = Phoenix_Common_Lerp_f(
          s_val_tab_hunting_suppress_high_fre[key_idx_hunting],
          s_val_tab_hunting_suppress_high_fre[key_idx_hunting+1],
          key_ratio_hunting);
      feed_ratio *= hunting_ratio;
      printf("    mag_diff=%0.1f, hunting_ratio=%0.2f, feed=%0.2f\n",
          mag_diff*57.29578F , hunting_ratio, feed_ratio);
    }

    printf("--->  d1_yaw_rate=%0.2f\n", item->d1_yaw_rate_cha*57.29578F);
  }
#endif
  
  //feed_ratio = 1.0F;

  Float32_t hunting_ratio = 
      ins->hunting_suppress.hunting_ratio_hig_fre < ins->hunting_suppress.hunting_ratio_d1_yaw_rate ? 
      ins->hunting_suppress.hunting_ratio_hig_fre : ins->hunting_suppress.hunting_ratio_d1_yaw_rate;
  feed_ratio *= hunting_ratio;

  //feed_ratio *= ins->hunting_suppress.hunting_ratio_hig_fre;
  //feed_ratio *= ins->hunting_suppress.hunting_ratio_d1_yaw_rate;

#if 0
  if ((ins->hunting_suppress.hunting_ratio_hig_fre < 1.0F) ||
      (ins->hunting_suppress.hunting_ratio_d1_yaw_rate < 1.0F)) {
    printf("&&&&& hunting: ratio1=%0.2f, ratio2=%0.2f, feed=%0.2f\n", 
        ins->hunting_suppress.hunting_ratio_hig_fre, 
        ins->hunting_suppress.hunting_ratio_d1_yaw_rate, 
        feed_ratio);
  }
#endif

#if 0
  printf(">>> key_idx=%d, key=%0.2f, feed_ratio=%0.3f\n",
      key_idx, trans_item->key*57.29578F, feed_ratio);
#endif

  return (feed_ratio);
}


