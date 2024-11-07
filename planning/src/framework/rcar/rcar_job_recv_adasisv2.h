/**
 * @file rcar_job_recv_adasisv2.h
 * @author wangwh
 * @brief 接收TBOX发出的ADASISv2 CAN报文(经由TC397 SPI发送到RCAR)，转换为结构体，存放到共享数据区，供Planning模块使用
 * @date 2023-08-15
 * 
 */

#ifndef PHOENIX_FRAMEWORK_RCAR_JOB_RECV_ADASISV2_H_
#define PHOENIX_FRAMEWORK_RCAR_JOB_RECV_ADASISV2_H_

#include "utils/macros.h"

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
#include "pcc_map/adasis_v2.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief 执行一次报文读取及解析过程，100ms周期调用，ADASISv2报文是事件触发，需要注意重复报文的过滤
 */
void RCAR_Job_RecvADASISv2_DoJob();


#ifdef __cplusplus
}
#endif

#endif // PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N

#endif // PHOENIX_FRAMEWORK_RCAR_JOB_RECV_ADASISV2_H_
