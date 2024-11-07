/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       log_c.h
 * @brief      调试日志
 * @details    定义了程序调试日志功能(C compiler)
 *
 * @author     pengc
 * @date       2020.05.12
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/12  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_COMMON_LOG_C_H_
#define PHOENIX_COMMON_LOG_C_H_


#define LOG_ERR_C Phoenix_Common_OutputErrLog
#define LOG_WARN_C Phoenix_Common_OutputWarnLog
#define LOG_INFO_C(level, info)	\
  if(level <= 6) {		    \
    Phoenix_Common_OutputInfoLog info ;		\
  }

#define COM_CHECK_C(cond)	\
  if(!(cond)) {		    \
    Phoenix_Common_OutputFatalLog(__FILE__, __LINE__, "CHECK: " #cond) ;		\
  }


#ifdef __cplusplus
extern "C" {
#endif


void Phoenix_Common_InitializeLogging();
void Phoenix_Common_ConfigLogging(
    int log_info_level,
    char enable_output_time_prefix,
    char enable_output_file_prefix,
    char enable_output_to_console,
    char enable_output_to_ring_buff,
    char enable_output_to_file);
int Phoenix_Common_GetLogFromRingBuff(char* log, int size);
void Phoenix_Common_OutputErrLog(const char* fmt, ... );
void Phoenix_Common_OutputWarnLog(const char* fmt, ... );
void Phoenix_Common_OutputInfoLog(const char* fmt, ... );
void Phoenix_Common_OutputFatalLog(
    const char* file, int line, const char* info);


#ifdef __cplusplus
}
#endif


#endif // PHOENIX_COMMON_LOG_C_H_

