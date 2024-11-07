#ifndef RADAR_COMMON_H_
#define RADAR_COMMON_H_



/**
 * @brief 该宏用于控制是否使用基于Eigen编写的卡尔曼滤波
 * 
 */
#define  ENABLE_USING_FILTER_BASED_ON_EIGEN      (0)

/**
 * @brief 该宏用于控制是否使用窗口滑动滤波器
 * 
 */
#define  ENABLE_USING_WINDOW_SMOOTH_FILTER       (0)

/**
 * @brief 该宏用于控制是否使用基于原本MAT库编写的卡尔曼滤波
 * 
 */
#define  ENABLE_USING_ORIGINAL_SMOOTH_FILTER       (1)






#endif
