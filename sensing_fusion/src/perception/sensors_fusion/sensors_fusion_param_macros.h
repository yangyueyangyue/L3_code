#ifndef SENSORS_FUSION_PARAM_MACROS_H_
#define SENSORS_FUSION_PARAM_MACROS_H_





namespace phoenix{
namespace perception{
namespace sensorsfusion{


/**
 * @brief 融合流程开始的准备过程的时空同步用到的参数
 * 
 */ 
/*
UPPER_LIMIT_TIME_ELAPSED_FOR_TRACK_LIST 表示相邻两次融合列表构建的间隔时间上限，单位：ms 。
*/
#define UPPER_LIMIT_TIME_ELAPSED_FOR_TRACK_LIST    (200)  //单位：ms        

/**
 * @brief kd-tree 相关参数
 */
#define MAX_LEAF_DIMENSION_PARAM_OF_KD_TREE    (5.0f)
#define MAX_LEAF_SIZE_PARAM_OF_KD_TREE    (16)

/**
 * @brief 处理传感器输入时的相关参数
 * 
 */
/*
UPPER_LIMIT_OF_TIME_ELAPSED_FOR_CAMERA_LIST_INPUT 表示相邻两次相机数据输入的间隔时间的上限，单位：ms
*/
#define UPPER_LIMIT_OF_TIME_ELAPSED_FOR_CAMERA_LIST_INPUT （500）//单位：ms

/*
UPPER_LIMIT_OF_TIME_ELAPSED_FOR_RADAR_LIST_INPUT 表示相邻两次radar数据输入的间隔时间的上限，单位：ms
*/
#define UPPER_LIMIT_OF_TIME_ELAPSED_FOR_RADAR_LIST_INPUT （500）//单位：ms

/*
* 输入相机数据，尺寸上限定最小值
*/
#define MIN_WIDTH_OF_CAMERA_DATA_FOR_ADDING   (0.5F)     // 宽
#define MIN_HEIGHT_OF_CAMERA_DATA_FOR_ADDING   (0.5F)    // 高
#define MIN_LENGTH_OF_CAMERA_DATA_FOR_ADDING   (0.5F)    // 长

/*
* 输入毫米波雷达数据，尺寸做限定
*/
#define DEFAULT_HEIGHT_OF_CAMERA_DATA_FOR_ADDING   (0.5F)    // 高
#define MIN_LENGTH_OF_RADAR_DATA_FOR_ADDING   (0.5F)    // 长

/**
 * @brief 匹配阶段相关参数，针对前视一体机而言
 * 
 */
/*
针对前视一体机而言，最低已经跟踪上的次数。
*/
#define MIN_MATCHED_TRACKING_TIMES_FOR_MAIN_CAMERA_MATCH_PARAM        (3)
/*
针对前视一体机而言，(粗选)最近最多未匹配上的次数
*/
#define MAX_DISMATCHED_TRACKING_TIMES_FOR_MAIN_CAMERA_MATCH_PARAM     (5)
/*
针对前视一体机而言，(粗选)匹配的obj最大允许的纵向速度差
*/
#define MAX_VX_DIFF_FOR_MATCH_MAIN_CAMERA_PARAM                       (10.0F/3.6F)
/*
针对前视一体机而言，(粗选)匹配的obj最大允许的横向速度差
*/
#define MAX_VY_DIFF_FOR_MATCH_MAIN_CAMERA_PARAM                       (30.0F/3.6F)
/*
针对前视一体机而言，表示是否有精选条件
*/
#define IS_VALID_CONDITION_FOR_MATCH_MAIN_CAMERA_PARAM                (true)
/*
针对前视一体机而言，（精选）匹配的obj最大允许的纵向速度差
*/
#define MAX_VX_DIFF_OF_CONDITION_FOR_MATCH_MAIN_CAMERA_PARAM          (30.0F/3.6F)
/*
针对前视一体机而言，（精选）匹配的obj最大允许的横向速度差
*/
#define MAX_VY_DIFF_OF_CONDITION_FOR_MATCH_MAIN_CAMERA_PARAM          (10.0F/3.6F)
/*
针对前视一体机而言，运行根据当前状态估计位置
*/
#define USING_EST_POS_FOR_MATCH_MAIN_CAMERA_PARAM                     (false)
/*
针对前视一体机而言，表示是否允许多个obj id进行匹配
*/
#define ENABLE_ADD_MUL_OBJ_ID_FOR_MATCH_MAIN_CAMERA_PARAM             (false)
/*
针对前视一体机而言，表示是否允许次优匹配
*/
#define ENABLE_SUB_OPTIMAL_MATCH_FOR_MATCH_MAIN_CAMERA_PARAM          (false)
/*
针对前视一体机而言，表示若根据位置未匹配上，是否根据ID重新匹配
*/
#define MATCH_AGAIN_BY_ID_IF_UNMATCHED_FOR_MATCH_MAIN_CAMERA_PARAM    (true)


/**
 * @brief 匹配阶段相关参数，针对环视相机而言
 */
/*
针对环视相机而言，最低已经跟踪上的次数。
*/
#define MIN_MATCHED_TRACKING_TIMES_FOR_AROUND_CAMERA_MATCH_PARAM        (-1)
/*
针对环视相机而言，(粗选)最近最多未匹配上的次数
*/
#define MAX_DISMATCHED_TRACKING_TIMES_FOR_AROUND_CAMERA_MATCH_PARAM     (-1)
/*
针对环视相机而言，(粗选)匹配的obj最大允许的纵向速度差
*/
#define MAX_VX_DIFF_FOR_MATCH_AROUND_CAMERA_PARAM                       (10.0F/3.6F)
/*
针对环视相机而言，(粗选)匹配的obj最大允许的横向速度差
*/
#define MAX_VY_DIFF_FOR_MATCH_AROUND_CAMERA_PARAM                       (30.0F/3.6F)
/*
针对环视相机而言，表示是否有精选条件
*/
#define IS_VALID_CONDITION_FOR_MATCH_AROUND_CAMERA_PARAM                (true)
/*
针对环视相机而言，（精选）匹配的obj最大允许的纵向速度差
*/
#define MAX_VX_DIFF_OF_CONDITION_FOR_MATCH_AROUND_CAMERA_PARAM          (30.0F/3.6F)
/*
针对环视相机而言，（精选）匹配的obj最大允许的横向速度差
*/
#define MAX_VY_DIFF_OF_CONDITION_FOR_MATCH_AROUND_CAMERA_PARAM          (10.0F/3.6F)
/*
针对环视相机而言，运行根据当前状态估计位置
*/
#define USING_EST_POS_FOR_MATCH_AROUND_CAMERA_PARAM                     (false)
/*
针对环视相机而言，表示是否允许多个obj id进行匹配
*/
#define ENABLE_ADD_MUL_OBJ_ID_FOR_MATCH_AROUND_CAMERA_PARAM             (false)
/*
针对环视相机而言，表示是否允许次优匹配
*/
#define ENABLE_SUB_OPTIMAL_MATCH_FOR_MATCH_AROUND_CAMERA_PARAM          (true)
/*
针对环视相机而言，表示若根据位置未匹配上，是否根据ID重新匹配
*/
#define MATCH_AGAIN_BY_ID_IF_UNMATCHED_FOR_MATCH_AROUND_CAMERA_PARAM    (true)

/**
 * @brief 匹配阶段相关参数，针对前向雷达而言
 * 
 */
/*
针对前向雷达而言，最低已经跟踪上的次数。
*/
#define MIN_MATCHED_TRACKING_TIMES_FOR_FRONT_RADAR_MATCH_PARAM        (-1)
/*
针对前向雷达而言，(粗选)最近最多未匹配上的次数
*/
#define MAX_DISMATCHED_TRACKING_TIMES_FOR_FRONT_RADAR_MATCH_PARAM     (-1)
/*
针对前向雷达而言，(粗选)匹配的obj最大允许的纵向速度差
*/
#define MAX_VX_DIFF_FOR_MATCH_FRONT_RADAR_PARAM                       (5.0F/3.6F)
/*
针对前向雷达而言，(粗选)匹配的obj最大允许的横向速度差
*/
#define MAX_VY_DIFF_FOR_MATCH_FRONT_RADAR_PARAM                       (5.0F/3.6F)
/*
针对前向雷达而言，表示是否有精选条件
*/
#define IS_VALID_CONDITION_FOR_MATCH_FRONT_RADAR_PARAM                (true)
/*
针对前向雷达而言，（精选）匹配的obj最大允许的纵向速度差
*/
#define MAX_VX_DIFF_OF_CONDITION_FOR_MATCH_FRONT_RADAR_PARAM          (10.0F/3.6F)
/*
针对前向雷达而言，（精选）匹配的obj最大允许的横向速度差
*/
#define MAX_VY_DIFF_OF_CONDITION_FOR_MATCH_FRONT_RADAR_PARAM          (10.0F/3.6F)
/*
针对前向雷达而言，运行根据当前状态估计位置
*/
#define USING_EST_POS_FOR_MATCH_FRONT_RADAR_PARAM                     (true)
/*
针对前向雷达而言，表示是否允许多个obj id进行匹配
*/
#define ENABLE_ADD_MUL_OBJ_ID_FOR_MATCH_FRONT_RADAR_PARAM             (true)
/*
针对前向雷达而言，表示是否允许次优匹配
*/
#define ENABLE_SUB_OPTIMAL_MATCH_FOR_MATCH_FRONT_RADAR_PARAM          (true)
/*
针对前向雷达而言，表示若根据位置未匹配上，是否根据ID重新匹配
*/
#define MATCH_AGAIN_BY_ID_IF_UNMATCHED_FOR_MATCH_FRONT_RADAR_PARAM    (true)


/**
 * @brief 匹配阶段相关参数，针对侧向毫米波雷达而言
 * 
 */
/*
针对侧向毫米波雷达而言，最低已经跟踪上的次数。
*/
#define MIN_MATCHED_TRACKING_TIMES_FOR_AROUND_RADAR_MATCH_PARAM        (-1)
/*
针对侧向毫米波雷达而言，(粗选)最近最多未匹配上的次数
*/
#define MAX_DISMATCHED_TRACKING_TIMES_FOR_AROUND_RADAR_MATCH_PARAM     (-1)
/*
针对侧向毫米波雷达而言，(粗选)匹配的obj最大允许的纵向速度差
*/
#define MAX_VX_DIFF_FOR_MATCH_AROUND_RADAR_PARAM                       (30.0F/3.6F)
/*
针对侧向毫米波雷达而言，(粗选)匹配的obj最大允许的横向速度差
*/
#define MAX_VY_DIFF_FOR_MATCH_AROUND_RADAR_PARAM                       (20.0F/3.6F)
/*
针对侧向毫米波雷达而言，表示是否有精选条件
*/
#define IS_VALID_CONDITION_FOR_MATCH_AROUND_RADAR_PARAM                (true)
/*
针对侧向毫米波雷达而言，（精选）匹配的obj最大允许的纵向速度差
*/
#define MAX_VX_DIFF_OF_CONDITION_FOR_MATCH_AROUND_RADAR_PARAM          (20.0F/3.6F)
/*
针对侧向毫米波雷达而言，（精选）匹配的obj最大允许的横向速度差
*/
#define MAX_VY_DIFF_OF_CONDITION_FOR_MATCH_AROUND_RADAR_PARAM          (15.0F/3.6F)
/*
针对侧向毫米波雷达而言，运行根据当前状态估计位置
*/
#define USING_EST_POS_FOR_MATCH_AROUND_RADAR_PARAM                     (true)
/*
针对侧向毫米波雷达而言，表示是否允许多个obj id进行匹配
*/
#define ENABLE_ADD_MUL_OBJ_ID_FOR_MATCH_AROUND_RADAR_PARAM             (true)
/*
针对侧向毫米波雷达而言，表示是否允许次优匹配
*/
#define ENABLE_SUB_OPTIMAL_MATCH_FOR_MATCH_AROUND_RADAR_PARAM          (true)
/*
针对侧向毫米波雷达而言，表示若根据位置未匹配上，是否根据ID重新匹配
*/
#define MATCH_AGAIN_BY_ID_IF_UNMATCHED_FOR_MATCH_AROUND_RADAR_PARAM    (true)



/**
 * @brief 其他匹配参数
 * 
 */
/*
针对相机类传感器，计算搜索半径时用到的界限速度
*/
#define LIMIT_OF_OBJ_VELOCITY_FOR_CAMERA_MATCH_SEARCH_RADIUS   (15.0F/3.6F)
/*
针对毫米波雷达类传感器，计算搜索半径时用到的界限速度
*/
#define LIMIT_OF_OBJ_VELOCITY_FOR_RADAR_MATCH_SEARCH_RADIUS   (15.0F/3.6F)


/*
针对相机匹配的搜索半径计算公式的系数
*/
#define COEFFICIENT0_OF_SEARCH_RADAIUS_FORMULA_0_FOR_CAMERA   (5.0F) //公式0的系数0
#define COEFFICIENT1_OF_SEARCH_RADAIUS_FORMULA_0_FOR_CAMERA   (0.2F) //公式0的系数1
#define COEFFICIENT2_OF_SEARCH_RADAIUS_FORMULA_0_FOR_CAMERA   (0.2F) //公式0的系数2

#define COEFFICIENT0_OF_SEARCH_RADAIUS_FORMULA_1_FOR_CAMERA   (5.0F) //公式1的系数0
#define COEFFICIENT1_OF_SEARCH_RADAIUS_FORMULA_1_FOR_CAMERA   (0.2F) //公式1的系数1
#define COEFFICIENT2_OF_SEARCH_RADAIUS_FORMULA_1_FOR_CAMERA   (0.2F) //公式1的系数2

/*
针对相机匹配的 max_x_diff_ 计算公式的系数针对相机匹配的 max_x_diff_ 计算公式的系数
*/
#define COEFFICIENT0_OF_MAX_X_DIFF_FORMULA_FOR_CAMERA   (5.0F) //系数0
#define COEFFICIENT1_OF_MAX_X_DIFF_FORMULA_FOR_CAMERA   (0.1F) //系数1
#define COEFFICIENT2_OF_MAX_X_DIFF_FORMULA_FOR_CAMERA   (0.2F) //系数2

/*
针对相机匹配的 max_y_diff_ 计算公式的系数
*/
#define COEFFICIENT0_OF_MAX_Y_DIFF_FORMULA_FOR_CAMERA   (0.5F)  //系数0
#define COEFFICIENT1_OF_MAX_Y_DIFF_FORMULA_FOR_CAMERA   (0.05F) //系数1
#define COEFFICIENT2_OF_MAX_Y_DIFF_FORMULA_FOR_CAMERA   (0.1F)  //系数2


/*
与函数相关的AddPartnerRelationship计算匹配代价的公式的系数
*/
#define COEFFICIENT0_OF_MATCH_COST_FORMULA_FOR_RALATIONSHIP    (5.0F)  //系数0
#define COEFFICIENT1_OF_MATCH_COST_FORMULA_FOR_RALATIONSHIP    (-0.1F)  //系数1
#define COEFFICIENT2_OF_MATCH_COST_FORMULA_FOR_RALATIONSHIP    (-10)  //系数2
#define COEFFICIENT3_OF_MATCH_COST_FORMULA_FOR_RALATIONSHIP    (0)  //系数3
#define COEFFICIENT4_OF_MATCH_COST_FORMULA_FOR_RALATIONSHIP    (1.0F * 3.6F)  //系数4
#define COEFFICIENT5_OF_MATCH_COST_FORMULA_FOR_RALATIONSHIP    (0)  //系数5
#define COEFFICIENT6_OF_MATCH_COST_FORMULA_FOR_RALATIONSHIP    (-5)  //系数6
#define COEFFICIENT7_OF_MATCH_COST_FORMULA_FOR_RALATIONSHIP    (0)  //系数7
#define COEFFICIENT8_OF_MATCH_COST_FORMULA_FOR_RALATIONSHIP    (2)  //系数8
#define COEFFICIENT9_OF_MATCH_COST_FORMULA_FOR_RALATIONSHIP    (-5)  //系数9
#define COEFFICIENT10_OF_MATCH_COST_FORMULA_FOR_RALATIONSHIP    (0)  //系数10
#define COEFFICIENT11_OF_MATCH_COST_FORMULA_FOR_RALATIONSHIP   (0)  //系数11
#define COEFFICIENT12_OF_MATCH_COST_FORMULA_FOR_RALATIONSHIP   (-5)  //系数12
#define COEFFICIENT13_OF_MATCH_COST_FORMULA_FOR_RALATIONSHIP   (0)  //系数13


/**
 * 针对函数UpdateUnmatchedObj相关的匹配参数
 */
#define VECTOR_DATA_NUM_OF_MATCHED_OBJS_FOR_UPDATE_UNMATICHED_OBJ   64   // matched objs vector容量大小
#define VECTOR_DATA_NUM_OF_UNMATCHED_OBJS_FOR_UPDATE_UNMATICHED_OBJ   64   //unmatched objs vector容量大小



/**
 * @brief 针对MatchObjAgainByIdWhenUnmatched 函数中的匹配参数的相关宏
 *
 */
/**
 * 针对函数针对MatchObjAgainByIdWhenUnmatched 中 max_x_diff 计算公式的系数 
 */
#define  COEFFICIENT0_OF_MAX_X_DIFF_FORMULA_WHEN_UNMATCHED   (1.0F)   //系数0 
#define  COEFFICIENT1_OF_MAX_X_DIFF_FORMULA_WHEN_UNMATCHED   (0.05F)  //系数1 
#define  COEFFICIENT2_OF_MAX_X_DIFF_FORMULA_WHEN_UNMATCHED   (0.2F)   //系数2 
/**
 * 针对函数针对MatchObjAgainByIdWhenUnmatched 中 max_y_diff 计算公式的系数 
 */
#define  COEFFICIENT0_OF_MAX_Y_DIFF_FORMULA_WHEN_UNMATCHED   (0.2F)   //系数0 
#define  COEFFICIENT1_OF_MAX_Y_DIFF_FORMULA_WHEN_UNMATCHED   (0.01F)  //系数1 
#define  COEFFICIENT2_OF_MAX_Y_DIFF_FORMULA_WHEN_UNMATCHED   (0.1F)   //系数2 
/**
 * 针对函数针对MatchObjAgainByIdWhenUnmatched 中 max_vx_diff 参数
 */
#define  VALUE_OF_MAX_VX_DIFF_FORMULA_WHEN_UNMATCHED   (10.0F/3.6F)   
/**
 * 针对函数针对MatchObjAgainByIdWhenUnmatched 中 max_vy_diff 参数
 */
#define  VALUE_OF_MAX_VY_DIFF_FORMULA_WHEN_UNMATCHED   (10.0F/3.6F)  

/**
 * @brief 障碍物存活时间相关参数
 */
/*
UPPER_LIMIT_DURATION_DIFF_WITH_MAX_AGE_FOR_SENSOR_REFS 表示与融合结果（障碍物）相关联的传感器结构体内部一个差值，
当其中一个传感器没有匹配上融合值时，该差值就代表了该传感器数据能够保持与融合结果关联关系的最大时间值，该值的单位为融合周期100ms。
现在值设置为10，表示关系保持 1 秒（s）
*/
#define UPPER_LIMIT_DURATION_DIFF_WITH_MAX_AGE_FOR_SENSOR_REFS (10) //单位：100ms



/**
 * @brief 当进入前视一体机的盲区时，由于感知数据的暂留，保存的瞬时速度有可能不准，需要毫米波雷达横向距离进行加权平均。
 * 而且当纵向距离较近，相机横向速度和距离值有可能不准，导致最后横向距离计算错误。
 * 这里将界限距离设置为10 
 * 
 * 
 */
#define   MIN_X_VALUE_LIMIT_FOR_FUSED_Y_VY_VALUE_CALC         (15) // 单位：m















} // sensors fusion
} // perception
} // phoenix

#endif


