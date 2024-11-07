/**
 * @file adasisv2_horizon_reconstructor.h
 * @author wangwh
 * @brief ADASIS v2 Horizon Reconstructor for PCC and PACC
 * @date 2023-06-14
 * 
 */

#include "pcc_map/adasisv2_horizon_reconstructor.h"
#include "utils/log.h"

#if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
#define ENABLE_ADASISV2_HR_TRACE (0)
#define ENABLE_ADASISV2_HR_TRACE_DETAIL (0)
#else
#define ENABLE_ADASISV2_HR_TRACE (1)
#define ENABLE_ADASISV2_HR_TRACE_DETAIL (0)
#endif


namespace phoenix {
namespace adasisv2 {

HorizonReconstructor::HorizonReconstructor() {
  Clear();
}

void HorizonReconstructor::Clear() {
  horizon_.Clear();
  profiles_.Clear();
}

HorizonReconstructor::~HorizonReconstructor() {
  Clear();
}

void HorizonReconstructor::Construct(const PositionMessage &pos, const ProfileShortMessage &profile) {
  // add new profile
  AddProfile(profile);

  //TODO: 处理update flag

  if (AddPosition(pos)) { // 自车位置更新，同步更新profile
    // update all profile offset
    common::RingBuffer<Profile, HorizonMaxProfilePointNum>::iterator it = profiles_.begin();
    common::RingBuffer<Profile, HorizonMaxProfilePointNum>::iterator it_end = profiles_.end();
    for (; it != it_end; ++it) {
      it->offset_from_ego = it->offset_from_start - horizon_.position_offset_from_start;
    }

    // 移除自车后方的profile point，RingBuffer没有提供删除特定元素的接口，但基于offset数据是按递增方式存放的假设，通过popfront也可以达到同样效果
    for (Int32_t i = 0; i < profiles_.Size(); ++i) {
      if (profiles_[i].offset_from_ego < -BackwardJumpDistance) {
        profiles_.PopFront(); //TODO: PopFront没有完全删除指定条件的元素
      }
    }
  } else { // 自车位置未更新或重复，复用上一周期的Horizon
    //TODO: 航迹推算
  }

  /**
   * 计算输出数据
   */
  // 对Profiles数据进行分类保存，根据需要分别重采样
  horizon_.slope_list.Clear();
  horizon_.curvature_list.Clear();
  
  // 将自车位置的slope值作为第一个元素：当前周期的坡度值
  Profile position_slope;
  position_slope.type = PROFILE_SHORT_TYPE_SLOPE_LINEAR;
  position_slope.offset_from_start = horizon_.position_offset_from_start;
  position_slope.cyclic_offset = horizon_.position.offset;
  position_slope.cyclic_segment_num = horizon_.position_offset_cyclic_num;
  position_slope.offset_from_ego = 0;

  if (std::abs(horizon_.position.slope) > 10.0F) {
    position_slope.value = 0.0F;
  } else {
    position_slope.value = horizon_.position.slope;
  }
  
  horizon_.slope_list.PushBack(position_slope);

  common::RingBuffer<Profile, HorizonMaxProfilePointNum>::iterator it = profiles_.begin();
  common::RingBuffer<Profile, HorizonMaxProfilePointNum>::iterator it_end = profiles_.end();
  for (; it != it_end; ++it) {
    if (PROFILE_SHORT_TYPE_SLOPE_LINEAR == it->type) {
      if (it->offset_from_ego >= 0) {
        horizon_.slope_list.PushBack(*it);
      }
    } else if (PROFILE_SHORT_TYPE_CURVATURE == it->type) {
      horizon_.curvature_list.PushBack(*it);
    }
  }

#if ENABLE_ADASISV2_HR_TRACE_DETAIL
  Int32_t slope_profile_num = horizon_.slope_list.Size();
  for (Int32_t i = 0; i < slope_profile_num; ++i) {
    const adasisv2::Profile& profile = horizon_.slope_list[i];
    LOG_INFO(5) << "[ADASISV2HR][slope] cyclic_offset =  " << profile.cyclic_offset << ", path offset = " << profile.offset_from_start << ", ego offset = " << profile.offset_from_ego 
                << ", slope(%) = " << profile.value << ", ego pos = " << horizon_.position_offset_from_start;
  }
  
  Int32_t curvature_profile_num = horizon_.curvature_list.Size();
  for (Int32_t i = 0; i < curvature_profile_num; ++i) {
    const adasisv2::Profile& profile = horizon_.curvature_list[i];
    LOG_INFO(5) << "[ADASISV2HR][curvature] cyclic_offset = " << profile.cyclic_offset << ", path offset = " << profile.offset_from_start  << ", ego offset = " << profile.offset_from_ego 
                << ", curvatue(%) = " << profile.value << ", ego pos = " << horizon_.position_offset_from_start;
  }   
#endif

  //TODO: 对slope和curvature数据进行重采样

  if (horizon_.slope_list.Size() > 100) { // 10m/point
    // 检查坡度数据长度是否满足PCC要求
    Float32_t last_slope_distance = horizon_.slope_list.Back().offset_from_ego;
    if ((INVALID_TYPE_PATH_OFFSET != last_slope_distance) && (last_slope_distance >= PCCSlopeMinLength)) {
      horizon_.is_ready = true;
    } else {
      horizon_.is_ready = false;
    }
  } else {
    horizon_.is_ready = false;
  }
}

const Horizon* HorizonReconstructor::GetHorizon() const {
    return &horizon_;
}

bool HorizonReconstructor::AddPosition(const PositionMessage& pos) {
  // 时间戳


  if (INVALID_TYPE_NORMAL != pos.path_index) { // 有效的POSITION消息
    if (horizon_.position.path_index == pos.path_index) { // 自车在Main Path上
      if (horizon_.position.offset == pos.offset) { // 重复数据，忽略
        return false;
      } else { // 更新POSITION数据，需要判断是否处理offset wrap
        int16_t diff = pos.offset - horizon_.position.offset;
        if (diff > 0) { // 自车位置增加
          if (diff > PositionOffsetDiffThreshold) { // 出现长时间定位不更新
            Clear();
            horizon_.position_offset_from_start = pos.offset;
#if ENABLE_ADASISV2_HR_TRACE
            LOG_INFO(5) << "[ADASISV2HR] position lost too long, reset horzion, diff = " << diff << ", new offset = " << pos.offset << ", old offset = " << horizon_.position.offset;
#endif
          } else {
            horizon_.position_offset_from_start = pos.offset + horizon_.position_offset_cyclic_num * (MaxinumOffsetRange + 1);
          }
        } else { // 分段循环offset计算
          
          if (horizon_.position.offset > CyclicOffsetThreshold) { // 快接近分段
            horizon_.position_offset_cyclic_num++;
            horizon_.position_offset_from_start = pos.offset + horizon_.position_offset_cyclic_num * (MaxinumOffsetRange + 1);
#if ENABLE_ADASISV2_HR_TRACE
            LOG_INFO(5) << "[ADASISV2HR] position cyclic calc = " << horizon_.position_offset_cyclic_num << ", new offset = " << pos.offset << ", old offset = " << horizon_.position.offset;
#endif
          } else { // 可能是异常数据且比上一帧小，忽略
            return false;
          }
        }
      }
    } else { // 第一次收到POSITION或自车偏离Main Path,处于Stub Path，需要重新建图
      // 对于规范5.5的情况: 定位后跳，U-Turn，倒车; Av2HP建议重建一个path来应对，当path发生变化时，重新构建horizon
#if ENABLE_ADASISV2_HR_TRACE
      LOG_INFO(5) << "[ADASISV2HR] position path changed, reset horzion";
#endif
      Clear();
      horizon_.position_offset_from_start = pos.offset;
    }

    horizon_.position = pos;

#if ENABLE_ADASISV2_HR_TRACE
    LOG_INFO(5) << "[ADASISV2HR] Add new POSITON : " << "cyclic_offset = " << horizon_.position.offset << ", offset_from_start = " << horizon_.position_offset_from_start
                << " cyclic num = " << horizon_.position_offset_cyclic_num << ", path index = " << horizon_.position.path_index;
#endif

    return true;
  } else {
    return false;
  }
}

void HorizonReconstructor::AddProfile(const ProfileShortMessage &profile) {
  static const Float32_t MaxinumSlopeABS = 10.0;
  // 对Profile Short消息进行有效性检查
  if (INVALID_TYPE_NORMAL == profile.path_index) {
    return;
  }
  if (PROFILE_SHORT_TYPE_CURVATURE != profile.profile_type && PROFILE_SHORT_TYPE_SLOPE_LINEAR != profile.profile_type) {
    return;
  }

  if (INVALID_TYPE_PATH_OFFSET == profile.offset) {
    return;
  }

  if (profile.path_index != horizon_.position.path_index) { // Profile Short与Position不在一条路径上，可能出现分叉道路
    return;
  } else {
    if (PROFILE_SHORT_TYPE_SLOPE_LINEAR == profile.profile_type) {
      if (std::abs(profile.value0) > MaxinumSlopeABS) { // 忽略 异常 坡度数据
        return;
      }

      if (horizon_.profile_slope.offset == profile.offset) { // 重复 坡度 数据，忽略
        return;
      } else {
        // 更新ProfileShort数据，需要判断是否处理offset wrap
        if (profile.offset > horizon_.profile_slope.offset) { // profile 数据增长
          horizon_.profile_slope_offset_from_start = profile.offset + horizon_.profile_slope_offset_cyclic_num * (MaxinumOffsetRange + 1);
        } else { // 分段循环offset计算
          if (horizon_.profile_slope.offset > CyclicOffsetThreshold) { // 快接近分段
            horizon_.profile_slope_offset_cyclic_num++;
            horizon_.profile_slope_offset_from_start = profile.offset + horizon_.profile_slope_offset_cyclic_num * (MaxinumOffsetRange + 1);
#if ENABLE_ADASISV2_HR_TRACE
            LOG_INFO(5) << "[ADASISV2HR] slope cyclic calc = " << horizon_.profile_slope_offset_cyclic_num << ", new offset = " << profile.offset << ", old offset = " << horizon_.profile_slope.offset;
#endif
          } else { // 可能是异常数据且比上一帧小，忽略
            return;
          }
        }
        
        if (horizon_.profile_slope_offset_from_start < horizon_.position_offset_from_start) { // 出现profile比position小的情况
#if ENABLE_ADASISV2_HR_TRACE
          LOG_INFO(5) << "[ADASISV2HR] slope offset smaller than position, slope offset = " << horizon_.profile_slope_offset_from_start << ", horizon offset = " << horizon_.position_offset_from_start;
#endif

          horizon_.profile_slope_offset_cyclic_num++;
          horizon_.profile_slope_offset_from_start += MaxinumOffsetRange + 1;
        }
        // 有效数据
        horizon_.profile_slope = profile;

        Profile* profile_short = profiles_.AllocateOverride();
        profile_short->type = (ProfileShortType)profile.profile_type;
        profile_short->offset_from_start = horizon_.profile_slope_offset_from_start;
        profile_short->cyclic_offset = profile.offset;
        profile_short->cyclic_segment_num = horizon_.profile_slope_offset_cyclic_num;
        profile_short->offset_from_ego = profile_short->offset_from_start - horizon_.position_offset_from_start;
        profile_short->value = profile.value0;
#if ENABLE_ADASISV2_HR_TRACE
        LOG_INFO(5) << "[ADASISV2HR] Add new SLOPE : " << "cyclic_offset = " << profile_short->cyclic_offset << ", offset_from_start = " << profile_short->offset_from_start
                      << " cyclic num = " <<  profile_short->cyclic_segment_num << ", offset_from_ego = " << profile_short->offset_from_ego << ", value = " << profile_short->value;
#endif
      }
    }

    if (PROFILE_SHORT_TYPE_CURVATURE == profile.profile_type) {
      if (horizon_.profile_curvatue.offset == profile.offset) { // 重复 曲率 数据，忽略
        return;
      } else {
        // 更新ProfileShort数据，需要判断是否处理offset wrap
        if (profile.offset > horizon_.profile_curvatue.offset) { // profile 数据增长
          horizon_.profile_curvatue_offset_from_start = profile.offset + horizon_.profile_curvatue_offset_cyclic_num * (MaxinumOffsetRange + 1);
        } else { // 分段循环offset计算
          if (horizon_.profile_curvatue.offset > CyclicOffsetThreshold) { // 快接近分段
            horizon_.profile_curvatue_offset_cyclic_num++;
            horizon_.profile_curvatue_offset_from_start = profile.offset + horizon_.profile_curvatue_offset_cyclic_num * (MaxinumOffsetRange + 1);
          } else { // 可能是异常数据且比上一帧小，忽略
            return;
          }
        }
        
        if (horizon_.profile_curvatue_offset_from_start < horizon_.position_offset_from_start) { // 出现profile比position小的情况
#if ENABLE_ADASISV2_HR_TRACE
          LOG_INFO(5) << "[ADASISV2HR] curvature offset smaller than position, curvature offset = " << horizon_.profile_curvatue_offset_from_start << ", horizon offset = " << horizon_.position_offset_from_start;
#endif
          horizon_.profile_curvatue_offset_cyclic_num++;
          horizon_.profile_curvatue_offset_from_start += MaxinumOffsetRange + 1;
        }

        // 有效数据
        horizon_.profile_curvatue = profile;

        Profile* profile_short = profiles_.AllocateOverride();
        profile_short->type = (ProfileShortType)profile.profile_type;
        profile_short->offset_from_start = horizon_.profile_curvatue_offset_from_start;
        profile_short->cyclic_offset = profile.offset;
        profile_short->cyclic_segment_num = horizon_.profile_curvatue_offset_cyclic_num;
        profile_short->offset_from_ego = profile_short->offset_from_start - horizon_.position_offset_from_start;
        profile_short->value = profile.value0;
#if ENABLE_ADASISV2_HR_TRACE
        LOG_INFO(5) << "[ADASISV2HR] Add new Curvatue : " << "cyclic_offset = " << profile_short->cyclic_offset << ", offset_from_start = " << profile_short->offset_from_start
                      << " cyclic num = " <<  profile_short->cyclic_segment_num << ", offset_from_ego = " << profile_short->offset_from_ego << ", value = " << profile_short->value;
#endif
      }

    }

  }
}

}  // namespace map
}  // namespace phoenix
