/**
 * @file adasisv2_horizon_reconstructor.h
 * @author wangwh
 * @brief ADASIS v2 Horizon Reconstructor for PCC and PACC
 * @date 2023-06-14
 * 
 */

#ifndef PHOENIX_MAP_ADASISV2_HORIZON_RECONSTRUCTOR_H_
#define PHOENIX_MAP_ADASISV2_HORIZON_RECONSTRUCTOR_H_

#include "pcc_map/adasis_v2.h"
#include "container/ring_buffer.h"

namespace phoenix {
namespace adasisv2 {

/**
 * @brief ADASIS v2 Horizon Reconstructor according to POSTION, Profile message
 *        根据收到的POSITION消息更新Profile，转换到以自车为原点
 */
class HorizonReconstructor {
public:
  HorizonReconstructor();
  ~HorizonReconstructor();

  // 调用者需保证消息的有效性
  void Construct(const PositionMessage& pos, const ProfileShortMessage& profile);

  const Horizon* GetHorizon() const;

private:
  bool AddPosition(const PositionMessage& pos);
  void AddProfile(const ProfileShortMessage& profile);

  void Clear();

private:

  static const Int32_t HorizonMaxProfilePointNum = 1000;

  static const Int32_t PCCSlopeMinLength = 2000;

  /// 剔除自车后面的profile时保留一定距离的点 @see 规范5.5.1
  static const Int32_t BackwardJumpDistance = 0;

  static const Int32_t CyclicOffsetThreshold = 8000;

  static const Int32_t PositionOffsetDiffThreshold = 200;
  
  static const Int32_t MaxinumOffsetRange = 8190;

  /**
   * 数据结构的选择:
   * 一种数据结构存储所有profile数据 vs 每种profile数据单独存储
   * 所有数据需要同时减去同一个数值
   * 过期数据s需要被清除/标记
   * 提取相同profile类型的数据以供下游使用
  */
  common::RingBuffer<Profile, HorizonMaxProfilePointNum> profiles_;

private:
  /// 输出接口
  Horizon horizon_;
};

}  // namespace adasisv2
}  // namespace phoenix

#endif  // PHOENIX_MAP_ADASISV2_HORIZON_RECONSTRUCTOR_H_
