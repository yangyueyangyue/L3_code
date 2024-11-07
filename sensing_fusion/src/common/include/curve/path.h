/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       path.h
 * @brief      路径处理
 * @details    实现了部分路径处理方法（最近点查询、相交判断，采样等）
 *
 * @author     boc
 * @date       2020.05.14
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/04/18  <td>1.0      <td>boc       <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_COMMON_PATH_H_
#define PHOENIX_COMMON_PATH_H_

#include "utils/macros.h"
#include "utils/log.h"

#include "geometry/vec2d.h"
#include "geometry/aabboxkdtree2d.h"
#include "container/static_vector.h"

namespace phoenix {
namespace common {

/**
 * @struct PathPoint
 * @brief 路径点，提供了路径上点的二维位置、航向角、曲率、沿着路径的弧长、侧向距离。
 */
struct PathPoint {
  /// 二维位置（单位：米）
  Vec2d point;
  /// 航向角（单位：弧度）
  geo_var_t heading;
  /// 曲率（单位：1/米）
  geo_var_t curvature;
  /// 沿着路径的弧长（单位：米）
  geo_var_t s;
  /// 侧向距离（单位：米），左正右负
  geo_var_t l;

  /**
   * @brief 初始化函数。
   */
  void Clear() {
    point.Clear();
    heading = 0.0f;
    curvature = 0.0f;
    s = 0.0f;
    l = 0.0f;
  }

  /**
   * @brief 构造函数。
   */
  PathPoint() {
    heading = 0.0f;
    curvature = 0.0f;
    s = 0.0f;
    l = 0.0f;
  }
};

/**
 * @struct TrajectoryPoint
 * @brief 轨迹点，除了含点的空间信息外，还包含速度、加速度等时间信息。
 */
struct TrajectoryPoint {
  /// 点的空间信息，包含位置、航向角、曲率、弧长等。
  PathPoint path_point;
  /// 速度（单位：m/s）
  geo_var_t v;
  /// 加速度（单位：m/s^2）
  geo_var_t a;
  /// 横摆角速度（单位：rad/s）
  geo_var_t yaw_rate;
  /// 相对于曲线起点已行驶的相对时间（单位：s）
  geo_var_t relative_time;

  /**
   * @brief 初始化函数。
   */
  void Clear() {
    path_point.Clear();
    v = 0.0f;
    a = 0.0f;
    yaw_rate = 0.0f;
    relative_time = 0.0f;
  }

  /**
   * @brief 构造函数。
   */
  TrajectoryPoint() {
    v = 0.0f;
    a = 0.0f;
    yaw_rate = 0.0f;
    relative_time = 0.0f;
  }
};

/**
 * @class Path
 * @brief 路径处理，提供了根据用户输入的点列构建一条路径，计算每个路径
 *        点的航向角和曲率，曲率分析等功能。
 */
class Path {
public:
  /// 弯道曲率评判阈值，曲率大于此阈值的路段才认为是弯道
  static const geo_var_t kCriticalCurvature;
  /**
   * 枚举类型
   */
  enum {
    /// 直行路段
    TYPE_STRAIGHT = 0,
    /// 左转弯路段
    TYPE_CURVING_LEFT,
    /// 右转弯路段
    TYPE_CURVING_RIGHT
  };
  /**
   * @struct CurveSegment
   * @brief 曲线段信息。曲率分析时需要用到此结构体。
   */
  struct CurveSegment {
    /// 曲线段的曲率类型，定义STRAIGHT = 0, LEFT_TURN = 1, RIGHT_TURN = 2。
    /// 定义曲率绝对值 < 0.003为直线，定义正曲率为左，定义负曲率为右。
    Int32_t type;
    /// 此曲线段在路径中的起始位置
    geo_var_t start_s;
    /// 此曲线段的长度
    geo_var_t length;
    /// 此曲线段的平均曲率
    geo_var_t ave_curvature;
    /// 此曲线段的最大曲率
    geo_var_t max_curvature;
  };

  /// 路径上点的最大个数
  static const Int32_t kMaxPathPointNum = 500;
  /// 曲线段的最大个数。直线段也认为是曲线段。
  static const Int32_t kMaxCurveSegmentNum = 20;

public:
  /**
   * @brief 构造函数，设置构造参数。
   * @param[in] min_valid_seg_dist_in      构造路径时，允许的两点之间的\n
   *                                       最小距离，单位：米。
   * @param[in] max_valid_delta_heading_in 构造路径时，允许的前后两个线段\n
   *                                       之间的最大角度变化，单位：弧度。
   */
  Path(geo_var_t min_valid_seg_dist_in = 0.3f,
       geo_var_t max_valid_delta_heading_in =
      phoenix::common::com_deg2rad(100.0F));

  /**
   * @brief 析构函数。
   */
  virtual ~Path();

  /**
   * @brief 清理函数，对内部成员变量分配的存储空间进行清理。
   */
  void Clear() {
    points_.Clear();
    headings_.Clear();
    curvatures_.Clear();
    accumulated_s_.Clear();
    lane_segment_kdtree_.Clear();
  }

  /**
   * @brief 根据二维点列构建一条路径。
   *        点列中仅包含点的坐标。
   * @details 使用用户输入的点集构建路径，删除距离过小的点，删除相邻
   *          线段航向变化过大的点，也会计算每个点的航向和曲率以及路
   *          径长，保存在成员变量中。
   * @param[in] points_in 二维点列。
   * @return true-成功；false-失败。
   */
  virtual bool Construct(
      const StaticVector<Vec2d, kMaxPathPointNum>& points_in);

  /**
   * @brief 根据二维路径点列构建一条路径。
   *        路径点中包含点的坐标、航向角、曲率。
   * @details 使用用户输入的点集构建路径，删除距离过小的点，删除相邻
   *          线段航向变化过大的点，也会计算每个点的航向和曲率以及路
   *          径长，保存在成员变量中。
   * @param[in] path_points 二维路径点列。
   * @return true-成功；false-失败。
   */
  virtual bool Construct(
      const StaticVector<PathPoint, kMaxPathPointNum>& path_points);

  /**
   * @brief 获取路径中点列的位置信息。
   * @return 路径中点列的位置信息。
   */
  const StaticVector<Vec2d, kMaxPathPointNum>& points() const {
    return (points_);
  }

  /**
   * @brief 获取路径中点列的航向角信息。
   * @return 路径中点列的航向角信息。
   */
  const StaticVector<geo_var_t, kMaxPathPointNum>& headings() const {
    return (headings_);
  }

  /**
   * @brief 获取路径中点列的曲率信息。
   * @return 路径中点列的曲率信息。
   */
  const StaticVector<geo_var_t, kMaxPathPointNum>& curvatures() const {
    return (curvatures_);
  }

  /**
   * @brief 获取路径中点列的弧长信息。
   * @return 路径中点列的弧长信息。
   */
  const StaticVector<geo_var_t, kMaxPathPointNum>& accumulated_s() const {
    return (accumulated_s_);
  }

  /**
   * @brief 获取路径的总长度。
   * @return 路径的总长度。
   */
  geo_var_t total_length() const { return total_length_; }

  /**
   * @brief 查找路径中与输入点最近的点。
   * @param[in]  p                   待查询的点
   * @param[out] path_point_nearest  路径中与输入点最近的点
   * @return     true 成功；false 失败。
   */
  bool FindNearest(const Vec2d& p, PathPoint * const path_point_nearest) const;

  /**
   * @brief 获取点在路径的某一条线段上的最近点。
   * @param[in]  p                   待查询的点
   * @param[in]  index_low           线段起点在路径点列中的索引
   * @param[out] path_point_nearest  点在路径的某一条线段上的最近点
   * @return     true 成功；false 失败。
   */
  bool GetSmoothNearestPointOnSegment(
      const Vec2d& p, const Int32_t index_low,
      PathPoint * const path_point_nearest) const;

  /**
   * @brief 查找输入点在路径上的投影点。
   * @param[in]  p           待查询的点
   * @param[out] path_point_projection  输入点在路径上的投影点
   * @return     true 成功；false 失败。
   * @attention  1、path_point_projection中的横向距离l存储的是输入点的横向距离；\n
   *             2、如果指定的点在曲线上的投影在曲线的起点之前，那么投影点为曲线上的
   *                第一个点及第二个点所构成的直线上的投影点。为了增强程序的鲁棒性，
   *                当这两个点之间的距离过近时，可以沿着曲线向前搜索满足大于某个距离的点
   *                作为第二个点；\n
   *             3、如果指定的点在曲线上的投影在曲线的终点之后，那么投影点为曲线上的
   *                最后一个点及倒数第二个点所构成的直线上的投影点。为了增强程序的鲁棒性，
   *                当这两个点之间的距离过近时，可以沿着曲线向后搜索满足大于某个距离的点
   *                作为倒数第二个点。
   */
  bool FindProjection(const Vec2d& p,
                      PathPoint * const path_point_projection) const;


  /**
   * @brief 查找路径中与输入点最近的点,返回找到点的索引
   * @param[in]  p                   待查询的点
   * @param[out] index               路径中与输入点最近的点的索引
   * @param[out] path_point_nearest  路径中与输入点最近的点
   * @return     true 成功；false 失败。
   */
  bool FindProjection(const Vec2d& p, Int32_t* index,
                   PathPoint * const path_point_nearest) const;

  /**
   * @brief 获取点在路径的某一条线段所在直线上的投影点。
   * @param[in]  p           待查询的点
   * @param[in]  index_low   线段起点在路径点列中的索引
   * @param[out] path_point_out  点在路径的某一条线段所在直线上的投影点
   * @return     true 成功；false 失败。
   */
  bool GetSmoothPojectivePointOnSegment(
      const Vec2d& p, const Int32_t index_low,
      PathPoint * const path_point_out) const;

  /**
   * @brief 查找路径上与路径长对应的点。
   * @param[in] s 路径长
   * @param[out] path_point_out 路径上与路径长对应的点
   * @return true 成功；false 失败。
   * @attention 1、如果路径长小于0，输出的点为路径的第一个点；\n
   *            2、如果路径长大于路径的总长度，输出的点为路径的最后一个点；\n
   *            3、path_point_out中的横向距离l为0。
   */
  bool FindSmoothPoint(geo_var_t s, PathPoint * const path_point_out) const;

  /**
   * @brief 查找路径上与路径长对应的点。
   * @param[in]  s              路径长
   * @param[out] path_point_out 路径上与路径长对应的点
   * @param[out] low_index      路径上与路径长对应的点所在线段起点的索引
   * @return true 成功；false 失败。
   * @attention 1、如果路径长小于0，输出的点为路径的第一个点；
   *            2、如果路径长大于路径的总长度，输出的点为路径的最后一个点；
   *            3、path_point中的横向距离l为0。
   */
  bool FindSmoothPoint(
      geo_var_t s, Int32_t * const low_index,
      PathPoint * const path_point_out) const;

  /**
   * @brief 将与路径对应的SL坐标转换为XY坐标
   * @param[in] s 路径长
   * @param[in] l 与路径之间的偏离值，左正右负
   * @return XY坐标形式的点
   * @note 如果s在路径的范围外，将使用路径的端点进行计算
   */
  Vec2d SLToXY(geo_var_t s, geo_var_t l) const;

  /**
   * @brief 获取路径的所有点列。
   * @param[in&out] sample_points 路径的所有点列
   * @note 当sample_points非空时，路径的点列会添加到sample_points最后
   *       一个点的后面。
   */
  void GetSamplePoints(
      StaticVector<PathPoint, kMaxPathPointNum> * const sample_points) const;

  /**
   * @brief 获取路径上指定段的点列。
   * @param[in] start_s 路径上指定段的起点的弧长
   * @param[in] end_s   路径上指定段的终点的弧长
   * @param[in&out] sample_points 路径上指定段的点列
   * @return true-获取成功；false-获取失败。
   * @note 当sample_points非空时，路径上指定段的点列会添加到
   *       sample_points最后一个点的后面。
   */
  bool GetSamplePoints(
      geo_var_t start_s, geo_var_t end_s,
      StaticVector<PathPoint, kMaxPathPointNum> * const sample_points) const;

  /**
   * @brief 获取路径的所有点横向偏移后的点列。
   * @param[in] lat_offset 与路径之间的偏离值
   * @param[out] sample_points 横向偏移后的点列
   */
  void CalcSamplePointsByLatOffsetting(
      geo_var_t lat_offset,
      StaticVector<Vec2d, kMaxPathPointNum>* const sample_points) const;

  /**
   * @brief 将路径向前等间距采样。
   * @param[in] start_s 起始路径长
   * @param[in] sample_num 需要采样的数量
   * @param[in] seg_len 采样间距
   * @param[out] sample_points 采样后的点
   * @return true 成功；false 失败。
   * @note 1. 将路径向前等间距采样。间距指的是路径长的间距，不是点之间的距离；\n
   *       2. 采样成功时，采样后的点的个数为sample_num+1。
   */
  bool UniformlySamplePathForward(
      geo_var_t start_s, Int32_t sample_num, geo_var_t seg_len,
      StaticVector<PathPoint, kMaxPathPointNum>* const sample_points) const;
  /**
   * @brief 将路径向前等间距采样。
   * @param[in] start_s 起始路径长
   * @param[in] sample_num 需要采样的数量
   * @param[in] seg_len 采样间距
   * @param[in] sample_l 与路径之间的偏离值
   * @param[out] sample_points 采样后的点
   * @return true 成功；false 失败。
   * @note 1. 将路径向前等间距采样，并与路径偏离一定距离。
   *       间距指的是路径长的间距，不是点之间的距离；\n
   *       2. 采样成功时，采样后的点的个数为sample_num+1。
   */
  bool UniformlySamplePathForward(
      geo_var_t start_s, Int32_t sample_num, geo_var_t seg_len,
      geo_var_t sample_l,
      StaticVector<PathPoint, kMaxPathPointNum>* const sample_points) const;
  /**
   * @brief 将路径向后等间距采样。
   * @param[in] start_s 起始路径长
   * @param[in] sample_num 需要采样的数量
   * @param[in] seg_len 采样间距
   * @param[out] sample_points 采样后的点
   * @return true 成功；false 失败。
   * @note 1. 将路径向后等间距采样。间距指的是路径长的间距，不是点之间的距离。\n
   *       2. 采样成功时，采样后的点的个数为sample_num+1。
   */
  bool UniformlySamplePathBackward(
      geo_var_t start_s, Int32_t sample_num, geo_var_t seg_len,
      StaticVector<PathPoint, kMaxPathPointNum> * const sample_points) const;
  /**
   * @brief 将路径向后等间距采样。
   * @param[in] start_s 起始路径长
   * @param[in] sample_num 需要采样的数量
   * @param[in] seg_len 采样间距
   * @param[in] sample_l 与路径之间的偏离值
   * @param[out] sample_points 采样后的点
   * @return true 成功；false 失败。
   * @note 1. 将路径向后等间距采样，并与路径偏离一定距离。
   *          间距指的是路径长的间距，不是点之间的距离。\n
   *       2. 采样成功时，采样后的点的个数为sample_num+1。
   */
  bool UniformlySamplePathBackward(
      geo_var_t start_s, Int32_t sample_num, geo_var_t seg_len,
      geo_var_t sample_l,
      StaticVector<PathPoint, kMaxPathPointNum> * const sample_points) const;
  /**
   * @brief 分析路径曲率。
   * @param[out] curve_segments 路径的曲率信息
   * @return true 成功；false 失败。
   * @note 分析路径曲率，将路径中所有直道、弯道信息提取出来。
   */
  bool AnalyzeCurvature(
      StaticVector<CurveSegment, kMaxCurveSegmentNum> * const
      curve_segments) const;

  /**
   * @brief 判断两条轨迹之间是否相交，如果相交则返回交点
   * @param[In] start_point 停车线的端点
   * @param[In] end_point 停车线的端点
   * @param[out] cross_point 轨迹之间的交点
   * @return true 相交 false 不相交
   * @note
   */
  bool IsIntersect(
      const common::Vec2d& start_point,
      const common::Vec2d& end_point,
      PathPoint* cross_point) const;

protected:
  /**
   * @brief 构造k-d树。
   * @note 构造路径点相邻点构成的线段的k-d树，用来加快最近点的检索速度。
   */
  void CreateKDTree();

private:
  // 弧长比较函数类
  class FuncCmpArcLen {
  public:
    inline bool operator ()(geo_var_t a, geo_var_t b) const {
      if (a > b) {
        return true;
      }
      return false;
    }
  };

private:
  /*
   * @brief 平滑曲率。
   * @param[in] index 路径点的索引
   * @param[in] left_range 向路径起点方向搜索的长度
   * @param[in] right_range 向路径终点方向搜索的长度
   * @return 平滑后的曲率。
   * @note 平滑路径上的点的曲率。
   */
  geo_var_t SmoothCurvature(Int32_t index,
      geo_var_t left_range = 3.0F, geo_var_t right_range = 3.0F);

  geo_var_t SmoothCurvature2(Int32_t index,
      geo_var_t left_range = 3.0F, geo_var_t right_range = 3.0F);
  /*
   * @brief 增添曲线段信息。
   * @param[in] start_index 曲线段起点的索引
   * @param[in] index_max_curvature 曲线段上曲率最大的点的索引
   * @param[in] sample_index 曲线段终点的索引
   * @param[in] type 曲线段的类型
   * @param[out] curve_segments 所有的曲线段信息，为一个可变数组。
   */
  void AddCurveSegment(
      Int32_t start_index, Int32_t index_max_curvature,
      Int32_t sample_index, Int32_t type,
      StaticVector<CurveSegment,
      kMaxCurveSegmentNum> * const curve_segments) const;
  /*
   * @brief 根据三角形的三条边的边长获取三角形的外接圆的半径。
   * @param[in] a 三角形的第一条边的边长
   * @param[in] b 三角形的第二条边的边长
   * @param[in] c 三角形的第三条边的边长
   * @param[out] r 三角形的外接圆的半径
   * @return 1               计算成功，三条边能够构成一个面积大于0的三角形；
   *         2               计算成功，三条边能够构成一个面积等于0的三角形；
   *         -1              计算失败，三边不能构成一个三角形。
   * @attention 1. 三点共线也会返回true；
   *            2. 可用于计算曲线段的曲率，曲率=1/转弯半径。
   */
  static Int32_t ObtainCircumcircleRadius(geo_var_t a, geo_var_t b, 
                                          geo_var_t c, geo_var_t* const r);
  /*
   * @brief 根据三角形的三个点的坐标获取三角形的外接圆的半径。
   * @param[in] a 三角形的第一个点的坐标
   * @param[in] b 三角形的第二个点的坐标
   * @param[in] c 三角形的第三个点的坐标
   * @param[out] r 三角形的外接圆的半径
   * @return 1               计算成功，三个点能够构成一个面积大于0的三角形；
   *         2               计算成功，三个点能够构成一个面积等于0的三角形；
   *         -1              计算失败，三个点不能构成一个三角形。
   * @attention 1. 三点共线也会返回true；
   *            2. 可用于计算曲线段的曲率，曲率=1/转弯半径。
   */
  static Int32_t ObtainCircumcircleRadius(const Vec2d& a, const Vec2d& b,
                                          const Vec2d& c, geo_var_t* const r);


private:
  // 两个路径点之间距离的最小值
  geo_var_t min_valid_seg_dist_;
  // 两个路径点之间航向角的最大偏差
  geo_var_t max_valid_delta_heading_;
  // 所有路径点的位置（单位：米）
  StaticVector<Vec2d, kMaxPathPointNum> points_;
  // 所有路径点的航向角（单位：弧度）
  StaticVector<geo_var_t, kMaxPathPointNum> headings_;
  // 所有路径点的曲率（单位：1/米）
  StaticVector<geo_var_t, kMaxPathPointNum> curvatures_;
  // 所有路径点的路径长，第1个点(index=0时)的路径长为0。（单位：米）
  StaticVector<geo_var_t, kMaxPathPointNum> accumulated_s_;
  // 路径总长度（单位：米）
  geo_var_t total_length_;
  // 用于加快最近路径点查询等功能的k-d树
  AABBoxKDTree2d<Int32_t, kMaxPathPointNum, 16> lane_segment_kdtree_;
};


/*
 * @struct PathSearch
 * @param PathAdapter 路径点查询信息的适配类
 * @brief 路径查询类，提供了路径点信息采样功能。
 */
template <typename PathAdapter>
class PathSearch {
public:
  class FuncCmpArcLen {
  public:
    inline bool operator ()(geo_var_t a, geo_var_t b) const {
      if (a > b) {
        return true;
      }
      return false;
    }
  };

  /*
   * @brief 构造函数。需要传入待查询信息，和信息数目。
   * @param[in]  path           待查询路径的信息
   * @param[in]  points_size    信息数目
   */
  PathSearch(const PathAdapter& path, size_t points_size)
      : path_(path), points_size_(points_size) {}

  /*
   * @brief 使用二分查找法，查找指定路径长处的路径点的索引和在对应路径段处的比例。
   * @param[in]  path_len       路径长
   * @param[out] t              指定路径长处的路径点在对应路径段处的比例
   * @retval 指定路径长处的路径点的索引
   */
  Int32_t BinarySearch(geo_var_t path_len, geo_var_t& t) const {
    Int32_t span = 0;
    t = 0;
    if (path_len < FLT_EPSILON) {
      span = 0;
    } else if (path_len > (path_[points_size_-1] - FLT_EPSILON)) {
      span = points_size_ - 1;
    } else {
      Int32_t index = path_.LowerBound(path_len, FuncCmpArcLen());
      if (index > (points_size_ - 1)) {
        span = points_size_ - 1;
      } else {
        span = index;
        if (span < 1) {
          span = 0;
        } else {
          --span;
          if ((span + 1) < points_size_) {
            t = (path_len - path_[span]) / (path_[span + 1] - path_[span]);
          }
        }
      }
    }
    return (span);
  }

  /*
   * @brief 在路径上向前采样一个点。
   * @param[in] curr_s       待采样处的起始弧长
   * @param[in] step_len     采样间距
   * @param[in] extend       待采样点不在路径上时，是否仍然进行采样
   * @param[in&out] span     查询到的采样点对应的路径点的索引
   * @param[out] t           查询到的采样点对应的路径段上的比例值
   * @retval true 采样成功；false 采样失败，待采样点不在路径上。
   */
  bool StepForward(geo_var_t curr_s, geo_var_t step_len, bool extend,
                   Int32_t *span, geo_var_t *t) const {
    bool found = false;
    while ((*span + 1) < points_size_) {
      if ((path_[*span + 1] - curr_s) > step_len) {
        *t = (curr_s + step_len - path_[*span]) /
            (path_[*span + 1] - path_[*span]);
        found = true;
        break;
      }
      ++(*span);
    }

    if (!found && extend) {
      *t = (curr_s + step_len - path_[*span - 1]) /
          (path_[*span] - path_[*span - 1]);
    }

    if ((*span >= points_size_-1) && (points_size_ > 1)) {
      *span = points_size_-2;
    }

    return (found);
  }

  /*
   * @brief 在路径上向后采样一个点。
   * @param[in] curr_s       待采样处的起始路径长
   * @param[in] step_len     采样间距
   * @param[out] span        查询到的采样点对应的路径点的索引
   * @param[out] t           查询到的采样点对应的路径段上的比例值
   * @param[out] extend      待采样点不在路径上时，是否仍然进行采样
   * @retval true 采样成功；false 采样失败，待采样点不在路径上。
   */
  bool StepBackward(geo_var_t curr_s, geo_var_t step_len, Int32_t *span,
      geo_var_t *t, bool extend = false) const {
    bool found = false;
    while (*span > 0) {
      if ((curr_s - path_[*span]) > step_len) {
        if ((*span + 1) < points_size_) {
          *t = (curr_s - step_len - path_[*span]) /
              (path_[*span + 1] - path_[*span]);
          found = true;
          break;
        }
      }
      --(*span);
    }
    if (!found) {
      if (0 == *span) {
        if ((curr_s - path_[*span]) > step_len) {
          if ((*span + 1) < points_size_) {
            *t = (curr_s - step_len - path_[*span])
                / (path_[*span + 1] - path_[*span]);
            found = true;
          }
        }
      }

      if (!found && extend) {
        if ((*span + 1) < points_size_) {
          *t = (curr_s - step_len - path_[*span]) /
              (path_[*span + 1] - path_[*span]);
        }
      }
    }

    return (found);
  }

private:
  // 路径点待查询信息
  const PathAdapter& path_;
  // 查询信息数目
  Int32_t points_size_;
};


}  // namespace common
}  // namespace phoenix

#endif  // PHOENIX_COMMON_PATH_H_


