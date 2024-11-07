#pragma once

/**
 * @file        hdi_decision_map_structs.hpp
 * @brief       决策图层内容中的结构体定义头文件
 * @copyright   Copyright (c) 2017-2027 [中海庭数据技术有限公司](http://www.headingdata.com)
 */

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <map>
#include <memory>
#include <vector>

#include "hdi_decision_map_constants.hpp"

namespace hdi
{

namespace decision_map
{

/**
 * @brief   坐标点
 * @details 在 ENU 坐标系统下，表示坐标时为相对基准点的偏移，而表示基准点时则为相对于上一个基准点的偏移
 */
struct Coordinate
{
    static constexpr double INVALID_VALUE = DBL_MAX;    ///< 无效的 X、Y 值

    Coordinate() noexcept = default;

    Coordinate(const double x, const double y, const double z = 0.0) noexcept
    {
        m_x = x;
        m_y = y;
        m_z = z;
    }

    inline bool valid() const
    {
        return (INVALID_VALUE != m_x && INVALID_VALUE != m_y);
    }

    inline bool operator ==(const Coordinate& other) const
    {
        return this == &other
            || (std::abs(this->m_x - other.m_x) < std::numeric_limits<double>::epsilon()
                && std::abs(this->m_y - other.m_y) < std::numeric_limits<double>::epsilon());
    }

    double m_x  = INVALID_VALUE;    ///< @brief     在坐标系统中的 X 值
                                    ///< @details   WGS84 坐标系：经度，单位：1E-7 角度
                                    ///< @details   ENU 坐标系：东向偏移，单位：米
    double m_y  = INVALID_VALUE;    ///< @brief     在坐标系统中的 Y 值
                                    ///< @details   WGS84 坐标系：纬度，单位：1E-7 角度
                                    ///< @details   ENU 坐标系：北向偏移，单位：米
    double m_z  = 0.0;              ///< @brief     在坐标系统中的 Z 值
                                    ///< @details   WGS84 坐标系：高程，单位：厘米，通常为 0
                                    ///< @details   ENU 坐标系：海拔偏移，单位：米，通常为 0
};

/**
 * @brief 道路层级
 */
struct RoadLevel
{
    static constexpr uint32_t COMMON_SCENE_ID           = UINT32_MAX;   ///< 城区/地库场景使用的场景 ID
    static constexpr int32_t  CROSSLAYER_INVALID_LEVEL  = UINT8_MAX;    ///< 高速/城市快速路场景中层级的无效值
    static constexpr int32_t  COMMON_INVALID_LEVEL      = 0x1F;         ///< 城区/地库场景中层级的无效值

    RoadLevel() = default;

    RoadLevel(const ZLevelType levelType, const int8_t level, const uint32_t sceneId = UINT32_MAX)
    {
        m_levelType = levelType;
        m_level = level;
        m_sceneId = sceneId;
    }

    RoadLevel(const RoadLevel& other)
    {
        m_levelType = other.m_levelType;
        m_level = other.m_level;
        m_sceneId = other.m_sceneId;
    }

    RoadLevel& operator=(const RoadLevel& other) = default;

    bool operator==(const RoadLevel &other) const
    {
        return (m_levelType == other.m_levelType)
            && (m_level == other.m_level)
            && (m_sceneId == other.m_sceneId);
    }

    bool operator!=(const RoadLevel &other) const
    {
        return !(*this == other);
    }

    void clear()
    {
        m_levelType = ZLevelType::Invalid;
        m_level = COMMON_INVALID_LEVEL;
        m_sceneId = COMMON_SCENE_ID;
    }

    /**
     * 是否为有效的层级
     */
    inline bool isValid() const
    {
        return (ZLevelType::Invalid != m_levelType);
    }

    /**
     * 检查是否高速/城快的立体交叉点层级
     */
    inline bool isCrossLayerLevel() const
    {
        return (isValid() && m_sceneId != COMMON_SCENE_ID);
    }

    /**
     * 是否与另一个有效的层级相等
     */
    inline bool isCompatibleLevel(const RoadLevel &other) const
    {
        return (!isValid() || !other.isValid() || *this == other);
    }

    ZLevelType  m_levelType  = ZLevelType::Invalid; ///< 道路层级类型
    int32_t     m_level     = COMMON_INVALID_LEVEL; ///< @brief     具体的对应层级
                                                    ///< @details   @ref m_sceneId 相同的两个层级可以相互比较
    uint32_t    m_sceneId   = COMMON_SCENE_ID;      ///< 体交叉点的分组 ID
};

/**
 * @brief 匹配信息
 */
struct MatchedPosition
{
    PathId          m_pathId                = INVALID_PATHID;   ///< 车道所在路径 ID
    Offset          m_offset                = INVALID_OFFSET;   ///< 车辆位置到路径起点的距离
    LaneIndex       m_currentLane           = 0;                ///< 当前车道编号
    VehicleSpeed    m_speed                 = 0.0;              ///< 车速
    float           m_relativeHeading       = 0.0;              ///< @brief     车辆与车道的偏差角度
                                                                ///< @details   车辆的航向 - 匹配车道的航向，单位：角度，范围：@f$[0, 360)@f$
    float           m_probability           = 0.0;              ///< @brief     匹配的置信度，单位：百分之一
                                                                ///< @details   范围：@f$[0, 100]@f$
    bool            m_onLane                = false;            ///< 是否在匹配的车道面内
    RoadLevel       m_roadLevel;                                ///< 匹配车道所对应的道路层级
    Distance        m_distToRightLaneLine   = UINT32_MAX;       ///< @brief     车辆位置到右侧车道边线的距离
                                                                ///< @details   仅当车辆位于车道上时为有效值
    Distance        m_distToLeftLaneLine    = UINT32_MAX;       ///< @brief     车辆位置到左侧车道边线的距离
                                                                ///< @details   仅当车辆位于车道上时为有效值
};

/**
 * @brief   车辆位置消息
 * @note    使用 ENU 坐标系统时，位置消息的基准点时间戳必须和图层数据的基准点时间戳一致才能结合使用
 */
struct PositionMessage
{
    static constexpr Timestamp  INVALID_POSITION_AGE    = UINT64_MAX;   ///< 无效的位置消息时间差

    Timestamp                       m_timestamp         = INVALID_TIMESTAMP;        ///< 从定位系统获取到 GPS 坐标坐标的时间戳，单位：毫秒
    Timestamp                       m_positionAge       = INVALID_POSITION_AGE;     ///< 获取到 GPS 位置到向车辆总线发送位置消息的时间差
    Timestamp                       m_datumTimestamp    = 0;                        ///< 当前基准点的时间戳
    float                           m_heading           = 0.0;                      ///< 车辆航向角，单位：度
    VehicleSpeed                    m_speed             = 0.0;                      ///< 车辆速度，单位：米/秒
    Coordinate                      m_point;                                        ///< 车辆位置坐标
    uint32_t                        m_confidence        = 0;                        ///< 车辆位置的置信度
    ODDStatusType                   m_oddStatus         = ODDStatusType::Unknown;   ///< 地理围栏的状态
    std::vector<MatchedPosition>    m_positions;                                    ///< 车辆位置的匹配结果列表
};

/**
 * @brief 属性值的基类
 */
struct ProfileValue
{
    ProfileValue()          = default;
    virtual ~ProfileValue() = default;
};

/**
 * @brief 属性信息
 */
struct Profile
{
    InstanceId                      m_instanceId        = 0;                    ///< @brief     属性实体 ID
                                                                                ///< @details   每一类型的属性之间唯一
    ChangeMode                      m_change            = ChangeMode::Create;   ///< 变化类型
    PathId                          m_pathId            = INVALID_PATHID;       ///< 路径 ID
    Offset                          m_offset            = INVALID_OFFSET;       ///< 控制范围的起始点在路径上的距离
    Offset                          m_endOffset         = INVALID_OFFSET;       ///< 控制范围的终止点在路径上的距离
    bool                            m_endOffsetFinal    = true;                 ///< @ref m_endOffset 是否不会再更新
    ProfileType                     m_profileType       = ProfileType::NA;      ///< 属性类型
    std::vector<LaneIndex>          m_lanes;                                    ///< @brief     控制的车道列表
                                                                                ///< @details   为空时表示控制全部车道
    std::shared_ptr<ProfileValue>   m_profileValue      = nullptr;              ///< 属性值

    template<typename T, typename std::enable_if<std::is_base_of<ProfileValue, T>::value>::type* = nullptr>
    inline std::shared_ptr<T> typedValue() const
    {
        return std::dynamic_pointer_cast<T>(m_profileValue);
    }
};

/**
 * @brief 属性消息
 */
using ProfileMessage = std::vector<std::shared_ptr<Profile>>;

/**
 * @brief 路口信息
 */
struct NodeArm
{
    PathId      m_subPath   = INVALID_PATHID;       ///< 该节点所指向的路径 ID
    TurnInfo    m_turnInfo  = TurnInfo::Unknown;    ///< 转向信息
    float       m_turnAgnle = 0.0;                  ///< 路口进入位置航向和脱出位置航向差值的绝对值
    PathType    m_pathType  = PathType::NA;         ///< 路径类型
};

/**
 * @brief 路口信息的属性值
 */
struct NodeProfileValue : public ProfileValue
{
    std::vector<NodeArm>    m_arms; ///< 路口信息列表
};

/**
 * @brief 车道信息
 *
 * 车道边线可能由多条线组成，ID 列表按从右往左的顺序存放
 */
struct LaneInfo
{
    LaneIndex           m_laneIndex     = INVALID_LANE_INDEX;                       ///< @brief     车道编号
                                                                                    ///< @details   从右往左，从1开始编号
    RelativeDirection   m_direction     = RelativeDirection::AlongPathDirection;    ///< 车道方向
    LaneTransition      m_transition    = LaneTransition::None;                     ///< 过渡类型
    LaneConnectType     m_connectType   = LaneConnectType::NA;                      ///< 连接关系
    Distance            m_length        = 0;                                        ///< 车道长度

    std::vector<LaneType>   m_types;    ///< 车道类型列表

    LineId                  m_centerLine;       ///< 中心线 ID
    std::vector<LineId>     m_leftBoundaries;   ///< 左边线 ID 列表
    std::vector<LineId>     m_rightBoundaries;  ///< 右边线 ID 列表
};

/**
 * @brief 车道模型的属性值
 */
struct LaneModelProfileValue : public ProfileValue
{
    std::vector<LaneInfo> m_lanes; ///< 车道信息列表
};

/**
 * @brief 车道拓扑
 */
struct LaneConnectivityPair
{
    LaneIndex   m_initialLane   = INVALID_LANE_INDEX;   ///< 当前的路径编号
    PathId      m_initialPath   = INVALID_PATHID;       ///< 当前的车道编号
    LaneIndex   m_newLane       = INVALID_LANE_INDEX;   ///< 脱出的路径编号
    PathId      m_newPath       = INVALID_PATHID;       ///< 脱出的车道编号
};

/**
 * @brief 车道拓扑的属性值
 */
struct LaneConnectivityProfileValue : public ProfileValue
{
    std::vector<LaneConnectivityPair>   m_connectivityPairs;    ///< 车道拓扑信息列表
};

/**
 * @brief 线型物体
 */
struct LinearObject
{
    LineId              m_lineId    = INVALID_LINE_ID;      ///< 线ID
    LinearObjectType    m_type      = LinearObjectType::NA; ///< 线型物体的类型
    LineMarking         m_marking   = LineMarking::NA;      ///< @brief     线型物体的标记类型
                                                            ///< @details   仅类型为边线时有效
                                                            ///<            仅输出首个值，建议使用 @ref ProfileType:LineMarkingType 获取完整列表
    LineColor           m_color     = LineColor::Unknown;   ///< @brief     线型物体的颜色
                                                            ///< @details   仅类型为边线时有效
                                                            ///<            仅输出首个值，建议使用 @ref ProfileType:LineMarkingColour 获取完整列表
    RoadSide            m_roadSide  = RoadSide::NA;         ///< 线性物体在道路的哪一侧
};

/**
 * @brief 线型物体的属性值
 */
struct LinearObjectProfileValue : public ProfileValue
{
    std::vector<LinearObject> m_linearObjects;  ///< 线型物体列表
};

/**
 * @brief 线型物体几何
 */
struct LineGeometry
{
    LineId                  m_lineId    = INVALID_LINE_ID;          ///< 线 ID
    CurveType               m_curveType = CurveType::NotPresent;    ///< 曲线类型
    std::vector<Coordinate> m_points;                               ///< 形点串
};

/**
 * @brief 车道几何属性值
 */
struct LaneGeometryProfileValue : public ProfileValue
{
    std::vector<LineGeometry>   m_lines;    ///< 线型物体几何列表
};

/**
 * @brief Offset - Float 类型的控制点
 *
 * @details
 * | @ref ProfileType | @ref m_value 的说明
 * | :--- | :---
 * | @ref ProfileType::LaneWidth    | 单位：厘米，没有明确的范围，一般根据实际的来
 * | @ref ProfileType::Curvature    | 范围：@f$[0, 1023]@f$
 * | @ref ProfileType::Slope        | 范围：@f$[0, 127]@f$
 * | @ref ProfileType::Heading      | 正北为0，顺时增加，单位：角度，范围：@f$[0, 360)@f$
 * | @ref ProfileType::CrossSlope   | 范围：@f$[0, 127]@f$
 */
struct OffsetFloatEntry
{
    Offset  m_offset    = INVALID_OFFSET;   ///< 路径上的距离
    float   m_value     = 0.0;              ///< 控制点的值
};

/**
 * @brief Offset - Float 类型的属性值
 */
struct OffsetFloatProfileValue : public ProfileValue
{
    std::vector<OffsetFloatEntry>   m_entries;  ///<  Offset - Float 类型的控制点列表
};

/**
 * @brief 道路类型的属性值
 */
struct FormOfWayProfileValue : public ProfileValue
{
    FormOfWay                   m_type          = FormOfWay::Unknown;   ///< 道路类型
    std::vector<RoadAttribute>  m_attributes;                           ///< 道路属性列表

    FormOfWayProfileValue& operator=(const FormOfWayProfileValue& other)
    {
        if (&other != this)
        {
            m_type = other.m_type;
            m_attributes = other.m_attributes;
        }
        return *this;
    }

    /**
     * @brief 查询是否存在道路属性
     */
    inline bool hasAttribute(RoadAttribute attribute) const
    {
        return m_attributes.end() != std::find(m_attributes.begin(), m_attributes.end(), attribute);
    }

    /**
     * @brief 查询是否高速与普通道路连接路（IC）
     */
    inline bool isICRamp() const
    {
        return FormOfWay::Ramp == m_type && hasAttribute(RoadAttribute::MotorWay);
    }

    /**
     * @brief 查询是否高速间连接路（JC）
     */
    inline bool isJCRamp() const
    {
        return FormOfWay::Ramp == m_type
            && hasAttribute(RoadAttribute::MotorWay)
            && hasAttribute(RoadAttribute::ControlAccess);
    }

    /**
     * @brief 查询是否路口
     */
    inline bool isCross() const
    {
        return FormOfWay::Ramp != m_type
                && hasAttribute(RoadAttribute::ComplexIntersection)
                && hasAttribute(RoadAttribute::PluralJunction);
    }
};

/**
 * @brief 单一属性的属性值
 */
template<typename T, T DEFAULT>
struct SingleValueProfileValue : public ProfileValue
{
    T m_value = DEFAULT;    ///< 具体的属性值
};

/**
 * @brief 布尔类型的属性值
 */
using BooleanProfileValue = SingleValueProfileValue<bool, false>;

/**
 * @brief 交通标志牌的属性值
 */
struct TrafficSignProfileValue: public ProfileValue
{
    SignType    m_type  = SignType::Unknown;    ///< 标志牌类型

    std::vector<Coordinate> m_shape;        ///< 标志牌的形状点
    std::vector<Coordinate> m_boundingBox;  ///< 标志牌的外接包围盒
};

/**
 * @brief 交通灯的属性值
 */
struct TrafficLightProfileValue : public ProfileValue
{
    Coordinate                      m_point;                                                        ///< 位置坐标点
    TrafficLightType                m_type              = TrafficLightType::Unknown;                ///< 信号灯类型
    TrafficLightConstructionType    m_constructionType  = TrafficLightConstructionType::Unknown;    ///< 安装位置

    std::vector<LateralPositionFlag>    m_lateralPositions; ///< 横向位置
    std::vector<Coordinate>             m_boundingBox;      ///< 包围盒
};

/**
 * @brief 特殊场景的属性值
 */
struct SpecialSituationProfileValue : public ProfileValue
{
    SpecialSituationType    m_type  = SpecialSituationType::Unknown;    ///< 特殊场景的类型
    std::vector<Coordinate> m_shape;                                    ///< 形状点串
};

/**
 * @brief 限速值
 */
struct Speed
{
    uint32_t    m_value = INVALID_SPEED_LIMIT;  ///< 速度的具体值
    SpeedUnit   m_unit  = SpeedUnit::KpH;       ///< 速度的单位
};

/**
 * @brief 有效限速的属性值
 */
struct EffectiveSpeedLimitProfileValue : public ProfileValue
{
    Speed                   m_value;                                            ///< 具体属性值
    EffectiveSpeedLimitType m_type          = EffectiveSpeedLimitType::Unknown; ///< 限速类型
                                                                                ///< @note 预留，暂不实现
    VehicleType             m_vehicleType   = VehicleType::None;                ///< 限速对应的车种类型
};

/**
 * @brief 驾驶方向的属性值
 */
using DrivingSideProfileValue = SingleValueProfileValue<DrivingSide, DrivingSide::Unknown>;

/**
 * @brief 无符号32位整型的属性值
 * @details
 * | @ref ProfileType | @ref m_value 的说明
 * | :--- | :---
 * | @ref ProfileType::VersionProtocol  | 组合方式：@f$2^{24} \times MAJOR + 2^{16} \times MINOR + PATCH@f$
 * | @ref ProfileType::MapAge           | 从 1970 年 1 月 1 日开始的小时数
 */
using UInt32ProfileValue = SingleValueProfileValue<uint32_t, 0>;

/**
 * @brief 无符号64位整型的属性值
 * @details
 * | @ref ProfileType | @ref m_value 的说明
 * | :--- | :---
 * | @ref ProfileType::VersionMap | 组合方式：@f$2^{48} \times MAJOR + 2^{32} \times MINOR + 2^{16} \times SUB@f$
 * | @ref ProfileType::VersionSoftware | 组合方式：EHR 版本号占前 32 位，EHP 版本号占后 32 位，版本号为 @f$2^{24} \times MAJOR + 2^{16} \times MINOR + PATCH@f$
 */
using UInt64ProfileValue = SingleValueProfileValue<uint64_t, 0>;

/**
 * @brief 地图状态的属性值
 */
using MapStatusProfileValue = SingleValueProfileValue<MapStatus, MapStatus::MapNotAvailable>;

/**
 * @brief 车辆位置
 */
struct AbsolutePosition : public Coordinate
{
    Timestamp       m_timestamp = 0;    ///< 定位信号时间戳
    float           m_heading   = 0.0;  ///< @brief     车辆航向角
                                        ///< @details   正北为 0，顺时增加，单位：角度，范围：@f$[0, 360)@f$
    VehicleSpeed    m_speed     = 0.0;  ///< 速度

    Timestamp   m_datumTimestamp    = 0;    ///< 当前基准点的时间戳
    Timestamp   m_lastTimestamp     = 0;    ///< 上一个信号的时间戳
    Coordinate  m_relativeVehicle;          ///< 相对上一个位置的偏移
    uint32_t    m_confidence        = 0;    ///< 车辆位置的置信度

    AbsolutePosition() noexcept = default;
};

/**
 * @brief 车辆的绝对位置的属性值
 */
struct AbsoluteVehiclePositionProfileValue : public ProfileValue
{
    AbsolutePosition    m_position;                             ///< 车辆的绝对位置和姿态
    ODDStatusType       m_oddStatus  = ODDStatusType::Unknown;  ///< 地理围栏的状态
};

/**
 * @brief 道路等级的属性值
 */
using RoadClassProfileValue = SingleValueProfileValue<RoadClass, RoadClass::Unknown>;

/**
 * @brief 线型物体的控制点
 */
template<typename T, T DEFAULT>
struct LineControlPoint
{
    Distance    m_dist;             ///< 距离线起点的距离
    T           m_value = DEFAULT;  ///< 控制点值

    LineControlPoint() = default;
    LineControlPoint(const Distance dist, const T value)
    {
        m_dist = dist;
        m_value = value;
    }
};

/**
 * @brief 线型物体的控制点列表
 */
template<typename T, T DEFAULT>
struct LineControlPoints
{
    using ControlPointType = LineControlPoint<T, DEFAULT>;
    LineId                          m_lineId;           ///< 线 ID
    std::vector<ControlPointType>   m_controlPoints;    ///< 线型物体的控制点列表
};

/**
 * @brief 线型物体的控制点列表的属性值
 */
template<typename T, T DEFAULT>
struct LineControlPointsProfileValue : public ProfileValue
{
    using LineType          = LineControlPoints<T, DEFAULT>;
    using ControlPointType  = typename LineType::ControlPointType;
    std::vector<LineType>   m_lines;    ///< 线型物体列表
};

/**
 * @brief 线型物体的标记类型的属性值
 */
using LineMarkingProfileValue = LineControlPointsProfileValue<LineMarking, LineMarking::Unknown>;

/**
 * @brief 线型物体的颜色的属性值
 */
using LineColorProfileValue = LineControlPointsProfileValue<LineColor, LineColor::Unknown>;

/**
 * @brief 线型物体的逻辑的属性值
 */
using LineLogicProfileValue = LineControlPointsProfileValue<LineLogic, LineLogic::NA>;

/**
 * @brief 道路层级的属性值
 */
struct RoadLevelProfileValue : public ProfileValue
{
    RoadLevel   m_roadLevel;    ///< 道路层级
};
/**
 * @brief   匝道关联分流汇流区间的信息
 * @details Offset 会动态更新以保证准确
 */
struct RampJunction
{
    ConnectRampType m_type      = ConnectRampType::ICIn;    ///< 匝道类型
    Offset          m_offset    = INVALID_OFFSET;           ///< 匝道关联分流汇流区间起始点在路径上的距离
    Offset          m_endOffset = INVALID_OFFSET;           ///< 匝道关联分流汇流区间终止点在路径上的距离
    uint32_t        m_index     = UINT32_MAX;               ///< 全局规划路径上的第几个匝道路口
};

/**
 * @brief 路径点信息
 */
struct NaviPoint
{
    NaviPointType   m_type = NaviPointType::Passby; ///< 路径点的类型
    Coordinate      m_point;                        ///< 路径点的坐标
    uint8_t         m_index;                        ///< 路径点的索引
    LaneIndex       m_laneIndex;                    ///< 路径点的车道序号
    Offset          m_offset;                       ///< 路径点在路径上的距离

    PathId  m_pathId;   ///< 路径点的路径 ID
};

/**
 * @brief 全局规划信息
 * @details 默认是输出车辆前方50km内的所有匝道口信息（即使规划路线不下匝道也会给出），目前只支持匝道节点，其他节点需要定制
 */
struct WholeRouteProfileValue : public ProfileValue
{
    uint32_t                    m_wholeRouteLength; ///< 路线的总长度，单位：厘米
    std::vector<RampJunction>   m_rampJunctions;    ///< 匝道关联分流汇流区间列表
    std::vector<NaviPoint>      m_naviPoints;       ///< 路径点列表
};

/**
 * @brief 车道扩展信息
 */
struct AuxLane
{
public:
    LaneIndex                       m_laneIndex = INVALID_LANE_INDEX;   ///< 车道编号
    std::vector<AuxLaneTypeFlag>    m_flags;                            ///< @brief 扩展属性列表
};

/**
 * @brief 车道的扩展信息的属性值
 */
struct AuxLaneInfoProfileValue : public ProfileValue
{
    std::vector<AuxLane> m_lanes;  ///< 车道扩展信息列表
};

/**
 * @brief 同向道路区间
 */
struct SameDirectionSection
{
public:
    PathId  m_iPathId   = INVALID_PATHID;   ///< 同向道路区间所属的路径
    Offset  m_iOffset   = INVALID_OFFSET;   ///< 同向道路区间在路径上的距离
};

/**
 * @brief 同向道路区间的属性值
 */
struct SameDirectionSectionsProfileValue : public ProfileValue
{
public:
    std::vector<SameDirectionSection> m_sections;    ///< 当前道路区间的同向道路区间列表
};

/**
 * @brief 道路信息的属性值
 * @details 没有内容
 */
struct RoadProfileValue : public ProfileValue { };

/**
 * @brief 道路宽度控制点
 */
struct RoadWidthControlPoint
{
    Distance    m_dist          = 0;            ///< 距离车道起点的距离，单位：厘米
    uint16_t    m_distToLeft    = UINT16_MAX;   ///< 车道中心线距离左侧道路外侧线的距离，单位：厘米
    uint16_t    m_distToRight   = UINT16_MAX;   ///< 车道中心线距离右侧道路外侧线的距离，单位：厘米
};

/**
 * @brief 道路宽度的属性信息
 */
struct RoadWidthProfileValue : public ProfileValue
{
    std::vector<RoadWidthControlPoint> m_controlPoints; ///< 当前车道的道路宽度控制点列表
};

/**
 * @brief 线型物体的宽度的属性值
 */
using LineWidthProfileValue = LineControlPointsProfileValue<uint32_t, 0>;

/**
 * @brief   线型物体的来源的属性值
 */
using LineSourceProfileValue = LineControlPointsProfileValue<LineSource, LineSource::Unknown>;

/**
 * @brief 许可证状态的属性值
 */
using LicenseStatusProfileValue = SingleValueProfileValue<LicenseStatus, LicenseStatus::Unknown>;

/**
 * @brief 坐标系统的属性值
 */
using CoordinateSystemProfileValue = SingleValueProfileValue<CoordinateSystem, CoordinateSystem::Unknown>;


/**
 * @brief   道路区间的键
 * @details 地图数据中能够标志一个道路区间的 ID 组合
 */
struct RoadSectionKey
{
    int16_t    m_urId           = INT16_MAX;    ///< 更新区域（Update Region）ID
    int32_t    m_tileId         = INT32_MAX;    ///< 瓦片 ID
    int16_t    m_laneGroupId    = INT16_MAX;    ///< 车道组 ID
};

/**
 * @brief 道路区间的属性信息
 */
struct RoadSectionProfileValue : public ProfileValue
{
    RoadSectionKey m_key; ///< 道路区间的键
};

/**
 * @brief 基准点信息
 */
struct DatumPointProfileValue : public ProfileValue
{
    uint64_t    m_timestamp     = 0;    ///< 当前基准点对应的信号时间戳
    uint64_t    m_lastTimestamp = 0;    ///< 上一个基准点对应的信号时间戳
    Coordinate  m_point;                ///< @brief     基准点的位置
                                        ///< @details   相对于上一个基准点的偏移
};

/**
 * @brief   车道边线的物理可跨越性的属性值
 * @note    仅列出存在不可跨越区间的车道边线（即若未列出某条车道边线，则表示其为完全可跨越）
 */
using LinePhysicalCrossProfileValue = LineControlPointsProfileValue<PhysicalCrossType, PhysicalCrossType::Unknown>;

/**
 * @brief   线上设施类型的控制点
 */
struct LineFacilityTypeControlPoint
{
    Distance                                m_dist   = 0;   ///< 距离线起点的距离
    std::vector<LaneBoundaryFacilityType>   m_values;       ///< 线上设施类型列表
};

/**
 * @brief   车道边线的线上设施类型的控制点列表
 */
struct LineFacilityTypeControlPoints
{
    LineId                                      m_lineId        = 0;    ///< 线 ID
    std::vector<LineFacilityTypeControlPoint>   m_controlPoints;        ///< 线型物体的控制点列表
};

/**
 * @brief   车道边线的线上设施类型的属性值
 * @note    仅列出包含线上设施的车道边线（即若未列出某条车道边线，则表示其完全不包含线上设施）
 */
struct LineFacilityTypeProfileValue : public ProfileValue
{
    std::vector<LineFacilityTypeControlPoints> m_lines; ///< 车道边线的线上设施类型的具体属性
};

/**
 * @brief   车道边线上的纵向减速标线的属性值
 * @note    仅列出包含纵向减速标线的车道边线（即若未列出某条车道边线，则表示其完全不包含纵向减速标线）
 */
using LineSpeedReductionProfileValue = LineControlPointsProfileValue<LineSide, LineSide::Unkown>;

/**
 * @brief 属性的控制数据
 */
struct ProfileControl
{
    PathId  m_pathId    = INVALID_PATHID;   ///< 需要删除属性的路径 ID
    Offset  m_offset    = INVALID_OFFSET;   ///< 此路径上要保留的最小 Offset
};

/**
 * @brief 属性的控制数据消息
 */
using ProfileControlMessage = std::vector<ProfileControl>;

/**
 * @brief 路径的控制数据
 */
struct PathControl
{
    PathId  m_pathId    = INVALID_PATHID;   ///< 需要保留的路径编号
    PathId  m_parentId  = INVALID_PATHID;   ///< 此路径的亲路径编号
    Offset  m_offset    = INVALID_OFFSET;   ///< 此路径的起点在亲路径上的 Offset
};

/**
 * @brief 路径的控制数据消息
 */
using PathControlMessage = std::vector<PathControl>;

/**
 * @brief 全局属性
 */
struct Global
{
    ProfileType                     m_profileType   = ProfileType::NA;  ///< 属性类型
    std::shared_ptr<ProfileValue>   m_profileValue  = nullptr;          ///< 属性值

    template<typename T>
    inline std::shared_ptr<T> typedValue() const
    {
        return std::dynamic_pointer_cast<T>(m_profileValue);
    }
};


/**
 * @brief 全局属性消息
 */
using GlobalMessage = std::vector<Global>;


/**
 * @brief 配准失败段信息
 */
struct MatchFailResult
{
    Offset       m_offset  = 0U;        ///< 配准失败路段起始位置在SD路线上的offset, 单位 cm
    Distance     m_length  = 0U;        ///< 配准失败路段的SD路线长度，单位 cm
};

/**
 * @brief 路径配准信息
 * @note  如果 PathMatchStatus 全部失败, 则 m_results 可能为空
 * @note  如果 PathMatchStatus 成功，且中间没有失败路段, 则 m_results 可能为空
 */
struct PathMatchInfoValue : public ProfileValue
{
    uint32_t                        m_sdRouteId  = UINT32_MAX;         ///< SD路线ID,默认值UINT32_MAX 也是无效值
    PathMatchStatus                 m_status = PathMatchStatus::Na;    ///< 配准状态
    std::vector<MatchFailResult>    m_results;                         ///< 失败分段列表

    inline bool valid() const
    {
        return (UINT32_MAX != m_sdRouteId);
    }
};

/**
 * @brief 收到SDRoute数据标志
 * @note  value是更新后的sdrouteid, 默认值 UINT32_MAX
 */
using ReceivedSDRouteValue = SingleValueProfileValue<uint32_t, UINT32_MAX>;

/**
 * @brief 客户定制消息
 */
struct Customization
{
    Timestamp                       m_timestamp = 0U;                  ///< 时间戳
    CustomizedType                  m_type = CustomizedType::NA;       ///< 定制消息类型
    ChangeMode                      m_changeMode = ChangeMode::Create; ///< 变化类型

    std::shared_ptr<ProfileValue>   m_profileValue  = nullptr;         ///< 属性值基类指针
                                                                       ///< ref PathMatchInfo
                                                                       ///< ref ReceivedSDRoute

    template<typename T, typename std::enable_if<std::is_base_of<ProfileValue, T>::value>::type* = nullptr>
    inline std::shared_ptr<T> typedValue() const
    {
        return std::dynamic_pointer_cast<T>(m_profileValue);
    }
};

/**
 * @brief 定制属性消息
 * @details 消息介绍
 *   目前有两类消息(根据 CustomizedType 进行区分):
 *     1.收到SDRoute数据标志
 *     2.路径配准信息
 */
using CustomizationMessage = std::vector<Customization>;

/**
 * @brief 道路区间的位置
 */
struct RoadSectionPosition
{
    PathId  m_path      = INVALID_PATHID;   ///< 道路区间所属路径的 ID
    Offset  m_offset    = INVALID_OFFSET;   ///< 道路区间在路径上的距离

    RoadSectionPosition() = default;

    RoadSectionPosition(PathId path, Offset offset)
    {
        m_path = path;
        m_offset = offset;
    }

    inline bool valid() const
    {
        return m_path != INVALID_PATHID;
    }

    inline bool operator<(const RoadSectionPosition& other) const
    {
        return (m_path != other.m_path) ? m_path < other.m_path : (m_offset < other.m_offset);
    }

    inline bool operator==(const RoadSectionPosition& other) const
    {
        return (m_path == other.m_path)  && (m_offset == other.m_offset);
    }
};

using TypeProfilesMap   = std::map<ProfileType, ProfileMessage>;            ///< 属性类型 - 属性列表字典
using RoadSectionMap    = std::map<RoadSectionPosition, TypeProfilesMap>;   ///< 道路区间的键 - 数据集合字典
using OffsetRange       = std::pair<Offset, Offset>;                        ///< 在路径上的距离的起止范围

/**
 * @brief 决策图层数据快照
 * @note  使用 ENU 坐标系统时，车辆位置消息的基准点时间戳和快照的基准点时间戳一致才能够结合使用二者的坐标
 */
struct DecisionMapData
{
public:
    DecisionMapData() = default;
    ~DecisionMapData() = default;

public:
    Timestamp           m_timestamp         = INVALID_TIMESTAMP;    ///< 生成快照的时间戳
    Timestamp           m_datumTimestamp    = INVALID_TIMESTAMP;    ///< 数据的基准点时间戳
    RoadSectionPosition m_mainKey;                                  ///< 主路径的首个道路区间键
    RoadSectionMap      m_data;                                     ///< 决策图层数据
};

} // namespace decision_map

} // namespace hdi
