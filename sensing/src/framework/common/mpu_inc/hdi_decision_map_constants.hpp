#pragma once

/**
 * @file        hdi_decision_map_constants.hpp
 * @brief       决策图层内容中的基础类型、常量和枚举定义头文件
 * @copyright   Copyright (c) 2017-2027 [中海庭数据技术有限公司](http://www.headingdata.com)
 */

#include <cstdint>

namespace hdi
{

namespace decision_map
{

using Distance      = uint32_t; ///< @brief 相对距离，单位：厘米
                                ///< @note  请注意与 @ref Offset 区分
using InstanceId    = uint64_t; ///< 全局的唯一编号
using LaneIndex     = uint16_t; ///< @brief     车道编号
                                ///< @details   从右到左，从 1 开始，0 表示未知或不在道路上
using LineId        = uint64_t; ///< 线编号
using Offset        = uint32_t; ///< @brief     在路径上的距离，单位：厘米
                                ///< @details   特指从规划路径的起点所在的道路区间的起点开始，沿此路径上最左侧车道中心线的累积距离
                                ///< @note      请注意与 @ref Distance 区分
using PathId        = uint32_t; ///< 路径编号
using Timestamp     = uint64_t; ///< 时间戳类型
using VehicleSpeed  = float;    ///< 车速，单位：米/秒

static constexpr PathId     INVALID_PATHID          = 0;            ///< 无效的路径编号
static constexpr Offset     INVALID_OFFSET          = UINT32_MAX;   ///< 无效的 Offset
static constexpr Timestamp  INVALID_TIMESTAMP       = UINT64_MAX;   ///< 无效的时间戳
static constexpr LaneIndex  INVALID_LANE_INDEX      = 0;            ///< 无效的车道编号
static constexpr LineId     INVALID_LINE_ID         = 0;            ///< 无效的线编号
static constexpr float      INVALID_HEADING_DEGREE  = 361;          ///< 无效的航向角度
static constexpr float      INVALID_CURVATURE       = 1023;         ///< 无效的曲率值
static constexpr float      INVALID_SLOPE           = 10000;        ///< 无效的坡度值
static constexpr float      INVALID_SPEED           = 10000;        ///< 无效的速度值
static constexpr float      INVALID_ACCSPEED        = 10000;        ///< 无效的加速度
static constexpr float      INVALID_PROBABILITY     = -1.0f;        ///< 无效的置信度
static constexpr uint32_t   INVALID_SPEED_LIMIT     = UINT32_MAX;   ///< 无效的限速值

static constexpr PathId VALID_PATHID_MIN    = 8;    ///< 有效的路径编号起始值，一般为首次规划成功后主路径的编号

/**
 * @brief 属性类型
 */
enum class ProfileType : uint8_t
{
    Node                            = 0,    ///< @brief     路口
                                            ///< @details   有道路合流/分歧的位置，对应 @ref NodeProfileValue
    Probability                     = 1,    ///< @brief     置信度
                                            ///< @note      预留，暂不实现
    HeadingChange                   = 2,    ///< @brief     航向变化
                                            ///< @note      预留，暂不实现
    LaneModel                       = 3,    ///< @brief     车道模型
                                            ///< @details   用于描述车道的属性，对应 @ref LaneModelProfileValue
    LaneConnectivity                = 4,    ///< @brief     车道拓扑
                                            ///< @details   对应 @ref LaneConnectivityProfileValue
    LinearObjects                   = 5,    ///< @brief     线型物体
                                            ///< @details   对应 @ref LinearObjectProfileValue
    LanesGeometry                   = 6,    ///< @brief     车道几何
                                            ///< @details   对应 @ref LaneGeometryProfileValue
    LaneWidth                       = 7,    ///< @brief     车道宽度
                                            ///< @details   对应 @ref OffsetFloatProfileValue
    RoadGeometry                    = 8,    ///< @brief     道路几何
                                            ///< @note      预留，暂不实现
    NumberOfLanesDrivingDirection   = 9,    ///< @brief     行驶方向上的车道个数
                                            ///< @note      预留，暂不实现
    NumberOfLanesOppositeDirection  = 10,   ///< @brief     相反方向上的车道个数
                                            ///< @note      预留，暂不实现
    ComplexIntersection             = 11,   ///< @brief     复杂路口
                                            ///< @note      预留，暂不实现
    LinkIdentifier                  = 12,   ///< @brief     道路Link的ID
                                            ///< @note      预留，暂不实现
    FunctionalRoadClass             = 13,   ///< @brief     道路功能等级
                                            ///< @note      预留，暂不实现
    RouteNumberTypes                = 14,   ///< @brief     路线号类型
                                            ///< @note      预留，暂不实现
    FormOfWay                       = 15,   ///< @brief     道路类型
                                            ///< @details   对应 @ref FormOfWayProfileValue
    RoadAccessibility               = 16,   ///< @brief     道路的通行类别
                                            ///< @note      预留，暂不实现
    Tunnel                          = 17,   ///< 隧道
                                            ///< @details   对应 @ref BooleanProfileValue
    Bridge                          = 18,   ///< 桥梁
                                            ///< @details   对应 @ref BooleanProfileValue
    DividedRoad                     = 19,   ///< @brief     分离道路
                                            ///< @note      预留，暂不实现
    Curvature                       = 20,   ///< 曲率
                                            ///< @details   范围为[0,1023]，对应 @ref OffsetFloatProfileValue
    Slope                           = 21,   ///< 纵向坡度
                                            ///< @details   范围为[0,127]，对应 @ref OffsetFloatProfileValue
    BuiltUpArea                     = 22,   ///< @brief     内建区域
                                            ///< @note      预留，暂不实现
    InTown                          = 23,   ///< @brief     在城镇
                                            ///< @note      预留，暂不实现
    Surface                         = 24,   ///< @brief     物理道路的表面类型
                                            ///< @note      预留，暂不实现
    TrafficSign                     = 25,   ///< @brief     交通标记
                                            ///< @details   对应 @ref TrafficSignProfileValue
    TrafficLight                    = 26,   ///< @brief     交通信号灯
                                            ///< @details   对应 @ref TrafficLightProfileValue
    SpecialSituation                = 27,   ///< @brief     特殊场景的数据
                                            ///< @details   对应 @ref SpecialSituationProfileValue
    EffectiveSpeedLimit             = 28,   ///< @brief     最高限速
                                            ///< @details   对应 @ref EffectiveSpeedLimitProfileValue
    ExtendedSpeedLimit              = 29,   ///< @brief     扩展限速
                                            ///< @note      预留，暂不实现
    AverageSpeed                    = 30,   ///< @brief     平均限速
                                            ///< @note      预留，暂不实现
    FlowSpeed                       = 31,   ///< @brief     实时的车辆流速
                                            ///< @note      预留，暂不实现
    RoadCondition                   = 32,   ///< @brief     道路的路面状态
                                            ///< @note      预留，暂不实现
    Weather                         = 33,   ///< @brief     天气状态
                                            ///< @note      预留，暂不实现
    LocationObject                  = 34,   ///< @brief     定位物体
                                            ///< @note      预留，暂不实现
    PartOfCalculatedRoute           = 35,   ///< @brief     是否属于规划结果
                                            ///< @details   对应 @ref BooleanProfileValue
    CountryCode                     = 36,   ///< @brief     国家代码
                                            ///< @note      预留，暂不实现
    RegionCode                      = 37,   ///< @brief     区域代码
                                            ///< @note      预留，暂不实现
    DrivingSide                     = 38,   ///< @brief     驾驶方向信息
                                            ///< @details   全局属性，对应 @ref DrivingSideProfileValue
    UnitSystem                      = 39,   ///< @brief     单位系统
                                            ///< @note      预留，暂不实现
    VersionProtocol                 = 40,   ///< @brief     协议版本信息
                                            ///< @details   全局属性，对应 @ref UInt32ProfileValue
    VersionHardware                 = 41,   ///< @brief     硬件版本信息
                                            ///< @note      预留，暂不实现
    VersionMap                      = 42,   ///< @brief     地图版本信息
                                            ///< @details   全局属性，对应 @ref UInt64ProfileValue
    MapAge                          = 43,   ///< @brief     地图图龄信息
                                            ///< @details   全局属性，对应 @ref UInt32ProfileValue
    MapProvider                     = 44,   ///< @brief     地图供应商
                                            ///< @note      预留，暂不实现
    MapStatus                       = 45,   ///< @brief     地图状态信息
                                            ///< @details   全局属性，对应 @ref MapStatusProfileValue
    SystemStatus                    = 46,   ///< @brief     系统状态
                                            ///< @note      预留，暂不实现
    TimeZoneOffset                  = 47,   ///< @brief     时区偏差
                                            ///< @note      预留，暂不实现
    AbsoluteVehiclePosition         = 48,   ///< @brief     车辆的绝对位置
                                            ///< @details   全局属性，对应 @ref AbsoluteVehiclePositionProfileValue
    AccessRestriction               = 49,   ///< @brief     通行限制
                                            ///< @note      预留，暂不实现
    OvertakingRestriction           = 50,   ///< @brief     超车限制
                                            ///< @note      预留，暂不实现
    Heading                         = 99,   ///< @brief     车道形点的航向
                                            ///< @details   对应 @ref OffsetFloatProfileValue
    RoadClass                       = 100,  ///< @brief     道路等级
                                            ///< @details   对应 @ref RoadClassProfileValue
    EffectiveMinSpeedLimit          = 101,  ///< @brief     最低限速
                                            ///< @details   对应 @ref EffectiveSpeedLimitProfileValue
    CrossSlope                      = 102,  ///< @brief     横向坡度
                                            ///< @details   对应 @ref OffsetFloatProfileValue
    LineMarkingType                 = 103,  ///< @brief     地面标线的类型
                                            ///< @details   对应 @ref LineMarkingProfileValue
    LineMarkingColor                = 104,  ///< @brief     地面标线的颜色
                                            ///< @details   对应 @ref LineColorProfileValue
    LineMarkingLogic                = 105,  ///< @brief     地面标线的逻辑属性
                                            ///< @details   对应 @ref LineLogicProfileValue
    RoadLevel                       = 106,  ///< @brief     道路层级
                                            ///< @details   对应 @ref RoadLevelProfileValue
    WholeRoute                      = 107,  ///< @brief     全局路径
                                            ///< @details   对应 @ref WholeRouteProfileValue
    AuxLaneInfo                     = 109,  ///< @brief     车道的扩展属性
                                            ///< @details   对应 @ref AuxLaneInfoProfileValue
    Passable                        = 110,  ///< @brief     可通行车道的信息
                                            ///< @details   对应 @ref BooleanProfileValue
    SameDirectionSections           = 111,  ///< @brief     同向道路信息
                                            ///< @details   对应 @ref SameDirectionSectionsProfileValue
    Road                            = 112,  ///< @brief     道路信息
                                            ///< @details   对应 @ref RoadProfileValue
    RoadWidth                       = 114,  ///< @brief     道路宽度
                                            ///< @details   对应 @ref RoadWidthProfileValue
    LineMarkingWidth                = 115,  ///< @brief     地面标线的宽度
                                            ///< @details   对应 @ref LineWidthProfileValue
    LineMarkingSource               = 116,  ///< @brief     地面标线的来源
                                            ///< @details   对应 @ref LineSourceProfileValue
    LicenseStatus                   = 117,  ///< @brief     许可证的状态信息
                                            ///< @details   全局属性，对应 @ref LicenseStatusProfileValue
    CoordinateSystem                = 118,  ///< @brief     坐标系统
                                            ///< @details   全局属性，用于描述当前使用的坐标系，对应 @ref CoordinateSystemProfileValue
    VersionSoftware                 = 119,  ///< @brief     软件版本信息
                                            ///< @note      全局属性，对应 @ref UInt64ProfileValue
    RoadSection                     = 120,  ///< @brief     道路区间
                                            ///< @details   对应 @ref RoadSectionProfileValue
    LineMarkingPhysicalCross        = 121,  ///< @brief     车道边线的物理可跨越性
                                            ///< @note      对应 @ref LinePhysicalCrossProfileValue
    LineMarkingFacilityType         = 122,  ///< @brief     车道边线的线上设施类型
                                            ///< @note      对应 @ref LineFacilityTypeProfileValue
    LineMarkingSpeedReduction       = 123,  ///< @brief     车道边线上的纵向减速标线
                                            ///< @note      对应 @ref LineSpeedReductionProfileValue
    DatumPoint                      = 254,  ///< @brief     基准点
                                            ///< @details   对应 @ref DatumPointProfileValue

    NA                              = 255,  ///< 无效
};

/**
 * @brief 定制消息类型
 */
enum class CustomizedType : uint8_t
{
    SDPathHasUpdate                 = 0,    ///< SD路径更新标志
    PathMatchInfo                   = 1,    ///< 配准结果
    NA                              = 255,  ///< 无效
};

/**
 * @brief 道路层级类型
 */
enum class ZLevelType : uint8_t
{
    Invalid             = 0,    ///< 未知
    Overground          = 1,    ///< 路面以上包含路路面
    ConnectOverground   = 2,    ///< 接驳上层
    Underground         = 3,    ///< 路面以下
    ConnectUnderground  = 4,    ///< 接驳下层
};

/**
 * @brief 变化模式
 */
enum class ChangeMode : uint8_t
{
    Create  = 0,    ///< 创建
    Update  = 1,    ///< 更新
    Delete  = 2,    ///< @brief 删除
                    ///< @note  预留，暂不实现
};

/**
 * @brief 相对方向
 */
enum class RelativeDirection : uint8_t
{
    None                    = 0,    ///< 未知
    Both                    = 1,    ///< @brief 双向
                                    ///< @note  预留，暂不实现
    AlongPathDirection      = 2,    ///< 和路径方向相同
    AgainstPathDirection    = 4,    ///< @brief 和路径方向相反
                                    ///< @note  预留，暂不实现
};

/**
 * @brief 有效性
 */
enum class Availability : uint8_t
{
    NotAvailable    = 0,    ///< @brief 无效
                            ///< @note  预留，暂不实现
    Valid           = 1,    ///< 有效
};

/**
 * @brief 路径类型
 */
enum class PathType : uint8_t
{
    Main    = 0,    ///< 主路径
    Out     = 1,    ///< 脱出子路径
    In      = 2,    ///< 汇入子路径
    NA      = 3,    ///< 无效
};

/**
 * @brief 转向信息
 */
enum class TurnInfo : uint8_t
{
    Unknown                         = 0,    ///< 未调查
    DivergenceStraight              = 1,    ///< 分歧路口直行
    DivergenceRightForwardTurn      = 2,    ///< 分歧路口右前转
    DivergenceRight                 = 3,   ///< 分歧路口右转
    DivergenceRightBackwardTurn     = 4,    ///< 分歧路口右后转
    DivergenceLeftForwardTurn       = 5,    ///< 分歧路口左前转
    DivergenceLeft                  = 6,    ///< 分歧路口左转
    DivergenceLeftBackwardTurn      = 7,    ///< 分歧路口左后转
    DivergenceOffsetStraight        = 8,    ///< 分歧路口错位直行
    DivergenceForbidTuring          = 9,    ///< 分歧路口转向禁止
    ConfluenceStraight              = 10,   ///< 合流直行路口区间
    ConfluenceRightForwardTurn      = 11,   ///< 合流路口右前转
    ConfluenceRight                 = 12,   ///< 合流路口右转
    ConfluenceRightBackwardTurn     = 13,   ///< 合流路口右后转
    ConfluenceLeftForwardTurn       = 14,   ///< 合流路口左前转
    ConfluenceLeft                  = 15,   ///< 合流路口左转
    ConfluenceLeftBackwardTurn      = 16,   ///< 合流路口左后转
    ConfluenceOffsetStraight        = 17,   ///< 合流路口错位直行
    ConfluenceForbidTuring          = 18,   ///< 合流路口转向禁止
    IntersectionStraight            = 19,   ///< 交叉路口直行区间
    IntersectionRightForwardTurn    = 20,   ///< 交叉路口右前转
    IntersectionRight               = 21,   ///< 交叉路口右转
    IntersectionRightBackwardTurn   = 22,   ///< 交叉路口右后转
    IntersectionLeftForwardTurn     = 23,   ///< 交叉路口左前转
    IntersectionLeft                = 24,   ///< 交叉路口左转
    IntersectionLeftBackwardTurn    = 25,   ///< 交叉路口左后转
    IntersectionOffsetStraight      = 26,   ///< 交叉路口错位直行
    IntersectionForbidTuring        = 27,   ///< 交叉路口转向禁止
    IntersectionUTurn               = 28,   ///< 交叉路口掉头区间
};

/**
 * @brief 车道类型
 */
enum class LaneType : uint8_t
{
    NonMotorVehicle = 0,    ///< 非机动车道
    BusOnly         = 1,    ///< 公交专用道
    CarpoolOnly     = 2,    ///< 多乘客专用道
    Tollgate        = 3,    ///< 收费站通道
    ETC             = 4,    ///< ETC 通道
    Emergency       = 5,    ///< 应急专用道
    Parking         = 6,    ///< 停车道
    Dummy           = 7,    ///< 虚拟车道
    Tidal           = 8,    ///< 潮汐车道
    CashExactChang  = 9,    ///< 人工收费车道
    Escape          = 10,   ///< 避险车道
    Climbing        = 11,   ///< 爬坡车道
    Waiting         = 12,   ///< 等待车道
    Common          = 13,   ///< 普通车道
    UTurnOnly       = 14,   ///< 掉头专用车道
    VariableGuide   = 15,   ///< 可变导向车道

    Entry           = 24,   ///< 入口车道
    Exit            = 25,   ///< 出口车道
    ExitOrEntry     = 26,   ///< 出口或入口车道
};

/**
 * @brief 过渡类型
 * @note  - Opening、Closing 属性判断的优先级高于合流、分歧（暂不考虑同时为开放车道、封闭车道的场景）
 *        - Splitting 优先级高于 Merging
 */
enum class LaneTransition : uint8_t
{
    None        = 0,    ///< 未知
    Opening     = 1,    ///< 开放车道
    Closing     = 2,    ///< 封闭车道
    Merging     = 3,    ///< 合流
    Splitting   = 4,    ///< 分歧
};

/**
 * @brief 车道连接关系
 */
enum class LaneConnectType : uint8_t
{
    None        = 0,    ///< 未知
    Split       = 1,    ///< 分歧
    Merge       = 2,    ///< 合流
    Continue    = 3,    ///< 保持直行
    SplitMerge  = 4,    ///< 分歧后合流
    Start       = 5,    ///< 开始（断头路起点）
    End         = 6,    ///< 结束（断头路终点）
    NA          = 255,  ///< 无效
};

/**
 * @brief 线型物体类型
 */
enum class LinearObjectType : uint8_t
{
    Centerline  = 0,    ///< 中心线
    LaneMarking = 1,    ///< 边线
    Guardrail   = 2,    ///< 护栏
    Fence       = 3,    ///< @brief 围栏、栅栏
                        ///< @note  预留，暂不实现
    Kerb        = 4,    ///< 路缘
    Wall        = 5,    ///< @brief 墙体
                        ///< @note  预留，暂不实现
    OutsideLine = 6,    ///< 道路外侧线
    NA          = 255,  ///< 无效
};

/**
 * @brief 线型物体的标记类型
 */
enum class LineMarking : uint8_t
{
    Unknown                 = 0,    ///< 未知
    None                    = 1,    ///< 无边线
    SolidLine               = 2,    ///< 单实线
    DashedLine              = 3,    ///< 单虚线
    DoubleSolidLine         = 4,    ///< 双实线
    DoubleDashedLine        = 5,    ///< 双虚线
    LeftSolidRightDashed    = 6,    ///< 左实右虚
    RightSolidLeftDashed    = 7,    ///< 左虚右实
    DashedBlocks            = 8,    ///< 虚状块
    ShadedArea              = 9,    ///< @brief 共享区域
                                    ///< @note  预留，暂不实现
    PhysicalDivider         = 10,   ///< @brief 物理隔离
                                    ///< @note  预留，暂不实现
    NA                      = 255,  ///< 无效
};

/**
 * @brief 线型物体的颜色
 */
enum class LineColor : uint8_t
{
    White       = 0,    ///< 白色
    LightGray   = 1,    ///< 浅灰
    Gray        = 2,    ///< 灰色
    DarkGray    = 3,    ///< 深灰
    Black       = 4,    ///< 黑色
    Red         = 5,    ///< 红色
    Yellow      = 6,    ///< 黄色
    Green       = 7,    ///< 绿色
    Cyan        = 8,    ///< 青色
    Blue        = 9,    ///< 蓝色
    Orange      = 10,   ///< 橙色
    Unknown     = 15,   ///< 未调查
};

/**
 * @brief 线型物体的逻辑类型
 */
enum class LineLogic : uint8_t
{
    Unknow              = 0,    ///< 未知
    Real                = 1,    ///< 真实边线
    DiversionGuide      = 2,    ///< 分歧路口导流线
    ConfluenceGuide     = 3,    ///< 汇流路口导流线
    LaneIncreaseGuide   = 4,    ///< 车道数增加导流线
    LaneDecreaseGuide   = 5,    ///< 车道数减少导流线
    LaneMissingZone     = 6,    ///< 车道线缺失区域
    SplitOutOfCollect   = 7,    ///< 收录范围外分割线
    LaneBreakZone       = 8,    ///< 车道线中断区
    LaneUnrecognized    = 9,    ///< 车道线无法识别
    LaneConn            = 10,   ///< 连接车道边线
    NA                  = 255,  ///< 无效
};

/**
 * @brief 线型物体的来源类型
 */
enum class LineSource : uint8_t
{
    Curb                    = 0,    ///< 路缘石
    GreenBelt               = 1,    ///< 绿化带
    Guardrail               = 2,    ///< 护栏
    GroundMarking           = 3,    ///< 地面印刷线
    DiversionZone           = 4,    ///< 导流线
    SourceVirtual           = 5,    ///< 虚拟
    Wall                    = 6,    ///< 墙壁
    Ditch                   = 7,    ///< 排水渠
    BarrierSound            = 8,    ///< 隔音棚
    Platform                = 9,    ///< 道外水泥台
    RoadMaterialBoundary    = 10,   ///< 道路材质铺设边界
    Fence                   = 11,   ///< 栅栏
    DikeDam                 = 12,   ///< 护堤
    Column                  = 13,   ///< 立柱
    Other                   = 15,   ///< 其他
    Unknown                 = 0xFF, ///< 未知
};

/**
 * @brief 道路侧边
 */
enum class RoadSide : uint8_t
{
    Unknown         = 0,   ///< 未知
    Left            = 1,   ///< 左侧
    Right           = 2,   ///< 右侧
    Overhead        = 3,   ///< 上方
    BothDirction    = 4,   ///< 双侧
    Surface         = 5,   ///< 表面
    NA              = 255, ///< 无效
};

/**
 * @brief 曲线类型
 */
enum CurveType : uint8_t
{
    NotPresent  = 0,    ///< 未描述
    Polyline    = 1,    ///< 多段线
    Bezier      = 2,    ///< @brief 贝塞尔曲线
                        ///< @note  预留，暂不实现
};

/**
 * @brief 道路类型
 */
enum class FormOfWay : uint8_t
{
    NoSpecial            = 0,    ///< 普通道路
    Ramp                 = 1,    ///< 匝道
    RoundAbout           = 2,    ///< 环岛
    Parallel             = 3,    ///< 平行于封闭道路
    ServiceRoad          = 4,    ///< 主辅连接路
    MainRoad             = 5,    ///< 主路
    Square               = 6,    ///< 未定义交通区域
    PedestrianZone       = 7,    ///< 园区内内部路
    Pedestrian           = 8,    ///< 步行街
    RoundAboutInterior   = 9,    ///< 环岛内连接路
    SlipRoad             = 10,   ///< 提前右转/左转/掉头专用道
    SpecialTrafficFigure = 11,   ///< 类似于环岛
    Boundary             = 12,   ///< 区域边界线（比如国界线，省界线）
    Unknown              = 255,  ///< 未调查
};

/**
 * @brief 道路属性
 */
enum class RoadAttribute : uint8_t
{
    ControlAccess       = 0,   ///< 封闭道路
    UTurn               = 1,   ///< 掉头路
    Parking             = 2,   ///< 园区内部路
    Elevated            = 3,   ///< 高架
    Underpass           = 4,   ///< 地下通道
    UnderConstruction   = 5,   ///< 在建道路
    MotorWay            = 6,   ///< 高速道路
    ServiceArea         = 7,   ///< 服务区道路
    Toll                = 8,   ///< 收费站道路
    ComplexIntersection = 9,   ///< 复杂交叉点
    PluralJunction      = 10,  ///< 复合路口
};

/**
 * @brief 道路等级
 */
enum class RoadClass : uint8_t
{
    Unknown         = 0,    ///< 未调查
    UrbanHighway    = 1,    ///< 都市间高速
    UrbanSealedRoad = 2,    ///< 都市内封闭路
    NationalRoad    = 3,    ///< 国道
    ProvincialRoad  = 4,    ///< 省道
    CountyRoad      = 5,    ///< 县道
    OrdinaryRoad    = 6,    ///< 一般道路
    NarrowRoad      = 7,    ///< 细道路
};

/**
 * @brief 连接匝道类型
 */
enum class ConnectRampType : uint8_t
{
    ICIn    = 0,    ///< 连接上匝道（驶入）
    ICOut   = 1,    ///< 连接下匝道（驶出）
    JCIn    = 2,    ///< 高速间匝道（驶入）
    JCOut   = 3,    ///< 高速间匝道（驶出）
};

/**
 * @brief 路径点的类型
 */
enum class NaviPointType : uint8_t
{
    Start   = 0,    ///< 起点
    Passby  = 1,    ///< 途经点
    Dest    = 2,    ///< 目的点
};

/**
 * @brief 标志牌大类
 */
enum class SignType : uint8_t
{
    Danger                                          = 0,    ///< 危险
    PassLeftOrRightSide                             = 1,    ///< 左右侧通过
    PassLeftSide                                    = 2,    ///< 左侧通过
    PassRightSide                                   = 3,    ///< 右侧通过
    DomesticAnimalsCrossing                         = 4,    ///< 注意驯化动物穿行
    WildAnimalsCrossing                             = 5,    ///< 注意野生动物穿行
    RoadWorks                                       = 6,    ///< 施工道路
    ResidentialArea                                 = 7,    ///< 住宅区域
    EndOfResidentialArea                            = 8,    ///< 住宅区域结束
    RightBend                                       = 9,    ///< 右转弯
    LeftBend                                        = 10,   ///< 左转弯
    DoubleBendRightFirst                            = 13,   ///< 双向转弯-右侧优先
    DoubleBendLeftFirst                             = 14,   ///< 双向转弯-左侧优先
    CurvyRoad                                       = 17,   ///< 连续弯路
    OvertakingByGoodsVehiclesProhibited             = 20,   ///< 禁止货运车辆超车
    EndOfProhibitionOnOvertakingForGoodsVehicles    = 21,   ///< 解除货运车辆超车禁止
    DangerousIntersection                           = 22,   ///< 危险交叉路口
    Tunnel                                          = 24,   ///< 隧道
    FerryTerminal                                   = 25,   ///< 轮渡码头
    NarrowBridge                                    = 26,   ///< 窄桥
    HumpbackBridge                                  = 28,   ///< 拱桥
    RiverBank                                       = 29,   ///< 河堤
    RiverBankLeft                                   = 30,   ///< 左侧河堤
    LightSignals                                    = 31,   ///< 信号灯
    GiveWay                                         = 32,   ///< 让行
    Stop                                            = 33,   ///< 停车
    PriorityRoad                                    = 34,   ///< 优先行驶
    Intersection                                    = 35,   ///< 交叉路口
    IntersectionWithMinorRoad                       = 36,   ///< 支路优先路口
    IntersectionWithPriorityToTheRight              = 37,   ///< 右侧优先路口
    DirectionToTheRight                             = 38,   ///< 通向右方道路
    DirectionToTheLeft                              = 39,   ///< 通行左方道路
    CarriageWayNarrows                              = 40,   ///< 车道变窄
    CarriageWayNarrowsRight                         = 41,   ///< 右侧车道变窄
    CarriageWayNarrowsLeft                          = 42,   ///< 左侧车道变窄
    LaneMergeLeft                                   = 43,   ///< 左侧合流
    LaneMergeRight                                  = 44,   ///< 右侧合流
    LaneMergeCenter                                 = 45,   ///< 中间合流
    OvertakingProhibited                            = 46,   ///< 禁止超车
    EndOfProhibitionOnOvertaking                    = 47,   ///< 解除超车禁止
    ProtectedPassingEnd                             = 48,   ///< 解除通行受限
    Pedestrians                                     = 50,   ///< 注意行人
    PedestrianCrossing                              = 51,   ///< 人行横道
    Children                                        = 52,   ///< 注意儿童
    SchoolZone                                      = 53,   ///< 学校区域
    Cyclists                                        = 54,   ///< 注意自行车
    TwoWayTraffic                                   = 55,   ///< 双向行驶
    RailwayCrossingWithGates                        = 56,   ///< 有人看守铁路道口
    RailwayCrossingWithoutGates                     = 57,   ///< 无人看守铁路道口
    RailwayCrossing                                 = 58,   ///< 铁路通行
    TramWay                                         = 59,   ///< 电车轨道
    FallingRocksLeft                                = 61,   ///< 左侧落石
    FallingRocksRight                               = 62,   ///< 右侧落石
    SteepDropLeft                                   = 63,   ///< 左侧急剧下降
    SteepDropRight                                  = 64,   ///< 右侧急剧下降
    VariableSignMechanicElements                    = 65,   ///< 可变交通标记
    SlipperyRoad                                    = 66,   ///< 易滑路段
    SteepAscent                                     = 67,   ///< 险升坡
    SteepDescent                                    = 68,   ///< 陡降坡
    UnevenRoad                                      = 69,   ///< 不平路面
    Hump                                            = 70,   ///< 驼峰路
    Dip                                             = 71,   ///< 低洼处（道路陡降然后上升）
    RoadFloods                                      = 72,   ///< 洪水
    IcyRoad                                         = 73,   ///< 冰封道路
    SideWinds                                       = 74,   ///< 注意横风
    TafficCongestion                                = 75,   ///< 交通拥挤
    HighAccidentArea                                = 76,   ///< 事故高发区域
    BeginningOfBuiltUpArea                          = 77,   ///< 建筑区域开始
    AudibleWarning                                  = 78,   ///< 鸣笛提醒
    EndOfAllProhibitions                            = 79,   ///< 解除所有禁止
    VariableSignLightElements                       = 80,   ///< 可变信号灯
    PriorityOverOncomingTraffic                     = 81,   ///< 优先于对向道路
    PriorityForOncomingTraffic                      = 82,   ///< 对向优先
    EndOfBuiltUpArea                                = 83,   ///< 建筑区域结束
    SpeedLimit                                      = 87,   ///< 限速
    EndOfSpeedLimit                                 = 88,   ///< 解除限速
    SwingBridge                                     = 89,   ///< 平旋/转桥
    Unknown                                         = 255,  ///< 未知
};

/**
 * @brief 横向位置
 */
enum class LateralPositionFlag : uint32_t
{
    Unknown   = 0,  ///< 未知
    Right     = 1,  ///< 右侧
    Left      = 2,  ///< 左侧
    Above     = 3,  ///< 上方
    Surface   = 4,  ///< 路面
};

/**
 * @brief 交通信号灯类型
 */
enum class TrafficLightType : uint8_t
{
    Intersection                        = 0,    ///< 道口信号灯
    RampMeter                           = 1,    ///< 匝道信号灯
    TollBooth                           = 2,    ///< 收费站信号灯
    PedestrianCrossingForPedestrians    = 3,    ///< 人行横道行人型号等
    PedestrianCrossingForVehicles       = 4,    ///< 人行横道机动车信号灯
    BicycleCrossing                     = 5,    ///< 非机动车道信号灯
    Tunnel                              = 6,    ///< 隧道信号灯
    Bridge                              = 7,    ///< 桥梁信号灯
    LaneControl                         = 8,    ///< 车道管制信号灯
    Tailway                             = 9,    ///< 铁路信号灯
    Tram                                = 10,   ///< 有轨电车信号灯
    DirectionIndicate                   = 11,   ///< 方向指示信号灯
    FlashingWarning                     = 12,   ///< 闪光警告灯
    NumericalTrafficLight               = 13,   ///< 数字信号灯
    Unknown                             = 255,  ///< 未知
};

/**
 * @brief 交通信号灯的排列方式
 */
enum class TrafficLightConstructionType : uint8_t
{
    Horizontal    = 0,    ///< 水平
    Vertical      = 1,    ///< 竖直
    Doghouse      = 2,    ///< 狗屋式
    Unknown       = 3,    ///< 未调查
};

/**
 * @brief 特殊场景类型
 */
enum class SpecialSituationType : uint16_t
{
    Unknown                 = 0,    ///< 未知
    DeadEnd                 = 248,  ///< @brief 死胡同
                                    ///< @note  预留，暂不实现
    FerryTerminal           = 249,  ///< @brief 轮渡码头
                                    ///< @note  预留，暂不实现
    TollBooth               = 250,  ///< 收费站
    RailroadCrossing        = 251,  ///< @brief 铁路道口
                                    ///< @note  预留，暂不实现
    PedestrianCrossing      = 252,  ///< 人行横道
    SpeedBump               = 253,  ///< 减速带
    StopLine                = 261,  ///< 停止线
    Gate                    = 262,  ///< 闸机
    Destination             = 263,  ///< 目的地
    ParkingSpace            = 264,  ///< 停车位
    LaneWidthStablePoint    = 265,  ///< 车道宽度变化稳定点

    ICSplit                 = 266,  ///< IC 分歧
    ICMerge                 = 267,  ///< IC 汇流
    JCSplit                 = 268,  ///< JC 分歧
    JCMerge                 = 269,  ///< JC 汇流
    RoadSplit               = 270,  ///< 普通车道分歧
    RoadMerge               = 271,  ///< 普通车道汇流
};

/**
 * @brief 速度单位
 */
enum class SpeedUnit : uint8_t
{
    KpH = 0,    ///< 千米每小时
    MpH = 1,    ///< @brief 英里每小时
                ///< @note  预留，暂不实现
};

/**
 * @brief 限速类型
 */
enum class EffectiveSpeedLimitType : uint8_t
{
    Unknown               = 0, ///< 未知
    Implicit              = 1, ///< 隐性的限速
    ExplicitOnTrafficSign = 2, ///< 交通标志牌上的限速
    ExplicitNight         = 3, ///< 晚间限速
    ExplicitDay           = 4, ///< 白天限速
    ExplicitTimeOrDay     = 5, ///< 时间或者白天限速
    ExplicitRain          = 6, ///< 雨天限速
    ExplicitSnow          = 7, ///< 雪天限速
    ExplicitFog           = 8, ///< 雾天限速
};

/**
 * @brief 加速/减速车道
 */
enum class AuxLaneTypeFlag : uint8_t
{
    SpeedUp         = 0,    ///< 加速车道
    SpeedDown       = 1,    ///< 减速车道
    SpeedUpOrDown   = 2,    ///< 加速或减速车道
};

/**
 * @brief 驾驶方向
 */
enum class DrivingSide : uint8_t
{
    RightHandDriving  = 0,    ///< 右侧行驶
    LeftHandDriving   = 1,    ///< 左侧行驶
    Unknown           = 127,  ///< 未知
};

/**
 * @brief 地图状态
 */
enum class MapStatus : uint8_t
{
    MapNotAvailable = 0,    ///< 地图无效
    MapLoading      = 1,    ///< @brief 地图加载中
                            ///< @note  预留，暂不实现
    MapAvailable    = 2,    ///< 地图有效
};

/**
 * @brief 许可证的状态
 */
enum class LicenseStatus : uint8_t
{
    Unknown     = 0,    ///< 未知
    Expired     = 1,    ///< 已过期
    Available   = 2,    ///< 未过期
    VerifyError = 3,    ///< 验证出错
};

/**
 * @brief 车种类型
 */
enum class VehicleType : uint8_t
{
    None           = 0,   ///< 不限制车种
    PersonalCar    = 1,   ///< 小客车/小汽车
    Taxi           = 2,   ///< 出租车
    Bus            = 3,   ///< 公交车
    MotorBus       = 4,   ///< 大客车
    Buggy          = 5,   ///< 小货车/小卡车
    BigTruck       = 6,   ///< 大货车/大卡车
    DangerousGoods = 7,   ///< 危险物品车
    Emergency      = 8,   ///< 急救车
    Trailer        = 9,   ///< 拖/挂车
    Tractor        = 10,  ///< 三轮车
    Motorcycle     = 11,  ///< 摩托车
    ElecBicycle    = 12,  ///< 电动自行车
    Bicycle        = 14 , ///< 自行车/人力车
    Tram           = 15,  ///< 有轨车
    TinyCar        = 16,  ///< 微型车
};

/**
 * @brief 坐标系统
 */
enum class CoordinateSystem : uint8_t
{
    Unknown = 0,    ///< 未知
    WGS84   = 1,    ///< @brief     WGS84 坐标系
                    ///< @details   如只是经过平移但表达方式未发生变更的（国测局偏转、中海庭偏转），仍表示为wgs84坐标系
    ENU     = 2,    ///< 站心地平直角（东-北-天）坐标系
};

/**
 * @brief 地理围栏的状态
 */
enum class ODDStatusType
{
    Unknown     = 0,    ///< 未知，即无法判断是在地理围栏内（通常是没有对应的数据支撑）
    InRange     = 1,    ///< 在地理围栏范围内（刚好在边界也认为在范围内）
    OutRange    = 2,    ///< 不在地理围栏范围内
};

/**
 * @brief 物理可跨越性
 */
enum class PhysicalCrossType
{
    Unknown     = 0,    ///< 未知
    NoFacility  = 1,    ///< 无物理设施
    CanCross    = 2,    ///< 可跨越
    CanNotCross = 3,    ///< 不可跨越
};

/**
 * @brief 车道边线的线上设施类型
 */
enum class LaneBoundaryFacilityType
{
    Plane          = 0, ///< 平面
    Guardrail      = 1, ///< 护栏
    Spike          = 2, ///< 道钉
    VibrationBulge = 3, ///< 振动凸起
    Curb           = 4, ///< 路缘石
    UprightColumn  = 5, ///< 立柱
    Other          = 6, ///< 其他
};

/**
 * @brief 线的相对位置
 */
enum class LineSide
{
    Unkown      = 0,    ///< 未调查
    Right       = 1,    ///< 右侧
    Left        = 2,    ///< 左侧
    Both        = 3,    ///< 两侧
    Inside      = 4,    ///< @brief  里面
                        ///< @note   预留，暂不实现
    Overhead    = 5,    ///< 上方
                        ///< @note   预留，暂不实现
    RoadSurface = 6,    ///< 道路表面
                        ///< @note   预留，暂不实现
};

/**
 * @brief sd路径配准状态
 */
enum class PathMatchStatus : uint8_t
{
    Success             = 0,    ///< 配准成功，中间某段失败归为成功
    FirstSegmentFailed  = 1,    ///< 首段失败
    AllFailed           = 2,    ///< 全部失败

    Na                  = 255,  ///< 未知
};


/**
 * @brief 处理结果
 */
enum class Result
{
    Succeed         = 0,    ///< 处理成功
    Failed          = 1,    ///< 处理失败
    NotSupported    = 2,    ///< 不支持
    InternalError   = 3,    ///< 内部错误
    NotInitialized  = 4,    ///< 实体未初始化
    ArgumentError   = 5,    ///< 参数错误

    NoPosition      = 6,    ///< 没有收到位置信号
    NotMatched      = 7,    ///< 没有匹配成功，可能没有地图信息或者不在路网上
    Deviated        = 8,    ///< 位置不在道路上，有一定的偏离
    NotPlanned      = 9,    ///< 没有规划数据
    NotMatchDatum   = 10,   ///< 没有匹配基准点的图层数据
    PosIsExpired    = 11,   ///< 车辆位置数据已失效
};

} // namespace decision_map

} // namespace hdi
