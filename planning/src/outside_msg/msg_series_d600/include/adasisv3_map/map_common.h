#ifndef MAP_COMMON_H
#define MAP_COMMON_H

#include<vector>
#include"Common.h"
#include"map_control.h"
//typedef signed long long  int64_t;
//typedef unsigned long long uint64_t;
typedef unsigned int uint32_t;
typedef signed int int32_t;
typedef double float_64;
typedef float float_32;
namespace MAP{

#define MAP_LaneInfo_N          "map_laneinfo"
#define MAP_Sign_N              "map_sign"
#define MAP_Routing_N           "map_routing"
#define MAP_RoadBoundary_N      "map_roadboundary"
#define MAP_SpecialSituation_N  "map_specialsituation"
#define MAP_Location_N          "map_location"
#define MAP_SensorData_N        "map_sensordata"
#define MAP_Pole_N              "map_pole"
#define MAP_MessageControl_Position_Rate 20             //Hz
#define MAP_MessageControl_Profile_Rate  5
#define MAP_MessageControl_Position_Duration 0.06       //s
#define MAP_MessageControl_Profile_Duration  0.24

///
/// \brief WGS84坐标
///
struct WGS84Point
{
    float_64 Longitude;                 //经度,单位:度
    float_64 Latitude;                  //纬度,单位:度
    float_32 Altitude;                  //高程,单位:m

    WGS84Point()
    {
        Longitude = -1;
        Latitude  = -1;
        Altitude  =  0;
    }

    WGS84Point(const int32_t lon, const int32_t lat, const int32_t alt)
    {
        Longitude = lon * 0.0000001;    //尺度转换 1e-7
        Latitude = lat * 0.0000001;     //尺度转换 1e-7
        Altitude = alt * 0.01;          //cm 转 m
    }
};

///
/// \brief 车道线类型
///
enum LINE_TYPE{
    LINE_TYPE_UNKNOW,                           //Unknow
    LINE_TYPE_SINGLE_DOTTED,                    //单虚线
    LINE_TYPE_SINGLE_SOLID,                     //单实线
    LINE_TYPE_DOUBLE_SOLID,                     //双实线
    LINE_TYPE_LEFT_SOLID_RIGHT_DOTTED,          //双线（左实右虚）
    LINE_TYPE_LEFT_DOTTED_RIGHT_SOLID,          //双线（左虚右实）
    LINE_TYPE_DOUBLE_DOTTED,                    //双虚线
    LINE_TYPE_DIVERTION,                        //
    LINE_TYPE_INTERSECTION_VIRTUAL_DIVIDER,     //
    LINE_TYPE_OTHER=127                         //其他
};

///
/// \brief 车道属性控制点
///
struct Lane_Control
{
    float_32 dist;                  //距离起点的距离，单位：m
    float_32 end_dist;              //控制范围结束位置距离起点的距离，单位：m
    float_32 value;                 //属性值
};

///
/// \brief 道路类型
///
enum ROAD_TYPE{
    ROAD_TYPE_UNKNOWN=0,            //Unknown
    ROAD_TYPE_ROAD=1,               //本线
    ROAD_TYPE_JC=3,                 //JC路（高速连接）
    ROAD_TYPE_CROSS_DUMMY=8,        //交叉路口虚拟连接路
    ROAD_TYPE_PARK=10,              //停车区路
    ROAD_TYPE_INNER_ROAD=18,        //园区内内部路
    ROAD_TYPE_INNER_INTERSECTION=21,//路口内部路
    ROAD_TYPE_INNER_TUNNEL=22,      //隧道内道路
    ROAD_TYPE_BRIDGE_CROSS=23,      //桥梁跨越道路
    ROAD_TYPE_NA =255               //无效值
};

///
/// \brief 车辆可能在的车道信息
///
struct Position
{
    uint8_t m_iPathid;              //车道所在路径ID
    float_32 m_iOffset;             //车辆实时相对Path起点的Offset，单位： m
    uint8_t m_iCurrentlane;         //车辆所在车道的编号，从右往左,从1开始
    float_32 m_speed;               //车辆速度,单位: m/s
    float_32 m_iRelativeHeading;    //车辆与车道的相对航向，单位：度，以车道中心线方向开始逆时针增加
    float_32 m_iProbability;        //概率/置信度
};

///
/// \brief 标牌、减速带等类型
///
enum OBJECT_TYPE{
    Speedbump,          //减速带
};

///
/// \brief 线类型
///
enum GEOMETRY_TYPE{
    NotPresent,
    Polyline,           //折线
    BezierSpline        //贝赛尔曲线
};

///
/// \brief 车道方向
///
enum LANE_DIRECTION{
    None,                       //无
    Both,                       //双向
    AlongDrivingDirection,      //与行驶方向相同
    AgainstDrivingDirection     //与行驶方向相反
};

///
/// \brief 车道类型
///
enum LANE_TYPE{
    LANE_TYPE_UNKNOWN=0,                //Unknown
    LANE_TYPE_NORMAL=1,                 //普通车道
    LANE_TYPE_ETC=8,                    //ETC 车道
    LANE_TYPE_CHARGE=9,                 //收费通道
    LANE_TYPE_BUS_ONLY=12,              //BUS 专用道
    LANE_TYPE_DUMMY=13,                 //DUMMY 车道
    LANE_TYPE_EMERGENCY_DRIVEWAY=14,    //应急车道
    LANE_TYPE_NA=127                    //N/A
};

///
/// \brief 车道连接关系
///
struct Lane_ConnectivityPair
{
    uint64_t  InitialLaneNumber;        //当前的车道编号
    uint32_t  InitialPath;              //当前的路径编号
    uint64_t  NewLaneNumber;            //脱出的路径编号
    uint32_t  NewPath;                  //脱出的车道编号
    enum TURN_TYPE{                     //转向信息
        TURN_INFO_UNKNOWN,              //Unknow
        TURN_INFO_TURN_STRAIGHT,        //直行
        TURN_INFO_TURN_LEFT,            //左转
        TURN_INFO_TURN_RIGHT,           //右转
        TURN_INFO_UTURN,                //掉头
        TURN_INFO_NA=127                //不涉及
    }m_eTurnInfo;                       //路口转向信息
};

//-------------------------

///
/// \brief 车道属性
///
struct MAP_LaneInfo
{
    uint64_t base_lane_id;                  //基准车道
    uint64_t left_lane_id;                  //左侧车道
    uint64_t right_lane_id;                 //右侧车道
    std::vector<uint64_t> next_lane_id;     //连接车道集合
    uint32_t pathId;                        //车道所在路径ID
    float_32 offset;                        //车辆实时相对Path起点的Offset，单位：m
    float_32 end_offset;                            //m
    uint8_t lanenumber;                     //车道的编号，从右往左，从1开始
    uint8_t lanes_count;                    //车道所属道路的车道总数
    enum LINE_TYPE left_line_type;          //左侧车道线类型
    enum LINE_TYPE right_line_type;         //右侧车道线类型
    LANE_DIRECTION direction;               //车道方向
    std::vector<WGS84Point> center_points;  //中心线形点
    std::vector<WGS84Point> left_line;      //左侧车道线形点
    std::vector<WGS84Point> right_line;     //右侧车道线形点
    int32_t speed_limit;                    //车道限速
    float_32 length;                        //车道长度
    WGS84Point start_point;                 //车道起点
    WGS84Point end_point;                   //车道终点
    std::vector<Lane_Control> curvature;    //车道曲率
    std::vector<Lane_Control> slope;        //车道纵向坡度
    ROAD_TYPE road_type;                    //道路类型
    LANE_TYPE lane_type;                    //车道类型
    std::vector<uint64_t> lane_ids;         //车道所属道路的车道ID集合
};

///
/// \brief 定位信息
///
struct MAP_Location
{
    task_t::S_BASE base;                    //消息头
    uint64_t timestamp;                     //时间戳
    WGS84Point location;                    //坐标点
    float_32 heading;                       //车辆航向角（正北为0，顺时增加），单位：度
    float_32 current_lane_width;            //当前车道（置信度最高的车道）宽度，单位：m
    float_32 left_lane_width;               //左侧车道宽度，单位：m
    float_32 right_lane_width;              //右侧车道宽度，单位：m
    std::vector<Position> position_vec;     //车辆可能在的车道信息集合
};

///
/// \brief 道路边界线形点
///
struct MAP_RoadBoundary
{
    uint32_t pathid;                        //所在路径ID
    std::vector<WGS84Point> left_boundary;  //左侧道路边线形点集合
    std::vector<WGS84Point> right_boundary; //右侧道路边线形点集合
};

///
/// \brief 减速带 标志牌 地面标记等定位地物属性
///
struct MAP_ObjectOrMarking
{
    enum OBJECT_TYPE object_type;           //属性类型
    uint32_t offset;                        //起始位置,单位: m
    WGS84Point center_location;             //所在坐标
    enum GEOMETRY_TYPE geometry_type;       //线类型
    std::vector<WGS84Point> geometry;       //轮廓形点描述
};

///
/// \brief 发送的连接关系数据
///
struct MAP_Routing
{
    task_t::S_BASE base;                                //消息头
    std::vector<Lane_ConnectivityPair> lane_connect;    //连接关系集合
};

///
/// \brief 发送的传感器数据
///
struct MAP_SensorData
{
    task_t::S_BASE base;        //消息头
    float_64 Time;              //时间戳
    float_64 Latitude;          //纬度
    float_64 Longitude;         //经度
    float_32 Height;            //高程
    float_32 roll;
    float_32 pitch;
    float_32 yaw;
    float_32 velocity_x;        //线速度
    float_32 velocity_y;
    float_32 velocity_z;
    float_32 acceleration_x;    //加速度
    float_32 acceleration_y;
    float_32 acceleration_z;
    float_32 roll_rate;         //角速度
    float_32 pitch_rate;
    float_32 yaw_rate;
};

///
/// \brief 杆状物属性
///
struct MAP_Pole
{
    uint64_t poleID;                        //所在路径ID
    WGS84Point position;                    //所在位置信息
    std::vector<WGS84Point> geometry;       //不确定提供
    float_32 height;                        //不确定提供
    float_32 width;                         //不确定提供
};

//-----------------------

///
/// \brief 发送的地图数据
///
struct MAP_VecLaneInfo
{
    task_t::S_BASE base;                    //消息头
    std::vector<MAP_LaneInfo> data;         //地图数据集合
};

///
/// \brief 发送的地面标记、标志牌等消息
///
struct MAP_VecObjectOrMarking
{
    task_t::S_BASE base;                    //消息头
    std::vector<MAP_ObjectOrMarking> data;  //数据
};

///
/// \brief 发送的道路边界线消息
///
struct MAP_VecRoadBoundary
{
    task_t::S_BASE base;                    //消息头
    std::vector<MAP_RoadBoundary> data;
};

///
/// \brief 发送的杆状物消息
///
struct MAP_PoleVec
{
    task_t::S_BASE base;                    //消息头
    std::vector<MAP_Pole> data;
};

}
#endif
