/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       gps_tools.cc
 * @brief      GPS坐标系转UTM坐标系
 * @details    实现了WGS84格式的GPS坐标系与UTM坐标系之间互相转换的函数
 *
 * @author     boc
 * @date       2020.07.02
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/07/02  <td>1.0      <td>boc       <td>First edition
 * </table>
 *
 ******************************************************************************/

#include <stdlib.h>
#include "utils/gps_tools.h"
#include "math/math_utils.h"
#include "utils/com_utils.h"
#include "utils/log.h"

// WGS84 Parameters
#define WGS84_A   6378137.0   // major axis
#define WGS84_B   6356752.31424518  // minor axis
#define WGS84_F   0.0033528107    // ellipsoid flattening
#define WGS84_E   0.0818191908    // first eccentricity
#define WGS84_EP  0.0820944379    // second eccentricity

// UTM Parameters
#define UTM_K0    0.9996               // scale factor
#define UTM_FE    500000.0             // false easting
#define UTM_FN_N  0.0                  // false northing, northern hemisphere
#define UTM_FN_S  10000000.0           // false northing, southern hemisphere
#define UTM_E2    (WGS84_E*WGS84_E)    // e^2
#define UTM_E4    (UTM_E2*UTM_E2)      // e^4
#define UTM_E6    (UTM_E4*UTM_E2)      // e^6
#define UTM_EP2   (UTM_E2/(1-UTM_E2))  // e'^2

namespace phoenix {
namespace common {

struct CalcDistParm {
  Float64_t dist_per_period;
  Float64_t lon_parm;
  Float64_t lat_parm;

  void Clear() {
    dist_per_period = 0.0;
    lon_parm = 38.1096;
    lat_parm = 33.2575;
  }

  CalcDistParm() {
    Clear();
  }
};

enum { LATITUDE_DIST_PER_LAT_TABLE_SIZE = 540 };
Float32_t s_lon_dist_per_lat[LATITUDE_DIST_PER_LAT_TABLE_SIZE] = {
  1.0067426F,1.0067362F,1.0067230F,1.0067003F,1.0066710F,1.0066286F,
  1.0065798F,1.0065211F,1.0064560F,1.0063812F,1.0062965F,1.0062053F,
  1.0061044F,1.0059969F,1.0058764F,1.0057495F,1.0056159F,1.0054727F,
  1.0053164F,1.0051568F,1.0049907F,1.0048084F,1.0046196F,1.0044274F,
  1.0042224F,1.0040075F,1.0037893F,1.0035582F,1.0033205F,1.0030731F,
  1.0028158F,1.0025522F,1.0022787F,1.0019988F,1.0017090F,1.0014063F,
  1.0011003F,1.0007877F,1.0004590F,1.0001270F,0.9997884F,0.9994369F,
  0.9990756F,0.9987110F,0.9983335F,0.9979494F,0.9975588F,0.9971552F,
  0.9967451F,0.9963285F,0.9958989F,0.9954661F,0.9950202F,0.9945679F,
  0.9941058F,0.9936371F,0.9931588F,0.9926739F,0.9921761F,0.9916717F,
  0.9911576F,0.9906402F,0.9901099F,0.9895730F,0.9890264F,0.9884701F,
  0.9879072F,0.9873379F,0.9867588F,0.9861699F,0.9855714F,0.9849663F,
  0.9843547F,0.9837302F,0.9831024F,0.9824617F,0.9818144F,0.9811574F,
  0.9804971F,0.9798207F,0.9791409F,0.9784515F,0.9777555F,0.9770466F,
  0.9763344F,0.9756125F,0.9748809F,0.9741427F,0.9733949F,0.9726374F,
  0.9718733F,0.9710996F,0.9703193F,0.9695293F,0.9687328F,0.9679266F,
  0.9671139F,0.9662915F,0.9654626F,0.9646208F,0.9637758F,0.9629209F,
  0.9620565F,0.9611855F,0.9603048F,0.9594176F,0.9585208F,0.9576206F,
  0.9567043F,0.9557848F,0.9548587F,0.9539198F,0.9529743F,0.9520192F,
  0.9510576F,0.9500896F,0.9491118F,0.9481276F,0.9471336F,0.9461300F,
  0.9451231F,0.9441032F,0.9430802F,0.9420443F,0.9410019F,0.9399529F,
  0.9388976F,0.9378294F,0.9367545F,0.9356735F,0.9345858F,0.9334853F,
  0.9323813F,0.9312680F,0.9301448F,0.9290151F,0.9278791F,0.9267334F,
  0.9255781F,0.9244192F,0.9232509F,0.9220729F,0.9208885F,0.9196976F,
  0.9184971F,0.9172900F,0.9160733F,0.9148502F,0.9136206F,0.9123813F,
  0.9111326F,0.9098772F,0.9086154F,0.9073471F,0.9060661F,0.9047817F,
  0.9034908F,0.9021873F,0.9008804F,0.8995640F,0.8982409F,0.8969085F,
  0.8955725F,0.8942240F,0.8928719F,0.8915105F,0.8901394F,0.8887647F,
  0.8873808F,0.8859904F,0.8845904F,0.8831835F,0.8817707F,0.8803481F,
  0.8789191F,0.8774838F,0.8760387F,0.8745872F,0.8731293F,0.8716617F,
  0.8701878F,0.8687074F,0.8672205F,0.8657273F,0.8642244F,0.8627151F,
  0.8611962F,0.8596712F,0.8581395F,0.8566012F,0.8550566F,0.8534996F,
  0.8519389F,0.8503718F,0.8487955F,0.8472157F,0.8456261F,0.8440274F,
  0.8424250F,0.8408135F,0.8391984F,0.8375708F,0.8359395F,0.8342992F,
  0.8326552F,0.8310021F,0.8293419F,0.8276761F,0.8260032F,0.8243212F,
  0.8226328F,0.8209376F,0.8192364F,0.8175257F,0.8158144F,0.8140908F,
  0.8123609F,0.8106272F,0.8088813F,0.8071322F,0.8053767F,0.8036116F,
  0.8018427F,0.8000681F,0.7982839F,0.7964932F,0.7946962F,0.7928928F,
  0.7910823F,0.7892662F,0.7874436F,0.7856115F,0.7837762F,0.7819345F,
  0.7800832F,0.7782255F,0.7763647F,0.7744943F,0.7726175F,0.7707376F,
  0.7688505F,0.7669546F,0.7650523F,0.7631469F,0.7612318F,0.7593105F,
  0.7573827F,0.7554518F,0.7535113F,0.7515644F,0.7496111F,0.7476524F,
  0.7456864F,0.7437173F,0.7417386F,0.7397568F,0.7377653F,0.7357708F,
  0.7337698F,0.7317593F,0.7297457F,0.7277257F,0.7256993F,0.7236665F,
  0.7216307F,0.7195853F,0.7175335F,0.7154785F,0.7134172F,0.7113496F,
  0.7092724F,0.7071953F,0.7051086F,0.7030155F,0.7009193F,0.6988136F,
  0.6967047F,0.6945894F,0.6924679F,0.6903431F,0.6882088F,0.6860715F,
  0.6839277F,0.6817776F,0.6796211F,0.6774616F,0.6752924F,0.6731201F,
  0.6709448F,0.6687598F,0.6665717F,0.6643741F,0.6621765F,0.6599694F,
  0.6577592F,0.6555427F,0.6533219F,0.6510926F,0.6488603F,0.6466216F,
  0.6443766F,0.6421284F,0.6398760F,0.6376152F,0.6353512F,0.6330810F,
  0.6308064F,0.6285267F,0.6262374F,0.6239482F,0.6216515F,0.6193497F,
  0.6170447F,0.6147322F,0.6124146F,0.6100939F,0.6077688F,0.6054354F,
  0.6031009F,0.6007581F,0.5984108F,0.5960585F,0.5937019F,0.5913402F,
  0.5889741F,0.5866029F,0.5842274F,0.5818436F,0.5794585F,0.5770704F,
  0.5746741F,0.5722733F,0.5698694F,0.5674573F,0.5650439F,0.5626243F,
  0.5602015F,0.5577738F,0.5553415F,0.5529029F,0.5504612F,0.5480164F,
  0.5455653F,0.5431079F,0.5406505F,0.5381836F,0.5357168F,0.5332437F,
  0.5307643F,0.5282817F,0.5257961F,0.5233040F,0.5208089F,0.5183108F,
  0.5158079F,0.5133002F,0.5107863F,0.5082693F,0.5057507F,0.5032242F,
  0.5006946F,0.4981635F,0.4956244F,0.4930871F,0.4905386F,0.4879918F,
  0.4854372F,0.4828809F,0.4803183F,0.4777544F,0.4751855F,0.4726136F,
  0.4700339F,0.4674557F,0.4648713F,0.4622805F,0.4596898F,0.4570928F,
  0.4544944F,0.4518879F,0.4492815F,0.4466721F,0.4440577F,0.4414388F,
  0.4388167F,0.4361916F,0.4335633F,0.4309287F,0.4282956F,0.4256548F,
  0.4230109F,0.4203652F,0.4177150F,0.4150631F,0.4124067F,0.4097453F,
  0.4070827F,0.4044151F,0.4017430F,0.3990691F,0.3963954F,0.3937140F,
  0.3910307F,0.3883443F,0.3856549F,0.3829623F,0.3802665F,0.3775676F,
  0.3748657F,0.3721606F,0.3694523F,0.3667410F,0.3640265F,0.3613101F,
  0.3585894F,0.3558655F,0.3531386F,0.3504096F,0.3476764F,0.3449444F,
  0.3422049F,0.3394667F,0.3367210F,0.3339764F,0.3312288F,0.3284769F,
  0.3257230F,0.3229659F,0.3202057F,0.3174456F,0.3146823F,0.3119127F,
  0.3091432F,0.3063705F,0.3035979F,0.3008190F,0.2980402F,0.2952582F,
  0.2924740F,0.2896857F,0.2868975F,0.2841071F,0.2813127F,0.2785160F,
  0.2757185F,0.2729155F,0.2701158F,0.2673089F,0.2645029F,0.2616937F,
  0.2588814F,0.2560660F,0.2532507F,0.2504321F,0.2476137F,0.2447921F,
  0.2419673F,0.2391426F,0.2363116F,0.2334838F,0.2306504F,0.2278196F,
  0.2249824F,0.2221459F,0.2193056F,0.2164661F,0.2136233F,0.2107768F,
  0.2079310F,0.2050852F,0.2022363F,0.1993835F,0.1965315F,0.1936763F,
  0.1908212F,0.1879628F,0.1851046F,0.1822437F,0.1793824F,0.1765178F,
  0.1736533F,0.1707862F,0.1679186F,0.1650478F,0.1621776F,0.1593069F,
  0.1564336F,0.1535602F,0.1506833F,0.1478068F,0.1449272F,0.1420476F,
  0.1391680F,0.1362852F,0.1334025F,0.1305199F,0.1276340F,0.1247482F,
  0.1218624F,0.1189738F,0.1160848F,0.1131959F,0.1103042F,0.1074121F,
  0.1045204F,0.1016252F,0.0987336F,0.0958387F,0.0929404F,0.0900455F,
  0.0871475F,0.0842495F,0.0813516F,0.0784504F,0.0755492F,0.0726513F,
  0.0697502F,0.0668459F,0.0639448F,0.0610407F,0.0581364F,0.0552323F,
  0.0523281F,0.0494240F,0.0465198F,0.0436125F,0.0407083F,0.0378010F,
  0.0348938F,0.0319865F,0.0290792F,0.0261719F,0.0232647F,0.0203574F,
  0.0174501F,0.0145396F,0.0116323F,0.0087251F,0.0058146F,0.0029073F
};


Float64_t CalcSphericalDistance(
    const GpsPoint &point1, const GpsPoint &point2) {
  if (point1 == point2) {
    return (0.0);
  }

  const static Float64_t DE2RA = 0.01745329252;
  // Earth flattening (WGS84)
  const static Float64_t FLATTENING = 1.000000 / 298.257223563;
  const static Float64_t ERAD = 6378.137;

  Float64_t lat1 = DE2RA * point1.latitude;
  Float64_t lon1 = -DE2RA * point1.longitude;
  Float64_t lat2 = DE2RA * point2.latitude;
  Float64_t lon2 = -DE2RA * point2.longitude;

  Float64_t F = (lat1 + lat2) / 2.0;
  Float64_t G = (lat1 - lat2) / 2.0;
  Float64_t L = (lon1 - lon2) / 2.0;

  Float64_t sing = com_sin(G);
  Float64_t cosl = com_cos(L);
  Float64_t cosf = com_cos(F);
  Float64_t sinl = com_sin(L);
  Float64_t sinf = com_sin(F);
  Float64_t cosg = com_cos(G);

  Float64_t S = sing*sing*cosl*cosl + cosf*cosf*sinl*sinl;
  Float64_t C = cosg*cosg*cosl*cosl + sinf*sinf*sinl*sinl;
  Float64_t W = com_atan2(sqrt(S),sqrt(C));
  Float64_t R = com_sqrt((S*C))/W;
  Float64_t H1 = (3.0 * R - 1.0) / (2.0 * C);
  Float64_t H2 = (3.0 * R + 1.0) / (2.0 * S);
  Float64_t D = 2.0 * W * ERAD;

  return (1.0 * 1000.0 * D *
          (1.0 + FLATTENING * H1 * sinf*sinf*cosg*cosg -
           FLATTENING*H2*cosf*cosf*sing*sing));
}

// 计算GPS坐标距离参数
void CalcLonLatParam(const GpsPoint& pos, CalcDistParm& dist_parm) {
  GpsPoint start;
  GpsPoint end;
  start.longitude = pos.longitude;
  start.latitude = pos.latitude;
  end.longitude = start.longitude + 1.0 / 36.0;
  end.latitude = start.latitude;

  Float64_t arc_dist = CalcSphericalDistance(start, end);
  if(com_abs(arc_dist) > NumLimits<Float64_t>::epsilon()) {
    dist_parm.lon_parm = 1.0 / arc_dist;
  } else {
    dist_parm.lon_parm = 0.0;
  }
  end.longitude = start.longitude;
  end.latitude = start.latitude + 1.0 / 36.0;

  arc_dist = CalcSphericalDistance(start, end);
  if(com_abs(arc_dist) > NumLimits<Float64_t>::epsilon()) {
    dist_parm.lat_parm = 1.0 / arc_dist;
  } else {
    dist_parm.lat_parm = 0.0;
  }
}

Float64_t CalcAngle(const GpsPoint &crd1, const GpsPoint &crd2) {
  Float64_t angle = 0.0;
  Int32_t latitude_ind = (Int32_t)(crd1.latitude * 6.0);
  Float64_t fac = 1.0;

  if ((latitude_ind >= 0) &&
      (latitude_ind < LATITUDE_DIST_PER_LAT_TABLE_SIZE)) {
    fac = s_lon_dist_per_lat[latitude_ind];
  } else {
    return (angle);
  }

  angle = com_atan2(crd2.latitude - crd1.latitude,
                    (crd2.longitude - crd1.longitude) * fac);

  return (angle);
}

GpsPoint CalcNextGpsPoint(
    const GpsPoint &pos, Float64_t angle, Float64_t meter_dist) {
  GpsPoint next_pos;
  CalcDistParm dist_parm;
  Float64_t cos_angle = com_cos(angle);
  Float64_t sin_angle = com_sin(angle);

  CalcLonLatParam(pos, dist_parm);

  next_pos.longitude = pos.longitude +
      cos_angle * meter_dist * dist_parm.lon_parm * (1.0 / 36.0);
  next_pos.latitude  = pos.latitude +
      sin_angle * meter_dist * dist_parm.lat_parm * (1.0 / 36.0);

  return (next_pos);
}

// 计算相对坐标  正东向为0度，逆时针为正，顺时针为负
Float64_t ConvGpsPointToLocalPoint(const GpsPoint &gps_origin,
                              const GpsPoint &gps_point,
                              Vec2d* local_point) {
  Float64_t distance = CalcSphericalDistance(gps_origin, gps_point);
  Float64_t angle = CalcAngle(gps_origin, gps_point);
  local_point->set_x(distance * com_cos(angle));
  local_point->set_y(distance * com_sin(angle));

  return (angle);
}


Char_t GetUTMLetterDesignator(Float64_t lat) {
  Char_t letter_designator;

  if ((84.0 >= lat) && (lat >= 72.0)) {
    letter_designator = 'X';
  } else if ((72.0 > lat) && (lat >= 64.0)) {
    letter_designator = 'W';
  } else if ((64.0 > lat) && (lat >= 56.0)) {
    letter_designator = 'V';
  } else if ((56.0 > lat) && (lat >= 48.0)) {
    letter_designator = 'U';
  } else if ((48.0 > lat) && (lat >= 40.0)) {
    letter_designator = 'T';
  } else if ((40.0 > lat) && (lat >= 32.0)) {
    letter_designator = 'S';
  } else if ((32.0 > lat) && (lat >= 24.0)) {
    letter_designator = 'R';
  } else if ((24.0 > lat) && (lat >= 16.0)) {
    letter_designator = 'Q';
  } else if ((16.0 > lat) && (lat >= 8.0)) {
    letter_designator = 'P';
  } else if ((8.0 > lat) && (lat >= 0.0)) {
    letter_designator = 'N';
  } else if ((0.0 > lat) && (lat >= -8.0)) {
    letter_designator = 'M';
  } else if ((-8.0 > lat) && (lat >= -16.0)) {
    letter_designator = 'L';
  } else if ((-16.0 > lat) && (lat >= -24.0)) {
    letter_designator = 'K';
  } else if ((-24.0 > lat) && (lat >= -32.0)) {
    letter_designator = 'J';
  } else if ((-32.0 > lat) && (lat >= -40.0)) {
    letter_designator = 'H';
  } else if ((-40.0 > lat) && (lat >= -48.0)) {
    letter_designator = 'G';
  } else if ((-48.0 > lat) && (lat >= -56.0)) {
    letter_designator = 'F';
  } else if ((-56.0 > lat) && (lat >= -64.0)) {
    letter_designator = 'E';
  } else if ((-64.0 > lat) && (lat >= -72.0)) {
    letter_designator = 'D';
  } else if ((-72.0 > lat) && (lat >= -80.0)) {
    letter_designator = 'C';
  } else {
    // 'Z' is an error flag, the Latitude is outside the UTM limits
    letter_designator = 'Z';
  }

  return (letter_designator);
}

Float64_t LLtoUTM(const Float64_t lat, const Float64_t lon,
                  Float64_t* utm_northing, Float64_t* utm_easting) {
  Float64_t a = WGS84_A;
  Float64_t ecc_squared = UTM_E2;
  Float64_t k0 = UTM_K0;
  Float64_t lon_origin = 0;
  Float64_t ecc_prime_squared = 0;
  Float64_t N = 0;
  Float64_t T = 0;
  Float64_t C = 0;
  Float64_t A = 0;
  Float64_t M = 0;
  Char_t utm_zone[32] = { '\0' };

  // Make sure the longitude is between -180.00 .. 179.9
  Float64_t lon_temp = (lon + 180) -
      static_cast<Int32_t>((lon + 180) / 360) * 360 - 180;

  Float64_t lat_rad = com_deg2rad(lat);
  Float64_t lon_rad = com_deg2rad(lon_temp);
  Float64_t lon_origin_rad = 0;
  Int32_t   zone_number = 0;

  zone_number = static_cast<Int32_t>((lon_temp + 180)/6) + 1;

  if (lat >= 56.0 && lat < 64.0 && lon_temp >= 3.0 && lon_temp < 12.0) {
    zone_number = 32;
  }

  // Special zones for Svalbard
  if ((lat >= 72.0) && (lat < 84.0)) {
    if ((lon_temp >= 0.0) && (lon_temp <  9.0)) {
      zone_number = 31;
    } else if ((lon_temp >= 9.0) && (lon_temp < 21.0)) {
      zone_number = 33;
    } else if ((lon_temp >= 21.0) && (lon_temp < 33.0)) {
      zone_number = 35;
    } else if ((lon_temp >= 33.0) && (lon_temp < 42.0)) {
      zone_number = 37;
    } else {
      // nothing to do
    }
  }
  // +3 puts origin in middle of zone
  lon_origin = static_cast<Float64_t>((zone_number - 1) * 6 - 180 + 3);
  lon_origin_rad = com_deg2rad(lon_origin);

  // Compute the UTM Zone from the latitude and longitude
  Char_t zone_buf[] = {'\0', '\0', '\0', '\0'};
  com_snprintf(zone_buf, sizeof(zone_buf), "%d%c",
               zone_number, GetUTMLetterDesignator(lat));
  com_memcpy(utm_zone, zone_buf, sizeof(zone_buf));

  ecc_prime_squared = (ecc_squared) / (1-ecc_squared);

  N = a / com_sqrt(1 - ecc_squared * Square(com_sin(lat_rad)));
  T = Square(com_tan(lat_rad));
  C = ecc_prime_squared * Square(com_cos(lat_rad));
  A = com_cos(lat_rad) * (lon_rad - lon_origin_rad);

  Float64_t ecc_squared_2 = ecc_squared * ecc_squared;
  Float64_t ecc_squared_3 = ecc_squared_2 * ecc_squared;
  M = a * ((1 - ecc_squared / 4 - 3 * ecc_squared_2 / 64
          - 5 * ecc_squared_3 / 256) * lat_rad
         - (3 * ecc_squared / 8 + 3 * ecc_squared_2 / 32
            + 45 * ecc_squared_3 / 1024) * com_sin(2*lat_rad)
         + (15 * ecc_squared_2 / 256
            + 45 * ecc_squared_3 / 1024) * com_sin(4*lat_rad)
         - (35 * ecc_squared_3 / 3072) * com_sin(6*lat_rad));

  Float64_t A_2 = A * A;
  Float64_t A_3 = A_2 * A;
  Float64_t A_4 = A_3 * A;
  Float64_t A_5 = A_4 * A;
  *utm_easting = static_cast<Float64_t>
      (k0*N*(A+(1-T+C)*A_3/6
             + (5-18*T+T*T+72*C-58*ecc_prime_squared)*A_5/120)
       + 500000.0);

  *utm_northing = static_cast<Float64_t>
      (k0*(M+N*com_tan(lat_rad)
           *(A_2/2+(5-T+9*C+4*C*C)*A_4/24
             + (61-58*T+T*T+600*C-330*ecc_prime_squared)*A*A_5/720)));

  if (lat < 0) {
    // 10000000 meter offset for southern hemisphere
    *utm_northing += 10000000.0;
  }

  //子午线收敛角，用来补偿航向角
  Float64_t converge_angle = com_sin(lat_rad) * (lon_temp - lon_origin);
  return (converge_angle);
}

void UTMtoLL(const Float64_t utm_northing, const Float64_t utm_easting,
             const Char_t utm_zone[32], Float64_t* lat,  Float64_t* lon) {
  Float64_t k0 = UTM_K0;
  Float64_t a = WGS84_A;
  Float64_t ecc_squared = UTM_E2;
  Float64_t ecc_prime_squared = 0.0;
  Float64_t e1 = (1.0 - com_sqrt(1.0 - ecc_squared)) /
      (1.0 + com_sqrt(1.0 - ecc_squared));
  Float64_t N1 = 0.0;
  Float64_t T1 = 0.0;
  Float64_t C1 = 0.0;
  Float64_t R1 = 0.0;
  Float64_t D = 0.0;
  Float64_t M = 0.0;
  Float64_t lon_origin = 0.0;
  Float64_t mu = 0.0;
  Float64_t phi1_rad = 0.0;
  Float64_t x = 0.0;
  Float64_t y = 0.0;
  Int32_t zone_number = 0X0;
  Char_t* zone_letter = Nullptr_t;

  x = utm_easting - 500000.0;  // remove 500,000 meter offset for longitude
  y = utm_northing;

  zone_number = strtoul(utm_zone, &zone_letter, 10);
  if ((static_cast<Int32_t>(*zone_letter) - static_cast<Int32_t>('N')) < 0) {
    // remove 10,000,000 meter offset used for southern hemisphere
    y -= 10000000.0;
  }

  // +3 puts origin in middle of zone
  lon_origin = static_cast<Float64_t>((zone_number - 1) * 6 - 180 + 3);
  ecc_prime_squared = (ecc_squared) / (1.0 - ecc_squared);

  M = y / k0;
  mu = M/(a*(1-ecc_squared/4-3*ecc_squared*ecc_squared/64
                   -5*ecc_squared*ecc_squared*ecc_squared/256));

  Float64_t e1_2 = e1 * e1;
  Float64_t e1_3 = e1_2 * e1;
  phi1_rad = mu + ((3*e1/2-27*e1_3/32)*com_sin(2*mu)
                   + (21*e1_2/16-55*e1*e1_3/32)*com_sin(4*mu)
                   + (151*e1_3/96)*com_sin(6*mu));

  N1 = a / com_sqrt(1 - ecc_squared * Square(com_sin(phi1_rad)));
  T1 = Square(com_tan(phi1_rad));
  C1 = ecc_prime_squared * Square(com_cos(phi1_rad));
  R1 = a * (1-ecc_squared) /
      std::pow(1-ecc_squared*Square(com_sin(phi1_rad)), 1.5);
  D = x / (N1 * k0);

  Float64_t D_2 = D * D;
  Float64_t D_3 = D_2 * D;
  Float64_t D_4 = D_3 * D;
  Float64_t D_5 = D_4 * D;
  *lat = phi1_rad - ((N1*com_tan(phi1_rad)/R1)
                    *(D*D/2
                      -(5+3*T1+10*C1-4*C1*C1-9*ecc_prime_squared)*D_4/24
                      +(61+90*T1+298*C1+45*T1*T1-252*ecc_prime_squared
                        -3*C1*C1)*D*D_5/720));

  *lat = com_rad2deg(*lat);

  *lon = ((D-(1+2*T1+C1)*D_3/6
                 +(5-2*C1+28*T1-3*C1*C1+8*ecc_prime_squared+24*T1*T1)
                 *D_5/120)
                / com_cos(phi1_rad));
  *lon = lon_origin + com_rad2deg(*lon);
}


}  // namespace common
}  // namespace phoenix


