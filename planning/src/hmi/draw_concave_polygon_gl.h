#ifndef PHOENIX_HMI_DRAW_CONCAVE_POLYGON_GL_H_
#define PHOENIX_HMI_DRAW_CONCAVE_POLYGON_GL_H_

#include "GL/gl.h"
#include "GL/glu.h"
#include "GL/glut.h"

#include "utils/macros.h"
#include "utils/log.h"


namespace phoenix {
namespace hmi {



/**
 * @class DrawConcavePolygon
 * @brief 画凹多边形
 *
 * @par Note:
 * @code
 *     通常可以采用一种叫做"分格化"的方法来画复杂的多边形。
 *     非凸多边形最简单的填充方法最简单的应该是GLU 网格化对象GLUtesselator
 *     (GLUT库或者libTess库)。要用分格化的方法画多边形，步骤如下：
 *     1. gluNewTess(); //创建一个新的分格化对象
 *     2. gluTessCallback(); //注册回调函数，完成分格化的一些操作，照着写就行了。
 *     3. gluTessProperty(); //设置一些分格化的属性值，如环绕数和环绕规则，
 *                             用来确定多边形的内部和外部
 *     4. gluTessBeginPolygon(); //开始画多边形
 *        gluTessBeginContour(tessobj);//设置多边形的边线 1
 *        gluTessEndContour(tessobj);//结束设置边线1
 *        gluTessBeginContour(tessobj);//，如果有边线2，设置多边形的边线 2
 *        gluTessEndContour(tessobj);//结束设置边线2
 *     5. gluTessEdnPolygon(); //结束画多边形
 *     6. gluDeleteTess(); //删除分格化对象
 *     当然也可以利用回调函数记录分格化的顶点和绘制类型，
 *     然后利用数组的绘制函数进行一次性的绘制，以提高绘制效率。
 * @endcode
 */
enum { MAX_CONCAVE_POLYGON_POINT_NUM = 30720 };
//GLdouble s_concave_polygon_points_buff[MAX_CONCAVE_POLYGON_POINT_NUM][3];
class DrawConcavePolygon {
public:
  DrawConcavePolygon() {
    point_index_ = 0;
    //points_buff_ = &s_concave_polygon_points_buff[0];
    points_buff_ = new GLdouble[MAX_CONCAVE_POLYGON_POINT_NUM][3];
  }

  ~DrawConcavePolygon() { delete [] points_buff_; }

  void Paint() {
    tess_obj_ = gluNewTess();
    gluTessCallback(tess_obj_, GLU_TESS_BEGIN, (_GLUfuncptr)&PolyLine3DBegin);
    gluTessCallback(tess_obj_, GLU_TESS_VERTEX, (_GLUfuncptr)&PolyLine3DVertex);
    gluTessCallback(tess_obj_, GLU_TESS_END, (_GLUfuncptr)&PolyLine3DEnd);
    gluTessCallback(tess_obj_, GLU_TESS_ERROR, (_GLUfuncptr)&HandleErr);

    gluTessBeginPolygon(tess_obj_, NULL);
    gluTessBeginContour(tess_obj_);
  }

  void DrawVertex(GLdouble x, GLdouble y, GLdouble z = 0) {
    points_buff_[point_index_][0] = x;
    points_buff_[point_index_][1] = y;
    points_buff_[point_index_][2] = z;

    if (IsSamePoint(point_index_)) {
      return;
    }

    gluTessVertex(tess_obj_,
                  points_buff_[point_index_],
                  points_buff_[point_index_]);

    point_index_++;
  }

  void EndPaint() {
    gluTessEndContour(tess_obj_);
    gluTessEndPolygon(tess_obj_);
    gluDeleteTess(tess_obj_);
  }

private:
  bool IsSamePoint(Int32_t index) {
    if (index > 0) {
      GLdouble dx = fabs(points_buff_[index][0]-points_buff_[index-1][0]);
      GLdouble dy = fabs(points_buff_[index][1]-points_buff_[index-1][1]);
      if (dx < 1e-5 && dy < 1e-5) {
        return true;
      }
    }

    return false;
  }

  static void PolyLine3DBegin(GLenum type) {
    glBegin(type);
  }

  static void PolyLine3DVertex(GLdouble* vertex) {
    const GLdouble *pointer = (GLdouble*) vertex;
    glVertex3dv(pointer);
  }

  static void PolyLine3DEnd() {
    glEnd();
  }

  static void HandleErr(GLenum error_code) {
    const GLubyte * estring;
    // 打印错误类型
    estring = gluErrorString(error_code);
    // std::cout << "Tessellation Error: " << estring << std::endl;
    LOG_ERR << "Tessellation Error: " << (Char_t*)estring;
  }

private:
  GLUtesselator* tess_obj_;
  Int32_t point_index_;
  GLdouble (*points_buff_)[3];
};


}  // namespace hmi
}  // namespace phoenix


#endif // PHOENIX_HMI_DRAW_CONCAVE_POLYGON_GL_H_

