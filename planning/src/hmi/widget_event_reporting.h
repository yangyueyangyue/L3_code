//
#ifndef PHOENIX_HMI_WIDGET_EVENT_REPORTING_H_
#define PHOENIX_HMI_WIDGET_EVENT_REPORTING_H_


#include "QtGui"
#include "QWidget"
// #include "QtMultimedia/QSound"
#include "QtWidgets/QHBoxLayout"
#include "QtWidgets/QTextBrowser"
#include "QLabel"

#include "msg_event_reporting.h"


namespace phoenix {
namespace hmi {


class WidgetEventReporting : public QWidget {
  Q_OBJECT

public:
  WidgetEventReporting(QWidget* parent = 0);
  ~WidgetEventReporting();

  void UpdatePos(const QPoint pos);

  const QPoint& GetPosOffsetToParent() {
    if (0) {
      return (pos_offset_to_parent_);
    } else {
      pos_offset_to_parent_.setX(this->parentWidget()->width()/2 - width()/2 + 50);
      pos_offset_to_parent_.setY(80);
      return (pos_offset_to_parent_);
    }
  }

  void ShowEventInfo(const ad_msg::EventReporting& event);

protected:
  void mousePressEvent(QMouseEvent *e);
  void mouseMoveEvent(QMouseEvent *e);
  void mouseReleaseEvent(QMouseEvent *e);
  void showEvent(QShowEvent *e);

private:
  void SetAreaMovable(const QRect rt);
  void CalcPosOffsetToParent();
  QString GetReasonOfRefusingChangingLane(
      const Int32_t reason, QString* audio_file_postfix);

private:
  QRect area_movable_;  //可移动窗口的区域，鼠标只有在该区域按下才能移动窗口
  bool is_mouse_ressed_;  //鼠标按下标志（不分左右键）
  QPoint point_mouse_pressed_;  //鼠标按下的初始位置
  QPoint pos_offset_to_parent_;

  struct AudioNotifying {
    Int32_t lifetime;
    QString audio_file;

    void Clear() {
      lifetime = 0;
      audio_file.clear();
    }

    AudioNotifying() {
      Clear();
    }
  } audio_nofifying_;

  QHBoxLayout* horizontal_layout_;
  //QTextBrowser* text_browser_log_;
  QLabel* label_image_;
  QLabel* label_text_;

  QPixmap icon_warning_;
  QPixmap icon_changing_lane_left_;
  QPixmap icon_changing_lane_right_;
  QPixmap icon_refuse_changing_lane_;
  QPixmap icon_abort_changing_lane_;
  QPixmap icon_complete_changing_lane_;
};


} // hmi
} // phoenix


#endif // PHOENIX_HMI_WIDGET_EVENT_REPORTING_H_

