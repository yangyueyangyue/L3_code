//
#include <string>
#include <iostream>
#include "widget_event_reporting.h"
#include "motion_planning.h"


namespace phoenix {
namespace hmi {


WidgetEventReporting::WidgetEventReporting(QWidget *parent) : QWidget(parent) {
  //设置无边框透明
  setWindowFlags(Qt::FramelessWindowHint | Qt::Tool);//无边框
  //this->setStyleSheet(QString("background-color: rgba(255, 128, 255, 100);"));
  this->setStyleSheet(QString("background-color: rgba(255, 255, 100, 100);"));
  //this->setAttribute(Qt::WA_TranslucentBackground, false);//背景透明

  // 窗口及其上面的控件都半透明
  this->setAttribute(Qt::WA_TranslucentBackground, true);
  //this->setAttribute(Qt::WA_WState_WindowOpacitySet);
  //this->setWindowOpacity(0.1);
  this->setFixedSize(550, 100); 

  pos_offset_to_parent_.setX(this->parentWidget()->width()/2 - width()/2 + 50);
  //pos_offset_to_parent_.setY(this->parentWidget()->height()/2 -height()/2);
  pos_offset_to_parent_.setY(80);

  is_mouse_ressed_ = false;

  horizontal_layout_ = new QHBoxLayout(this);
  // text_browser_log_ = new QTextBrowser(this);
  // text_browser_log_->document()->setMaximumBlockCount(500);
  // text_browser_log_->setReadOnly(true);
  // text_browser_log_->setStyleSheet(QString("background-color: rgb(255, 128, 255);"));
  // text_browser_log_->setAttribute(Qt::WA_TranslucentBackground, true);
  // text_browser_log_->setTextColor(QColor(255, 255, 255));
  // horizontal_layout_->addWidget(text_browser_log_);
  label_image_ = new QLabel(this);
  label_image_->setFixedSize(height()-22, height()-22);
  //label_image_->setStyleSheet(QString("background-color: rgb(255, 128, 255);"));
  //label_image_->setAttribute(Qt::WA_TranslucentBackground, true);
  label_image_->setAlignment(Qt::AlignCenter);
  label_text_ = new QLabel(this);
  //label_text_->setStyleSheet(QString("background-color: rgb(255, 128, 255);"));
  //label_text_->setAttribute(Qt::WA_TranslucentBackground, true);
  QPalette pe;
  pe.setColor(QPalette::WindowText,Qt::red);
  label_text_->setPalette(pe);
  QFont ft;
  ft.setPointSize(23);
  label_text_->setFont(ft);
  label_text_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  horizontal_layout_->addWidget(label_image_);
  horizontal_layout_->addWidget(label_text_);

  if (!icon_warning_.load(":/images/warning.png")) {
    std::cerr << "Failed to load image \":/images/warning.png\""
              << std::endl;
  }
  if (!icon_changing_lane_left_.load(":/images/start_changing_lane_to_left.png")) {
    std::cerr << "Failed to load image \":/images/start_changing_lane_to_left.png\""
              << std::endl;
  }
  if (!icon_changing_lane_right_.load(":/images/start_changing_lane_to_right.png")) {
    std::cerr << "Failed to load image \":/images/start_changing_lane_to_right.png\""
              << std::endl;
  }
  if (!icon_refuse_changing_lane_.load(":/images/refuse_changing_lane.png")) {
    std::cerr << "Failed to load image \":/images/refuse_changing_lane.png\""
              << std::endl;
  }
  if (!icon_abort_changing_lane_.load(":/images/abort_changing_lane.png")) {
    std::cerr << "Failed to load image \":/images/abort_changing_lane.png\""
              << std::endl;
  }
  if (!icon_complete_changing_lane_.load(":/images/complete_changing_lane.png")) {
    std::cerr << "Failed to load image \":/images/complete_changing_lane.png\""
              << std::endl;
  }
}

WidgetEventReporting::~WidgetEventReporting() {
}

void WidgetEventReporting::SetAreaMovable(const QRect rt) {
  if(area_movable_ != rt) {
    area_movable_ = rt;
  }
}

void WidgetEventReporting::UpdatePos(const QPoint pos) {
  QSize size = this->size();
  QPoint parent_pos = this->parentWidget()->pos();
  QSize parent_size = this->parentWidget()->size();
  QPoint cur_pos = pos;
  if(cur_pos.x() < parent_pos.x()) {
    //left
    cur_pos.setX(parent_pos.x());
  }
  if(cur_pos.y() < parent_pos.y()) {
    //top
    cur_pos.setY(parent_pos.y());
  }
  if( (cur_pos.x()+size.width()) > (parent_pos.x()+parent_size.width())) {
    //right
    cur_pos.setX(parent_pos.x() + parent_size.width() - size.width());
  }
  if( (cur_pos.y()+size.height()) > (parent_pos.y()+parent_size.height())) {
    //bottom
    cur_pos.setY(parent_pos.y() + parent_size.height() - size.height());
  }

  pos_offset_to_parent_.setX(cur_pos.x() - this->parentWidget()->pos().x());
  pos_offset_to_parent_.setY(cur_pos.y() - this->parentWidget()->pos().y());

  move(cur_pos);
}

void WidgetEventReporting::mousePressEvent(QMouseEvent *e) {
  if(Qt::LeftButton == e->button()) {
    point_mouse_pressed_ = e->pos();
    is_mouse_ressed_ = area_movable_.contains(point_mouse_pressed_);
    SetAreaMovable(this->rect());
  }
}

void WidgetEventReporting::mouseMoveEvent(QMouseEvent *e) {
  if(is_mouse_ressed_) {
    UpdatePos(pos() + e->pos() - point_mouse_pressed_);
  }
}

void WidgetEventReporting::mouseReleaseEvent(QMouseEvent *e) {
  is_mouse_ressed_ = false;
}

void WidgetEventReporting::showEvent(QShowEvent *e) {
  SetAreaMovable(this->rect());

  //std::cout << "****************** SH **************" << std::endl;

  UpdatePos(this->parentWidget()->pos() + pos_offset_to_parent_);
}


void WidgetEventReporting::ShowEventInfo(const ad_msg::EventReporting& event) {
  QString audio_file;
  QString text;
  switch (event.event_id) {
  case (ad_msg::EVENT_REPORTING_ID_START_ROBOTIC):
    label_image_->setPixmap(icon_warning_);
    label_text_->setText("进入自动模式");
    audio_file = ":/audio/start_auto.wav";
    break;
  case (ad_msg::EVENT_REPORTING_ID_STOP_ROBOTIC):
    label_image_->setPixmap(icon_warning_);
    label_text_->setText("退出自动模式");
    audio_file = ":/audio/stop_auto.wav";
    break;

  case (ad_msg::EVENT_REPORTING_ID_SENSOR_MSG_TIMEOUT):
    label_image_->setPixmap(icon_warning_);
    label_text_->setText("传感器数据超时");
    audio_file = ":/audio/sensor_timeout.wav";
    break;
  case (ad_msg::EVNET_REPORTING_ID_CHASSIS_MSG_TIMEOUT):
    label_image_->setPixmap(icon_warning_);
    label_text_->setText("车身数据超时");
    audio_file = ":/audio/chassis_timeout.wav";
    break;
  case (ad_msg::EVNET_REPORTING_ID_PLANNING_TIMEOUT):
    label_image_->setPixmap(icon_warning_);
    label_text_->setText("规划超时");
    audio_file = ":/audio/planning_timeout.wav";
    break;

  case (ad_msg::EVENT_REPORTING_ID_CAMERA_LANE_LOST):
    label_image_->setPixmap(icon_warning_);
    label_text_->setText("车道线丢失");
    audio_file = ":/audio/camera_lane_lost.wav";
    break;
  case (ad_msg::EVENT_REPORTING_ID_AEB_WARNING):
    label_image_->setPixmap(icon_warning_);
    label_text_->setText("请保持车距");
    audio_file = ":/audio/warning.wav";
    break;
  case (ad_msg::EVENT_REPORTING_ID_AEB_ACTION):
    label_image_->setPixmap(icon_warning_);
    label_text_->setText("紧急制动");
    audio_file = ":/audio/aeb.wav";
    break;
  case (ad_msg::EVENT_REPORTING_ID_UNCERTAIN_OBSTACLE):
    label_image_->setPixmap(icon_warning_);
    label_text_->setText("不确定的障碍物");
    // audio_file = ":/audio/uncertain_obstacles.wav";
    break;

  case (ad_msg::EVENT_REPORTING_ID_START_FOLLOWING_OBJ):
    /// TODO: 替换为合适的图标
    label_image_->setPixmap(icon_warning_);
    label_text_->setText("进入跟随模式");
    audio_file = ":/audio/start_following_mode.wav";
    break;
  case (ad_msg::EVENT_REPORTING_ID_STOP_FOLLOWING_OBJ):
    /// TODO: 替换为合适的图标
    label_image_->setPixmap(icon_warning_);
    label_text_->setText("退出跟随模式");
    audio_file = ":/audio/stop_following_mode.wav";
    break;

  case (ad_msg::EVENT_REPORTING_ID_VELOCITY_PLANNING_ACCORDING_TO_TUNNEL):
    label_image_->setPixmap(icon_warning_);
    label_text_->setText("隧道");
    // audio_file = ":/audio/close_to_tunnel.wav";
    break;
  case (ad_msg::EVENT_REPORTING_ID_VELOCITY_PLANNING_ACCORDING_TO_RAMP):
    label_image_->setPixmap(icon_warning_);
    label_text_->setText("匝道");
    // audio_file = ":/audio/close_to_ramp.wav";
    break;

  case (ad_msg::EVENT_REPORTING_ID_CHANGING_LANE_REQ):
    label_image_->setPixmap(icon_warning_);
    label_text_->setText("请求变道");
    audio_file = ":/audio/request_changing_lane.wav";
    break;
  case (ad_msg::EVENT_REPORTING_ID_CHANGING_LANE_RSP): {
    QString audio_file_postfix;
    switch (event.param[0]) {
    case (ad_msg::EVENT_REPORTING_CHANGING_LANE_STATUS_PREPARE_FOR_CHANGING_LEFT):
      label_image_->setPixmap(icon_changing_lane_left_);
      text = "准备左变道";
      label_text_->setText(text + GetReasonOfRefusingChangingLane(event.param[1], &audio_file_postfix));
      audio_file = ":/audio/prepare_changing_lane_to_left" + audio_file_postfix + ".wav";
      break;
    case (ad_msg::EVENT_REPORTING_CHANGING_LANE_STATUS_PREPARE_FOR_CHANGING_RIGHT):
      label_image_->setPixmap(icon_changing_lane_right_);
      text = "准备右变道";
      label_text_->setText(text + GetReasonOfRefusingChangingLane(event.param[1], &audio_file_postfix));
      audio_file = ":/audio/prepare_changing_lane_to_right" + audio_file_postfix + ".wav";
      break;
    case (ad_msg::EVENT_REPORTING_CHANGING_LANE_STATUS_START_CHANGING_LEFT):
      label_image_->setPixmap(icon_changing_lane_left_);
      label_text_->setText("开始左变道");
      audio_file = ":/audio/start_changing_lane_to_left.wav";
      break;
    case (ad_msg::EVENT_REPORTING_CHANGING_LANE_STATUS_START_CHANGING_RIGHT):
      label_image_->setPixmap(icon_changing_lane_right_);
      label_text_->setText("开始右变道");
      audio_file = ":/audio/start_changing_lane_to_right.wav";
      break;
    case (ad_msg::EVENT_REPORTING_CHANGING_LANE_STATUS_REFUSE_CHANGING_LEFT):
      label_image_->setPixmap(icon_refuse_changing_lane_);
      text = "拒绝左变道";
      label_text_->setText(text + GetReasonOfRefusingChangingLane(event.param[1], &audio_file_postfix));
      audio_file = ":/audio/refuse_changing_lane_to_left"  + audio_file_postfix + ".wav";
      break;
    case (ad_msg::EVENT_REPORTING_CHANGING_LANE_STATUS_REFUSE_CHANGING_RIGHT):
      label_image_->setPixmap(icon_refuse_changing_lane_);
      text = "拒绝右变道";
      label_text_->setText(text + GetReasonOfRefusingChangingLane(event.param[1], &audio_file_postfix));
      audio_file = ":/audio/refuse_changing_lane_to_right" + audio_file_postfix + ".wav";
      break;
    case (ad_msg::EVENT_REPORTING_CHANGING_LANE_STATUS_ABORT_CHANGING_LEFT):
      label_image_->setPixmap(icon_abort_changing_lane_);
      text = "中断左变道";
      label_text_->setText(text + GetReasonOfRefusingChangingLane(event.param[1], &audio_file_postfix));
      audio_file = ":/audio/abort_changing_lane_to_left"  + audio_file_postfix + ".wav";
      break;
    case (ad_msg::EVENT_REPORTING_CHANGING_LANE_STATUS_ABORT_CHANGING_RIGHT):
      label_image_->setPixmap(icon_abort_changing_lane_);
      text = "中断右变道";
      label_text_->setText(text + GetReasonOfRefusingChangingLane(event.param[1], &audio_file_postfix));
      audio_file = ":/audio/abort_changing_lane_to_right"  + audio_file_postfix + ".wav";
      break;
    case (ad_msg::EVENT_REPORTING_CHANGING_LANE_STATUS_COMPLETE_CHANGING_LEFT):
      label_image_->setPixmap(icon_complete_changing_lane_);
      label_text_->setText("完成左变道");
      audio_file = ":/audio/complete_changing_lane_to_left.wav";
      break;
    case (ad_msg::EVENT_REPORTING_CHANGING_LANE_STATUS_COMPLETE_CHANGING_RIGHT):
      label_image_->setPixmap(icon_complete_changing_lane_);
      label_text_->setText("完成右变道");
      audio_file = ":/audio/complete_changing_lane_to_right.wav";
      break;
    case (ad_msg::EVENT_REPORTING_CHANGING_LANE_STATUS_REFUSE_ABORT_CHANGING_LANE):
      label_image_->setPixmap(icon_complete_changing_lane_);
      text = "拒绝中止变道";
      label_text_->setText(text + GetReasonOfRefusingChangingLane(event.param[1], &audio_file_postfix));
      break;
    default:
      label_image_->setPixmap(icon_warning_);
      label_text_->setText("未知的变道类型");
      audio_file = ":/audio/unknown_changing_lane_type.wav";
      break;
    }
  }
    break;

  default:
    label_image_->setPixmap(icon_warning_);
    label_text_->setText("未知的事件ID");
    audio_file = ":/audio/unknown_event_id.wav";
    break;
  }

  if (!audio_file.isEmpty()) {
    if ((audio_nofifying_.lifetime <= -1000) ||
        (audio_nofifying_.audio_file != audio_file)) {
      audio_nofifying_.lifetime = event.lifetime;
      audio_nofifying_.audio_file = audio_file;

      //QSound::play(audio_nofifying_.audio_file);
    }
  }
  audio_nofifying_.lifetime -= 100;
  if (audio_nofifying_.lifetime < -1000) {
    audio_nofifying_.lifetime = -1000;
  }
}

QString WidgetEventReporting::GetReasonOfRefusingChangingLane(
    const Int32_t reason, QString* audio_file_postfix) {
  QString text;
  audio_file_postfix->clear();

  switch (reason) {
  case (planning::REFUSE_CHANGINHG_LANE_REASON_NONE):
    text = ":未知";
    *audio_file_postfix = "";
    break;
  case (planning::REFUSE_CHANGINHG_LANE_REASON_OBSTACLE):
    text = ":有障碍物";
    *audio_file_postfix = "_reason_obstacle";
    break;
  case (planning::REFUSE_CHANGINHG_LANE_REASON_UNCERTAIN_OBSTACLE):
    text = ":有不确定障碍物";
    *audio_file_postfix = "_reason_uncertain_obstacle";
    break;
  case (planning::REFUSE_CHANGINHG_LANE_REASON_INVALID_LANE):
    text = ":无效车道";
    *audio_file_postfix = "_reason_invalid_lane";
    break;
  case (planning::REFUSE_CHANGINHG_LANE_REASON_LOW_QUALITY_LANE):
    text = ":车道质量不好";
    *audio_file_postfix = "_reason_low_quality_lane";
    break;
  case (planning::REFUSE_CHANGINHG_LANE_REASON_SOLID_BOUNDARY):
    text = ":实线拒绝变道";
    *audio_file_postfix = "_reason_solid_boundary";
    break;
  case (planning::REFUSE_CHANGINHG_LANE_REASON_IN_TUNNEL):
    text = ":隧道内禁止变道";
    //*audio_file_postfix = "_reason_in_tunnel";
    *audio_file_postfix = "";
    break;
  case (planning::REFUSE_CHANGINHG_LANE_REASON_IN_CURVE):
    text = ":弯道禁止变道";
    //*audio_file_postfix = "_reason_in_curve";
    *audio_file_postfix = "";
    break;
  case (planning::REFUSE_CHANGINHG_LANE_REASON_BY_DRIVER):
    text = ":人工请求";
    //*audio_file_postfix = "_reason_by_driver";
    *audio_file_postfix = "";
    break;
  case (planning::REFUSE_CHANGINHG_LANE_REASON_OUT_OF_SPEEDLIMIT):
  text = ":当前速度禁止变道";
  //*audio_file_postfix = "_reason_by_driver";
  *audio_file_postfix = "";
    break;
  default :
    text = ":未知";
    *audio_file_postfix = "";
    break;
  }

  return text;
}


} // hmi
} // phoenix

