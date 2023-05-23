#ifndef TRAJECTORY_PANEL_HPP_
#define TRAJECTORY_PANEL_HPP_


#ifndef Q_MOC_RUN
#include <memory>
#endif

// ros
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include <tf2/LinearMath/Quaternion.h>

// ros msgs
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"

// rviz
#include "rviz_common/panel.hpp"

// QT
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QTimer>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QRadioButton>
#include <QButtonGroup>
#include <QTableWidget>
#include <QPushButton>

// trajectory
#include "Lissajous.hpp"

namespace dasc_robot_gui {

class TrajectoryPanel : public rviz_common::Panel {

  Q_OBJECT

public:
  
  explicit TrajectoryPanel(QWidget *parent = 0);

  virtual void load(const rviz_common::Config &config);
  virtual void save(rviz_common::Config config) const;


public Q_SLOTS:
  void setTopic(const QString &topic);

protected Q_SLOTS:

  void timer_callback();
  void viz_timer_callback();
  void updateTopic();
  


protected:

  // ROS Topic name:
  QLineEdit *output_topic_editor_;


  // Radio Buttons
  QRadioButton * traj_hover_button_;
  QRadioButton * traj_lissa_button_;
  QButtonGroup * traj_type_button_group_;


  QPushButton *set_hover_button_, * set_circle_button_, * set_figure8_button_, * set_lissa_button_;
  QPushButton *takeoff, *start, *hover, *stop;

  // LissaTable
  QDoubleSpinBox * amplitude_x, *amplitude_y, *amplitude_z, *amplitude_yaw;
  QDoubleSpinBox * freq_x, *freq_y, *freq_z, *freq_yaw;
  QDoubleSpinBox * phi_x, *phi_y, *phi_z, *phi_yaw;
  QDoubleSpinBox * offset_x, *offset_y, *offset_z, *offset_yaw;


  // name of the output topic;
  QString output_topic_;


  // ROS
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr vis_path_pub_;




  enum Mode {
	  STOPPED,
	  TAKEOFF,
	  STARTED,
	  HOVER
  };

  Mode mode = STOPPED;


  px4_msgs::msg::TrajectorySetpoint msg;
  Lissajous<double, 4> lissa;

  uint64_t traj_start_time;

  void update_lissa();
  void visualize_trajectory();


  void set_traj_hover();
  void set_traj_circle();
  void set_traj_figure8();
  void set_traj_lissa();

}; // class TrajectoryPanel





} // end namespace dasc_robot_gui





#endif // TRAJECTORY_PANEL_HPP_
