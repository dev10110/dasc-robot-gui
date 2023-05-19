#ifndef TRAJECTORY_PANEL_HPP_
#define TRAJECTORY_PANEL_HPP_


#ifndef Q_MOC_RUN
#include <memory>
#endif

// ros
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

// ros msgs
#include "px4_msgs/msg/trajectory_setpoint.hpp"

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
  void updateTopic();
  


protected:

  // ROS Topic name:
  QLineEdit *output_topic_editor_;


  // Radio Buttons
  QRadioButton * traj_hover_button_;
  QRadioButton * traj_lissa_button_;
  QButtonGroup * traj_type_button_group_;


  // name of the output topic;
  QString output_topic_;


  // ROS
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;


}; // class TrajectoryPanel





} // end namespace dasc_robot_gui





#endif // TRAJECTORY_PANEL_HPP_
