#include "trajectory_panel.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace dasc_robot_gui {

using std::placeholders::_1;



TrajectoryPanel::TrajectoryPanel(QWidget *parent) : rviz_common::Panel(parent) {

  // construct the layout

  // topic layout
  QHBoxLayout *topic_layout = new QHBoxLayout;
  topic_layout -> addWidget(new QLabel("Robot Namespace:"));
  output_topic_editor_ = new QLineEdit;
  topic_layout -> addWidget(output_topic_editor_);


  // Radio Buttons
  QHBoxLayout *traj_type_layout = new QHBoxLayout;
  traj_type_button_group_ = new QButtonGroup;
  traj_hover_button_ = new QRadioButton("Hover", this);
  traj_lissa_button_ = new QRadioButton("Lissajous", this);
  traj_type_button_group_->addButton(traj_hover_button_);
  traj_type_button_group_->addButton(traj_lissa_button_);
  traj_type_layout -> addWidget(traj_hover_button_);
  traj_type_layout -> addWidget(traj_lissa_button_);


  // stack it all together
  QVBoxLayout *layout = new QVBoxLayout;
  layout -> addLayout(topic_layout);
  layout -> addLayout(traj_type_layout);

  setLayout(layout);

  // create the timer
  QTimer * output_timer = new QTimer(this);
  connect(output_timer, SIGNAL(timeout()), this, SLOT(timer_callback()));

  // connect the buttons


  // connect the slots
  connect(output_topic_editor_, SIGNAL(editingFinished()), this, SLOT(updateTopic()));

  // start the main timer
  output_timer -> start(200); //ms

  // create the node
  node_ = std::make_shared<rclcpp::Node>("dasc_trajectory_panel_node");
}

void TrajectoryPanel::updateTopic() { 

  std::cout << "update topic!" << std::endl;

  setTopic(output_topic_editor_->text()); 

  return;
 
}

void TrajectoryPanel::setTopic(const QString &new_string) {

  // only take action if the name has changed.
  if (new_string == output_topic_) {
    return;
  }

  output_topic_ = new_string;

  if (trajectory_setpoint_pub_ != NULL) {
    trajectory_setpoint_pub_.reset();
  }

  // reset

  // if topic is empty, dont publish anything
  if (output_topic_ != "") {

    // create publishers
    trajectory_setpoint_pub_ = node_ -> create_publisher<px4_msgs::msg::TrajectorySetpoint>( 
    output_topic_.toStdString() + "/fmu/in/trajectory_setpoint", 1);


    // create subscribers
    // rmw_qos_profile_t ....

  }

  // emit config changed signal
  Q_EMIT configChanged();
}


void TrajectoryPanel::timer_callback() {

  rclcpp::spin_some(node_);

  if (!(rclcpp::ok() && trajectory_setpoint_pub_ != NULL)) {
    return;
  }
  

  // construct the setpoint
  px4_msgs::msg::TrajectorySetpoint msg;
  msg.raw_mode = false;
  for (size_t i=0; i<3 ; i++) {

    msg.position[i] = 1.0;
    msg.velocity[i] = 0.0;
    msg.acceleration[i] = 0.0;
    msg.jerk[i] = 0.0;
  }

  msg.yaw = 0.0;
  msg.yawspeed = 0.0;

  trajectory_setpoint_pub_ -> publish(msg);

}


void TrajectoryPanel::save(rviz_common::Config config) const {

  rviz_common::Panel::save(config);
  config.mapSetValue("Topic", output_topic_);

}

void TrajectoryPanel::load(const rviz_common::Config &config) { 
  rviz_common::Panel::load(config);

  QString topic;
  if (config.mapGetString("Topic", &topic) ){

    output_topic_editor_ -> setText(topic);
    updateTopic();
  }



}

} // namespace dasc_robot_gui

PLUGINLIB_EXPORT_CLASS(dasc_robot_gui::TrajectoryPanel, rviz_common::Panel)


