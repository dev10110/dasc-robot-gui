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


  // lissajous 
  QHBoxLayout * lissa_params_layout = new QHBoxLayout;
  QGridLayout * grid_layout = new QGridLayout;
  grid_layout -> setHorizontalSpacing(3);
  lissa_params_layout -> addItem(grid_layout);

  // header row
  grid_layout -> addWidget(new QLabel("[FRD]"), 0, 0);
  grid_layout -> addWidget(new QLabel("x [m]"), 0, 1);
  grid_layout -> addWidget(new QLabel("y [m]"), 0, 2);
  grid_layout -> addWidget(new QLabel("z [m]"), 0, 3);
  grid_layout -> addWidget(new QLabel("yaw [°]"), 0, 4);

  // amplitude
  grid_layout -> addWidget(new QLabel("Amplitude [m, °]:"), 1, 0);
  amplitude_x = new QDoubleSpinBox;
  amplitude_y= new QDoubleSpinBox;
  amplitude_z = new QDoubleSpinBox;
  amplitude_yaw = new QDoubleSpinBox;
  int index =1 ;
  for (auto s : {amplitude_x, amplitude_y, amplitude_z, amplitude_yaw} ) {
    s-> setSingleStep(0.1);
    s->setValue(0.0);
    s -> setRange(-1000.0, 1000.0);
    s->setWrapping(false);
    grid_layout -> addWidget(s, 1, index);
    index ++;
  }

  
  // freq 
  grid_layout -> addWidget(new QLabel("Freq [1/s]:"), 2, 0);
  freq_x = new QDoubleSpinBox;
  freq_y= new QDoubleSpinBox;
  freq_z = new QDoubleSpinBox;
  freq_yaw = new QDoubleSpinBox;
  index =1 ;
  for (auto s : {freq_x, freq_y, freq_z, freq_yaw} ) {
    s-> setSingleStep(0.001);
    s->setValue(1.0/30.0);
    s -> setRange(0.0, 1.0);
    s->setWrapping(false);
    grid_layout -> addWidget(s, 2, index);
    index ++;
  }

  // phi 
  grid_layout -> addWidget(new QLabel("Phase [°]:"), 3, 0);
  phi_x = new QDoubleSpinBox;
  phi_y= new QDoubleSpinBox;
  phi_z = new QDoubleSpinBox;
  phi_yaw = new QDoubleSpinBox;
  index =1 ;
  for (auto s : {phi_x, phi_y, phi_z, phi_yaw} ) {
    s-> setSingleStep(5.0);
    s->setValue(0.0);
    s -> setRange(0.0, 360.0);
    s->setWrapping(true);
    grid_layout -> addWidget(s, 3, index);
    index ++;
  }

  // offset 
  grid_layout -> addWidget(new QLabel("Offset [m, °]:"), 4, 0);
  offset_x = new QDoubleSpinBox;
  offset_y= new QDoubleSpinBox;
  offset_z = new QDoubleSpinBox;
  offset_yaw = new QDoubleSpinBox;
  index =1 ;
  for (auto s : {offset_x, offset_y, offset_z, offset_yaw} ) {
    s-> setSingleStep(0.1);
    s->setValue(0.0);
    s->setWrapping(false);
    grid_layout -> addWidget(s, 4, index);
    index ++;
  }


  // buttons
  QHBoxLayout  *button_layout = new QHBoxLayout;
  takeoff = new QPushButton("Go To Start", this);
  start = new QPushButton("Start Trajectory", this);
  hover = new QPushButton("Hover", this);
  stop = new QPushButton("Stop", this);

  for (auto b : {takeoff, start, hover, stop}) {
    button_layout -> addWidget(b);
  }


  // stack it all together
  QVBoxLayout *layout = new QVBoxLayout;
  layout -> addLayout(topic_layout);
  layout -> addLayout(traj_type_layout);
  layout -> addLayout(lissa_params_layout);
  layout -> addLayout(button_layout);

  setLayout(layout);

  // create the timer
  QTimer * output_timer = new QTimer(this);
  connect(output_timer, SIGNAL(timeout()), this, SLOT(timer_callback()));

  // connect the buttons
  connect(takeoff, &QPushButton::clicked, this, [this]() {
      update_lissa();
      mode = Mode::TAKEOFF;
  });

  connect(start, &QPushButton::clicked, this, [this]() {
      update_lissa();
      mode = Mode::STARTED;
  });
  connect(hover, &QPushButton::clicked, this, [this]() {
      mode = Mode::HOVER;
  });
  connect(stop, &QPushButton::clicked, this, [this]() {
      mode = Mode::STOPPED;
  });


  // connect the slots
  connect(output_topic_editor_, SIGNAL(editingFinished()), this, SLOT(updateTopic()));


  // create the node
  node_ = std::make_shared<rclcpp::Node>("dasc_trajectory_panel_node");
  
  // start the main timer
  output_timer -> start(200); //ms
}


void TrajectoryPanel::update_lissa() {

  double amplitude[4] = {
    amplitude_x->value(),
    amplitude_y->value(),
    amplitude_z->value(),
    amplitude_yaw->value() * M_PI / 180.0f};
  
  double omega[4] = {
    2*M_PI*freq_x->value(),
    2*M_PI*freq_y->value(),
    2*M_PI*freq_z->value(),
    2*M_PI*freq_yaw->value()};

  double phi[4] = {
    M_PI / 180.0f * phi_x->value(),
    M_PI / 180.0f * phi_y->value(),
    M_PI / 180.0f * phi_z->value(),
    M_PI / 180.0f * phi_yaw->value()};

  double offset[4] = {
    offset_x->value(),
    offset_y->value(),
    offset_z->value(),
    M_PI / 180.0f * offset_yaw->value()};

  lissa.set_params(amplitude, omega, phi, offset);

  std::cout << "params updated" << std::endl;


  traj_start_time = node_ -> get_clock() -> now().nanoseconds();

  // set start time too
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


  if (mode == Mode::STOPPED) {
    return;
  }

  if (mode == Mode::TAKEOFF) {
  // construct the setpoint
  msg.raw_mode = false;

  double res[4];
  lissa.evaluate(res, 0.0);

  for (size_t i=0; i<3 ; i++) {

    msg.position[i] = res[i];
    msg.velocity[i] = 0.0;
    msg.acceleration[i] = 0.0;
    msg.jerk[i] = 0.0;
  }

  msg.yaw = res[3];
  msg.yawspeed = 0.0;
  trajectory_setpoint_pub_ -> publish(msg);
  return;

  }
  
  if (mode == Mode::STARTED)  {
  // construct the setpoint
  msg.raw_mode = false;


   auto now = node_ -> get_clock() -> now().nanoseconds();

  double elapsed_s = 1e-9 * static_cast<double>(now - traj_start_time);

  double res_p[4];
  double res_v[4];
  double res_a[4];
  double res_j[4];
  lissa.evaluate(res_p, elapsed_s);
  lissa.evaluate(res_v, elapsed_s, 1);
  lissa.evaluate(res_a, elapsed_s, 2);
  lissa.evaluate(res_j, elapsed_s, 3);

  for (size_t i=0; i<3 ; i++) {

    msg.position[i] = res_p[i];
    msg.velocity[i] = res_v[i];
    msg.acceleration[i] = res_a[i];
    msg.jerk[i] = res_j[i];
  }

  msg.yaw = res_p[3];
  msg.yawspeed = res_v[3];
  trajectory_setpoint_pub_ -> publish(msg);
  return;

  }

  if (mode == Mode::HOVER) {

    // publish the last position
  for (size_t i=0; i<3 ; i++) {

    msg.velocity[i] = 0.0;
    msg.acceleration[i] = 0.0;
    msg.jerk[i] = 0.0;
  }

  msg.yawspeed = 0.0;
  trajectory_setpoint_pub_ -> publish(msg);
  return;


  }


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


