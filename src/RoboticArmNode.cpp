
#include "RoboticArmNode.h"

RoboticArmNode::RoboticArmNode()
{
  //rosParam
  ros::NodeHandle privNh("~");
  // std::string string_val;
  // double double_val;
  // int int_val;
  // bool bool_val;
  int max_speed;
  double rate;


  // privNh.param(         "string_val" ,    string_val,   std::string("string"));
  // privNh.param<double>( "double_val" ,    double_val,   100.0);
  // privNh.param<int>(    "int_val"    ,    int_val   ,   1.0);
  // privNh.param<bool>(   "bool_val"   ,    bool_val  ,   true);

  privNh.param<int>(    "max_speed"    ,    max_speed   ,   2   );
  privNh.param<double>( "rate"         ,    rate        ,   50.0);

  _max_speed = std::abs(max_speed);
  if(rate < 1.0)
    _rate = 1.0;
  _rate = rate;

  //init publisher
  // _pub = _nh.advertise<std_msgs::Bool>("pub_name", 1);
  _pub_arm_pos = _nh.advertise<francor_msgs::RoboticArmCmd>("robotic_arm/set_joints_pos", 1);


  //inti subscriber
  //_sub = _nh.subscribe("subname", 1, &RoboticArmNode::subCallback, this);
  _sub_speed_joints = _nh.subscribe("robotic_arm/set_joint_speed", 1, &RoboticArmNode::sub_speed_joints_callback, this);
  _sub_tcp_speed    = _nh.subscribe("robotic_arm/set_tcp_speed", 1, &RoboticArmNode::sub_speed_tcp_callback, this);

  _srv_set_stand_by = _nh.advertiseService("/robotic_arm/set_stand_by", &RoboticArmNode::srv_set_stand_by_callback, this);
  _srv_set_active   = _nh.advertiseService("/robotic_arm/set_active", &RoboticArmNode::srv_set_active_callback, this);

  _curr_pos = RoboticArmNode::get_stand_by_msg();

  _curr_speed.joint_0 = 0;
  _curr_speed.joint_1 = 0;
  _curr_speed.joint_2 = 0;
  _curr_speed.joint_3 = 0;
  _curr_speed.joint_4 = 0;
  _curr_speed.joint_5 = 0;

}

RoboticArmNode::~RoboticArmNode()
{
}

void RoboticArmNode::start()
{
  //create timer
  _loopTimer = _nh.createTimer(ros::Duration(1.0 / _rate), &RoboticArmNode::loop_callback, this);
  this->run();
}

void RoboticArmNode::run()
{
  ros::spin();
}

void RoboticArmNode::loop_callback(const ros::TimerEvent& e)
{
  _curr_pos.joint_0 += _curr_speed.joint_0;
  _curr_pos.joint_0 = RoboticArmNode::constrain(_curr_pos.joint_0, -90, 90);
  _curr_pos.joint_1 += _curr_speed.joint_1;
  _curr_pos.joint_1 = RoboticArmNode::constrain(_curr_pos.joint_1, -90, 90);
  _curr_pos.joint_2 += _curr_speed.joint_2;
  _curr_pos.joint_2 = RoboticArmNode::constrain(_curr_pos.joint_2, -90, 90);
  _curr_pos.joint_3 += _curr_speed.joint_3;
  _curr_pos.joint_3 = RoboticArmNode::constrain(_curr_pos.joint_3, -70, 90);
  _curr_pos.joint_4 += _curr_speed.joint_4;
  _curr_pos.joint_4 = RoboticArmNode::constrain(_curr_pos.joint_4, -90, 90);
  _curr_pos.joint_5 += _curr_speed.joint_5;
  _curr_pos.joint_5 = RoboticArmNode::constrain(_curr_pos.joint_5, -35, 90);
  _pub_arm_pos.publish(_curr_pos);
}



void RoboticArmNode::sub_speed_joints_callback(const std_msgs::Float64MultiArray& msg)
{
  if(msg.data.size() != 6)
  {
    ROS_WARN("Invalid number of elements do nothing");
    return;
  }
  _curr_speed.joint_0 = std::round(msg.data[0] * static_cast<double>(_max_speed));
  _curr_speed.joint_1 = std::round(msg.data[1] * static_cast<double>(_max_speed));
  _curr_speed.joint_2 = std::round(msg.data[2] * static_cast<double>(_max_speed));
  _curr_speed.joint_3 = std::round(msg.data[3] * static_cast<double>(_max_speed));
  _curr_speed.joint_4 = std::round(msg.data[4] * static_cast<double>(_max_speed));
  _curr_speed.joint_5 = std::round(msg.data[5] * static_cast<double>(_max_speed));

}

void RoboticArmNode::sub_speed_tcp_callback(const std_msgs::Float64MultiArray& msg)
{
  if(msg.data.size() != 3)
  {
    ROS_WARN("Invalid number of elements do nothing");
    return;
  }
}

bool RoboticArmNode::srv_set_stand_by_callback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res)
{
  _curr_pos = RoboticArmNode::get_stand_by_msg();
  _pub_arm_pos.publish(_curr_pos);
  return true;
}

bool RoboticArmNode::srv_set_active_callback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res)
{
  _curr_pos = RoboticArmNode::get_active_msg();
  _pub_arm_pos.publish(_curr_pos);
  return true;
}

// ------------- main ---------------
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "francor_robotic_arm_node");
  ros::NodeHandle nh("~");

  RoboticArmNode node;
  node.start();
}
