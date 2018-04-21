#ifndef ROBOTICARMNODE_H_
#define ROBOTICARMNODE_H_

#include <iostream>
#include <cmath>


#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <francor_msgs/RoboticArmCmd.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Empty.h>



class RoboticArmNode
{

public:
  RoboticArmNode();
  virtual ~RoboticArmNode();

  /**
     *
     * @brief
     *
     * @return  void
     */
  void start();

private: //functions
  /**
     *
     * @brief this function containts the main working loop
     *
     * @param[in,out]  void
     *
     * @return 		   void
     */
  void run();

  void loop_callback(const ros::TimerEvent& e);

  //void subCallback(const ROS_PACK::MESSAGE& msg);

  void sub_speed_joints_callback(const std_msgs::Float64MultiArray& msg);
  void sub_speed_tcp_callback(const std_msgs::Float64MultiArray& msg);

  bool srv_set_stand_by_callback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);
  bool srv_set_active_callback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);

  //a la arduino
  static inline double rescale(const double x, const double in_min, const double in_max, const double out_min, const double out_max)
  {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  static inline double constrain(const double vel, const double low, const double high)
  {
    return vel < low ? low : (vel > high ? high : vel);
  } 

  static inline francor_msgs::RoboticArmCmd get_stand_by_msg()
  {
    francor_msgs::RoboticArmCmd msg;
    msg.joint_0 = 0;
    msg.joint_1 = 90;
    msg.joint_2 = 90;
    msg.joint_3 = 0;
    msg.joint_4 = 0;
    msg.joint_5 = 0;
    return msg;
  }

  static inline francor_msgs::RoboticArmCmd get_active_msg()
  {
    francor_msgs::RoboticArmCmd msg;
    msg.joint_0 = 0;
    msg.joint_1 = 0;
    msg.joint_2 = 0;
    msg.joint_3 = 0;
    msg.joint_4 = 0;
    msg.joint_5 = 0;
    return msg;
  }

  //void dynreconfig_callback(RoboticArmNode::RoboticArmNodeConfig &config, uint32_t level);
private: //dataelements
  ros::NodeHandle _nh;

  ros::Publisher _pub_arm_pos;

  ros::Subscriber _sub_tcp_speed;
  ros::Subscriber _sub_speed_joints;

  ros::ServiceServer _srv_set_stand_by;
  ros::ServiceServer _srv_set_active;

  francor_msgs::RoboticArmCmd _curr_pos;
  francor_msgs::RoboticArmCmd _curr_speed;

  double _rate; 

  int _max_speed;

  ros::Timer _loopTimer;
};

#endif  //ROBOTICARMNODE_H_
