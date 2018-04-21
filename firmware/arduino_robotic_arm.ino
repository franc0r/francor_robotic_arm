// #include "Arduino.h"
#include <ros.h>

#include <Servo.h>

// #include <std_msgs/String.h>
// #include <francor_msgs/SensorHeadCmd.h>
#include <francor_msgs/RoboticArmCmd.h>
#include <std_msgs/Int16.h>

class FrancorServo{
public:
  FrancorServo(const int min = -90, const int max = 90, const int min_us = 500, const int max_us = 2400) :
    _MIN_US(min_us),
    _MAX_US(max_us),
    _min(::map(min, -90, 90, _MIN_US, _MAX_US)),
    _max(::map(max, -90, 90, _MIN_US, _MAX_US))
  {
    _speed = 0;
  }
  void attach(const int servo)
  {
    constrain(0,1,2);
    _servo.attach(servo);
  }

  /**
   * @brief Set the Pos object
   * 
   * @param pos  -> min = -90, max = +90
   */
  void setPos(const int pos)
  {
    _speed  = 0;
    _curr_pos = ::map(pos, -90, 90, _MIN_US, _MAX_US);
    _curr_pos = constrain(_curr_pos, _min, _max);

    this->write(_curr_pos);
  }
  void setSpeed(const int speed)
  {
    _speed  = speed;
  }

  void tick()
  {
    _curr_pos  += _speed;

    _curr_pos = constrain(_curr_pos, _min, _max);

    this->write(_curr_pos);
  }

  const int getPos() const
  {
    return ::map(_curr_pos, _MIN_US, _MAX_US, -90, 90);
  }

  const int getSpeed() const
  {
    return _speed;
  }

private:

  void write(const int pos)
  {
    _servo.writeMicroseconds(pos);
  }

  const int _MIN_US;
  const int _MAX_US;

  const int _min;
  const int _max; 

  Servo _servo;

  int _speed;

  int _curr_pos;
};


class FrancorTwinServo{
public:
  FrancorTwinServo(const int min = -90, const int max = 90) :
    // _servo_0(-1 * min, -1 * max),
    _servo_0(     min,      max, 500, 2400),
    _servo_1(     min,      max, 500, 2400)
  { }

  void attach(const int servo_pin_0, const int servo_pin_1)
  {
    _servo_0.attach(servo_pin_0);
    _servo_1.attach(servo_pin_1);
    _servo_0.setPos(-90);
    _servo_1.setPos(90);
  }

  void tick()
  {
    _servo_0.tick();
    _servo_1.tick();
  }

  void setPos(const int pos)
  {
    _servo_0.setPos(-1 * pos);
    // _servo_0.setPos( pos);
    _servo_1.setPos(     pos);
  }

  void setSpeed(const int speed)
  {
    _servo_0.setSpeed(-1 * speed);
    // _servo_0.setSpeed(speed);
    _servo_1.setSpeed(     speed);
  }

  int getPos_0() const
  { 
    return _servo_0.getPos();
  }
  int getPos_1() const
  {
    return _servo_1.getPos();
  }

private:
  FrancorServo _servo_0;
  FrancorServo _servo_1;

};

// FrancorServo g_servo;
FrancorServo g_servo_0;             //servo 0
FrancorTwinServo g_twin_servo_1;    //servo 1 and 2
FrancorServo g_servo_2;             //servo 3
FrancorServo g_servo_3;             //servo 4
FrancorServo g_servo_4;             //servo 5
FrancorServo g_servo_5;             //servo 6

ros::NodeHandle  nh;
// std_msgs::String log_msg;
// francor_msgs::SensorHeadCmd sensor_head_pos;
// std_msgs::Int16 servo_pos_0;
// std_msgs::Int16 servo_pos_1;
// ros::Publisher pub_log("robotic_arm/log", &log_msg);
// ros::Publisher pub_pos("robotic_arm/pos", &sensor_head_pos);
// ros::Publisher pub_pos_0("robotic_arm/pos_0", &servo_pos_0);
// ros::Publisher pub_pos_1("robotic_arm/pos_1", &servo_pos_1);
// -- functions --

// void log(const char* msg)
// {
//   log_msg.data = msg;
//   pub_log.publish(&log_msg);
// }

void sub_set_speed_callback(const std_msgs::Int16& msg)
{
  digitalWrite(13, HIGH-digitalRead(13));
  // log_msg.data = "got set_speed_callback()";
  // pub_log.publish(&log_msg);

  g_twin_servo_1.setSpeed(msg.data);
}

void sub_set_pos_callback(const std_msgs::Int16& msg)
{
  digitalWrite(13, HIGH-digitalRead(13));
  // log_msg.data = "got set_pos_callback()";
  // pub_log.publish(&log_msg);

  //g_twin_servo_1.setPos(msg.data);
  //g_servo_0.setPos(msg.data);
  g_servo_2.setPos(msg.data);
}

void sub_set_joints_pos_callback(const francor_msgs::RoboticArmCmd& msg)
{
  g_servo_0.setPos     (msg.joint_0);
  g_twin_servo_1.setPos(msg.joint_1);
  g_servo_2.setPos     (msg.joint_2);
  g_servo_3.setPos     (msg.joint_3);
  g_servo_4.setPos     (msg.joint_4);
  g_servo_5.setPos     (msg.joint_5);
}

// void sub_set_joints_speed_callback(const francor_msgs::RoboticArmCmd& msg)
// {

// }

// -- subs --

//ros::Subscriber<std_msgs::Int16> sub_set_speed("robotic_arm/set_speed", &sub_set_speed_callback);
//ros::Subscriber<std_msgs::Int16> sub_set_pos("robotic_arm/set_pos", &sub_set_pos_callback);
ros::Subscriber<francor_msgs::RoboticArmCmd> sub_set_joints_pos("robotic_arm/set_joints_pos", &sub_set_joints_pos_callback);
// ros::Subscriber<francor_msgs::RoboticArmCmd> sub_set_joints_speed("robotic_arm/set_joints_speed", &sub_set_joints_speed_callback);


void setup()
{
  // Serial.begin(9600);
   g_servo_0.attach(2);
   g_twin_servo_1.attach(3,4);
   g_servo_2.attach(5);
   g_servo_3.attach(6);
   g_servo_4.attach(7);
   g_servo_5.attach(8);

   g_servo_0.setPos(0);
   g_servo_2.setPos(90);
   g_servo_3.setPos(0);
   g_servo_4.setPos(0);
   g_servo_5.setPos(0);

  nh.getHardware()->setBaud(57600);
  nh.initNode();
  // nh.advertise(pub_log);
  // nh.advertise(pub_pos_0);
  // nh.advertise(pub_pos_1);

  // nh.subscribe(sub_set_speed);
  // nh.subscribe(sub_set_pos);
  nh.subscribe(sub_set_joints_pos);
  // nh.subscribe(sub_set_joints_speed);
}


int g_cnt = 0;


void loop()
{
   g_servo_0.tick();
   g_twin_servo_1.tick();
   g_servo_2.tick();
   g_servo_3.tick();
   g_servo_4.tick();
   g_servo_5.tick();
  

  // if(g_cnt++ % 10 == 0)
  // {

  //   //pub current pos
  //   servo_pos_0.data = g_twin_servo_1.getPos_0();
  //   pub_pos_0.publish(&servo_pos_0);

  //   servo_pos_1.data = g_twin_servo_1.getPos_1();
  //   pub_pos_1.publish(&servo_pos_1);
  // }
  
  nh.spinOnce();

  delay(10);
}

