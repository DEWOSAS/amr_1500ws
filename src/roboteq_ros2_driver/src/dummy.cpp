#include <ros/ros.h>
#include <ros/console.h>
#include <serial/serial.h>
#include <signal.h>
#include <string>
#include <sstream>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

#include "roboteq_diff_msgs/DiffDriveEncoder.h"

#define DELTAT(_nowtime,_thentime) ((_thentime>_nowtime)?((0xffffffff-_thentime)+_nowtime):(_nowtime-_thentime))
#define NORMALIZE(_z) atan2(sin(_z), cos(_z))

void mySigintHandler(int sig)
{
  ROS_INFO("Received SIGINT signal, shutting down...");
  ros::shutdown();
}

uint32_t millis()
{
  ros::WallTime walltime = ros::WallTime::now();
  return (uint32_t)(walltime.toNSec()/1000000);
}

class MainNode
{

public:
  MainNode();

public:
  // cmd_vel subscriber
  void cmdvel_setup();
  void cmdvel_send();
  void cmdvel_callback( const geometry_msgs::Twist& twist_msg);
  void query_cmd_callback( const std_msgs::String::ConstPtr& query_cmd_msg);

  // odom publisher
  void odom_setup();
  void odom_loop();
  void odom_publish();

  int run();

protected:
  ros::NodeHandle nh;

  serial::Serial controller;

  // cmd_vel subscriber
  ros::Subscriber cmdvel_sub;
  ros::Subscriber query_cmd_sub;

  // odom publisher
  geometry_msgs::TransformStamped tf_msg;
  tf::TransformBroadcaster odom_broadcaster;
  nav_msgs::Odometry odom_msg;
  roboteq_diff_msgs::DiffDriveEncoder encoder_msg;
  ros::Publisher odom_pub;
  ros::Publisher encoder_pub;

  // buffer for reading encoder counts
  int odom_idx;
  int recv_count;
  char odom_buf[24];

  // toss out initial encoder readings
  char odom_encoder_toss;

  int32_t odom_encoder_left;
  int32_t odom_encoder_right;

  float odom_x;
  float odom_y;
  float odom_yaw;
  float odom_last_x;
  float odom_last_y;
  float odom_last_yaw;

  uint32_t odom_last_time;

  // settings
  bool pub_odom_tf;
  std::string odom_frame;
  std::string base_frame;
  std::string cmdvel_topic;
  std::string odom_topic;
  std::string port;
  int baud;
  double wheel_circumference;
  double track_width;
  int encoder_ppr;
  int encoder_cpr;
  double max_amps;
  int max_rpm;
  int gear_ratio;
  std::string query_cmd_topic;
  float right_speed;
  float left_speed;
  int32_t right_rpm;
  int32_t left_rpm;
};

MainNode::MainNode() : 
  odom_idx(0),
  recv_count(0),
  odom_encoder_toss(5),
  odom_encoder_left(0),
  odom_encoder_right(0),
  odom_x(0.0),
  odom_y(0.0),
  odom_yaw(0.0),
  odom_last_x(0.0),
  odom_last_y(0.0),
  odom_last_yaw(0.0),
  odom_last_time(0),
  pub_odom_tf(true),
  baud(115200),
  wheel_circumference(0),
  track_width(0),
  encoder_ppr(0),
  encoder_cpr(0),
  max_amps(0.0),
  max_rpm(0),
  gear_ratio(0),
  right_speed(0.0),
  left_speed(0.0),
  right_rpm(0),
  left_rpm(0)
{
  // CBA Read local params (from launch file)
  ros::NodeHandle nhLocal("~");
  nhLocal.param("pub_odom_tf", pub_odom_tf, true);
  ROS_INFO_STREAM("pub_odom_tf: " << pub_odom_tf);
  nhLocal.param<std::string>("odom_frame", odom_frame, "odom");
  ROS_INFO_STREAM("odom_frame: " << odom_frame);
  nhLocal.param<std::string>("base_frame", base_frame, "base_link");
  ROS_INFO_STREAM("base_frame: " << base_frame);
  nhLocal.param<std::string>("cmdvel_topic", cmdvel_topic, "cmd_vel");
  ROS_INFO_STREAM("cmdvel_topic: " << cmdvel_topic);
  nhLocal.param<std::string>("odom_topic", odom_topic, "odom");
  ROS_INFO_STREAM("odom_topic: " << odom_topic);
  nhLocal.param<std::string>("port", port, "/dev/ttyACM0");
  ROS_INFO_STREAM("port: " << port);
  nhLocal.param("baud", baud, 115200);
  ROS_INFO_STREAM("baud: " << baud);
  nhLocal.param("wheel_circumference", wheel_circumference, 0.4712389);
  ROS_INFO_STREAM("wheel_circumference: " << wheel_circumference);
  nhLocal.param("track_width", track_width, 0.48);
  ROS_INFO_STREAM("track_width: " << track_width);
  nhLocal.param("encoder_ppr", encoder_ppr, 50000);
  ROS_INFO_STREAM("encoder_ppr: " << encoder_ppr);
  nhLocal.param("encoder_cpr", encoder_cpr, 200000);
  ROS_INFO_STREAM("encoder_cpr: " << encoder_cpr);
  nhLocal.param("max_amps", max_amps, 5.0);
  ROS_INFO_STREAM("max_amps: " << max_amps);
  nhLocal.param("max_rpm", max_rpm, 100);
  ROS_INFO_STREAM("max_rpm: " << max_rpm);
  nhLocal.param("gear_ratio", gear_ratio, 20);
  ROS_INFO_STREAM("gear_ratio: " << gear_ratio);
  nhLocal.param<std::string>("query_cmd_topic", query_cmd_topic, "query_cmd");
  ROS_INFO_STREAM("query_cmd_topic: " << query_cmd_topic);
}

void MainNode::query_cmd_callback( const std_msgs::String::ConstPtr& query_cmd_msg)
{
  std::stringstream command;
  command  << "!" << query_cmd_msg->data << "\r";
  ROS_INFO_STREAM("Writing to serial port : !" << query_cmd_msg->data);
  controller.write(command.str());
  controller.flush();
}

void MainNode::cmdvel_callback( const geometry_msgs::Twist& twist_msg)
{
  if (left_rpm == 0 and right_rpm == 0){
    if (twist_msg.linear.x != 0 or twist_msg.angular.z != 0){
      controller.write("!DS 03\r");
    }
  }
  // wheel speed (m/s)
  right_speed = twist_msg.linear.x + track_width * twist_msg.angular.z / 2.0;
  left_speed = twist_msg.linear.x - track_width * twist_msg.angular.z / 2.0;
}

void MainNode::cmdvel_send()
{
  // motor speed (rpm)
  right_rpm = right_speed / wheel_circumference * 60.0 * gear_ratio;
  left_rpm = left_speed / wheel_circumference * 60.0 * gear_ratio;
  
  // send to controller
  std::stringstream right_cmd;
  std::stringstream left_cmd;
  right_cmd << "!S 1 " << right_rpm << "\r";
  left_cmd << "!S 2 " << left_rpm << "\r";
  controller.write(right_cmd.str());
  controller.write(left_cmd.str());
  controller.flush();

  if (abs(right_speed)<0.1) {
    right_speed = 0;
  }
  else{
    if (right_speed>0){
      right_speed = right_speed-0.1;
    }
    else{
      right_speed = right_speed+0.1;
    }
  }

  if (abs(left_speed)<0.1) {
    left_speed = 0;
  }
  else{
    if (left_speed>0){
      left_speed = left_speed-0.1;
    }
    else{
      left_speed = left_speed+0.1;
    }
  }
  
}

void MainNode::cmdvel_setup()
{
  controller.flush();

  // stop motors
  controller.write("!G 1 0\r");
  controller.write("!G 2 0\r");
  controller.write("!S 1 0\r");
  controller.write("!S 2 0\r");
  controller.flush();

  // clear break
  controller.write("!DS 03\r");

  // disable echo
  controller.write("^ECHOF 1\r");
  controller.flush();

  // enable watchdog timer (1000 ms)
  controller.write("^RWD 1000\r");
  controller.flush();

  ROS_INFO_STREAM("Subscribing to topic " << query_cmd_topic);
  query_cmd_sub = nh.subscribe(query_cmd_topic, 3, &MainNode::query_cmd_callback, this);
  ROS_INFO_STREAM("Subscribing to topic " << cmdvel_topic);
  cmdvel_sub = nh.subscribe(cmdvel_topic, 3, &MainNode::cmdvel_callback, this);
}

void MainNode::odom_setup()
{
  if ( pub_odom_tf )
  {
  ROS_INFO("Broadcasting odom tf");
  }

  ROS_INFO_STREAM("Publishing to topic " << odom_topic);
  odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 3);
  encoder_pub = nh.advertise<roboteq_diff_msgs::DiffDriveEncoder>("diff_encoder", 3);

  tf_msg.header.seq = 0;
  tf_msg.header.frame_id = odom_frame;
  tf_msg.child_frame_id = base_frame;

  odom_msg.header.seq = 0;
  odom_msg.header.frame_id = odom_frame;
  odom_msg.child_frame_id = base_frame;
  odom_last_time = millis();

  encoder_msg.header.seq = 0;
  encoder_msg.wheel_space = track_width;
  encoder_msg.wheel_circumference = wheel_circumference;
  encoder_msg.encoder_cpr = encoder_cpr;
  encoder_msg.linear_vel = 0;
  encoder_msg.angular_vel = 0;
}

void MainNode::odom_loop()
{
  // ROS_INFO("Hello Odom");

  uint32_t nowtime = millis();
  uint32_t odom_start_time = millis();

  controller.write("?CR\r");
  controller.flush();
  odom_idx = 0;
  recv_count = 0;

  // read sensor data stream from motor controller
  if (controller.available())
  {
  // Clear Recive Data Buffer
  // std::string msg;
  // msg = controller.read(controller.available());
  // controller.read(controller.available());
  // ROS_DEBUG_STREAM("Message :" << msg);

  char ch = 0;
  while(recv_count < 24)
  {
    // ROS_INFO("Looping");
    nowtime = millis();
    ch = 0;
    if ( controller.read((uint8_t*)&ch, 1) != 0 )
    {
      recv_count++;
      if (ch == '\r')
      {
        odom_buf[odom_idx++] = 0;
        if ( odom_buf[0] == 'C' && odom_buf[1] == 'R' && odom_buf[2] == '=' )
        {
          int delim;
          for ( delim = 3; delim < odom_idx; delim++ )
          {
            // if ( odom_encoder_toss > 0 )
            // {
            //  --odom_encoder_toss;
            //  break;
            // }
            if (odom_buf[delim] == ':')
            {
              odom_buf[delim] = 0;
              odom_encoder_right = (int32_t)strtol(odom_buf+3, NULL, 10);
              odom_encoder_left = (int32_t)strtol(odom_buf+delim+1, NULL, 10);
              odom_publish();
              return;
            }
          }
        }
        odom_idx = 0;
      }
      else if ( odom_idx < (sizeof(odom_buf)-1) )
      {
        odom_buf[odom_idx++] = ch;
      }
    }
  }
  }
}

void MainNode::odom_publish()
{
  // determine delta time in seconds
  uint32_t nowtime = millis();
  float dt = (float)DELTAT(nowtime,odom_last_time) / 1000.0;
  odom_last_time = nowtime;
  
  // determine deltas of distance and angle
  encoder_msg.header.seq++;
  encoder_msg.header.stamp = ros::Time::now();
  encoder_msg.count_time = dt;
  encoder_msg.left_encoder_count = (float)odom_encoder_left;
  encoder_msg.left_wheel_distance = (float)odom_encoder_left / (float)encoder_cpr * wheel_circumference;
  encoder_msg.right_encoder_count = (float)odom_encoder_right;
  encoder_msg.right_wheel_distance = (float)odom_encoder_right / (float)encoder_cpr * wheel_circumference;

  float linear = (encoder_msg.right_wheel_distance + encoder_msg.left_wheel_distance) / 2.0 ;
  float angular = (encoder_msg.right_wheel_distance - encoder_msg.left_wheel_distance) / track_width ;
  float linear_vel = linear/dt;
  float angular_vel = angular/dt;
  
  encoder_msg.linear_acc = (linear_vel-encoder_msg.linear_vel)/dt;
  encoder_msg.linear_vel = linear_vel;
  encoder_msg.angular_acc = (angular_vel-encoder_msg.angular_vel)/dt;
  encoder_msg.angular_vel = angular_vel;
  encoder_pub.publish(encoder_msg);

  // Update odometry
  odom_x += linear * cos(odom_yaw);        // m
  odom_y += linear * sin(odom_yaw);        // m
  odom_yaw = NORMALIZE(odom_yaw + angular);  // rad
  
  // Calculate velocities
  float vx = (odom_x - odom_last_x) / dt;
  float vy = (odom_y - odom_last_y) / dt;
  float vyaw = (odom_yaw - odom_last_yaw) / dt;

  odom_last_x = odom_x;
  odom_last_y = odom_y;
  odom_last_yaw = odom_yaw;

  geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(odom_yaw);

  if ( pub_odom_tf )
  {
  tf_msg.header.seq++;
  tf_msg.header.stamp = encoder_msg.header.stamp;
  tf_msg.transform.translation.x = odom_x;
  tf_msg.transform.translation.y = odom_y;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation = quat;
  odom_broadcaster.sendTransform(tf_msg);
  }
  odom_msg.header.seq++;
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.pose.pose.position.x = odom_x;
  odom_msg.pose.pose.position.y = odom_y;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = quat;
  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = vy;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = vyaw;
  odom_pub.publish(odom_msg);
}

int MainNode::run()
{
  ROS_INFO("Beginning setup...");
  serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
  controller.setPort(port);
  controller.setBaudrate(baud);
  controller.setTimeout(timeout);

  // TODO: support automatic re-opening of port after disconnection
  while ( ros::ok() )
  {
    ROS_INFO_STREAM("Opening serial port on " << port << " at " << baud << "..." );
      try
      {
     controller.open();
     if ( controller.isOpen() )
     {
    ROS_INFO("Successfully opened serial port");
    break;
  }
  }
  catch (serial::IOException e)
{
  ROS_WARN_STREAM("serial::IOException: " << e.what());
}
ROS_WARN("Failed to open serial port");
sleep(5);
  }

  ros::Rate loop_rate(10);
  cmdvel_setup();
  odom_setup();

  ROS_INFO("Beginning looping...");

  while (ros::ok())
  {
    odom_loop();
    cmdvel_send();
    ros::spinOnce();
    loop_rate.sleep();
  }

  if ( controller.isOpen() )
  {
    controller.close();
  }
  ROS_INFO("Exiting");
  return 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "main_node");
  MainNode node;
  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, mySigintHandler);
  return node.run();
}