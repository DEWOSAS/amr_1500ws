#ifndef ROBOTEQ_ROS2_DRIVER__ROBOTEQ_ROS2_DRIVER_HPP_
#define ROBOTEQ_ROS2_DRIVER__ROBOTEQ_ROS2_DRIVER_HPP_

#include <math.h>
#include <unistd.h>

#include <cstdio>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "amr_interfaces/msg/error_flags.hpp"
#include "amr_interfaces/msg/wheel_encoder_state.hpp"

namespace Roboteq
{
  class Roboteq : public rclcpp::Node
  {
  public:
    explicit Roboteq(); //(nodeOptions options?)
    ~Roboteq();

  private:
    // class atributes
    // rclcpp::Node::SharedPtr nh{};
    std::unique_ptr<tf2_ros::TransformBroadcaster> odom_baselink_transform_;

    uint32_t starttime{};
    uint32_t hstimer{};
    uint32_t mstimer{};
    uint32_t lstimer{};
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr param_update_timer;

    // buffer for reading encoder counts
    unsigned int odom_idx{};
    char odom_buf[64]{};

    // toss out initial encoder readings
    char odom_encoder_toss{};

    int32_t odom_encoder_left{};
    int32_t odom_encoder_right{};

    int32_t odom_encoder_left_abs{0};
    int32_t odom_encoder_right_abs{0};

    float odom_x{};
    float odom_y{};
    float odom_yaw{};
    float odom_last_x{};
    float odom_last_y{};
    float odom_last_yaw{};

    uint32_t odom_last_time{};

    // settings
    bool pub_odom_tf{};
    std::string odom_frame{};
    std::string base_frame{};
    std::string cmdvel_topic{};
    std::string odom_topic{};
    std::string port{};
    int baud{};
    bool open_loop{};
    double wheel_circumference{};
    double track_width{};
    int encoder_ppr{};
    int encoder_cpr{};
    double max_amps{};
    int max_rpm{};
    float gear_ratio = {};
    // Test different odom msg memory
    // nav_msgs::msg::Odometry odom_msg{};
    nav_msgs::msg::Odometry odom_msg{};
    // geometry_msgs::msg::Twist twist_msg{};

    uint32_t error_ff_{0};
    uint32_t error_fs_{0};
    uint32_t error_fm1_{0};
    uint32_t error_fm2_{0};

    // เพิ่มเข้าไปใน private section:
    rclcpp::CallbackGroup::SharedPtr io_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
    rclcpp::CallbackGroup::SharedPtr cmdvel_cb_group_;

    rclcpp::TimerBase::SharedPtr io_timer_;
    rclcpp::TimerBase::SharedPtr param_update_timer_;


    //
    // cmd_vel subscriber
    //
    void cmdvel_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg);
    void cmdvel_setup();
    void cmdvel_loop();
    void cmdvel_run();

    //
    // odom publisher
    //
    void odom_setup();
    void odom_stream();
    void odom_loop();
    // void odom_hs_run();
    void odom_ms_run();
    void odom_ls_run();
    void odom_publish();
    void connect();

    void update_parameters();
    int run();

    void publish_error_flags();
    void serial_read_thread();

    // subscriber
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdvel_sub;

    // publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<amr_interfaces::msg::WheelEncoderState>::SharedPtr wheel_state_pub_;

    // error flags publisher
    rclcpp::Publisher<amr_interfaces::msg::ErrorFlags>::SharedPtr error_pub_;
  };

}

#endif
