#include "roboteq_ros2_driver/roboteq_ros2_driver.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include <iostream>

#include "std_msgs/msg/string.hpp"


#include <serial/serial.h>
#include <signal.h>
#include <string>
#include <sstream>

#define DELTAT(_nowtime, _thentime) ((_thentime > _nowtime) ? ((0xffffffff - _thentime) + _nowtime) : (_nowtime - _thentime))

// Define following to enable cmdvel debug output
#define _CMDVEL_DEBUG

// Define following to enable odom debug output
#define _ODOM_DEBUG

// Define following to publish additional sensor information; comment out to not publish

// #define _ODOM_SENSORS

// Define following to enable service for returning covariance
// #define _ODOM_COVAR_SERVER

#define NORMALIZE(_z) atan2(sin(_z), cos(_z))

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// change to ros2 libraries
// #include <tf2/Math/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>

serial::Serial controller;

uint32_t millis()
{
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}

namespace Roboteq
{
    Roboteq::Roboteq() : Node("roboteq_ros2_driver")
    {

        io_cb_group_      = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_cb_group_   = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cmdvel_cb_group_  = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        pub_odom_tf = this->declare_parameter("pub_odom_tf", false);
        odom_frame = this->declare_parameter("odom_frame", "odom");
        base_frame = this->declare_parameter("base_frame", "base_footprint");
        cmdvel_topic = this->declare_parameter("cmdvel_topic", "cmd_vel");
        odom_topic = this->declare_parameter("odom_topic", "/wheel_odom");
        port = this->declare_parameter("port", "/dev/ttyACM0");
        baud = this->declare_parameter("baud", 115200);
        open_loop = this->declare_parameter("open_loop", false);
        wheel_circumference = this->declare_parameter("wheel_circumference", 0.565486);
        track_width = this->declare_parameter("track_width", 0.815);
        encoder_ppr = this->declare_parameter("encoder_ppr", 50000);
        encoder_cpr = this->declare_parameter("encoder_cpr", 200000);
        max_amps = this->declare_parameter("max_amps", 9.5);
        max_rpm = this->declare_parameter("max_rpm", 300);
        gear_ratio = this->declare_parameter("gear_ratio", 20.0);

        // total_encoder_pulses=0;
        starttime = 0;
        hstimer = 0;
        mstimer = 0;
        odom_idx = 0;
        odom_encoder_toss = 5;
        odom_encoder_left = 0;
        odom_encoder_right = 0;
        odom_x = 0.0;
        odom_y = 0.0;
        odom_yaw = 0.0;
        odom_last_x = 0.0;
        odom_last_y = 0.0;
        odom_last_yaw = 0.0;
        odom_last_time = 0;

        odom_msg = nav_msgs::msg::Odometry();

        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        controller.setPort(port);
        controller.setBaudrate(baud);
        controller.setTimeout(timeout);
        // connect to serial port
        connect();
        // configure motor controller
        cmdvel_setup();
        odom_setup();
        //+
        //  odom publisher
        //
        odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 1);
        wheel_state_pub_ = this->create_publisher<amr_interfaces::msg::WheelEncoderState>("wheel_encoder_state", 10);


        // custom //
        error_pub_ = this->create_publisher<amr_interfaces::msg::ErrorFlags>(
            "motor_error_flags", 
            10                    
        );

        // 2) เตรียม SubscriptionOptions
        rclcpp::SubscriptionOptions io_sub_opts;
        io_sub_opts.callback_group = io_cb_group_;

        rclcpp::SubscriptionOptions cmdvel_sub_opts;
        cmdvel_sub_opts.callback_group = cmdvel_cb_group_;

        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::TimerBase::SharedPtr param_timer;

        //
        // cmd_vel subscriber
        //
        cmdvel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            cmdvel_topic, // topic name
            1,            // QoS history depth
            std::bind(&Roboteq::cmdvel_callback, this, std::placeholders::_1),
            cmdvel_sub_opts 
        );

        rclcpp::SubscriptionOptions cmdvel_opts;
        cmdvel_opts.callback_group = cmdvel_cb_group_;


        using namespace std::chrono_literals;

        // 1ms run loop timer
        timer_ = this->create_wall_timer(
        1ms,
        std::bind(&Roboteq::run, this),
        timer_cb_group_
        );

        // 1ms serial‐read + odom parser
        io_timer_ = this->create_wall_timer(
        1ms,
        std::bind(&Roboteq::odom_loop, this),
        io_cb_group_
        );

        // 1s parameter updater
        param_update_timer_ = this->create_wall_timer(
        1000ms,
        std::bind(&Roboteq::update_parameters, this),
        timer_cb_group_
        );


        odom_baselink_transform_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        geometry_msgs::msg::TransformStamped init_tf;
        init_tf.header.frame_id    = odom_frame;
        init_tf.child_frame_id     = base_frame;
        init_tf.transform.translation.x = 0.0;
        init_tf.transform.translation.y = 0.0;
        init_tf.transform.translation.z = 0.0;
        init_tf.transform.rotation.x = 0.0;
        init_tf.transform.rotation.y = 0.0;
        init_tf.transform.rotation.z = 0.0;
        init_tf.transform.rotation.w = 1.0;

        for (int i = 0; i < 3; ++i) {
        init_tf.header.stamp = this->get_clock()->now();
        odom_baselink_transform_->sendTransform(init_tf);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    void Roboteq::update_parameters()
    {
        // RCLCPP_INFO(this->get_logger(), "Parameters updated ...");
        this->get_parameter("pub_odom_tf", pub_odom_tf);
        this->get_parameter("odom_frame", odom_frame);
        this->get_parameter("base_frame", base_frame);
        this->get_parameter("cmdvel_topic", cmdvel_topic);
        this->get_parameter("odom_topic", odom_topic);
        this->get_parameter("port", port);
        this->get_parameter("baud", baud);
        this->get_parameter("open_loop", open_loop);
        this->get_parameter("wheel_circumference", wheel_circumference);
        this->get_parameter("track_width", track_width);
        this->get_parameter("encoder_ppr", encoder_ppr);
        this->get_parameter("encoder_cpr", encoder_cpr);
        this->get_parameter("max_amps", max_amps);
        this->get_parameter("max_rpm", max_rpm);
        this->get_parameter("gear_ratio", gear_ratio);
    }

    void Roboteq::connect()
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Opening serial port on " << port << " at " << baud << "...");
        try
        {
            controller.open();
            if (controller.isOpen())
            {
                RCLCPP_INFO(this->get_logger(), "Successfully opened serial port");
                return;
            }
        }
        catch (serial::IOException &e)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "serial::IOException: ");
            throw;
        }
        RCLCPP_WARN(this->get_logger(), "Failed to open serial port");
        sleep(5);
    }

    void Roboteq::cmdvel_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
    {                                                                                  
        // wheel speed (m/s)
        float right_speed = twist_msg->linear.x + track_width * twist_msg->angular.z / 2.0;
        float left_speed = twist_msg->linear.x - track_width * twist_msg->angular.z / 2.0;

        std::stringstream right_cmd;
        std::stringstream left_cmd;

        // motor speed (rpm)
        int32_t right_rpm = right_speed * gear_ratio / wheel_circumference * 60.0;
        int32_t left_rpm = left_speed * gear_ratio / wheel_circumference * 60.0;

        // right_rpm = std::clamp(right_rpm, -static_cast<int32_t>(max_rpm), static_cast<int32_t>(max_rpm));
        // left_rpm  = std::clamp(left_rpm,  -static_cast<int32_t>(max_rpm), static_cast<int32_t>(max_rpm));

        // debug
        RCLCPP_INFO(this->get_logger(),
            "Received cmd_vel → linear: %.3f m/s | angular: %.3f rad/s\n"
            "→ Left RPM: %d | Right RPM: %d",
            twist_msg->linear.x, twist_msg->angular.z,
            left_rpm, right_rpm);

        right_cmd << "!S 1 " << right_rpm << "\r";
        left_cmd << "!S 2 " << left_rpm << "\r";

// write cmd to motor controller
#ifndef _CMDVEL_FORCE_RUN
        controller.write(right_cmd.str());
        controller.write(left_cmd.str());
        controller.flush();
#endif
    }
    void Roboteq::cmdvel_setup()
    {
        RCLCPP_INFO(this->get_logger(), "configuring motor controller...");

        // ชุดคำสั่งหยุดมอเตอร์
        controller.write("!G 1 0\r");
        controller.write("!G 2 0\r");
        controller.write("!S 1 0\r");
        controller.write("!S 2 0\r");
        controller.flush();

        // ชุดคำสั่ง clear break + reset encoder
        controller.write("!DS 0\r");
        controller.write("!C 1 0\r");
        controller.write("!C 2 0\r");
        controller.flush();

        // ชุดคำสั่ง disable echo + watchdog
        controller.write("^ECHOF 1\r");
        controller.write("^RWD 1000\r");
        controller.flush();

        // เปลี่ยนโหมดการทำงานเป็น closed loop velocity
        // controller.write("^MMOD 1 1\r");
        // controller.write("^MMOD 2 1\r");
        // controller.flush();

        // ชุดคำสั่ง data stream ที่คุณเพิ่ม
        controller.write("# C\r");
        controller.write("/\"E=\",\",\"?CR_?C_# 30\r");
        controller.write("/\"F=\",\",\"?FF_?FS_?FM_# 1000\r");
        controller.flush();
    }

    void Roboteq::cmdvel_loop()
    {
    }

    void Roboteq::cmdvel_run()
    {
#ifdef _CMDVEL_FORCE_RUN
        if (open_loop)
        {
            controller.write("!G 1 100\r");
            controller.write("!G 2 100\r");
        }
        else
        {
            std::stringstream right_cmd;
            std::stringstream left_cmd;
            right_cmd << "!S 1 " << (int)(max_rpm * 0.1) << "\r";
            left_cmd << "!S 2 " << (int)(max_rpm * 0.1) << "\r";
            controller.write(right_cmd.str());
            controller.write(left_cmd.str());
        }
        controller.flush();
#endif
    }

    void Roboteq::odom_setup()
    {
        RCLCPP_INFO(this->get_logger(), "setting up odom...");
        // TODO: implement tf2 broadcaster
        //  RCLCPP_INFO(this->get_logger(), "Broadcasting odom tf"); // might use this-> instead of node
        odom_baselink_transform_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // RCLCPP_INFO_STREAM(get_logger(), "Publishing to topic " << odom_topic);

        // odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 1000);
        // odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 1000)

        // Set up the header
        /*
        tf_msg.header.stamp = 0;
        tf_msg.header.frame_id = odom_frame;
        tf_msg.child_frame_id = base_frame;
        */

        odom_msg.header.stamp = this->get_clock()->now();

        odom_msg.header.frame_id = odom_frame;
        odom_msg.child_frame_id = base_frame;
        /*
            auto message = nav_msgs::msg::Odometry();
        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = "odom";
        message.pose.pose.position.x = new_state.x;
        message.pose.pose.position.y = new_state.y;
        message.pose.pose.orientation.x = quat.x();
        message.pose.pose.orientation.y = quat.y();
        message.pose.pose.orientation.z = quat.z();
        message.pose.pose.orientation.w = quat.w();
        */
        // Set up the pose covariance
        for (size_t i = 0; i < 36; i++)
        {
            odom_msg.pose.covariance[i] = 0;
            odom_msg.twist.covariance[i] = 0;
        }

        odom_msg.pose.covariance[7] = 0.1;
        odom_msg.pose.covariance[14] = 0.1;
        odom_msg.pose.covariance[21] = 0.1;
        odom_msg.pose.covariance[28] = 0.1;
        odom_msg.pose.covariance[35] = 0.1;

        // Set up the twist covariance
        odom_msg.twist.covariance[0] = 0.1;
        odom_msg.twist.covariance[7] = 0.1;
        odom_msg.twist.covariance[14] = 0.1;
        odom_msg.twist.covariance[21] = 0.1;
        odom_msg.twist.covariance[28] = 0.1;
        odom_msg.twist.covariance[35] = 0.1;

        // Set up the transform message: move to odom_publish

        tf2::Quaternion q;
        q.setRPY(0, 0, odom_yaw);

        // tf_msg.transform.translation.x = x;
        // tf_msg.transform.translation.y = y;
        // tf_msg.transform.translation.z = 0.0;
        // tf_msg.transform.rotation.x = q.x();
        // tf_msg.transform.rotation.y = q.y();
        // tf_msg.transform.rotation.z = q.z();
        // tf_msg.transform.rotation.w = q.w();

        // start encoder streaming
        RCLCPP_INFO_STREAM(this->get_logger(), "covariance set");
        RCLCPP_INFO_STREAM(this->get_logger(), "odometry stream starting...");
        // odom_stream();

        odom_last_time = millis();
#ifdef _ODOM_SENSORS
        current_last_time = millis();
#endif
    }

    // Odom msg streams
    void Roboteq::odom_stream()
    {

#ifdef _ODOM_SENSORS
        // start encoder and current output (30 hz)
        // doubling frequency since one value is output at each cycle
        //  controller.write("# C_?CR_?BA_# 17\r");
        // start encoder, current and voltage output (30 hz)
        // tripling frequency since one value is output at each cycle
        controller.write("# C_?CR_?BA_?V_# 11\r");
#else
        //  // start encoder output (10 hz)
        //  controller.write("# C_?CR_# 100\r");
        // start encoder output (30 hz)
        
        // auto sending motor feedback
        // controller.write("# C_?F_# 33\r");

        //auto sending motor encoders feedback
        // controller.write("# C_?CR_?C_?FF_?FS_?FM_# 1\r");

        controller.write("# C\r");
        controller.write("/\"E=\",\",\"?CR_?C_# 1000\r");
        controller.write("/\"F=\",\",\"?FF_?FS_?FM_# 3000\r");
        

#endif
        controller.flush();
    }

    void Roboteq::odom_loop()
    {

        uint32_t nowtime = millis();

        // if we haven't received encoder counts in some time then restart streaming
        if (DELTAT(nowtime, odom_last_time) >= 1000)
        {
            // odom_stream();
            odom_last_time = nowtime;
        }

        if (controller.available())
        {
            char ch = 0;
            if (controller.read((uint8_t *)&ch, 1) == 0)
                return;

            if (ch == '\r')
            {
                odom_buf[odom_idx] = '\0';
                std::string line(odom_buf, odom_idx);
                if (odom_idx > 0) {

                    // DEBUG
                    
                    // RCLCPP_INFO(this->get_logger(),
                    // "Received line [%.*s] (%u bytes)",
                    // static_cast<int>(odom_idx), odom_buf, odom_idx);
                }

                // // (A) อ่าน Absolute Encoder Count
                // if (odom_buf[0]=='C' && odom_buf[1]=='=' && odom_buf[2] != 'R')
                // {
                //     // รูปแบบ "C=right:left"
                //     char *p = strchr(odom_buf + 2, ':');
                //     if (p) {
                //         *p = '\0';
                //         odom_encoder_right_abs = atoi(odom_buf + 2);
                //         odom_encoder_left_abs  = atoi(p + 1);
                //     }
                // }
                // // (B) อ่าน Relative Encoder Count แล้วคำนวณ odometry
                // else if (odom_buf[0]=='C' && odom_buf[1]=='R' && odom_buf[2]=='=')
                // {
                //     unsigned int delim;
                //     for (delim = 3; delim < odom_idx; delim++)
                //     {
                //         if (odom_encoder_toss > 0)
                //         {
                //             --odom_encoder_toss;
                //             break;
                //         }
                //         if (odom_buf[delim] == ':')
                //         {
                //             odom_buf[delim] = '\0';
                //             odom_encoder_right = strtol(odom_buf + 3, NULL, 10);
                //             odom_encoder_left  = strtol(odom_buf + delim + 1, NULL, 10);
                //             odom_publish();
                //             break;
                //         }
                //     }
                // }

                if (line.rfind("E=", 0) == 0) {
                    std::string payload = line.substr(2);      
                    auto comma = payload.find(',');
                    if (comma != std::string::npos) {
                        std::string abs_pair = payload.substr(0, comma);        
                        std::string rel_pair = payload.substr(comma + 1);       

                        auto c1 = abs_pair.find(':');
                        auto c2 = rel_pair.find(':');

                        if (c1 != std::string::npos && c2 != std::string::npos) {
                            try {
                                odom_encoder_right = std::stoi(abs_pair.substr(0, c1));
                                odom_encoder_left  = std::stoi(abs_pair.substr(c1 + 1));
                                odom_encoder_right_abs = static_cast<int32_t>(std::stoll(rel_pair.substr(0, c2)));
                                odom_encoder_left_abs  = static_cast<int32_t>(std::stoll(rel_pair.substr(c2 + 1)));

                                odom_publish();
                            } catch (const std::invalid_argument& e) {
                                RCLCPP_WARN(this->get_logger(), "Invalid E= line (stoi failed): '%s'", line.c_str());
                            }
                        } else {
                            RCLCPP_WARN(this->get_logger(), "Malformed E= line (missing ':'): '%s'", line.c_str());
                        }
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Malformed E= line (missing ','): '%s'", line.c_str());
                    }
                }
                else if (line.rfind("F=", 0) == 0) {
                    std::string payload = line.substr(2);  // ตัด "F=" ออก → "20,72,8:8"

                    std::vector<std::string> tokens;
                    std::stringstream ss(payload);
                    std::string item;
                    while (std::getline(ss, item, ',')) {
                        tokens.push_back(item);
                    }

                    if (tokens.size() >= 3) {
                        try {
                            error_ff_ = std::stoul(tokens[0], nullptr, 0); // "20"
                            error_fs_ = std::stoul(tokens[1], nullptr, 0); // "72"

                            auto fm_tokens = tokens[2];
                            auto colon = fm_tokens.find(':');
                            if (colon != std::string::npos) {
                                error_fm1_ = std::stoul(fm_tokens.substr(0, colon), nullptr, 0);
                                error_fm2_ = std::stoul(fm_tokens.substr(colon + 1), nullptr, 0);
                            } else {
                                RCLCPP_WARN(this->get_logger(), "Malformed FM flags (missing ':'): '%s'", fm_tokens.c_str());
                            }

                            publish_error_flags();
                        } catch (const std::exception& e) {
                            RCLCPP_WARN(this->get_logger(), "Failed to parse F= line: '%s'", line.c_str());
                        }
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Malformed F= line (expected 3 fields): '%s'", line.c_str());
                    }
                }

                odom_idx = 0;
            }
            else if (odom_idx < (sizeof(odom_buf) - 1))
            {
                odom_buf[odom_idx++] = ch;
            }
        }
    }
    void Roboteq::odom_publish()
    {
        geometry_msgs::msg::TransformStamped tf_msg;

        // determine delta time in seconds
        uint32_t nowtime = millis();
        float dt = (float)DELTAT(nowtime, odom_last_time) / 1000.0;
        odom_last_time = nowtime;
        // total_encoder_pulses+=odom_encoder_right;
        // determine deltas of distance and angle

        // float linear = ((float)odom_encoder_right * wheel_circumference / (60 * gear_ratio) + (float)odom_encoder_left * wheel_circumference / (60 * gear_ratio)) / 2;
        // float angular = ((float)odom_encoder_right * wheel_circumference / (60 * gear_ratio) - (float)odom_encoder_left * wheel_circumference / (60 * gear_ratio)) / track_width;

        // Update odometry
        // odom_x += linear * dt * cos(odom_yaw);         // m
        // odom_y += linear * dt * sin(odom_yaw);         // m
        // odom_yaw = NORMALIZE(odom_yaw + angular * dt); // rad

        // // Calculate velocities
        // float vx = (odom_x - odom_last_x) / dt;
        // float vy = (odom_y - odom_last_y) / dt;
        // float vyaw = (odom_yaw - odom_last_yaw) / dt;

        //----------------------------odometry from wheel-encoder-feedback-------------------------------------------------//

        // (1) แปลง counts → ระยะทาง (meters)
        float left_dist  = (float)odom_encoder_left  / (float)encoder_cpr * wheel_circumference;
        float right_dist = (float)odom_encoder_right / (float)encoder_cpr * wheel_circumference;

        float left_rpm  = (left_dist  / wheel_circumference) / dt * 60.0f * 20.0f;
        float right_rpm = (right_dist / wheel_circumference) / dt * 60.0f * 20.0f;
        // (2) คำนวณระยะ displacement และมุมหมุน
        float linear  = (left_dist + right_dist) / 2.0f;        // [m]
        float angular = (right_dist - left_dist) / track_width; // [rad]

        // (3) อัปเดต pose (linear เป็นระยะทาง จึงไม่ต้องคูณ dt อีก)
        odom_x   += linear * cos(odom_yaw);
        odom_y   += linear * sin(odom_yaw);
        odom_yaw  = NORMALIZE(odom_yaw + angular);

        // (4) ความเร็วเชิงเส้นและเชิงมุม
        float vx   = linear  / dt;
        // float vy   = 0.0f;
        float vyaw = angular / dt;
        
        // เก็บตำแหน่งเชิงมุมของล้อ
        float left_theta_abs  = odom_encoder_left_abs  / float(encoder_cpr) * 2.0f * M_PI;
        float right_theta_abs = odom_encoder_right_abs / float(encoder_cpr) * 2.0f * M_PI;
        

        // RCLCPP_INFO(this->get_logger(),
        //     "Wheel angles (rad)  Left: %.3f , Right: %.3f",
        //     left_theta, right_theta);
        //-----------------------------------------------------------------------------//

        odom_last_x = odom_x;
        odom_last_y = odom_y;
        odom_last_yaw = odom_yaw;
        // convert yaw to quat;
        tf2::Quaternion tf2_quat;
        tf2_quat.setRPY(0, 0, odom_yaw);
        // Convert tf2::Quaternion to geometry_msgs::msg::Quaternion
        geometry_msgs::msg::Quaternion quat = tf2::toMsg(tf2_quat);

        // tf2::Quaternion quat = tf2::createQuaternionMsgFromYaw(odom_yaw);
        //  TODO: set up tf2_ros

        quat = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), odom_yaw));
        if (pub_odom_tf)
        {
            tf_msg.header.stamp = this->get_clock()->now();
            tf_msg.header.frame_id = odom_frame;
            tf_msg.child_frame_id = base_frame;

            tf_msg.transform.translation.x = odom_x;
            tf_msg.transform.translation.y = odom_y;
            tf_msg.transform.translation.z = 0.0;
            tf_msg.transform.rotation = quat;
            odom_baselink_transform_->sendTransform(tf_msg);
        }

            // คำนวณระยะห่างครึ่งหนึ่งระหว่างล้อ
            float half_track = track_width / 2.0f;
            // เก็บเวลาปัจจุบันให้ตรงกันทั้งสองล้อ
            rclcpp::Time now = this->get_clock()->now();

            // --- Left wheel frame ---
            {
            geometry_msgs::msg::TransformStamped left_tf;
            left_tf.header.stamp    = now;
            left_tf.header.frame_id = base_frame;
            left_tf.child_frame_id  = "left_wheel_frame";
            left_tf.transform.translation.x = 0.0;
            left_tf.transform.translation.y =  half_track;
            left_tf.transform.translation.z = 0.0;

            // (1) quaternion offset หมุนรอบ X -90°:
            tf2::Quaternion q_offset;
            q_offset.setRPY(-M_PI/2.0, 0.0, 0.0);

            // (2) quaternion สำหรับ yaw = left_theta_abs:
            tf2::Quaternion q_yaw;
            q_yaw.setRPY(0.0, 0.0, left_theta_abs);

            // (3) คูณ offset ก่อน แล้วค่อย yaw:
            tf2::Quaternion ql = q_offset * q_yaw;

            left_tf.transform.rotation = tf2::toMsg(ql);
            odom_baselink_transform_->sendTransform(left_tf);
            }

            // --- Right wheel frame ---
            {
            geometry_msgs::msg::TransformStamped right_tf;
            right_tf.header.stamp    = now;
            right_tf.header.frame_id = base_frame;
            right_tf.child_frame_id  = "right_wheel_frame";
            right_tf.transform.translation.x = 0.0;
            right_tf.transform.translation.y = -half_track;
            right_tf.transform.translation.z = 0.0;

            tf2::Quaternion q_offset;
            q_offset.setRPY(-M_PI/2.0, 0.0, 0.0);

            tf2::Quaternion q_yaw;
            q_yaw.setRPY(0.0, 0.0, right_theta_abs);

            tf2::Quaternion qr = q_offset * q_yaw;
            right_tf.transform.rotation = tf2::toMsg(qr);
            odom_baselink_transform_->sendTransform(right_tf);
            }

        

        // update odom msg

        // odom_msg->header.seq++; //? not used in ros2 ?
        odom_msg.header.stamp = this->get_clock()->now();

        // delete weird syntax *dt
        odom_msg.pose.pose.position.x = odom_x;
        odom_msg.pose.pose.position.y = odom_y;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = quat;
        odom_msg.twist.twist.linear.x = vx;
        
        //correct syntax for ros2
        // odom_msg.twist.twist..y = 0.0;
        // odom_msg.twist.twist..z = 0.0;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;

        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = vyaw;
        odom_pub->publish(odom_msg);
        // odom_pub.publish(odom_msg); ROS1
        // สร้าง msg แล้วใส่ข้อมูล encoder state
        amr_interfaces::msg::WheelEncoderState state_msg;
        state_msg.header.stamp = this->get_clock()->now();
        state_msg.left_count  = odom_encoder_left_abs;
        state_msg.right_count = odom_encoder_right_abs;
        state_msg.left_angle  = left_theta_abs;
        state_msg.right_angle = right_theta_abs;
        state_msg.left_rpm    = left_rpm;
        state_msg.right_rpm   = right_rpm;

        // ส่งออก topic
        wheel_state_pub_->publish(state_msg);

    }

    int Roboteq::run()
    {

        // TODO: support automatic re-opening of port after disconnection

        starttime = millis();
        hstimer = starttime;
        mstimer = starttime;
        lstimer = starttime;

        cmdvel_loop();
        odom_loop();
        cmdvel_run();

        return 0;
    }

    Roboteq::~Roboteq()
    {

        if (controller.isOpen())
        {
            controller.close();
        }
        // rclcpp::shutdown(); // uncomment if node doesnt destroy properly
    }

    void Roboteq::publish_error_flags()
    {
    auto msg = amr_interfaces::msg::ErrorFlags();
    msg.ff = error_ff_;
    msg.fs = error_fs_;
    msg.fm1 = error_fm1_;
    msg.fm2 = error_fm2_;
    error_pub_->publish(msg);
    }
    

} // end of namespace

int main(int argc, char *argv[])
{

    rclcpp::init(argc, argv);

    // rclcpp::executors::SingleThreadedExecutor exec;
    // rclcpp::NodeOptions options;

    rclcpp::executors::MultiThreadedExecutor exec(
        rclcpp::ExecutorOptions(),  // default options
        4 /* จำนวน threads ที่ต้องการ ให้เหมาะกับ CPU/core */
    );

    auto node = std::make_shared<Roboteq::Roboteq>();
    exec.add_node(node);
    exec.spin();
    printf("stop");
    rclcpp::shutdown();
    // signal(SIGINT, mySigintHandler); // rclcpp::shutdown();
    return 0;
}
