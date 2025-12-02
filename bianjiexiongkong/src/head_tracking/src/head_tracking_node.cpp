#include "rclcpp/rclcpp.hpp"
#include "head_tracking/kalman_filter.hpp"
#include "chest_interfaces/msg/head_tracking.hpp"
#include "chest_interfaces/msg/button_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <cmath>
#include <random>
#include <array>

using namespace std::chrono_literals;

class HeadTrackingNode : public rclcpp::Node
{
public:
    HeadTrackingNode() : Node("head_tracking_node"), is_tracking_(false)
    {
        // 参数配置
        this->declare_parameter("simulate_imu", true);
        bool simulate_imu = this->get_parameter("simulate_imu").as_bool();

        RCLCPP_INFO(this->get_logger(), "头动解算模块启动中...");

        // 初始化卡尔曼滤波器
        pitch_filter_ = std::make_unique<KalmanFilter>(0.01, 0.1);
        yaw_filter_ = std::make_unique<KalmanFilter>(0.01, 0.1);
        RCLCPP_INFO(this->get_logger(), "卡尔曼滤波器初始化完成");

        // 订阅IMU数据
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/tk_chest/sensors/imu", 10,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg)
            {
                if (is_tracking_)
                {
                    process_imu(msg);
                }
            });

        // 订阅按键状态 - 使用新的ButtonState消息
        button_sub_ = create_subscription<chest_interfaces::msg::ButtonState>(
            "/tk_chest/button/state", 10,
            [this](const chest_interfaces::msg::ButtonState::SharedPtr msg)
            {
                bool old_tracking = is_tracking_;
                is_tracking_ = msg->head_tracking_enabled;
                if (old_tracking != is_tracking_)
                {
                    RCLCPP_INFO(this->get_logger(), "头动跟踪状态: %s", 
                               is_tracking_ ? "启用" : "禁用");
                    
                    // 控制IMU仿真定时器
                    if (sim_imu_timer_)
                    {
                        if (is_tracking_)
                        {
                            sim_imu_timer_->reset();
                            RCLCPP_INFO(this->get_logger(), "IMU仿真已启动");
                        }
                        else
                        {
                            sim_imu_timer_->cancel();
                            RCLCPP_INFO(this->get_logger(), "IMU仿真已停止");
                        }
                    }
                }
            });

        // 发布头动信息 - 使用新的HeadTracking消息
        tracking_pub_ = create_publisher<chest_interfaces::msg::HeadTracking>(
            "/tk_chest/head_tracking/state", 10);

        // 模拟IMU数据（如果没有真实IMU）
        if (simulate_imu)
        {
            RCLCPP_INFO(this->get_logger(), "启用IMU仿真模式（等待头动跟踪开关启动）");
            imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("tk_chest/sensors/imu", 10);
            // 创建定时器但不立即启动，等待头动跟踪开关控制
            sim_imu_timer_ = create_wall_timer(10ms, [this]()
                                               { simulate_imu_data(); });
            sim_imu_timer_->cancel(); // 默认取消，等待开关启动
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "使用真实IMU数据");
        }

        RCLCPP_INFO(this->get_logger(), "头动解算模块启动完成，等待数据...");
    }

private:
    void process_imu(const sensor_msgs::msg::Imu::SharedPtr imu)
    {
        // 使用加速度计计算俯仰角
        float ax = imu->linear_acceleration.x;
        float ay = imu->linear_acceleration.y;
        float az = imu->linear_acceleration.z;
        float pitch_rad = atan2(-ax, sqrt(ay * ay + az * az));

        // 使用陀螺仪计算偏航角 (积分)
        static float yaw_rad = 0.0;
        static rclcpp::Time last_time = this->now();
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time).seconds();
        last_time = current_time;

        // 积分计算偏航角
        yaw_rad += imu->angular_velocity.z * dt;

        // 归一化到[-π, π]
        yaw_rad = fmod(yaw_rad + M_PI, 2 * M_PI) - M_PI;

        // 应用卡尔曼滤波
        pitch_deg_ = pitch_filter_->update(pitch_rad * 180.0 / M_PI);
        yaw_deg_ = yaw_filter_->update(yaw_rad * 180.0 / M_PI);

        // 保存弧度值用于发布
        pitch_rad_ = pitch_deg_ * M_PI / 180.0;
        yaw_rad_ = yaw_deg_ * M_PI / 180.0;

        // 零速修正
        zero_velocity_update_ = false;
        const float gravity_threshold = 0.5;
        const float acceleration_threshold = 0.3;
        if (fabs(sqrt(ax * ax + ay * ay + az * az) - 9.8) < gravity_threshold &&
            fabs(ax) < acceleration_threshold &&
            fabs(ay) < acceleration_threshold)
        {
            pitch_filter_->init(0);
            yaw_filter_->init(0);
            zero_velocity_update_ = true;
            RCLCPP_DEBUG(this->get_logger(), "应用零速修正");
        }

        // 计算置信度 (简单示例)
        confidence_ = 1.0f;
        if (!zero_velocity_update_)
        {
            // 运动时的置信度基于加速度变化
            float motion = sqrt(ax * ax + ay * ay);
            confidence_ = std::min(1.0f, 0.8f + motion * 0.2f);
        }

        // 发布头动信息 - 确保使用正确的消息格式
        auto tracking_msg = chest_interfaces::msg::HeadTracking();
        tracking_msg.header.stamp = this->now();
        tracking_msg.header.frame_id = "head";
        tracking_msg.pitch = pitch_rad_; // 俯仰角 (弧度)
        tracking_msg.yaw = yaw_rad_;     // 偏航角 (弧度)
        tracking_msg.is_tracking = is_tracking_; // 使用新消息字段名称
        tracking_msg.confidence = confidence_;

        tracking_pub_->publish(tracking_msg);

    }

    void simulate_imu_data()
    {
        static float t = 0.0;
        t += 0.05;

        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "imu_link";

        // 增加模拟幅度：俯仰角在-45°~45°，偏航角在-90°~90°之间波动
        float pitch_angle = 0.785 * sin(t);    // ±45° (0.785 rad)
        float yaw_angle = 1.57 * sin(t * 0.5); // ±90° (1.57 rad)

        // 计算重力分量 (模拟头部运动)
        imu_msg.linear_acceleration.x = -9.8 * sin(pitch_angle);                 // 俯仰方向加速度
        imu_msg.linear_acceleration.y = 9.8 * sin(yaw_angle);                    // 偏航方向加速度
        imu_msg.linear_acceleration.z = 9.8 * cos(pitch_angle) * cos(yaw_angle); // 垂直方向

        // 添加角速度数据 (用于偏航角计算)
        imu_msg.angular_velocity.x = 0.1 * cos(t);       // 滚转角速度
        imu_msg.angular_velocity.y = 0.2 * cos(t * 0.8); // 俯仰角速度
        imu_msg.angular_velocity.z = 0.3 * sin(t * 0.6); // 偏航角速度

        // 添加噪声
        std::normal_distribution<float> dist(0, 0.05);
        imu_msg.linear_acceleration.x += dist(gen_);
        imu_msg.linear_acceleration.y += dist(gen_);
        imu_msg.linear_acceleration.z += dist(gen_);
        imu_msg.angular_velocity.x += dist(gen_) * 0.1;
        imu_msg.angular_velocity.y += dist(gen_) * 0.1;
        imu_msg.angular_velocity.z += dist(gen_) * 0.1;

        // 发布模拟IMU数据
        imu_pub_->publish(imu_msg);
    }

    // ROS 订阅和发布
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<chest_interfaces::msg::ButtonState>::SharedPtr button_sub_;
    rclcpp::Publisher<chest_interfaces::msg::HeadTracking>::SharedPtr tracking_pub_;

    // 模拟IMU相关
    rclcpp::TimerBase::SharedPtr sim_imu_timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    // 滤波器
    std::unique_ptr<KalmanFilter> pitch_filter_;
    std::unique_ptr<KalmanFilter> yaw_filter_;

    // 状态变量
    bool is_tracking_; // 修改为与新消息字段一致
    bool zero_velocity_update_ = false;
    float pitch_deg_ = 0.0;  // 俯仰角 (度)
    float yaw_deg_ = 0.0;    // 偏航角 (度)
    float pitch_rad_ = 0.0;  // 俯仰角 (弧度)
    float yaw_rad_ = 0.0;    // 偏航角 (弧度)
    float confidence_ = 1.0; // 置信度

    // 随机数生成
    std::random_device rd_;
    std::mt19937 gen_{rd_()};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HeadTrackingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}