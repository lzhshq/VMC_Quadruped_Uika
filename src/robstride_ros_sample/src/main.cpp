#include "motor_ros2/motor_cfg.h"
#include "interfaces/msg/motor_command12.hpp"
#include "interfaces/msg/motor_feedback12.hpp"
#include "stdint.h"
#include <atomic>
#include <iostream>
#include <memory>
#include <mutex>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <unistd.h>
#include <vector>

// 电机配置: CAN通道, 主站ID, 电机ID, 电机类型
struct MotorConfig {
  std::string can_iface;
  uint8_t master_id;
  uint8_t motor_id;
  int actuator_type;
};

// 电机标定参数
struct MotorCalibration {
    double reduction_ratio;   // 减速比
    double zero_offset;       // 零位偏移 (rad)
    double direction;        // 方向 (+1 或 -1)
};

// 12个电机独立标定参数 (需要实际标定)
// 电机索引按腿顺序: fl, rl, rr, fr
// fl=(0,1,2), rl=(3,4,5), rr=(6,7,8), fr=(9,10,11)
static const std::array<MotorCalibration, 12> MOTOR_CALIB = {{
    // FL腿 (can0)
    {1.0, 0.0, -1.0},   // 0: fl_hip
    {1.0, 0.0, -1.0},   // 1: fl_thigh
    {1.87, 0.0, 1.0},  // 2: fl_calf
    // RL腿 (can1)
    {1.0, 0.0, 1.0},   // 3: rl_hip
    {1.0, 0.0, -1.0},   // 4: rl_thigh
    {1.87, 0.0, 1.0},  // 5: rl_calf
    // RR腿 (can2)
    {1.0, 0.0, 1.0},   // 6: rr_hip
    {1.0, 0.0, 1.0},   // 7: rr_thigh
    {1.87, 0.0, 1.0},  // 8: rr_calf
    // FR腿 (can3)
    {1.0, 0.0, 1.0},   // 9: fr_hip
    {1.0, 0.0, 1.0},   // 10: fr_thigh
    {1.87, 0.0, 1.0},  // 11: fr_calf
}};

// 12个电机配置: CAN0-3各3个电机, 全部使用ROBSTRIDE_02类型
// 腿顺序: fl(can0), rl(can1), rr(can2), fr(can3)
static const std::vector<MotorConfig> MOTOR_CONFIGS = {
    // CAN0: FL腿
    {"can0", 0xFF, 1,  2},   // fl_hip
    {"can0", 0xFF, 2,  2},   // fl_thigh
    {"can0", 0xFF, 3,  2},   // fl_calf
    // CAN1: RL腿
    {"can1", 0xFF, 4,  2},   // rl_hip
    {"can1", 0xFF, 5,  2},   // rl_thigh
    {"can1", 0xFF, 6,  2},   // rl_calf
    // CAN2: RR腿
    {"can2", 0xFF, 7,  2},   // rr_hip
    {"can2", 0xFF, 8,  2},   // rr_thigh
    {"can2", 0xFF, 9,  2},   // rr_calf
    // CAN3: FR腿
    {"can3", 0xFF, 10, 2},   // fr_hip
    {"can3", 0xFF, 11, 2},   // fr_thigh
    {"can3", 0xFF, 12, 2},   // fr_calf
};

class MotorControlSample : public rclcpp::Node {
public:
  MotorControlSample()
      : rclcpp::Node("motor_control_set_node"),
        received_first_command_(false) {

    // 初始化12个电机
    motors_.reserve(MOTOR_CONFIGS.size());
    for (const auto &cfg : MOTOR_CONFIGS) {
      motors_.push_back(std::make_unique<RobStrideMotor>(
          cfg.can_iface, cfg.master_id, cfg.motor_id, cfg.actuator_type));
    }

    // 依次初始化每个电机，跳过失败的
    for (size_t i = 0; i < motors_.size(); ++i) {
      try {
        motors_[i]->Get_RobStrite_Motor_parameter(0x7005);
        usleep(1000);
        motors_[i]->enable_motor();
        usleep(1000);
        RCLCPP_INFO(this->get_logger(), "Motor %zu initialized on %s", i,
                    MOTOR_CONFIGS[i].can_iface.c_str());
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "Motor %zu init failed: %s", i,
                    e.what());
      }
    }

    // 创建电机反馈发布者
    feedback_pub_ = this->create_publisher<interfaces::msg::MotorFeedback12>("/motor_feedback", 10);

    // 创建定时器，每20ms发布一次反馈 (50Hz)
    feedback_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        [this]() {
          this->publish_feedback();
        });

    RCLCPP_INFO(this->get_logger(), "Motor feedback publisher started on /motor_feedback");

    // 创建电机命令订阅
    command_sub_ = this->create_subscription<interfaces::msg::MotorCommand12>(
        "/motor_command", 10,
        [this](const interfaces::msg::MotorCommand12::SharedPtr msg) {
          this->command_callback(msg);
        });

    RCLCPP_INFO(this->get_logger(), "Subscribed to /motor_command, waiting for commands...");

    worker_thread_ = std::thread(&MotorControlSample::excute_loop, this);
  }

  ~MotorControlSample() {
    // 禁用所有电机
    for (size_t i = 0; i < motors_.size(); ++i) {
      motors_[i]->Disenable_Motor(0);
    }
    running_ = false;
    if (worker_thread_.joinable())
      worker_thread_.join();
  }

private:
  void command_callback(const interfaces::msg::MotorCommand12::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    latest_command_ = *msg;
    received_first_command_ = true;
    RCLCPP_INFO_ONCE(this->get_logger(), "First command received! Starting motor control.");
  }

  void publish_feedback() {
    auto msg = interfaces::msg::MotorFeedback12();
    msg.header.stamp = this->now();
    msg.header.frame_id = "motor_feedback";

    // 填充12个电机的反馈数据
    // 电机索引: fl=(0,1,2), rl=(3,4,5), rr=(6,7,8), fr=(9,10,11)
    msg.fl_hip.torque =- motors_[0]->torque_;
    msg.fl_hip.position =- motors_[0]->position_;
    msg.fl_hip.velocity =- motors_[0]->velocity_;
    msg.fl_hip.temperature = motors_[0]->temperature_;

    msg.fl_thigh.torque =- motors_[1]->torque_;
    msg.fl_thigh.position =- motors_[1]->position_;
    msg.fl_thigh.velocity =- motors_[1]->velocity_;
    msg.fl_thigh.temperature = motors_[1]->temperature_;

    msg.fl_calf.torque = motors_[2]->torque_;
    msg.fl_calf.position = motors_[2]->position_;
    msg.fl_calf.velocity = motors_[2]->velocity_;
    msg.fl_calf.temperature = motors_[2]->temperature_;

    msg.rl_hip.torque =- motors_[3]->torque_;
    msg.rl_hip.position =- motors_[3]->position_;
    msg.rl_hip.velocity =- motors_[3]->velocity_;
    msg.rl_hip.temperature = motors_[3]->temperature_;

    msg.rl_thigh.torque =- motors_[4]->torque_;
    msg.rl_thigh.position =- motors_[4]->position_;
    msg.rl_thigh.velocity =- motors_[4]->velocity_;
    msg.rl_thigh.temperature = motors_[4]->temperature_;

    msg.rl_calf.torque = motors_[5]->torque_;
    msg.rl_calf.position = motors_[5]->position_;
    msg.rl_calf.velocity = motors_[5]->velocity_;
    msg.rl_calf.temperature = motors_[5]->temperature_;

    msg.rr_hip.torque =- motors_[6]->torque_;
    msg.rr_hip.position =- motors_[6]->position_;
    msg.rr_hip.velocity =- motors_[6]->velocity_;
    msg.rr_hip.temperature = motors_[6]->temperature_;

    msg.rr_thigh.torque =- motors_[7]->torque_;
    msg.rr_thigh.position =- motors_[7]->position_;
    msg.rr_thigh.velocity =- motors_[7]->velocity_;
    msg.rr_thigh.temperature = motors_[7]->temperature_;

    msg.rr_calf.torque = motors_[8]->torque_;
    msg.rr_calf.position = motors_[8]->position_;
    msg.rr_calf.velocity = motors_[8]->velocity_;
    msg.rr_calf.temperature = motors_[8]->temperature_;

    msg.fr_hip.torque =- motors_[9]->torque_;
    msg.fr_hip.position =- motors_[9]->position_;
    msg.fr_hip.velocity =- motors_[9]->velocity_;
    msg.fr_hip.temperature = motors_[9]->temperature_;

    msg.fr_thigh.torque =- motors_[10]->torque_;
    msg.fr_thigh.position =- motors_[10]->position_;
    msg.fr_thigh.velocity =- motors_[10]->velocity_;
    msg.fr_thigh.temperature = motors_[10]->temperature_;

    msg.fr_calf.torque = motors_[11]->torque_;
    msg.fr_calf.position = motors_[11]->position_;
    msg.fr_calf.velocity = motors_[11]->velocity_;
    msg.fr_calf.temperature = motors_[11]->temperature_;

    feedback_pub_->publish(msg);
  }

  void excute_loop() {
    // 等待接收第一个命令
    int wait_count = 0;
    while (rclcpp::ok() && running_ && !received_first_command_) {
      wait_count++;
      if (wait_count % 1000 == 0) {
        RCLCPP_INFO(this->get_logger(), "Waiting for first command...");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (!rclcpp::ok() || !running_) {
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Start sending motion commands");

    while (rclcpp::ok() && running_) {
      // 获取最新命令
      std::lock_guard<std::mutex> lock(command_mutex_);
      const auto& cmd = latest_command_;

      // 向所有电机发送运控命令
      // 电机索引: fl=(0,1,2), rl=(3,4,5), rr=(6,7,8), fr=(9,10,11)
      motors_[0]->send_motion_command(
          cmd.fl_hip.torque, -cmd.fl_hip.position, -cmd.fl_hip.velocity,
          cmd.fl_hip.kp, cmd.fl_hip.kd);
      motors_[1]->send_motion_command(
          cmd.fl_thigh.torque, -cmd.fl_thigh.position, -cmd.fl_thigh.velocity,
          cmd.fl_thigh.kp, cmd.fl_thigh.kd);
      motors_[2]->send_motion_command(
          cmd.fl_calf.torque, cmd.fl_calf.position*1.87, cmd.fl_calf.velocity,
          cmd.fl_calf.kp, cmd.fl_calf.kd);
      motors_[3]->send_motion_command(
          cmd.rl_hip.torque, -cmd.rl_hip.position, -cmd.rl_hip.velocity,
          cmd.rl_hip.kp, cmd.rl_hip.kd);
      motors_[4]->send_motion_command(
          cmd.rl_thigh.torque, -cmd.rl_thigh.position, -cmd.rl_thigh.velocity,
          cmd.rl_thigh.kp, cmd.rl_thigh.kd);
      motors_[5]->send_motion_command(
          cmd.rl_calf.torque, cmd.rl_calf.position*1.87, cmd.rl_calf.velocity,
          cmd.rl_calf.kp, cmd.rl_calf.kd);
      motors_[6]->send_motion_command(
          cmd.rr_hip.torque, -cmd.rr_hip.position, -cmd.rr_hip.velocity,
          cmd.rr_hip.kp, cmd.rr_hip.kd);
      motors_[7]->send_motion_command(
          cmd.rr_thigh.torque, -cmd.rr_thigh.position, -cmd.rr_thigh.velocity,
          cmd.rr_thigh.kp, cmd.rr_thigh.kd);
      motors_[8]->send_motion_command(
          cmd.rr_calf.torque, cmd.rr_calf.position*1.87, cmd.rr_calf.velocity,
          cmd.rr_calf.kp, cmd.rr_calf.kd);
      motors_[9]->send_motion_command(
          cmd.fr_hip.torque, -cmd.fr_hip.position, -cmd.fr_hip.velocity,
          cmd.fr_hip.kp, cmd.fr_hip.kd);
      motors_[10]->send_motion_command(
          cmd.fr_thigh.torque, -cmd.fr_thigh.position, -cmd.fr_thigh.velocity,
          cmd.fr_thigh.kp, cmd.fr_thigh.kd);
      motors_[11]->send_motion_command(
          cmd.fr_calf.torque, cmd.fr_calf.position*1.87, cmd.fr_calf.velocity,
          cmd.fr_calf.kp, cmd.fr_calf.kd);

      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  rclcpp::Publisher<interfaces::msg::MotorFeedback12>::SharedPtr feedback_pub_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
  rclcpp::Subscription<interfaces::msg::MotorCommand12>::SharedPtr command_sub_;
  std::thread worker_thread_;
  std::atomic<bool> running_ = true;
  std::atomic<bool> received_first_command_;
  std::mutex command_mutex_;
  interfaces::msg::MotorCommand12 latest_command_;
  std::vector<std::unique_ptr<RobStrideMotor>> motors_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto controller = std::make_shared<MotorControlSample>();

  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(controller);

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
