/*
 * ============================================================
 * UIKA 机器人 RL 控制 ROS2 节点
 * ============================================================
 *
 * 【功能说明】
 *   - 订阅外部传感器数据（关节状态、IMU）
 *   - 将传感器数据组装成45维观测向量（包含6帧历史）
 *   - 运行ONNX策略模型推理
 *   - 发布12维关节目标位置
 *
 * 【输入话题】
 *   /uika/joint_state (std_msgs/Float64MultiArray)
 *     - 24个float: [q0,q1,...,q11, dq0,dq1,...,dq11]
 *     - 前12个: 关节位置 (rad)
 *     - 后12个: 关节速度 (rad/s)
 *
 *   /uika/imu (std_msgs/Float64MultiArray)
 *     - 7个float: [qw, qx, qy, qz, gx, gy, gz]
 *     - 前4个: 四元数 (w,x,y,z) 姿态
 *     - 后3个: 陀螺仪角速度 (rad/s)
 *
 * 【输出话题】
 *   /uika/target_position (std_msgs/Float64MultiArray)
 *     - 12个float: 关节目标位置 (rad)
 *     - 直接发送给下位机电机控制器
 *
 * 【观测向量构成】(45维)
 *   commands(3)   + ang_vel(3)  + gravity_vec(3)  + dof_pos(12) + dof_vel(12) + actions(12) = 45
 *   命令         + 角速度       + 重力向量         + 关节位置    + 关节速度    + 上一步动作
 *
 * 【6帧历史缓冲】
 *   observations_history: [0,1,2,3,4,5] 表示使用最近6帧
 *   实际输入 = 45 x 6 = 270维
 *
 * 【使用方法】
 *   1. 启动RL节点: ros2 run rl_sar uika
 *   2. 确保外部节点发布 /uika/joint_state 和 /uika/imu
 *   3. 本节点自动订阅并推理，发布 /uika/target_position
 *
 * ============================================================
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "inference_runtime.hpp"
#include "loop.hpp"
#include "fsm_uika.hpp"
#include "vector_math.hpp"

#include <yaml-cpp/yaml.h>
#include <memory>
#include <mutex>
#include <vector>
#include <algorithm>
#include <limits>

/**
 * @brief UIKA RL 控制节点
 *
 * 核心工作流程:
 *  1. 初始化: 加载配置、ONNX模型、创建ROS2接口
 *  2. 数据接收: 回调函数接收 joint_state 和 imu 数据
 *  3. 推理循环: 定时器触发，组装观测 -> 推理 -> 发布
 */
class UINode : public rclcpp::Node
{
public:
    UINode()
        : Node("uika_rl_node"),
          // 默认值初始化
          num_of_dofs_(12),          // 12个关节
          dt_(0.005),                // 控制周期 5ms
          decimation_(4),            // 推理间隔: 5ms * 4 = 20ms (50Hz)
          last_action_(12, 0.0f),   // 上一步动作初始为零
          last_filtered_action_(12, 0.0f),  // 上一步滤波后动作
          robot_name_("uika"),       // 机器人名称
          rl_init_done_(false),      // RL未初始化
          model_(nullptr),            // ONNX模型指针
          // 初始化数据接收标志为 false
          joint_state_received_(false),
          imu_received_(false)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing UIKA RL Node...");

        // ============================================================
        // 1. 加载参数
        // ============================================================
        this->declare_parameter("robot_name", robot_name_);
        this->get_parameter("robot_name", robot_name_);

        // ============================================================
        // 2. 创建发布器: /uika/target_position
        //    - 12个关节目标位置 (rad)
        //    - QoS队列深度10
        // ============================================================
        target_position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/uika/target_position", 10);

        // ============================================================
        // 3. 创建订阅器
        // ============================================================

        // 3.1 关节状态订阅: /uika/joint_state
        //    数据格式: [q0,q1,...,q11, dq0,dq1,...,dq11] 共24个float
        //    - q: 关节位置 (rad)
        //    - dq: 关节速度 (rad/s)
        joint_state_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/uika/joint_state", 10,
            std::bind(&UINode::JointStateCallback, this, std::placeholders::_1));

        // 3.2 IMU订阅: /uika/imu
        //    数据格式: [qw, qx, qy, qz, gx, gy, gz] 共7个float
        //    - quat: 四元数姿态 (w,x,y,z)
        //    - gyro: 陀螺仪角速度 (rad/s)
        imu_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/uika/imu", 10,
            std::bind(&UINode::ImuCallback, this, std::placeholders::_1));

        // ============================================================
        // 4. 初始化RL模块
        //    - 加载YAML配置
        //    - 加载ONNX策略模型
        //    - 初始化观测缓冲区和变量
        // ============================================================
        InitRL();

        // ============================================================
        // 5. 创建推理定时器
        //    - 频率 = 1 / (dt * decimation) = 1 / (0.005 * 4) = 50Hz
        //    - 每次触发执行一次: 观测组装 -> 模型推理 -> 发布结果
        // ============================================================
        inference_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(dt_ * decimation_ * 1000)),
            std::bind(&UINode::InferenceLoop, this));

        // ============================================================
        // 6. 打印启动信息
        // ============================================================
        RCLCPP_INFO(this->get_logger(), "UIKA RL Node initialized successfully");
        RCLCPP_INFO(this->get_logger(), "  [Subscribers]");
        RCLCPP_INFO(this->get_logger(), "    /uika/joint_state (24D: 12 positions + 12 velocities)");
        RCLCPP_INFO(this->get_logger(), "    /uika/imu (7D: 4 quaternion + 3 gyroscope)");
        RCLCPP_INFO(this->get_logger(), "  [Publishers]");
        RCLCPP_INFO(this->get_logger(), "    /uika/target_position (12D: joint target positions)");
        RCLCPP_INFO(this->get_logger(), "  [Inference]");
        RCLCPP_INFO(this->get_logger(), "    Rate: %.1f Hz (dt=%.4f, decimation=%d)",
                    1.0 / (dt_ * decimation_), dt_, decimation_);
        RCLCPP_INFO(this->get_logger(), "    Observation: %d dims + %d frame history = %d total",
                    obs_dims_.empty() ? 0 : 45, observations_history_.size(),
                    obs_dims_.empty() ? 0 : 45 * (observations_history_.size() + 1));
        RCLCPP_INFO(this->get_logger(), "  [Low-pass Filter]");
        RCLCPP_INFO(this->get_logger(), "    Alpha: %.2f (%.0f%% new, %.0f%% old)",
                    action_filter_alpha_, action_filter_alpha_ * 100, (1 - action_filter_alpha_) * 100);
    }

private:
    // ================================================================
    // InitRL(): 初始化RL模块
    // ================================================================
    void InitRL()
    {
        // ------------------------------------------------------------
        // 1. 确定配置文件和模型路径
        //    优先使用: policy/<robot_name>/himloco/config.yaml
        //    备选使用: policy/<robot_name>/config.yaml
        // ------------------------------------------------------------
        std::string policy_dir = std::string(POLICY_DIR);
        std::string config_path = policy_dir + "/" + robot_name_ + "/himloco/config.yaml";
        std::string model_subdir = "himloco/";
        std::string model_name_from_config = "";

        // 如果himloco路径不存在，尝试根目录
        if (!std::ifstream(config_path))
        {
            config_path = policy_dir + "/" + robot_name_ + "/config.yaml";
            model_subdir = "";
        }

        RCLCPP_INFO(this->get_logger(), "Loading config from: %s", config_path.c_str());

        // ------------------------------------------------------------
        // 2. 加载YAML配置
        // ------------------------------------------------------------
        try
        {
            YAML::Node config = YAML::LoadFile(config_path);

            // 配置可能嵌套在 robot_name/himloco 下
            std::string config_key = robot_name_;
            if (config[robot_name_ + "/himloco"])
                config_key = robot_name_ + "/himloco";

            YAML::Node robot_config = config[config_key];

            // 尝试从base.yaml加载机器人物理参数（如果himloco config没有）
            std::string base_config_path = policy_dir + "/" + robot_name_ + "/base.yaml";
            YAML::Node base_config;
            if (std::ifstream(base_config_path))
            {
                base_config = YAML::LoadFile(base_config_path)[robot_name_];
            }

            // 宏: 从robot_config获取参数，失败则从base_config获取
            #define GET_PARAM(key, var) \
                if (robot_config[#key]) var = robot_config[#key].as<decltype(var)>(); \
                else if (base_config[#key]) var = base_config[#key].as<decltype(var)>(); \
                else RCLCPP_WARN(this->get_logger(), "Parameter '%s' not found, using default", #key);

            // 2.1 控制参数
            GET_PARAM(dt, dt_)                      // 控制周期
            GET_PARAM(decimation, decimation_)      // 推理降采样
            GET_PARAM(num_of_dofs, num_of_dofs_)    // 关节数

            // 2.2 模型文件名 - 从config读取
            if (robot_config["model_name"])
                model_name_from_config = robot_config["model_name"].as<std::string>();
            else
                RCLCPP_WARN(this->get_logger(), "Parameter 'model_name' not found in config, will use default");

            // 2.3 观测配置列表
            //     从config.yaml读取observations字段
            //     典型值: ["commands", "ang_vel", "gravity_vec", "dof_pos", "dof_vel", "actions"]
            if (robot_config["observations"])
            {
                for (const auto &obs : robot_config["observations"])
                    observations_.push_back(obs.as<std::string>());
            }

            // 2.4 观测缩放因子
            GET_PARAM(lin_vel_scale, lin_vel_scale_)    // 线速度缩放
            GET_PARAM(ang_vel_scale, ang_vel_scale_)    // 角速度缩放
            GET_PARAM(dof_pos_scale, dof_pos_scale_)    // 关节位置缩放
            GET_PARAM(dof_vel_scale, dof_vel_scale_)    // 关节速度缩放

            // 2.5 动作和命令缩放 - 带越界检查
            if (robot_config["action_scale"])
            {
                action_scale_.clear();
                for (const auto &s : robot_config["action_scale"])
                    action_scale_.push_back(s.as<float>());
            }
            if (robot_config["commands_scale"])
            {
                commands_scale_.clear();
                for (const auto &s : robot_config["commands_scale"])
                    commands_scale_.push_back(s.as<float>());
            }

            // 2.6 默认关节位置（站立姿态）
            if (robot_config["default_dof_pos"])
            {
                default_dof_pos_.clear();
                for (const auto &p : robot_config["default_dof_pos"])
                    default_dof_pos_.push_back(p.as<float>());
            }

            // 2.7 历史观测配置
            //     observations_history: [0,1,2,3,4,5] 表示使用最近6帧
            if (robot_config["observations_history"])
            {
                observations_history_.clear();
                for (const auto &h : robot_config["observations_history"])
                    observations_history_.push_back(h.as<int>());
            }

            // 2.8 裁剪限制
            GET_PARAM(clip_obs, clip_obs_)  // 观测裁剪范围
            if (robot_config["clip_actions_upper"])
            {
                clip_actions_upper_.clear();
                for (const auto &c : robot_config["clip_actions_upper"])
                    clip_actions_upper_.push_back(c.as<float>());
            }
            if (robot_config["clip_actions_lower"])
            {
                clip_actions_lower_.clear();
                for (const auto &c : robot_config["clip_actions_lower"])
                    clip_actions_lower_.push_back(c.as<float>());
            }

            RCLCPP_INFO(this->get_logger(), "Config loaded: dt=%.3f, decimation=%d, dofs=%d",
                        dt_, decimation_, num_of_dofs_);
        }
        catch (YAML::BadFile &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load config: %s", e.what());
            return;
        }

        // ------------------------------------------------------------
        // 3. 确定模型路径
        // ------------------------------------------------------------
        std::string model_path;
        if (!model_name_from_config.empty())
        {
            // 使用config中的模型名
            model_path = policy_dir + "/" + robot_name_ + "/" + model_subdir + model_name_from_config;
        }
        else
        {
            // 后备: 尝试扫描目录下最新的onnx文件
            RCLCPP_WARN(this->get_logger(), "No model_name in config, trying to find .onnx file");
            model_path = policy_dir + "/" + robot_name_ + "/" + model_subdir + "policy.onnx";
        }

        RCLCPP_INFO(this->get_logger(), "Loading model from: %s", model_path.c_str());

        // ------------------------------------------------------------
        // 4. 加载ONNX模型
        // ------------------------------------------------------------
        model_ = InferenceRuntime::ModelFactory::load_model(model_path);
        if (!model_)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load model!");
            return;
        }

        // ------------------------------------------------------------
        // 5. 检查缩放因子长度，不足时填充默认值
        // ------------------------------------------------------------
        EnsureScaleVectorSize();

        // ------------------------------------------------------------
        // 6. 初始化观测维度
        //    计算每个观测组件的维度
        //    典型: {3, 3, 3, 12, 12, 12} = 6个组件，共45维
        // ------------------------------------------------------------
        ComputeObsDims();

        // ------------------------------------------------------------
        // 7. 初始化历史观测缓冲区
        //    用于存储最近6帧观测，实现时序依赖
        // ------------------------------------------------------------
        int history_length = 6;  // 默认6帧
        if (!observations_history_.empty())
        {
            // 从配置计算历史长度: max(indices) + 1
            history_length = *std::max_element(observations_history_.begin(), observations_history_.end()) + 1;
        }
        // 创建环形缓冲区: 1环境 x obs_dims x history_length
        history_obs_buf_ = ObservationBuffer(1, obs_dims_, history_length, "time");

        // ------------------------------------------------------------
        // 8. 初始化观测变量
        //    这些变量在推理时会被外部数据更新
        // ------------------------------------------------------------
        obs_.lin_vel    = {0.0f, 0.0f, 0.0f};      // 线速度（未使用）
        obs_.ang_vel    = {0.0f, 0.0f, 0.0f};      // 角速度（来自IMU陀螺仪）
        obs_.gravity_vec = {0.0f, 0.0f, -1.0f};   // 重力向量（从四元数计算）
        obs_.commands   = {0.0f, 0.0f, 0.0f};      // 命令（设为零，自主模式）
        obs_.base_quat  = {0.0f, 0.0f, 0.0f, 1.0f};  // 基座姿态（四元数）
        obs_.dof_pos    = default_dof_pos_;         // 关节位置（来自/joint_state）
        obs_.dof_vel.resize(num_of_dofs_, 0.0f);   // 关节速度（来自/joint_state）
        obs_.actions.resize(num_of_dofs_, 0.0f);    // 动作（上一步输出）

        // 初始化滤波动作
        last_filtered_action_.resize(num_of_dofs_, 0.0f);

        rl_init_done_ = true;
        RCLCPP_INFO(this->get_logger(), "RL initialization complete.");
    }

    // ================================================================
    // EnsureScaleVectorSize(): 确保缩放因子向量长度足够
    // ================================================================
    void EnsureScaleVectorSize()
    {
        // action_scale 检查和填充
        if (action_scale_.size() < static_cast<size_t>(num_of_dofs_))
        {
            RCLCPP_WARN(this->get_logger(), "action_scale has %zu elements, expected %d. Padding with 1.0",
                        action_scale_.size(), num_of_dofs_);
            while (action_scale_.size() < static_cast<size_t>(num_of_dofs_))
                action_scale_.push_back(1.0f);
        }

        // commands_scale 检查和填充
        if (commands_scale_.size() < 3)
        {
            RCLCPP_WARN(this->get_logger(), "commands_scale has %zu elements, expected 3. Padding with 1.0",
                        commands_scale_.size());
            while (commands_scale_.size() < 3)
                commands_scale_.push_back(1.0f);
        }

        // default_dof_pos 检查
        if (default_dof_pos_.size() < static_cast<size_t>(num_of_dofs_))
        {
            RCLCPP_WARN(this->get_logger(), "default_dof_pos has %zu elements, expected %d. Using zeros.",
                        default_dof_pos_.size(), num_of_dofs_);
            default_dof_pos_.resize(num_of_dofs_, 0.0f);
        }
    }

    // ================================================================
    // ComputeObsDims(): 计算观测向量各组件的维度
    // ================================================================
    void ComputeObsDims()
    {
        obs_dims_.clear();

        for (const std::string &obs : observations_)
        {
            int dim = 0;
            if (obs == "lin_vel")      dim = 3;      // 线速度
            else if (obs == "ang_vel") dim = 3;      // 角速度
            else if (obs == "gravity_vec") dim = 3;  // 重力向量
            else if (obs == "commands") dim = 3;     // 命令
            else if (obs == "dof_pos") dim = num_of_dofs_;    // 关节位置
            else if (obs == "dof_vel") dim = num_of_dofs_;    // 关节速度
            else if (obs == "actions") dim = num_of_dofs_;    // 动作

            if (dim > 0)
                obs_dims_.push_back(dim);
        }
    }

    // ================================================================
    // NormalizeQuaternion(): 归一化四元数
    // ================================================================
    std::vector<float> NormalizeQuaternion(const std::vector<float>& q)
    {
        float norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        if (norm < 1e-6f)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Quaternion norm too small: %.6f, returning unit quaternion", norm);
            return {1.0f, 0.0f, 0.0f, 0.0f};  // 单位四元数
        }
        return {q[0]/norm, q[1]/norm, q[2]/norm, q[3]/norm};
    }

    // ================================================================
    // JointStateCallback(): 关节状态回调
    //
    // 接收: /uika/joint_state 消息
    // 数据: [q0,q1,...,q11, dq0,dq1,...,dq11] 共24个float
    //   - q[0:12]:  关节位置 (rad)
    //   - dq[12:24]: 关节速度 (rad/s)
    //
    // 处理: 存入缓冲区，设置接收标志
    // ================================================================
    void JointStateCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // 验证数据长度
        if (msg->data.size() != 24)
        {
            RCLCPP_WARN(this->get_logger(), "Invalid joint_state size: %zu, expected 24", msg->data.size());
            return;
        }

        std::lock_guard<std::mutex> lock(data_mutex_);

        // 解析并存储关节数据
        // 前12个: 位置 q
        // 后12个: 速度 dq
        for (int i = 0; i < 12; ++i)
        {
            joint_pos_[i] = msg->data[i];       // 关节位置
            joint_vel_[i] = msg->data[i + 12];  // 关节速度
        }

        joint_state_received_ = true;  // 标记数据已接收
    }

    // ================================================================
    // ImuCallback(): IMU数据回调
    //
    // 接收: /uika/imu 消息
    // 数据: [qw, qx, qy, qz, gx, gy, gz] 共7个float
    //   - quat[0:4]:  四元数 (w,x,y,z) 姿态
    //   - gyro[4:7]:  陀螺仪角速度 (rad/s)
    //
    // 处理: 存入缓冲区，设置接收标志
    // ================================================================
    void ImuCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // 验证数据长度
        if (msg->data.size() != 7)
        {
            RCLCPP_WARN(this->get_logger(), "Invalid imu size: %zu, expected 7", msg->data.size());
            return;
        }

        std::lock_guard<std::mutex> lock(data_mutex_);

        // 解析并存储IMU数据
        // 前4个: 四元数
        for (int i = 0; i < 4; ++i)
            imu_quat_[i] = msg->data[i];

        // 后3个: 陀螺仪
        for (int i = 0; i < 3; ++i)
            imu_gyro_[i] = msg->data[i + 4];

        imu_received_ = true;  // 标记数据已接收
    }

    // ================================================================
    // ComputeObservation(): 组装当前时刻的观测向量
    //
    // 观测组成 (45维):
    //   commands(3)   - 命令，设为零
    //   ang_vel(3)    - 角速度，来自IMU陀螺仪
    //   gravity_vec(3) - 重力向量，从四元数实时计算
    //   dof_pos(12)   - 关节位置，来自/joint_state前12个
    //   dof_vel(12)   - 关节速度，来自/joint_state后12个
    //   actions(12)   - 上一步动作
    //
    // 返回: 45维观测向量（已缩放和裁剪）
    // ================================================================
    std::vector<float> ComputeObservation()
    {
        std::vector<float> obs_vec;

        // --------------------------------------------------------
        // P0 Fix: 实时计算重力向量
        // 使用 QuatRotateInverse 将世界坐标重力旋转到机体系
        // --------------------------------------------------------
        std::vector<float> world_gravity = {0.0f, 0.0f, -1.0f};
        std::vector<float> body_gravity = QuatRotateInverse(obs_.base_quat, world_gravity);

        // 按配置顺序组装各观测组件
        for (const std::string &obs_name : observations_)
        {
            if (obs_name == "commands")
            {
                // 命令: 设为零（自主模式）
                // 乘以命令缩放因子
                std::vector<float> scaled = obs_.commands;
                for (size_t i = 0; i < scaled.size() && i < commands_scale_.size(); ++i)
                    scaled[i] *= commands_scale_[i];
                obs_vec.insert(obs_vec.end(), scaled.begin(), scaled.end());
            }
            else if (obs_name == "ang_vel")
            {
                // 角速度: 来自IMU陀螺仪
                // 乘以角速度缩放因子
                std::vector<float> scaled = obs_.ang_vel;
                for (auto &v : scaled) v *= ang_vel_scale_;
                obs_vec.insert(obs_vec.end(), scaled.begin(), scaled.end());
            }
            else if (obs_name == "gravity_vec")
            {
                // 重力向量: 在机体坐标系下（实时计算）
                obs_vec.insert(obs_vec.end(), body_gravity.begin(), body_gravity.end());
            }
            else if (obs_name == "dof_pos")
            {
                // 关节位置: 减去默认位置得到相对位置
                // 乘以关节位置缩放因子
                std::vector<float> dof_pos_rel(num_of_dofs_);
                for (int i = 0; i < num_of_dofs_; ++i)
                    dof_pos_rel[i] = obs_.dof_pos[i] - default_dof_pos_[i];
                for (auto &v : dof_pos_rel) v *= dof_pos_scale_;
                obs_vec.insert(obs_vec.end(), dof_pos_rel.begin(), dof_pos_rel.end());
            }
            else if (obs_name == "dof_vel")
            {
                // 关节速度: 直接使用
                // 乘以关节速度缩放因子
                std::vector<float> scaled = obs_.dof_vel;
                for (auto &v : scaled) v *= dof_vel_scale_;
                obs_vec.insert(obs_vec.end(), scaled.begin(), scaled.end());
            }
            else if (obs_name == "actions")
            {
                // 上一步动作: 用于实现时序依赖
                obs_vec.insert(obs_vec.end(), obs_.actions.begin(), obs_.actions.end());
            }
        }

        // 裁剪观测值到限制范围
        std::vector<float> clamped_obs = obs_vec;
        for (auto &v : clamped_obs)
            v = std::max(-clip_obs_, std::min(clip_obs_, v));

        return clamped_obs;
    }

    // ================================================================
    // Forward(): 运行策略模型推理
    //
    // 输入: 观测向量（当前帧 或 当前帧+历史帧）
    //   - 无历史: 45维
    //   - 有历史: 45 x 6 = 270维
    //
    // 处理:
    //   1. 如果配置了历史，将当前观测插入历史缓冲区
    //   2. 获取历史观测向量
    //   3. 调用ONNX模型推理
    //   4. 裁剪输出动作
    //   5. 检查推理结果，失败时回退
    //
    // 返回: 12维动作向量
    // ================================================================
    std::vector<float> Forward(const std::vector<float> &clamped_obs)
    {
        std::vector<float> actions;

        if (!observations_history_.empty())
        {
            // 插入当前观测到历史缓冲区
            history_obs_buf_.insert(clamped_obs);

            // 获取历史观测向量
            history_obs_ = history_obs_buf_.get_obs_vec(observations_history_);

            // 用历史观测推理
            actions = model_->forward({history_obs_});
        }
        else
        {
            // 用当前观测推理
            actions = model_->forward({clamped_obs});
        }

        // --------------------------------------------------------
        // P1 Fix: 检查推理结果是否有效
        // --------------------------------------------------------
        if (actions.empty())
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Model inference returned empty actions! Using last filtered action.");
            return last_filtered_action_;
        }

        if (actions.size() != static_cast<size_t>(num_of_dofs_))
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Model inference returned %zu actions, expected %d! Using last filtered action.",
                actions.size(), num_of_dofs_);
            return last_filtered_action_;
        }

        // 裁剪动作到限制范围
        if (!clip_actions_upper_.empty() && !clip_actions_lower_.empty())
        {
            for (size_t i = 0; i < actions.size(); ++i)
            {
                if (i < clip_actions_lower_.size() && i < clip_actions_upper_.size())
                {
                    actions[i] = std::max(clip_actions_lower_[i], std::min(clip_actions_upper_[i], actions[i]));
                }
            }
        }

        return actions;
    }

    // ================================================================
    // ApplyLowPassFilter(): 低通滤波
    //
    // 滤波公式: filtered = alpha * raw + (1 - alpha) * last_filtered
    //   - alpha = 0.2 (20% 新值, 80% 历史值)
    //   - 平滑高频抖动，保留低频趋势
    //
    // 输入: raw_action - 原始动作
    // 返回: 滤波后动作
    // ================================================================
    std::vector<float> ApplyLowPassFilter(const std::vector<float>& raw_action)
    {
        std::vector<float> filtered_action(num_of_dofs_);

        for (int i = 0; i < num_of_dofs_; ++i)
        {
            // filtered = alpha * raw + (1 - alpha) * last
            filtered_action[i] = action_filter_alpha_ * raw_action[i] +
                                (1.0f - action_filter_alpha_) * last_filtered_action_[i];
        }

        return filtered_action;
    }

    // ================================================================
    // InferenceLoop(): 推理循环（定时器触发，50Hz）
    //
    // 工作流程:
    //   1. 检查RL是否初始化
    //   2. 检查是否有新数据（joint_state + imu）
    //   3. 更新观测变量（从回调数据）
    //   4. 归一化四元数
    //   5. 计算观测向量
    //   6. 运行模型推理
    //   7. 应用低通滤波
    //   8. 计算输出关节位置
    //   9. 发布目标位置
    //   10. 重置数据接收标志
    // ================================================================
    void InferenceLoop()
    {
        // 1. 检查RL是否初始化
        if (!rl_init_done_)
            return;

        std::lock_guard<std::mutex> lock(data_mutex_);

        // 2. 检查是否有新数据（两个话题都收到才推理）
        if (!joint_state_received_ || !imu_received_)
            return;

        // --------------------------------------------------------
        // 3. 更新观测变量
        // --------------------------------------------------------

        // 角速度: 直接使用IMU陀螺仪数据
        for (int i = 0; i < 3; ++i)
            obs_.ang_vel[i] = imu_gyro_[i];

        // 姿态: 直接使用IMU四元数
        for (int i = 0; i < 4; ++i)
            obs_.base_quat[i] = imu_quat_[i];

        // --------------------------------------------------------
        // P2 Fix: 四元数归一化
        // --------------------------------------------------------
        obs_.base_quat = NormalizeQuaternion(obs_.base_quat);

        // 关节位置和速度: 直接使用/joint_state数据
        for (int i = 0; i < num_of_dofs_; ++i)
        {
            obs_.dof_pos[i] = joint_pos_[i];
            obs_.dof_vel[i] = joint_vel_[i];
        }

        // 命令: 设为零（自主模式，无外部命令）
        obs_.commands = {0.0f, 0.0f, 0.0f};

        // --------------------------------------------------------
        // 4-5. 计算观测向量并推理
        // --------------------------------------------------------
        std::vector<float> clamped_obs = ComputeObservation();
        std::vector<float> raw_actions = Forward(clamped_obs);

        // --------------------------------------------------------
        // P2 Fix: 应用低通滤波
        // --------------------------------------------------------
        std::vector<float> filtered_actions = ApplyLowPassFilter(raw_actions);

        // 更新历史滤波动作
        last_filtered_action_ = filtered_actions;

        // --------------------------------------------------------
        // 6. 计算输出关节目标位置
        //    output_pos = filtered_action * action_scale + default_pos
        // --------------------------------------------------------
        std::vector<float> output_dof_pos(num_of_dofs_);
        for (int i = 0; i < num_of_dofs_; ++i)
        {
            float action_scaled = filtered_actions[i] * action_scale_[i];
            output_dof_pos[i] = action_scaled + default_dof_pos_[i];
        }

        // --------------------------------------------------------
        // 7. 发布目标位置到 /uika/target_position
        // --------------------------------------------------------
        std_msgs::msg::Float64MultiArray target_msg;
        target_msg.data.resize(num_of_dofs_);
        for (int i = 0; i < num_of_dofs_; ++i)
            target_msg.data[i] = output_dof_pos[i];
        target_position_pub_->publish(target_msg);

        // --------------------------------------------------------
        // 8. 保存动作用于下一帧
        // --------------------------------------------------------
        last_action_ = raw_actions;
        obs_.actions = filtered_actions;  // 使用滤波后的动作

        // --------------------------------------------------------
        // 9. 重置数据接收标志，等待下一组新数据
        // --------------------------------------------------------
        joint_state_received_ = false;
        imu_received_ = false;
    }

    // ================================================================
    // ROS2 发布器/订阅器
    // ================================================================
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr target_position_pub_;  // 关节目标位置
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_state_sub_; // 关节状态
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr imu_sub_;          // IMU数据
    rclcpp::TimerBase::SharedPtr inference_timer_;                                     // 推理定时器

    // ================================================================
    // 数据缓冲区和锁
    // ================================================================
    std::mutex data_mutex_;              // 线程安全锁
    std::array<float, 12> joint_pos_;   // 关节位置 [q0-q11]
    std::array<float, 12> joint_vel_;   // 关节速度 [dq0-dq11]
    std::array<float, 4> imu_quat_;      // 四元数 [w,x,y,z]
    std::array<float, 3> imu_gyro_;      // 陀螺仪 [gx,gy,gz]
    std::vector<float> last_action_;     // 上一步原始动作

    // ================================================================
    // RL内部状态
    // ================================================================
    Observations<float> obs_;             // 当前观测结构体
    ObservationBuffer history_obs_buf_;  // 历史观测环形缓冲区
    std::vector<float> history_obs_;     // 历史观测向量
    std::unique_ptr<InferenceRuntime::Model> model_;  // ONNX模型
    std::vector<int> obs_dims_;          // 各观测组件维度
    std::vector<std::string> observations_;     // 观测组件名称列表
    std::vector<int> observations_history_;    // 历史观测索引

    // ================================================================
    // 低通滤波
    // ================================================================
    std::vector<float> last_filtered_action_;  // 上一步滤波后动作
    const float action_filter_alpha_ = 0.2f;   // 滤波系数 (20% 新值)

    // ================================================================
    // 配置参数
    // ================================================================
    std::string robot_name_;             // 机器人名称
    int num_of_dofs_;                    // 关节数 (12)
    float dt_;                           // 控制周期 (0.005s)
    int decimation_;                     // 推理降采样 (4)
    float lin_vel_scale_;                // 线速度缩放
    float ang_vel_scale_;                // 角速度缩放 (0.25)
    float dof_pos_scale_;               // 关节位置缩放
    float dof_vel_scale_;                // 关节速度缩放 (0.05)
    float clip_obs_;                     // 观测裁剪范围
    std::vector<float> action_scale_;   // 动作缩放因子
    std::vector<float> commands_scale_; // 命令缩放因子
    std::vector<float> default_dof_pos_; // 默认关节位置（站立）
    std::vector<float> clip_actions_upper_;  // 动作上限
    std::vector<float> clip_actions_lower_;   // 动作下限

    // ================================================================
    // 状态标志
    // ================================================================
    bool joint_state_received_;  // 是否收到关节状态
    bool imu_received_;          // 是否收到IMU数据
    bool rl_init_done_;           // RL是否初始化完成
};

// ================================================================
// 信号处理: 优雅退出
// ================================================================
volatile sig_atomic_t g_shutdown_requested = 0;
void signalHandler(int signum)
{
    std::cout << "Received signal " << signum << ", shutting down..." << std::endl;
    g_shutdown_requested = 1;
}

// ================================================================
// main(): 程序入口
// ================================================================
int main(int argc, char **argv)
{
    // 注册信号处理器
    signal(SIGINT, signalHandler);

    // 初始化ROS2
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting UIKA RL Node...");

    // 创建节点
    auto node = std::make_shared<UINode>();

    // 多线程执行器（处理订阅回调和定时器）
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    // 主循环
    while (rclcpp::ok() && !g_shutdown_requested)
    {
        executor.spin_once(std::chrono::milliseconds(10));
    }

    RCLCPP_INFO(rclcpp::get_logger("main"), "UIKA RL Node shutting down...");
    rclcpp::shutdown();
    return 0;
}
