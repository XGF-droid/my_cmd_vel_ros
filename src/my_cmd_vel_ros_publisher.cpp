#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>
#include<rcl_interfaces/msg/set_parameters_result.hpp>
namespace {

// ---------------- 运动状态字符串 ----------------
constexpr const char* kStateInit = "INIT";
constexpr const char* kStateStop = "STOP";
constexpr const char* kStateMoving = "MOVING";
constexpr const char* kStateEstop = "E_STOP";

}  // namespace

class RobotStatusPublisher : public rclcpp::Node {
 public:
  RobotStatusPublisher() : rclcpp::Node("robot_status_publisher") {
    // ---------- 参数（ROS 2 必须先声明再使用）----------
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    status_topic_ = declare_parameter<std::string>("status_topic", "/robot_status");
    status_rate_ = declare_parameter<double>("status_rate", 10.0);       // 状态发布频率 Hz
    stale_timeout_ = declare_parameter<double>("stale_timeout", 0.5);    // 多久没指令算失联 s
    stopped_speed_ = declare_parameter<double>("stopped_speed", 0.01);   // 小于此速度视为静止
    deadband_ = declare_parameter<double>("deadband", 0.0);              // 死区，0 表示不启用
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");  // 状态帧坐标系
    verbose_ = declare_parameter<bool>("verbose", false);                // 是否打印状态切换日志
    publish_before_first_cmd_ =
        declare_parameter<bool>("publish_before_first_cmd", true);       // 未收到指令时是否也发状态

    // 启动参数校验：不合法就标记失败，由 main 负责报错退出（构造函数里不抛异常）
    valid_ = validateParameters();
    if (!valid_) {
      return;
    }

    // ---------- 发布者：/robot_status ----------
    // 状态是"最新值语义"，队列深度 1 即可，避免监控端读到过期状态
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, rclcpp::QoS(1));

    // ---------- 订阅者：/cmd_vel ----------
    // 控制指令同样只关心最新值，QoS(1)：积压旧指令反而危险
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, rclcpp::QoS(1),
        [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) { onCmdVel(msg); });

    // ---------- 定时器：按固定频率发布状态 ----------
    createStatusTimer();

    // ---------- 运行时参数更新 ----------
    param_cb_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) { return onParametersSet(params); });

    last_cmd_time_ = std::chrono::steady_clock::now();  // 仅用于未收到指令时的占位

    RCLCPP_INFO(get_logger(), "robot_status 状态发布节点已启动");
    RCLCPP_INFO(get_logger(), "  订阅指令  : %s  (geometry_msgs/msg/Twist)", cmd_vel_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  发布状态  : %s  (std_msgs/msg/String, JSON)", status_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  状态频率  : %.1f Hz", status_rate_);
    RCLCPP_INFO(get_logger(), "  看门狗    : %.2f s 无指令 -> E_STOP", stale_timeout_);
    RCLCPP_INFO(get_logger(), "  静止阈值  : %.3f m/s / %.3f rad/s", stopped_speed_, stopped_speed_);
    RCLCPP_INFO(get_logger(), "  在线调参  : ros2 param set /robot_status_publisher stale_timeout 0.2");
  }

  bool valid() const { return valid_; }

 private:
  // ---------------- 参数校验 ----------------
  bool validateParameters() const {
    bool ok = true;
    if (status_rate_ <= 0.0) {
      RCLCPP_ERROR(get_logger(), "status_rate 必须为正数（当前 %.3f）", status_rate_);
      ok = false;
    }
    if (stale_timeout_ <= 0.0) {
      RCLCPP_ERROR(get_logger(), "stale_timeout 必须为正数（当前 %.3f），否则看门狗无意义",
                   stale_timeout_);
      ok = false;
    }
    if (stopped_speed_ < 0.0 || deadband_ < 0.0) {
      RCLCPP_ERROR(get_logger(), "stopped_speed / deadband 不能为负");
      ok = false;
    }
    if (cmd_vel_topic_.empty() || status_topic_.empty()) {
      RCLCPP_ERROR(get_logger(), "话题名不能为空");
      ok = false;
    }
    return ok;
  }

  // ---------------- 收到 /cmd_vel ----------------
  void onCmdVel(const geometry_msgs::msg::Twist::ConstSharedPtr& msg) {
    last_cmd_time_ = std::chrono::steady_clock::now();
    ++cmd_count_;

    // 死区：极小值归零，防止浮点抖动让状态在 STOP / MOVING 之间反复跳
    double vx = msg->linear.x;
    double vy = msg->linear.y;
    double wz = msg->angular.z;
    if (deadband_ > 0.0) {
      if (std::fabs(vx) < deadband_) vx = 0.0;
      if (std::fabs(vy) < deadband_) vy = 0.0;
      if (std::fabs(wz) < deadband_) wz = 0.0;
    }

    // 非法值保护：NaN / Inf 一律按 0 处理并计数，不能让脏数据污染上报状态
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz)) {
      RCLCPP_WARN(get_logger(), "收到含 NaN / Inf 的指令速度，已按 0 处理");
      vx = vy = wz = 0.0;
      ++invalid_count_;
    }

    cmd_vx_ = vx;
    cmd_vy_ = vy;
    cmd_wz_ = wz;
    linear_speed_ = std::hypot(cmd_vx_, cmd_vy_);
    angular_speed_ = std::fabs(cmd_wz_);

    // 一旦重新收到指令，立即恢复在线状态
    if (!ever_received_ || !online_) {
      if (ever_received_) {
        RCLCPP_INFO(get_logger(), "指令通道恢复：重新收到 %s", cmd_vel_topic_.c_str());
      }
      online_ = true;
    }
    ever_received_ = true;

    if (isStopped()) {
      ++stop_count_;
    }
  }

  // ---------------- 是否视为静止 ----------------
  bool isStopped() const {
    return linear_speed_ <= stopped_speed_ && angular_speed_ <= stopped_speed_;
  }

  // ---------------- 状态机 ----------------
  // INIT  : 还没收到过任何指令
  // E_STOP: 收到过指令，但已超时 -> 判定上位机失联
  // STOP  : 指令新鲜且速度约等于 0
  // MOVING: 指令新鲜且速度非 0
  void updateMotionState(bool stale) {
    const char* next;
    if (!ever_received_) {
      next = kStateInit;
    } else if (stale) {
      next = kStateEstop;
    } else {
      next = isStopped() ? kStateStop : kStateMoving;
    }

    if (std::string(next) != motion_state_) {
      const char* prev = motion_state_;
      motion_state_ = next;
      // 状态切换是关键事件，无论 verbose 都打出来（状态没变时不刷屏）
      if (verbose_ || std::string(next) == kStateEstop || std::string(prev) == kStateEstop) {
        RCLCPP_INFO(get_logger(), "运动状态切换: %s -> %s", prev, next);
      }
    }
  }

  // ---------------- 定时器 ----------------
  void createStatusTimer() {
    const auto period = std::chrono::duration<double>(1.0 / status_rate_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() { onStatusTimer(); });
  }

  void onStatusTimer() {
    // 频率被动态改过：重建定时器（只在参数变化后执行一次）
    if (rate_changed_) {
      rate_changed_ = false;
      applied_rate_hz_ = status_rate_;
      createStatusTimer();
      RCLCPP_INFO(get_logger(), "状态发布频率已改为 %.1f Hz", status_rate_);
    }

    // 未收到任何指令时，按参数决定是否发状态
    if (!ever_received_ && !publish_before_first_cmd_) {
      return;
    }

    // 注意：看门狗用单调时钟 steady_clock，且局部变量不能命名为 now，
    // 否则会遮蔽 rclcpp::Node::now()（下面取 ROS 时间时要用）
    const auto now_mono = std::chrono::steady_clock::now();
    const double age = ever_received_
                           ? std::chrono::duration<double>(now_mono - last_cmd_time_).count()
                           : -1.0;  // -1 表示"从未收到过指令"

    // ---------- 看门狗 ----------
    const bool stale = (age >= 0.0) && (age > stale_timeout_);
    if (ever_received_ && stale && online_) {
      online_ = false;
      ++timeout_count_;
      RCLCPP_ERROR(get_logger(), "%.2f s 未收到 %s 指令，判定上位机失联，状态置为 E_STOP",
                   age, cmd_vel_topic_.c_str());
    }

    updateMotionState(stale);

    // ---------- 组装 JSON 状态 ----------
    // 时间戳拆成 sec / nanosec 两个整数，避免直接打印 double 丢精度
    const rclcpp::Time stamp = now();
    const int64_t ns = stamp.nanoseconds();

    char buf[512];
    std::snprintf(buf, sizeof(buf),
                  "{\"sec\":%lld,\"nanosec\":%lld,"
                  "\"frame_id\":\"%s\",\"node_name\":\"%s\",\"source_topic\":\"%s\","
                  "\"online\":%s,\"stale\":%s,\"motion_state\":\"%s\","
                  "\"cmd_vel\":{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f},"
                  "\"cmd_age\":%.6f,\"linear_speed\":%.6f,\"angular_speed\":%.6f,"
                  "\"cmd_count\":%llu,\"timeout_count\":%llu,\"stop_count\":%llu}",
                  static_cast<long long>(ns / 1000000000LL),
                  static_cast<long long>(ns % 1000000000LL),
                  frame_id_.c_str(), get_name(), cmd_vel_topic_.c_str(),
                  online_ ? "true" : "false", stale ? "true" : "false", motion_state_,
                  cmd_vx_, cmd_vy_, cmd_wz_,
                  age, linear_speed_, angular_speed_,
                  static_cast<unsigned long long>(cmd_count_),
                  static_cast<unsigned long long>(timeout_count_),
                  static_cast<unsigned long long>(stop_count_));

    std_msgs::msg::String out;
    out.data = buf;
    status_pub_->publish(out);

    if (verbose_ && !logged_first_) {
      logged_first_ = true;
      RCLCPP_INFO(get_logger(), "已开始发布状态: %s", buf);
    }
  }

  // ---------------- 参数写入校验 + 生效 ----------------
  rcl_interfaces::msg::SetParametersResult onParametersSet(const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    // 第一遍：先全部校验，任一不合法整体拒绝，不污染当前状态
    for (const auto& p : params) {
      const std::string& name = p.get_name();
      if (name == "status_rate" || name == "stale_timeout") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE || p.as_double() <= 0.0) {
          result.successful = false;
          result.reason = name + " 必须为正的浮点数";
          return result;
        }
      } else if (name == "stopped_speed" || name == "deadband") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE || p.as_double() < 0.0) {
          result.successful = false;
          result.reason = name + " 不能为负";
          return result;
        }
      }
    }

    // 第二遍：写回成员变量，立即生效
    for (const auto& p : params) {
      const std::string& name = p.get_name();
      if (name == "status_rate") {
        const double new_rate = p.as_double();
        // 只有频率真的变化了才重建定时器（避免无意义的重建）
        if (std::fabs(new_rate - applied_rate_hz_) > 1e-6) {
          status_rate_ = new_rate;
          rate_changed_ = true;  // 定时器重建交给定时器回调处理
        }
      } else if (name == "stale_timeout") {
        stale_timeout_ = p.as_double();
      } else if (name == "stopped_speed") {
        stopped_speed_ = p.as_double();
      } else if (name == "deadband") {
        deadband_ = p.as_double();
      } else if (name == "frame_id") {
        frame_id_ = p.as_string();
      } else if (name == "verbose") {
        verbose_ = p.as_bool();
      } else if (name == "publish_before_first_cmd") {
        publish_before_first_cmd_ = p.as_bool();
      }
    }
    return result;
  }

  // ---------------- 成员 ----------------
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // 参数
  std::string cmd_vel_topic_, status_topic_, frame_id_;
  double status_rate_ = 10.0;
  double stale_timeout_ = 0.5;
  double stopped_speed_ = 0.01;
  double deadband_ = 0.0;
  bool verbose_ = false;
  bool publish_before_first_cmd_ = true;
  bool valid_ = true;

  double applied_rate_hz_ = 10.0;  // 已生效频率，用于判断是否需要重建定时器
  bool rate_changed_ = false;

  // 最近一次指令与状态
  double cmd_vx_ = 0.0, cmd_vy_ = 0.0, cmd_wz_ = 0.0;
  double linear_speed_ = 0.0, angular_speed_ = 0.0;
  bool ever_received_ = false;
  bool online_ = false;
  bool logged_first_ = false;
  const char* motion_state_ = kStateInit;  // 指向常量字符串，无需管理内存

  // 计时（看门狗用单调时钟）
  std::chrono::steady_clock::time_point last_cmd_time_;

  // 统计
  uint64_t cmd_count_ = 0;
  uint64_t timeout_count_ = 0;
  uint64_t stop_count_ = 0;
  uint64_t invalid_count_ = 0;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RobotStatusPublisher>();
  if (!node->valid()) {
    RCLCPP_FATAL(rclcpp::get_logger("robot_status_publisher"), "参数非法，节点启动失败");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}