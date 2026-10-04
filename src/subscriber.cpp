#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

namespace {

// 安全处理结果，便于统计
struct SafetyResult {
  geometry_msgs::msg::Twist msg;
  bool clamped = false;   // 是否发生了钳制
  bool invalid = false;   // 是否包含非法值（NaN / Inf）
};

// 把单个速度分量钳制到 [-limit, limit]；NaN / Inf 一律视为非法，返回 0
double sanitize(double value, double limit, bool& clamped, bool& invalid) {
  if (!std::isfinite(value)) {
    invalid = true;
    return 0.0;
  }
  if (std::fabs(value) > limit) {
    clamped = true;
    return (value > 0.0 ? limit : -limit);
  }
  return value;
}

SafetyResult makeSafe(const geometry_msgs::msg::Twist& in,
                      double max_linear, double max_angular) {
  SafetyResult r;
  r.msg.linear.x = sanitize(in.linear.x, max_linear, r.clamped, r.invalid);
  r.msg.linear.y = sanitize(in.linear.y, max_linear, r.clamped, r.invalid);
  r.msg.linear.z = 0.0;  // 平面移动底盘：竖直方向始终锁死
  r.msg.angular.x = 0.0;  // 不允许翻滚/俯仰，只保留偏航
  r.msg.angular.y = 0.0;
  r.msg.angular.z = sanitize(in.angular.z, max_angular, r.clamped, r.invalid);
  return r;
}

const char* describe(const geometry_msgs::msg::Twist& t) {
  // 用一个静态缓冲拼一句可读的中文速度描述（仅用于日志）
  static thread_local char buf[160];
  std::snprintf(buf, sizeof(buf), "linear[x=%.3f y=%.3f] angular[z=%.3f]",
                t.linear.x, t.linear.y, t.angular.z);
  return buf;
}

}  // namespace

class CmdVelSubscriber : public rclcpp::Node {
 public:
  CmdVelSubscriber() : rclcpp::Node("sub") {
    // ---------- 参数 ----------
    topic_ = declare_parameter<std::string>("topic", "/cmd_vel");
    out_topic_ = declare_parameter<std::string>("out_topic", "safe_cmd_vel");
    timeout_ = declare_parameter<double>("timeout", 0.5);          // 看门狗超时（秒），<=0 关闭
    max_linear_ = declare_parameter<double>("max_linear", 1.0);    // 线速度上限 m/s
    max_angular_ = declare_parameter<double>("max_angular", 2.0);  // 角速度上限 rad/s
    verbose_ = declare_parameter<bool>("verbose", true);           // 是否每次打印收到的速度
    deadband_ = declare_parameter<double>("deadband", 0.0);        // 小于该值视为 0，抑制抖动
    const bool publish_safe = declare_parameter<bool>("publish_safe", true);

    if (max_linear_ <= 0.0 || max_angular_ <= 0.0) {
      RCLCPP_FATAL(get_logger(), "max_linear / max_angular 必须为正数");
      valid_ = false;
      return;
    }
    if (!std::isfinite(timeout_)) {
      RCLCPP_FATAL(get_logger(), "timeout 必须是有限数值");
      valid_ = false;
      return;
    }

    // 订阅 /cmd_vel；队列深度 1，只关心最新指令（控制量过时无意义）
    sub_ = create_subscription<geometry_msgs::msg::Twist>(
        topic_, rclcpp::QoS(1),
        [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) { onCmdVel(msg); });

    // 可选：把安全处理后的速度转发出去
    if (publish_safe) {
      safe_pub_ = create_publisher<geometry_msgs::msg::Twist>(out_topic_, rclcpp::QoS(1));
    }

    // 看门狗定时器：按 timeout 的 1/4 周期检查，最多迟 25% 触发
    last_msg_time_ = std::chrono::steady_clock::now();
    started_ = false;
    if (timeout_ > 0.0) {
      const double check_period = std::max(timeout_ / 4.0, 0.01);
      watchdog_ = create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::duration<double>(check_period)),
          [this]() { onWatchdog(); });
    }

    RCLCPP_INFO(get_logger(), "cmd_vel 订阅节点 [sub] 已启动");
    RCLCPP_INFO(get_logger(), "  订阅话题  : %s  (geometry_msgs/msg/Twist)", topic_.c_str());
    if (safe_pub_) {
      RCLCPP_INFO(get_logger(), "  转发话题  : %s", out_topic_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "  转发话题  : 已关闭（publish_safe=false）");
    }
    RCLCPP_INFO(get_logger(), "  速度上限  : |linear|<=%.2f m/s  |angular|<=%.2f rad/s",
                max_linear_, max_angular_);
    if (timeout_ > 0.0) {
      RCLCPP_INFO(get_logger(), "  看门狗    : %.2f s 无指令则自动停车", timeout_);
    } else {
      RCLCPP_WARN(get_logger(), "  看门狗    : 已关闭（timeout<=0），失联时不会自动停车");
    }
  }

  bool valid() const { return valid_; }

 private:
  // ---------------- 收到 cmd_vel ----------------
  void onCmdVel(const geometry_msgs::msg::Twist::ConstSharedPtr& msg) {
    last_msg_time_ = std::chrono::steady_clock::now();
    started_ = true;
    ++count_;

    // 死区处理：极小值归零，避免底盘因浮点抖动持续微动
    geometry_msgs::msg::Twist in = *msg;
    if (deadband_ > 0.0) {
      if (std::fabs(in.linear.x) < deadband_) in.linear.x = 0.0;
      if (std::fabs(in.linear.y) < deadband_) in.linear.y = 0.0;
      if (std::fabs(in.angular.z) < deadband_) in.angular.z = 0.0;
    }

    const SafetyResult safe = makeSafe(in, max_linear_, max_angular_);

    if (safe.invalid) {
      RCLCPP_ERROR(get_logger(), "收到非法速度值（NaN / Inf），已按 0 处理！原始: %s",
                   describe(*msg));
      ++invalid_count_;
    }
    if (safe.clamped) {
      RCLCPP_WARN(get_logger(), "速度超限已钳制: 收到 %s -> 限制为 %s",
                  describe(*msg), describe(safe.msg));
      ++clamped_count_;
    }

    if (safe_pub_) {
      safe_pub_->publish(safe.msg);
    }

    if (verbose_) {
      // 首帧、以及每 100 帧打印一次，避免刷屏；异常帧上面已经报过
      if (count_ == 1 || count_ % 100 == 0) {
        RCLCPP_INFO(get_logger(), "已接收 %lu 帧 | 当前 %s", count_, describe(safe.msg));
      }
    }
  }

  // ---------------- 看门狗：超时自动停车 ----------------
  void onWatchdog() {
    if (!started_) {
      return;  // 还没收到过任何指令，不报警（避免启动就刷错误）
    }
    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - last_msg_time_).count();
    if (elapsed < timeout_) {
      return;
    }
    if (!stopped_by_watchdog_) {
      stopped_by_watchdog_ = true;
      RCLCPP_ERROR(get_logger(), "%.2f s 未收到 %s 指令，判定上位机失联，速度已清零！",
                   elapsed, topic_.c_str());
      ++timeout_count_;
      if (safe_pub_) {
        geometry_msgs::msg::Twist zero;  // 默认全 0
        safe_pub_->publish(zero);
      }
    }
  }

  // ---------------- 成员 ----------------
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_;

  std::string topic_;
  std::string out_topic_;
  double timeout_ = 0.5;
  double max_linear_ = 1.0;
  double max_angular_ = 2.0;
  double deadband_ = 0.0;
  bool verbose_ = true;
  bool valid_ = true;

  std::chrono::steady_clock::time_point last_msg_time_;
  bool started_ = false;
  bool stopped_by_watchdog_ = false;

  // 统计
  size_t count_ = 0;
  size_t clamped_count_ = 0;
  size_t invalid_count_ = 0;
  size_t timeout_count_ = 0;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CmdVelSubscriber>();
  if (!node->valid()) {
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);  // 纯回调驱动，spin 即可
  rclcpp::shutdown();
  return 0;
}