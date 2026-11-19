// SPDX-License-Identifier: Apache-2.0
//
// ROS 2 适配节点：把消息接到 TeleopCore 上。
//
// 本文件只做四件事：
//   1. 参数 -> TeleopCore::Config / IK 后端
//   2. 订阅 -> core.submit*
//   3. 200Hz 定时器 -> core.tick -> 发布 JointStream
//   4. 降频发布 TeleopStatus
//
// 不含任何插值、状态机或运动学逻辑——那些全在 core/ 里。
//
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <flex_teleop/teleop_core.hpp>
#include <flex_teleop_msgs/msg/joint_feedback.hpp>
#include <flex_teleop_msgs/msg/joint_stream.hpp>
#include <flex_teleop_msgs/msg/joint_target.hpp>
#include <flex_teleop_msgs/msg/pose_frame.hpp>
#include <flex_teleop_msgs/msg/teleop_status.hpp>

#include "conversions.hpp"

#ifdef FLEX_TELEOP_WITH_QP_IK
#include <flex_teleop/qp_ik_backend.hpp>
#endif

namespace flex_teleop_ros {
namespace {

constexpr double kPi = 3.14159265358979323846;

class TeleopNode : public rclcpp::Node {
 public:
  TeleopNode() : Node("flex_teleop_node") {
    const int side_i = declare_parameter<int>("side", 0);
    const double delay_s = declare_parameter<double>("fixed_delay_s", 0.035);
    const double out_rate_hz = declare_parameter<double>("output_rate_hz", 200.0);
    const double status_rate_hz = declare_parameter<double>("status_rate_hz", 20.0);
    const int degraded_frames = declare_parameter<int>("degraded_hold_frames", 20);
    const int fault_frames = declare_parameter<int>("fault_hold_frames", 100);

    const std::vector<double> lo =
        declare_parameter<std::vector<double>>("joint_lower", std::vector<double>(7, -kPi));
    const std::vector<double> hi =
        declare_parameter<std::vector<double>>("joint_upper", std::vector<double>(7, kPi));
    const std::string ik_type = declare_parameter<std::string>("ik_type", "none");

    flex_teleop::TeleopCore::Config cfg;
    cfg.side = (side_i == 1) ? flex_teleop::Side::kRight : flex_teleop::Side::kLeft;
    cfg.fixed_delay_s = delay_s;
    cfg.output_rate_hz = out_rate_hz;
    cfg.degraded_hold_frames = degraded_frames;
    cfg.fault_hold_frames = fault_frames;

    core_ = std::make_unique<flex_teleop::TeleopCore>(cfg);

    if (lo.size() == static_cast<size_t>(flex_teleop::kDof) &&
        hi.size() == static_cast<size_t>(flex_teleop::kDof)) {
      double l[flex_teleop::kDof];
      double u[flex_teleop::kDof];
      for (int i = 0; i < flex_teleop::kDof; ++i) {
        l[i] = lo[i];
        u[i] = hi[i];
      }
      core_->setJointLimits(l, u);
    } else {
      RCLCPP_ERROR(get_logger(), "joint_lower/upper 长度必须为 7，改用默认值");
    }

    setupIK(ik_type);

    // 订阅
    pose_sub_ = create_subscription<flex_teleop_msgs::msg::PoseFrame>(
        "pose_target", rclcpp::SensorDataQoS(),
        [this](const flex_teleop_msgs::msg::PoseFrame::SharedPtr m) {
          // ★ 传本地到达时刻，不是 msg.stamp：跨机时钟不同源，
          //   stamp 只做诊断（见 docs/04-interface.md「帧身份」）。
          const std::int64_t t_arrival = now().nanoseconds();
          std::lock_guard<std::mutex> lk(mu_);
          core_->submitPose(toPose(m->t_base_tcp), m->pose_id, t_arrival);
        });

    joint_sub_ = create_subscription<flex_teleop_msgs::msg::JointTarget>(
        "joint_target", rclcpp::SensorDataQoS(),
        [this](const flex_teleop_msgs::msg::JointTarget::SharedPtr m) {
          double q[flex_teleop::kDof];
          fromArray7(m->q, q);
          const std::int64_t t_arrival = now().nanoseconds();
          std::lock_guard<std::mutex> lk(mu_);
          core_->submitJoint(q, m->pose_id, t_arrival);
        });

    fb_sub_ = create_subscription<flex_teleop_msgs::msg::JointFeedback>(
        "joint_feedback", rclcpp::SensorDataQoS(),
        [this](const flex_teleop_msgs::msg::JointFeedback::SharedPtr m) {
          flex_teleop::JointFeedback fb;
          fb.tick_seq = m->tick_seq;
          fb.side = static_cast<flex_teleop::Side>(m->side);
          fromArray7(m->q, fb.q);
          fromArray7(m->qd, fb.qd);
          fromArray7(m->tau, fb.tau);
          std::lock_guard<std::mutex> lk(mu_);
          core_->submitFeedback(fb);
        });

    // 发布
    stream_pub_ = create_publisher<flex_teleop_msgs::msg::JointStream>(
        "joint_stream", rclcpp::QoS(10).reliable());
    status_pub_ = create_publisher<flex_teleop_msgs::msg::TeleopStatus>(
        "teleop_status", rclcpp::SensorDataQoS());

    status_div_ = std::max(1, static_cast<int>(out_rate_hz / std::max(1.0, status_rate_hz)));

    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(1.0, out_rate_hz)));
    timer_ = create_wall_timer(period_ns, [this]() { onTick(); });

    RCLCPP_INFO(get_logger(),
                "flex_teleop_node 启动：side=%d, %.0fHz, delay=%.3fs, ik=%s",
                side_i, out_rate_hz, delay_s, ik_type.c_str());
  }

 private:
  void setupIK(const std::string& ik_type) {
    if (ik_type == "none") {
      return;
    }
#ifdef FLEX_TELEOP_WITH_QP_IK
    if (ik_type == "qp") {
      flex_teleop::QpIkBackend::Config qc;

      const auto a = declare_parameter<std::vector<double>>("qp.a", std::vector<double>(7, 0.0));
      const auto alpha = declare_parameter<std::vector<double>>(
          "qp.alpha", std::vector<double>{0.0, kPi / 2, -kPi / 2, kPi / 2, -kPi / 2, kPi / 2, 0.0});
      const auto d = declare_parameter<std::vector<double>>(
          "qp.d", std::vector<double>{0.30, 0.0, 0.30, 0.0, 0.20, 0.0, 0.10});
      const auto qlo =
          declare_parameter<std::vector<double>>("qp.lower", std::vector<double>(7, -2.9));
      const auto qhi =
          declare_parameter<std::vector<double>>("qp.upper", std::vector<double>(7, 2.9));

      if (a.size() != 7 || alpha.size() != 7 || d.size() != 7 || qlo.size() != 7 ||
          qhi.size() != 7) {
        RCLCPP_ERROR(get_logger(), "qp 几何参数长度必须为 7，IK 未启用");
        return;
      }
      for (int i = 0; i < 7; ++i) {
        qc.geometry.a[i] = a[i];
        qc.geometry.alpha[i] = alpha[i];
        qc.geometry.d[i] = d[i];
        qc.geometry.lower[i] = qlo[i];
        qc.geometry.upper[i] = qhi[i];
      }
      qc.tick_s = declare_parameter<double>("qp.tick_s", 0.005);
      qc.one_shot = declare_parameter<bool>("qp.one_shot", false);

      try {
        qp_ik_ = std::make_unique<flex_teleop::QpIkBackend>(qc);
        core_->setIK(qp_ik_.get());
        RCLCPP_INFO(get_logger(), "QP IK 后端已启用（%s）",
                    qc.one_shot ? "one-shot" : "servo");
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "QP IK 构造失败：%s", e.what());
      }
      return;
    }
    RCLCPP_WARN(get_logger(), "未知 ik_type=%s（可选：none / qp）", ik_type.c_str());
#else
    RCLCPP_ERROR(get_logger(), "本构建未启用 QP IK（重新编译需 -DFLEX_TELEOP_WITH_QP_IK=ON）");
#endif
  }

  void onTick() {
    const std::int64_t t_ns = now().nanoseconds();

    flex_teleop_msgs::msg::JointStream stream_msg;
    {
      std::lock_guard<std::mutex> lk(mu_);
      const flex_teleop::JointStream s = core_->tick(t_ns);
      fromCore(s, t_ns, stream_msg);

      if (++status_counter_ % status_div_ == 0) {
        flex_teleop_msgs::msg::TeleopStatus st_msg;
        fromCore(core_->status(), st_msg);
        status_msg_ = st_msg;
        publish_status_ = true;
      } else {
        publish_status_ = false;
      }
    }

    stream_pub_->publish(stream_msg);
    if (publish_status_) {
      status_pub_->publish(status_msg_);
    }
  }

  std::mutex mu_;
  std::unique_ptr<flex_teleop::TeleopCore> core_;
#ifdef FLEX_TELEOP_WITH_QP_IK
  std::unique_ptr<flex_teleop::QpIkBackend> qp_ik_;
#endif

  rclcpp::Subscription<flex_teleop_msgs::msg::PoseFrame>::SharedPtr pose_sub_;
  rclcpp::Subscription<flex_teleop_msgs::msg::JointTarget>::SharedPtr joint_sub_;
  rclcpp::Subscription<flex_teleop_msgs::msg::JointFeedback>::SharedPtr fb_sub_;
  rclcpp::Publisher<flex_teleop_msgs::msg::JointStream>::SharedPtr stream_pub_;
  rclcpp::Publisher<flex_teleop_msgs::msg::TeleopStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  int status_div_{10};
  std::uint64_t status_counter_{0};
  bool publish_status_{false};
  flex_teleop_msgs::msg::TeleopStatus status_msg_;
};

}  // namespace
}  // namespace flex_teleop_ros

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<flex_teleop_ros::TeleopNode>());
  rclcpp::shutdown();
  return 0;
}
