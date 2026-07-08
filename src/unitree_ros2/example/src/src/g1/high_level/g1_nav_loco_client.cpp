#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <g1/g1_loco_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unitree/idl/ros2/Twist_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

class G1NavLocoClientNode : public rclcpp::Node {
 public:
  G1NavLocoClientNode(int domain_id, std::string network,
                      std::string cmd_vel_topic, bool auto_velocity)
      : Node("g1_nav_loco_client"),
        client_(this),
        domain_id_(domain_id),
        network_(std::move(network)),
        cmd_vel_topic_(std::move(cmd_vel_topic)),
        auto_velocity_(auto_velocity) {
    unitree::robot::ChannelFactory::Instance()->Init(domain_id_, network_);

    setupRobot();

    const auto dds_topic = rosTopicToDdsTopic(cmd_vel_topic_);
    cmd_vel_sub_ = std::make_shared<
        unitree::robot::ChannelSubscriber<geometry_msgs::msg::dds_::Twist_>>(
        dds_topic,
        [this](const void *message) {
          const auto *twist =
              static_cast<const geometry_msgs::msg::dds_::Twist_ *>(message);
          onCmdVel(twist);
        },
        10);
    cmd_vel_sub_->InitChannel();

    watchdog_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&G1NavLocoClientNode::watchdog, this));

    RCLCPP_INFO(this->get_logger(),
                "G1 high-level loco nav controller ready. DDS cmd_vel topic: %s",
                dds_topic.c_str());
  }

  ~G1NavLocoClientNode() override {
    try {
      client_.StopMove();
      client_.Damp();
    } catch (...) {
    }
  }

 private:
  static std::string rosTopicToDdsTopic(const std::string &topic) {
    if (topic.rfind("rt/", 0) == 0) {
      return topic;
    }
    if (!topic.empty() && topic.front() == '/') {
      return "rt" + topic;
    }
    return "rt/" + topic;
  }

  void setupRobot() {
    auto ret = client_.Damp();
    if (ret != 0) {
      RCLCPP_WARN(this->get_logger(), "Damp failed with code %d", ret);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    ret = client_.StandUp();
    if (ret != 0) {
      RCLCPP_WARN(this->get_logger(), "StandUp failed with code %d", ret);
    }

    int fsm_mode = -1;
    float stand_height = 0.0f;
    while (stand_height < 0.5f) {
      stand_height += 0.02f;
      ret = client_.SetStandHeight(stand_height);
      if (ret != 0) {
        RCLCPP_WARN(this->get_logger(), "SetStandHeight(%.2f) failed with code %d",
                    stand_height, ret);
      }

      ret = client_.GetFsmMode(fsm_mode);
      if (ret == 0) {
        RCLCPP_INFO(this->get_logger(), "FSM mode %d at stand height %.2f",
                    fsm_mode, stand_height);
        if (fsm_mode == 0 && stand_height > 0.2f) {
          break;
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    ret = client_.BalanceStand();
    if (ret != 0) {
      RCLCPP_WARN(this->get_logger(), "BalanceStand failed with code %d", ret);
    }

    ret = client_.SetStandHeight(stand_height);
    if (ret != 0) {
      RCLCPP_WARN(this->get_logger(), "SetStandHeight(%.2f) failed with code %d",
                  stand_height, ret);
    }

    if (auto_velocity_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));

      ret = client_.ContinuousGait(true);
      if (ret != 0) {
        RCLCPP_WARN(this->get_logger(), "ContinuousGait(true) failed with code %d",
                    ret);
      }

      client_.SwitchMoveMode(true);
    } else {
      client_.SwitchMoveMode(false);
      RCLCPP_INFO(this->get_logger(), "Stand-only mode enabled; gait is not started.");
    }

    last_cmd_time_ = std::chrono::steady_clock::now();
  }

  void onCmdVel(const geometry_msgs::msg::dds_::Twist_ *msg) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    last_vx_ = static_cast<float>(msg->linear().x());
    last_vy_ = static_cast<float>(msg->linear().y());
    last_wz_ = static_cast<float>(msg->angular().z());
    last_cmd_time_ = std::chrono::steady_clock::now();
    has_command_ = true;
    stop_sent_ = false;
  }

  bool isZeroCommand() const {
    constexpr float deadband = 0.02f;
    return std::fabs(last_vx_) < deadband && std::fabs(last_vy_) < deadband &&
           std::fabs(last_wz_) < deadband;
  }

  void watchdog() {
    if (!auto_velocity_ || !has_command_) {
      return;
    }

    float vx = 0.0f;
    float vy = 0.0f;
    float wz = 0.0f;
    auto last_cmd_time = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      vx = last_vx_;
      vy = last_vy_;
      wz = last_wz_;
      last_cmd_time = last_cmd_time_;
    }

    const auto elapsed = std::chrono::steady_clock::now() - last_cmd_time;
    if (elapsed > std::chrono::milliseconds(500) ||
        (std::fabs(vx) < 0.02f && std::fabs(vy) < 0.02f && std::fabs(wz) < 0.02f)) {
      if (!stop_sent_) {
        auto ret = client_.StopMove();
        if (ret != 0) {
          RCLCPP_WARN(this->get_logger(), "StopMove failed with code %d", ret);
        }
        stop_sent_ = true;
      }
      return;
    }

    auto ret = client_.Move(vx, vy, wz, true);
    if (ret != 0) {
      RCLCPP_WARN(this->get_logger(), "Move failed with code %d", ret);
    }
  }

  std::mutex command_mutex_;
  float last_vx_{0.0f};
  float last_vy_{0.0f};
  float last_wz_{0.0f};
  bool has_command_{false};
  bool stop_sent_{false};
  std::chrono::steady_clock::time_point last_cmd_time_{};

  unitree::robot::g1::LocoClient client_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<geometry_msgs::msg::dds_::Twist_>>
      cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  int domain_id_{0};
  std::string network_;
  std::string cmd_vel_topic_;
  bool auto_velocity_{true};
};

static std::map<std::string, std::string> parseArgs(int argc, char **argv) {
  std::map<std::string, std::string> args;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg.rfind("--", 0) != 0) {
      continue;
    }

    const auto pos = arg.find('=');
    std::string key;
    std::string value;
    if (pos != std::string::npos) {
      key = arg.substr(2, pos - 2);
      value = arg.substr(pos + 1);
    } else {
      key = arg.substr(2);
      if (i + 1 < argc) {
        std::string next_arg = argv[i + 1];
        if (next_arg.rfind("--", 0) != 0) {
          value = next_arg;
          ++i;
        }
      }
    }

    if (value.size() >= 2 && value.front() == '"' && value.back() == '"') {
      value = value.substr(1, value.size() - 2);
    }

    args[key] = value;
  }
  return args;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  const auto args = parseArgs(argc, argv);
  const auto network = args.count("network") ? args.at("network") : std::string("lo");
  const auto cmd_vel_topic = args.count("cmd_vel_topic") ? args.at("cmd_vel_topic") : std::string("/cmd_vel");
  const int domain_id = args.count("domain") ? std::stoi(args.at("domain")) : 0;
  const bool auto_velocity = !args.count("no_auto_velocity");

  if (args.count("keyboard")) {
    std::cout << "Keyboard mode is not supported in g1_nav_loco_client; using cmd_vel flow.\n";
  }

  auto node = std::make_shared<G1NavLocoClientNode>(domain_id, network, cmd_vel_topic, auto_velocity);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}