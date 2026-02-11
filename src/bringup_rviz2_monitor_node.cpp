#include "rclcpp/rclcpp.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include <string>
#include <sstream>
#include <vector>
#include <map>

using namespace std::chrono_literals;

class BringupRviz2Monitor : public rclcpp::Node
{
public:
  BringupRviz2Monitor(const std::vector<std::string> & rviz_targets)
  : Node("bringup_rviz2_monitor"), controllers_(rviz_targets), rviz_launched_(false)
  {
    this->declare_parameter<std::string>("pkg_name", "");
    this->declare_parameter<std::string>("robot_pkg_path", "");
    this->declare_parameter<double>("poll_period_sec", 0.05);
    this->declare_parameter<double>("timeout_sec", 30.0);  // 0なら無限待ち

    pkg_name_ = this->get_parameter("pkg_name").as_string();
    robot_pkg_path_ = this->get_parameter("robot_pkg_path").as_string();
    poll_period_sec_ = this->get_parameter("poll_period_sec").as_double();
    timeout_sec_ = this->get_parameter("timeout_sec").as_double();

    for (const auto & name : rviz_targets) {
      states_[name] = "unknown";
      last_reported_states_[name] = "";  // 初回は必ずログを出すため
    }

      // ---- サブスクライバ ----
    cli_ = this->create_client<controller_manager_msgs::srv::ListControllers>(
      "/controller_manager/list_controllers");

    start_time_ = this->now();

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(poll_period_sec_),
      std::bind(&BringupRviz2Monitor::poll, this));

    RCLCPP_INFO(this->get_logger(), "Monitoring controllers via /controller_manager/list_controllers");
  }

private:
  void poll() {
    if (rviz_launched_) return;

    if (timeout_sec_ > 0.0) {
      const double elapsed = (this->now() - start_time_).seconds();
      if (elapsed > timeout_sec_) {
        RCLCPP_ERROR(this->get_logger(), "Timeout waiting controllers to become active.");        
        rclcpp::shutdown();
      }
    }

    if (!cli_->service_is_ready()) {
      // controller_manager がまだ立っていない
      return;
    }

    auto req = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    auto fut = cli_->async_send_request(req,
      std::bind(&BringupRviz2Monitor::on_list, this, std::placeholders::_1));
  }

  void on_list(rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture future) {
    if (rviz_launched_) return;

    controller_manager_msgs::srv::ListControllers::Response::SharedPtr res;
    try {
      res = future.get();
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "list_controllers call failed: %s", e.what());
      return;
    }

    // 取得結果を states_ に反映
    std::map<std::string, std::string> latest;
    for (const auto & c : res->controller) {
      latest[c.name] = c.state;  // "active" / "inactive" / ...
    }

    for (auto & kv : states_) {
      auto it = latest.find(kv.first);
      if (it == latest.end()) kv.second = "unknown";
      else kv.second = it->second;
    }

    // 判定
    if (!all_active("rviz", controllers_)) {
      return;
    }

    RCLCPP_INFO(this->get_logger(), "All required controllers are active. Launching RViz...");

    rviz_launched_ = true;
    if (timer_) {
      timer_->cancel();
    }

    if (!rclcpp::ok()) return;
    rclcpp::shutdown();
  }

  bool all_active(const std::string &label,
                  const std::vector<std::string> &targets) {
    bool ok = true;
    for (const auto & name : targets) {
      auto it = states_.find(name);
      if (it == states_.end()) {
        if (last_reported_states_[name] != "missing") {
          RCLCPP_WARN(this->get_logger(),
                      "[%s not register] %s's state does not exist in states_",
                      label.c_str(), name.c_str());
          last_reported_states_[name] = "missing";
        }
        ok = false;
      } else if (it->second != "active") {
        if (last_reported_states_[name] != it->second) {
          RCLCPP_INFO(this->get_logger(),
                      "[%s not completed] %s state=%s",
                      label.c_str(), name.c_str(), it->second.c_str());
          last_reported_states_[name] = it->second;
        }
        ok = false;
      } else {
        // active になった瞬間だけ出したい場合（任意）
        if (last_reported_states_[name] != "active") {
          RCLCPP_INFO(this->get_logger(),
                      "[%s completed] %s is active",
                      label.c_str(), name.c_str());
          last_reported_states_[name] = "active";
        }
      }
    }
    return ok;
  }

  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr cli_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;

  std::vector<std::string> controllers_;
  std::map<std::string, std::string> states_;

  bool rviz_launched_;
  std::string pkg_name_;
  std::string robot_pkg_path_;
  double poll_period_sec_;
  double timeout_sec_;
  std::map<std::string, std::string> last_reported_states_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::vector<std::string> rviz_nodes = {
    "joint_state_broadcaster",
    "mechanum_controller",
  };

  auto node = std::make_shared<BringupRviz2Monitor>(rviz_nodes);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

