// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under #include <memory>the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "trajectory_follower.hpp"

#include <fstream>
#include <limits>

namespace autoware_practice_course
{

TrajectoryFollowerNode::TrajectoryFollowerNode() : Node("trajectory_follower"), kp_(0.0), lookahead_distance_(5.0)
{
  using std::placeholders::_1;
  declare_parameter<double>("kp", kp_);
  get_parameter("kp", kp_);
  declare_parameter<double>("lookahead_distance", lookahead_distance_);
  get_parameter("lookahead_distance", lookahead_distance_);
  param_file_ = "./src/autoware_practice_simulator/config/vehicle.param.yaml";
  wheel_base_ = load_parameters(param_file_, "wheel_base");

  RCLCPP_INFO(this->get_logger(), "Wheel base: %f", wheel_base_);

  pub_command_ = create_publisher<AckermannControlCommand>("/control/command/control_cmd", rclcpp::QoS(1));
  sub_trajectory_ = create_subscription<Trajectory>(
    "/planning/scenario_planning/trajectory", rclcpp::QoS(1),
    std::bind(&TrajectoryFollowerNode::update_target_velocity, this, _1));
  sub_kinematic_state_ = create_subscription<Odometry>(
    "/localization/kinematic_state", rclcpp::QoS(1),
    std::bind(&TrajectoryFollowerNode::update_current_state, this, _1));

  const auto period = rclcpp::Rate(10).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this] { on_timer(); });
}

void TrajectoryFollowerNode::update_target_velocity(const Trajectory & msg)
{
  double min_distance = std::numeric_limits<double>::max();

  for (size_t i = 0; i < msg.points.size(); ++i) {
    double dx = msg.points[i].pose.position.x - current_position_.x;
    double dy = msg.points[i].pose.position.y - current_position_.y;
    double dz = msg.points[i].pose.position.z - current_position_.z;
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (distance < min_distance) {
      min_distance = distance;
      closest_point_index_ = i;
    }
  }

  trajectory_ = msg;
  target_velocity_ = msg.points[closest_point_index_].longitudinal_velocity_mps;
};

// 機能説明 : パラメータファイルからパラメータを読み込む
// Retrieves the value of the specified parameter from the YAML file
double TrajectoryFollowerNode::load_parameters(const std::string & param_file, const std::string & param_tag)

{
  std::ifstream file(param_file);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open parameter file: %s", param_file.c_str());
    return -1.0;
  }

  // Read the file line by line
  std::string line;
  
  //ファイルから一行ずつ読み込む
  while (std::getline(file, line)) {
    //パラメータタグが見つかったら、その行をパースしてパラメータ値を取得
    if (line.find(param_tag + ":") != std::string::npos) {
      size_t pos = line.find(":");
      //"param_tag: param_value"の形式でパラメータが記述されているので、":"の位置を基準にパラメータ値を取得
      if (pos != std::string::npos) {
        // ":" の後にある文字列を double 型に変換する
        double param_value = std::stod(line.substr(pos + 1)); //pos+1 以降の文字列を取得
        return param_value;
      }
    }
  }

  RCLCPP_ERROR(this->get_logger(), "Parameter %s not found in file: %s", param_tag.c_str(), param_file.c_str());
  return -1.0;
}

void TrajectoryFollowerNode::update_current_state(const Odometry & msg)
{
  current_velocity_ = msg.twist.twist.linear.x;
  current_position_ = msg.pose.pose.position;
  current_orientation_ = msg.pose.pose.orientation; //クォータニオン
};

void TrajectoryFollowerNode::on_timer()
{
  const auto stamp = now();

  AckermannControlCommand command;
  command.stamp = stamp;

  double velocity_error = target_velocity_ - current_velocity_; //速度誤差を計算
  command.longitudinal.acceleration = longitudinal_controller(velocity_error); //速度誤差を元に加速度を計算、別ノードで実装
  command.longitudinal.speed = target_velocity_; //目標速度を設定

  command.lateral.steering_tire_angle = lateral_controller(); //ステアリング角を計算,下の関数で実装

  pub_command_->publish(command);
}

double TrajectoryFollowerNode::longitudinal_controller(double velocity_error)
{
  return kp_ * velocity_error;
}


// 機能説明 : 与えられた軌道に対して、最も近い点から見た前方の点を探し、その点を目標点として、目標点に向かうような制御を行う

double TrajectoryFollowerNode::lateral_controller()
{
  double min_distance = std::numeric_limits<double>::max();
  size_t lookahead_point_index = closest_point_index_;
  min_distance = std::numeric_limits<double>::max();


  // Find the lookahead point
  // 最も近い点から見て、前方にある点を探す
  for (size_t i = closest_point_index_; i < trajectory_.points.size(); ++i) {
    double dx = trajectory_.points[i].pose.position.x - current_position_.x;
    double dy = trajectory_.points[i].pose.position.y - current_position_.y;
    double dz = trajectory_.points[i].pose.position.z - current_position_.z;
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz); //2点間の距離を計算、
    if (distance >= lookahead_distance_ && distance < min_distance) { //目標点との距離がlookahead_distance_以上で、最小距離を更新
      min_distance = distance;
      lookahead_point_index = i;
    }
  }
  if (lookahead_point_index == closest_point_index_) { //目標点が見つからなかった場合
    RCLCPP_WARN(this->get_logger(), "No valid lookahead point found."); //警告を出力
    return 0.0;
  }
  double dx = trajectory_.points[lookahead_point_index].pose.position.x - current_position_.x; //目標点とのx座標の差分
  double dy = trajectory_.points[lookahead_point_index].pose.position.y - current_position_.y;  
  double alpha = std::atan2(dy, dx) - calculate_yaw_from_quaternion(current_orientation_); //目標点との角度の差分
  double steering_angle = std::atan2(2.0 * wheel_base_ * std::sin(alpha), lookahead_distance_); //ステアリング角を計算
  RCLCPP_INFO(this->get_logger(), "Calculated steering angle: %f", steering_angle);
  return steering_angle;
}


double TrajectoryFollowerNode::calculate_yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)

{
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  double yaw = std::atan2(siny_cosp, cosy_cosp);

  return yaw;
}

}  // namespace autoware_practice_course

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<autoware_practice_course::TrajectoryFollowerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
