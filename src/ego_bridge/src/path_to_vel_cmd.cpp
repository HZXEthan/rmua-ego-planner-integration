#include <cmath>
#include <vector>

#include <ros/ros.h>
#include <airsim_ros/VelCmd.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

namespace {

double clamp(double value, double min_value, double max_value) {
  return std::max(min_value, std::min(max_value, value));
}

double wrapAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double yawFromQuaternion(const geometry_msgs::Quaternion& q) {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace

class PathToVelCmd {
 public:
  PathToVelCmd() : nh_(), pnh_("~") {
    pnh_.param("control_rate_hz", control_rate_hz_, 20.0);
    pnh_.param("waypoint_tolerance", waypoint_tolerance_, 0.5);
    pnh_.param("lookahead_points", lookahead_points_, 3);
    pnh_.param("kp_xy", kp_xy_, 0.8);
    pnh_.param("kp_z", kp_z_, 0.8);
    pnh_.param("kp_yaw", kp_yaw_, 1.5);
    pnh_.param("max_vx", max_vx_, 2.0);
    pnh_.param("max_vy", max_vy_, 2.0);
    pnh_.param("max_vz", max_vz_, 1.0);
    pnh_.param("max_yaw_rate", max_yaw_rate_, 1.0);
    pnh_.param("final_stop_tolerance", final_stop_tolerance_, 0.3);
    pnh_.param("cmd_acc", cmd_acc_, 6);

    std::string odom_topic;
    std::string path_topic;
    std::string cmd_topic;
    pnh_.param<std::string>("odom_topic", odom_topic, "/eskf_odom");
    pnh_.param<std::string>("path_topic", path_topic, "/planning/path");
    pnh_.param<std::string>("cmd_topic", cmd_topic, "/airsim_node/drone_1/vel_body_cmd");

    odom_sub_ = nh_.subscribe(odom_topic, 1, &PathToVelCmd::odomCallback, this);
    path_sub_ = nh_.subscribe(path_topic, 1, &PathToVelCmd::pathCallback, this);
    cmd_pub_ = nh_.advertise<airsim_ros::VelCmd>(cmd_topic, 1);
    target_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/planning/current_target", 1);

    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, control_rate_hz_)),
                             &PathToVelCmd::controlLoop, this);
  }

 private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    odom_ = *msg;
    has_odom_ = true;
  }

  void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    path_ = *msg;
    target_index_ = 0;
    has_path_ = !path_.poses.empty();
    ROS_INFO_STREAM("Received path with " << path_.poses.size() << " waypoints.");
  }

  void publishStop() {
    airsim_ros::VelCmd cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.vx = 0.0;
    cmd.vy = 0.0;
    cmd.vz = 0.0;
    cmd.yawRate = 0.0;
    cmd.va = static_cast<uint8_t>(std::max(0, std::min(8, cmd_acc_)));
    cmd.stop = 0;
    cmd_pub_.publish(cmd);
  }

  void controlLoop(const ros::TimerEvent&) {
    if (!has_odom_ || !has_path_ || path_.poses.empty()) {
      publishStop();
      return;
    }

    const auto& cur_pose = odom_.pose.pose;
    const double cur_x = cur_pose.position.x;
    const double cur_y = cur_pose.position.y;
    const double cur_z = cur_pose.position.z;

    while (target_index_ < path_.poses.size()) {
      const auto& wp = path_.poses[target_index_].pose.position;
      const double dx = wp.x - cur_x;
      const double dy = wp.y - cur_y;
      const double dz = wp.z - cur_z;
      const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (distance > waypoint_tolerance_) {
        break;
      }
      ++target_index_;
    }

    if (target_index_ >= path_.poses.size()) {
      const auto& last_wp = path_.poses.back().pose.position;
      const double dx = last_wp.x - cur_x;
      const double dy = last_wp.y - cur_y;
      const double dz = last_wp.z - cur_z;
      const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (distance < final_stop_tolerance_) {
        publishStop();
        return;
      }
      target_index_ = path_.poses.size() - 1;
    }

    const std::size_t tracked_index =
        std::min(target_index_ + static_cast<std::size_t>(std::max(0, lookahead_points_)),
                 path_.poses.size() - 1);
    const auto& target_pose = path_.poses[tracked_index].pose;
    const double dx_world = target_pose.position.x - cur_x;
    const double dy_world = target_pose.position.y - cur_y;
    const double dz_world = target_pose.position.z - cur_z;

    const double yaw = yawFromQuaternion(cur_pose.orientation);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    const double dx_body = cos_yaw * dx_world + sin_yaw * dy_world;
    const double dy_body = -sin_yaw * dx_world + cos_yaw * dy_world;

    const double desired_yaw = std::atan2(dy_world, dx_world);
    const double yaw_error = wrapAngle(desired_yaw - yaw);

    airsim_ros::VelCmd cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.vx = clamp(kp_xy_ * dx_body, -max_vx_, max_vx_);
    cmd.vy = clamp(kp_xy_ * dy_body, -max_vy_, max_vy_);
    cmd.vz = clamp(kp_z_ * dz_world, -max_vz_, max_vz_);
    cmd.yawRate = clamp(kp_yaw_ * yaw_error, -max_yaw_rate_, max_yaw_rate_);
    cmd.va = static_cast<uint8_t>(std::max(0, std::min(8, cmd_acc_)));
    cmd.stop = 0;
    cmd_pub_.publish(cmd);

    geometry_msgs::PoseStamped target_msg = path_.poses[tracked_index];
    target_msg.header.stamp = ros::Time::now();
    target_pub_.publish(target_msg);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber path_sub_;
  ros::Publisher cmd_pub_;
  ros::Publisher target_pub_;
  ros::Timer timer_;

  nav_msgs::Odometry odom_;
  nav_msgs::Path path_;
  bool has_odom_ = false;
  bool has_path_ = false;
  std::size_t target_index_ = 0;

  double control_rate_hz_ = 20.0;
  double waypoint_tolerance_ = 0.5;
  int lookahead_points_ = 3;
  double kp_xy_ = 0.8;
  double kp_z_ = 0.8;
  double kp_yaw_ = 1.5;
  double max_vx_ = 2.0;
  double max_vy_ = 2.0;
  double max_vz_ = 1.0;
  double max_yaw_rate_ = 1.0;
  double final_stop_tolerance_ = 0.3;
  int cmd_acc_ = 6;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_to_vel_cmd");
  PathToVelCmd node;
  ros::spin();
  return 0;
}
