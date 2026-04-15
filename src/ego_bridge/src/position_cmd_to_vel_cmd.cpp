#include <cmath>

#include <airsim_ros/VelCmd.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <ros/duration.h>

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

class PositionCmdToVelCmd {
 public:
  PositionCmdToVelCmd() : nh_(), pnh_("~") {
    std::string odom_topic;
    std::string pos_cmd_topic;
    std::string vel_cmd_topic;
    pnh_.param<std::string>("odom_topic", odom_topic, "/eskf_odom");
    pnh_.param<std::string>("pos_cmd_topic", pos_cmd_topic, "/planning/pos_cmd");
    pnh_.param<std::string>("vel_cmd_topic", vel_cmd_topic, "/airsim_node/drone_1/vel_body_cmd");

    pnh_.param("kp_pos_xy", kp_pos_xy_, 1.0);
    pnh_.param("kp_pos_z", kp_pos_z_, 1.0);
    pnh_.param("kp_yaw", kp_yaw_, 1.5);
    pnh_.param("max_vx", max_vx_, 3.0);
    pnh_.param("max_vy", max_vy_, 3.0);
    pnh_.param("max_vz", max_vz_, 1.5);
    pnh_.param("max_yaw_rate", max_yaw_rate_, 1.0);
    pnh_.param("cmd_acc", cmd_acc_, 6);
    pnh_.param("min_safe_z", min_safe_z_, 0.8);
    pnh_.param("climb_boost_vz", climb_boost_vz_, 0.25);
    pnh_.param("allow_descend_above_z", allow_descend_above_z_, 1.2);
    pnh_.param("cruise_z", cruise_z_, 1.5);
    pnh_.param("use_fixed_altitude", use_fixed_altitude_, true);
    pnh_.param("kd_vel_xy", kd_vel_xy_, 0.4);
    pnh_.param("kd_vel_z", kd_vel_z_, 0.5);
    pnh_.param("max_delta_vxy", max_delta_vxy_, 0.08);
    pnh_.param("max_delta_vz", max_delta_vz_, 0.04);
    pnh_.param("max_delta_yaw_rate", max_delta_yaw_rate_, 0.03);
    pnh_.param("cmd_lpf_alpha", cmd_lpf_alpha_, 0.25);

    odom_sub_ = nh_.subscribe(odom_topic, 1, &PositionCmdToVelCmd::odomCallback, this);
    pos_cmd_sub_ = nh_.subscribe(pos_cmd_topic, 1, &PositionCmdToVelCmd::positionCmdCallback, this);
    vel_cmd_pub_ = nh_.advertise<airsim_ros::VelCmd>(vel_cmd_topic, 1);
  }

 private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    odom_ = *msg;
    has_odom_ = true;
  }

  void positionCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
    pos_cmd_ = *msg;

    if (!has_odom_) {
      return;
    }

    const auto& pose = odom_.pose.pose;
    const double cur_x = pose.position.x;
    const double cur_y = pose.position.y;
    const double cur_z = pose.position.z;
    const double cur_yaw = yawFromQuaternion(pose.orientation);
    const double cur_vx_world = odom_.twist.twist.linear.x;
    const double cur_vy_world = odom_.twist.twist.linear.y;
    const double cur_vz_world = odom_.twist.twist.linear.z;

    const double ex_world = pos_cmd_.position.x - cur_x;
    const double ey_world = pos_cmd_.position.y - cur_y;
    const double target_z = use_fixed_altitude_ ? cruise_z_ : pos_cmd_.position.z;
    const double ez_world = target_z - cur_z;

    const double cos_yaw = std::cos(cur_yaw);
    const double sin_yaw = std::sin(cur_yaw);

    const double evx_world = pos_cmd_.velocity.x + kp_pos_xy_ * ex_world -
                             kd_vel_xy_ * (cur_vx_world - pos_cmd_.velocity.x);
    const double evy_world = pos_cmd_.velocity.y + kp_pos_xy_ * ey_world -
                             kd_vel_xy_ * (cur_vy_world - pos_cmd_.velocity.y);
    double vx_body = cos_yaw * evx_world + sin_yaw * evy_world;
    double vy_body = -sin_yaw * evx_world + cos_yaw * evy_world;
    const double target_vz = use_fixed_altitude_ ? 0.0 : pos_cmd_.velocity.z;
    double vz_world = target_vz + kp_pos_z_ * ez_world -
                      kd_vel_z_ * (cur_vz_world - target_vz);

    const double yaw_error = wrapAngle(pos_cmd_.yaw - cur_yaw);
    double yaw_rate = pos_cmd_.yaw_dot + kp_yaw_ * yaw_error;

    if (cur_z < min_safe_z_) {
      // Keep the vehicle climbing until it clears a minimum safe altitude.
      vz_world = std::max(vz_world, climb_boost_vz_ + kp_pos_z_ * (min_safe_z_ - cur_z));
    } else if (cur_z < allow_descend_above_z_) {
      // Near the ground, suppress downward commands to avoid ground-skimming oscillation.
      vz_world = std::max(vz_world, 0.0);
    }

    vx_body = clamp(vx_body, -max_vx_, max_vx_);
    vy_body = clamp(vy_body, -max_vy_, max_vy_);
    vz_world = clamp(vz_world, -max_vz_, max_vz_);
    yaw_rate = clamp(yaw_rate, -max_yaw_rate_, max_yaw_rate_);

    if (has_last_cmd_) {
      vx_body = clamp(vx_body, last_cmd_.vx - max_delta_vxy_, last_cmd_.vx + max_delta_vxy_);
      vy_body = clamp(vy_body, last_cmd_.vy - max_delta_vxy_, last_cmd_.vy + max_delta_vxy_);
      vz_world = clamp(vz_world, last_cmd_.vz - max_delta_vz_, last_cmd_.vz + max_delta_vz_);
      yaw_rate = clamp(yaw_rate, last_cmd_.yawRate - max_delta_yaw_rate_,
                       last_cmd_.yawRate + max_delta_yaw_rate_);

      const double alpha = clamp(cmd_lpf_alpha_, 0.0, 1.0);
      vx_body = last_cmd_.vx + alpha * (vx_body - last_cmd_.vx);
      vy_body = last_cmd_.vy + alpha * (vy_body - last_cmd_.vy);
      vz_world = last_cmd_.vz + alpha * (vz_world - last_cmd_.vz);
      yaw_rate = last_cmd_.yawRate + alpha * (yaw_rate - last_cmd_.yawRate);
    }

    airsim_ros::VelCmd vel_cmd;
    vel_cmd.header.stamp = ros::Time::now();
    vel_cmd.vx = vx_body;
    vel_cmd.vy = vy_body;
    vel_cmd.vz = vz_world;
    vel_cmd.yawRate = yaw_rate;
    vel_cmd.va = static_cast<uint8_t>(std::max(0, std::min(8, cmd_acc_)));
    vel_cmd.stop = 0;
    vel_cmd_pub_.publish(vel_cmd);
    last_cmd_ = vel_cmd;
    has_last_cmd_ = true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber pos_cmd_sub_;
  ros::Publisher vel_cmd_pub_;

  nav_msgs::Odometry odom_;
  quadrotor_msgs::PositionCommand pos_cmd_;
  bool has_odom_ = false;
  airsim_ros::VelCmd last_cmd_;
  bool has_last_cmd_ = false;

  double kp_pos_xy_ = 1.0;
  double kp_pos_z_ = 1.0;
  double kp_yaw_ = 1.5;
  double kd_vel_xy_ = 0.4;
  double kd_vel_z_ = 0.5;
  double max_vx_ = 3.0;
  double max_vy_ = 3.0;
  double max_vz_ = 1.5;
  double max_yaw_rate_ = 1.0;
  double max_delta_vxy_ = 0.08;
  double max_delta_vz_ = 0.04;
  double max_delta_yaw_rate_ = 0.03;
  double cmd_lpf_alpha_ = 0.25;
  int cmd_acc_ = 6;
  double min_safe_z_ = 0.8;
  double climb_boost_vz_ = 0.25;
  double allow_descend_above_z_ = 1.2;
  double cruise_z_ = 1.5;
  bool use_fixed_altitude_ = true;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "position_cmd_to_vel_cmd");
  PositionCmdToVelCmd node;
  ros::spin();
  return 0;
}
