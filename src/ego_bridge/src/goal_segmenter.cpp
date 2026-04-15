#include <cmath>
#include <limits>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

class GoalSegmenter {
 public:
  GoalSegmenter() : nh_(), pnh_("~") {
    std::string odom_topic;
    std::string input_goal_topic;
    std::string output_goal_topic;
    pnh_.param<std::string>("odom_topic", odom_topic, "/eskf_odom");
    pnh_.param<std::string>("input_goal_topic", input_goal_topic, "/move_base_simple/goal");
    pnh_.param<std::string>("output_goal_topic", output_goal_topic, "/planning/segmented_goal");
    pnh_.param("segment_length", segment_length_, 8.0);
    pnh_.param("lookahead_distance", lookahead_distance_, segment_length_);
    pnh_.param("reach_tolerance", reach_tolerance_, 1.5);
    pnh_.param("min_goal_z", min_goal_z_, 1.0);
    pnh_.param("publish_rate_hz", publish_rate_hz_, 10.0);
    pnh_.param("smoothing_alpha", smoothing_alpha_, 0.35);
    pnh_.param("final_approach_radius", final_approach_radius_, 3.0);
    pnh_.param("progress_margin", progress_margin_, 0.5);

    odom_sub_ = nh_.subscribe(odom_topic, 1, &GoalSegmenter::odomCallback, this);
    goal_sub_ = nh_.subscribe(input_goal_topic, 1, &GoalSegmenter::goalCallback, this);
    segmented_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(output_goal_topic, 1, true);
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, publish_rate_hz_)),
                             &GoalSegmenter::timerCallback, this);
  }

 private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    odom_ = *msg;
    has_odom_ = true;
  }

  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    final_goal_ = *msg;
    if (final_goal_.header.frame_id.empty()) {
      final_goal_.header.frame_id = "world";
    }
    if (final_goal_.pose.orientation.w == 0.0 && final_goal_.pose.orientation.x == 0.0 &&
        final_goal_.pose.orientation.y == 0.0 && final_goal_.pose.orientation.z == 0.0) {
      final_goal_.pose.orientation.w = 1.0;
    }
    final_goal_.pose.position.z = std::max(final_goal_.pose.position.z, min_goal_z_);
    route_start_ = has_odom_ ? odom_.pose.pose.position : final_goal_.pose.position;
    route_start_.z = std::max(route_start_.z, min_goal_z_);
    furthest_progress_ = 0.0;
    has_last_published_goal_ = false;
    has_final_goal_ = true;
    ROS_INFO_STREAM("Received final goal: " << final_goal_.pose.position.x << ", "
                                            << final_goal_.pose.position.y << ", "
                                            << final_goal_.pose.position.z);
  }

  double distanceTo(const geometry_msgs::Point& point) const {
    const auto& cur = odom_.pose.pose.position;
    const double dx = point.x - cur.x;
    const double dy = point.y - cur.y;
    const double dz = point.z - cur.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  double clamp(double value, double lower, double upper) const {
    return std::max(lower, std::min(value, upper));
  }

  double dot3(double ax, double ay, double az, double bx, double by, double bz) const {
    return ax * bx + ay * by + az * bz;
  }

  geometry_msgs::Point interpolatePoint(const geometry_msgs::Point& from,
                                        const geometry_msgs::Point& to,
                                        double ratio) const {
    geometry_msgs::Point out;
    out.x = from.x + (to.x - from.x) * ratio;
    out.y = from.y + (to.y - from.y) * ratio;
    out.z = from.z + (to.z - from.z) * ratio;
    return out;
  }

  geometry_msgs::Point computeBufferedGoal() {
    const auto& cur = odom_.pose.pose.position;
    const auto& goal = final_goal_.pose.position;

    const double line_dx = goal.x - route_start_.x;
    const double line_dy = goal.y - route_start_.y;
    const double line_dz = goal.z - route_start_.z;
    const double total_dist =
        std::sqrt(line_dx * line_dx + line_dy * line_dy + line_dz * line_dz);

    if (total_dist < 1e-6) {
      return goal;
    }

    const double dir_x = line_dx / total_dist;
    const double dir_y = line_dy / total_dist;
    const double dir_z = line_dz / total_dist;

    const double cur_dx = cur.x - route_start_.x;
    const double cur_dy = cur.y - route_start_.y;
    const double cur_dz = cur.z - route_start_.z;
    const double projected_progress =
        clamp(dot3(cur_dx, cur_dy, cur_dz, dir_x, dir_y, dir_z), 0.0, total_dist);

    furthest_progress_ = std::max(furthest_progress_, projected_progress);
    const double buffered_progress =
        clamp(furthest_progress_ + lookahead_distance_, 0.0, total_dist);

    geometry_msgs::Point target;
    if ((total_dist - furthest_progress_) <= final_approach_radius_ ||
        distanceTo(goal) <= reach_tolerance_) {
      target = goal;
    } else {
      const double ratio = buffered_progress / total_dist;
      target = interpolatePoint(route_start_, goal, ratio);
    }

    target.z = std::max(target.z, min_goal_z_);
    return target;
  }

  void publishBufferedGoal() {
    active_segment_ = final_goal_;
    active_segment_.header.stamp = ros::Time::now();

    geometry_msgs::Point target = computeBufferedGoal();
    if (has_last_published_goal_) {
      const double alpha = clamp(smoothing_alpha_, 0.0, 1.0);
      target = interpolatePoint(last_published_goal_, target, alpha);
    }

    active_segment_.pose.position = target;
    segmented_goal_pub_.publish(active_segment_);
    last_published_goal_ = target;
    has_last_published_goal_ = true;
    ROS_INFO_STREAM_THROTTLE(0.5, "Published buffered goal: " << active_segment_.pose.position.x
                                                              << ", "
                                                              << active_segment_.pose.position.y
                                                              << ", "
                                                              << active_segment_.pose.position.z);
  }

  void timerCallback(const ros::TimerEvent&) {
    if (!has_odom_ || !has_final_goal_) {
      return;
    }

    if (distanceTo(final_goal_.pose.position) <= reach_tolerance_) {
      if (!has_last_published_goal_ ||
          distanceTo(last_published_goal_) > reach_tolerance_) {
        active_segment_ = final_goal_;
        active_segment_.header.stamp = ros::Time::now();
        segmented_goal_pub_.publish(active_segment_);
        last_published_goal_ = final_goal_.pose.position;
        has_last_published_goal_ = true;
      }
      ROS_INFO_THROTTLE(1.0, "Final buffered goal reached.");
      return;
    }

    publishBufferedGoal();
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher segmented_goal_pub_;
  ros::Timer timer_;

  nav_msgs::Odometry odom_;
  geometry_msgs::PoseStamped final_goal_;
  geometry_msgs::PoseStamped active_segment_;
  geometry_msgs::Point route_start_;
  geometry_msgs::Point last_published_goal_;
  bool has_odom_ = false;
  bool has_final_goal_ = false;
  bool has_last_published_goal_ = false;

  double segment_length_ = 8.0;
  double lookahead_distance_ = 8.0;
  double reach_tolerance_ = 1.5;
  double min_goal_z_ = 1.0;
  double publish_rate_hz_ = 10.0;
  double smoothing_alpha_ = 0.35;
  double final_approach_radius_ = 3.0;
  double progress_margin_ = 0.5;
  double furthest_progress_ = 0.0;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "goal_segmenter");
  GoalSegmenter node;
  ros::spin();
  return 0;
}
