#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

class GoalRelay {
 public:
  GoalRelay() : nh_(), pnh_("~") {
    std::string input_topic;
    std::string output_topic;
    pnh_.param<std::string>("input_topic", input_topic, "/airsim_node/end_goal");
    pnh_.param<std::string>("output_topic", output_topic, "/move_base_simple/goal");

    input_sub_ = nh_.subscribe(input_topic, 1, &GoalRelay::callback, this);
    output_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(output_topic, 1, true);
  }

 private:
  void callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    geometry_msgs::PoseStamped out = *msg;
    if (out.header.frame_id.empty()) {
      out.header.frame_id = "world";
    }
    out.header.stamp = ros::Time::now();
    output_pub_.publish(out);
    ROS_INFO_STREAM("Relayed goal to " << output_pub_.getTopic() << ": "
                                       << out.pose.position.x << ", "
                                       << out.pose.position.y << ", "
                                       << out.pose.position.z);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber input_sub_;
  ros::Publisher output_pub_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "goal_relay");
  GoalRelay node;
  ros::spin();
  return 0;
}
