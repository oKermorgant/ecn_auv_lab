#include "waypoints.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace geometry_msgs::msg;
using namespace std::chrono_literals;

class Control : public rclcpp::Node
{
public:
  Control() : Node("waypoints")
  {
    set_parameter(rclcpp::Parameter("use_sim_time", true));

    // load waypoints and thresholds
    Waypoint::load(waypoints, position_thr, orientation_thr);

    pose_cmd.header.frame_id = "world";
    pub = create_publisher<PoseStamped>("/bluerov2/cmd_pose", 1);

    static auto timer{create_wall_timer(10ms, [&]()
      {
        if(!buffer.canTransform("world", "bluerov2/base_link", tf2::TimePointZero))
          return;
        const auto transform{buffer.lookupTransform("world", "bluerov2/base_link", tf2::TimePointZero)};
        trackWaypoint(transform.transform);
      })};
  }

private:

  std::vector<Waypoint> waypoints;
  double position_thr, orientation_thr;

  tf2_ros::Buffer buffer{get_clock()};
  tf2_ros::TransformListener tl{buffer};

  PoseStamped pose_cmd;
  rclcpp::Publisher<PoseStamped>::SharedPtr pub;


  void trackWaypoint(const Transform &pose)
  {
    static auto idx{0};
    const auto &cur_wp{waypoints[idx]};

    // TODO update 'idx' to cycle through the waypoints when the current one 'cur_wp' is reached




    // write new waypoint as msg and publish as reference pose
    waypoints[idx].write(pose_cmd);
    pose_cmd.header.stamp = get_clock()->now();
    pub->publish(pose_cmd);
  }
};



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Control>());
  rclcpp::shutdown();
  return 0;
}
