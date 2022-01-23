#include <vector>

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <nav_core/base_local_planner.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace following_planner
{
  class FollowingPlannerROS : public nav_core::BaseLocalPlanner
  {
  public:
    FollowingPlannerROS();
#if (ROS_VERSION_MINOR == 14)
    void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros);
#else
    void initialize(std::string name, tf::TransformListener *tf, costmap_2d::Costmap2DROS *costmap_ros);
#endif
    ~FollowingPlannerROS();
    bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);
    bool setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);
    bool isGoalReached();
    bool isInitialized()
    {
      return initialized_;
    }

  private:
    void getRosparam();
    geometry_msgs::PoseStamped computeLookAheadPoint(geometry_msgs::PoseStamped self_position);
    geometry_msgs::PoseStamped getRobotPosition(std::string src_frame, std::string target_frame);
    void checkPoseDiff(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2, double &xy, double &y);
    void GetQuaternionMsg(double roll, double pitch, double yaw, geometry_msgs::Quaternion &q);
    void GetRPY(geometry_msgs::Pose p, double &roll, double &pitch, double &yaw);
    geometry_msgs::Twist computePurePursuit(geometry_msgs::Point look_ahead, geometry_msgs::PoseStamped self_position,
                                            double vx);
    double setAttitude(geometry_msgs::PoseStamped self_position, geometry_msgs::PoseStamped target);

    // member
    ros::NodeHandle nh_;
    ros::Publisher look_ahead_pub_;
#if (ROS_VERSION_MINOR == 14)
    tf2_ros::Buffer* tf_;
#else
    tf2_ros::Buffer tf_;
    tf2_ros::TransformListener *tf_listener_;
#endif
    costmap_2d::Costmap2DROS *costmap_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    geometry_msgs::PoseStamped robot_position_;

    double xy_goal_tolerance_, yaw_goal_tolerance_;
    double look_ahead_distance_;
    double vx_, vx_slow_, vw_max_, vw_;
    double slow_range_;
    double rotate_angle_th_;

    int status_;

    bool initialized_;
    bool latch_xy_goal_tolerance_;

    std::string robot_frame_, map_frame_;
  };

} // namespace following_planner