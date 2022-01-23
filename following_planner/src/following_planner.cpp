
#include <angles/angles.h>
#include <following_planner/following_planner.h>
#include <geometry_msgs/Quaternion.h>
// #include <nav_core/parameter_magic.h>
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

PLUGINLIB_EXPORT_CLASS(following_planner::FollowingPlannerROS, nav_core::BaseLocalPlanner)

namespace following_planner
{
	enum GoalStatus
	{
		MOVING = 0,
		ROTATING = 1,
		GOALREACHED = 2
	};

	FollowingPlannerROS::FollowingPlannerROS()
	{
	}
#if (ROS_VERSION_MINOR == 14)
	void FollowingPlannerROS::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
#else
	void FollowingPlannerROS::initialize(std::string name, tf::TransformListener *tf, costmap_2d::Costmap2DROS *costmap_ros)
#endif
	{
		std::cout << ROS_VERSION_MAJOR << "." << ROS_VERSION_MINOR << std::endl;
#if (ROS_VERSION_MINOR == 14)
		tf_ = tf;
#else
		tf_listener_ = new tf2_ros::TransformListener(tf_);
#endif

		look_ahead_pub_ = nh_.advertise<geometry_msgs::PointStamped>("look_ahead", 1, true);

		status_ = GoalStatus::MOVING;
		getRosparam();
		robot_frame_ = "base_footprint";
		map_frame_ = "map";
	}

	void FollowingPlannerROS::getRosparam()
	{
		// nh_.getParam("look_ahead_distance", look_ahead_, 0.5);
		nh_.param<double>("look_ahead_distance", look_ahead_distance_, 0.1);
		nh_.param<double>("xy_goal_tolerance", xy_goal_tolerance_, 0.05);
		nh_.param<double>("yaw_goal_tolerance", yaw_goal_tolerance_, 0.1);
		nh_.param<double>("vx", vx_, 0.22);
		nh_.param<double>("vx_slow", vx_slow_, 0.05);
		nh_.param<double>("vw_max", vw_max_, 2.75);
		nh_.param<double>("vw_", vw_, 1.57);
		nh_.param<double>("slow_range", slow_range_, 0.1);
		nh_.param<double>("rotate_angle_th", rotate_angle_th_, 30);
		nh_.param<bool>("latch_xy_goal_tolerance", latch_xy_goal_tolerance_, true);
	}

	FollowingPlannerROS::~FollowingPlannerROS()
	{
	}

	bool FollowingPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
	{
		global_plan_ = orig_global_plan;
		return true;
	}

	bool FollowingPlannerROS::isGoalReached()
	{
		geometry_msgs::PoseStamped robot_position_ = getRobotPosition(robot_frame_, map_frame_);
		geometry_msgs::PoseStamped goal = global_plan_[global_plan_.size() - 1];
		//   std::cout << "get goal" << std::endl;
		double xy, yaw;
		checkPoseDiff(robot_position_, goal, xy, yaw);

		bool yaw_reached = std::abs(yaw) < yaw_goal_tolerance_;
		bool xy_reached;
		if(latch_xy_goal_tolerance_ && status_ == GoalStatus::ROTATING)
		{
			xy_reached = true;
		}
		else
		{
			xy_reached = std::abs(xy) < xy_goal_tolerance_;
		}
		
		if (xy_reached && yaw_reached)
		{
			status_ = GoalStatus::GOALREACHED;
			ROS_INFO_STREAM("reached goal !!");
			ROS_INFO_STREAM("distance: " << xy);
			ROS_INFO_STREAM("yaw diff:" << yaw);
			return true;
		}
		else if (xy_reached && !yaw_reached) //|| status_ == GoalStatus::ROTATING)
		{
			status_ = GoalStatus::ROTATING;
		}
		else
		{
			status_ = GoalStatus::MOVING;
		}
		return false;
	}

	void FollowingPlannerROS::checkPoseDiff(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2,
											double &xy_diff, double &yaw_diff)
	{
		//   std::cout << "checkPoseDiff" << std::endl;
		double dx = pose2.pose.position.x - pose1.pose.position.x;
		double dy = pose2.pose.position.y - pose1.pose.position.y;
		xy_diff = std::sqrt(dx * dx + dy * dy);

		double roll, pitch, yaw1, yaw2;

		geometry_msgs::Quaternion q = pose1.pose.orientation;
		GetRPY(pose1.pose, roll, pitch, yaw1);
		GetRPY(pose2.pose, roll, pitch, yaw2);
		yaw_diff = angles::normalize_angle(yaw2 - yaw1);
		return;
	}

	geometry_msgs::PoseStamped FollowingPlannerROS::getRobotPosition(std::string src_frame, std::string target_frame)
	{
		//   std::cout << "getRobotPosition" << std::endl;
		geometry_msgs::TransformStamped tf_pose;
		try
		{
			// std::cout << "get tf" << std::endl;
#if (ROS_VERSION_MINOR == 14)
			tf_pose = tf_->lookupTransform(target_frame, src_frame, ros::Time(0));
#else
			tf_pose = tf_.lookupTransform(target_frame, src_frame, ros::Time(0));
#endif
		}
		catch (tf2::TransformException &ex)
		{
			ROS_WARN("%s", ex.what());
			ros::Duration(1.0).sleep();
			return robot_position_;
		}
		//   std::cout << tf_pose << std::endl;
		geometry_msgs::PoseStamped position;
		position.header = tf_pose.header;
		position.pose.position.x = tf_pose.transform.translation.x;
		position.pose.position.y = tf_pose.transform.translation.y;
		position.pose.position.z = tf_pose.transform.translation.z;
		position.pose.orientation = tf_pose.transform.rotation;
		return position;
	}

	bool FollowingPlannerROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
	{
		//   std::cout << "computeVelocityCommands" << std::endl;
		if (status_ == GoalStatus::MOVING)
		{
			geometry_msgs::PoseStamped self_position = getRobotPosition(robot_frame_, map_frame_);
			geometry_msgs::PoseStamped look_ahead = computeLookAheadPoint(self_position);

			cmd_vel = computePurePursuit(look_ahead.pose.position, self_position, vx_);
			// std::cout << "computeVelocityCommands end" << std::endl;

			geometry_msgs::PointStamped look_ahead_msg;
			look_ahead_msg.header = look_ahead.header;
			look_ahead_msg.point = look_ahead.pose.position;
			look_ahead_pub_.publish(look_ahead);
			return true;
		}
		else if (status_ == GoalStatus::ROTATING)
		{
			cmd_vel.linear.x = 0.0;
			geometry_msgs::PoseStamped self_position = getRobotPosition(robot_frame_, map_frame_);
			cmd_vel.angular.z = setAttitude(self_position, global_plan_[global_plan_.size() - 1]);
			return true;
		}
		else if (status_ == GoalStatus::GOALREACHED)
		{
			cmd_vel.linear.x = 0.0;
			cmd_vel.angular.z = 0.0;
			return true;
		}
		else
		{
			ROS_WARN("unknown status !!");
			cmd_vel.linear.x = 0.0;
			cmd_vel.angular.z = 0.0;
			return true;
		}
	}

	geometry_msgs::PoseStamped FollowingPlannerROS::computeLookAheadPoint(geometry_msgs::PoseStamped self_position)
	{
		double min_distance = DBL_MAX;
		int min_id = 0;
		for (int i = 0; i < global_plan_.size(); i++)
		{
			double dx = global_plan_[i].pose.position.x - self_position.pose.position.x;
			double dy = global_plan_[i].pose.position.y - self_position.pose.position.y;
			double distance = std::sqrt(dx * dx + dy * dy);
			if (min_distance > distance)
			{
				min_distance = distance;
				min_id = i;
			}
		}

		double distance = 0;
		geometry_msgs::PoseStamped look_ahead;
		for (int i = min_id + 1; i < global_plan_.size(); i++)
		{
			double dx = global_plan_[i].pose.position.x - global_plan_[i - 1].pose.position.x;
			double dy = global_plan_[i].pose.position.y - global_plan_[i - 1].pose.position.y;
			distance += std::sqrt(dx * dx + dy * dy);

			if (distance > look_ahead_distance_)
			{
				look_ahead = global_plan_[i];
				return look_ahead;
			}
		}
		look_ahead = *global_plan_.end();
		return look_ahead;
	}

	geometry_msgs::Twist FollowingPlannerROS::computePurePursuit(geometry_msgs::Point look_ahead,
																 geometry_msgs::PoseStamped self_position, double vx)
	{
		double goal_distance, goal_direction_diff;
		checkPoseDiff(self_position, global_plan_[global_plan_.size() - 1], goal_distance, goal_direction_diff);
		if (goal_distance < slow_range_)
			vx = vx_slow_;
		double dx = look_ahead.x - self_position.pose.position.x;
		double dy = look_ahead.y - self_position.pose.position.y;
		double distance = std::sqrt(dx * dx + dy * dy);
		double to_look_ahead = angles::normalize_angle(std::atan2(dy, dx));

		double roll, pitch, yaw;
		GetRPY(self_position.pose, roll, pitch, yaw);
		double direction_diff = angles::normalize_angle(to_look_ahead - yaw);

		std::cout << direction_diff << std::endl;
		geometry_msgs::Twist cmd_vel;
		double direction = 1;
		if (std::abs(direction_diff) < rotate_angle_th_ * M_PI / 180)
		{
			ROS_INFO("move forward");
			direction = 1;
		}
		else if (std::abs(direction_diff) > (180 - rotate_angle_th_) * M_PI / 180)
		{
			ROS_INFO("move backwaord");
			direction = -1;
			direction_diff = angles::normalize_angle(direction_diff - M_PI);
		}
		else
		{
			ROS_INFO("spin turn");
			geometry_msgs::PoseStamped target;
			target.header.frame_id = map_frame_;
			target.pose.position = look_ahead;
			GetQuaternionMsg(0.0, 0.0, to_look_ahead, target.pose.orientation);
			std::cout << direction_diff * 180 / M_PI << std::endl;
			if (std::abs(direction_diff) >= 90 * M_PI / 180)
			{
				std::cout << "adjust backward" << std::endl;
				geometry_msgs::PoseStamped self_position_dummy = self_position;
				double roll, pitch, yaw;
				GetRPY(self_position.pose, roll, pitch, yaw);
				yaw = angles::normalize_angle(yaw - M_PI);
				GetQuaternionMsg(0.0, 0.0, yaw, self_position_dummy.pose.orientation);
				cmd_vel.angular.z = setAttitude(self_position_dummy, target);
				// std::cout << cmd_vel.angular.z << "[rad/s]" << std::endl;
			}
			else
			{
				cmd_vel.angular.z = setAttitude(self_position, target);
			}
			cmd_vel.linear.x = 0.0;
			return cmd_vel;
		}

		double sin_val = std::sin(direction_diff);

		if (std::abs(distance) < DBL_EPSILON)
		{
			ROS_WARN_STREAM("zero division. look ahead is too near so stop.");
			cmd_vel.linear.x = 0.0;
			cmd_vel.angular.z = 0.0;
			return cmd_vel;
		}

		if (std::abs(sin_val) < DBL_EPSILON)
		{
			ROS_WARN_STREAM("yaw diff is too small");
			cmd_vel.linear.x = vx;
			cmd_vel.angular.z = 0;
			return cmd_vel;
		}

		double omega = 2.0 * vx * sin_val / distance;
		if (std::abs(omega) > vw_max_)
		{
			ROS_INFO_STREAM("omega > " << vw_max_);
			omega = ((omega > 0) - (omega < 0)) * vw_max_;
			vx = omega * distance / sin_val;
		}

		cmd_vel.linear.x = direction * vx;
		cmd_vel.angular.z = omega;

		//   std::cout << cmd_vel << std::endl;
		return cmd_vel;
	}

	double FollowingPlannerROS::setAttitude(geometry_msgs::PoseStamped self_position, geometry_msgs::PoseStamped target)
	{
		double roll, pitch, yaw1, yaw2;
		GetRPY(self_position.pose, roll, pitch, yaw1);
		GetRPY(target.pose, roll, pitch, yaw2);
		//   std::cout << self_position.header.frame_id << ", " << global_plan_.end()->header.frame_id << std::endl;
		double angle_diff = angles::normalize_angle(yaw1 - yaw2);
		double vw = vw_;
		if (std::abs(angle_diff) < 30 * M_PI / 180)
		{
			vw = std::abs(angle_diff) * 2.0;
		}
		return -1 * ((angle_diff > 0) - (angle_diff < 0)) * vw;
	}

	void FollowingPlannerROS::GetQuaternionMsg(double roll, double pitch, double yaw, geometry_msgs::Quaternion &q)
	{
		tf::Quaternion quat = tf::createQuaternionFromRPY(roll, pitch, yaw);
		quaternionTFToMsg(quat, q);
	}

	void FollowingPlannerROS::GetRPY(const geometry_msgs::Pose p, double &roll, double &pitch, double &yaw)
	{
		tf::Quaternion quat(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
	}
}; // namespace following_planner
