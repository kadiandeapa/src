/**
 * *********************************************************
 *
 * @file: pid_controller.cpp
 * @brief: Contains the Proportional–Integral–Derivative (PID) controller local controller class
 * @author: Yang Haodong, Guo Zhanyu
 * @date: 2024-01-20
 * @version: 1.2
 *
 * Copyright (c) 2024, Yang Haodong, Guo Zhanyu.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <pluginlib/class_list_macros.h>

#include "controller/pid_controller.h"

/**
 * 1) PLUGINLIB_EXPORT_CLASS：这是 pluginlib 库提供的一个宏，用于将某个类注册为插件。
 * 注册后，这个类可以在运行时动态加载，而无需在编译时直接链接
 * 
 * 2) pid_controller::PIDController：这是要导出的具体类的完整名称（包括命名空间）。
 * 在这个例子中，PIDController 是类名，位于 pid_controller 命名空间下。
 * 
 * 3) nav_core::BaseLocalPlanner：这是 PIDController 类实现的基类或接口。
 * 在这个例子中，BaseLocalPlanner 是 ROS 中 nav_core 包的一部分，定义了局部路径规划器的通用接口。
 * 通过将 PIDController 注册为 BaseLocalPlanner 的插件，ROS 系统就可以在运行时通过 
 * BaseLocalPlanner 接口动态加载并使用 PIDController。
 */

PLUGINLIB_EXPORT_CLASS(pid_controller::PIDController, nav_core::BaseLocalPlanner)

namespace pid_controller
{
/**
 * @brief Construct a new PIDController object
 */
PIDController::PIDController() : initialized_(false), goal_reached_(false), tf_(nullptr)  //, costmap_ros_(nullptr)
{
}

/**
 * @brief Construct a new PIDController object
 */
PIDController::PIDController(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
  : PIDController()
{
  initialize(name, tf, costmap_ros);
}

/**
 * @brief Destroy the PIDController object
 */
PIDController::~PIDController()
{
}

/**
 * @brief Initialization of the local planner
 * @param name        the name to give this instance of the trajectory planner
 * @param tf          a pointer to a transform listener
 * @param costmap_ros the cost map to use for assigning costs to trajectories
 */
void PIDController::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!initialized_)
  {
    initialized_ = true;
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    ros::NodeHandle nh = ros::NodeHandle("~/" + name);
    
    // base
    nh.param("goal_dist_tolerance", goal_dist_tol_, 0.2);
    nh.param("rotate_tolerance", rotate_tol_, 0.5);
    nh.param("base_frame", base_frame_, base_frame_);
    nh.param("map_frame", map_frame_, map_frame_);

    // lookahead
    nh.param("lookahead_time", lookahead_time_, 0.5);
    nh.param("min_lookahead_dist", min_lookahead_dist_, 0.3);
    nh.param("max_lookahead_dist", max_lookahead_dist_, 0.9);

    // linear velocity
    nh.param("max_v", max_v_, 0.5);
    nh.param("min_v", min_v_, 0.0);
    nh.param("max_v_inc", max_v_inc_, 0.5);

    // angular velocity
    nh.param("max_w", max_w_, 1.57);
    nh.param("min_w", min_w_, 0.0);
    nh.param("max_w_inc", max_w_inc_, 1.57);

    // PID parameters
    nh.param("k_v_p", k_v_p_, 1.00);
    nh.param("k_v_i", k_v_i_, 0.01);
    nh.param("k_v_d", k_v_d_, 0.10);
    nh.param("k_w_p", k_w_p_, 1.00);
    nh.param("k_w_i", k_w_i_, 0.01);
    nh.param("k_w_d", k_w_d_, 0.10);
    nh.param("k_theta", k_theta_, 0.5);
    nh.param("k", k_, 1.0);
    nh.param("l", l_, 0.2);

    e_v_ = i_v_ = 0.0;
    e_w_ = i_w_ = 0.0;

    double controller_freqency;
    nh.param("/move_base/controller_frequency", controller_freqency, 10.0);
    d_t_ = 1 / controller_freqency;

    target_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/target_pose", 10);
    current_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);

    ROS_INFO("PID Controller initialized!");
  }
  else
  {
    ROS_WARN("PID Controller has already been initialized.");
  }
}

/**
 * @brief Set the plan that the controller is following
 * @param orig_global_plan the plan to pass to the controller
 * @return true if the plan was updated successfully, else false
 */
bool PIDController::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  ROS_INFO("Got new plan");

  // set new plan
  global_plan_.clear();
  global_plan_ = orig_global_plan;

  // receive a plan for a new goal
  if (goal_x_ != global_plan_.back().pose.position.x || goal_y_ != global_plan_.back().pose.position.y)
  {
    goal_x_ = global_plan_.back().pose.position.x;
    goal_y_ = global_plan_.back().pose.position.y;
    goal_theta_ = getYawAngle(global_plan_.back());
    goal_reached_ = false;

    e_v_ = i_v_ = 0.0;
    e_w_ = i_w_ = 0.0;
  }

  return true;
}

/**
 * @brief Check if the goal pose has been achieved
 * @return true if achieved, false otherwise
 */
bool PIDController::isGoalReached()
{
  if (!initialized_)
  {
    ROS_ERROR("PID Controller has not been initialized");
    return false;
  }

  if (goal_reached_)
  {
    ROS_INFO("GOAL Reached!");
    return true;
  }
  return false;
}

/**
 * @brief Given the current position, orientation, and velocity of the robot, compute the velocity commands
 * @param cmd_vel will be filled with the velocity command to be passed to the robot base
 * @return true if a valid trajectory was found, else false
 */
bool PIDController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!initialized_)
  {
    ROS_ERROR("PID Controller has not been initialized");
    return false;
  }

  // 设置一个时间容差，例如 0.1 秒
  ros::Duration tf_tolerance(0.1);

  // odometry observation - 获取里程计信息
  nav_msgs::Odometry base_odom;
  odom_helper_->getOdom(base_odom);

  // 获取机器人在 odom 中的位置
  geometry_msgs::PoseStamped current_ps_odom, current_ps_map, target_ps_map;
  if (!costmap_ros_->getRobotPose(current_ps_odom))
  {
    ROS_WARN("Failed to get robot pose in odom frame");
    return false;
  }
  // transformPose(tf_, map_frame_, current_ps_odom, current_ps_map);

  // 添加时间容差来转换 odom 坐标到 map 坐标
  try
  {
    tf_->transform(current_ps_odom, current_ps_map, map_frame_, tf_tolerance);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("Transform failed: %s", ex.what());
    return false;
  }

  // 剪枝全局路径
  std::vector<geometry_msgs::PoseStamped> prune_plan = prune(current_ps_map);

  // 计算前瞻距离
  double vt = std::hypot(base_odom.twist.twist.linear.x, base_odom.twist.twist.linear.y);
  double wt = base_odom.twist.twist.angular.z;
  double L = getLookAheadDistance(vt);

  // 获取前瞻点并计算目标方向
  geometry_msgs::PointStamped lookahead_pt;
  double theta_d, theta_dir, theta_trj, kappa;
  getLookAheadPoint(L, current_ps_map, prune_plan, lookahead_pt, theta_trj, kappa);
  target_ps_map.pose.position.x = lookahead_pt.point.x;
  target_ps_map.pose.position.y = lookahead_pt.point.y;
  theta_dir = atan2((target_ps_map.pose.position.y - current_ps_map.pose.position.y),
                    (target_ps_map.pose.position.x - current_ps_map.pose.position.x));
  theta_d = regularizeAngle((1 - k_theta_) * theta_trj + k_theta_ * theta_dir);
  tf2::Quaternion q;
  q.setRPY(0, 0, theta_d);
  tf2::convert(q, target_ps_map.pose.orientation);

  // 当前角度
  double theta = tf2::getYaw(current_ps_map.pose.orientation);  // [-pi, pi]

  // 位置是否达到目标
  if (shouldRotateToGoal(current_ps_map, global_plan_.back()))
  {
    double e_theta = regularizeAngle(goal_theta_ - theta);

    // 目标角度已达到
    if (!shouldRotateToPath(std::fabs(e_theta)))
    {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      goal_reached_ = true;
    }
    // 目标角度未达到
    else
    {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = angularRegularization(base_odom, e_theta / d_t_);
    }
  }
  // 位置未达到目标
  else
  {
    Eigen::Vector3d s(current_ps_map.pose.position.x, current_ps_map.pose.position.y, theta);    // current state
    Eigen::Vector3d s_d(target_ps_map.pose.position.x, target_ps_map.pose.position.y, theta_d);  // desired state
    Eigen::Vector2d u_r(vt, wt);                                                                 // refered input
    Eigen::Vector2d u = _pidControl(s, s_d, u_r);

    cmd_vel.linear.x = linearRegularization(base_odom, u[0]);
    cmd_vel.angular.z = angularRegularization(base_odom, u[1]);
  }

  // 发布下一步目标位姿
  target_ps_map.header.frame_id = "map";
  target_ps_map.header.stamp = ros::Time::now();
  target_pose_pub_.publish(target_ps_map);

  // 发布机器人当前位置
  current_ps_map.header.frame_id = "map";
  current_ps_map.header.stamp = ros::Time::now();
  current_pose_pub_.publish(current_ps_map);

  return true;
}

/**
 * @brief Execute PID control process (no model pid)
 * @param s   current state
 * @param s_d desired state
 * @param u_r refered input
 * @return u  control vector
 */
// Eigen::Vector2d PIDController::_pidControl(Eigen::Vector3d s, Eigen::Vector3d s_d, Eigen::Vector2d u_r)
// {
//   Eigen::Vector2d u;
//   Eigen::Vector3d e = s_d - s;

//   double e_x = e[0];
//   double e_y = e[1];
//   double e_theta = e[2];

//   double v_d = std::hypot(e_x, e_y) / d_t_;
//   double w_d = e_theta / d_t_;

//   if (std::fabs(v_d) > max_v_)
//     v_d = std::copysign(max_v_, v_d);
//   if (std::fabs(w_d) > max_w_)
//     w_d = std::copysign(max_w_, w_d);

//   double e_v = v_d - u_r[0];
//   double e_w = w_d - u_r[1];

//   i_v_ += e_v * d_t_;
//   i_w_ += e_w * d_t_;

//   double d_v = (e_v - e_v_) / d_t_;
//   double d_w = (e_w - e_w_) / d_t_;

//   e_v_ = e_v;
//   e_w_ = e_w;

//   double v_inc = k_v_p_ * e_v + k_v_i_ * i_v_ + k_v_d_ * d_v;
//   double w_inc = k_w_p_ * e_w + k_w_i_ * i_w_ + k_w_d_ * d_w;

//   if (std::fabs(v_inc) > max_v_inc_)
//     v_inc = std::copysign(max_v_inc_, v_inc);
//   if (std::fabs(w_inc) > max_w_inc_)
//     w_inc = std::copysign(max_w_inc_, w_inc);

//   double v_cmd = u_r[0] + v_inc;
//   if (std::fabs(v_cmd) > max_v_)
//     v_cmd = std::copysign(max_v_, v_cmd);
//   else if (std::fabs(v_cmd) < min_v_)
//     v_cmd = std::copysign(min_v_, v_cmd);

//   double w_cmd = u_r[1] + w_inc;
//   if (std::fabs(w_cmd) > max_w_)
//     w_cmd = std::copysign(max_w_, w_cmd);
//   else if (std::fabs(w_cmd) < min_w_)
//     w_cmd = std::copysign(min_w_, w_cmd);

//   u[0] = v_cmd;
//   u[1] = w_cmd;

//   return u;
// }

/**
 * @brief Execute PID control process (with model)
 * @param s   current state
 * @param s_d desired state
 * @param u_r refered input
 * @return u  control vector
 */
Eigen::Vector2d PIDController::_pidControl(Eigen::Vector3d s, Eigen::Vector3d s_d, Eigen::Vector2d u_r)
{
  Eigen::Vector2d u;
  Eigen::Vector3d e(s_d - s);
  Eigen::Vector2d sx_dot(k_ * e[0], k_ * e[1]);
  Eigen::Matrix2d R_inv;
  R_inv << cos(s[2]), sin(s[2]), -sin(s[2]) / l_, cos(s[2]) / l_;
  u = R_inv * sx_dot;

  return u;
}
}  // namespace pid_controller
