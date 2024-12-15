#include "pid_local_planner/pid_local_planner.h"
#include <pluginlib/class_list_macros.h>

/**
 * PLUGINLIB_EXPORT_CLASS: 属于 ROS（Robot Operating System）中的 pluginlib 库，用于将自定义插件类暴露给 ROS 系统。
 * 它的作用是将 pid_local_planner::PidLocalPlanner 类注册为一个可用的插件，
 * 并指定该插件继承自 nav_core::BaseLocalPlanner 接口类。
 */
PLUGINLIB_EXPORT_CLASS(pid_local_planner::PidLocalPlanner, nav_core::BaseLocalPlanner)

namespace pid_local_planner
{
    PidLocalPlanner::PidLocalPlanner() : initialized_(false), goal_reached_(false), tf_(nullptr), costmap_ros_(nullptr)
    {
    }

    PidLocalPlanner::PidLocalPlanner(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros) : PidLocalPlanner()
    {
        initialize(name, tf, costmap_ros);
    }

    /**
     * @brief 初始化 PID 控制器
     * @param name 控制器的名称
     * @param tf 用于坐标变换的 tf2_ros::Buffer 指针
     * @param costmap_ros 用于代价地图的 costmap_2d::Costmap2DROS 指针
     */
    void PidLocalPlanner::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!initialized_)
        {
            initialized_ = true;
            tf_ = tf;                   // 存储 tf 指针
            costmap_ros_ = costmap_ros; // 存储 costmap_ros 指针

            // 创建一个 ROS 节点句柄，用于读取参数
            ros::NodeHandle nh = ros::NodeHandle("~/" + name);
            // 从参数服务器读取 PID 控制器的参数
            nh.param("goal_dist_tolerance", goal_dist_tol_, 0.2); // 目标距离容差
            nh.param("rotate_tolerance", rotate_tol_, 0.5);       // 旋转容差
            nh.param("base_frame", base_frame_, base_frame_);     // 机器人基座坐标系
            nh.param("map_frame", map_frame_, map_frame_);        // 地图坐标系

            // 读取前瞻参数
            nh.param("lookahead_time", lookahead_time_, 0.5);         // 前瞻时间
            nh.param("min_lookahead_dist", min_lookahead_dist_, 0.3); // 最小前瞻距离
            nh.param("max_lookahead_dist", max_lookahead_dist_, 0.9); // 最大前瞻距离

            // 读取线速度参数
            nh.param("max_v", max_v_, 0.5);         // 最大线速度
            nh.param("min_v", min_v_, 0.0);         // 最小线速度
            nh.param("max_v_inc", max_v_inc_, 0.5); // 最大线速度增量

            // 读取角速度参数
            nh.param("max_w", max_w_, 1.57);         // 最大角速度
            nh.param("min_w", min_w_, 0.0);          // 最小角速度
            nh.param("max_w_inc", max_w_inc_, 1.57); // 最大角速度增量

            // 读取 PID 参数
            nh.param("k_v_p", k_v_p_, 1.00);    // 线速度比例系数
            nh.param("k_v_i", k_v_i_, 0.01);    // 线速度积分系数
            nh.param("k_v_d", k_v_d_, 0.10);    // 线速度微分系数
            nh.param("k_w_p", k_w_p_, 1.00);    // 角速度比例系数
            nh.param("k_w_i", k_w_i_, 0.01);    // 角速度积分系数
            nh.param("k_w_d", k_w_d_, 0.10);    // 角速度微分系数
            nh.param("k_theta", k_theta_, 0.5); // 角度系数
            nh.param("k", k_, 1.0);             // 系数 k
            nh.param("l", l_, 0.2);             // 系数 l
        }
    }
} // namespace pid_local_planner
