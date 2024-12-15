/**
 * @file: pid_local_planner.h
 * @brief: Contains the PID local planner
 * @author: kadiandeapa
 * @date: 2024-11-24
 * @version: 1.0
 */

#ifndef __PID_LOCAL_PLANNER_H__
#define __PID_LOCAL_PLANNER_H__

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include "pid_local_planner/controller.h"

namespace pid_local_planner
{
    class PidLocalPlanner : public nav_core::BaseLocalPlanner, controller::Controller
    {
    public:
        PidLocalPlanner();

        PidLocalPlanner(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros);

        /**
         * 以下4个函数是基类定义的接口函数（纯虚函数）
         */
        bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);
        bool isGoalReached();
        bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan);
        void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros);

        ~PidLocalPlanner();

    private:
        bool initialized_;                      // 初始化标志符
        bool goal_reached_;                     // 到达目标标志符
        tf2_ros::Buffer *tf_;                   // tf缓存
        costmap_2d::Costmap2DROS *costmap_ros_; // costmap

        // pid controller params
        double k_v_p_, k_v_i_, k_v_d_;
        double k_w_p_, k_w_i_, k_w_d_;
        double k_theta_;
        double k_, l_;
    };
}; // namespace pid_local_planner

#endif