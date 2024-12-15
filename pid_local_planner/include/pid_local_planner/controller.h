#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

namespace controller
{
    class Controller
    {
    private:
        /* data */
    public:
        Controller(/* args */);
        ~Controller();

    protected:
        // frame name of base link, map and odometry
        std::string base_frame_, map_frame_, odom_frame_;

        double goal_dist_tol_, rotate_tol_;

        double lookahead_time_;     // lookahead time gain
        double min_lookahead_dist_; // minimum lookahead distance
        double max_lookahead_dist_; // maximum lookahead distance
    
        double max_v_, min_v_, max_v_inc_;  // linear velocity
        double max_w_, min_w_, max_w_inc_;  // angular velocity
    };

} // namespace controller

#endif
