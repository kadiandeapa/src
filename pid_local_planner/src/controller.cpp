#include "pid_local_planner/controller.h"

namespace controller
{
    Controller::Controller() : base_frame_("base_link"), map_frame_("map"), odom_frame_("odom")
    {

    }
} // namespace controller
