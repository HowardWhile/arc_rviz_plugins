#include "tf_trajectory_display.hpp"

#include <rviz_common/display_context.hpp>

namespace arc_rviz_plugins
{

    TFTrajectoryDisplay::TFTrajectoryDisplay() : Display()
    {
    }

    TFTrajectoryDisplay::~TFTrajectoryDisplay() = default;

    void TFTrajectoryDisplay::onInitialize()
    {
    }

    void TFTrajectoryDisplay::onEnable()
    {
    }

    void TFTrajectoryDisplay::onDisable()
    {
    }

    void TFTrajectoryDisplay::update(float dt, float ros_dt)
    {
        (void)dt;
        (void)ros_dt;
    }

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(arc_rviz_plugins::TFTrajectoryDisplay, rviz_common::Display)
