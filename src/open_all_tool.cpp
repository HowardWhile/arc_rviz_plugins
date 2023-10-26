#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/display_context.hpp>
// #include <rviz_common/view_manager.hpp>
// #include <rviz_common/display.hpp>

#include "open_all_tool.hpp"

namespace arc_rviz_plugins
{

    OpenAllTool::OpenAllTool() : rviz_common::Tool()
    {
    }

    OpenAllTool::~OpenAllTool()
    {
    }

    void OpenAllTool::onInitialize()
    {
    }

    void OpenAllTool::openProperty(rviz_common::properties::Property *property)
    {
        property->expand();
        if (property->numChildren() > 0)
        {
            for (size_t i = 0; i < (size_t)property->numChildren(); i++)
            {
                this->openProperty(property->childAt(i));
            }
            this->context_->queueRender();
        }
    }

    void OpenAllTool::activate()
    {
        rviz_common::DisplayGroup *display_group = this->context_->getRootDisplayGroup();
        openProperty(display_group);
    }

    void OpenAllTool::deactivate()
    {
    }

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(arc_rviz_plugins::OpenAllTool, rviz_common::Tool)