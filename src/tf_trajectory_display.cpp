#include "tf_trajectory_display.hpp"

#include <boost/format.hpp>
#include <iomanip>

// --------------------------------------------------------
// console log macro
// --------------------------------------------------------
#define NODE_HANDLER rviz_node_
#define CONSOLE_TAG "TFTrajectoryDisplay"

#define console(format, ...) \
    RCLCPP_INFO(NODE_HANDLER->get_logger(), "[" CONSOLE_TAG "] " format, ##__VA_ARGS__)
#define console_preiod(period, format, ...) \
    RCLCPP_INFO_THROTTLE(NODE_HANDLER->get_logger(), *NODE_HANDLER->get_clock(), period, "[" CONSOLE_TAG "] " format, ##__VA_ARGS__)
// --------------------------------------------------------

namespace arc_rviz_plugins
{
    TFTrajectoryDisplay::TFTrajectoryDisplay() : Display(), clock_(RCL_SYSTEM_TIME)
    {
        frame_property_ = new rviz_common::properties::TfFrameProperty(
            "frame", "", "frame to visualize trajectory", this, NULL, false, SLOT(updateFrame()));
        duration_property_ = new rviz_common::properties::FloatProperty(
            "duration", 10.0, "duration to visualize trajectory", this, SLOT(updateDuration()));
        line_width_property_ = new rviz_common::properties::FloatProperty(
            "line_width", 0.01, "line width", this, SLOT(updateLineWidth()));
        color_property_ = new rviz_common::properties::ColorProperty(
            "color", QColor(25, 255, 240), "color of trajectory", this, SLOT(updateColor()));
        duration_property_->setMin(0.0);
        line_width_property_->setMin(0.0);
    }

    TFTrajectoryDisplay::~TFTrajectoryDisplay()
    {
        delete line_width_property_;
        delete frame_property_;
        delete duration_property_;
        delete color_property_;
        delete line_;
    }

    void TFTrajectoryDisplay::onInitialize()
    {
        this->rviz_node_ = this->context_->getRosNodeAbstraction().lock()->get_raw_node();

        frame_property_->setFrameManager(context_->getFrameManager());
        line_ = new rviz_rendering::BillboardLine(context_->getSceneManager(), scene_node_);
        updateFrame();
        updateDuration();
        updateColor();
        updateLineWidth();
    }

    void TFTrajectoryDisplay::updateFrame()
    {
        frame_ = frame_property_->getFrame().toStdString();
        trajectory_.clear();
    }

    void TFTrajectoryDisplay::reset()
    {
        trajectory_.clear();
    }

    void TFTrajectoryDisplay::updateDuration()
    {
        duration_ = duration_property_->getFloat();
    }

    void TFTrajectoryDisplay::updateColor()
    {
        color_ = color_property_->getColor();
    }

    void TFTrajectoryDisplay::updateLineWidth()
    {
        line_width_ = line_width_property_->getFloat();
    }

    void TFTrajectoryDisplay::onEnable()
    {
        // console("[onEnable]");
        line_->clear();
        trajectory_.clear();
    }

    void TFTrajectoryDisplay::onDisable()
    {
        // console("[onDisable]");
        line_->clear();
        trajectory_.clear();
    }

    void TFTrajectoryDisplay::update(float wall_dt, float ros_dt)
    {
        (void)wall_dt;
        (void)ros_dt;
        // console("[update] wall_dt: %.2f, ros_dt: %.2f", wall_dt, ros_dt);

        if (frame_.empty())
        {
            return;
        }

        std::string fixed_frame_id = context_->getFrameManager()->getFixedFrame();
        if (fixed_frame_ != fixed_frame_id)
        {
            fixed_frame_ = fixed_frame_id;
            line_->clear();
            trajectory_.clear();
            return;
        }

        fixed_frame_ = fixed_frame_id;
        rclcpp::Time now = context_->getFrameManager()->getTime();

        std_msgs::msg::Header header;
        header.stamp = rclcpp::Time(0.0);
        header.frame_id = frame_;

        Ogre::Vector3 position;
        Ogre::Quaternion orientation;
        if (!context_->getFrameManager()->getTransform(header, position, orientation))
        {
            setStatus(rviz_common::properties::StatusProperty::Error,
                      "transformation",
                      QString("Failed transforming from frame %1 to frame %2")
                          .arg(header.frame_id.c_str())
                          .arg(fixed_frame_id.c_str()));

            return;
        }
        setStatus(rviz_common::properties::StatusProperty::Ok, "transformation", "Ok");

        geometry_msgs::msg::PointStamped new_point;
        new_point.header.stamp = now;
        new_point.point.x = position[0];
        new_point.point.y = position[1];
        new_point.point.z = position[2];
        trajectory_.push_back(new_point);
        // check old data, is it too slow??
        for (std::vector<geometry_msgs::msg::PointStamped>::iterator it = trajectory_.begin(); it != trajectory_.end();)
        {
            rclcpp::Duration duration = now - it->header.stamp;
            if (duration.seconds() > duration_)
            {
                it = trajectory_.erase(it);
            }
            else
            {
                break;
            }
        }
        line_->clear();
        line_->setNumLines(1);
        line_->setMaxPointsPerLine(trajectory_.size());
        line_->setLineWidth(line_width_);
        line_->setColor(color_.red() * 255.0, color_.green() * 255.0, color_.blue() * 255.0, 255.0);
        for (size_t i = 0; i < trajectory_.size(); i++)
        {
            Ogre::Vector3 p;
            p[0] = trajectory_[i].point.x;
            p[1] = trajectory_[i].point.y;
            p[2] = trajectory_[i].point.z;
            line_->addPoint(p);
        }
    }
} // namespace arc_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(arc_rviz_plugins::TFTrajectoryDisplay, rviz_common::Display)
