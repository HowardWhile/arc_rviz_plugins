#include "tf_trajectory_display.hpp"

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
        this->frame_property_ = new rviz_common::properties::TfFrameProperty(
            "Frame ID", "", "frame to visualize trajectory", this, NULL, false, SLOT(updateFrame()));
            
        this->duration_property_ = new rviz_common::properties::FloatProperty(
            "Duration", 10.0, "duration to visualize trajectory", this, SLOT(updateDuration()));
        this->duration_property_->setMin(0.0);

        this->line_width_property_ = new rviz_common::properties::FloatProperty(
            "Line Width", 0.01, "line width", this, SLOT(updateLineWidth()));
        this->line_width_property_->setMin(0.0);

        this->color_property_ = new rviz_common::properties::ColorProperty(
            "Color", QColor(25, 255, 240), "color of trajectory", this, SLOT(updateColor()));
    }

    TFTrajectoryDisplay::~TFTrajectoryDisplay()
    {
        // delete line_width_property_;
        delete frame_property_;
        delete duration_property_;
        delete color_property_;
    }

    void TFTrajectoryDisplay::onInitialize()
    {
        this->rviz_node_ = this->context_->getRosNodeAbstraction().lock()->get_raw_node();

        frame_property_->setFrameManager(context_->getFrameManager());
        this->line_ = std::make_shared<rviz_rendering::BillboardLine>(scene_manager_, scene_node_ );
        this->axes_ = std::make_shared<rviz_rendering::Axes>(scene_manager_, scene_node_);
        this->axes_->getSceneNode()->setVisible(isEnabled());

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
                      "TransFormation",
                      QString("Failed transforming from frame <b>%1</b> to frame <b>%2</b>")
                          .arg(header.frame_id.c_str())
                          .arg(fixed_frame_id.c_str()));

            deleteStatus("Trajectory");
            return;
        }
        setStatus(rviz_common::properties::StatusProperty::Ok, "TransFormation", "Ok");
        setStatus(rviz_common::properties::StatusProperty::Ok, "Trajectory", QString("size %1").arg(trajectory_.size()));

        // geometry_msgs::msg::PointStamped new_point;
        // new_point.header.stamp = now;
        // new_point.point.x = position[0];
        // new_point.point.y = position[1];
        // new_point.point.z = position[2];

        geometry_msgs::msg::PoseStamped new_pose;
        new_pose.header.frame_id = fixed_frame_id;
        new_pose.header.stamp = now;
        this->updatePose(position, orientation, new_pose.pose);

        trajectory_.push_back(new_pose);
        // check old data, is it too slow??
        for (std::vector<geometry_msgs::msg::PoseStamped>::iterator it = trajectory_.begin(); it != trajectory_.end();)
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

        for (const auto &pose_stamp : trajectory_)
        {
            Ogre::Vector3 p;
            p[0] = pose_stamp.pose.position.x;
            p[1] = pose_stamp.pose.position.y;
            p[2] = pose_stamp.pose.position.z;
            line_->addPoint(p);
        }
    }

    void TFTrajectoryDisplay::updatePose(Ogre::Vector3 position, Ogre::Quaternion orientation, geometry_msgs::msg::Pose &pose)
    {
        pose.position.x = position.x;
        pose.position.y = position.y;
        pose.position.z = position.z;

        pose.orientation.x = orientation.x;        
        pose.orientation.y = orientation.y;           
        pose.orientation.z = orientation.z;            
        pose.orientation.w = orientation.w;          
    }
} // namespace arc_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(arc_rviz_plugins::TFTrajectoryDisplay, rviz_common::Display)
