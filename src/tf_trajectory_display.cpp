#include "tf_trajectory_display.hpp"

#include <rviz_common/msg_conversions.hpp> // rviz_common::pointMsgToOgre

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

        this->line_style_property_ = new rviz_common::properties::EnumProperty(
            "Line Style", "Billboards", "The rendering operation to use to draw the grid lines.",
            this, SLOT(updateStyle()));
        this->line_style_property_->addOption("Lines", LINE);
        this->line_style_property_->addOption("Billboards", BILLBOARD);

        this->line_width_property_ = new rviz_common::properties::FloatProperty(
            "Line Width", 0.01, "line width", this, SLOT(updateLineWidth()));
        this->line_width_property_->setMin(0.0);

        this->line_color_property_ = new rviz_common::properties::ColorProperty(
            "Line Color", QColor(25, 255, 240), "color of trajectory", this, SLOT(updateColor()));

        pose_style_property_ = new rviz_common::properties::EnumProperty(
            "Pose Style", "None",
            "Shape to display the pose as.",
            this, SLOT(updatePoseStyle()));
        pose_style_property_->addOption("None", NONE);
        pose_style_property_->addOption("Axes", AXES);
        pose_style_property_->addOption("Arrows", ARROWS);

        pose_axes_length_property_ = new rviz_common::properties::FloatProperty(
            "Length", 0.3f,
            "Length of the axes.",
            this, SLOT(updatePoseAxisGeometry()));
        pose_axes_radius_property_ = new rviz_common::properties::FloatProperty(
            "Radius", 0.03f,
            "Radius of the axes.",
            this, SLOT(updatePoseAxisGeometry()));

        pose_arrow_color_property_ = new rviz_common::properties::ColorProperty(
            "Pose Color",
            QColor(255, 85, 255),
            "Color to draw the poses.",
            this, SLOT(updatePoseArrowColor()));
        pose_arrow_shaft_length_property_ = new rviz_common::properties::FloatProperty(
            "Shaft Length",
            0.1f,
            "Length of the arrow shaft.",
            this,
            SLOT(updatePoseArrowGeometry()));
        pose_arrow_head_length_property_ = new rviz_common::properties::FloatProperty(
            "Head Length", 0.2f,
            "Length of the arrow head.",
            this,
            SLOT(updatePoseArrowGeometry()));
        pose_arrow_shaft_diameter_property_ = new rviz_common::properties::FloatProperty(
            "Shaft Diameter",
            0.1f,
            "Diameter of the arrow shaft.",
            this,
            SLOT(updatePoseArrowGeometry()));
        pose_arrow_head_diameter_property_ = new rviz_common::properties::FloatProperty(
            "Head Diameter",
            0.3f,
            "Diameter of the arrow head.",
            this,
            SLOT(updatePoseArrowGeometry()));
        pose_axes_length_property_->hide();
        pose_axes_radius_property_->hide();
        pose_arrow_color_property_->hide();
        pose_arrow_shaft_length_property_->hide();
        pose_arrow_head_length_property_->hide();
        pose_arrow_shaft_diameter_property_->hide();
        pose_arrow_head_diameter_property_->hide();
    }

    TFTrajectoryDisplay::~TFTrajectoryDisplay()
    {
        // delete line_width_property_;
        delete frame_property_;
        delete duration_property_;
        delete line_color_property_;
    }

    void TFTrajectoryDisplay::onInitialize()
    {
        this->rviz_node_ = this->context_->getRosNodeAbstraction().lock()->get_raw_node();

        frame_property_->setFrameManager(context_->getFrameManager());
        this->billboard_line_ = std::make_shared<rviz_rendering::BillboardLine>(scene_manager_, scene_node_);

        // manual line initial
        this->manual_line_ = this->scene_manager_->createManualObject(); // 創建對象
        this->scene_node_->attachObject(this->manual_line_);             // 將對象附加到場景節點

        // 創建線的材質
        static int count = 0;
        std::string material_name = "arc_rviz_plugins::TFTrajectoryDisplay/LinesMaterial" + std::to_string(count++); // 材質名稱不能重複
        this->manual_line_material_ = rviz_rendering::MaterialManager::createMaterialWithNoLighting(material_name);

        updateFrame();
        updateDuration();
        updateStyle();
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

    void TFTrajectoryDisplay::updateStyle()
    {
        auto style = static_cast<LineStyle>(line_style_property_->getOptionInt());

        if (style == BILLBOARD)
        {
            this->line_width_property_->show();
        }
        else
        {
            this->line_width_property_->hide();
        }
    }

    void TFTrajectoryDisplay::updateColor()
    {
        color_ = line_color_property_->getColor();
    }

    void TFTrajectoryDisplay::updateLineWidth()
    {
        line_width_ = line_width_property_->getFloat();
    }

    void TFTrajectoryDisplay::updatePoseStyle()
    {
        auto pose_style = static_cast<PoseStyle>(pose_style_property_->getOptionInt());
        switch (pose_style)
        {
        case AXES:
            pose_axes_length_property_->show();
            pose_axes_radius_property_->show();
            pose_arrow_color_property_->hide();
            pose_arrow_shaft_length_property_->hide();
            pose_arrow_head_length_property_->hide();
            pose_arrow_shaft_diameter_property_->hide();
            pose_arrow_head_diameter_property_->hide();
            break;
        case ARROWS:
            pose_axes_length_property_->hide();
            pose_axes_radius_property_->hide();
            pose_arrow_color_property_->show();
            pose_arrow_shaft_length_property_->show();
            pose_arrow_head_length_property_->show();
            pose_arrow_shaft_diameter_property_->show();
            pose_arrow_head_diameter_property_->show();
            break;
        default:
            pose_axes_length_property_->hide();
            pose_axes_radius_property_->hide();
            pose_arrow_color_property_->hide();
            pose_arrow_shaft_length_property_->hide();
            pose_arrow_head_length_property_->hide();
            pose_arrow_shaft_diameter_property_->hide();
            pose_arrow_head_diameter_property_->hide();
        }
    }
    void TFTrajectoryDisplay::updatePoseAxisGeometry()
    {
    }
    void TFTrajectoryDisplay::updatePoseArrowColor()
    {
    }
    void TFTrajectoryDisplay::updatePoseArrowGeometry()
    {
    }

    void TFTrajectoryDisplay::onEnable()
    {
        // console("[onEnable]");
        billboard_line_->clear();
        trajectory_.clear();
    }

    void TFTrajectoryDisplay::onDisable()
    {
        // console("[onDisable]");
        billboard_line_->clear();
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

        // -------------------------------------------------------
        // Target Frame Infomation
        // -------------------------------------------------------
        std::string fixed_frame_id = context_->getFrameManager()->getFixedFrame();
        if (fixed_frame_ != fixed_frame_id)
        {
            fixed_frame_ = fixed_frame_id;
            billboard_line_->clear();
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

        // -------------------------------------------------------
        // 緩衝pose來製作軌跡 & check old data
        // -------------------------------------------------------
        geometry_msgs::msg::PoseStamped new_pose;
        new_pose.header.frame_id = fixed_frame_id;
        new_pose.header.stamp = now;
        this->updatePose(position, orientation, new_pose.pose);

        trajectory_.push_back(new_pose);
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

        // -------------------------------------------------------
        // Display trajectory line
        // -------------------------------------------------------
        this->billboard_line_->clear();
        this->manual_line_->clear();
        auto line_style = static_cast<LineStyle>(this->line_style_property_->getOptionInt());
        switch (line_style)
        {
        case LINE: // simple lines with fixed width of 1px
        {
            this->manual_line_->estimateVertexCount(trajectory_.size());

            // 開始繪圖
            this->manual_line_->begin(this->manual_line_material_->getName(), Ogre::RenderOperation::OT_LINE_STRIP, "rviz_rendering");
            auto tmp_color = this->line_color_property_->getOgreColor();
            for (const auto &pose_stamp : trajectory_)
            {
                // 添加線段的頂點
                this->manual_line_->position(rviz_common::pointMsgToOgre(pose_stamp.pose.position));
                rviz_rendering::MaterialManager::enableAlphaBlending(this->manual_line_material_, tmp_color.a);
                this->manual_line_->colour(tmp_color);
            }
            // 結束繪圖
            this->manual_line_->end();
            break;
        }

        case BILLBOARD: // billboards with configurable width
            billboard_line_->setNumLines(1);
            billboard_line_->setMaxPointsPerLine(trajectory_.size());
            billboard_line_->setLineWidth(line_width_);
            billboard_line_->setColor(color_.red() * 255.0, color_.green() * 255.0, color_.blue() * 255.0, 255.0);

            for (const auto &pose_stamp : trajectory_)
            {
                Ogre::Vector3 p;
                p[0] = pose_stamp.pose.position.x;
                p[1] = pose_stamp.pose.position.y;
                p[2] = pose_stamp.pose.position.z;
                billboard_line_->addPoint(p);
            }
            break;
        }

        // -------------------------------------------------------
        // Display trajectory pose
        // -------------------------------------------------------
        auto pose_style = static_cast<PoseStyle>(this->pose_style_property_->getOptionInt());
        this->allocateAxesVector(this->axes_list_, 0);
        this->allocateArrowVector(this->arrow_list_, 0);
        switch (pose_style)
        {
        case NONE:
            break;
        case AXES:
        {
            auto num = this->trajectory_.size();
            this->allocateAxesVector(this->axes_list_, num);
            for (size_t idx = 0; idx < num; ++idx)
            {
                // update position
                this->axes_list_[idx]->setPosition(rviz_common::pointMsgToOgre(this->trajectory_[idx].pose.position));

                // update rotation
                Ogre::Quaternion orientation(rviz_common::quaternionMsgToOgre(this->trajectory_[idx].pose.orientation));
                this->axes_list_[idx]->setOrientation(orientation);
            }

            context_->queueRender();
        }
        break;
        case ARROWS:
            break;
        }
    }

    void TFTrajectoryDisplay::allocateAxesVector(std::vector<rviz_rendering::Axes *> &axes_vect, size_t num)
    {
        auto vector_size = axes_vect.size();
        if (num > vector_size)
        {
            axes_vect.reserve(num);
            for (auto i = vector_size; i < num; ++i)
            {
                axes_vect.push_back(
                    new rviz_rendering::Axes(
                        scene_manager_, scene_node_,
                        pose_axes_length_property_->getFloat(),
                        pose_axes_radius_property_->getFloat()));
            }
        }
        else if (num < vector_size)
        {
            for (auto i = num; i < vector_size; ++i)
            {
                delete axes_vect[i];
            }
            axes_vect.resize(num);
        }
    }

    void TFTrajectoryDisplay::allocateArrowVector(std::vector<rviz_rendering::Arrow *> &arrow_vect, size_t num)
    {
        auto vector_size = arrow_vect.size();
        if (num > vector_size)
        {
            arrow_vect.reserve(num);
            for (auto i = vector_size; i < num; ++i)
            {
                arrow_vect.push_back(new rviz_rendering::Arrow(scene_manager_, scene_node_));
            }
        }
        else if (num < vector_size)
        {
            for (auto i = num; i < vector_size; ++i)
            {
                delete arrow_vect[i];
            }
            arrow_vect.resize(num);
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
