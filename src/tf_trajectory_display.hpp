#ifndef ARC_RVIZ_PLUGINS_TF_TRAJECTORY_DISPLAY_H_
#define ARC_RVIZ_PLUGINS_TF_TRAJECTORY_DISPLAY_H_

#include "arc_rendering/axes_array.hpp"

#ifndef Q_MOC_RUN
#include <vector>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_common/properties/tf_frame_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>
#include "rviz_rendering/material_manager.hpp"
#include <rviz_rendering/objects/arrow.hpp>
#endif

namespace arc_rviz_plugins
{
    class TFTrajectoryDisplay : public rviz_common::Display
    {
        Q_OBJECT
    public:
        TFTrajectoryDisplay();
        ~TFTrajectoryDisplay();

    protected:
        // -------------------------------------------------------
        // override rviz_common::Display event
        // -------------------------------------------------------
        void onInitialize() override;
        void onEnable() override;
        void onDisable() override;
        void update(float wall_dt, float ros_dt) override;
        void reset() override;
        // -------------------------------------------------------
        std::string frame_;
        std::string fixed_frame_;
        float duration_;
        QColor line_color_;
        float line_width_;

        rclcpp::Clock clock_;
    protected Q_SLOTS:
        void updateFrame();
        void updateDuration();
        void updateStyle();
        void updateColor();
        void updateLineWidth();
        void updatePoseStyle();
        void updatePoseAxisGeometry();
        void updatePoseArrowColor();
        void updatePoseArrowGeometry();
        void updatePoseArrowDirection();

    private:
        // -------------------------------------------------------
        // User-editable property variables.
        // -------------------------------------------------------
        rviz_common::properties::TfFrameProperty *frame_property_;
        rviz_common::properties::FloatProperty *duration_property_;

        // line style
        rviz_common::properties::EnumProperty *line_style_property_;
        rviz_common::properties::ColorProperty *line_color_property_;
        rviz_common::properties::FloatProperty *line_width_property_;
        enum LineStyle
        {
            LINE,
            BILLBOARD
        };

        // pose marker property
        rviz_common::properties::EnumProperty *pose_style_property_;
        rviz_common::properties::FloatProperty *pose_axes_length_property_;
        rviz_common::properties::ColorProperty *pose_arrow_color_property_;
        rviz_common::properties::FloatProperty *pose_arrow_shaft_length_property_;
        rviz_common::properties::FloatProperty *pose_arrow_head_length_property_;
        rviz_common::properties::FloatProperty *pose_arrow_shaft_diameter_property_;
        rviz_common::properties::FloatProperty *pose_arrow_head_diameter_property_;
        rviz_common::properties::EnumProperty *pose_arrow_direction_property_;
        enum PoseStyle
        {
            NONE,
            AXES,
            ARROWS,
        };
        enum ArrowDirection
        {
            DIR_PX,
            DIR_PY,
            DIR_PZ,
            DIR_NX,
            DIR_NY,
            DIR_NZ
        };

        // -------------------------------------------------------
        rclcpp::Node::SharedPtr rviz_node_; // ros node handler
        // -------------------------------------------------------
        std::vector<geometry_msgs::msg::PoseStamped> trajectory_;
        // line style
        std::shared_ptr<rviz_rendering::BillboardLine> billboard_line_;
        Ogre::ManualObject *manual_line_;
        Ogre::MaterialPtr manual_line_material_;
        // pose style
        // axes
        std::shared_ptr<arc_rendering::AxesArray> axes_array_;

        // arrow
        std::vector<rviz_rendering::Arrow *> arrow_list_;
        void allocateArrowVector(std::vector<rviz_rendering::Arrow *> &arrow_vect, size_t num);
        Ogre::ColourValue arrow_color_;
        Ogre::Vector3 arrow_direction_;
        // -------------------------------------------------------
        void updatePose(Ogre::Vector3 position, Ogre::Quaternion orientation, geometry_msgs::msg::Pose &pose);
        // -------------------------------------------------------
    };
} // namespace arc_rviz_plugins

#endif // ARC_RVIZ_PLUGINS_TF_TRAJECTORY_DISPLAY_H_
