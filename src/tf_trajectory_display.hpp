#ifndef ARC_RVIZ_PLUGINS_TF_TRAJECTORY_DISPLAY_H_
#define ARC_RVIZ_PLUGINS_TF_TRAJECTORY_DISPLAY_H_

#ifndef Q_MOC_RUN
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_common/properties/tf_frame_property.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>
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
        // -------------------------------------------------------
        rviz_rendering::BillboardLine *line_;
        
        std::string frame_;
        std::string fixed_frame_;
        float duration_;
        QColor color_;
        float line_width_;
        rclcpp::Clock clock_;
    protected Q_SLOTS:
        void updateFrame();
        void updateDuration();
        void updateColor();
        void updateLineWidth();
    private:
        // -------------------------------------------------------            
        // User-editable property variables.
        // -------------------------------------------------------
        rviz_common::properties::TfFrameProperty *frame_property_;
        rviz_common::properties::FloatProperty *duration_property_;
        rviz_common::properties::ColorProperty *color_property_;
        rviz_common::properties::FloatProperty *line_width_property_;
        // -------------------------------------------------------
        rclcpp::Node::SharedPtr rviz_node_; // ros node handler
        // -------------------------------------------------------
        std::vector<geometry_msgs::msg::PointStamped> trajectory_;
    };
} // namespace arc_rviz_plugins

#endif // ARC_RVIZ_PLUGINS_TF_TRAJECTORY_DISPLAY_H_
