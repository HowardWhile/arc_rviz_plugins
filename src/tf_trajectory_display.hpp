#ifndef ARC_RVIZ_PLUGINS_DISPLAY_TEMPLATE_HPP
#define ARC_RVIZ_PLUGINS_DISPLAY_TEMPLATE_HPP

#include <rviz_common/display.hpp>

namespace arc_rviz_plugins
{

    class TFTrajectoryDisplay : public rviz_common::Display
    {
        Q_OBJECT
    public:
        TFTrajectoryDisplay();

        ~TFTrajectoryDisplay() override;

        void onInitialize() override;
        void update(float dt, float ros_dt) override;

    protected:
        void onEnable() override;
        void onDisable() override;

    private Q_SLOTS:
        // void updateShape();

    private:
    };

}

#endif // ARC_RVIZ_PLUGINS_DISPLAY_TEMPLATE_HPP
