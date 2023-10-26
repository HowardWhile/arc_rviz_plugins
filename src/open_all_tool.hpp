#ifndef ARC_RVIZ_PLUGINS_OPEN_ALL_TOOL_HPP
#define ARC_RVIZ_PLUGINS_OPEN_ALL_TOOL_HPP

#include <rviz_common/tool.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/property_tree_model.hpp>

namespace arc_rviz_plugins
{

    class OpenAllTool : public rviz_common::Tool
    {
    public:
        OpenAllTool();
        virtual ~OpenAllTool();
        virtual void onInitialize();
        virtual void activate();
        virtual void deactivate();

    protected:
        virtual void openProperty(rviz_common::properties::Property *property);

    private:
    };

}

#endif // ARC_RVIZ_PLUGINS_OPEN_ALL_TOOL_HPP
