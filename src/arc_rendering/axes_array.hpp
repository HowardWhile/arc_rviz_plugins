#ifndef ARC_RENDERING_AXES_ARRAY_HPP_
#define ARC_RENDERING_AXES_ARRAY_HPP_

#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#include <OgreColourValue.h>

namespace arc_rendering
{
    class AxesArray
    {
    private:
        /* data */
    public:
        AxesArray(Ogre::SceneManager *scene_manager,
                  Ogre::SceneNode *parent_node = nullptr,
                  float length = 1.0f,
                  int radius = 1) // pixel
        {
            this->scene_manager_ = scene_manager;
            if (!parent_node)
            {
                parent_node = scene_manager_->getRootSceneNode();
            }
            scene_node_ = parent_node->createChildSceneNode();

            length_ = length;
            radius_ = radius;

            this->x_axes_lines_ = this->scene_manager_->createManualObject();
            this->scene_node_->attachObject(this->x_axes_lines_);

            // test
            this->x_axes_lines_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);

            this->x_axes_lines_->position(Ogre::Vector3(0, 0, 0));
            this->x_axes_lines_->colour(Ogre::ColourValue::Red);
            this->x_axes_lines_->position(Ogre::Vector3(1, 0, 0));

            this->x_axes_lines_->position(Ogre::Vector3(0, 0, 0));
            this->x_axes_lines_->colour(Ogre::ColourValue::Green);
            this->x_axes_lines_->position(Ogre::Vector3(0, 1, 0));

            this->x_axes_lines_->position(Ogre::Vector3(0, 0, 0));
            this->x_axes_lines_->colour(Ogre::ColourValue::Blue);
            this->x_axes_lines_->position(Ogre::Vector3(0, 0, 1));
            this->x_axes_lines_->end();
            // this->x_axes_lines_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
        }

        ~AxesArray()
        {
        }

        void setPoseArray(std::vector<geometry_msgs::msg::PoseStamped> pose_stamped_array)
        {
            geometry_msgs::msg::PoseArray pose_array;

            for (const auto &pose_stamped : pose_stamped_array)
            {
                pose_array.poses.push_back(pose_stamped.pose);
            }
            setPoseArray(pose_array);
        }

        void setPoseArray(geometry_msgs::msg::PoseArray pose_array)
        {
            pose_array_ = pose_array;
        }

    private:
        Ogre::SceneManager *scene_manager_;
        Ogre::SceneNode *scene_node_;
        float length_;
        int radius_;

        Ogre::ManualObject *x_axes_lines_;
        Ogre::ManualObject *y_axes_lines_;
        Ogre::ManualObject *z_axes_lines_;
        Ogre::ColourValue x_axes_colour_;
        Ogre::ColourValue y_axes_colour_;
        Ogre::ColourValue z_axes_colour_;

        geometry_msgs::msg::PoseArray pose_array_;
    };

}

#endif