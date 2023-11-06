#ifndef ARC_RENDERING_AXES_ARRAY_HPP_
#define ARC_RENDERING_AXES_ARRAY_HPP_

#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <rviz_common/msg_conversions.hpp>

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
                  float length = 1.0f) // pixel
        {
            this->scene_manager_ = scene_manager;
            if (!parent_node)
            {
                parent_node = scene_manager_->getRootSceneNode();
            }
            scene_node_ = parent_node->createChildSceneNode();

            length_ = length;

            this->manual_axes_ = this->scene_manager_->createManualObject();
            this->scene_node_->attachObject(this->manual_axes_);
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

            this->manual_axes_->clear();
            this->manual_axes_->estimateVertexCount(pose_array_.poses.size() * 2 * 3);
            this->manual_axes_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
            for (const auto &pose : pose_array.poses)
            {
                Ogre::Vector3 axis_origin = rviz_common::pointMsgToOgre(pose.position);                 // 軸的起始位置
                Ogre::Quaternion axis_orientation = rviz_common::quaternionMsgToOgre(pose.orientation); // 軸的方向

                // 繪製X軸
                manual_axes_->position(axis_origin);
                manual_axes_->colour(Ogre::ColourValue::Red);
                manual_axes_->position(axis_origin + axis_orientation * Ogre::Vector3::UNIT_X * this->length_);

                // 繪製X軸
                manual_axes_->position(axis_origin);
                manual_axes_->colour(Ogre::ColourValue::Green);
                manual_axes_->position(axis_origin + axis_orientation * Ogre::Vector3::UNIT_Y * this->length_);

                // 繪製X軸
                manual_axes_->position(axis_origin);
                manual_axes_->colour(Ogre::ColourValue::Blue);
                manual_axes_->position(axis_origin + axis_orientation * Ogre::Vector3::UNIT_Z * this->length_);
            }
            this->manual_axes_->end();
        }

        void updateLength(float length)
        {
            this->length_ = length;
        }

    private:
        Ogre::SceneManager *scene_manager_;
        Ogre::SceneNode *scene_node_;
        float length_;

        Ogre::ManualObject *manual_axes_;

        geometry_msgs::msg::PoseArray pose_array_;
    };

}

#endif