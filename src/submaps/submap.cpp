/*
 * Copyright (c) 2023 Achala Athukorala <chhathuranga@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <map_merge_2d/submaps/submap.hpp>

using namespace map_merge_2d;

SubMap::SubMap(std::shared_ptr<ros::NodeHandle> node, std::string map_topic) 
:   available(false),
    name(ros_names::parentNamespace(map_topic)),
    known_pose_(false)
{
    if (node == nullptr)
    {
        ROS_ERROR_STREAM(ros::this_node::getName() << " : [SubMap] : " << "Invalid node[null] passed to submap creation. Topic: " << map_topic);
        throw std::invalid_argument("Null pointer passed as node to SubMap");
    }

    // Check for initial_pose parameters
    std::string map_namespace = ros_names::parentNamespace(map_topic);

    double x, y, z, yaw;
    if (node->getParam(ros_names::append(map_namespace, "init_pose_x").erase(0,1), x) &&
        node->getParam(ros_names::append(map_namespace, "init_pose_y").erase(0,1), y) &&
        node->getParam(ros_names::append(map_namespace, "init_pose_z").erase(0,1), z) &&
        node->getParam(ros_names::append(map_namespace, "init_pose_yaw").erase(0,1), yaw) )
    {
        transform_.setOrigin(tf2::Vector3(x, y, z));
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        transform_.setRotation(q);
        known_pose_ = true;
        
        ROS_INFO("%s : %s : map subscribed. Initial pose(xyz yaw) [%.3f, %.3f, %.3f, %.3f]",
                                        ros::this_node::getName().c_str(), map_topic.c_str(), x, y, z, yaw);
    }
    else
    {
        ROS_WARN_STREAM(ros::this_node::getName() << " : " << map_topic << " initial pose not set. " 
            << "Did you set " << map_namespace << "/init_pose[x,y,z,yaw] parameters? "
            << "We will try to auto calculate initial pose");
        transform_.setIdentity();
    }
    
    subscriber_ = node->subscribe<nav_msgs::OccupancyGrid>(map_topic, 10, &SubMap::update_map, this);

}

SubMap::Map SubMap::get_map(void)
{
    std::unique_lock<std::mutex> lock(mutex_);
    Map map;
    map.map = map_;
    map.known_pose = known_pose_;
    map.transform_ = transform_;
    map.transform_confidence_ = transform_confidence_;
    map.name_ = name;
    
    return map;
}

void SubMap::update_transform(tf2::Transform transform)
{
    std::unique_lock<std::mutex> lock(mutex_);
    transform_ = transform;

    // Set to known_pose configuration once map transform is initialized
    if(!known_pose_)
    {
        ROS_DEBUG("%s : %s map transformation established!", ros::this_node::getName().c_str(), name.c_str());
    }
    known_pose_ = true;
}

void SubMap::update_transform(tf2::Transform transform, double confidence)
{
    std::unique_lock<std::mutex> lock(mutex_);
    transform_ = transform;
    transform_confidence_ = confidence;

    // Set to known_pose configuration once map transform is initialized
    if(!known_pose_)
    {
        ROS_DEBUG("%s : %s map transformation established!", ros::this_node::getName().c_str(), name.c_str());
    }
    known_pose_ = true;
}

void SubMap::update_map(nav_msgs::OccupancyGrid msg)
{
    std::unique_lock<std::mutex> lock(mutex_);
    map_ = msg;

    // Set map available flag
    if (!available)
        available = true;
}