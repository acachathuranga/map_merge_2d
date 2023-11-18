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

#include <map_merge_2d/map_merger.hpp>

using namespace map_merge_2d;

MapMerger::MapMerger()
{
  node_ = std::make_shared<ros::NodeHandle>("~");

  TopicDiscovery::TopicInfo info;
  std::string merged_map_topic;

  // Get parameters
  node_->param<double>("discovery_rate", info.discovery_rate, 0.5);
  node_->param<std::string>("map_topic", info.topic_name, "map");
  node_->param<std::string>("map_namespace", info.topic_namespace, "");
  node_->param<std::string>("merged_map_topic", merged_map_topic, "merged_map");
  node_->param<std::string>("world_frame", world_frame_, "world");

  info.exclusions.emplace_back(ros_names::append(ros::this_node::getName(), merged_map_topic));
  info.topic_type = "nav_msgs/OccupancyGrid";

  // Start Map TF Subscriber
  map_tf_subscriber_ = node_->subscribe<tf2_msgs::TFMessage>("submap_transforms", 10, &MapMerger::map_tf_callback, this);

  // start topic discovery
  map_discovery_ = std::make_shared<TopicDiscovery>(node_, info, 
                            std::bind(&MapMerger::topic_discovery_callback, this, std::placeholders::_1));

  // start map matcher
  map_matcher_ = std::make_shared<SubMapMatcher>(node_, std::bind(&MapMerger::get_submaps, this));
  
  // start map merger
  map_merger_ = std::make_shared<SubMapMerger>(node_, std::bind(&MapMerger::get_submaps, this));

}

void MapMerger::topic_discovery_callback(std::string topic_name)
{
  std::unique_lock<std::mutex> lock(submap_mutex_);
  std::shared_ptr<SubMap> submap_ptr = std::make_shared<SubMap>(node_, topic_name);
  if (submap_ptr->get_map().known_pose) {
    ROS_INFO("Anchor map added :  %s", topic_name.c_str());
    submaps_.emplace(submaps_.begin(), submap_ptr);
  } else {
    submaps_.emplace_back(submap_ptr);
  }
}

std::vector<std::shared_ptr<SubMap>> MapMerger::get_submaps()
{
  std::unique_lock<std::mutex> lock(submap_mutex_);

  // Return only valid maps (received maps)
  std::vector<std::shared_ptr<SubMap>> submaps;
  for (auto &submap : submaps_)
  {
    if(submap->available)
      submaps.emplace_back(submap);
  }

  return submaps;
}

void MapMerger::map_tf_callback(tf2_msgs::TFMessage msg)
{
  for (auto transform : msg.transforms)
  {
    // Clean frame names
    std::string world_frame = world_frame_;
    if (transform.header.frame_id.length() > 0 && transform.header.frame_id.at(0) == '/') transform.header.frame_id.erase(0, 1);
    if (transform.child_frame_id.length() > 0 && transform.child_frame_id.at(0) == '/') transform.child_frame_id.erase(0, 1);
    if (world_frame.length() > 0 && world_frame.at(0) == '/') world_frame.erase(0, 1);

    if (transform.header.frame_id != world_frame){
      ROS_ERROR_STREAM("Input transform frame [" << transform.header.frame_id << "] mismatch with world frame id [" << world_frame_ << "]. Ignoring transform");
      continue;
    }

    std::unique_lock<std::mutex> lock(submap_mutex_);
    ROS_INFO("Received transform for map %s  Translation: [%.2f, %.2f, %.2f]  Rotation: [%.2f, %.2f, %.2f, %.2f]",
              transform.child_frame_id.c_str(), 
              transform.transform.translation.x,
              transform.transform.translation.y,
              transform.transform.translation.z,
              transform.transform.rotation.x,
              transform.transform.rotation.y,
              transform.transform.rotation.z,
              transform.transform.rotation.w);

    // Check for submap existance
    std::vector<std::shared_ptr<SubMap>>::iterator submap_it = submaps_.begin();
    for(; submap_it < submaps_.end(); submap_it++){
      std::string submap_name = (*submap_it)->name;
      if (submap_name.length() > 0 && submap_name.at(0) == '/') submap_name.erase(0, 1);
      if (submap_name == transform.child_frame_id){
        tf2::Transform tf_transform;
        tf_transform.setOrigin(tf2::Vector3(transform.transform.translation.x, 
                                            transform.transform.translation.y, 
                                            transform.transform.translation.z));
        tf_transform.setRotation(tf2::Quaternion(transform.transform.rotation.x,
                                                  transform.transform.rotation.y,
                                                  transform.transform.rotation.z,
                                                  transform.transform.rotation.w));
        (*submap_it)->update_transform(tf_transform);
        ROS_INFO("SubMap Transform updated!");
        break;
      }
    }
    // Create ros parameter if submap doesn't exist
    if (submap_it == submaps_.end()){
      std::string node_ns =  node_->getNamespace();
      tf2::Quaternion q(transform.transform.rotation.x,
                        transform.transform.rotation.y,
                        transform.transform.rotation.z,
                        transform.transform.rotation.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3 m(q);
      m.getEulerYPR(yaw, pitch, roll);
      ros::param::set(node_ns + "/" + transform.child_frame_id + "/init_pose_x", transform.transform.translation.x);
      ros::param::set(node_ns + "/" + transform.child_frame_id + "/init_pose_y", transform.transform.translation.y);
      ros::param::set(node_ns + "/" + transform.child_frame_id + "/init_pose_z", transform.transform.translation.z);
      ros::param::set(node_ns + "/" + transform.child_frame_id + "/init_pose_yaw", yaw);
      ROS_INFO("SubMap Not Available! Parameterizing transform data for future use! [%.2f, %.2f, %.2f, %.2f]",
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
                yaw);
    }

  }
}