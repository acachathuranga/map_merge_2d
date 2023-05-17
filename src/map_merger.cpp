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

  info.exclusions.emplace_back(ros_names::append(ros::this_node::getName(), merged_map_topic));
  info.topic_type = "nav_msgs/msg/OccupancyGrid";

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
  submaps_.emplace_back(std::make_shared<SubMap>(node_, topic_name));
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