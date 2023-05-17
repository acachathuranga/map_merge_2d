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

#include <map_merge_2d/map_discovery/topic_discovery.hpp>

using namespace map_merge_2d;

/**
 * @brief Construct a new Topic Discovery:: Topic Discovery object
 * 
 * @param node 
 * @param info
 * @param callback discovered callback(topic_full_name, topic_namespace)
 */
TopicDiscovery::TopicDiscovery(ros::NodeHandle *node, TopicInfo info, std::function<void (std::string)> callback)
{
    if (node == nullptr)
    {
        ROS_ERROR_STREAM(ros::this_node::getName() << " : [TopicDiscovery] : " << "Null node handle passed for discovery" );
        return;
    }

    if (info.topic_name == "")
    {
        ROS_ERROR_STREAM(ros::this_node::getName() << " : [TopicDiscovery] : " << "Cannot discover an empty topic" );
        return;
    }

    if (info.discovery_rate <= 0)
    {
        ROS_ERROR_STREAM(ros::this_node::getName() << " : [TopicDiscovery] : " << "Discovery rate should be positive. Given: " << info.discovery_rate );
        return;
    }

    if (callback == nullptr)
    {
        ROS_ERROR_STREAM(ros::this_node::getName() << " : [TopicDiscovery] : " << "Discovery without a callback initiated" );
        return;
    }

    // Store args
    node_ = node;
    info_ = info;
    callback_ = callback;

    // Create timers
    discover_timer_ = node_->createTimer(ros::Duration(1.0/info_.discovery_rate), 
                        std::bind(&TopicDiscovery::discovery_callback, this));
}

void TopicDiscovery::discovery_callback()
{
  std::vector<std::string> topics = get_topic_matches();
  for (const auto& topic : topics)
  {
    if(!discovered_topics_.count(topic))
    {
        // New topic discovered
        discovered_topics_.insert({topic, ros_names::parentNamespace(topic)});
        callback_(topic);
    }
  }
}

std::vector<std::string> TopicDiscovery::get_topic_matches()
{
    std::vector<std::string> topics;

    ros::master::V_TopicInfo topic_infos;
    ros::master::getTopics(topic_infos);

    // Iterate over all topics
    for (const auto& topic_info : topic_infos)
    {
        // test whether topic matches given topic name 
        std::string topic_namespace = ros_names::parentNamespace(topic_info.name);
        bool is_map_topic = (ros_names::append(topic_namespace, info_.topic_name) == topic_info.name);

        // test whether topic contains *anywhere* given topic namespace 
        auto pos = topic_info.name.find(info_.topic_namespace);
        bool contains_namespace = (pos != std::string::npos);

        // check topic type match
        bool is_type_match = (topic_info.datatype == info_.topic_type);

        // check topic exclusions list
        auto itr = std::find(info_.exclusions.begin(), info_.exclusions.end(), topic_info.name);
        bool excluded_topic = (itr != info_.exclusions.end());

        if (is_map_topic && contains_namespace && is_type_match && (!excluded_topic))
            topics.emplace_back(topic_info.name);
    }
    return topics;
}
