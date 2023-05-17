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

#ifndef TOPIC_DISCOVERY_H
#define TOPIC_DISCOVERY_H

#include <string>
#include <vector>
#include <functional>
#include <unordered_map>
#include <ros/ros.h>

#include <map_merge_2d/util/topic_name_utils.hpp>

namespace map_merge_2d
{
    class TopicDiscovery
    {   
        public:
            /**
             * @brief Information of topic to be discovered
             * @example /ground_robots/robot1/map
             * 
             * @param topic_name : map
             * @param topic_namespace : ground_robots (any regex match)
             */
            struct TopicInfo
            {
                std::string topic_name;
                std::string topic_namespace;
                std::string topic_type = "nav_msgs/OccupancyGrid";
                std::vector<std::string> exclusions;
                double discovery_rate = 1.0;
            };

            TopicDiscovery(std::shared_ptr<ros::NodeHandle> node, TopicInfo info, std::function<void (std::string)> callback);

        private:
            void discovery_callback();
            std::vector<std::string> get_topic_matches();

            std::shared_ptr<ros::NodeHandle> node_;
            TopicInfo info_;
            ros::Timer discover_timer_;
            std::function<void (std::string)> callback_;
            std::unordered_map<std::string, std::string> discovered_topics_; // map {topic_full_name : namespace}
};

}  // namespace map_merge_2d

#endif  // TOPIC_DISCOVERY_H