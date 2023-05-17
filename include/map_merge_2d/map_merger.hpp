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

#ifndef MAP_MERGE_2D_H
#define MAP_MERGE_2D_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <mutex>
#include <shared_mutex>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <map_merge_2d/submaps/submap.hpp>
#include <map_merge_2d/util/topic_name_utils.hpp>
#include <map_merge_2d/map_discovery/topic_discovery.hpp>
#include <map_merge_2d/submaps/submap_matcher.hpp>
#include <map_merge_2d/submaps/submap_merger.hpp>

namespace map_merge_2d
{
    class MapMerger
    {
        public:
            MapMerger();

        private:
            void topic_discovery_callback(std::string topic_name);
            std::vector<std::shared_ptr<SubMap>> get_submaps();
            void publish_tf(void);

            std::shared_ptr<TopicDiscovery> map_discovery_;
            std::shared_ptr<SubMapMatcher> map_matcher_;
            std::shared_ptr<SubMapMerger> map_merger_;
            std::shared_ptr<ros::NodeHandle> node_;

            // Protected variables - to be accessed with thread safety only
            std::vector<std::shared_ptr<SubMap>> submaps_;
            std::mutex submap_mutex_;
};

}  // namespace map_merge_2d

#endif  // MAP_MERGE_2D_H