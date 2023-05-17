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

#ifndef SUBMAP_H
#define SUBMAP_H

#include <string>
#include <stdexcept>
#include <atomic>
#include <mutex>

#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <nav_msgs/OccupancyGrid.h>

#include <map_merge_2d/util/topic_name_utils.hpp>

namespace map_merge_2d
{
    class SubMap
    {
        public:
            struct Map
            {
                nav_msgs::OccupancyGrid map;
                bool known_pose;
                tf2::Transform transform_;
                double transform_confidence_;
                std::string name_;
            };

            SubMap(ros::NodeHandle *node, std::string map_topic);

            Map get_map(void);
            void update_transform(tf2::Transform transform);
            void update_transform(tf2::Transform transform, double confidence);

            std::atomic<bool> available;
            const std::string name;

        private:
            void update_map(nav_msgs::OccupancyGrid msg);

            bool known_pose_;
            tf2::Transform transform_;
            double transform_confidence_ = -1;
            ros::Subscriber subscriber_;
            std::mutex mutex_;
            nav_msgs::OccupancyGrid map_;

};

}  // namespace map_merge_2d

#endif  // SUBMAP_H