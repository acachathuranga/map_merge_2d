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

#ifndef SUBMAP_MATCHER_H
#define SUBMAP_MATCHER_H

#include <functional>
#include <vector>

#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <nav_msgs/OccupancyGrid.h>

#include <map_merge_2d/submaps/submap.hpp>
#include <map_merge_2d/cv_core/cv_core.hpp>
#include <map_merge_2d/util/tf_utils.hpp>

namespace map_merge_2d
{
    class SubMapMatcher
    {
        public:
            struct MatcherOptions
            {
                double confidence = 0.5;
                int dilation = 4;
            };

            SubMapMatcher(MatcherOptions options);
            SubMapMatcher(std::shared_ptr<ros::NodeHandle> node, std::function<std::vector<std::shared_ptr<SubMap>> ()> submap_reader);

            void match(std::vector<std::shared_ptr<SubMap>> submaps);
            void match(void);

        private:
            bool has_known_tf(std::vector<SubMap::Map> maps);
            bool has_known_tf(std::vector<SubMap::Map> maps, std::map<int, cv::Mat> estimates, std::vector<int> &idx);
            double get_overlap(nav_msgs::OccupancyGrid map1, nav_msgs::OccupancyGrid map2, tf2::Transform map1_tf, tf2::Transform map2_tf);

            std::shared_ptr<ros::NodeHandle> node_;
            MatcherOptions options_;
            ros::Timer matcher_timer_;
            std::function<std::vector<std::shared_ptr<SubMap>> ()> submap_reader_;
    };
}  // namespace map_merge_2d

#endif  // SUBMAP_MATCHER_H