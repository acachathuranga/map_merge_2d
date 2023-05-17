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

#ifndef SUBMAP_MERGER_H
#define SUBMAP_MERGER_H

#include <functional>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <map_merge_2d/submaps/submap.hpp>
#include <map_merge_2d/cv_core/cv_core.hpp>
#include <map_merge_2d/util/tf_utils.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/transform_datatypes.h>

namespace map_merge_2d
{
    class SubMapMerger
    {
        public:
            SubMapMerger();
            SubMapMerger(std::shared_ptr<ros::NodeHandle> node, std::function<std::vector<std::shared_ptr<SubMap>> ()> submap_reader);

            void merge(void);
            bool merge(std::vector<std::shared_ptr<SubMap>> submaps);
        
        private:
            cv_core::CVImage merge_map_images(std::vector<cv_core::CVImage> images);
            void publish_map(cv_core::CVImage map, double resolution);
            void publish_map_transforms(std::vector<SubMap::Map> maps);

            std::shared_ptr<ros::NodeHandle> node_;
            tf2_ros::TransformBroadcaster tf_broadcaster_;
            ros::Publisher map_publisher_;
            std::function<std::vector<std::shared_ptr<SubMap>> ()> submap_reader_;
            ros::Timer merging_timer_;
            bool publish_merged_map_ = false;
            bool publish_tf = false;
            std::string world_frame_;
            
    }; // namespace submap_merger

} // namespace map_merge_2d



# endif // SUBMAP_MERGER_H