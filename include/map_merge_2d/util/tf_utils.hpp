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

#ifndef TF_UTIL_H
#define TF_UTIL_H

#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/MapMetaData.h>

namespace map_merge_2d
{
    namespace tf_utils
    {
        cv::Mat tf2_to_cv_transform(double resolution, tf2::Transform transform);
        tf2::Transform cv_transform_to_tf2(double resolution, cv::Mat transform); 
        tf2::Transform pose_to_tf2(geometry_msgs::Pose pose);

        /**
         * @brief Given a transform for an map image in some reference frame (child frame = image upper left corner)
         *          returns the transform of map origin in the given reference frame
         * 
         * @param map_info 
         * @param transform 
         * @return tf2::Transform 
         */
        tf2::Transform image_to_map_transform(nav_msgs::MapMetaData map_info, cv::Mat transform);

        /**
         * @brief Given a transform for an map image "origin" in some reference frame (child frame = map origin)
         *          returns the transform of image left corner in the given reference frame
         * 
         * @param map_info 
         * @param transform 
         * @return cv::Mat 
         */
        cv::Mat map_to_image_transform(nav_msgs::MapMetaData map_info, tf2::Transform transform);
        
        /**
         * @brief Get the map origin tf : Transform from Image upper left corner to map frame origin
         * 
         * @param map_info 
         * @param transform 
         * @return tf2::Transform 
         */
        tf2::Transform get_map_origin_tf(nav_msgs::MapMetaData map_info);
    }
}

#endif // TF_UTIL_H