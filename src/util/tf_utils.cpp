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

#include <map_merge_2d/util/tf_utils.hpp>

using namespace map_merge_2d;

cv::Mat tf_utils::tf2_to_cv_transform(double resolution, tf2::Transform transform)
{
    cv::Mat cv_transform(2, 3, CV_32F, 3);

    tf2::Matrix3x3 rot = transform.getBasis();
    tf2::Vector3 origin = transform.getOrigin();
    cv_transform.at<float>(0,0) = (_Float32)rot[0][0];
    cv_transform.at<float>(0,1) = (_Float32)rot[0][1];
    cv_transform.at<float>(0,2) = (_Float32)(origin[0] / resolution);

    cv_transform.at<float>(1,0) = (_Float32)rot[1][0];
    cv_transform.at<float>(1,1) = (_Float32)rot[1][1];
    cv_transform.at<float>(1,2) = (_Float32)(origin[1] / resolution);
    
    return cv_transform;
}
        
tf2::Transform tf_utils::cv_transform_to_tf2(double resolution, cv::Mat transform)
{
    tf2::Transform tf;
    tf.setBasis(tf2::Matrix3x3( transform.at<float>(0,0), transform.at<float>(0,1), 0.0,
                                transform.at<float>(1,0), transform.at<float>(1,1), 0.0,
                                0.0,                       0.0,                       1.0));
    tf.setOrigin(tf2::Vector3( transform.at<float>(0,2) * resolution, transform.at<float>(1,2) * resolution, 0.0));
    
    return tf;
}

tf2::Transform tf_utils::pose_to_tf2(geometry_msgs::Pose pose)
{
    tf2::Transform tf;
    tf.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
    tf.setRotation(tf2::Quaternion(pose.orientation.x,
                                    pose.orientation.y,
                                    pose.orientation.z,
                                    pose.orientation.w));
    return tf;
}

tf2::Transform tf_utils::image_to_map_transform(nav_msgs::MapMetaData map_info, cv::Mat transform)
{
    return cv_transform_to_tf2(map_info.resolution, transform) * pose_to_tf2(map_info.origin).inverse();
}

cv::Mat tf_utils::map_to_image_transform(nav_msgs::MapMetaData map_info, tf2::Transform transform)
{
    return tf2_to_cv_transform(map_info.resolution, transform * pose_to_tf2(map_info.origin));
}

tf2::Transform tf_utils::get_map_origin_tf(nav_msgs::MapMetaData map_info)
{
    // Map info origin is defined in the map_frame itself
    // Hence inverse -> transform in image frame (upper left corner)
    return pose_to_tf2(map_info.origin).inverse();
}