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

#include <map_merge_2d/submaps/submap_matcher.hpp>

using namespace map_merge_2d;

SubMapMatcher::SubMapMatcher(MatcherOptions options)
{
    options_ = options;
}

SubMapMatcher::SubMapMatcher(std::shared_ptr<ros::NodeHandle> node, std::function<std::vector<std::shared_ptr<SubMap>> ()> submap_reader)
{
    if (node == nullptr)
    {
        ROS_ERROR_STREAM(ros::this_node::getName() << " : [SubMapMatcher] : Invalid node[null] passed to submap matcher.");
        return;
    }

    if (submap_reader == nullptr)
    {
        ROS_ERROR_STREAM(ros::this_node::getName() << " : [SubMapMatcher] : Invalid submap_reader[null] passed to submap matcher.");
        return;
    }

    node_ = node;
    submap_reader_ = submap_reader;

    double matching_rate;

    // Get Parameters
    node_->param<double>("matching_rate", matching_rate, 0.3);
    node_->param<double>("matching_confidence", options_.confidence, 0.5);
    node_->param<int>("dilation", options_.dilation, 4);
    node_->param<int>("blur_kernel_size", options_.blur_radius, 9);

    // Create timers
    matcher_timer_ = node_->createTimer(ros::Duration(1.0/matching_rate),
                                std::bind((void(SubMapMatcher::*)(void))&SubMapMatcher::match, this));
}   

void SubMapMatcher::match(void)
{
    match(submap_reader_());
}

void SubMapMatcher::match(std::vector<std::shared_ptr<SubMap>> submaps)
{
    // Remove static merge maps
    for (auto itr = submaps.begin(); itr != submaps.end(); )
    {
        if (itr->get()->static_merge) {
            itr = submaps.erase(itr);
        } else {
            itr++;
        }
    }

    // Submap vector should at least have one map
    if (submaps.size() < 1)
    {
        ROS_WARN_STREAM(ros::this_node::getName() << " : [SubMapMatcher] : No submaps available for matching");
        return;
    }

    // Read maps
    std::vector<SubMap::Map> maps;
    for (auto &submap : submaps)
    { 
        if (!submap->available)
        {
            // Abort matching. Unavailable submaps passed
            return;
        }

        maps.emplace_back(submap->get_map());
    }

    // Convert OccupancyGrids to CV Mat
    std::vector<cv::Mat> cv_maps;
    cv_maps.reserve(submaps.size());
    for (auto &map : maps)
    {
        cv_maps.emplace_back(map.map.info.height, 
                                map.map.info.width, 
                                CV_8UC1, 
                                map.map.data.data());

        if (!(cv_maps.back().cols > 0 && cv_maps.back().rows > 0))
        {
            // Empty submaps passed
            return;
        }

        /**
         *  OccupancyGrid 'unknown' areas are denoted by -1
         *  When converting OccupancyGrid to CV Mat, [signed to unsigned], 
         *  unknown areas will get converted to white regions (255). 
         */

        /**
         *  OccupancyGrid 'known free' areas are denoted by 0
         *  These areas will get converted to black regions (0). 
         */

        /* For feature mapping, we do not consider 'known free' area. Only obstacles */
        cv_maps.back().setTo(255, cv_maps.back() == 0);

        /* Obstacle Dilation for better feature extraction */
        cv::Mat kernel = cv::getStructuringElement( 0, cv::Size(options_.dilation, options_.dilation));
        cv::Mat dilated_map;
        cv::erode(cv_maps.back(), dilated_map, kernel); // Map CV image is color inverted. 
                                                        // Hence obstacle dilation is done with CV erosion
        dilated_map.copyTo(cv_maps.back());
    }
    cv_maps.shrink_to_fit();

    /* Estimate map transforms */
    std::map<int, double> transform_confidence;
    std::map<int, cv::Mat> relative_transforms = cv_core::estimateTransforms(cv_maps, 
                                                        cv_core::FeatureType::AKAZE, 
                                                        options_.confidence,
                                                        transform_confidence,
                                                        options_.blur_radius);

    /* Check if sufficient matches available. Else abort */
    if (relative_transforms.size() < 2)
    {
        ROS_INFO_STREAM_THROTTLE(10, ros::this_node::getName() << " : [SubMapMatcher] : Failed to create sufficient matches. Try reducing confidence score?");
        return;
    }

    /* Publish warning for unmatched maps */
    if (submaps.size() > relative_transforms.size())
    {
        std::string unmatched_maps_msg = "Could not estimate transforms for maps: ";
        for (uint i = 0; i < submaps.size(); i++)
        {
            if (!relative_transforms.count(i))
                unmatched_maps_msg += maps.at(i).name_ + " ";
        }
        ROS_DEBUG_STREAM(ros::this_node::getName() << " : [SubMapMatcher] : " << unmatched_maps_msg);
    } 

    /* Check if any map has known tf. If not initialize a map with identity transform */
    if (!has_known_tf(maps))
    {
        tf2::Transform initial_tf;
        initial_tf.setIdentity();
        submaps.at(relative_transforms.begin()->first)->update_transform(initial_tf);
        // Update local map transform
        maps.at(relative_transforms.begin()->first).transform_.setIdentity();
        maps.at(relative_transforms.begin()->first).known_pose = true;
    }

    /* Check for matched map with known transform */
    std::vector<int> anchor_maps;
    if (has_known_tf(maps, relative_transforms, anchor_maps))
    {
        /**
         * @note T_a  : anchor map transform in global reference frame
         *       T_cv_a : anchor map transform in CV matching reference frame (relative transform)
         *       T_cv_b : another map transform in CV matching reference frame (relative transform)
         *      
         *       To convert 'b' map to global frame,
         *          T_b = T_a * inverse(T_cv_a) * T_cv_b
         * 
         *       anchor_tf => T_a * inverse(T_cv_a)
         */
        tf2::Transform anchor_tf = maps.at(anchor_maps.front()).transform_ * 
                                    tf_utils::get_map_origin_tf(
                                        maps.at(anchor_maps.front()).map.info).inverse() *
                                    tf_utils::cv_transform_to_tf2(
                                        maps.at(anchor_maps.front()).map.info.resolution,
                                        relative_transforms.at(anchor_maps.front()));

        // Remove anchor map from relative transform list
        relative_transforms.erase(anchor_maps.front());

        for (auto &relative_transform : relative_transforms)
        {
            tf2::Transform submap_transform = anchor_tf * 
                                                tf_utils::cv_transform_to_tf2(
                                                    maps.at(relative_transform.first).map.info.resolution,
                                                    relative_transform.second).inverse() *
                                                tf_utils::get_map_origin_tf(
                                                    maps.at(relative_transform.first).map.info);
            
            double overlap = get_overlap(maps.at(anchor_maps.front()).map,
                                    maps.at(relative_transform.first).map,
                                    maps.at(anchor_maps.front()).transform_,
                                    submap_transform);

            // If submap transform known, check overlap change
            if (maps.at(relative_transform.first).known_pose)
            {
                double current_overlap = get_overlap(maps.at(anchor_maps.front()).map,
                                            maps.at(relative_transform.first).map,
                                            maps.at(anchor_maps.front()).transform_,
                                            maps.at(relative_transform.first).transform_);
                
                // If new transform results in lesser map overlap, ignore new transform
                if (overlap < current_overlap) return;
            }

            //if (transform_confidence.at(relative_transform.first) > maps.at(relative_transform.first).transform_confidence_)
            {
                double x =  submap_transform.getOrigin().getX();
                double y = submap_transform.getOrigin().getY();
                double q_z = submap_transform.getRotation().getZ();

                // if (maps.at(relative_transform.first).known_pose)
                // {
                //     // Add transform validity check using existing transfrom 
                //     // (Sudden large deviations in transforms should indicate some error)
                //     double prev_x = maps.at(relative_transform.first).transform_.getOrigin().getX();
                //     double prev_y = maps.at(relative_transform.first).transform_.getOrigin().getY();
                //     double prev_q_z = maps.at(relative_transform.first).transform_.getRotation().getZ();

                //     if ( (abs(prev_x - x) + abs(prev_y - y) > 5.0) || (abs(prev_q_z - q_z) > 1.0) )
                //     {
                //         ROS_WARN_STREAM(ros::this_node::getName() << " : [SubMapMatcher] : " <<
                //                                      "Large changes in submap transform detected. Did you set a wrong initial transform?" <<
                //                                     " Other possible causes : opencv feature matcher divergence" <<
                //                                     "\n [x, y, q.z] : " << "[" << prev_x << ", "  << prev_y << ", " << prev_q_z << "]" <<
                //                                     " to [" << x << ", "  << y << ", " << q_z << "]");
                //         return;
                //     }
                // }

                ROS_INFO_STREAM(ros::this_node::getName() << " : [SubMapMatcher] : " << "Map " << maps.at(relative_transform.first).name_ << "transform updated.\n " <<
                                                        "confidence: " << transform_confidence.at(relative_transform.first) <<
                                                        "\n overlap: " << overlap <<
                                                        "\ntransform[x, y, q.z]: [" << x << ", "  << y << ", " << q_z << "]");
                submaps.at(relative_transform.first)->update_transform(submap_transform, 
                                                                        transform_confidence.at(relative_transform.first));
            }
        }
    }
    else
    {
        std::string unlinked_anchor_msg = "Cannot find transform between existing anchor map and maps : ";
        for (auto &itr : relative_transforms)
            unlinked_anchor_msg += maps.at(itr.first).name_ + " ";
        ROS_WARN_STREAM(ros::this_node::getName() << " : [SubMapMatcher] : " << unlinked_anchor_msg);
    }
}

/**
 * @brief Calculates obstacle overlap between given two occupancy grids using two given map transforms
 * 
 * @param map1 
 * @param map2 
 * @param map1_tf 
 * @param map2_tf 
 * @return double       Overlapping obstacle pixel count
 */
double SubMapMatcher::get_overlap(nav_msgs::OccupancyGrid map1, nav_msgs::OccupancyGrid map2, tf2::Transform map1_tf, tf2::Transform map2_tf)
{
    // Occupancy grids
    std::vector<nav_msgs::OccupancyGrid> maps;
    maps.reserve(2);
    maps.push_back(map1);
    maps.push_back(map2);

    // Transforms
    std::vector<tf2::Transform> tfs;
    tfs.reserve(2);
    tfs.push_back(map1_tf);
    tfs.push_back(map2_tf);

    // CV Transforms
    std::vector<cv::Mat> cv_tfs;
    cv_tfs.reserve(maps.size());

    // Transformed maps
    std::vector<cv_core::CVImage> transformed_maps;
    transformed_maps.reserve(maps.size());

    // Convert OccupancyGrids to CV Mat
    std::vector<cv::Mat> cv_maps;
    cv_maps.reserve(maps.size());

    // Common Canvas Region
    int min_x = 0;
    int min_y = 0;
    int max_x = 0;
    int max_y = 0;

    for (uint id = 0; id < maps.size(); id++)
    {
        cv_maps.emplace_back(maps.at(id).info.height, maps.at(id).info.width, CV_8UC1, maps.at(id).data.data());

        // Calculate CV Transforms
        /* The map transform is inverted, because for merging, we need to remove the existing frame shift */
        cv_tfs.emplace_back(tf_utils::tf2_to_cv_transform(
            maps.at(id).info.resolution, tfs.at(id) * tf_utils::get_map_origin_tf(
                maps.at(id).info).inverse()));
        
        // Transform map image
        cv_core::CVImage transformed_map;
        cv_core::image_transform(cv_core::CVImage(cv_maps.at(id)), transformed_map, cv_tfs.at(id));
        transformed_maps.emplace_back(transformed_map);

        // Check map bounds
        if (min_x > transformed_map.origin.x) min_x = transformed_map.origin.x;
        if (min_y > transformed_map.origin.y) min_y = transformed_map.origin.y;
        if (max_x < (transformed_map.origin.x + transformed_map.image.cols)) max_x = (transformed_map.origin.x + transformed_map.image.cols);
        if (max_y < (transformed_map.origin.y + transformed_map.image.rows)) max_y = (transformed_map.origin.y + transformed_map.image.rows);    
    }

    cv::Size dsize(max_x - min_x, max_y - min_y);
    std::vector<cv::Mat> common_roi_maps;
    common_roi_maps.reserve(transformed_maps.size());
    
    for (auto &image : transformed_maps)
    {
        /* Translate all images by minimum offsets + individual image offset */
        /* This will align all maps to optimal positive pixel coefficient area */
        double translation_mat[] = {1, 0, (-min_x + image.origin.x), 
                                    0, 1, (-min_y + image.origin.y)};
        cv::Mat transform = cv::Mat(2, 3, CV_64FC1, translation_mat);

        cv::Mat dest;
        cv::warpAffine(image.image, dest, transform, dsize, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255)); // 255: White border
        common_roi_maps.emplace_back(dest);
    }

    // Remove unknown region from both maps
    common_roi_maps.at(0).setTo(0, common_roi_maps.at(0) == 255);
    common_roi_maps.at(1).setTo(0, common_roi_maps.at(1) == 255);

    // Calculating overlap
    cv::Mat cross_map = common_roi_maps.at(0).mul(common_roi_maps.at(1));
    cross_map.setTo(1, cross_map > 0);
    return cv::sum(cross_map)[0];
}

/**
 * @brief Check for any map with known transforms
 * 
 * @param maps 
 * @return true 
 * @return false 
 */
bool SubMapMatcher::has_known_tf(std::vector<SubMap::Map> maps)
{
    for (auto &map : maps)
    {
        if (map.known_pose)
        {
            return true;
        }
    }
    return false;
}

/**
 * @brief Checks for maps with known transforms within the given mask
 * 
 * @param maps 
 * @param mask 
 * @param idx     ids of maps with known_tf within the given mask
 * @return true 
 * @return false 
 */
bool SubMapMatcher::has_known_tf(std::vector<SubMap::Map> maps, std::map<int, cv::Mat> estimates, std::vector<int> &idx)
{
    idx.clear();

    for (int i = 0 ; i < maps.size(); i++)
    {
        if (maps.at(i).known_pose) {
            if (estimates.find(i) != estimates.end() ) {
                idx.emplace_back(i);
            }
        }
    }

    if (idx.size())
    {
        return true;
    }
    else
    {
        return false;
    }
    
}