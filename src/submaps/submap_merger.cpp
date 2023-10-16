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

#include <map_merge_2d/submaps/submap_merger.hpp>

using namespace map_merge_2d;

SubMapMerger::SubMapMerger()
{

}

SubMapMerger::SubMapMerger(std::shared_ptr<ros::NodeHandle> node, std::function<std::vector<std::shared_ptr<SubMap>> ()> submap_reader)
{
    if (node == nullptr)
    {
        ROS_ERROR_STREAM(ros::this_node::getName() << " : [SubMapMerger] : Invalid node[null] passed to submap merger.");
        return;
    }

    if (submap_reader == nullptr)
    {
        ROS_ERROR_STREAM(ros::this_node::getName() << " : [SubMapMerger] : Invalid submap_reader[null] passed to submap merger.");
        return;
    }

    node_ = node;
    submap_reader_ = submap_reader;

    double merging_rate;
    std::string merged_map_topic;

    // Get Parameters
    node_->param<double>("merging_rate", merging_rate, 0.3);
    node_->param<bool>("publish_merged_map", publish_merged_map_, true);
    node_->param<bool>("publish_tf", publish_tf, true);
    node_->param<std::string>("merged_map_topic", merged_map_topic, "merged_map");
    node_->param<std::string>("world_frame", world_frame_, "world");

    // Create publisher
    map_publisher_ = node->advertise<nav_msgs::OccupancyGrid>(merged_map_topic, 10);

    // Create timers
    merging_timer_ = node_->createTimer(ros::Duration(1.0/merging_rate),
                                std::bind((void(SubMapMerger::*)(void))&SubMapMerger::merge, this));
}

void SubMapMerger::merge(void)
{
    merge(submap_reader_());
}

bool SubMapMerger::merge(std::vector<std::shared_ptr<SubMap>> submaps)
{   
    
    // Submap vector should at least have one map
    if (submaps.size() < 1)
    {
        ROS_WARN_STREAM(ros::this_node::getName() << " : [SubMapMerger] : No submaps received!");
        return false;
    }

    // Read available maps with known transforms
    std::vector<SubMap::Map> maps;
    for (auto &submap : submaps)
    { 
        if (submap->available)
        {
            SubMap::Map _map_ = submap->get_map();
            if (_map_.known_pose)
                maps.emplace_back(_map_);
        }
    }

    // If no map has transforms, end merge
    if (!maps.size())
    {
        ROS_WARN_STREAM(ros::this_node::getName() << " : [SubMapMerger] : Submaps not available for merging!");
        return false;
    }

    // Convert OccupancyGrids to CV Mat and transform the maps
    std::vector<cv_core::CVImage> cv_maps;
    for (auto &map : maps)
    {
        cv::Mat cv_map = cv::Mat(map.map.info.height,
                                    map.map.info.width,
                                    CV_8UC1,
                                    map.map.data.data());
        
        // Check if converted maps are valid
        if (! (cv_map.rows > 0 && cv_map.cols > 0))
        {
            return false;
        }

        cv_core::CVImage transformed_map;
        /* The map transform is inverted, because for merging, we need to remove the existing frame shift */
        // cv::Mat cv_transform = tf_utils::map_to_image_transform(map.map.info, map.transform_.inverse());
        cv::Mat cv_transform = tf_utils::tf2_to_cv_transform(
                                            map.map.info.resolution,
                                            map.transform_ * tf_utils::get_map_origin_tf(map.map.info).inverse());

        cv_core::image_transform(cv_core::CVImage(cv_map), transformed_map, cv_transform);
        cv_maps.emplace_back(transformed_map);
    }

    // Create merged canvas
    cv_core::CVImage merged_image = merge_map_images(cv_maps);

    // Publish tf
    if (publish_tf)
    {
        publish_map_transforms(maps);
    }

    // Currently supports same resolution maps only.
    /* TODO support multiresolution maps */
    if (publish_merged_map_)
    {
        publish_map(merged_image, maps.front().map.info.resolution);
    }
    

    // // tf printer
    // tf2::Transform mytf = transform;
    // tf2::Matrix3x3 myrot = mytf.getBasis();
    // tf2::Vector3 mytrans = mytf.getOrigin();
    // std::cout << "[" << myrot[0][0] << "," << myrot[0][1] << "," << myrot[0][2] << "\n " <<
    //                     myrot[1][0] << "," << myrot[1][1] << "," << myrot[1][2] << "\n " <<
    //                     myrot[2][0] << "," << myrot[2][1] << "," << myrot[2][2] << "]" << std::endl;

    // std::cout << "[" << mytrans[0] << "," << mytrans[1] << "," << mytrans[2] << "]" << std::endl;
    // std::cout << cv_transform << std::endl;
    return true;
}

cv_core::CVImage SubMapMerger::merge_map_images(std::vector<cv_core::CVImage> images)
{
    double min_x = images.front().origin.x;
    double min_y = images.front().origin.y;
    double max_x = images.front().origin.x + images.front().image.cols;
    double max_y = images.front().origin.y + images.front().image.rows;
    for (auto &image : images)
    {
        /* Find minimum map image coordinates [x,y] */
        if (image.origin.x < min_x) min_x = image.origin.x;
        if (image.origin.y < min_y) min_y = image.origin.y;

        /* Find maximum image coordinates */
        if ( (image.origin.x + image.image.cols) > max_x ) max_x = image.origin.x + image.image.cols;
        if ( (image.origin.y + image.image.rows) > max_y ) max_y = image.origin.y + image.image.rows;
    }

    /* Transform images */
    std::vector<cv::Mat> transformed_images;
    cv::Size dsize(max_x - min_x, max_y - min_y);

    for (auto &image : images)
    {
        /* Translate all images by minimum offsets + individual image offset */
        /* This will align all maps to optimal positive pixel coefficient area */
        double translation_mat[] = {1, 0, (-min_x + image.origin.x), 
                                    0, 1, (-min_y + image.origin.y)};
        cv::Mat transform = cv::Mat(2, 3, CV_64FC1, translation_mat);

        cv::Mat dest;
        cv::warpAffine(image.image, dest, transform, dsize, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255)); // 255: White border
        transformed_images.emplace_back(dest);
    }
    
    /* Merge all images into one canvas */
    cv_core::CVImage merged_image(cv::Mat(dsize.height, dsize.width, CV_8UC1, 255), min_x, min_y);
    cv::Mat merged_map_obstacles = merged_image.image.clone();
    merged_map_obstacles.setTo(0, merged_image.image > 100); // Clear unknown area
    for (auto &image : transformed_images)
    {
        /**
         * @brief Copy obstacles into merged image (accumulated)
         * 
         * Select obstacle pixels from merged map and submap
         * Add pixel probabilities and clamp to 100
         */
        cv::Mat sub_map_obstacles = image.clone();
        sub_map_obstacles.setTo(0, image > 100); // Clear unknown area
        merged_map_obstacles += sub_map_obstacles;
        merged_map_obstacles.setTo(100, merged_map_obstacles > 100); // Clamp probabilities to 100
        /**
         * @brief Copy known free area to merged map
         * 
         * if a pixel in merged map is unknown -> 255
         * and
         * if corresponding pixel in a sub map is known-free -> 0
         * Then mark merged map pixel as known-free.
         * (If any submap marks this pixel as occupied, that will take precidence)
         */
        merged_image.image.setTo(0,  image == 0); //merged_image.image == 255 &
    }
    merged_map_obstacles.copyTo(merged_image.image, merged_map_obstacles > 0); // Copy obstacle values to merged map

    // for (auto &image : transformed_images)
    // {
    //     /**
    //      * @brief Copy  all known free areas to merged map
    //      * 
    //      * if corresponding pixel in a sub map is known-free -> 0
    //      * Then mark merged map pixel as known-free.
    //      */
    //     merged_image.image.setTo(0,  image == 0); 
    // }

    // cv::Mat scaled_image;
    // cv::Size scale(dsize.width * 3, dsize.height * 3);
    // resize(merged_image.image, scaled_image, scale, cv::INTER_LINEAR);
    // scaled_image = (255 - scaled_image); // Inverting colors for visualization
    // scaled_image.setTo(150, scaled_image == 0); // Changing unknown area color
    // cv::imshow("merged", scaled_image);
    // cv::waitKey(500);

    return merged_image;
}

void SubMapMerger::publish_map(cv_core::CVImage map, double resolution)
{
    nav_msgs::OccupancyGrid map_msg;
    map_msg.header.frame_id = world_frame_;
    map_msg.header.stamp = ros::Time::now();
    map_msg.info.height = map.image.rows;
    map_msg.info.width = map.image.cols;
    map_msg.info.resolution = resolution;
    map_msg.info.origin.position.x = map.origin.x * resolution;
    map_msg.info.origin.position.y = map.origin.y * resolution;
    map_msg.data.assign(map.image.begin<int8_t>(), map.image.end<int8_t>());
    map_publisher_.publish(map_msg);
}

void SubMapMerger::publish_map_transforms(std::vector<SubMap::Map> maps)
{
    for (auto &map : maps)
    {
        geometry_msgs::TransformStamped transform;
        transform.header.frame_id = world_frame_;
        transform.header.stamp = ros::Time::now();
        transform.child_frame_id = map.name_ + "/map";

        tf2::Vector3 origin = map.transform_.getOrigin();
        tf2::Quaternion rotation = map.transform_.getRotation();
        transform.transform.translation.x = origin.getX();
        transform.transform.translation.y = origin.getY();
        transform.transform.translation.z = origin.getZ();

        transform.transform.rotation.x = rotation.getX();
        transform.transform.rotation.y = rotation.getY();
        transform.transform.rotation.z = rotation.getZ();
        transform.transform.rotation.w = rotation.getW();

	    double q_sum = transform.transform.rotation.x * transform.transform.rotation.x + 
                        transform.transform.rotation.y * transform.transform.rotation.y + 
                        transform.transform.rotation.z * transform.transform.rotation.z + 
                        transform.transform.rotation.w * transform.transform.rotation.w;
        double q_norm = sqrt(q_sum);
        transform.transform.rotation.x /= q_norm;
        transform.transform.rotation.y /= q_norm;
        transform.transform.rotation.z /= q_norm;
        transform.transform.rotation.w /= q_norm;
        
        tf_broadcaster_.sendTransform(transform);
    }
}