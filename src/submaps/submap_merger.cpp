#include <map_merge_2d/submaps/submap_merger.hpp>

using namespace map_merge_2d;

SubMapMerger::SubMapMerger()
: logger_(rclcpp::get_logger("SubMapMerger"))
{

}

SubMapMerger::SubMapMerger(rclcpp::Node *node, std::function<std::vector<std::shared_ptr<SubMap>> ()> submap_reader)
: logger_(rclcpp::get_logger("SubMapMerger"))
{
    if (node == nullptr)
    {
        RCLCPP_ERROR_STREAM(logger_, "Invalid node[null] passed to submap merger.");
        return;
    }

    if (submap_reader == nullptr)
    {
        RCLCPP_ERROR_STREAM(logger_, "Invalid submap_reader[null] passed to submap merger.");
        return;
    }

    node_ = node;
    logger_ = node_->get_logger();
    submap_reader_ = submap_reader;

    // Declare parameters
    node_->declare_parameter("merging_rate", 0.3);
    node_->declare_parameter("publish_merged_map", true);
    node_->declare_parameter("publish_tf", true);

    // Get Parameters
    double merging_rate = node_->get_parameter("merging_rate").as_double();
    publish_merged_map_ = node_->get_parameter("publish_merged_map").as_bool();
    publish_tf_ = node_->get_parameter("publish_tf").as_bool();

    // Create timers
    merging_timer_ = node_->create_wall_timer(std::chrono::duration<double>(1.0/merging_rate),
                                std::bind((void(SubMapMerger::*)(void))&SubMapMerger::merge, this));
}

void SubMapMerger::merge(void)
{
    nav_msgs::msg::OccupancyGrid map;
    merge(submap_reader_(), map);
}

bool SubMapMerger::merge(std::vector<std::shared_ptr<SubMap>> submaps, nav_msgs::msg::OccupancyGrid merged_map)
{   
    
    // Submap vector should at least have one map
    if (submaps.size() < 1)
    {
        RCLCPP_WARN(logger_, "No submaps received!");
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
        RCLCPP_WARN(logger_, "Submaps not available for merging");
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

        cv_core::CVImage transformed_map;
        /* The map transform is inverted, because for merging, we need to remove the existing frame shift */
        // cv::Mat cv_transform = tf_utils::map_to_image_transform(map.map.info, map.transform_.inverse());
        cv::Mat cv_transform = tf_utils::tf2_to_cv_transform(
                                            map.map.info.resolution,
                                            map.transform_ * tf_utils::get_map_origin_tf(map.map.info).inverse());

        std::cout << "individual transform : " << cv_transform << std::endl;

        cv_core::image_transform(cv_core::CVImage(cv_map), transformed_map, cv_transform);
        cv_maps.emplace_back(transformed_map);
    }

    // Create merged canvas
    cv_core::CVImage merged_image = merge_map_images(cv_maps);

    // // tf printer
    // tf2::Transform mytf = transform;
    // tf2::Matrix3x3 myrot = mytf.getBasis();
    // tf2::Vector3 mytrans = mytf.getOrigin();
    // std::cout << "[" << myrot[0][0] << "," << myrot[0][1] << "," << myrot[0][2] << "\n " <<
    //                     myrot[1][0] << "," << myrot[1][1] << "," << myrot[1][2] << "\n " <<
    //                     myrot[2][0] << "," << myrot[2][1] << "," << myrot[2][2] << "]" << std::endl;

    // std::cout << "[" << mytrans[0] << "," << mytrans[1] << "," << mytrans[2] << "]" << std::endl;
    // std::cout << cv_transform << std::endl;
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
    for (auto &image : transformed_images)
    {
        // Test purpose only
        cv::Mat clone_image = image.clone();
        clone_image.setTo(254, image == 0);
        /////////

        /**
         * @brief Copy obstacles into merged image (accumulated)
         * 
         * Select obstacle pixels from merged map and submap
         * Add pixel probabilities and clamp to 100
         */
        cv::Mat merged_map_obstacles = merged_image.image.clone();
        merged_map_obstacles.setTo(0, merged_image.image > 100); // Clear unknown area
        cv::Mat sub_map_obstacles = image.clone();
        sub_map_obstacles.setTo(0, image > 100); // Clear unknown area
        merged_map_obstacles += sub_map_obstacles;
        merged_map_obstacles.setTo(100, merged_map_obstacles > 100); // Clamp probabilities to 100
        merged_map_obstacles.copyTo(merged_image.image, merged_map_obstacles > 0); // Copy obstacle values to merged map

        /**
         * @brief Copy known free area to merged map
         * 
         * if a pixel in merged map is unknown -> 255
         * and
         * if corresponding pixel in a sub map is known-free -> 0
         * Then mark merged map pixel as known-free.
         * (If any submap marks this pixel as occupied, that will take precidence)
         */
        merged_image.image.setTo(0, merged_image.image == 255 & image == 0);
    }

    cv::Mat scaled_image;
    cv::Size scale(dsize.width * 2, dsize.height * 2);
    resize(merged_image.image, scaled_image, scale, cv::INTER_LINEAR);
    scaled_image = (255 - scaled_image); // Inverting colors for visualization
    scaled_image.setTo(150, scaled_image == 0); // Changing unknown area color
    cv::imshow("merged", scaled_image);
    cv::waitKey(500);

    return merged_image;
}