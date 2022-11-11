#ifndef SUBMAP_MERGER_H
#define SUBMAP_MERGER_H

#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <map_merge_2d/submaps/submap.hpp>
#include <map_merge_2d/cv_core/cv_core.hpp>
#include <map_merge_2d/util/tf_utils.hpp>

namespace map_merge_2d
{
    class SubMapMerger
    {
        public:
            SubMapMerger();
            SubMapMerger(rclcpp::Node *node, std::function<std::vector<std::shared_ptr<SubMap>> ()> submap_reader);

            void merge(void);
            bool merge(std::vector<std::shared_ptr<SubMap>> submaps, nav_msgs::msg::OccupancyGrid merged_map);
        
        private:
            cv_core::CVImage merge_map_images(std::vector<cv_core::CVImage> images);
            void publish_map(cv_core::CVImage map, double resolution);

            rclcpp::Node *node_;
            rclcpp::Logger logger_;
            rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
            std::function<std::vector<std::shared_ptr<SubMap>> ()> submap_reader_;
            rclcpp::TimerBase::SharedPtr merging_timer_;
            bool publish_merged_map_ = false;
            std::string world_frame_;
            
    }; // namespace submap_merger

} // namespace map_merge_2d



# endif // SUBMAP_MERGER_H