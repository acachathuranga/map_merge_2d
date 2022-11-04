#ifndef SUBMAP_H
#define SUBMAP_H

#include <string>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <map_merge_2d/util/topic_name_utils.hpp>

namespace map_merge_2d
{
    class SubMap
    {
        public:
            struct Map
            {
                nav_msgs::msg::OccupancyGrid map;
                bool known_pose;
                tf2::Transform transform_;
            };

            SubMap(rclcpp::Node *node, std::string map_topic);

            Map get_map(void);
            void update_transform(tf2::Transform transform);

        private:
            void update_map(nav_msgs::msg::OccupancyGrid msg);

            bool known_pose_;
            tf2::Transform transform_;
            rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscriber_;
            mutable std::shared_mutex mutex_;
            nav_msgs::msg::OccupancyGrid map_;

};

}  // namespace map_merge_2d

#endif  // SUBMAP_H