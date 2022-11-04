#include <map_merge_2d/map_merger.hpp>

using namespace map_merge_2d;

MapMerger::MapMerger() : Node("map_merge_2d"), count_(0)
{
  // Declare parameters
  this->declare_parameter("discovery_rate", 0.5);
  this->declare_parameter("map_topic", "map");
  this->declare_parameter("map_namespace", "");
  this->declare_parameter("merged_map_topic", "map");

  TopicDiscovery::TopicInfo info;
  info.topic_name = this->get_parameter("map_topic").as_string();
  info.topic_namespace = this->get_parameter("map_namespace").as_string();
  info.exclusions.emplace_back(ros_names::append(this->get_name(), 
                                                this->get_parameter("merged_map_topic").as_string()));
  info.exclusions.emplace_back("/map_merge/map"); // TODO remove. Testing only
  info.topic_type = "nav_msgs/msg/OccupancyGrid";
  info.discovery_rate = this->get_parameter("discovery_rate").as_double();

  // start topic discovery
  map_discovery_ = std::make_shared<TopicDiscovery>(this, info, 
                            std::bind(&MapMerger::topic_discovery_callback, this, std::placeholders::_1));

  
  //////////////// TODO Delete
  publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  // timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MapMerger::timer_callback, this));
}

void MapMerger::topic_discovery_callback(std::string topic_name)
{
  std::unique_lock lock(submap_mutex_);
  submaps_.emplace_back(std::make_shared<SubMap>(this, topic_name));
}

void MapMerger::timer_callback()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}
