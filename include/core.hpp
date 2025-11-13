#ifndef SIMPLE_LIFECYCLE_NODE_HPP_
#define SIMPLE_LIFECYCLE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <deque>

namespace Filters
{
class Derivative : public rclcpp::Node
{
public:
    Derivative();

protected:

private:
    void callback(const std_msgs::msg::Float32 &);
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_;
    std::string pub_topic_name_;
    std::string sub_topic_name_;
    bool initialized_;
    float bef_;
};

class MovingAverage : public rclcpp::Node
{
public:
    MovingAverage();

protected:

private:
    void callback(const std_msgs::msg::Float32 &);
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_;
    std::string pub_topic_name_;
    std::string sub_topic_name_;
    bool initialized_;
    std::size_t size_;
    std::deque<double> bef_;
};
}

#endif

