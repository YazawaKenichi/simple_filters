//! SPDX-FileCopyrightText: 2024 YAZAWA Kenichi <s21c1036hn@gmail.com>
//! SPDX-License-Identifier: MIT-LICENSE

#include "simple_filters/core.hpp"

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>

#include <numeric>
#include <cmath>
#include <limits>

namespace Filters
{
Caster::Caster() : Node("Caster"), pub_topic_name_("pub_topic"), sub_topic_name_("sub_topic")
{
    this->declare_parameter("pub_topic_name", this->pub_topic_name_);
    this->declare_parameter("sub_topic_name", this->sub_topic_name_);
    this->pub_topic_name_ = this->get_parameter("pub_topic_name").as_string();
    this->sub_topic_name_ = this->get_parameter("sub_topic_name").as_string();

    RCLCPP_INFO(this->get_logger(), "sub_topic_name: %s, pub_topic_name: %s", this->sub_topic_name_.c_str(), this->pub_topic_name_.c_str());

    this->publisher_ = this->create_publisher<std_msgs::msg::Float32>(this->pub_topic_name_, 10);
    this->subscriber_ = this->create_subscription<std_msgs::msg::Int32>(this->sub_topic_name_, 10, std::bind(&Caster::callback, this, std::placeholders::_1));
}

void Caster::callback(const std_msgs::msg::Int32 & msg)
{
    std_msgs::msg::Float32 pub_msg_;
    pub_msg_.data = static_cast<float>(msg.data);
    this->publisher_->publish(pub_msg_);
}

Derivative::Derivative() : Node("Derivative"), pub_topic_name_("pub_topic"), sub_topic_name_("sub_topic"), initialized_(false), bef_(0)
{
    this->declare_parameter("pub_topic_name", this->pub_topic_name_);
    this->declare_parameter("sub_topic_name", this->sub_topic_name_);
    this->pub_topic_name_ = this->get_parameter("pub_topic_name").as_string();
    this->sub_topic_name_ = this->get_parameter("sub_topic_name").as_string();

    RCLCPP_INFO(this->get_logger(), "sub_topic_name: %s, pub_topic_name: %s", this->sub_topic_name_.c_str(), this->pub_topic_name_.c_str());

    this->publisher_ = this->create_publisher<std_msgs::msg::Float32>(this->pub_topic_name_, 10);
    this->subscriber_ = this->create_subscription<std_msgs::msg::Float32>(this->sub_topic_name_, 10, std::bind(&Derivative::callback, this, std::placeholders::_1));
}

void Derivative::callback(const std_msgs::msg::Float32 & msg)
{
    float now = msg.data;

    if(this->initialized_)
    {
        float diff = now - this->bef_;

        std_msgs::msg::Float32 pub_msg_;
        pub_msg_.data = diff;
        this->publisher_->publish(pub_msg_);
    }
    else
    {
        this->initialized_ = true;
    }

    this->bef_ = now;
}

Average::Average() : Node("Average"), pub_topic_name_("pub_topic"), sub_topic_name_("sub_topic"), initialized_(false), size_(this->declare_parameter<int>("window_size", 2)), bef_(size_, 0.0)
{
    this->declare_parameter("pub_topic_name", this->pub_topic_name_);
    this->declare_parameter("sub_topic_name", this->sub_topic_name_);
    this->pub_topic_name_ = this->get_parameter("pub_topic_name").as_string();
    this->sub_topic_name_ = this->get_parameter("sub_topic_name").as_string();

    this->size_ = static_cast<std::size_t>(this->get_parameter("window_size").as_int());

    RCLCPP_INFO(this->get_logger(), "sub_topic_name: %s, pub_topic_name: %s", this->sub_topic_name_.c_str(), this->pub_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "window_size: %ld", this->size_);

    this->publisher_ = this->create_publisher<std_msgs::msg::Float32>(this->pub_topic_name_, 10);
    this->subscriber_ = this->create_subscription<std_msgs::msg::Float32>(this->sub_topic_name_, 10, std::bind(&Average::callback, this, std::placeholders::_1));
}

float Average::calculate()
{
    double sum = std::accumulate(this->bef_.begin(), this->bef_.end(), 0.0);
    std_msgs::msg::Float32 average;

    return (float) sum / this->bef_.size();
}

void Average::callback(const std_msgs::msg::Float32 & msg)
{
    while(this->bef_.size() >= this->size_)
    {
        this->bef_.pop_front();
    }
    this->bef_.push_back(msg.data);

    std_msgs::msg::Float32 average;
    average.data = this->calculate();

    this->publisher_->publish(average);
}

Median::Median() : Node("Median"), pub_topic_name_("pub_topic"), sub_topic_name_("sub_topic"), initialized_(false), size_(this->declare_parameter<int>("window_size", 2)), bef_(size_, 0.0)
{
    this->declare_parameter("pub_topic_name", this->pub_topic_name_);
    this->declare_parameter("sub_topic_name", this->sub_topic_name_);
    this->pub_topic_name_ = this->get_parameter("pub_topic_name").as_string();
    this->sub_topic_name_ = this->get_parameter("sub_topic_name").as_string();

    this->size_ = static_cast<std::size_t>(this->get_parameter("window_size").as_int());

    RCLCPP_INFO(this->get_logger(), "sub_topic_name: %s, pub_topic_name: %s", this->sub_topic_name_.c_str(), this->pub_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "window_size: %ld", this->size_);

    this->publisher_ = this->create_publisher<std_msgs::msg::Float32>(this->pub_topic_name_, 10);
    this->subscriber_ = this->create_subscription<std_msgs::msg::Float32>(this->sub_topic_name_, 10, std::bind(&Median::callback, this, std::placeholders::_1));
}

float Median::calculate()
{
    std::vector<double> sorted(this->bef_.begin(), this->bef_.end());
    std::sort(sorted.begin(), sorted.end());
    const std::size_t n = sorted.size();
    if(n ==0)
    {
        return std::numeric_limits<float>::quiet_NaN();
    }
    else if(n % 2 == 1)
    {
        return static_cast<float>(sorted[n / 2]);
    }
    else
    {
        return static_cast<float>((sorted[n / 2 - 1] + sorted[n / 2]) / 2.0f);
    }
    return std::numeric_limits<float>::quiet_NaN();
}

void Median::callback(const std_msgs::msg::Float32 & msg)
{
    while(this->bef_.size() >= this->size_)
    {
        this->bef_.pop_front();
    }
    this->bef_.push_back(msg.data);

    std_msgs::msg::Float32 median;
    median.data = this->calculate();

    this->publisher_->publish(median);
}

Distributed::Distributed() : Node("Distributed"), pub_topic_name_("pub_topic"), sub_topic_name_("sub_topic"), initialized_(false), size_(this->declare_parameter<int>("window_size", 2)), bef_(size_, 0.0)
{
    this->declare_parameter("pub_topic_name", this->pub_topic_name_); this->declare_parameter("sub_topic_name", this->sub_topic_name_);
    this->pub_topic_name_ = this->get_parameter("pub_topic_name").as_string();
    this->sub_topic_name_ = this->get_parameter("sub_topic_name").as_string();

    this->size_ = static_cast<std::size_t>(this->get_parameter("window_size").as_int());

    RCLCPP_INFO(this->get_logger(), "sub_topic_name: %s, pub_topic_name: %s", this->sub_topic_name_.c_str(), this->pub_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "window_size: %ld", this->size_);

    this->publisher_ = this->create_publisher<std_msgs::msg::Float32>(this->pub_topic_name_, 10);
    this->subscriber_ = this->create_subscription<std_msgs::msg::Float32>(this->sub_topic_name_, 10, std::bind(&Distributed::callback, this, std::placeholders::_1));
}

float Distributed::calculate()
{
    const std::size_t n = this->bef_.size();
    if (n == 0)
    {
        return std::numeric_limits<float>::quiet_NaN();
    }
    double mean = std::accumulate(this->bef_.begin(), this->bef_.end(), 0.0f) / static_cast<double>(n);
    double var = 0.0f;
    for(double x : this->bef_)
    {
        const double d = x - mean;
        var += d * d;
    }
    var /= static_cast<double>(n);
    return static_cast<float>(std::sqrt(var));
}

void Distributed::callback(const std_msgs::msg::Float32 & msg)
{
    while(this->bef_.size() >= this->size_)
    {
        this->bef_.pop_front();
    }
    this->bef_.push_back(msg.data);

    std_msgs::msg::Float32 distributed;
    distributed.data = this->calculate();

    this->publisher_->publish(distributed);
}
}



