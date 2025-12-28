//! SPDX-FileCopyrightText: 2024 YAZAWA Kenichi <s21c1036hn@gmail.com>
//! SPDX-License-Identifier: MIT-LICENSE

#include "core.hpp"

#include <std_msgs/msg/float32.hpp>

namespace Filters
{
Derivative::Derivative() : Node("Derivative"), pub_topic_name_("pub_topic"), sub_topic_name_("sub_topic"), initialized_(false), bef_(0)
{
    this->declare_parameter("pub_topic_name", this->pub_topic_name_);
    this->declare_parameter("sub_topic_name", this->sub_topic_name_);
    this->pub_topic_name_ = this->get_parameter("pub_topic_name").as_string();
    this->sub_topic_name_ = this->get_parameter("sub_topic_name").as_string();

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

MovingAverage::MovingAverage() : Node("MovingAverage"), pub_topic_name_("pub_topic"), sub_topic_name_("sub_topic"), initialized_(false), size_(this->declare_parameter<int>("window_size", 2)), bef_(size_, 0.0)
{
    this->declare_parameter("pub_topic_name", this->pub_topic_name_);
    this->declare_parameter("sub_topic_name", this->sub_topic_name_);
    this->pub_topic_name_ = this->get_parameter("pub_topic_name").as_string();
    this->sub_topic_name_ = this->get_parameter("sub_topic_name").as_string();

    this->size_ = static_cast<std::size_t>(this->get_parameter("window_size").as_int());

    this->publisher_ = this->create_publisher<std_msgs::msg::Float32>(this->pub_topic_name_, 10);
    this->subscriber_ = this->create_subscription<std_msgs::msg::Float32>(this->sub_topic_name_, 10, std::bind(&MovingAverage::callback, this, std::placeholders::_1));
}

void MovingAverage::callback(const std_msgs::msg::Float32 & msg)
{
    while(this->bef_.size() >= this->size_)
    {
        this->bef_.pop_front();
    }
    this->bef_.push_back(msg.data);

    double sum = std::accumulate(this->bef_.begin(), this->bef_.end(), 0.0);
    std_msgs::msg::Float32 average;

    average.data = sum / this->bef_.size();

    this->publisher_->publish(average);
}
}



