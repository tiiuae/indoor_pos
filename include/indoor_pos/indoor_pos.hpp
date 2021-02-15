#ifndef _INDOOR_POS_HPP_
#define _INDOOR_POS_HPP_

#include <sys/types.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class IndoorPosPrivate;

class IndoorPos : public rclcpp::Node
{
public:
    IndoorPos();
    virtual ~IndoorPos();

private:
    void surviveSpinTimerCallback();
    void Control(const std_msgs::msg::String::SharedPtr msg) const;

    std::unique_ptr<IndoorPosPrivate> _impl;
    rclcpp::TimerBase::SharedPtr _survive_spin_timer;
};

#endif // _INDOOR_POS_HPP_
