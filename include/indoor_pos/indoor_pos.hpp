#ifndef _INDOOR_POS_HPP_
#define _INDOOR_POS_HPP_

#include <sys/types.h>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_baro.hpp>
#include <std_msgs/msg/string.hpp>
#include <queue>

class IndoorPosPrivate;

class IndoorPos : public rclcpp::Node
{
public:
    IndoorPos();
    virtual ~IndoorPos();
    void Worker();

private:
    void surviveSpinTimerCallback();
    void SensorBaro(const px4_msgs::msg::SensorBaro::SharedPtr msg) const;
    void Control(const std_msgs::msg::String::SharedPtr msg) const;

    int _spin_timer_ms = 100;
    bool _initialized = false;
    std::queue<int> _que;
    std::unique_ptr<IndoorPosPrivate> _impl;
    rclcpp::TimerBase::SharedPtr _survive_spin_timer;
};

#endif // _INDOOR_POS_HPP_
