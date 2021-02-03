#ifndef _INDOOR_POS_HPP_
#define _INDOOR_POS_HPP_

#include <sys/types.h>
#include <rclcpp/rclcpp.hpp>

class IndoorPosPrivate;

class IndoorPos : public rclcpp::Node
{
public:
    IndoorPos();
    virtual ~IndoorPos();

private:
    void surviveSpinTimerCallback();

    std::unique_ptr<IndoorPosPrivate> _impl;
    rclcpp::TimerBase::SharedPtr _survive_spin_timer;
};

#endif // _INDOOR_POS_HPP_
