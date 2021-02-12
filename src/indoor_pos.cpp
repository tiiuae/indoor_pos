/*
Convert indoor positioning data from lighthouse to PX4 positioning sensor data

*/


#include "indoor_pos/indoor_pos.hpp"
#include "geodesy/utm.h"
#include "geodesy/wgs84.h"
#include "libsurvive/survive_api.h"
#include <cmath>
#include <px4_msgs/msg/sensor_gps.hpp>

//#define INDOOR_USE_SIMULATOR

const uint64_t MaxRttSample = 1000000;

using std::placeholders::_1;

class IndoorPosPrivate
{
public:

    void surviveInit();
    void surviveSpin();
    void IndoorPosUpdate(SurvivePose pose);

    IndoorPos *_node;
    int _update_freq;
    geodesy::UTMPoint _home;
    SurviveSimpleContext *_actx;
    int _lighthouse_count = 0;
    uint64_t _start_time = 0;
    int64_t _timesync_offset_us = 0;

    rclcpp::Publisher<px4_msgs::msg::SensorGps>::SharedPtr _publisher;
    rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr  _timesync_sub;

};

IndoorPos::IndoorPos()
: Node("IndoorPos"),
  _impl(std::make_unique<IndoorPosPrivate>())
{
    RCLCPP_INFO(this->get_logger(), "Indoor positioning");
    _impl->_node = this;
    _impl->_start_time = this->now().nanoseconds();

    this->declare_parameter<double>("home_lat", 0.0);
    this->declare_parameter<double>("home_lon", 0.0);
    this->declare_parameter<double>("home_alt", 0.0);
    this->declare_parameter<int>("frequency", 10);

    auto point = geographic_msgs::msg::GeoPoint();
    this->get_parameter("home_lat", point.latitude);
    this->get_parameter("home_lon", point.longitude);
    this->get_parameter("home_alt", point.altitude);
    this->get_parameter("frequency", _impl->_update_freq);

    RCLCPP_INFO(this->get_logger(), "Home coordinates: lat: %lf, lon: %lf, alt: %lf",
        point.latitude, point.longitude, point.altitude);

    _impl->_home = geodesy::UTMPoint(point);

    RCLCPP_INFO(this->get_logger(), "east: %lf north: %lf alt: %lf  -  zone: %u%c",
        _impl->_home.easting,
        _impl->_home.northing,
        _impl->_home.altitude,
        _impl->_home.zone,
        _impl->_home.band
        );

    _impl->_timesync_sub = this->create_subscription<px4_msgs::msg::Timesync>(
        "Timesync_PubSubTopic", 10, std::bind(&IndoorPos::TimeSync, this, _1));

    _impl->_publisher = this->create_publisher<px4_msgs::msg::SensorGps>("SensorGps_PubSubTopic", 10);

    if (_impl->_update_freq > 0)
    {
        int timer_ms = 1000 / _impl->_update_freq;
        if (timer_ms == 0)
            timer_ms = 1;

        _impl->surviveInit();
        _survive_spin_timer = this->create_wall_timer(
            std::chrono::milliseconds(timer_ms),
            std::bind(&IndoorPos::surviveSpinTimerCallback, this));

    } else {
        RCLCPP_ERROR(this->get_logger(), "ERROR: update frequency must be > 0");
        exit(EXIT_FAILURE);
    }
}

IndoorPos::~IndoorPos()
{
    RCLCPP_INFO(this->get_logger(), "Destructor");
    survive_simple_close(_impl->_actx);
}

void IndoorPos::surviveSpinTimerCallback()
{
    _impl->surviveSpin();
}

void IndoorPos::TimeSync(const px4_msgs::msg::Timesync::SharedPtr msg) const
{
    if (msg->tc1 > 0) {
        // Calculate time offset between this system and the remote system, assuming RTT for
	    // the timesync packet is roughly equal both ways.
		int64_t offset_us = (int64_t)((msg->ts1 / 1000ULL) + msg->timestamp - (msg->tc1 / 1000ULL) * 2) / 2 ;

		// Calculate the round trip time (RTT) it took the timesync packet to bounce back to us from remote system
		uint64_t rtt_us = msg->timestamp - (msg->ts1 / 1000ULL);

		if (rtt_us < MaxRttSample) {	// Only use samples with low RTT
            //RCLCPP_INFO(this->get_logger(), "TimeSync! %lu, tc1:%lu, ts1:%lu, offset_us:%ld, rtt_us:%lu => SYNC", msg->timestamp, msg->tc1, msg->ts1, offset_us, rtt_us);
            _impl->_timesync_offset_us = offset_us;
        }

    }
}

void IndoorPosPrivate::surviveSpin()
{
    if (survive_simple_wait_for_update(_actx) && rclcpp::ok()) {
        for (const SurviveSimpleObject *it = survive_simple_get_next_updated(_actx); it != 0;
            it = survive_simple_get_next_updated(_actx))
        {
            SurvivePose pose;
            survive_simple_object_get_latest_pose(it, &pose);
            IndoorPosUpdate(pose);
        }

    }
}

void IndoorPosPrivate::IndoorPosUpdate(SurvivePose pose)
{
    geodesy::UTMPoint utm = geodesy::UTMPoint(_home);

    utm.easting += pose.Pos[0];
    utm.northing += pose.Pos[1];
    utm.altitude += pose.Pos[2];

    geographic_msgs::msg::GeoPoint point = toMsg(utm);

    timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    uint64_t timecode = static_cast<int64_t>(t.tv_sec * 1000000000LL + t.tv_nsec) / 1000LL;
    timecode = timecode - (_timesync_offset_us / 1000);

/*
    RCLCPP_INFO(this->_node->get_logger(), "[%lu] lat: %.15lf, lon: %.15lf, alt: %.15lf",
        timecode, point.latitude, point.longitude, point.altitude);
*/
    px4_msgs::msg::SensorGps sensor_gps;
    sensor_gps.timestamp = timecode;
    sensor_gps.lat = (uint32_t) (point.latitude  * 10000000);
    sensor_gps.lon = (uint32_t) (point.longitude * 10000000);
    sensor_gps.alt = (uint32_t) (point.altitude  * 1000);
    sensor_gps.s_variance_m_s = 0.2f;

    sensor_gps.fix_type = 3;
    sensor_gps.eph = 0.5f;
    sensor_gps.epv = 0.8f;
    sensor_gps.hdop = 0.0f;
    sensor_gps.vdop = 0.0f;

    sensor_gps.vel_n_m_s = 0.0f;
    sensor_gps.vel_e_m_s = 0.0f;
    sensor_gps.vel_d_m_s = 0.0f;
    sensor_gps.vel_ned_valid = 1;

    sensor_gps.satellites_used = 16; //_lighthouse_count;
    sensor_gps.heading = NAN;
    sensor_gps.heading_offset = 0.0f;

    _publisher->publish(sensor_gps);
}

void log_fn(SurviveSimpleContext *actx, SurviveLogLevel logLevel, const char *msg) {
    actx=actx;
    logLevel=logLevel;
    fprintf(stderr, "SimpleApi: %s\n", msg);
    
}

void IndoorPosPrivate::surviveInit()
{
    RCLCPP_INFO(this->_node->get_logger(), "surviveInit");

#ifdef INDOOR_USE_SIMULATOR
    const char* argstring[] = {"--v", "10", "--simulator"};
    const int argc = 3;
#else
    //const char* argstring[] = {"-l", "4", "--lighthouse-gen", "2"};
    //const int argc = 4;

    const char** argstring = nullptr;
    const int argc = 0;
#endif

    char* const* argv = (char* const*) argstring;
    _actx = survive_simple_init_with_logger(argc, argv, log_fn);

    if (_actx == 0) {
        // implies -help or similiar
        RCLCPP_WARN(this->_node->get_logger(), "WARNING: survive not initialized.");
        return;
    }

    survive_simple_start_thread(_actx);

    for (const SurviveSimpleObject *it = survive_simple_get_first_object(_actx); it != 0;
        it = survive_simple_get_next_object(_actx, it))
    {
        RCLCPP_INFO(this->_node->get_logger(), "Found '%s'", survive_simple_object_name(it));
        if (survive_simple_object_get_type(it) == SurviveSimpleObject_LIGHTHOUSE)
        {
            ++_lighthouse_count;
        }
	}
    RCLCPP_INFO(this->_node->get_logger(), "Lighthouse count: %d", _lighthouse_count);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<IndoorPos> node = std::make_shared<IndoorPos>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
