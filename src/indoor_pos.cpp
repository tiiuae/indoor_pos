/*
Convert indoor positioning data from lighthouse to PX4 positioning sensor data

*/


#include "indoor_pos/indoor_pos.hpp"
#include "geodesy/utm.h"
#include "geodesy/wgs84.h"
#include "libsurvive/survive_api.h"
#include <cmath>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <thread>
#include <chrono>

//#define INDOOR_USE_SIMULATOR

const double Pi = 3.141592654;
const uint8_t MagSensorAvgSampleCount = 12;

using std::placeholders::_1;

class IndoorPosPrivate
{
public:
    enum IndoorNodeState {
        Idle, ReqCalibrate, ReqRestart, Calibrating, Restarting, Error
    };

    int  surviveInit();
    void surviveSpin();
    void IndoorPosUpdate(SurvivePose pose, SurviveVelocity velocity);
    int  restart();
    void calcAngle(double rad_a);
    int64_t getSystemTimeUSec();

    IndoorPos *_node;
    int _update_freq;
    geodesy::UTMPoint _home;
    SurviveSimpleContext *_actx;
    int _lighthouse_count = 0;
    int64_t _msg_ts_diff = 0;
    double _north_offset = 0.0;
    double _declination = 0.0;
    int _use_mag = 0;
    uint64_t _last_timestamp = 0;

    double _angles[MagSensorAvgSampleCount];
    int _angles_idx = 0;
    double _last_angle = 0.0;
    bool _valid_angle = false;

    IndoorNodeState _node_state = IndoorNodeState::Idle;

    rclcpp::Publisher<px4_msgs::msg::SensorGps>::SharedPtr _publisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  _control_sub;
    rclcpp::Subscription<px4_msgs::msg::SensorMag>::SharedPtr  _sensorMag_sub;
};

IndoorPos::IndoorPos()
: Node("IndoorPos"),
  _impl(std::make_unique<IndoorPosPrivate>())
{
    RCLCPP_INFO(this->get_logger(), "Indoor positioning");
    _impl->_node = this;

    this->declare_parameter<double>("home_lat", 0.0);
    this->declare_parameter<double>("home_lon", 0.0);
    this->declare_parameter<double>("home_alt", 0.0);
    this->declare_parameter<int>("frequency", 10);
    this->declare_parameter<double>("north_offset", 0.0);
    this->declare_parameter<double>("declination", 0.0);
    this->declare_parameter<int>("use_mag", 0);

    double n_off = 0.0;
    auto point = geographic_msgs::msg::GeoPoint();
    this->get_parameter("home_lat", point.latitude);
    this->get_parameter("home_lon", point.longitude);
    this->get_parameter("home_alt", point.altitude);
    this->get_parameter("frequency", _impl->_update_freq);
    this->get_parameter("north_offset", n_off);
    this->get_parameter("declination", _impl->_declination);
    this->get_parameter("use_mag", _impl->_use_mag);
    if (_impl->_use_mag != 0)
        n_off = 0;
    _impl->_north_offset = (n_off / 360.0) * 2 * Pi;

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

    RCLCPP_INFO(this->get_logger(), "Update freq: %d Hz, north_offset: %.3lf deg (%lf rad), declination: %.3lf, use_mag: %d",
        _impl->_update_freq, n_off, _impl->_north_offset, _impl->_declination, _impl->_use_mag);

    for (int i=0; i<MagSensorAvgSampleCount; i++) {
        _impl->_angles[i] = 0.0;
    }
    _impl->_angles_idx = 0;

    _impl->_control_sub = this->create_subscription<std_msgs::msg::String>(
        "IndoorPos_ctrl", rclcpp::SystemDefaultsQoS(), std::bind(&IndoorPos::Control, this, _1));
    _impl->_sensorMag_sub = this->create_subscription<px4_msgs::msg::SensorMag>(
        "SensorMag_PubSubTopic", rclcpp::SystemDefaultsQoS(), std::bind(&IndoorPos::SensorMag, this, _1));

    _impl->_publisher = this->create_publisher<px4_msgs::msg::SensorGps>("SensorGps_PubSubTopic", rclcpp::SystemDefaultsQoS() );

    if (_impl->_update_freq > 0)
    {
        _spin_timer_ms = 1000 / _impl->_update_freq;
        if (_spin_timer_ms == 0)
            _spin_timer_ms = 1;

        if (_impl->surviveInit() < 0) {
            RCLCPP_ERROR(this->get_logger(), "ERROR: Survive Init failed!");
            exit(EXIT_FAILURE);
        }

    } else {
        RCLCPP_ERROR(this->get_logger(), "ERROR: update frequency must be > 0");
        exit(EXIT_FAILURE);
    }

    _initialized = true;
}

IndoorPos::~IndoorPos()
{
    RCLCPP_INFO(this->get_logger(), "Destructor");
    survive_simple_close(_impl->_actx);
}

/* Worker method is running in separate thread */
void IndoorPos::Worker()
{
    RCLCPP_INFO(this->get_logger(), "[Worker] Wait for node initialization");
    while (! _initialized && rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(_spin_timer_ms));
    }

    RCLCPP_INFO(this->get_logger(), "[Worker] Initialization done.");
    while (rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(_spin_timer_ms));
        _impl->surviveSpin();
    }
    RCLCPP_INFO(this->get_logger(), "[Worker] Exit.");
}

void IndoorPos::surviveSpinTimerCallback()
{
    _que.push(1);
}

void IndoorPos::SensorMag(const px4_msgs::msg::SensorMag::SharedPtr msg) const
{
    uint64_t curr_stamp = _impl->getSystemTimeUSec();
    _impl->_last_timestamp = msg->timestamp;
    int64_t diff = (int64_t) curr_stamp - (int64_t) _impl->_last_timestamp;
    if (_impl->_msg_ts_diff == 0 || _impl->_msg_ts_diff > diff)
    {
        _impl->_msg_ts_diff = diff;
    }
    double rad_a = -atan2(msg->y, msg->x);
    _impl->calcAngle(rad_a);
}

void IndoorPos::Control(const std_msgs::msg::String::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "Request command: '%s'", msg->data.c_str());
    if (strcmp(msg->data.c_str(), "calibrate") == 0) {
        if (_impl->_node_state == IndoorPosPrivate::IndoorNodeState::Idle) {
            RCLCPP_INFO(this->get_logger(), "Request Calibration");
            _impl->_node_state = IndoorPosPrivate::IndoorNodeState::ReqCalibrate;
        } else {
            RCLCPP_WARN(this->get_logger(), "Calibrate request not allowed");
        }
    }
    else if (strcmp(msg->data.c_str(), "restart") == 0) {
        if (_impl->_node_state == IndoorPosPrivate::IndoorNodeState::Idle) {
            RCLCPP_INFO(this->get_logger(), "Request Restart");
            _impl->_node_state = IndoorPosPrivate::IndoorNodeState::ReqRestart;
        } else {
            RCLCPP_WARN(this->get_logger(), "Restart request not allowed");
        }
    }
    else {
        RCLCPP_INFO(this->get_logger(), "Unknown command: '%s'", msg->data.c_str());
    }
}


void IndoorPosPrivate::calcAngle(double rad_a)
{
    if (rad_a <= 0)
        rad_a += 2*Pi;

    _angles[_angles_idx] = rad_a;
    if (++_angles_idx >= MagSensorAvgSampleCount) {
        _angles_idx = 0;
        _valid_angle = true;
    }

    double max_val = _angles[0];
    for (int i=0; i<MagSensorAvgSampleCount; i++)
        if(max_val < _angles[i])
            max_val = _angles[i];

    double angle = 0.0;
    for (int i=0; i<MagSensorAvgSampleCount; i++) {
        angle += _angles[i];
        if (max_val - _angles[i] > Pi)
            angle += 360.0;
    }
    angle /= MagSensorAvgSampleCount;
    if (angle > 2*Pi) angle -= 2*Pi;
    _last_angle = angle;

/*
    double deg = (angle / (2*Pi)) * 360.0;
    RCLCPP_INFO(this->_node->get_logger(), "SensorMag: angle %.15lf (%lf), noff:%lf",
        deg, angle, _north_offset);
*/
}

int IndoorPosPrivate::restart()
{
    RCLCPP_INFO(this->_node->get_logger(), "Restart survive");
    survive_simple_close(_actx);
    return surviveInit();
}

void IndoorPosPrivate::surviveSpin()
{
    if (_node_state == IndoorNodeState::Idle) {
        if (survive_simple_wait_for_update(_actx) && rclcpp::ok()) {
            for (const SurviveSimpleObject *it = survive_simple_get_next_updated(_actx); it != 0;
                it = survive_simple_get_next_updated(_actx))
            {
                SurvivePose     pose;
                SurviveVelocity velocity;
                survive_simple_object_get_latest_pose(it, &pose);
                survive_simple_object_get_latest_velocity(it, &velocity);
                IndoorPosUpdate(pose, velocity);
            }

        }
    } else if (_node_state == IndoorNodeState::ReqCalibrate) {
        RCLCPP_INFO(this->_node->get_logger(), "Start calibrate sequence..");
        _node_state = IndoorNodeState::Calibrating;
        if (restart() < 0) {
            _node_state = IndoorNodeState::Error;
            RCLCPP_ERROR(this->_node->get_logger(), "Calibrate error!");
        }
        RCLCPP_INFO(this->_node->get_logger(), ".. calibrate sequence done.");
    } else if (_node_state == IndoorNodeState::ReqRestart) {
        RCLCPP_INFO(this->_node->get_logger(), "Start restart sequence");
        _node_state = IndoorNodeState::Restarting;
        if (restart() < 0) {
            _node_state = IndoorNodeState::Error;
            RCLCPP_ERROR(this->_node->get_logger(), "Initialization error!");
        }
        RCLCPP_INFO(this->_node->get_logger(), ".. restart sequence done.");
    }
}

void IndoorPosPrivate::IndoorPosUpdate(SurvivePose pose, SurviveVelocity velocity)
{
    // Position
    geodesy::UTMPoint utm = geodesy::UTMPoint(_home);
    double x = pose.Pos[0];
    double y = pose.Pos[1];
    double z = pose.Pos[2];

    double rotated_x = x*cos(_north_offset) - y*sin(_north_offset);
    double rotated_y = x*sin(_north_offset) + y*cos(_north_offset);

    utm.easting  += rotated_x;
    utm.northing += rotated_y;
    utm.altitude += z;

    geographic_msgs::msg::GeoPoint point = toMsg(utm);

    // Velocity
    double vx = velocity.Pos[0];
    double vy = velocity.Pos[1];
    double vz = velocity.Pos[2];

    double rotated_vx = vx*cos(_north_offset) - vy*sin(_north_offset);
    double rotated_vy = vx*sin(_north_offset) + vy*cos(_north_offset);

    uint64_t timecode = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();

    double qw = pose.Rot[0];
    double qx = pose.Rot[1];
    double qy = pose.Rot[2];
    double qz = pose.Rot[3];

    double siny_cosp = 2.0*(qw*qz + qx*qy);
    double cosy_cosp = 1.0 - 2.0*(qy*qy + qz*qz);
    double heading_rad = -atan2(siny_cosp, cosy_cosp);

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

    sensor_gps.vel_m_s = sqrt(rotated_vx * rotated_vx + rotated_vy * rotated_vy);
    sensor_gps.vel_n_m_s = rotated_vy;
    sensor_gps.vel_e_m_s = rotated_vx;
    sensor_gps.vel_d_m_s = -vz;
    sensor_gps.cog_rad = atan2(rotated_vx, rotated_vy);
    sensor_gps.vel_ned_valid = 1;

    sensor_gps.time_utc_usec = _node->now().nanoseconds() / 1000ULL;

    sensor_gps.satellites_used = 16; //_lighthouse_count;
    sensor_gps.heading = heading_rad;
    sensor_gps.heading_offset = 0.0f;

    _publisher->publish(sensor_gps);
}

int64_t IndoorPosPrivate::getSystemTimeUSec() {
    timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return static_cast<int64_t>(t.tv_sec * 1000000000LL + t.tv_nsec) / 1000LL;
}

void log_fn(SurviveSimpleContext *actx, SurviveLogLevel logLevel, const char *msg) {
    actx=actx;
    logLevel=logLevel;
    fprintf(stderr, "SimpleApi: %s\n", msg);
}

int IndoorPosPrivate::surviveInit()
{
    RCLCPP_INFO(this->_node->get_logger(), "surviveInit");
    std::vector<const char*> argstring;

#ifdef INDOOR_USE_SIMULATOR
    RCLCPP_INFO(this->_node->get_logger(), "Use Simulator");
    //const char* argstring[] = {"--v", "100", "--simulator"};
    //const int argc = 3;
    argstring.push_back("--v");
    argstring.push_back("10");
    argstring.push_back("--simulator");
#endif

    if (_node_state == IndoorNodeState::Calibrating) {
        if (_use_mag && _valid_angle) {
            _north_offset = _last_angle;
        }
        argstring.push_back("--force-calibrate");
    }
    argstring.push_back("--center-on-lh0");
    argstring.push_back("1");

    const int argc = argstring.size();
    char* const* argv = (char* const*) argstring.data();
    _actx = survive_simple_init_with_logger(argc, argv, log_fn);

    if (_actx == 0) {
        // implies -help or similiar
        RCLCPP_WARN(this->_node->get_logger(), "WARNING: survive not initialized.");
        return -1;
    }

    survive_simple_start_thread(_actx);
    _node_state = IndoorNodeState::Idle;

    _lighthouse_count = 0;
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

    return 0;
}

void indoor_pos_worker(std::shared_ptr<IndoorPos> node)
{
    node->Worker();
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<IndoorPos> node = std::make_shared<IndoorPos>();
    std::thread w_thread(indoor_pos_worker, node);
    rclcpp::spin(node);
    w_thread.join();
    rclcpp::shutdown();
    return 0;
}
