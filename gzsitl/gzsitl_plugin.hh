#ifndef _DRONE_CAMERA_PLUGIN_HH_
#define _DRONE_CAMERA_PLUGIN_HH_

#include <string>
#include <vector>
#include <stdio.h>
#include <mavlink.h>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <mavlink_types.h>
#include <boost/array.hpp>
#include <boost/thread.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <boost/shared_ptr.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

using boost::asio::ip::udp;
class MavServer
{
  public:
    MavServer(short port);

    // Helpers
    void run();
    bool target_is_flight_ready();
    int target_get_status();

    // Messages
    bool set_mode_guided();
    bool request_takeoff(float init_alt);
    bool command_long_ack_wait(int mav_cmd, int *cmd_result);
    void data_prepare_to_send(const uint8_t *data, int data_len);
    void command_long_prepare_to_send(mavlink_command_long_t mav_cmd);
    bool recv_home_position_wait(mavlink_home_position_t *home_position);
    void guided_goto_prepare_to_send(mavlink_mission_item_t mav_waypoint);
    bool request_home_position_wait(mavlink_global_position_int_t *home_pos);
    mavlink_mission_item_t pose_to_waypoint_relative_alt(double x, double y,
                                                         double z, double yaw);
    // State Variables
    bool attitude_isnew;
    bool heartbeat_isnew;
    bool command_ack_isnew;
    bool gps_raw_int_isnew;
    bool local_pos_ned_isnew;
    bool home_position_isnew;
    bool global_pos_int_isnew;
    mavlink_attitude_t get_svar_attitude();
    mavlink_heartbeat_t get_svar_heartbeat();
    mavlink_command_ack_t get_svar_command_ack();
    mavlink_gps_raw_int_t get_svar_gps_raw_int();
    mavlink_home_position_t get_svar_home_position();
    mavlink_local_position_ned_t get_svar_local_pos_ned();
    mavlink_global_position_int_t get_svar_global_pos_int();

  private:
    // Helpers
    boost::thread t;
    boost::mutex data_to_send_access_mtx;
    boost::mutex svar_access_mtx;
    boost::mutex attitude_svar_access_mtx;
    boost::mutex local_pos_ned_svar_access_mtx;
    udp::endpoint sender_endpoint;
    boost::asio::io_service io_service;
    udp::socket socket;

    // Transport
    int data_to_send_len;
    enum { max_len = 1024 };
    uint8_t data_recv[max_len];
    uint8_t data_to_send[max_len];
    
    void send_receive();
    void handle_message(const mavlink_message_t *msg);
    void handle_send(const boost::system::error_code &error, size_t bytes_sent);
    void handle_receive(const boost::system::error_code &error,
                        size_t bytes_recvd);

    // State Variables
    mavlink_attitude_t attitude;
    mavlink_heartbeat_t heartbeat;
    mavlink_command_ack_t command_ack;
    mavlink_gps_raw_int_t gps_raw_int;
    mavlink_home_position_t home_position;
    mavlink_local_position_ned_t local_pos_ned;
    mavlink_global_position_int_t global_pos_int;
};

namespace gazebo
{

enum set_pose_u { ANGLES, POSITION };

class GAZEBO_VISIBLE DroneCameraPlugin : public ModelPlugin
{

  public:
    DroneCameraPlugin();

    bool target_has_takenoff();
    void set_global_pos_coord_system(mavlink_global_position_int_t position);
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  private:
    enum sim_state {
        INIT,
        INIT_ON_GROUND,
        INIT_AIRBORNE,
        ACTIVE_AIRBORNE,
        ACTIVE_GROUND,
        ERROR
    };

    int simstate;
    MavServer mavserver;
    physics::Link_V links;
    physics::ModelPtr model;
    physics::ModelPtr target;
    physics::JointPtr joint;
    event::ConnectionPtr update_connection;
    mavlink_global_position_int_t init_global_pos;
    common::SphericalCoordinates global_pos_coord_system;
    
    void OnUpdate();
    bool init_global_pos_is_ready();
    void OnMsg_angles(ConstVector3dPtr &_msg);
    void OnMsg_position(ConstVector3dPtr &_msg);
    math::Pose coord_gzlocal_to_mavlocal(math::Pose gzpose);
    void calculate_pose(math::Pose *pose, mavlink_attitude_t attitude,
                        mavlink_local_position_ned_t local_position);
};
}

#endif
