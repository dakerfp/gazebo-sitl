#ifndef _DRONE_CAMERA_PLUGIN_HH_
#define _DRONE_CAMERA_PLUGIN_HH_

#include <thread>
#include <mutex>
#include <mavlink.h>
#include <sys/socket.h>
#include <mavlink_types.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

class MavServer
{
  public:
    MavServer(short port);
    virtual ~MavServer();

    // Helpers
    void run();
    bool heartbeat_is_time_to_send();
    mavlink_mission_item_t pose_to_waypoint_relative_alt(double x, double y,
                                                         double z, double yaw);

    // Vehicle Communication
    bool vehicle_is_ready();
    int vehicle_get_status();
    bool vehicle_set_mode_guided();
    void vehicle_send_our_heartbeat();
    void vehicle_send_data(const uint8_t *data, int data_len);
    void vehicle_send_cmd_long(mavlink_command_long_t mav_cmd);
    void vehicle_send_waypoint(mavlink_mission_item_t mav_waypoint);
    bool vehicle_cmd_long_ack_recvd(int mav_cmd_id, int mav_result_expected);
    bool vehicle_send_cmd_long_until_ack(int cmd, float p1, float p2, float p3,
                                         float p4, float p5, float p6, float p7,
                                         int timeout);

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
    // Threading
    bool send_recv_thread_run;
    std::thread send_recv_thread;

    std::mutex svar_access_mtx;
    std::mutex data_to_send_access_mtx;
    std::mutex attitude_svar_access_mtx;
    std::mutex local_pos_ned_svar_access_mtx;

    // Vehicle Communication
    int sock;
    socklen_t fromlen;
    struct sockaddr_in local_addr;
    struct sockaddr_in remote_addr;

    int data_to_send_len;
    enum { BUFFER_LEN = 2041 };
    uint8_t data_recv[BUFFER_LEN];
    uint8_t data_to_send[BUFFER_LEN];

    void send_recv();
    void handle_send();
    void handle_recv();
    void handle_message(const mavlink_message_t *msg);

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

class GAZEBO_VISIBLE DroneCameraPlugin : public ModelPlugin
{

  public:
    DroneCameraPlugin();
    virtual ~DroneCameraPlugin();

    void OnUpdate();
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  private:
    // Simulation State
    int simstate;
    enum sim_state {
        INIT,
        INIT_ON_GROUND,
        INIT_AIRBORNE,
        ACTIVE_AIRBORNE,
        ACTIVE_ON_GROUND,
        ERROR
    };

    // Vehicle Status
    bool vehicle_is_flying();
    bool vehicle_ground_pos_locked();

    // Coordinates
    mavlink_global_position_int_t init_global_pos;
    common::SphericalCoordinates global_pos_coord_system;

    mavlink_global_position_int_t
    home_pos_to_global(mavlink_home_position_t home);
    math::Pose coord_gzlocal_to_mavlocal(math::Pose gzpose);
    void set_global_pos_coord_system(mavlink_global_position_int_t position);
    void calculate_pose(math::Pose *pose, mavlink_attitude_t attitude,
                        mavlink_local_position_ned_t local_position);

    // Mavlink
    MavServer mavserver;

    // Gazebo
    physics::ModelPtr model;
    physics::ModelPtr target;
    event::ConnectionPtr update_connection;
};
}

#endif
