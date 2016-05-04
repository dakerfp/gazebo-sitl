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

#if DEBUG_MAVLINK
#define print_debug_mav(...) printf(__VA_ARGS__)
#else
#define print_debug_mav(...)                                                   \
    do {                                                                       \
    } while (0)
#endif

#if DEBUG_STATE
#define print_debug_state(...) printf(__VA_ARGS__)
#else
#define print_debug_state(...)                                                 \
    do {                                                                       \
    } while (0)
#endif

class MavServer
{
  public:
    MavServer(short port);
    virtual ~MavServer();

    // Helpers
    void run();
    int target_get_status();
    bool target_is_flight_ready();

    // Messages
    bool set_mode_guided();
    bool heartbeat_is_time_to_send();
    void heartbeat_prepare_to_send();
    bool request_takeoff(float init_alt);
    bool command_long_ack_wait(int mav_cmd, int *cmd_result);
    void data_prepare_to_send(const uint8_t *data, int data_len);
    void command_long_prepare_to_send(mavlink_command_long_t mav_cmd);
    void guided_goto_prepare_to_send(mavlink_mission_item_t mav_waypoint);
    void request_home_position(mavlink_global_position_int_t *home_pos);
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
    std::thread send_receive_thread;
    bool send_receive_thread_run;
    std::mutex svar_access_mtx;
    std::mutex data_to_send_access_mtx;
    std::mutex attitude_svar_access_mtx;
    std::mutex local_pos_ned_svar_access_mtx;

    // Transport
    int data_to_send_len;
    enum { BUFFER_LEN = 2041 };
    uint8_t data_recv[BUFFER_LEN];
    uint8_t data_to_send[BUFFER_LEN];

    static int sock;
    socklen_t fromlen;
    static struct sockaddr_in local_addr;
    static struct sockaddr_in remote_addr;

    void send_receive();
    void handle_send();
    void handle_receive();
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

    bool target_has_takenoff();
    void set_global_pos_coord_system(mavlink_global_position_int_t position);
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  private:
    enum sim_state {
        INIT,
        INIT_ON_GROUND,
        INIT_AIRBORNE,
        ACTIVE_AIRBORNE,
        ACTIVE_ON_GROUND,
        ERROR
    };

    int simstate;
    MavServer mavserver;
    physics::ModelPtr model;
    physics::ModelPtr target;
    event::ConnectionPtr update_connection;
    mavlink_global_position_int_t init_global_pos;
    common::SphericalCoordinates global_pos_coord_system;

    void OnUpdate();
    bool init_global_pos_is_ready();
    mavlink_global_position_int_t
    home_pos_to_global(mavlink_home_position_t home);
    math::Pose coord_gzlocal_to_mavlocal(math::Pose gzpose);
    void calculate_pose(math::Pose *pose, mavlink_attitude_t attitude,
                        mavlink_local_position_ned_t local_position);
};
}

#endif
