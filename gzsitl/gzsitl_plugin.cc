#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

#include "gzsitl_plugin.hh"

// TODO: Put these definitions somewhere else.
#define DEBUG_MAVLINK false
#define DEBUG false
#define MAVPROXY_PORT 14551
#define MAVPROXY_IP "127.0.0.1"
#define INIT_POS_NUMSAMPLES 3
#define DEFAULT_TARGET_COMPONENT_ID 0
#define DEFAULT_TARGET_SYSTEM_ID 1
#define DEFAULT_SYSTEM_ID 1
#define GZSITL_TARGET_MODEL_NAME "gzsitl_target"
#define CMD_ACK_CHECK_INTERVAL_US 200000
#define CMD_ACK_CHECK_MAXTIME_MS 3000
#define TAKEOFF_AUTO true
#define TAKEOFF_INIT_ALT_M 0.5

int MavServer::target_get_status()
{
    mavlink_heartbeat_t status = get_svar_heartbeat();
    return status.system_status;
}

bool MavServer::target_is_flight_ready()
{
    // Check if GPS is locked and if system status is OK
    mavlink_heartbeat_t status = get_svar_heartbeat();
    mavlink_gps_raw_int_t gps = get_svar_gps_raw_int();

    return gps.fix_type >= 2 && (status.system_status == MAV_STATE_STANDBY ||
                                 status.system_status == MAV_STATE_ACTIVE);
}

void MavServer::run()
{
    t = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));
}

mavlink_home_position_t MavServer::get_svar_home_position()
{
    svar_access_mtx.lock();
    mavlink_home_position_t home_position_copy = home_position;
    home_position_isnew = false;
    svar_access_mtx.unlock();
    return home_position_copy;
}

mavlink_command_ack_t MavServer::get_svar_command_ack()
{
    svar_access_mtx.lock();
    mavlink_command_ack_t command_ack_copy = command_ack;
    command_ack_isnew = false;
    svar_access_mtx.unlock();
    return command_ack_copy;
}

mavlink_gps_raw_int_t MavServer::get_svar_gps_raw_int()
{
    svar_access_mtx.lock();
    mavlink_gps_raw_int_t gps_raw_int_copy = gps_raw_int;
    gps_raw_int_isnew = false;
    svar_access_mtx.unlock();
    return gps_raw_int_copy;
}

mavlink_attitude_t MavServer::get_svar_attitude()
{
    svar_access_mtx.lock();
    mavlink_attitude_t attitude_copy = attitude;
    attitude_isnew = false;
    svar_access_mtx.unlock();
    return attitude_copy;
}

mavlink_heartbeat_t MavServer::get_svar_heartbeat()
{
    svar_access_mtx.lock();
    mavlink_heartbeat_t heartbeat_copy = heartbeat;
    heartbeat_isnew = false;
    svar_access_mtx.unlock();
    return heartbeat_copy;
}

mavlink_local_position_ned_t MavServer::get_svar_local_pos_ned()
{

    svar_access_mtx.lock();
    mavlink_local_position_ned_t local_pos_int_copy = local_pos_ned;
    local_pos_ned_isnew = false;
    svar_access_mtx.unlock();
    return local_pos_int_copy;
}

mavlink_global_position_int_t MavServer::get_svar_global_pos_int()
{

    svar_access_mtx.lock();
    mavlink_global_position_int_t global_pos_int_copy = global_pos_int;
    global_pos_int_isnew = false;
    svar_access_mtx.unlock();
    return global_pos_int_copy;
}

MavServer::MavServer(short port)
    : svar_access_mtx(), io_service(),
      socket(io_service, udp::endpoint(udp::v4(), port))
{
    data_to_send_len = 0;

    // State Variables Initialization
    home_position = {0};
    command_ack = {0};
    gps_raw_int = {0};
    heartbeat = {0};
    local_pos_ned = {0};
    global_pos_int = {0};
    attitude = {0};

    home_position_isnew = false;
    command_ack_isnew = false;
    gps_raw_int_isnew = false;
    heartbeat_isnew = false;
    local_pos_ned_isnew = false;
    global_pos_int_isnew = false;
    attitude_isnew = false;

    send_receive();
}

mavlink_mission_item_t MavServer::pose_to_waypoint_relative_alt(double x,
                                                                double y,
                                                                double z,
                                                                double yaw)
{
    mavlink_mission_item_t mav_waypoint;

    mav_waypoint.param1 = 0;
    mav_waypoint.param2 = 0.01;
    mav_waypoint.param3 = 0;
    mav_waypoint.param4 = yaw;
    mav_waypoint.x = x;
    mav_waypoint.y = y;
    mav_waypoint.z = z;
    mav_waypoint.seq = 0;
    mav_waypoint.command = MAV_CMD_NAV_WAYPOINT;
    mav_waypoint.target_system = DEFAULT_TARGET_SYSTEM_ID;
    mav_waypoint.target_component = DEFAULT_TARGET_COMPONENT_ID;

    // Arducopter supports only MAV_FRAME_GLOBAL_RELATIVE_ALT.
    mav_waypoint.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
    mav_waypoint.current = 2;
    mav_waypoint.autocontinue = 0;

    return mav_waypoint;
}

void MavServer::handle_message(const mavlink_message_t *msg)
{

    svar_access_mtx.lock();

    static uint32_t prev_time_att = 0;
    static uint32_t prev_time_pos = 0;

    switch (msg->msgid) {
    case MAVLINK_MSG_ID_HOME_POSITION: {
        mavlink_msg_home_position_decode(msg, &home_position);
        home_position_isnew = true;
        break;
    }
    case MAVLINK_MSG_ID_COMMAND_ACK:
        mavlink_msg_command_ack_decode(msg, &command_ack);
        command_ack_isnew = true;
        break;
    case MAVLINK_MSG_ID_GPS_RAW_INT:
        mavlink_msg_gps_raw_int_decode(msg, &gps_raw_int);
        gps_raw_int_isnew = true;
        break;
    case MAVLINK_MSG_ID_HEARTBEAT:
        mavlink_msg_heartbeat_decode(msg, &heartbeat);
        heartbeat_isnew = true;
        break;
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        mavlink_msg_local_position_ned_decode(msg, &local_pos_ned);
        local_pos_ned_isnew = true;
        break;
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        mavlink_msg_global_position_int_decode(msg, &global_pos_int);
        global_pos_int_isnew = true;
        if (DEBUG_MAVLINK) {
            std::cout << "pos_msg_time = "
                      << global_pos_int.time_boot_ms - prev_time_pos
                      << std::endl;
            prev_time_pos = global_pos_int.time_boot_ms;
        }
        break;
    case MAVLINK_MSG_ID_ATTITUDE:
        mavlink_msg_attitude_decode(msg, &attitude);
        attitude_isnew = true;
        if (DEBUG_MAVLINK) {
            std::cout << "att_msg_time = "
                      << attitude.time_boot_ms - prev_time_att << std::endl;
            prev_time_att = attitude.time_boot_ms;
        }
        break;
    default:
        break;
    }

    svar_access_mtx.unlock();
}

bool MavServer::recv_home_position_wait(mavlink_home_position_t *home_position)
{
    std::chrono::time_point<std::chrono::system_clock> start_time, curr_time;
    start_time = std::chrono::system_clock::now();

    while (true) {
        usleep(CMD_ACK_CHECK_INTERVAL_US);

        // Check if wait time has been exceeded
        curr_time = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            curr_time - start_time);
        if (elapsed.count() > CMD_ACK_CHECK_MAXTIME_MS) {
            goto err_time_exceeded;
        }

        // Check if ack has been received
        if (home_position_isnew) {
            *home_position = get_svar_home_position();
            goto ack_ok;
        }
    }

err_time_exceeded:
    return false;

ack_ok:
    return true;
}

bool MavServer::command_long_ack_wait(int mav_cmd, int *result)
{
    std::chrono::time_point<std::chrono::system_clock> start_time, curr_time;
    start_time = std::chrono::system_clock::now();
    mavlink_command_ack_t cmd_ack;

    while (true) {
        usleep(CMD_ACK_CHECK_INTERVAL_US);

        // Check if wait time has been exceeded
        curr_time = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            curr_time - start_time);
        if (elapsed.count() > CMD_ACK_CHECK_MAXTIME_MS) {
            goto err_time_exceeded;
        }

        // Check if ack has been received
        cmd_ack = get_svar_command_ack();
        if (cmd_ack.command == mav_cmd) {
            *result = cmd_ack.result;
            goto ack_ok;
        }
    }

err_time_exceeded:
    return false;

ack_ok:
    return true;
}

bool MavServer::request_home_position_wait(
    mavlink_global_position_int_t *home_pos)
{

    mavlink_command_long_t mav_cmd;
    mavlink_home_position_t home;
    int mav_result;
    bool cmd_ack_ok;

    usleep(100000);

    mav_cmd.target_system = DEFAULT_TARGET_SYSTEM_ID;
    mav_cmd.target_component = DEFAULT_TARGET_COMPONENT_ID;
    mav_cmd.command = MAV_CMD_GET_HOME_POSITION;
    mav_cmd.confirmation = 0;
    mav_cmd.param1 = 0;
    mav_cmd.param2 = 0;
    mav_cmd.param3 = 0;
    mav_cmd.param4 = 0;
    mav_cmd.param5 = 0;
    mav_cmd.param6 = 0;
    mav_cmd.param7 = 0;

    command_long_prepare_to_send(mav_cmd);
    cmd_ack_ok = command_long_ack_wait(MAV_CMD_GET_HOME_POSITION, &mav_result);

    if (!cmd_ack_ok || mav_result != MAV_RESULT_ACCEPTED) {
        return false;
    }

    if (!recv_home_position_wait(&home)) {
        return false;
    }

    home_pos->lat = home.latitude;
    home_pos->lon = home.longitude;
    home_pos->alt = home.altitude;

    return true;
}

bool MavServer::set_mode_guided()
{
    int mav_result;
    bool cmd_ack_ok;

    usleep(1000000);

    mavlink_set_mode_t mav_cmd_set_mode;

    mav_cmd_set_mode.target_system = DEFAULT_TARGET_COMPONENT_ID;

    // Arducopter does not use the standard MAV_MODE_FLAG. It uses
    // a custom mode instead. GUIDED mode is defined as 4.
    mav_cmd_set_mode.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    mav_cmd_set_mode.custom_mode = 4; // GUIDED == 4

    mavlink_message_t mav_msg;
    static uint8_t mav_data_buffer[max_len];

    mavlink_msg_set_mode_encode(DEFAULT_SYSTEM_ID, DEFAULT_TARGET_COMPONENT_ID,
                                &mav_msg, &mav_cmd_set_mode);

    int n = mavlink_msg_to_send_buffer(mav_data_buffer, &mav_msg);
    data_prepare_to_send(mav_data_buffer, n);
    cmd_ack_ok = command_long_ack_wait(MAVLINK_MSG_ID_SET_MODE, &mav_result);

    if (!cmd_ack_ok || mav_result != MAV_RESULT_ACCEPTED) {
        return false;
    }

    return true;
}

bool MavServer::request_takeoff(float alt)
{
    mavlink_command_long_t mav_cmd;
    int mav_result;
    bool cmd_ack_ok;

    usleep(100000);

    // First Arm Throttle
    mav_cmd.target_system = DEFAULT_TARGET_SYSTEM_ID;
    mav_cmd.target_component = DEFAULT_TARGET_COMPONENT_ID;
    mav_cmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
    mav_cmd.confirmation = 0;
    mav_cmd.param1 = 1;
    mav_cmd.param2 = 0;
    mav_cmd.param3 = 0;
    mav_cmd.param4 = 0;
    mav_cmd.param5 = 0;
    mav_cmd.param6 = 0;
    mav_cmd.param7 = 0;

    command_long_prepare_to_send(mav_cmd);
    cmd_ack_ok =
        command_long_ack_wait(MAV_CMD_COMPONENT_ARM_DISARM, &mav_result);

    if (!cmd_ack_ok || mav_result != MAV_RESULT_ACCEPTED) {
        return false;
    }

    usleep(100000);

    // Then Request Takeoff
    mav_cmd.target_system = DEFAULT_TARGET_SYSTEM_ID;
    mav_cmd.target_component = DEFAULT_TARGET_COMPONENT_ID;
    mav_cmd.command = MAV_CMD_NAV_TAKEOFF;
    mav_cmd.confirmation = 0;
    mav_cmd.param1 = 0;
    mav_cmd.param2 = 0;
    mav_cmd.param3 = 0;
    mav_cmd.param4 = 0;
    mav_cmd.param5 = 0;
    mav_cmd.param6 = 0;
    mav_cmd.param7 = alt;

    command_long_prepare_to_send(mav_cmd);
    cmd_ack_ok = command_long_ack_wait(MAV_CMD_NAV_TAKEOFF, &mav_result);

    if (!cmd_ack_ok || mav_result != MAV_RESULT_ACCEPTED) {
        return false;
    }

    return true;
}

void MavServer::command_long_prepare_to_send(mavlink_command_long_t cmd)
{
    mavlink_message_t mav_msg;
    static uint8_t mav_data_buffer[max_len];

    mavlink_msg_command_long_encode(
        DEFAULT_SYSTEM_ID, DEFAULT_TARGET_COMPONENT_ID, &mav_msg, &cmd);

    int n = mavlink_msg_to_send_buffer(mav_data_buffer, &mav_msg);
    data_prepare_to_send(mav_data_buffer, n);
}

void MavServer::guided_goto_prepare_to_send(mavlink_mission_item_t mav_waypoint)
{
    mavlink_message_t mav_msg;
    static uint8_t mav_data_buffer[max_len];

    mav_waypoint.current = 2; // Set as a Guided waypoint

    mavlink_msg_mission_item_encode(DEFAULT_SYSTEM_ID,
                                    DEFAULT_TARGET_COMPONENT_ID, &mav_msg,
                                    &mav_waypoint);

    int n = mavlink_msg_to_send_buffer(mav_data_buffer, &mav_msg);
    data_prepare_to_send(mav_data_buffer, n);
}

void MavServer::data_prepare_to_send(const uint8_t *data, int data_len)
{

    data_to_send_access_mtx.lock();

    int safe_data_len =
        std::min((int)data_len, (int)(max_len - data_to_send_len));

    if (safe_data_len < data_len) {
        // Do not send incomplete messages
        return;
    }

    memcpy(&data_to_send[data_to_send_len], data,
           safe_data_len * sizeof(*data));
    data_to_send_len += safe_data_len;

    data_to_send_access_mtx.unlock();
}

void MavServer::handle_send(const boost::system::error_code &error,
                            size_t bytes_sent)
{
    send_receive();
}

void MavServer::handle_receive(const boost::system::error_code &error,
                               size_t bytes_recvd)
{
    mavlink_message_t msg;
    mavlink_status_t status;

    if (!error && bytes_recvd > 0) {
        for (unsigned int i = 0;
             i < std::min((unsigned int)bytes_recvd, (unsigned int)max_len);
             i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, data_recv[i], &msg,
                                   &status)) {
                handle_message(&msg);
            }
        }
    }
    send_receive();
}

void MavServer::send_receive()
{
    socket.async_receive_from(
        boost::asio::buffer(data_recv, max_len), sender_endpoint,
        boost::bind(&MavServer::handle_receive, this,
                    boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));

    // Check if there's data to be sent
    data_to_send_access_mtx.lock();
    if (data_to_send_len > 0) {
        socket.async_send_to(
            boost::asio::buffer(data_to_send, data_to_send_len),
            sender_endpoint,
            boost::bind(&MavServer::handle_send, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));

        data_to_send_len = 0;
    }
    data_to_send_access_mtx.unlock();
}

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(DroneCameraPlugin)

DroneCameraPlugin::DroneCameraPlugin()
    : mavserver(MAVPROXY_PORT),
      global_pos_coord_system(common::SphericalCoordinates::EARTH_WGS84)
{
    this->model = NULL;
}

void DroneCameraPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // Output the name of the model
    std::cerr << "\nThe drone_camera plugin is attach to model \""
              << _model->GetName() << "\"\n";

    // Store the model pointer for convenience
    this->model = _model;

    // Also store the target pointer
    this->target = this->model->GetWorld()->GetModel(GZSITL_TARGET_MODEL_NAME);

    // Run MavServer thread
    this->mavserver.run();

    // Set initial simulation parameters
    simstate = INIT;

    // Listen to the update event
    this->update_connection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&DroneCameraPlugin::OnUpdate, this));
}

void DroneCameraPlugin::set_global_pos_coord_system(
    mavlink_global_position_int_t position)
{
    ignition::math::Angle ref_lat =
        ignition::math::Angle(((double)position.lat / 1E7) * (M_PI / 180.0));
    ignition::math::Angle ref_lon =
        ignition::math::Angle(((double)position.lon / 1E7) * (M_PI / 180.0));

    // global_pos_coord_system.SetElevationReference(position.relative_alt /
    // 1000.0);
    global_pos_coord_system.SetElevationReference(position.alt / 1000.0);
    global_pos_coord_system.SetLatitudeReference(ref_lat);
    global_pos_coord_system.SetLongitudeReference(ref_lon);
}

bool DroneCameraPlugin::init_global_pos_is_ready()
{
    static int n = 0;

    // If vehicle is not airborne, receive Initial Position at least
    // INIT_POS_NUMSAMPLES times
    if (n < INIT_POS_NUMSAMPLES) {
        if (mavserver.global_pos_int_isnew &&
            mavserver.target_is_flight_ready()) {
            n++;
        }
        return false;
    }

    if (n == INIT_POS_NUMSAMPLES) {
        init_global_pos = mavserver.get_svar_global_pos_int();

        set_global_pos_coord_system(init_global_pos);

        n = INIT_POS_NUMSAMPLES + 1;
        std::cout << "Target Ready\n";
    }

    return true;
}

bool DroneCameraPlugin::target_has_takenoff()
{
    static bool has_takenoff_once = false;

    if (has_takenoff_once) {
        return true;
    }

    mavlink_heartbeat_t status = mavserver.get_svar_heartbeat();
    if (status.system_status == MAV_STATE_ACTIVE) {
        has_takenoff_once = true;
    }

    return has_takenoff_once;
}

void DroneCameraPlugin::OnUpdate()
{

    switch (simstate) {

    case INIT: {

        if (!mavserver.target_is_flight_ready()) {
            return;
        }

        int target_status = mavserver.target_get_status();

        if (target_status == MAV_STATE_STANDBY) {
            simstate = INIT_ON_GROUND;
        } else if (target_status == MAV_STATE_ACTIVE) {
            simstate = INIT_AIRBORNE;
        }

        return;
    }

    case INIT_AIRBORNE: {

        // Request home location and set as initial position
        mavlink_global_position_int_t home_position;
        bool home_position_ok =
            mavserver.request_home_position_wait(&home_position);

        if (home_position_ok) {
            init_global_pos = home_position;

            set_global_pos_coord_system(init_global_pos);

            simstate = ACTIVE_AIRBORNE;
        }

        return;
    }

    case INIT_ON_GROUND: {

        // Wait until initial global position is achieved
        if (!init_global_pos_is_ready()) {
            return;
        }

        // Initial takeoff if AUTOTAKEOFF and if not already on air
        if (TAKEOFF_AUTO) {
            if (!mavserver.set_mode_guided()) {
                std::cerr
                    << "Takeoff Unsuccessful - Error setting GUIDED mode.\n";
                return;
            }
            if (!mavserver.request_takeoff(TAKEOFF_INIT_ALT_M)) {
                std::cerr << "Takeoff Unsuccessful - Error taking off.\n";
                return;
            } else {
                std::cout << "Takeoff Sucessfull.\n";
                simstate = ACTIVE_AIRBORNE;
            }
        } else {
            simstate = ACTIVE_GROUND;
        }

        return;
    }

    case ACTIVE_GROUND:
    case ACTIVE_AIRBORNE: {

        // Make sure the target still exists
        if (!this->target) {
            std::cerr << "Target not found.";
            simstate = ERROR;
            return;
        }

        // Get Target Position
        math::Pose curr_pose;
        static math::Pose tpose = math::Pose(math::Pose::Zero);
        math::Pose tpose_new = this->target->GetWorldPose();
        if (target_has_takenoff() && tpose_new != tpose) {
            tpose = tpose_new;

            // Convert from Gazebo Local Coordinates to Mav Local NED
            // Coordinates
            math::Pose pose_mavlocal = coord_gzlocal_to_mavlocal(tpose_new);

            // Convert from Mav Local NED Coordinates to Global Coordinates
            math::Vector3 global_coord =
                global_pos_coord_system.SphericalFromLocal(
                    ignition::math::Vector3d(-pose_mavlocal.pos.y,
                                             -pose_mavlocal.pos.x,
                                             -pose_mavlocal.pos.z));

            // Convert from Global Coordinates to Global Coordinates with
            // Relative Alt
            global_coord.z = global_coord.z -
                             global_pos_coord_system.GetElevationReference();

            // Send target coordinates through mavlink
            mavserver.guided_goto_prepare_to_send(
                mavserver.pose_to_waypoint_relative_alt(
                    global_coord.x, global_coord.y, global_coord.z,
                    pose_mavlocal.rot.GetYaw()));
        }

        // Calculate pose according to new attitude and position
        if (mavserver.local_pos_ned_isnew || mavserver.attitude_isnew) {
            calculate_pose(&curr_pose, mavserver.get_svar_attitude(),
                           mavserver.get_svar_local_pos_ned());

            // Set New Drone Pose in Gazebo
            this->model->SetWorldPose(curr_pose);
        }

        return;
    }

    case ERROR:
    default:
        break;
    }

    return;
}

math::Pose DroneCameraPlugin::coord_gzlocal_to_mavlocal(math::Pose gzpose)
{
    return math::Pose(gzpose.pos.x, -gzpose.pos.y, -gzpose.pos.z,
                      gzpose.rot.GetRoll(), -gzpose.rot.GetPitch(),
                      -gzpose.rot.GetYaw());
}

void DroneCameraPlugin::calculate_pose(
    math::Pose *pose, mavlink_attitude_t attitude,
    mavlink_local_position_ned_t local_position)
{
    pose->Set(local_position.x, -local_position.y, -local_position.z,
              attitude.roll, -attitude.pitch, -attitude.yaw);
}
