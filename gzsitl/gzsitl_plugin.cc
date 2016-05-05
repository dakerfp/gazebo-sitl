#include "gzsitl_plugin.hh"

// TODO: Put these definitions somewhere else.
#define DEBUG_STATE true
#define DEBUG_MAVLINK false
#define GZSITL_TARGET_MODEL_NAME "gzsitl_target"
#define MAVPROXY_IP "127.0.0.1"
#define MAVPROXY_PORT 14556
#define LOCAL_PORT 14550
#define DEFAULT_TARGET_COMPONENT_ID 0
#define DEFAULT_TARGET_SYSTEM_ID 1
#define DEFAULT_SYSTEM_ID 1
#define HEARTBEAT_SEND_INTERVAL_MS 1000
#define CMD_ACK_CHECK_INTERVAL_US 200000
#define CMD_ACK_CHECK_MAXTIME_MS 3000
#define INIT_POS_NUMSAMPLES 3
#define TAKEOFF_AUTO true
#define TAKEOFF_INIT_ALT_M 0.5
#define HOME_POSITION_REQUEST_INTERVAL_MS 500
#define MODE_SET_REQUEST_INTERVAL_MS 500
#define TAKEOFF_REQUEST_INTERVAL_MS 500

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

MavServer::MavServer(short port)
    : svar_access_mtx(), data_to_send_access_mtx(), attitude_svar_access_mtx(),
      local_pos_ned_svar_access_mtx()
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

    // Socket Initialization
    sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;
    local_addr.sin_port = htons(port);

    if (-1 ==
        bind(sock, (struct sockaddr *)&local_addr, sizeof(struct sockaddr))) {
        perror("error bind failed");
        close(sock);
        exit(EXIT_FAILURE);
    }

    /* Attempt to make it non blocking */
    if (fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
        fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
        close(sock);
        exit(EXIT_FAILURE);
    }

    remote_addr.sin_family = AF_INET;
    remote_addr.sin_addr.s_addr = inet_addr(MAVPROXY_IP);
    remote_addr.sin_port = htons(LOCAL_PORT);
}

MavServer::~MavServer()
{
    send_recv_thread_run = false;
    send_recv_thread.join();
    close(sock);
}

void MavServer::run()
{
    send_recv_thread_run = true;
    send_recv_thread = std::thread(&MavServer::send_recv, this);
    send_recv_thread.detach();
}

bool MavServer::heartbeat_is_time_to_send()
{
    static std::chrono::time_point<std::chrono::system_clock> last_hb_sendtime =
        std::chrono::system_clock::now();

    int time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::system_clock::now() - last_hb_sendtime)
                           .count();
    if (time_elapsed > HEARTBEAT_SEND_INTERVAL_MS) {
        last_hb_sendtime = std::chrono::system_clock::now();
        return true;
    }

    return false;
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

bool MavServer::vehicle_is_ready()
{
    // Check if GPS is locked and if system status is OK
    mavlink_heartbeat_t status = get_svar_heartbeat();
    mavlink_gps_raw_int_t gps = get_svar_gps_raw_int();

    return gps.fix_type >= 2 && (status.system_status == MAV_STATE_STANDBY ||
                                 status.system_status == MAV_STATE_ACTIVE);
}

int MavServer::vehicle_get_status()
{
    mavlink_heartbeat_t status = get_svar_heartbeat();
    return status.system_status;
}

bool MavServer::vehicle_set_mode_guided()
{
    mavlink_set_mode_t mav_cmd_set_mode;
    mavlink_message_t mav_msg;
    static uint8_t mav_data_buffer[BUFFER_LEN];

    static std::chrono::time_point<std::chrono::system_clock> send_time =
        std::chrono::system_clock::from_time_t(0);

    // Send command to change mode to Guided
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now() - send_time)
            .count() > MODE_SET_REQUEST_INTERVAL_MS) {
        mav_cmd_set_mode.target_system = DEFAULT_TARGET_COMPONENT_ID;

        // Arducopter does not use the standard MAV_MODE_FLAG. It uses
        // a custom mode instead. GUIDED mode is defined as 4.
        mav_cmd_set_mode.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        mav_cmd_set_mode.custom_mode = 4; // GUIDED == 4

        mavlink_msg_set_mode_encode(DEFAULT_SYSTEM_ID,
                                    DEFAULT_TARGET_COMPONENT_ID, &mav_msg,
                                    &mav_cmd_set_mode);

        int n = mavlink_msg_to_send_buffer(mav_data_buffer, &mav_msg);
        vehicle_send_data(mav_data_buffer, n);

        send_time = std::chrono::system_clock::now();
    }

    // Check if mode has changed and ACK has been received
    if (vehicle_cmd_long_ack_recvd(MAVLINK_MSG_ID_SET_MODE,
                                   MAV_RESULT_ACCEPTED)) {
        return true;
    }

    return false;
}

void MavServer::vehicle_send_our_heartbeat()
{

    mavlink_heartbeat_t mav_heartbeat;
    mavlink_message_t mav_msg;
    static uint8_t mav_data_buffer[BUFFER_LEN];

    heartbeat.type = MAV_TYPE_GCS;
    heartbeat.autopilot = MAV_AUTOPILOT_INVALID;
    heartbeat.base_mode = 0;
    heartbeat.custom_mode = 0;
    heartbeat.system_status = MAV_STATE_ACTIVE;

    mavlink_msg_heartbeat_encode(DEFAULT_SYSTEM_ID, DEFAULT_TARGET_COMPONENT_ID,
                                 &mav_msg, &mav_heartbeat);

    int n = mavlink_msg_to_send_buffer(mav_data_buffer, &mav_msg);
    vehicle_send_data(mav_data_buffer, n);
}

void MavServer::vehicle_send_data(const uint8_t *data, int data_len)
{

    data_to_send_access_mtx.lock();

    int safe_data_len =
        std::min((int)data_len, (int)(BUFFER_LEN - data_to_send_len));

    if (safe_data_len < data_len) {
        // Do not send incomplete messages
        return;
    }

    memcpy(&data_to_send[data_to_send_len], data,
           safe_data_len * sizeof(*data));
    data_to_send_len += safe_data_len;

    data_to_send_access_mtx.unlock();
}

void MavServer::vehicle_send_cmd_long(mavlink_command_long_t cmd)
{
    mavlink_message_t mav_msg;
    static uint8_t mav_data_buffer[BUFFER_LEN];

    mavlink_msg_command_long_encode(
        DEFAULT_SYSTEM_ID, DEFAULT_TARGET_COMPONENT_ID, &mav_msg, &cmd);

    int n = mavlink_msg_to_send_buffer(mav_data_buffer, &mav_msg);
    vehicle_send_data(mav_data_buffer, n);
}

void MavServer::vehicle_send_waypoint(mavlink_mission_item_t mav_waypoint)
{
    mavlink_message_t mav_msg;
    static uint8_t mav_data_buffer[BUFFER_LEN];

    mav_waypoint.current = 2; // Set as a Guided waypoint

    mavlink_msg_mission_item_encode(DEFAULT_SYSTEM_ID,
                                    DEFAULT_TARGET_COMPONENT_ID, &mav_msg,
                                    &mav_waypoint);

    int n = mavlink_msg_to_send_buffer(mav_data_buffer, &mav_msg);
    vehicle_send_data(mav_data_buffer, n);
}

bool MavServer::vehicle_cmd_long_ack_recvd(int mav_cmd_id,
                                           int mav_result_expected)
{
    if (command_ack_isnew) {
        mavlink_command_ack_t cmd_ack = get_svar_command_ack();

        if (cmd_ack.command == mav_cmd_id &&
            cmd_ack.result == mav_result_expected) {
            return true;
        }
    }

    return false;
}

bool MavServer::vehicle_send_cmd_long_until_ack(int cmd, float p1, float p2,
                                                float p3, float p4, float p5,
                                                float p6, float p7, int timeout)
{
    mavlink_command_long_t mav_cmd;

    static std::unordered_map<
        int, std::chrono::time_point<std::chrono::system_clock>> send_time = {};

    if (!send_time.count(cmd) ||
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now() - send_time[cmd])
                .count() > timeout) {

        mav_cmd.target_system = DEFAULT_TARGET_SYSTEM_ID;
        mav_cmd.target_component = DEFAULT_TARGET_COMPONENT_ID;
        mav_cmd.command = cmd;
        mav_cmd.confirmation = 0;
        mav_cmd.param1 = p1;
        mav_cmd.param2 = p2;
        mav_cmd.param3 = p3;
        mav_cmd.param4 = p4;
        mav_cmd.param5 = p5;
        mav_cmd.param6 = p6;
        mav_cmd.param7 = p7;

        vehicle_send_cmd_long(mav_cmd);
        send_time[cmd] = std::chrono::system_clock::now();
    }

    if (vehicle_cmd_long_ack_recvd(cmd, MAV_RESULT_ACCEPTED)) {
        return true;
    }

    return false;
}

mavlink_attitude_t MavServer::get_svar_attitude()
{
    attitude_svar_access_mtx.lock();
    mavlink_attitude_t attitude_copy = attitude;
    attitude_isnew = false;
    attitude_svar_access_mtx.unlock();
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

mavlink_home_position_t MavServer::get_svar_home_position()
{
    svar_access_mtx.lock();
    mavlink_home_position_t home_position_copy = home_position;
    home_position_isnew = false;
    svar_access_mtx.unlock();
    return home_position_copy;
}

mavlink_local_position_ned_t MavServer::get_svar_local_pos_ned()
{

    local_pos_ned_svar_access_mtx.lock();
    mavlink_local_position_ned_t local_pos_int_copy = local_pos_ned;
    local_pos_ned_isnew = false;
    local_pos_ned_svar_access_mtx.unlock();
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

void MavServer::send_recv()
{
    while (send_recv_thread_run) {
        handle_recv();

        data_to_send_access_mtx.lock();
        if (data_to_send_len > 0) {
            handle_send();
            data_to_send_len = 0;
        }
        data_to_send_access_mtx.unlock();
    }
}

void MavServer::handle_send()
{
    sendto(sock, (void *)data_to_send, data_to_send_len, 0,
           (struct sockaddr *)&remote_addr, sizeof(struct sockaddr_in));
}

void MavServer::handle_recv()
{
    mavlink_message_t msg;
    mavlink_status_t status;

    memset(data_recv, 0, BUFFER_LEN);
    ssize_t bytes_recvd = recvfrom(sock, (void *)data_recv, BUFFER_LEN, 0,
                                   (struct sockaddr *)&remote_addr, &fromlen);

    if (bytes_recvd > 0) {
        print_debug_mav("Bytes Received: %d\nDatagram: ", (int)bytes_recvd);
        for (unsigned int i = 0; i < bytes_recvd; ++i) {
            print_debug_mav("%02x ", (unsigned char)data_recv[i]);
            if (mavlink_parse_char(MAVLINK_COMM_0, data_recv[i], &msg,
                                   &status)) {
                print_debug_mav("\nReceived packet: CHK: %d, MGC: %d, SYS: %d, "
                                "COMP: %d, LEN: %d, MSG ID: %d, SEQ: %d\n",
                                msg.checksum, msg.magic, msg.sysid, msg.compid,
                                msg.len, msg.msgid, msg.seq);
                handle_message(&msg);
            }
        }
        print_debug_mav("\n");
    }
    memset(data_recv, 0, BUFFER_LEN);
}

void MavServer::handle_message(const mavlink_message_t *msg)
{

#if DEBUG_MAVLINK
    static uint32_t prev_time_att = 0;
    static uint32_t prev_time_local_pos = 0;
#endif

    switch (msg->msgid) {
    case MAVLINK_MSG_ID_HOME_POSITION: {
        svar_access_mtx.lock();
        mavlink_msg_home_position_decode(msg, &home_position);
        home_position_isnew = true;
        svar_access_mtx.unlock();
        break;
    }
    case MAVLINK_MSG_ID_COMMAND_ACK:
        svar_access_mtx.lock();
        mavlink_msg_command_ack_decode(msg, &command_ack);
        command_ack_isnew = true;
        svar_access_mtx.unlock();
        break;
    case MAVLINK_MSG_ID_GPS_RAW_INT:
        svar_access_mtx.lock();
        mavlink_msg_gps_raw_int_decode(msg, &gps_raw_int);
        gps_raw_int_isnew = true;
        svar_access_mtx.unlock();
        break;
    case MAVLINK_MSG_ID_HEARTBEAT:
        svar_access_mtx.lock();
        mavlink_msg_heartbeat_decode(msg, &heartbeat);
        heartbeat_isnew = true;
        svar_access_mtx.unlock();
        break;
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        local_pos_ned_svar_access_mtx.lock();
        mavlink_msg_local_position_ned_decode(msg, &local_pos_ned);
        local_pos_ned_isnew = true;
        print_debug_mav("locpos_msg_time = %d\n",
                        local_pos_ned.time_boot_ms - prev_time_local_pos);
        local_pos_ned_svar_access_mtx.unlock();
        break;
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        svar_access_mtx.lock();
        mavlink_msg_global_position_int_decode(msg, &global_pos_int);
        global_pos_int_isnew = true;
        svar_access_mtx.unlock();
        break;
    case MAVLINK_MSG_ID_ATTITUDE:
        static int attnum = 1;
        attnum++;
        attitude_svar_access_mtx.lock();
        mavlink_msg_attitude_decode(msg, &attitude);
        attitude_isnew = true;
        print_debug_mav("att_msg_time = %d\n",
                        attitude.time_boot_ms - prev_time_att);
        attitude_svar_access_mtx.unlock();
        break;
    default:
        break;
    }
}

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(DroneCameraPlugin)

DroneCameraPlugin::DroneCameraPlugin()
    : global_pos_coord_system(common::SphericalCoordinates::EARTH_WGS84),
      mavserver(MAVPROXY_PORT)
{
    this->model = NULL;
}

DroneCameraPlugin::~DroneCameraPlugin()
{
    event::Events::DisconnectWorldUpdateBegin(this->update_connection);
}

void DroneCameraPlugin::OnUpdate()
{
    if (mavserver.heartbeat_is_time_to_send()) {
        mavserver.vehicle_send_our_heartbeat();
    }

    // Execute according to simulation state
    switch (simstate) {

    case INIT: {

        if (!mavserver.vehicle_is_ready()) {
            return;
        }

        int target_status = mavserver.vehicle_get_status();

        if (target_status == MAV_STATE_STANDBY) {
            simstate = INIT_ON_GROUND;
            print_debug_state("state: INIT_ON_GROUND\n");
        } else if (target_status == MAV_STATE_ACTIVE) {
            simstate = INIT_AIRBORNE;
            print_debug_state("state: INIT_AIRBORNE\n");
        }

        return;
    }

    case INIT_AIRBORNE: {

        // Check if home position has already been received
        if (mavserver.home_position_isnew) {
            mavlink_home_position_t home = mavserver.get_svar_home_position();
            mavlink_global_position_int_t home_pos = home_pos_to_global(home);
            set_global_pos_coord_system(home_pos);
            simstate = ACTIVE_AIRBORNE;
            print_debug_state("state: ACTIVE_AIRBORNE\n");
            return;
        }

        // Home position is critical. Request home position every
        // HOME_POSITION_REQUEST_INTERVAL_MS ms until it received.
        if (!mavserver.vehicle_send_cmd_long_until_ack(
                MAV_CMD_GET_HOME_POSITION, 0, 0, 0, 0, 0, 0, 0,
                HOME_POSITION_REQUEST_INTERVAL_MS)) {
            static bool debug_msg_print = false;
            if (!debug_msg_print) {
                print_debug_state("Requesting Home Position...\n");
                debug_msg_print = true;
            }
        }

        return;
    }

    case INIT_ON_GROUND: {

        // Get Status
        mavlink_heartbeat_t hb = mavserver.get_svar_heartbeat();
        bool is_guided = hb.base_mode & MAV_MODE_FLAG_GUIDED_ENABLED;
        bool is_armed = hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;
        bool is_flying = (hb.system_status == MAV_STATE_ACTIVE);

        // Wait until initial global position is achieved
        if (!vehicle_ground_pos_locked()) {
            return;
        }

        // Initial takeoff if AUTOTAKEOFF and if not already on air
        if (!TAKEOFF_AUTO) {
            simstate = ACTIVE_ON_GROUND;
            print_debug_state("state: ACTIVE_ON_GROUND\n");
            return;
        }

        if (!is_guided) {
            mavserver.vehicle_set_mode_guided();
            static bool debug_msg_print = false;
            if (!debug_msg_print) {
                print_debug_state("Changing to GUIDED mode...\n");
                debug_msg_print = true;
            }
            return;
        }

        if (!is_armed) {
            mavserver.vehicle_send_cmd_long_until_ack(
                MAV_CMD_COMPONENT_ARM_DISARM, 1, 0, 0, 0, 0, 0, 0,
                TAKEOFF_REQUEST_INTERVAL_MS);
            static bool debug_msg_print = false;
            if (!debug_msg_print) {
                print_debug_state("Arming Throttle...\n");
                debug_msg_print = true;
            }
            return;
        }

        if (!is_flying) {
            mavserver.vehicle_send_cmd_long_until_ack(
                MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, TAKEOFF_INIT_ALT_M,
                TAKEOFF_REQUEST_INTERVAL_MS);
            static bool debug_msg_print = false;
            if (!debug_msg_print) {
                print_debug_state("Requesting Takeoff...\n");
                debug_msg_print = true;
            }
            return;
        }

        print_debug_state("Takeoff Sucessfull.\n");
        simstate = ACTIVE_AIRBORNE;
        print_debug_state("state: ACTIVE_AIRBORNE\n");

        return;
    }

    case ACTIVE_ON_GROUND:
    case ACTIVE_AIRBORNE: {

        // Make sure the target still exists
        if (!this->target) {
            print_debug_state("Target not found.\n");
            simstate = ERROR;
            print_debug_state("state: ERROR\n");
            return;
        }

        // Get Target Position
        math::Pose curr_pose;
        math::Vector3 curr_vel;
        math::Vector3 curr_ang_vel;
        static math::Pose tpose = math::Pose(math::Pose::Zero);
        math::Pose tpose_new = this->target->GetWorldPose();
        if (vehicle_is_flying() && tpose_new != tpose) {
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
            mavserver.vehicle_send_waypoint(
                mavserver.pose_to_waypoint_relative_alt(
                    global_coord.x, global_coord.y, global_coord.z,
                    pose_mavlocal.rot.GetYaw()));
        }

        // Calculate pose according to new attitude and position
        if (mavserver.local_pos_ned_isnew || mavserver.attitude_isnew) {

            // Set New Drone Pose in Gazebo
            calculate_pose(&curr_pose, mavserver.get_svar_attitude(),
                           mavserver.get_svar_local_pos_ned());
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

void DroneCameraPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{

    // Output the name of the model
    print_debug_state("The gzsitl plugin is attached to the model\n");

    // Store the model pointer for convenience
    this->model = _model;

    // Also store the target pointer
    this->target = this->model->GetWorld()->GetModel(GZSITL_TARGET_MODEL_NAME);

    // Run MavServer thread
    this->mavserver.run();

    // Set initial simulation parameters
    printf("init\n");
    simstate = INIT;
    print_debug_state("state: INIT\n");

    // Listen to the update event
    this->update_connection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&DroneCameraPlugin::OnUpdate, this));
}

bool DroneCameraPlugin::vehicle_is_flying()
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

bool DroneCameraPlugin::vehicle_ground_pos_locked()
{
    static int n = 0;

    // If vehicle is not airborne, receive Initial Position at least
    // INIT_POS_NUMSAMPLES times
    if (n < INIT_POS_NUMSAMPLES) {
        if (mavserver.global_pos_int_isnew && mavserver.vehicle_is_ready()) {
            n++;
        }
        return false;
    }

    if (n == INIT_POS_NUMSAMPLES) {
        init_global_pos = mavserver.get_svar_global_pos_int();

        set_global_pos_coord_system(init_global_pos);

        n = INIT_POS_NUMSAMPLES + 1;
    }

    return true;
}

mavlink_global_position_int_t
DroneCameraPlugin::home_pos_to_global(mavlink_home_position_t home)
{
    mavlink_global_position_int_t global_pos;

    global_pos.lat = home.latitude;
    global_pos.lon = home.longitude;
    global_pos.alt = home.altitude;

    return global_pos;
}

math::Pose DroneCameraPlugin::coord_gzlocal_to_mavlocal(math::Pose gzpose)
{
    return math::Pose(gzpose.pos.x, -gzpose.pos.y, -gzpose.pos.z,
                      gzpose.rot.GetRoll(), -gzpose.rot.GetPitch(),
                      -gzpose.rot.GetYaw());
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

void DroneCameraPlugin::calculate_pose(
    math::Pose *pose, mavlink_attitude_t attitude,
    mavlink_local_position_ned_t local_position)
{
    pose->Set(local_position.x, -local_position.y, -local_position.z,
              attitude.roll, -attitude.pitch, -attitude.yaw);
}