//
// Takeoff, perform helical trajectory then land using MAVSDK.
//

#include <chrono>
#include <cstdint>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/log_files/log_files.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/log_callback.h>
#include <iostream>
#include <future>
#include <memory>
#include <fstream>
#include <queue>
#include <thread>

using namespace mavsdk;
using std::this_thread::sleep_for;

// HELIX PARAMETERS
float helix_origin[3] = {0.0f, 0.0f, -5.0f};
float helix_radius = 5.0f;
float helix_start[3] = {helix_radius, 0.0f, -5.0f};
float vertical_separation = 1.0f;
float trajectory_speed = 0.4f;
uint64_t unix_epoch_time_us;
uint32_t boot_time_ms;
bool logging = true;

// store messages
std::queue<mavlink_position_target_local_ned_t> pos_stp_queue;
std::queue<mavlink_local_position_ned_t> pos_queue;           

void usage(const std::string &bin_name)
{
    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
              << "Connection URL format should be :\n"
              << " For TCP : tcp://[server_host][:server_port]\n"
              << " For UDP : udp://[bind_host][:bind_port]\n"
              << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
              << "For example, to connect to the simulator use URL: udp://:14540\n";
}

auto connect_to_system(Mavsdk &mavsdk, char *url)
{
    ConnectionResult connection_result = mavsdk.add_any_connection(url);

    if (connection_result != ConnectionResult::Success)
    {
        std::cerr << "Connection failed: " << connection_result << std::endl;
        exit(1);
    }

    std::cout << "Searching for systems..." << std::endl;
    while (!mavsdk.systems().size())
    {
        sleep_for(std::chrono::seconds(1));
    }

    for (auto system : mavsdk.systems())
    {
        std::cout << "Found system with MAVLink system ID: " << static_cast<int>(system->get_system_id())
                  << ", connected: " << (system->is_connected() ? "yes" : "no")
                  << ", has autopilot: " << (system->has_autopilot() ? "yes" : "no") << std::endl;
    }

    auto system = mavsdk.systems()[0];
    if (!system)
    {
        std::cerr << "Timed out waiting for system\n";
        exit(1);
    }
    return system;
}

void preflight_check(Telemetry &telemetry)
{
    // Exit if calibration is required
    Telemetry::Health check_health = telemetry.health();
    bool calibration_required = false;
    if (!check_health.is_gyrometer_calibration_ok)
    {
        std::cerr << "Gyro requires calibration." << std::endl;
        calibration_required = true;
    }
    if (!check_health.is_accelerometer_calibration_ok)
    {
        std::cerr << "Accelerometer requires calibration." << std::endl;
        calibration_required = true;
    }
    if (!check_health.is_magnetometer_calibration_ok)
    {
        std::cerr << "Magnetometer (compass) requires calibration." << std::endl;
        calibration_required = true;
    }
    if (calibration_required)
    {
        // exit(1);
    }

    // Check if ready to arm (reporting status)
    while (telemetry.health_all_ok() != true)
    {
        std::cerr << "Vehicle not ready to arm. Waiting on:" << std::endl;
        Telemetry::Health current_health = telemetry.health();
        if (!current_health.is_global_position_ok)
        {
            std::cerr << "  - GPS fix." << std::endl;
        }
        if (!current_health.is_local_position_ok)
        {
            std::cerr << "  - Local position estimate." << std::endl;
        }
        if (!current_health.is_home_position_ok)
        {
            std::cerr << "  - Home position to be set." << std::endl;
        }
        sleep_for(std::chrono::seconds(1));
    }
}

void arm(Action &action, Telemetry &telemetry)
{
    // Arm vehicle
    std::cout << "Arming..." << '\n';

    while (!telemetry.health().is_armable)
    {
        sleep_for(std::chrono::milliseconds(500));
    }
    const Action::Result arm_result = action.arm();

    if (arm_result != Action::Result::Success)
    {
        std::cout << "Arming failed: " << arm_result << std::endl;
        exit(1); // Exit if arming fails
    }
}

void takeoff(Action &action, float height, Telemetry &telemetry)
{
    action.set_takeoff_altitude(height);

    // Command Take off
    std::cout << "Taking off..." << std::endl;
    const Action::Result takeoff_result = action.takeoff();
    if (takeoff_result != Action::Result::Success)
    {
        std::cout << "Takeoff failed:" << takeoff_result << std::endl;
        exit(1);
    }

    // float target_alt = action.get_takeoff_altitude();
    float current_position = 0;
    while (current_position < height)
    {
        current_position = telemetry.position().relative_altitude_m;
        sleep_for(std::chrono::seconds(1));
    }
    // Reached target altitude
    std::cout << "Takeoff Completed" << std::endl;
}

void land_and_disarm(Action &action, Telemetry &telemetry)
{
    const Action::Result land_result = action.land();
    if (land_result != Action::Result::Success)
    {
        // Land failed, so exit (in reality might try a return to land or kill.)
        exit(1);
    }

    // Check if vehicle is still in air
    std::cout << "Vehicle is landing..." << std::endl;
    while (telemetry.in_air())
    {
        sleep_for(std::chrono::seconds(1));
    }
    std::cout << "Landed!" << std::endl;

    // wait for automatic disarm
    while (telemetry.armed())
    {
        sleep_for(std::chrono::seconds(1));
    }
    std::cout << "Drone Disarmed" << std::endl;
}

void start_offboard_mode(Offboard &offboard)
{
    // Create a setpoint before starting offboard mode (in this case a null setpoint)
    offboard.set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});

    // Start offboard mode.
    Offboard::Result offboard_result = offboard.start();
    if (offboard_result != Offboard::Result::Success)
    {
        std::cerr << "Cannot start offboard mode: " << offboard_result << std::endl;
    }
    else
    {
        std::cout << "Switched to offboard mode" << std::endl;
    }
}

void stop_offboard_mode(Offboard &offboard)
{
    // Stop offboard mode
    Offboard::Result offboard_result = offboard.stop();
    if (offboard_result != Offboard::Result::Success)
    {
        std::cerr << "Offboard::stop() failed: " << offboard_result << std::endl;
    }
    else
    {
        std::cout << "Stopped offboard mode" << std::endl;
    }
}

bool estimated_position_close_to(const Offboard::PositionNedYaw &target_pos, float acceptance_radius_m, Telemetry &telemetry)
{
    Telemetry::PositionNed est_pos = telemetry.position_velocity_ned().position;
    const float distance_m = std::sqrt(std::pow((est_pos.north_m - target_pos.north_m), 2) +
                                       std::pow((est_pos.east_m - target_pos.east_m), 2) +
                                       std::pow((est_pos.down_m - target_pos.down_m), 2));
    return distance_m < acceptance_radius_m;
}

void offboard_goto(const Offboard::PositionNedYaw &target_pos, Offboard &offboard, Telemetry &telemetry)
{
    // TODO approach with a certain velocity
    // TODO smoothly reach the target (copy from px4?)
    offboard.set_position_ned(target_pos);
    if (offboard.start() != Offboard::Result::Success)
    {
        std::cerr << "Cannot start setpoint positioning" << std::endl;
    }
    while (!estimated_position_close_to(target_pos, 0.1f, telemetry))
    {
        sleep_for(std::chrono::milliseconds(500));
    }
    std::cout << "Target reached" << std::endl;
}

void compute_helix_setpoint(Offboard::PositionNedYaw &target_pos, Offboard::VelocityNedYaw &target_vel, double trajectory_time)
{
    target_pos.north_m = helix_origin[0] + helix_radius * cosf(trajectory_time * trajectory_speed);
    target_pos.east_m = helix_origin[1] + helix_radius * sinf(trajectory_time * trajectory_speed);
    target_pos.down_m = helix_origin[2] - vertical_separation * trajectory_time * trajectory_speed; // NED frame

    target_vel.north_m_s = -helix_radius * trajectory_speed * sinf(trajectory_time * trajectory_speed);
    target_vel.east_m_s = helix_radius * trajectory_speed * cosf(trajectory_time * trajectory_speed);
    target_vel.down_m_s = -vertical_separation * trajectory_speed; // NED frame
}

void download_ulg(LogFiles &logfiles, std::string dest_path)
{
    std::cout << "Downloading log file..." << std::endl;
    auto query_log = logfiles.get_entries();
    if (query_log.first == LogFiles::Result::Success)
    {
        auto log_entries = query_log.second;
        if (log_entries.size() == 1)
        {
            // download the only logfile
            std::cout << "Downloading log file..." << std::endl;
            auto download_res = logfiles.download_log_file(log_entries.at(0), dest_path + log_entries.at(0).date + ".ulg");
            if (download_res.first == LogFiles::Result::Success)
            {
                // delete the logfile from the drone so to leave it empty
                std::cout << "File correctly saved, deleting the logs onboard..." << std::endl;
                LogFiles::Result erase_res = logfiles.erase_all_log_files();
                if (erase_res == LogFiles::Result::Success)
                {
                    std::cout << "Onboard log files wiped out" << std::endl;
                }
                else
                {
                    std::cerr << "Cannot erase onboard log files" << std::endl;
                }
            }
            else
            {
                std::cerr << "Cannot download log file from drone" << std::endl;
            }
        }
        else
        {
            std::cerr << "Too many logfiles, please delete all onboard logs manually" << std::endl;
        }
    }
    else
    {
        std::cerr << "Cannot find any log file onboard" << std::endl;
    }
}

mavsdk::MavlinkPassthrough::Result set_message_rate(MavlinkPassthrough &mavlinkpassthrough, uint message_id, float frequency)
{
    float interval = 1/frequency *1e6f; //interval needs to be expressed in us

    MavlinkPassthrough::CommandLong cmd;
    cmd.target_sysid = mavlinkpassthrough.get_target_sysid();
    cmd.target_compid = mavlinkpassthrough.get_target_compid();
    cmd.command = MAV_CMD_SET_MESSAGE_INTERVAL;
    cmd.param1 = float (message_id); //message ID
    cmd.param2 = interval; //interval
    cmd.param3 = NAN;
    cmd.param4 = NAN;
    cmd.param5 = NAN;
    cmd.param6 = NAN;
    cmd.param7 = (float) 0; //response target

    return mavlinkpassthrough.send_command_long(cmd);
} 

void time_callback(uint64_t unix_time)
{
    unix_epoch_time_us = unix_time;
}

void setpoint_callback(const mavlink_message_t &msg)
{
    mavlink_position_target_local_ned_t stp;
    mavlink_msg_position_target_local_ned_decode(&msg, &stp);
    pos_stp_queue.push(stp);
    // printf("Setpoint %f , %f , %f\n", stp.x, stp.y, stp.z);
}

void position_callback(const mavlink_message_t &msg)
{
    mavlink_local_position_ned_t pos;
    mavlink_msg_local_position_ned_decode(&msg, &pos);
    pos_queue.push(pos);
    // printf("Position %f , %f , %f\n", pos.x, pos.y, pos.z);
}

void systime_callback(const mavlink_message_t &msg)
{
    mavlink_system_time_t time;
    mavlink_msg_system_time_decode(&msg, &time);
    boot_time_ms = time.time_boot_ms;
}

void subscribe_function(MavlinkPassthrough &mavlinkpassthrough, uint mavlink_topic, std::function<void (const mavlink_message_t &)> callback)
{
    //subscribe requested topic at 50Hz
    mavlinkpassthrough.subscribe_message_async(mavlink_topic, callback);
    set_message_rate(mavlinkpassthrough, mavlink_topic, 50.f);   

    while (logging)
    {
    }

    // unsubscribe from topics
    mavlinkpassthrough.subscribe_message_async(mavlink_topic, nullptr);
}

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        usage(argv[0]);
        exit(1);
    }

    Mavsdk mavsdk;

    mavsdk::log::subscribe(nullptr); // avoid mavsdk print debug messages to console
    auto system = connect_to_system(mavsdk, argv[1]);

    // Instantiate plugins
    Telemetry telemetry = Telemetry{system};
    Action action = Action{system};
    Offboard offboard = Offboard{system};
    LogFiles logfiles = LogFiles{system};
    MavlinkPassthrough mavlinkpassthrough = MavlinkPassthrough{system}; // to access the undelying mavlink protocol

    // Subscriptions
    telemetry.set_rate_unix_epoch_time(50.0); // TODO increase hz or use Telemetry::unix_epoch_time();
    telemetry.subscribe_unix_epoch_time(time_callback);

    // subscribe to mavlink topics
    // mavlinkpassthrough.subscribe_message_async(MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED, setpoint_callback);
    // mavlinkpassthrough.subscribe_message_async(MAVLINK_MSG_ID_LOCAL_POSITION_NED, position_callback);
    // mavlinkpassthrough.subscribe_message_async(MAVLINK_MSG_ID_SYSTEM_TIME, systime_callback);

    ///// FLIGHT /////
    preflight_check(telemetry);
    // Arm vehicle
    arm(action, telemetry);
    // Take off
    takeoff(action, 5.0, telemetry);

    // helical trajectory
    start_offboard_mode(offboard);

    // reach starting position
    Offboard::PositionNedYaw starting_pos{helix_start[0], helix_start[1], helix_start[2], 0.0f};
    offboard_goto(starting_pos, offboard, telemetry);

    uint64_t start_unix_time = unix_epoch_time_us;
    // uint32_t start_time_ms = boot_time_ms;
    Offboard::PositionNedYaw pos_stp;
    Offboard::VelocityNedYaw vel_stp;

    // start threads to log incoming messages
    std::thread thread_pos(subscribe_function, std::ref(mavlinkpassthrough), MAVLINK_MSG_ID_LOCAL_POSITION_NED, position_callback);
    std::thread thread_pos_stp(subscribe_function, std::ref(mavlinkpassthrough), MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED, setpoint_callback);

    double trajectory_time = 0.0;
    while (trajectory_time < 30.0)
    {
        trajectory_time = (unix_epoch_time_us - start_unix_time) / 1e6;
        // trajectory_time = (boot_time_ms - start_time_ms) / 1e3;

        compute_helix_setpoint(pos_stp, vel_stp, trajectory_time);
        offboard.set_position_velocity_ned(pos_stp, vel_stp);
        sleep_for(std::chrono::milliseconds(20)); // send setpoints @50Hz

        //offboard.set_position_ned(pos_stp); //TODO what is the difference wrt using set_position_velocity_ned?
    }
    stop_offboard_mode(offboard);
    logging = false;

    land_and_disarm(action, telemetry);

    ///// POST FLIGHT /////
    
    //join logging threads
    thread_pos.join();
    thread_pos_stp.join();

    download_ulg(logfiles, "./");

    //save logs to csv file
    std::ofstream csv_file;
    csv_file.open ("out.csv");
    csv_file << "pos_t,pos_x,pos_y,pos_z,stp_t,stp_x,stp_y,stp_z\n";

    int shortest_queue = pos_queue.size() < pos_stp_queue.size()? pos_queue.size() : pos_stp_queue.size(); 
    //TODO save 2 files and merge with pandas?
    while (shortest_queue)
    {
        mavlink_local_position_ned_t& pos = pos_queue.front();
        mavlink_position_target_local_ned_t& stp = pos_stp_queue.front();

        csv_file << pos.time_boot_ms << "," << pos.x << "," << pos.y << "," << pos.z << ","
                 << stp.time_boot_ms << "," << stp.x << "," << stp.y << "," << stp.z << "\n";

        pos_queue.pop();
        pos_stp_queue.pop();
        shortest_queue --;
    }
    csv_file.close();

    return 0;
}
