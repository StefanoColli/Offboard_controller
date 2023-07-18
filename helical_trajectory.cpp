//
// Takeoff, perform helical trajectory then land using MAVSDK.
//

#include <chrono>
#include <cstdint>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/plugins/info/info.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/log_files/log_files.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/log_callback.h>
#include <iostream>
#include <memory>
#include <fstream>
#include <queue>
#include <thread>
#include "trajectory_generator.hpp"

#define PI 3.14159265358979323846

using namespace mavsdk;
using std::this_thread::sleep_for;
 
/* Helical trajectory parameters 
 * The helix is a space curve with parametric equations (https://mathworld.wolfram.com/Helix.html):
 *      x =	r*cos(t)	         
 *      y =	r*sin(t)	 
 *      z =	c*t 
 * For t in [0, N_loops * 2*PI]
 * Where r is the helix radius, and 2*PI*c is a constant giving the vertical separation of the helix's loops.
 */
constexpr float helix_origin[3] = {0.0f, 0.0f, -5.0f};            //!< origin of the helical path (NED frame)
constexpr float helix_radius = 5.0f;                              //!< radius of the helical trajectory [m]
constexpr float helix_start[3] = {helix_radius, 0.0f, -5.0f};     //!< starting point of the helical trajectory (NED frame)
constexpr float vertical_separation = 2.0f;                       //!< vertical separation between loops in the helix [m]
constexpr float c = vertical_separation / (2*PI);                 //!< c parameter of the helix
constexpr unsigned int N_loops = 2;                               //!< number of loops in the helical trajectory
constexpr double max_trajectory_speed = 0.4;                      //!< maximum speed of the trajectory
constexpr double helix_duration = 50.0;                           //!< helical trajectory duration [s]

// global variables
uint64_t unix_epoch_time_us;                                                //!< onboard unix time [us]

// general settings
constexpr float takeoff_height = 5.0;                                       //!< height after takeoff

// ground station settings 
bool logging = true;                                                        //!< start/stop logging the position and position setpoint messages

// message queues for logging
std::queue<mavlink_position_target_local_ned_t> pos_stp_queue;
std::queue<mavlink_local_position_ned_t> pos_queue;           

/**
 * @brief Search and connect to Mavlink system
 * 
 * @param mavsdk MAVSDK instance
 * @param url connection URL string 
 * @return auto Mavlink system connected to
 */
auto connect_to_system(Mavsdk &mavsdk, const std::string& url)
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
        Info info = Info{system};
        // wait for system information to be retrieved
        while (info.get_identification().first==Info::Result::InformationNotReceivedYet) 
        {
            sleep_for(std::chrono::milliseconds(300));
        }

        // Get the system Version struct
        const Info::Version &system_version =  info.get_version().second;

        std::cout << "Found system with MAVLink system ID: " << static_cast<int>(system->get_system_id())
                  << "\n\tFirmware version: " << system_version.flight_sw_major << "." << system_version.flight_sw_minor
                  << "." << system_version.flight_sw_patch << "-" << system_version.flight_sw_vendor_major << "." 
                  << system_version.flight_sw_vendor_minor << "." << system_version.flight_sw_vendor_patch
                  << "\n\tConnected: " << (system->is_connected() ? "yes" : "no")
                  << "\n\tHas autopilot: " << (system->has_autopilot() ? "yes" : "no") << std::endl;
    }

    auto system = mavsdk.systems()[0];
    if (!system)
    {
        std::cerr << "Timed out waiting for system" << std::endl;
        exit(1);
    }
    return system;
}

/**
 * @brief Check if all calibrations are done and if the drone is ready to be armed
 * 
 * @param telemetry Telemetry plugin instance
 */
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

/**
 * @brief Set the drone parameters needed to perform a simulation (DO NOT USE THIS FUNCTION WHEN FLYING REAL HW)
 * 
 * @param param Param plugin instance
 */
void set_simulation_parameters(Param &param)
{
    std::cout << "Setting all parameters necessary for simulation" << std::endl;

    // ignore RC loss in any flight mode so to avoid triggering failsafe
    if (param.set_param_int("COM_RCL_EXCEPT", 7) != Param::Result::Success) 
    {
        std::cerr << "Unable to set COM_RCL_EXCEPT parameter" << std::endl; 
    }
    
    // disable joystick control, RC input handling and relative checks
    if ( param.set_param_int("COM_RC_IN_MODE", 4) != Param::Result::Success) 
    {
        std::cerr << "Unable to set COM_RC_IN_MODE parameter" << std::endl; 
    }

    // do nothing on data link loss
    if (param.set_param_int("NAV_DLL_ACT", 0) != Param::Result::Success) 
    {
        std::cerr << "Unable to set NAV_DLL_ACT parameter" << std::endl; 
    }
}

/**
 * @brief Arm the drone
 * 
 * @param action Action plugin instance
 * @param telemetry Telemetry plugin instance
 */
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

/**
 * @brief Send takeoff command and wait for the drone to reach the desired altitude
 * 
 * @param height desired takeoff altitude
 * @param action Action plugin instance
 * @param telemetry Telemetry plugin instance
 */
void takeoff(float height, Action &action, Telemetry &telemetry)
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

/**
 * @brief Send land command and wait touchdown and automatic disarm
 * 
 * @param action Action plugin instance
 * @param telemetry Telemetry plugin instance
 */
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

/**
 * @brief Switch drone flight mode to 'Offboard' (https://docs.px4.io/main/en/flight_modes/offboard.html)
 * 
 * @param offboard Offboard plugin instance
 */
void start_offboard_mode(Offboard &offboard)
{
    // Create a setpoint before starting offboard mode (in this case a null setpoint)
    offboard.set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});

    // Start offboard mode.
    Offboard::Result offboard_result = offboard.start();
    if (offboard_result != Offboard::Result::Success)
    {
        std::cerr << "Cannot start Offboard mode: " << offboard_result << std::endl;
    }
    else
    {
        std::cout << "Switched to Offboard mode" << std::endl;
    }
}

/**
 * @brief Stop 'Offboard' flight mode, the drone is put in 'Hold' mode (https://docs.px4.io/main/en/flight_modes/hold.html)
 * 
 * @param offboard Offboard plugin instance 
 */
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
        std::cout << "Stopped Offboard mode, drone is in Hold mode" << std::endl;
    }
}

/**
 * @brief Test wether drone position is in the neighborhood of the target position 
 * 
 * @param target_pos desired position
 * @param acceptance_radius_m neighborhood radius [m]
 * @param telemetry Telemetry plugin instance
 * @return true if the drone is in the neighborhood of the target position
 * @return false otherwise
 */
bool estimated_position_close_to(const Offboard::PositionNedYaw &target_pos, float acceptance_radius_m, Telemetry &telemetry)
{
    Telemetry::PositionNed est_pos = telemetry.position_velocity_ned().position;
    const float distance_m = std::sqrt(std::pow((est_pos.north_m - target_pos.north_m), 2) +
                                       std::pow((est_pos.east_m - target_pos.east_m), 2) +
                                       std::pow((est_pos.down_m - target_pos.down_m), 2));
    return distance_m < acceptance_radius_m;
}

/**
 * @brief Steer drone to target position and wait until it is in the neighborhood of the destination
 *  
 * @param target_pos desired target position
 * @param duration time allowed to the drone to reach target position [s]
 * @param offboard Offboard plugin instance
 * @param telemetry Telemetry plugin instance
 */
void offboard_goto(const Offboard::PositionNedYaw &target_pos, double duration, Offboard &offboard, Telemetry &telemetry)
{
    Telemetry::PositionNed current_pos = telemetry.position_velocity_ned().position;
    
    // use 3rd grade poly to have a smooth trajectory from current position to the target position
    CubicTrajectory north_trajectory = CubicTrajectory(0.0, duration, current_pos.north_m, target_pos.north_m, 0.0, 0.0);   
    CubicTrajectory east_trajectory = CubicTrajectory(0.0, duration, current_pos.east_m, target_pos.east_m, 0.0, 0.0);
    CubicTrajectory down_trajectory = CubicTrajectory(0.0, duration, current_pos.down_m, target_pos.down_m, 0.0, 0.0);

    Offboard::PositionNedYaw setpoint;
    uint64_t trajectory_start_unix_us = unix_epoch_time_us;
    double elapsed_time = 0.0;
    while (elapsed_time <= duration)
    {
        // update time
        elapsed_time = (unix_epoch_time_us - trajectory_start_unix_us) / 1e6;

        // compute setpoint
        setpoint.north_m = north_trajectory.compute_trajectory_position(elapsed_time);
        setpoint.east_m = east_trajectory.compute_trajectory_position(elapsed_time);
        setpoint.down_m = down_trajectory.compute_trajectory_position(elapsed_time);
        setpoint.yaw_deg = target_pos.yaw_deg; //not interested in smoothing yaw
        
        // send setpoint to the drone
        offboard.set_position_ned(setpoint);
    }

    // check if we reached target position, if not wait until it is reached
    while (!estimated_position_close_to(target_pos, 0.1f, telemetry))
    {
        sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "Target reached" << std::endl;
}

/**
 * @brief Compute next position and velocity setpoints of the helical trajectory
 * 
 * @param[out] target_pos output position setpoint
 * @param[out] target_vel output velocity setpoint
 * @param trajectory_gen TrajectoryGenerator instance used to smooth the trajectory
 * @param t time elapsed from the start of the helical motion [s] (t in [t_i, t_f])
 */
void compute_helix_setpoint(Offboard::PositionNedYaw &target_pos, Offboard::VelocityNedYaw &target_vel, TrajectoryGenerator &trajectory_gen, double t)
{   
    // compute trajectory with a trapezoidal profile speed
    double q = trajectory_gen.compute_trajectory_position(t);
    double q_dot = trajectory_gen.compute_trajectory_velocity(t); 

    // compute position and velocity setpoints
    target_pos.north_m = helix_origin[0] + helix_radius * cosf(q);
    target_pos.east_m = helix_origin[1] + helix_radius * sinf(q);
    target_pos.down_m = helix_origin[2] - c * q; // NED frame

    target_vel.north_m_s = - helix_radius * q_dot * sinf(q);
    target_vel.east_m_s = helix_radius * q_dot * cosf(q);
    target_vel.down_m_s = - c * q_dot; // NED frame
}

/**
 * @brief Download the log file (.ulg) from the drone and delete it from the onboard memory afterwards 
 * (assuming there exists a single log file on the drone. If there are more files, the function does nothing and the 
 * desired operation has to be carried out manually)
 * 
 * @param dest_path path where to store the log file 
 * @param logfiles LogFiles plugin instance
 */
void download_ulg(std::string dest_path ,LogFiles &logfiles)
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

/**
 * @brief Saves data logged by the control station to output file in csv format
 * 
 * @param filename name of the output .csv file (default is "out.csv")
 */
void log_to_csv(std::string filename = "out.csv")
{
    std::ofstream csv_file;
    csv_file.open(filename);
    csv_file << "pos_t,pos_x,pos_y,pos_z,stp_t,stp_x,stp_y,stp_z\n";

    int shortest_queue = pos_queue.size() < pos_stp_queue.size()? pos_queue.size() : pos_stp_queue.size(); 
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

    std::cout << "Saved log data to " << filename << std::endl;

}

/**
 * @brief Set the message rate of a particular topic to the desired frequency 
 * (the Offboard default link sends messages at low frequency )
 * 
 * @param message_id ID of the Mavlink message topic we want to set the frequency (https://mavlink.io/en/messages/common.html)
 * @param frequency desired frequency [Hz]
 * @param mavlinkpassthrough MavlinkPassthrough plugin instance
 */
void set_message_rate(uint message_id, float frequency, MavlinkPassthrough &mavlinkpassthrough)
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

    mavsdk::MavlinkPassthrough::Result res = mavlinkpassthrough.send_command_long(cmd);
    if (res != MavlinkPassthrough::Result::Success)
    {
        std::cerr << "Can't set the desired rate of " << frequency 
                  << " Hz, for message topic number " << message_id << std::endl;
    }
    
} 

/**
 * @brief Callback of the unix epoch time updates subscriber, it keeps the control station in synch with the drone updating global variable unix_epoch_time_us
 * 
 * @param unix_time Unix time update from the drone
 */
void time_callback(uint64_t unix_time)
{
    unix_epoch_time_us = unix_time;
}

/**
 * @brief Callback of the POSITION_TARGET_LOCAL_NED topic subscriber (https://mavlink.io/en/messages/common.html#POSITION_TARGET_LOCAL_NED), it pushes
 * all received messages to the log queue
 * 
 * @param msg incoming Mavlink message
 */
void setpoint_callback(const mavlink_message_t &msg)
{
    mavlink_position_target_local_ned_t stp;
    mavlink_msg_position_target_local_ned_decode(&msg, &stp);
    pos_stp_queue.push(stp);
}

/**
 * @brief Callback of the LOCAL_POSITION_NED topic subscriber (https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED), it pushes
 * all received messages to the log queue
 * 
 * @param msg incoming Mavlink message
 */
void position_callback(const mavlink_message_t &msg)
{
    mavlink_local_position_ned_t pos;
    mavlink_msg_local_position_ned_decode(&msg, &pos);
    pos_queue.push(pos);
}

/**
 * @brief Set requested topic rate to 50Hz, subscribe to requested topic and keep logging all received messages until logging variable is false 
 * (this function is meant to be run on a separate thread for each topic we want to log)
 * 
 * @param message_id ID of the Mavlink message topic we want to subscribe to and log (https://mavlink.io/en/messages/common.html) 
 * @param callback callback function to process each incoming message
 * @param mavlinkpassthrough MavlinkPassthrough plugin instance
 */
void subscribe_function(uint message_id, std::function<void (const mavlink_message_t &)> callback, MavlinkPassthrough &mavlinkpassthrough)
{
    // subscribe to requested topic at 50Hz
    mavlinkpassthrough.subscribe_message_async(message_id, callback);
    set_message_rate(message_id, 50.f, mavlinkpassthrough);   

    while (logging)
    {
    }

    // unsubscribe from topics
    mavlinkpassthrough.subscribe_message_async(message_id, nullptr);
}

int main(int argc, char **argv)
{
    std::string system_url;

    if (argc != 2)
    {
        std::cerr << "No connection URL specified, using udp://:14540\n"
                  << "For more options visit https://mavsdk.mavlink.io/main/en/cpp/guide/connections.html" << std::endl;
        system_url = "udp://:14540";
    }
    else
    {
        system_url = argv[1];
    }

    Mavsdk mavsdk;

    mavsdk::log::subscribe(nullptr); // avoid mavsdk print debug messages to console
    auto system = connect_to_system(mavsdk, system_url);

    // Instantiate plugins
    Telemetry telemetry = Telemetry{system};
    Action action = Action{system};
    Offboard offboard = Offboard{system};
    LogFiles logfiles = LogFiles{system};
    Param param = Param{system};
    MavlinkPassthrough mavlinkpassthrough = MavlinkPassthrough{system}; // to access the undelying mavlink protocol

    // Time subscription
    telemetry.set_rate_unix_epoch_time(50.0);
    telemetry.subscribe_unix_epoch_time(time_callback);

    // subscribe to mavlink topics
    // mavlinkpassthrough.subscribe_message_async(MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED, setpoint_callback);
    // mavlinkpassthrough.subscribe_message_async(MAVLINK_MSG_ID_LOCAL_POSITION_NED, position_callback);

    set_simulation_parameters(param);

    /////////////////////////////////////////////////
    /// FLIGHT
    /////////////////////////////////////////////////
    preflight_check(telemetry);
    // Arm vehicle
    arm(action, telemetry);
    // Take off
    takeoff(takeoff_height, action, telemetry);

    // offboard phase
    start_offboard_mode(offboard);

    // reach starting position
    std::cout << "Approaching starting position" << std::endl;
    Offboard::PositionNedYaw target_position{helix_start[0], helix_start[1], helix_start[2], 0.0f};
    offboard_goto(target_position, 10.0, offboard, telemetry);

    uint64_t start_unix_time_us = unix_epoch_time_us;
    Offboard::PositionNedYaw pos_stp;
    Offboard::VelocityNedYaw vel_stp;

    // We describe a path for the parameter of the helical trajectory (parameter t in https://mathworld.wolfram.com/Helix.html)
    //  having a trapezoidal velocity profile
    TrapezoidalVelocityProfile trapezoidal_trajectory = TrapezoidalVelocityProfile(0.0, helix_duration, 0.0, N_loops*2*PI, max_trajectory_speed);

    // start threads to log incoming messages
    std::thread thread_pos(subscribe_function, MAVLINK_MSG_ID_LOCAL_POSITION_NED, position_callback, std::ref(mavlinkpassthrough));
    std::thread thread_pos_stp(subscribe_function, MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED, setpoint_callback, std::ref(mavlinkpassthrough));

    std::cout << "Performing helical trajectory" << std::endl;
    double trajectory_time = 0.0;
    while (trajectory_time <= helix_duration)
    {
        trajectory_time = (unix_epoch_time_us - start_unix_time_us) / 1e6;

        compute_helix_setpoint(pos_stp, vel_stp, trapezoidal_trajectory,  trajectory_time);
        offboard.set_position_velocity_ned(pos_stp, vel_stp);

        sleep_for(std::chrono::milliseconds(20)); // send setpoints at 50Hz
    }
    stop_offboard_mode(offboard);
    logging = false;

    land_and_disarm(action, telemetry);

    /////////////////////////////////////////////////
    /// POST FLIGHT
    /////////////////////////////////////////////////
    
    // join logging threads
    thread_pos.join();
    thread_pos_stp.join();

    // if needed we can download the entire log file from the drone
    //download_ulg("./", logfiles);

     //disconnect from drone and free resources
    mavsdk.~Mavsdk();

    // save logs to csv file
    log_to_csv();

    return 0;
}
