#include "trajectory_generator.hpp"
#include <iostream>
#include <cmath>

TrapezoidalVelocityProfile::TrapezoidalVelocityProfile(double starting_time, double ending_time, 
                                        double starting_pos, double final_pos, double max_speed)
{
    t_i = starting_time;
    t_f = ending_time;
    q_i = starting_pos;
    q_f = final_pos;
    max_trajectory_speed = max_speed;
    T =  t_f - t_i;
    h = q_f - q_i;
    t_acc = (max_trajectory_speed * T - h) / max_trajectory_speed;
    t_dec = t_acc;

    if(!(max_trajectory_speed >= h/T))
    {
        std::cout << "Cannot define a suitable trajectory with given parameters; increase max speed or trajectory time, or decrease trajectory space" << std::endl;
        exit(1);
    }
}

double TrapezoidalVelocityProfile::compute_trajectory_position(double t)
{
    if(t < t_i)     // before t_i
    {
        return 0.0;
    }
    else if (t <= t_i + t_acc)      // acceleration phase
    {
        return q_i + max_trajectory_speed/(2*t_acc) * pow((t-t_i),2);
    }
    else if (t < t_f - t_dec)       // constant velocity phase
    {
        return q_i + max_trajectory_speed * (t-t_i-t_acc/2);
    }
    else if(t <= t_f)       // deceleration phase
    {
        return q_f - max_trajectory_speed/(2*t_dec) * pow((t_f-t),2);
    }
    else        // after t_f
    {
        return q_f; 
    }
}

double TrapezoidalVelocityProfile::compute_trajectory_velocity(double t)
{
    if (t >= t_i && t <= t_i + t_acc)   // acceleration phase
    {
        return (max_trajectory_speed / t_acc) * (t - t_i);
    }
    else if (t < t_f - t_dec) // constant velocity phase
    { 
        return max_trajectory_speed;
    }
    else if(t <= t_f) // deceleration phase
    {
        return (max_trajectory_speed / t_dec) * (t_f - t);
    }
    else // outside [t_i, t_f]
    {
        return 0.0;
    }
}

double TrapezoidalVelocityProfile::compute_trajectory_acceleration(double t)
{
    if (t >= t_i && t <= t_i + t_acc)   // acceleration phase
    {
        return max_trajectory_speed / t_acc;
    }
    else if (t < t_f - t_dec) // constant velocity phase
    { 
        return 0.0;
    }
    else if(t <= t_f) // deceleration phase
    {
        return max_trajectory_speed / t_dec;
    }
    else // outside [t_i, t_f]
    {
        return 0.0;
    }
}

double TrapezoidalVelocityProfile::compute_trajectory_jerk(double t)
{
    return 0.0;
}

CubicTrajectory::CubicTrajectory(double starting_time, double ending_time, double starting_pos, double final_pos,
                         double starting_vel, double final_vel)
{
    t_i = starting_time;
    t_f = ending_time;
    q_i = starting_pos;
    q_f = final_pos; 
    q_i_dot = starting_vel;
    q_f_dot = final_vel;
    T = t_f - t_i;
    a_0 = q_i;
    a_1 = q_i_dot;
    a_2 = (-3*(q_i - q_f) - (2*q_i_dot + q_f_dot)*T)/pow(T,2);
    a_3 = (2*(q_i - q_f) + (q_i_dot + q_f_dot)*T)/pow(T,3);
}

double CubicTrajectory::compute_trajectory_position(double t)
{
    return a_0 + a_1*(t-t_i) + a_2*pow((t-t_i),2) + a_3*pow((t-t_i),3);
}

double CubicTrajectory::compute_trajectory_velocity(double t)
{
    return a_1 + 2*a_2*(t-t_i) + 3*a_3*pow((t-t_i),2);
}

double CubicTrajectory::compute_trajectory_acceleration(double t)
{
    return 2*a_2 + 6*a_3*(t-t_i);
}

double CubicTrajectory::compute_trajectory_jerk(double t)
{
    return 6*a_3;
}