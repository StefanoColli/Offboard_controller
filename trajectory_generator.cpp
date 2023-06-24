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
        return (max_trajectory_speed / t_acc) * t;
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