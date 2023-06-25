class TrajectoryGenerator
{
protected:
    double t_i;     //!< trajectory starting time [s]
    double t_f;     //!< trajectory ending time [s]
    double q_i;     //!< trajectory starting position
    double q_f;     //!< trajectory ending position
public: 
    virtual ~TrajectoryGenerator() = default;

    /**
     * @brief Compute position along trajectory at a given time instant
     * 
     * @param t time instant
     * @return double position
     */
    virtual double compute_trajectory_position(double t) = 0;

    /**
     * @brief Compute velocity along trajectory at a given time instant
     * 
     * @param t time instant
     * @return double velocity
     */
    virtual double compute_trajectory_velocity(double t) = 0;
};


/* Trapezoidal velocity profile
 * 
 *     max     |
 *  trajectory |___________________________
 *    speed    |           /|             |\                
 *             |          / |             | \               A --> v(t) = max_trajectory_speed/t_acc * (t-t_i)
 *             |         /  |             |  \                    q(t) = q_i + max_trajectory_speed/(2*t_acc) * (t-t_i)^2 
 *             |        /   |             |   \             B --> v(t) = max_trajectory_speed
 *             |       /    |             |    \                  q(t) = q_i + max_trajectory_speed * (t-t_i-t_acc/2)
 *             |      /     |             |     \           C --> v(t) = max_trajectory_speed/t_dec * (t_f-t)
 *             |     /  A   |      B      |  C   \                q(t) = q_f - max_trajectory_speed/(2*t_dec) * (t_f-t)^2 
 *             |    /       |             |       \
 *             |___/________|_____________|________\_____>t
 *            0    |        |             |        |
 *                t_i    t_i+t_acc     t_f-t_dec  t_f
 * 
 * This implementation assumes t_acc = t_dec.
 * We choose the maximum trajectory speed so, we have the following contraint on acceleration time (= deceleration time):
 *      t_acc = t_dec = (max_trajectory_speed * T - h) / max_trajectory_speed
 * where T = (t_f - t_i) is the duration of the entire trajectory, and h = (q_f - q_i) is the space covered by the trajectory.
 * We need to satisfy the condition max_trajectory_speed >= h/T
 */
class TrapezoidalVelocityProfile : public TrajectoryGenerator
{
private:
    double max_trajectory_speed;                      //!< maximum speed of the trajectory
    double T;                                         //!< duration of the trajectory (t_f - t_i)
    double h;                                         //!< space covered by the trajectory (q_f - q_i)
    double t_acc;                                     //!< acceleration time [s]
    double t_dec;                                     //!< deceleration time [s]
public:
    TrapezoidalVelocityProfile(const TrapezoidalVelocityProfile&) = delete;                 //disable copy-constructor
    TrapezoidalVelocityProfile& operator=(const TrapezoidalVelocityProfile&) = delete;      //disable copy-assignment

    /**
     * @brief Construct a new Trapezoidal Velocity Profile object
     * 
     * @param starting_time trajectory starting instant
     * @param ending_time trajectory ending instant
     * @param starting_pos trajectory starting position 
     * @param final_pos trajectory final position (target position)
     * @param max_trajectory_speed maximum speed allowed during trajectory
     */
    TrapezoidalVelocityProfile(double starting_time, double ending_time, double starting_pos, double final_pos, double max_trajectory_speed);

    ~TrapezoidalVelocityProfile() = default;

    /**
     * @brief Compute position along trajectory at a given time instant
     * 
     * @param t time instant
     * @return double position
     */
    double compute_trajectory_position(double t);

    /**
     * @brief Compute velocity along trajectory at a given time instant
     * 
     * @param t time instant
     * @return double velocity
     */
    double compute_trajectory_velocity(double t); 
};


/*
 * Polynomial trajectory with grade 3 poly:
 * q(t) = a_0 + a_1*(t-t_i) + a_2*(t-t_i)^2 + a_3*(t-t_i)^3
 * q_dot(t) = a_1 + 2*a_2*(t-t_i) + 3*a_3*(t-t_i)^2
 * where: 
 *  - T = t_f - t_i
 *  - a_0 = q_i
 *  - a_1 = q_i_dot
 *  - a_2 = [-3*(q_i - q_f) - (2*q_i_dot + q_f_dot)*T]/T^2
 *  - a_3 = [2*(q_i - q_f) + (q_i_dot + q_f_dot)*T]/T^3
*/
class CubicTrajectory : public TrajectoryGenerator
{
private:    
    double q_i_dot;                 //!< initial velocity
    double q_f_dot;                 //!< final velocity
    double T;                       //!< total trajectory duration [s]
    double a_0, a_1, a_2, a_3;      //!< poly coefficients
public:
    CubicTrajectory(const CubicTrajectory&) = delete;                 //disable copy-constructor
    CubicTrajectory& operator=(const CubicTrajectory&) = delete;      //disable copy-assignment

    /**
     * @brief Construct a new Cubic Trajectory object
     * 
     * @param starting_time trajectory starting instant
     * @param ending_time trajectory ending instant
     * @param starting_pos trajectory starting position 
     * @param final_pos trajectory final position (target position)
     * @param starting_vel trajectory initial velocity
     * @param final_vel trajectory final velocity
     */
    CubicTrajectory(double starting_time, double ending_time, double starting_pos, double final_pos,
                         double starting_vel, double final_vel);
    ~CubicTrajectory() = default;

    /**
     * @brief Compute position along trajectory at a given time instant
     * 
     * @param t time instant
     * @return double position
     */
    double compute_trajectory_position(double t);

    /**
     * @brief Compute velocity along trajectory at a given time instant
     * 
     * @param t time instant
     * @return double velocity
     */
    double compute_trajectory_velocity(double t); 
};


