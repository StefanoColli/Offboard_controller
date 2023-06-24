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



