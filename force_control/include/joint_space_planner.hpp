#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>

class LocalTrapecPlanner
{

public:

    void init(double max_velocity, double max_acceleration, double time_step)
    {
        half_T = 0.5*time_step;
        q_dot_max = max_velocity;
        q_ddot_max = max_acceleration;
        q_diff_available = q_dot_max*(q_dot_max/q_ddot_max);
    }

    // q_i -- q initial; q_e -- q end; t_f -- time of end of trajectory
    void setup(double q_i, double q_e)
    {
        q0 = q_i; q1 = q_e;
        double q_diff = std::abs(q_e - q_i);

        do_calculation = q_diff > q_diff_min;
        if (!do_calculation) {
            time = 0;
            q_actual = q1;
            q_dot_actual = 0;
            return;
        }


        // Initialize some variable;
        time = 0;
        q_actual = q0;
        q_dot_actual = 0;

        // Calculate velocity and time parameters
        q_dot_const = q_diff > q_diff_available ? q_dot_max : sqrt(q_diff*q_ddot_max);
        t1 = q_dot_const/q_ddot_max;
        t2 = q_diff/q_dot_const;
        t3 = t2 + t1;

        // Add direction
        direction = ((q_e > q_i) - (q_e < q_i));
        // ROS_INFO_STREAM(direction);
    }

    void step()
    {
        if (!do_calculation) return;

        // Velocity increasing part
        if (time >= 0 && time < t1) {
            q_actual += half_T*q_dot_actual;
            q_dot_actual = direction*q_ddot_max*time;
            q_actual += half_T*q_dot_actual;

            time += 2*half_T;
            return;
        }

        // Velocity constant part
        if (time >= t1 && time < t2) {
            q_actual += half_T*q_dot_actual;
            q_dot_actual = direction*q_dot_const;
            q_actual += half_T*q_dot_actual;

            time += 2*half_T;
            return;
        }

        // Velocity decreasing part
        if (time >= t2 && time < t3) {
            q_actual += half_T*q_dot_actual;
            q_dot_actual = direction*(q_dot_const - q_ddot_max*(time - t2));
            q_actual += half_T*q_dot_actual;

            time += 2*half_T;
            return;
        }

        if (time > t3) {
            q_actual = q1;
            q_dot_actual = 0;
            return;
        }

    }

    double get_positon()
    {
        return q_actual;
    }

    double get_velocity()
    {
        return q_dot_actual;
    }
    
    /// Parameters

    // Initial and End angles
    double q0, q1;
    
    // Velocity and Acceleration max values
    double q_dot_max, q_ddot_max;
    
    // Minimal position difference
    double q_diff_min;
    
    // Time variables
    double half_T, time;

    /// Calculation
    double q_diff_available;
    double q_dot_const;
    bool do_calculation;
    double t1, t2, t3;
    double q_actual;
    double q_dot_actual;
    double direction;
};

class TrapezPlanner
{
public:

    void init(size_t joints_number, double max_velocity, double max_acceleration, double time_step)
    {
        n = joints_number;
        local_planner.resize(n);
        for (size_t i = 0; i < n; ++i)
            local_planner[i].init(max_velocity, max_acceleration, time_step);
    }

    void setup(Eigen::VectorXd & q0, Eigen::VectorXd & q1)
    {
        for (size_t i = 0; i < n; ++i)
            local_planner[i].setup(q0[i], q1[i]);
    }

    void step()
    {
        for (size_t i = 0; i < n; ++i)
            local_planner[i].step();
    }

    void update_position(Eigen::VectorXd & q)
    {
        for (size_t i = 0; i < n; ++i)
            q[i] = local_planner[i].get_positon();
    }

    void update_velocity(Eigen::VectorXd & q_dot)
    {
        for (size_t i = 0; i < n; ++i)
            q_dot[i] = local_planner[i].get_velocity();
    }

    std::vector<LocalTrapecPlanner> local_planner;
    size_t n;
};