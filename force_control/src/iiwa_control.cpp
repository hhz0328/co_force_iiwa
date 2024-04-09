// Common
#include <sstream>

// ROS
#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

// Services
#include <controller_manager_msgs/SwitchController.h>

// Messages
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Point.h>

// Local
#include "joint_space_planner.hpp"


#define TIME_STEP 0.005

struct PID
{
    inline void calcI()
    {
        I += 0.5*(e + e0)*T;
    }


    Eigen::MatrixXd Kp;
    Eigen::MatrixXd Kd;
    Eigen::MatrixXd Ki;

    Eigen::VectorXd e;
    Eigen::VectorXd e0;

    Eigen::VectorXd I;
    double T;
};

class iiwa
{

public:

    iiwa(ros::NodeHandle nh, KDL::Tree & tree, KDL::Chain & chain, KDL::Vector g)
    : nodh(nh), system_tree(tree), arm_chain(chain), gravity(g),
    dynamics_kdl(arm_chain, gravity), jacobian_kdl(arm_chain), fk_kdl(arm_chain)
    {
        //// Eigen and KDL part
        // Initialize KDL state vecotrs
        n = arm_chain.getNrOfJoints();
        q.resize(n); q_dot.resize(n); qd_dot = Eigen::VectorXd::Zero(n);
        C.resize(n); G.resize(n); M.resize(n);
        J.resize(n);
        E.resize(3, 4);
        force_pid.e.resize(6);
        fk_frame = KDL::Frame::Identity();
        Fe.resize(6); Fd.resize(6); Fzd = 0; z_I = 0; z_r;
        camera_point = Eigen::Vector3d(0, 0, 0);
        base_point = Eigen::Vector3d(0, 0, 0);
        z_angle = 0;


        /// Control
        // Setup FLC control
        torque.resize(n);
        position_pid.Kp = Eigen::MatrixXd::Identity(n, n);
        position_pid.Kd = Eigen::MatrixXd::Identity(n, n);
        position_pid.Ki = Eigen::MatrixXd::Identity(n, n);
        position_pid.Kp.diagonal() << 700, 700, 500, 700, 700, 700, 10;
        position_pid.Kd.diagonal() << 5, 100, 5, 10, 5, 5, 1;
        position_pid.Ki.diagonal() << 100, 1000, 500, 1000, 100, 100, 50;
        position_pid.T = TIME_STEP;
        position_pid.e  = Eigen::VectorXd::Zero(n);
        position_pid.e0 = Eigen::VectorXd::Zero(n);
        position_pid.I = Eigen::VectorXd::Zero(n);

        // Setup Force control
        force_pid.Kp = Eigen::MatrixXd::Identity(6, 6);
        force_pid.Kp.diagonal() << 500, 500, 500, 10, 10, 10;
        force_pid.Kd = Eigen::MatrixXd::Identity(n, n);
        force_pid.Kd.diagonal() << 2, 2, 2, 2, 1, 1, 1;

        // Setup trapez planner
        trapez_planner.init(n, M_PI_2, M_PI_4, TIME_STEP);

        // Setup test initial positon (initial positon)
        qd.resize(n);
		qd[0] = 0;
		qd[1] = 0.72;
		qd[2] = 0;
		qd[3] = -1.94;
		qd[4] = 0;
		qd[5] = 0.48;
		qd[6] = M_PI_2;
        q_i = qd;

        pd = KDL::Vector(0.46, 0, 0.12);

        // Desired orientation (y rotation to pi)
        quatd = Eigen::Quaterniond(cos(-M_PI/4), 0, 0, sin(-M_PI/4))*Eigen::Quaterniond(cos(M_PI/2), 0, sin(M_PI/2), 0);

        // Desired Force z  (-120)
        Fzd = -65;

        // Test planner
        Eigen::VectorXd q0 = Eigen::VectorXd::Zero(n);
        trapez_planner.setup(q0, qd);

        // Force control
        initial_positon = false;
        force_control = false;

        //// ROS part
        // Publishers and Subscribers
        torque_control_pub.resize(n);
        std::stringstream ss;
        for(size_t i = 0; i < n; ++i) {
            ss << "/iiwa/joint" << (i + 1) << "_torque_controller/command";
            torque_control_pub[i] = nodh.advertise<std_msgs::Float64>(ss.str(), 10);
            ss.str("");
        }
        joint_states_sub = nodh.subscribe("/iiwa/joint_states", 10, &iiwa::update_joints_state, this);
        ft_sensor_sub = nodh.subscribe("/iiwa/state/CartesianWrench", 10, &iiwa::update_ft_state, this);
        camer_point_sub = nodh.subscribe("/iiwa/camera1/line_coordinate", 2, &iiwa::update_desired_point, this);

        torque_msg.resize(n);
    }

    ~iiwa()
    {}

    void update_joints_state(const sensor_msgs::JointState::ConstPtr & msg)
    {
        /// Update joint state
        for (size_t i = 0; i < n; ++i) {
            q(i) = msg->position[i];
            q_dot(i) = msg->velocity[i];
        }
        // M ddq + C dq + g = tau
        // f(q,dq, ddq) = tau
        /// Get Dynamic model matrices (M, C, G)
        // dynamics_kdl.JntToMass(q, M);
        dynamics_kdl.JntToCoriolis(q, q_dot, C);
        dynamics_kdl.JntToGravity(q, G);

        if (!initial_positon) {
            /// Simple control
            trapez_planner.step();
            trapez_planner.update_position(qd);
            trapez_planner.update_velocity(qd_dot);

            position_pid.e0 = position_pid.e;
            position_pid.e  = qd - q.data;
            position_pid.calcI();

            torque = (position_pid.Kp*position_pid.e + position_pid.Ki*position_pid.I + position_pid.Kd*(qd_dot - q_dot.data) + G.data);
            // ROS_INFO_STREAM("e: " << position_pid.e.transpose());

            if ((q_i - q.data).norm() < 5e-2) {
                initial_positon = true;
                force_control = true;
            }
        }

        if (force_control) {

            // Forwark Kinematics
            fk_kdl.JntToCart(q, fk_frame);
            fk_frame.M.GetQuaternion(quate.x(), quate.y(), quate.z(), quate.w());
            // ROS_INFO_STREAM(fk_frame.p.x() << ", " << fk_frame.p.y() << ", " << fk_frame.p.z());
            // ROS_INFO_STREAM(quate.x() << ", " << quate.y() << ", " << quate.z() << ", " << quate.w());

            // Rotate point from camera
            base_point = (quate * camera_point);
            // ROS_INFO_STREAM(base_point.transpose());

            // Jacobian
            jacobian_kdl.JntToJac(q, J);

            // Quaternion E matrix
            calculateEmatrixFromQuaternion(E, quate);

            // [x y fz roll pitch yaw]

            // Hybrid Posion/Force control (position)
            z_I += 0.00002*(Fzd - Fe[2]);
            z_r = pd.data[2];
            z_r += 0.0002*(Fzd - Fe[2]) + z_I;

            // Orientation
            z_angle = 0.005*atan2(camera_point[1], -camera_point[0]);
            // ROS_INFO_STREAM(z_angle);
            quatd = Eigen::Quaterniond(cos(z_angle/2), 0, 0, sin(z_angle/2))*quatd;

            pd.data[0] += 0.0001*base_point[0];
            pd.data[1] += 0.0001*base_point[1];
            // ROS_INFO_STREAM(z_r);

            // error
            p_err.data[0] = pd.data[0] - fk_frame.p.data[0];
            p_err.data[1] = pd.data[1] - fk_frame.p.data[1];
            p_err.data[2] = z_r        - fk_frame.p.data[2];

            w_err = 2*E*(quatd.coeffs() - quate.coeffs());
            force_pid.e[0] = p_err.x();
            force_pid.e[1] = p_err.y();
            force_pid.e[2] = p_err.z();
            force_pid.e[3] = w_err.x();
            force_pid.e[4] = w_err.y();
            force_pid.e[5] = w_err.z();
            // ROS_INFO_STREAM("e: " << force_pid.e.transpose());

            // S(q) (PD+) + (I - S(q)) (MDC)
            torque = (J.data.transpose())*force_pid.Kp*force_pid.e - force_pid.Kd*q_dot.data + G.data;
        }

        for (size_t i = 0; i < n; ++i) {
            torque_msg[i].data = torque[i];
            torque_control_pub[i].publish(torque_msg[i]);
        }
    }

    void update_ft_state(const geometry_msgs::WrenchStamped::ConstPtr & msg)
    {
        Fe[0] = msg->wrench.force.x;
        Fe[1] = msg->wrench.force.y;
        Fe[2] = msg->wrench.force.z;
        Fe[3] = msg->wrench.torque.x;
        Fe[4] = msg->wrench.torque.y;
        Fe[5] = msg->wrench.torque.z;
    }

    void update_desired_point(const geometry_msgs::Point::ConstPtr & msg)
    {
        if (force_control) {
            camera_point[0] = msg->x;
            camera_point[1] = msg->y;
        }
    }

private:

    void calculateEmatrixFromQuaternion(Eigen::MatrixXd & E, Eigen::Quaterniond & quat)
    {
        E(0, 0) =  quat.w();
        E(1, 0) =  quat.z();
        E(2, 0) = -quat.y();

        E(0, 1) = -quat.z();
        E(1, 1) =  quat.w();
        E(2, 1) =  quat.x();

        E(0, 2) =  quat.y();
        E(1, 2) = -quat.x();
        E(2, 2) =  quat.w();

        E(0, 3) = -quat.x();
        E(1, 3) = -quat.y();
        E(2, 3) = -quat.z();
    }

    std::string base_link;
    std::string end_effector;
    std::string robot_description;

    KDL::Tree system_tree;
    KDL::Chain arm_chain;

    KDL::Vector gravity;

    /// Joint number
    uint n;

    /// Joint state
    KDL::JntArray q;
    KDL::JntArray q_dot;



    /// Dynamics and Kinematics
    // M, C, G matrices
    KDL::ChainDynParam dynamics_kdl;
    // Jacobian matrix
    KDL::ChainJntToJacSolver jacobian_kdl;
    // Forwark kinematics solver KDL
    KDL::ChainFkSolverPos_recursive fk_kdl;
    KDL::Frame fk_frame;
    KDL::Vector pd;         // Desired position
    KDL::Vector p_err;      // Position error

    /// Dynamic and Kinematic model matrices
    KDL::JntArray G;
    KDL::JntArray C;
    KDL::JntSpaceInertiaMatrix M;
    KDL::Jacobian J;




    /// Control
    // Common
    Eigen::VectorXd torque;
    Eigen::VectorXd qd, q_i;
    Eigen::VectorXd qd_dot;

    // Quaternion (desired and measured)
    Eigen::Quaterniond quatd;
    Eigen::Quaterniond quate;

    Eigen::Vector3d w_err;      // rotation error;
    Eigen::MatrixXd E;          // Quaternion transfrom matrix

    Eigen::VectorXd Fe;         // Measured force
    Eigen::VectorXd Fd;         // Desired force
    double Fzd;                 // Desired force z
    double z_I;                 // Integral part of z
    double z_r;
    double z_angle;

    // Point from camera and relative base
    Eigen::Vector3d camera_point;
    Eigen::Vector3d base_point;

    // joint positoions control
    PID position_pid;
    TrapezPlanner trapez_planner;
    // force control
    PID force_pid;



    /// NodeHandle
    ros::NodeHandle nodh;

    /// Publishers and Subscribers
    std::vector<ros::Publisher> torque_control_pub;
    ros::Subscriber joint_states_sub;
    ros::Subscriber ft_sensor_sub;
    ros::Subscriber camer_point_sub;

    std::vector<std_msgs::Float64> torque_msg;


    /// State machine
    bool initial_positon;
    bool force_control;
};

void setupControllerManager(ros::NodeHandle & nh)
{
    ros::ServiceClient switchTorqueController = nh.serviceClient<controller_manager_msgs::SwitchController>("/iiwa/controller_manager/switch_controller");

    // Fill the service message
    controller_manager_msgs::SwitchController srv;

    std::vector<std::string> start_controllers(7);
    std::stringstream ss;
    for (size_t i = 0; i < 7; ++i) {
        ss << "joint" << (i + 1) << "_torque_controller";
        start_controllers[i] = ss.str();
        ss.str("");
    }
    srv.request.start_controllers = start_controllers;

    srv.request.strictness = srv.request.BEST_EFFORT;
    ROS_INFO_STREAM(srv.request);

    switchTorqueController.call(srv);
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "iiwa_control");
    ros::NodeHandle nh;

    ROS_INFO("Hi");
    setupControllerManager(nh);

    std::string base_link = "iiwa_link_0";
    std::string end_effector = "tool_link_ee_kuka";
    std::string robot_description = "robot_description";
    KDL::Vector gravity(0, 0, -9.80);
    KDL::Tree system_tree;
    KDL::Chain arm_chain;

    // Parse urdf from ROS param and get arm chain
    if (!kdl_parser::treeFromParam(robot_description, system_tree))
        ROS_INFO_STREAM("Problems with import robot from param");
    system_tree.getChain(base_link, end_effector, arm_chain);

    iiwa iiwa_robot(nh, system_tree, arm_chain, gravity);

    ros::spin();
}