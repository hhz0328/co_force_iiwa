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
#include <geometry_msgs/WrenchStamped.h> //用于传递力和力矩数据的消息
#include <geometry_msgs/Point.h> //用于传递三维点位置信息的消息

// Local
#include "joint_space_planner.hpp"

// 控制周期（每5ms一个控制周期）
#define TIME_STEP 0.005

struct PID
{
    // 使用梯形积分对误差进行积分
    inline void calcI()
    {
        I += 0.5*(e + e0)*T;
    }


    Eigen::MatrixXd Kp;
    Eigen::MatrixXd Kd;
    Eigen::MatrixXd Ki;

    Eigen::VectorXd e;  // 当前误差
    Eigen::VectorXd e0; // 上一次的误差

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
        // 当前关节角度和角速度（KDL格式）
        q.resize(n); q_dot.resize(n); 
        // 期望关节速度的Eigen向量，用于控制过程中给定期望速度参考
        qd_dot = Eigen::VectorXd::Zero(n);
        // 动力学相关数组-C：科里奥利项;G：重力项M：惯性矩阵;
        C.resize(n); G.resize(n); M.resize(n);
        // 雅可比矩阵-J：雅可比矩阵
        J.resize(n);
        // 用于姿态误差计算的矩阵（四元数误差转化矩阵）
        E.resize(3, 4);
        // 对应末端笛卡尔空间的6维误差（3维线速度/位置，3维角速度/姿态）
        force_pid.e.resize(6);
        // 前向运动学的结果，即当前末端位姿
        fk_frame = KDL::Frame::Identity();
        // Fe、Fd 是测量和期望的末端力（以及力矩）向量
        Fe.resize(6); Fd.resize(6);
        // Fzd 是期望的Z方向力，z_I 是Z方向力误差的积分项，z_r 将存储经过力控制调整的期望Z位移
        Fzd = 0; z_I = 0; z_r;
        // 相机检测的点的坐标
        camera_point = Eigen::Vector3d(0, 0, 0);
        // 转换后在基座坐标系下的坐标
        base_point = Eigen::Vector3d(0, 0, 0);
        // 存储从相机信息中计算出的关节或者末端角度偏差参考量
        z_angle = 0;


        /// Control
        // Setup FLC control
        torque.resize(n);
        // Identity(n, n)用于生成一个 n×nn×n 的单位矩阵，对角线元素为 1，其余元素为 0
        position_pid.Kp = Eigen::MatrixXd::Identity(n, n);
        position_pid.Kd = Eigen::MatrixXd::Identity(n, n);
        position_pid.Ki = Eigen::MatrixXd::Identity(n, n);
        // diagonal() 访问对角线元素，返回一个可赋值的向量
        position_pid.Kp.diagonal() << 700, 700, 500, 700, 700, 700, 10;
        position_pid.Kd.diagonal() << 5, 100, 5, 10, 5, 5, 1;
        position_pid.Ki.diagonal() << 100, 1000, 500, 1000, 100, 100, 50;
        position_pid.T = TIME_STEP;
        position_pid.e  = Eigen::VectorXd::Zero(n);
        position_pid.e0 = Eigen::VectorXd::Zero(n);
        position_pid.I = Eigen::VectorXd::Zero(n);

        // Setup Force control
        // force_pid 用于笛卡尔空间的力控制（6维：3个平移方向+3个旋转方向）
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
        // 期望末端位置 pd
        pd = KDL::Vector(0.46, 0, 0.12);

        // Desired orientation (y rotation to pi)
        // 设置期望姿态四元数 quatd，这里通过两个旋转的复合得到一个期望的末端姿态
        quatd = Eigen::Quaterniond(cos(-M_PI/4), 0, 0, sin(-M_PI/4))*Eigen::Quaterniond(cos(M_PI/2), 0, sin(M_PI/2), 0);

        // Desired Force z  (-65 N)
        Fzd = -65;

        // Test planner
        Eigen::VectorXd q0 = Eigen::VectorXd::Zero(n);
        // 将规划器设置从当前关节值 q0 到设定的 qd 的轨迹
        trapez_planner.setup(q0, qd);

        // Force control
        // 两个标志位：initial_positon 表示机器人是否已经抵达初始位置；force_control 表示是否开始进行力控制模式；
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
        // torque_msg 用于存储发布给扭矩话题的消息数据
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

        // 使用KDL函数计算出当前关节状态下的科里奥利项 C 和重力补偿项 G
        dynamics_kdl.JntToCoriolis(q, q_dot, C);
        dynamics_kdl.JntToGravity(q, G);

        // 机器人未达到初始设定位置时 , 使用梯形速度规划器和PID位置控制器
        if (!initial_positon) {
            /// Simple control
            trapez_planner.step();
            trapez_planner.update_position(qd);
            trapez_planner.update_velocity(qd_dot);

            position_pid.e0 = position_pid.e;
            position_pid.e  = qd - q.data;
            position_pid.calcI();

            torque = (position_pid.Kp*position_pid.e + position_pid.Ki*position_pid.I + position_pid.Kd*(qd_dot - q_dot.data) + G.data);

            if ((q_i - q.data).norm() < 5e-2) {
                initial_positon = true;
                force_control = true;
            }
        }
        // 当机器人已到达初始位置后 , 通过融合末端力传感器数据和视觉信息，对机器人进行混合位置/力控制
        if (force_control) {

            // Forwark Kinematics
            // 前向运动学求解器 fk_kdl 计算当前末端位姿（位置和方向）
            fk_kdl.JntToCart(q, fk_frame);
            // fk_frame 中包含末端位置和方向，从中提取当前末端姿态的四元数 quate
            fk_frame.M.GetQuaternion(quate.x(), quate.y(), quate.z(), quate.w());

            // Rotate point from camera
            // 使用当前末端姿态旋转向量,将从相机坐标系得到的点（camera_point）转换到基座坐标系下
            base_point = (quate * camera_point);
            // ROS_INFO_STREAM(base_point.transpose());

            // Jacobian 雅可比矩阵 
            jacobian_kdl.JntToJac(q, J);

            // Quaternion E matrix 用于姿态误差计算的矩阵
            calculateEmatrixFromQuaternion(E, quate);

            // [x y fz roll pitch yaw]

            // Hybrid Posion/Force control (position)
            // 利用力传感器反馈的末端法向力 Fe[2]（Z方向）来进行Z方向的混合位置/力控制
            // Fzd 为期望Z方向力
            // 根据力误差 (Fzd - Fe[2]) 调整期望末端Z位置 z_r，并对误差进行积分保存到 z_I
            z_I += 0.00002*(Fzd - Fe[2]);
            // 末端Z位置 z_r ,在Z方向上会根据实际力和期望力进行位置微调
            z_r = pd.data[2];
            z_r += 0.0002*(Fzd - Fe[2]) + z_I;


            


            // Orientation
            // 从相机数据计算一个 z_angle，用于调整期望姿态 quatd; 引导末端朝特定方向旋转
            z_angle = 0.005*atan2(camera_point[1], -camera_point[0]);
            // ROS_INFO_STREAM(z_angle);
            quatd = Eigen::Quaterniond(cos(z_angle/2), 0, 0, sin(z_angle/2))*quatd;

            // 根据 base_point 调整末端期望位置的X/Y坐标，实现对平面位置进行微调
            pd.data[0] += 0.0001*base_point[0];
            pd.data[1] += 0.0001*base_point[1];
            // ROS_INFO_STREAM(z_r);

            // error 机械臂末端xyz的位置误差
            p_err.data[0] = pd.data[0] - fk_frame.p.data[0];
            p_err.data[1] = pd.data[1] - fk_frame.p.data[1];
            p_err.data[2] = z_r        - fk_frame.p.data[2];
            // 计算姿态误差 w_err
            w_err = 2*E*(quatd.coeffs() - quate.coeffs());

            // 将位置误差和姿态误差整合到 force_pid.e 中（6维误差：3维平移，3维旋转），用于力控制计算
            force_pid.e[0] = p_err.x();
            force_pid.e[1] = p_err.y();
            force_pid.e[2] = p_err.z();
            force_pid.e[3] = w_err.x();
            force_pid.e[4] = w_err.y();
            force_pid.e[5] = w_err.z();

            // S(q) (PD+) + (I - S(q)) (MDC)
            /*
            最终通过雅可比转置控制律计算力矩指令：
            J^T * Kp * e：  通过末端误差计算所需的关节空间力矩
            - Kd * q_dot:   加入关节速度阻尼项
            + G：           重力补偿项
            */
            torque = (J.data.transpose())*force_pid.Kp*force_pid.e - force_pid.Kd*q_dot.data + G.data;
        }

        // 将计算得到的各关节力矩值发布到对应的关节力矩控制器话题，以驱动关节执行器
        for (size_t i = 0; i < n; ++i) {
            torque_msg[i].data = torque[i];
            torque_control_pub[i].publish(torque_msg[i]);
        }
    }

    // 该回调函数通过订阅 geometry_msgs::WrenchStamped 类型消息获得外部传感器（如末端力传感器）测得的力和力矩数据
    void update_ft_state(const geometry_msgs::WrenchStamped::ConstPtr & msg)
    {
        // 力
        Fe[0] = msg->wrench.force.x;
        Fe[1] = msg->wrench.force.y;
        Fe[2] = msg->wrench.force.z;
        // 力矩
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

    /*
    1. E 是一个用于姿态误差计算的特殊矩阵（3x4），与四元数相关。
    2. 输入的 quat 是末端当前姿态的四元数表示。通过该函数可将四元数映射到一个线性代数形式，
    便于计算角度误差 w_err。
    3. 在控制中，姿态误差 w_err 往往通过 w_err = 2 * E * (quatd.coeffs() - quate.coeffs()) 得到，
    其中 quatd 是期望姿态的四元数，quate 是当前姿态的四元数。
    4. E 矩阵的构造是基于四元数的分量进行特定排列，用来实现从四元数差转到空间误差向量的计算。
    */
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

// setupControllerManager 函数的作用是通过与 controller_manager 通讯，将机器人的关节控制器切换到力矩控制模式
// 该函数完成后，机器人应处于可通过发布力矩指令进行控制的状态
void setupControllerManager(ros::NodeHandle & nh)
{
    // 创建一个客户端，用于调用控制器管理器的服务
    // 这个服务用于切换当前机器人中控制器的状态（启动或停止）
    ros::ServiceClient switchTorqueController = nh.serviceClient<controller_manager_msgs::SwitchController>("/iiwa/controller_manager/switch_controller");

    // Fill the service message 准备一个 SwitchController 服务请求对象
    controller_manager_msgs::SwitchController srv;

    // 准备一个字符串向量 start_controllers，用于指定要启动的控制器列表
    std::vector<std::string> start_controllers(7);
    std::stringstream ss;
    for (size_t i = 0; i < 7; ++i) {
        ss << "joint" << (i + 1) << "_torque_controller";
        start_controllers[i] = ss.str();
        ss.str("");
    }
    srv.request.start_controllers = start_controllers;

    // strictness 设置为 BEST_EFFORT 表示如果不能启动所有控制器，也尽量启动能启动的
    srv.request.strictness = srv.request.BEST_EFFORT;
    // 输出请求信息，以便调试
    ROS_INFO_STREAM(srv.request);
    // 调用服务，以启动上述指定的关节力矩控制器
    switchTorqueController.call(srv);
}

int main(int argc, char ** argv)
{
    // 初始化ROS节点，节点名为 "iiwa_control"
    ros::init(argc, argv, "iiwa_control");
    ros::NodeHandle nh;

    ROS_INFO("Hi");
    // 在程序启动后立即调用控制器管理器配置函数，确保机器人处于力矩控制模式
    setupControllerManager(nh);

    // 定义一些字符串和变量，用于解析机器人URDF并创建KDL模型
    std::string base_link = "iiwa_link_0";
    std::string end_effector = "tool_link_ee_kuka";
    std::string robot_description = "robot_description";
    KDL::Vector gravity(0, 0, -9.80);
    KDL::Tree system_tree;
    KDL::Chain arm_chain;

    // 从参数服务器中获取robot_description（URDF），通过kdl_parser解析为KDL::Tree
    // 如果解析失败，打印提示信息
    if (!kdl_parser::treeFromParam(robot_description, system_tree))
        ROS_INFO_STREAM("Problems with import robot from param");
    // 从系统树中获取从 base_link 到 end_effector 的链
    system_tree.getChain(base_link, end_effector, arm_chain);
    // 创建 iiwa 类的实例，这个类中包含了动力学、运动学求解器、控制逻辑以及话题订阅和发布等内容
    // 构造函数会初始化控制参数、订阅话题、发布力矩指令等
    iiwa iiwa_robot(nh, system_tree, arm_chain, gravity);
    // ros::spin() 使程序进入循环等待回调状态，不断处理订阅的话题回调
    // 这样在回调函数中便可实现实时控制逻辑（如接收关节状态，计算力矩并发布）
    ros::spin();
}