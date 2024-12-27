/*------------------------------------------
         single position regulation
-------------------------------------------*/
// Common
#include <sstream>
#include <ros/ros.h>

// KDL相关
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

// Local
#include "joint_space_planner.hpp"  

// 控制周期（每5ms一个控制周期）
#define TIME_STEP 0.005

// ----------------------- PID 数据结构  -----------------------
struct PID
{
    // 使用梯形积分对误差进行积分
    inline void calcI()
    {
        I += 0.5*(e + e0)*T;
    }

    // 增益矩阵
    Eigen::MatrixXd Kp;
    Eigen::MatrixXd Kd;
    Eigen::MatrixXd Ki;

    // 误差
    Eigen::VectorXd e;    // 当前误差
    Eigen::VectorXd e0;   // 上一次的误差
    Eigen::VectorXd I;    // 积分项
    double T;             // 采样周期
};

// ----------------------- iiwa 类  -----------------------
class iiwa
{
public:

    iiwa(ros::NodeHandle nh, KDL::Tree & tree, KDL::Chain & chain, KDL::Vector g)
    : nodh(nh), system_tree(tree), arm_chain(chain), gravity(g),
      dynamics_kdl(arm_chain, gravity), jacobian_kdl(arm_chain), fk_kdl(arm_chain)
    {
        // ----------------------- 关节/动力学相关初始化  -----------------------
        n = arm_chain.getNrOfJoints();
        q.resize(n);        // 关节位置
        q_dot.resize(n);    // 关节速度

        // 规划器输出(期望关节角度及角速度)
        qd     = Eigen::VectorXd::Zero(n);
        qd_dot = Eigen::VectorXd::Zero(n);

        // 动力学相关: C(科里奥利项), G(重力项), M(惯性矩阵)
        C.resize(n); 
        G.resize(n); 
        M.resize(n);

        // 雅可比矩阵
        J.resize(n);

        // 用于前向运动学
        fk_frame = KDL::Frame::Identity();

        // ----------------------- 关节空间位置PID -----------------------
        torque.resize(n);
        position_pid.Kp = Eigen::MatrixXd::Identity(n, n);
        position_pid.Kd = Eigen::MatrixXd::Identity(n, n);
        position_pid.Ki = Eigen::MatrixXd::Identity(n, n);

        // 这里对角线赋初值，可根据实际需求调整
        position_pid.Kp.diagonal() << 700, 700, 500, 700, 700, 700, 10;
        position_pid.Kd.diagonal() << 5, 100, 5, 10, 5, 5, 1;
        position_pid.Ki.diagonal() << 100, 1000, 500, 1000, 100, 100, 50;
        position_pid.T  = TIME_STEP;
        position_pid.e  = Eigen::VectorXd::Zero(n);
        position_pid.e0 = Eigen::VectorXd::Zero(n);
        position_pid.I  = Eigen::VectorXd::Zero(n);

        // ----------------------- 梯形速度规划器初始化  -----------------------
        trapez_planner.init(n, M_PI_2, M_PI_4, TIME_STEP);

        // 设置期望位置qd并传给梯形规划器
        qd[0] = 0;     
        qd[1] = 0.72;
        qd[2] = 0;
        qd[3] = -1.94;
        qd[4] = 0;
        qd[5] = 0.48;
        qd[6] = M_PI_2;
        q_i = qd;  // 保存初始期望位置

        // 设置梯形规划的起点和终点
        Eigen::VectorXd q0 = Eigen::VectorXd::Zero(n);
        trapez_planner.setup(q0, qd);

        // 机器人初始状态标志位
        initial_position = false;

        // ----------------------- ROS 发布者/订阅者  -----------------------
        torque_control_pub.resize(n);
        std::stringstream ss;
        for(size_t i = 0; i < n; ++i) {
            ss << "/iiwa/joint" << (i + 1) << "_torque_controller/command";
            torque_control_pub[i] = nodh.advertise<std_msgs::Float64>(ss.str(), 10);
            ss.str("");
        }

        // 订阅关节状态
        joint_states_sub = nodh.subscribe("/iiwa/joint_states", 10, &iiwa::update_joints_state, this);

        // 用于发布给扭矩话题的消息
        torque_msg.resize(n);
    }

    ~iiwa() {}

    // ----------------------- 关节状态回调函数  -----------------------
    void update_joints_state(const sensor_msgs::JointState::ConstPtr & msg)
    {
        // 1. 更新关节位置及速度
        for (size_t i = 0; i < n; ++i) {
            q(i)     = msg->position[i];
            q_dot(i) = msg->velocity[i];
        }

        // 2. 计算重力和科里奥利补偿
        dynamics_kdl.JntToCoriolis(q, q_dot, C);
        dynamics_kdl.JntToGravity(q, G);

        // 3. 若未到达初始位置，执行梯形速度规划 + 位置PID控制
        if (!initial_position)
        {
            // 梯形速度规划器迭代
            trapez_planner.step();
            trapez_planner.update_position(qd);
            trapez_planner.update_velocity(qd_dot);

            // 位置PID误差计算
            position_pid.e0 = position_pid.e;
            position_pid.e  = qd - q.data;
            position_pid.calcI();

            // PID + 重力补偿 + 科里奥利补偿
            torque = position_pid.Kp * position_pid.e
                     + position_pid.Ki * position_pid.I
                     + position_pid.Kd * (qd_dot - q_dot.data)
                     + G.data;

            // 如果到达期望初始位置的判定条件
            if ((q_i - q.data).norm() < 5e-2) {

            }
        }

        // 4. 发布力矩
        for (size_t i = 0; i < n; ++i) {
            torque_msg[i].data = torque[i];
            torque_control_pub[i].publish(torque_msg[i]);
        }
    }

private:
    // NodeHandle
    ros::NodeHandle nodh;

    // 基链和末端
    std::string base_link;
    std::string end_effector;
    std::string robot_description;

    // KDL树和链
    KDL::Tree  system_tree;
    KDL::Chain arm_chain;

    // 动力学重力向量
    KDL::Vector gravity;

    // 关节数
    uint n;

    // 当前关节状态
    KDL::JntArray q;
    KDL::JntArray q_dot;

    // 动力学求解器
    KDL::ChainDynParam dynamics_kdl;
    // 雅可比
    KDL::ChainJntToJacSolver jacobian_kdl;
    // 前向运动学
    KDL::ChainFkSolverPos_recursive fk_kdl;
    KDL::Frame fk_frame;

    // 动力学矩阵
    KDL::JntArray G;  // 重力
    KDL::JntArray C;  // 科里奥利
    KDL::JntSpaceInertiaMatrix M; // 惯性矩阵
    KDL::Jacobian J;

    // 控制量
    Eigen::VectorXd torque;   // 力矩
    Eigen::VectorXd qd;       // 期望关节角
    Eigen::VectorXd q_i;      // 初始期望关节角（用于判断是否到达）
    Eigen::VectorXd qd_dot;   // 期望关节角速度

    // 位置控制PID
    PID position_pid;

    // 梯形速度规划器
    TrapezPlanner trapez_planner;

    // 初始位置是否到位标志
    bool initial_position;

    // ROS 发布与订阅
    std::vector<ros::Publisher> torque_control_pub;
    std::vector<std_msgs::Float64> torque_msg;
    ros::Subscriber joint_states_sub;
};

// ----------------------- 切换到力矩控制模式  -----------------------
void setupControllerManager(ros::NodeHandle & nh)
{
    // 创建Service客户端
    ros::ServiceClient switchTorqueController
        = nh.serviceClient<controller_manager_msgs::SwitchController>(
            "/iiwa/controller_manager/switch_controller");

    // 准备SwitchController服务请求
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

    // 调用服务，启动关节力矩控制器
    switchTorqueController.call(srv);
}

// ----------------------- 主函数  -----------------------
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "iiwa_control");
    ros::NodeHandle nh;

    ROS_INFO("Start iiwa_control node...");

    // 1. 切换到力矩控制模式
    setupControllerManager(nh);

    // 2. 从参数服务器读取机器人描述，构建 KDL Tree 和 Chain
    std::string base_link = "iiwa_link_0";
    std::string end_effector = "tool_link_ee_kuka";
    std::string robot_description = "robot_description";
    KDL::Vector gravity(0, 0, -9.80);
    KDL::Tree system_tree;
    KDL::Chain arm_chain;

    if (!kdl_parser::treeFromParam(robot_description, system_tree)){
        ROS_INFO_STREAM("Failed to import robot from param: " << robot_description);
    }
    system_tree.getChain(base_link, end_effector, arm_chain);

    // 3. 创建iiwa对象，初始化控制器与动态补偿
    iiwa iiwa_robot(nh, system_tree, arm_chain, gravity);

    // 4. 进入回调循环
    ros::spin();
    return 0;
}
