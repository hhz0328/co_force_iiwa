/*------------------------------------------
         multi positions regulation
-------------------------------------------*/
#include <ros/ros.h>
#include <sstream>

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

// Local (自定义的梯形速度规划器头文件，需要自行提供或修改为实际路径)
#include "joint_space_planner.hpp"

// 控制周期（每5ms一个控制周期）
#define TIME_STEP 0.005

// -------------------------- PID 数据结构 --------------------------
struct PID
{
    // 使用梯形积分对误差进行积分
    inline void calcI()
    {
        I += 0.5 * (e + e0) * T;
    }

    // 增益矩阵
    Eigen::MatrixXd Kp;
    Eigen::MatrixXd Kd;
    Eigen::MatrixXd Ki;

    // 误差
    Eigen::VectorXd e;     // 当前误差
    Eigen::VectorXd e0;    // 上一次的误差
    Eigen::VectorXd I;     // 积分项
    double T;              // 采样周期
};

// -------------------------- iiwa 类 --------------------------
class iiwa
{
public:
    iiwa(ros::NodeHandle nh, KDL::Tree & tree, KDL::Chain & chain, KDL::Vector g)
      : nodh(nh), system_tree(tree), arm_chain(chain), gravity(g),
        dynamics_kdl(arm_chain, gravity), jacobian_kdl(arm_chain), fk_kdl(arm_chain)
    {
        // 关节数量
        n = arm_chain.getNrOfJoints();

        // 关节状态 (KDL::JntArray)
        q.resize(n);      
        q_dot.resize(n);  

        // 动力学相关: C(科里奥利), G(重力), M(惯性)
        C.resize(n);
        G.resize(n);
        M.resize(n);

        // 雅可比矩阵（如果只做关节空间控制，可不显式使用）
        J.resize(n);

        // 用于前向运动学
        fk_frame = KDL::Frame::Identity();

        // ---------------------- 多目标位置设置 ----------------------
        // 在这里添加 3 个目标位置，仅作示例，可按需修改
        Eigen::VectorXd goal1(n), goal2(n), goal3(n);

        // 示例关节角度
        goal1 << 0, 0.72, 0, -1.94, 0, 0.48, M_PI_2;
        goal2 << 0, 1.0,  0, -1.57, 0, 0.48, 1.2;
        goal3 << 0, 0.5,  0, -1.2,  0, 0.6,  0;

        goals.push_back(goal1);
        goals.push_back(goal2);
        goals.push_back(goal3);

        // 当前要去的目标索引
        current_goal_idx = 0;

        // ---------------------- 位置PID初始化 ----------------------
        torque.resize(n);
        position_pid.Kp = Eigen::MatrixXd::Identity(n, n);
        position_pid.Kd = Eigen::MatrixXd::Identity(n, n);
        position_pid.Ki = Eigen::MatrixXd::Identity(n, n);

        // 示例增益，可根据实际情况微调
        position_pid.Kp.diagonal() << 700, 700, 500, 700, 700, 700, 10;
        position_pid.Kd.diagonal() << 5, 100, 5, 10, 5, 5, 1;
        position_pid.Ki.diagonal() << 100, 1000, 500, 1000, 100, 100, 50;

        position_pid.T  = TIME_STEP;
        position_pid.e  = Eigen::VectorXd::Zero(n);
        position_pid.e0 = Eigen::VectorXd::Zero(n);
        position_pid.I  = Eigen::VectorXd::Zero(n);

        // ---------------------- 梯形速度规划器初始化 ----------------------
        trapez_planner.init(n, M_PI_2, M_PI_4, TIME_STEP);

        // 初始目标：goals[0]
        qd     = goals[current_goal_idx];
        qd_dot = Eigen::VectorXd::Zero(n);

        // 假设机器人从零位姿(q0)开始规划，也可改成从当前关节角度开始
        Eigen::VectorXd q0 = Eigen::VectorXd::Zero(n);
        trapez_planner.setup(q0, qd);

        // ---------------------- ROS相关(发布/订阅) ----------------------
        torque_control_pub.resize(n);
        std::stringstream ss;
        for(size_t i = 0; i < n; ++i) {
            ss << "/iiwa/joint" << (i + 1) << "_torque_controller/command";
            torque_control_pub[i] = nodh.advertise<std_msgs::Float64>(ss.str(), 10);
            ss.str("");
        }

        // 订阅关节状态
        joint_states_sub = nodh.subscribe("/iiwa/joint_states", 10, 
                                          &iiwa::update_joints_state, this);

        // 用于发布给力矩话题的消息
        torque_msg.resize(n);
    }

    ~iiwa() {}

    // ---------------------- 回调：关节状态更新 ----------------------
    void update_joints_state(const sensor_msgs::JointState::ConstPtr & msg)
    {
        // 1. 读取当前关节位置、速度
        for (size_t i = 0; i < n; ++i) {
            q(i)     = msg->position[i];
            q_dot(i) = msg->velocity[i];
        }

        // 2. 计算动力学补偿（重力 + 科里奥利）
        dynamics_kdl.JntToCoriolis(q, q_dot, C);
        dynamics_kdl.JntToGravity(q, G);

        // 3. 若还有目标未完成，则进行控制
        if (current_goal_idx < (int)goals.size())
        {
            // 当前目标
            qd = goals[current_goal_idx];

            // 梯形速度规划器
            trapez_planner.step();
            trapez_planner.update_position(qd);
            trapez_planner.update_velocity(qd_dot);

            // PID误差
            position_pid.e0 = position_pid.e;
            position_pid.e  = qd - q.data;
            position_pid.calcI();

            // 力矩 = PID + G + C
            torque = position_pid.Kp * position_pid.e
                     + position_pid.Ki * position_pid.I
                     + position_pid.Kd * (qd_dot - q_dot.data)
                     + G.data;

            // 判断是否到达当前目标
            double dist = (qd - q.data).norm(); 
            if (dist < 5e-2) 
            {
                // 输出提示信息
                ROS_INFO_STREAM("Arrived at goal [" << current_goal_idx << "]!");
                // 切换到下一个目标
                current_goal_idx++;

                // 若还有后续目标，就重新setup规划器
                if (current_goal_idx < (int)goals.size()) {
                    trapez_planner.setup(q.data, goals[current_goal_idx]);
                } else {
                    // 全部目标完成
                    ROS_INFO_STREAM("All goals completed!");
                }
            }
        }
        else
        {
            // 所有目标都已到达，此处可让力矩归零或做其他处理
            torque.setZero();
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

    // KDL相关
    KDL::Tree  system_tree;
    KDL::Chain arm_chain;
    KDL::Vector gravity;

    // 动力学
    KDL::ChainDynParam         dynamics_kdl;
    KDL::ChainJntToJacSolver   jacobian_kdl;
    KDL::ChainFkSolverPos_recursive fk_kdl;
    KDL::Frame fk_frame;

    // 关节数
    uint n;

    // 关节状态
    KDL::JntArray q;
    KDL::JntArray q_dot;

    // 动力学矩阵
    KDL::JntArray G;  // 重力
    KDL::JntArray C;  // 科氏力
    KDL::JntSpaceInertiaMatrix M;
    KDL::Jacobian J;

    // 多个目标位置
    std::vector<Eigen::VectorXd> goals;
    int current_goal_idx;  // 当前目标索引

    // 期望关节角度、角速度
    Eigen::VectorXd qd;
    Eigen::VectorXd qd_dot;

    // 力矩
    Eigen::VectorXd torque;

    // PID
    PID position_pid;

    // 梯形速度规划器
    TrapezPlanner trapez_planner;

    // ROS相关
    std::vector<ros::Publisher> torque_control_pub;
    std::vector<std_msgs::Float64> torque_msg;
    ros::Subscriber joint_states_sub;
};

// ---------------------- 切换到力矩控制模式 ----------------------
void setupControllerManager(ros::NodeHandle & nh)
{
    // 创建客户端，用于调用/switch_controller服务
    ros::ServiceClient switchTorqueController
        = nh.serviceClient<controller_manager_msgs::SwitchController>(
            "/iiwa/controller_manager/switch_controller");

    // 准备SwitchController服务请求
    controller_manager_msgs::SwitchController srv;
    std::vector<std::string> start_controllers(7);
    std::stringstream ss;
    for (size_t i = 0; i < 7; ++i) {
        ss.str("");
        ss << "joint" << (i + 1) << "_torque_controller";
        start_controllers[i] = ss.str();
    }
    srv.request.start_controllers = start_controllers;
    srv.request.strictness = srv.request.BEST_EFFORT;

    // 调用服务，启动关节力矩控制器
    if (switchTorqueController.call(srv))
        ROS_INFO("Switched to torque controllers successfully");
    else
        ROS_WARN("Failed to switch controllers!");
}

// ---------------------- 主函数 ----------------------
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "iiwa_control");
    ros::NodeHandle nh;

    ROS_INFO("Start iiwa_control node...");

    // 1. 切换到力矩控制模式
    setupControllerManager(nh);

    // 2. 从参数服务器读取robot_description，构建 KDL Tree & Chain
    std::string base_link = "iiwa_link_0";
    std::string end_effector = "tool_link_ee_kuka";
    std::string robot_description = "robot_description";
    KDL::Tree system_tree;
    KDL::Chain arm_chain;
    KDL::Vector gravity(0, 0, -9.81);

    if (!kdl_parser::treeFromParam(robot_description, system_tree)) {
        ROS_ERROR_STREAM("Failed to import robot from param: " << robot_description);
        return 1;
    }
    if (!system_tree.getChain(base_link, end_effector, arm_chain)) {
        ROS_ERROR_STREAM("Failed to get chain from " << base_link << " to " << end_effector);
        return 1;
    }

    // 3. 创建 iiwa 对象
    iiwa iiwa_robot(nh, system_tree, arm_chain, gravity);

    // 4. 回调循环
    ros::spin();
    return 0;
}

