#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <array>
#include <vector>

#include <Eigen/Eigen>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <franka_semantic_components/franka_robot_model.hpp>

#ifndef FR3_DOF
#define FR3_DOF 7
#endif

#ifndef WHEEL_PER_SIDE
#define WHEEL_PER_SIDE 2 // front, rear
#endif

namespace fr3_husky_controller
{
struct ManipulatorState
{
    // Initial
    std::vector<Eigen::Vector<double, FR3_DOF>> q_init;
    std::vector<Eigen::Vector<double, FR3_DOF>> qdot_init;
    std::vector<Eigen::Vector<double, FR3_DOF>> qddot_init;
    Eigen::VectorXd q_total_init;
    Eigen::VectorXd qdot_total_init;
    Eigen::VectorXd qddot_total_init;

    // Current
    std::vector<Eigen::Vector<double, FR3_DOF>> q;
    std::vector<Eigen::Vector<double, FR3_DOF>> qdot;
    std::vector<Eigen::Vector<double, FR3_DOF>> qddot;
    std::vector<Eigen::Vector<double, FR3_DOF>> torque;
    Eigen::VectorXd q_total;
    Eigen::VectorXd qdot_total;
    Eigen::VectorXd qddot_total;
    Eigen::VectorXd torque_total;

    // Desired
    std::vector<Eigen::Vector<double, FR3_DOF>> q_desired;
    std::vector<Eigen::Vector<double, FR3_DOF>> qdot_desired;
    std::vector<Eigen::Vector<double, FR3_DOF>> torque_desired;
    Eigen::VectorXd q_desired_total;
    Eigen::VectorXd qdot_desired_total;
    Eigen::VectorXd torque_desired_total;

    // Dynamics
    std::vector<Eigen::Matrix<double, FR3_DOF, FR3_DOF>> M;
    std::vector<Eigen::Matrix<double, FR3_DOF, FR3_DOF>> M_inv;
    std::vector<Eigen::Vector<double, FR3_DOF>> c;
    std::vector<Eigen::Vector<double, FR3_DOF>> g;
    Eigen::MatrixXd M_total;
    Eigen::MatrixXd M_inv_total;
    Eigen::VectorXd c_total;
    Eigen::VectorXd g_total;
};

struct MobileState
{
    // Pose (world) and velocity (world/base)
    Eigen::Affine2d base_pose_w;
    Eigen::Affine2d base_pose_w_desired;
    Eigen::Affine2d base_pose_w_init;

    Eigen::Vector3d base_vel_w;
    Eigen::Vector3d base_vel_w_desired;
    Eigen::Vector3d base_vel_w_init;

    Eigen::Vector3d base_vel_b;
    Eigen::Vector3d base_vel_b_desired;
    Eigen::Vector3d base_vel_b_init;

    // Wheels and command
    Eigen::Vector2d wheel_pos; // (left, right)
    Eigen::Vector2d wheel_vel; // (left, right)
    Eigen::Vector2d wheel_vel_desired; // (left, right)
};

struct JointHandle
{
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> state;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> command;
    
    explicit JointHandle(std::reference_wrapper<hardware_interface::LoanedCommandInterface> cmd)
    : command(cmd)
    {
    }
};

struct WheelHandle
{
    std::vector<std::reference_wrapper<const hardware_interface::LoanedStateInterface>> state; // position, velocity
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> command; // only velocity
};

struct RobotHandle
{
  std::vector<JointHandle> mani_joints;

  std::vector<WheelHandle> left_wheels;
  std::vector<WheelHandle> right_wheels;

};

class ModelUpdaterBase
{
    public:
        virtual ~ModelUpdaterBase();

        void setNode(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) { node_ = node; }
        void setInterfaceFlags(bool has_position_state_interface,
                               bool has_velocity_state_interface,
                               bool has_effort_state_interface,
                               bool has_position_command_interface,
                               bool has_velocity_command_interface,
                               bool has_effort_command_interface);
        virtual bool initialize(size_t num_robots,
                                size_t manipulator_dof,
                                double dt, 
                                const std::vector<std::string>& ee_names);
        void setFrankaModel(std::vector<std::unique_ptr<franka_semantic_components::FrankaRobotModel>>* franka_robot_model) { franka_robot_model_ = franka_robot_model; }
        void setRobotHandles(RobotHandle&& robot_handle) { robot_handle_ = std::move(robot_handle); }
        virtual void updateJointStates() = 0;
        virtual void updateRobotData() = 0;
        virtual void haltCommands() = 0;
        virtual bool getHandlesReady() const = 0;

        const bool HasPositionStateInterface()   { return has_position_state_interface_; }
        const bool HasVelocityStateInterface()   { return has_velocity_state_interface_; }
        const bool HasEffortStateInterface()     { return has_effort_state_interface_; }
        const bool HasPositionCommandInterface() { return has_position_command_interface_; }
        const bool HasVelocityCommandInterface() { return has_velocity_command_interface_; }
        const bool HasEffortCommandInterface()   { return has_effort_command_interface_; }

        const int getNumRobots() { return (int)(num_robots_); }
        const int getManipulatorDOF() { return (int)(manipulator_dof_); }
        const double getDT() { return dt_; }

        std::vector<std::string>& getEEName()             { return ee_names_; }
        const std::vector<std::string>& getEEName() const { return ee_names_; }

        std::vector<std::unique_ptr<franka_semantic_components::FrankaRobotModel>>*getFrankaRobotModel() { return franka_robot_model_; }

    protected:
        int jointNameToIndex(const std::string& iface_name);
        
    protected:
        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

        size_t num_robots_{0}; // number of FR3 arms
        size_t manipulator_dof_{0};
        const size_t mobile_dof_{2};
        const size_t virtual_dof_{3};
        double dt_{0.0};
        const std::string arm_id_{"fr3"};
        std::vector<std::string> ee_names_;


        std::mutex robot_data_mutex_;
        
        bool is_configured_{false};
        bool halt_initialized_{false};
        std::map<std::string, double> halt_position_; // [joint_name][initial pos], for manipulator

        std::vector<std::unique_ptr<franka_semantic_components::FrankaRobotModel>>* franka_robot_model_{nullptr};
        RobotHandle robot_handle_;

        static constexpr size_t kPositionIndex         = 0;
        static constexpr size_t kVelocityIndex         = 1;
        static constexpr size_t kEffortIndex           = 2;
        static constexpr size_t kFeedbackPositionIndex = 0;
        static constexpr size_t kFeedbackVelocityIndex = 1;
        static constexpr size_t kFeedbackEffortIndex   = 2;

        bool has_position_state_interface_{false};
        bool has_velocity_state_interface_{false};
        bool has_effort_state_interface_{false};
        bool has_position_command_interface_{false};
        bool has_velocity_command_interface_{false};
        bool has_effort_command_interface_{false};

};

}  // namespace fr3_husky_controller
