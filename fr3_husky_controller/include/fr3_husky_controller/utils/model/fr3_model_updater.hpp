#pragma once

#include <array>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <franka_semantic_components/franka_robot_model.hpp>

#include "dyros_robot_controller/manipulator/robot_data.h"

#ifndef FR3_DOF
#define FR3_DOF 7
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

struct JointHandle
{
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> state;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> command;

    explicit JointHandle(std::reference_wrapper<hardware_interface::LoanedCommandInterface> cmd)
        : command(cmd) {}
};

class FR3ModelUpdater
{
public:
    FR3ModelUpdater() = default;

    void setNode(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);
    void setInterfaceFlags(bool has_position_state_interface,
                           bool has_velocity_state_interface,
                           bool has_effort_state_interface,
                           bool has_position_command_interface,
                           bool has_velocity_command_interface,
                           bool has_effort_command_interface);
    void setModelSources(const std::shared_ptr<drc::Manipulator::RobotData>& robot_data,
                         std::vector<std::unique_ptr<franka_semantic_components::FrankaRobotModel>>* franka_robot_model);
    bool initialize(size_t num_robots,
                    size_t manipulator_dof,
                    double dt,
                    const std::vector<std::string>& ee_names);

    void setJointHandles(std::vector<JointHandle>&& joint_handles);
    void updateJointStates();
    void updateRobotModelAndData();
    void setInitFromCurrent();
    void writeCommand(const Eigen::VectorXd& command);
    void haltCommands();

    ManipulatorState& getState() { return mani_state_; }
    const ManipulatorState& getState() const { return mani_state_; }
    
    std::vector<std::string>& getEEName() { return ee_names_; }
    const std::vector<std::string>& getEEName() const { return ee_names_; }

    std::map<std::string, drc::TaskSpaceData>& getEEData() { return ee_data_; }
    const std::map<std::string, drc::TaskSpaceData>& getEEData() const { return ee_data_; }

    std::shared_ptr<drc::Manipulator::RobotData> getRobotData() { return robot_data_; }
    bool getHandlesReady() const { return !joint_handles_.empty(); }

    const bool HasPositionStateInterface()   { return has_position_state_interface_; }
    const bool HasVelocityStateInterface()   { return has_velocity_state_interface_; }
    const bool HasEffortStateInterface()     { return has_effort_state_interface_; }
    const bool HasPositionCommandInterface() { return has_position_command_interface_; }
    const bool HasVelocityCommandInterface() { return has_velocity_command_interface_; }
    const bool HasEffortCommandInterface()   { return has_effort_command_interface_; }

    const int getNumRobots() { return (int)(num_robots_); }
    const int getManipulatorDOF() { return (int)(manipulator_dof_); }
    const double getDT() { return dt_; }

private:
    static constexpr size_t kPositionIndex = 0;
    static constexpr size_t kVelocityIndex = 1;
    static constexpr size_t kEffortIndex   = 2;

    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

    size_t num_robots_{0};
    size_t manipulator_dof_{0};
    double dt_{0.0};
    const std::string arm_id_{"fr3"};

    bool has_position_state_interface_{false};
    bool has_velocity_state_interface_{false};
    bool has_effort_state_interface_{false};

    bool has_position_command_interface_{false};
    bool has_velocity_command_interface_{false};
    bool has_effort_command_interface_{false};

    std::vector<std::string> ee_names_;
    std::map<std::string, drc::TaskSpaceData> ee_data_;

    std::vector<JointHandle> joint_handles_;
    std::vector<std::unique_ptr<franka_semantic_components::FrankaRobotModel>>* franka_robot_model_{nullptr};
    std::shared_ptr<drc::Manipulator::RobotData> robot_data_;

    ManipulatorState mani_state_;
    std::mutex robot_data_mutex_;
    bool is_configured_{false};

    bool halt_initialized_{false};
    std::map<std::string, double> halt_position_; // [joint_name][initial pos]
};

}  // namespace fr3_husky_controller
