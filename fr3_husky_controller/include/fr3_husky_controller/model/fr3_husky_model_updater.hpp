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

#include "fr3_husky_controller/model/fr3_model_updater.hpp"

#include "dyros_robot_controller/mobile_manipulator/robot_data.h"
#include "dyros_robot_controller/mobile_manipulator/robot_controller.h"

namespace fr3_husky_controller
{

class FR3HuskyModelUpdater final : public ModelUpdaterBase
{
    public:
        FR3HuskyModelUpdater() = default;

        bool initialize(size_t num_robots,
                        size_t manipulator_dof,
                        double dt,
                        const std::vector<std::string>& ee_names) override;
        void setDRCRobotData(const std::shared_ptr<drc::MobileManipulator::RobotData>&& robot_data) { robot_data_ = std::move(robot_data); }
        void setDRCRobotController(const std::shared_ptr<drc::MobileManipulator::RobotController>&& robot_controller) { robot_controller_ = std::move(robot_controller); }
        void updateJointStates() override;
        void updateRobotData() override;
        void haltCommands() override;
        bool getHandlesReady() const override { return !(robot_handle_.mani_joints.empty() || robot_handle_.left_wheels.empty() || robot_handle_.right_wheels.empty()); }
        void setInitFromCurrent();
        void writeCommand(const Eigen::VectorXd& command_mani, 
                        const Eigen::Vector2d& command_mobi);
        void forceStopMobile();
    
    public:
        std::map<std::string, drc::TaskSpaceData> ee_data_;
        std::shared_ptr<drc::MobileManipulator::RobotData> robot_data_;
        std::shared_ptr<drc::MobileManipulator::RobotController> robot_controller_;

        // Initial
        std::vector<Eigen::Vector<double, FR3_DOF>> q_init_;
        std::vector<Eigen::Vector<double, FR3_DOF>> qdot_init_;
        std::vector<Eigen::Vector<double, FR3_DOF>> qddot_init_;
        Eigen::VectorXd q_total_init_;
        Eigen::VectorXd qdot_total_init_;
        Eigen::VectorXd qddot_total_init_;

        // Current
        std::vector<Eigen::Vector<double, FR3_DOF>> q_;
        std::vector<Eigen::Vector<double, FR3_DOF>> qdot_;
        std::vector<Eigen::Vector<double, FR3_DOF>> qddot_;
        std::vector<Eigen::Vector<double, FR3_DOF>> torque_;
        Eigen::VectorXd q_total_;
        Eigen::VectorXd qdot_total_;
        Eigen::VectorXd qddot_total_;
        Eigen::VectorXd torque_total_;

        // Desired
        std::vector<Eigen::Vector<double, FR3_DOF>> q_desired_;
        std::vector<Eigen::Vector<double, FR3_DOF>> qdot_desired_;
        std::vector<Eigen::Vector<double, FR3_DOF>> torque_desired_;
        Eigen::VectorXd q_desired_total_;
        Eigen::VectorXd qdot_desired_total_;
        Eigen::VectorXd torque_desired_total_;

        // Dynamics
        std::vector<Eigen::Matrix<double, FR3_DOF, FR3_DOF>> M_;
        std::vector<Eigen::Matrix<double, FR3_DOF, FR3_DOF>> M_inv_;
        std::vector<Eigen::Vector<double, FR3_DOF>> c_;
        std::vector<Eigen::Vector<double, FR3_DOF>> g_;
        Eigen::MatrixXd M_total_;
        Eigen::MatrixXd M_inv_total_;
        Eigen::VectorXd c_total_;
        Eigen::VectorXd g_total_;

        // Pose (world) and velocity (world/base)
        Eigen::Affine2d base_pose_w_;
        Eigen::Affine2d base_pose_w_desired_;
        Eigen::Affine2d base_pose_w_init_;

        Eigen::Vector3d base_vel_w_;
        Eigen::Vector3d base_vel_w_desired_;
        Eigen::Vector3d base_vel_w_init_;

        Eigen::Vector3d base_vel_b_;
        Eigen::Vector3d base_vel_b_desired_;
        Eigen::Vector3d base_vel_b_init_;

        // Wheels and command
        Eigen::Vector2d wheel_pos_; // (left, right)
        Eigen::Vector2d wheel_vel_; // (left, right)
        Eigen::Vector2d wheel_vel_desired_; // (left, right)
};

}  // namespace fr3_husky_controller
