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

#include "fr3_husky_controller/model/model_updater_base.hpp"

#include "dyros_robot_controller/manipulator/robot_data.h"


namespace fr3_husky_controller
{

class FR3ModelUpdater final : public ModelUpdaterBase
{
    public:
        FR3ModelUpdater() = default;
        bool initialize(size_t num_robots,
                        size_t manipulator_dof,
                        double dt,
                        const std::vector<std::string>& ee_names) override;
        void setDRCRobotData(const std::shared_ptr<drc::Manipulator::RobotData>&& robot_data) { robot_data_ = std::move(robot_data); }
        void updateJointStates() override;
        void updateRobotData() override;
        void haltCommands() override;
        bool getHandlesReady() const override { return !robot_handle_.mani_joints.empty(); }
        void setInitFromCurrent();
        void writeCommand(const Eigen::VectorXd& command);

        ManipulatorState& getManipulatorState()             { return mani_state_; }
        const ManipulatorState& getManipulatorState() const { return mani_state_; }

        std::map<std::string, drc::TaskSpaceData>& getEEData()             { return ee_data_; }
        const std::map<std::string, drc::TaskSpaceData>& getEEData() const { return ee_data_; }

        std::shared_ptr<drc::Manipulator::RobotData> getRobotData() { return robot_data_; }

    private:
        std::vector<std::string> ee_names_;
        std::map<std::string, drc::TaskSpaceData> ee_data_;

        std::shared_ptr<drc::Manipulator::RobotData> robot_data_;

        ManipulatorState mani_state_;
};

}  // namespace fr3_husky_controller
