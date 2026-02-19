#pragma once

#include <mutex>
#include <memory>

#include <rclcpp_action/rclcpp_action.hpp>
#include <fr3_husky_msgs/action/gravity_compensation.hpp>

#include <fr3_husky_controller/servers/action_server_base.hpp>

#include "dyros_robot_controller/manipulator/robot_controller.h"

namespace fr3_husky_controller::servers
{

class GravityCompensation final : public ActionServerBase
{
public:
    using ActionT = fr3_husky_msgs::action::GravityCompensation;
    using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;

    GravityCompensation(const std::string& name, const NodePtr& node, FR3ModelUpdater& model_updater);
    ~GravityCompensation() override = default;

    // Controller(owner) queries
    bool isActive() const override;
    int priority() const override;

    // Called ONLY when this server is owner in controller update-loop
    bool compute(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    void onActivated() override;
    void onDeactivated() override;

private:
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const ActionT::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);

private:
    std::shared_ptr<rclcpp_action::Server<ActionT>> server_;

    mutable std::mutex mutex_;
    std::shared_ptr<GoalHandle> active_goal_;

private:
    std::unique_ptr<drc::Manipulator::RobotController> robot_controller_;
};

}  // namespace fr3_husky_controller::servers
