#pragma once

#include <atomic>
#include <memory>
#include <type_traits>

#include <action_msgs/msg/goal_status.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <fr3_husky_msgs/action/move_to_joint.hpp>
#include <fr3_husky_msgs/action/vive_tracker.hpp>

#include <fr3_husky_controller/servers/action_server_base.hpp>
#include <fr3_husky_controller/model/fr3_model_updater.hpp>
#include <fr3_husky_controller/utils/dyros_math.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

namespace fr3_husky_controller::servers::fr3
{

class ViveTracker final : public ActionServerBase<fr3_husky_msgs::action::ViveTracker>
{
public:
    using ActionT = fr3_husky_msgs::action::ViveTracker;
    using Base = ActionServerBase<ActionT>;
    using ComputeResult = typename Base::ComputeResult;
    using StopReason = typename Base::StopReason;
    using ResultPtr = typename Base::ResultPtr;

    ViveTracker(const std::string& name, const NodePtr& node, ModelUpdaterBase& model_updater);
    ~ViveTracker() override = default;

    int priority() const override { return 7; }
    bool allowPreemption() const override { return true; }

private:
    bool acceptGoal(const ActionT::Goal& goal) override;
    void onGoalAccepted(const ActionT::Goal& goal) override;
    void onStart() override;
    ComputeResult compute(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    void onStop(StopReason reason) override;
    ResultPtr makeResult(StopReason reason) override;

private:
    FR3ModelUpdater& fr3_model_updater_;

private:
    // publishers & subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr  pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr l_button_state_sub_; // off: 0 | on: 1, button idx:[trigger, grip, a, b]
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr r_button_state_sub_; // off: 0 | on: 1, button idx:[trigger, grip, a, b]

    void subPoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void subLButtonCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    void subRButtonCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

    // vive controller state data
    std::vector<Eigen::Affine3d> tracker_poses_;      // left, right, head
    std::vector<Eigen::Affine3d> tracker_poses_init_; // left, right, head
    std::vector<std::vector<bool>> button_states_;    // [left, right][trigger, grip, a, b]
    std::vector<bool> is_mouse_mode_on_;              // left, right

    std::vector<Eigen::Matrix3d> tracker_base2robot_base_;

    // robot data
    std::map<std::string, drc::TaskSpaceData> ee_data_;

    // action goal data
    int control_mode_;                  // 0: CLIK, 1: OSF, 2:QPIK, 3:QPID
    std::string control_left_ee_name_;  // EE name for tracking left vive controller twist 
    std::string control_right_ee_name_; // EE name for tracking right vive controller twist 
    bool move_ori_;
    double tracker_pos_multiplier_;
    double tracker_ori_multiplier_;

    std::mutex tracker_pose_mutex_;
    std::mutex button_state_mutex_;

    // initialize mode: button A -> send goal to fr3_move_to_joint
    bool is_initialize_mode_on_{false};
    const Eigen::Vector<double, FR3_DOF> HomePose{0., -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
    bool prev_l_button0_{false};
    bool prev_r_button0_{false};
    ActionT::Goal saved_vive_goal_{};  // saved goal params for auto-resume after init

    // action clients
    using MoveToJointAction = fr3_husky_msgs::action::MoveToJoint;
    rclcpp_action::Client<MoveToJointAction>::SharedPtr move_to_joint_client_;
    rclcpp_action::Client<ActionT>::SharedPtr           vt_self_client_;  // self-client for resume

    // JTC completion monitoring: wait for JTC to finish executing before re-activating
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr jtc_status_sub_;
    std::atomic<bool> waiting_for_jtc_{false};
};

}  // namespace fr3_husky_controller::servers::fr3
