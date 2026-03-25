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

#define NUM_TRACKERS    3 // left, right, head
#define NUM_CONTROLLERS 2 // left, right (only Focus3 controllers)
#define IDX_LEFT_CON    0 // index of left controller pose in "tracker_pose" topic
#define IDX_RIGHT_CON   1 // index of right controller pose in "tracker_pose" topic
#define IDX_HEAD_CON    2 // index of HMD head pose in "tracker_pose" topic

#define NUM_BUTTONS        4 // number of buttons in Focu3 controller [trigger, grip, a, b]
#define IDX_TRIGGER_BUTTON 0 // index of Trigger button in "l/rhand_button" topic
#define IDX_GRIP_BUTTON    1 // index of Trigger button in "l/rhand_button" topic
#define IDX_A_BUTTON       2 // index of Trigger button in "l/rhand_button" topic
#define IDX_B_BUTTON       3 // index of Trigger button in "l/rhand_button" topic

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
    std::vector<Eigen::Affine3d> controller_poses_;      // left, right, head
    std::vector<Eigen::Affine3d> controller_poses_init_; // left, right, head
    std::vector<std::vector<bool>> button_states_;       // [left, right][trigger, grip, a, b]
    std::vector<std::vector<bool>> prev_button_states_;  // previous button states for edge detection

    // states for ...
    std::vector<bool> is_mouse_mode_on_{false, false};
    bool is_initialize_mode_on_{false};
    std::vector<bool> is_gripper_mode_on_{false, false};
    
    // robot data
    std::vector<Eigen::Matrix3d> tracker_base2robot_base_;
    std::map<std::string, drc::TaskSpaceData> ee_data_;

    // action goal data
    int control_mode_;                     // 0: CLIK, 1: OSF, 2:QPIK, 3:QPID
    std::string left_controller_ee_name_;  // EE name for tracking left vive controller
    std::string right_controller_ee_name_; // EE name for tracking right vive controller
    bool move_ori_;
    double controller_pos_multiplier_;
    double controller_ori_multiplier_;

    std::mutex tracker_pose_mutex_;
    std::mutex button_state_mutex_;

    // initialize mode: button A -> send goal to fr3_move_to_joint
    const Eigen::Vector<double, FR3_DOF> HomePose{0., -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
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
