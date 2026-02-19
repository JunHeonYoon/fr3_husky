#include <fr3_husky_controller/servers/gravity_compensation_action_server.hpp>

namespace fr3_husky_controller::servers
{

GravityCompensation::GravityCompensation(const std::string& name, const NodePtr& node, FR3ModelUpdater& model_updater)
: ActionServerBase(name, node, model_updater)
{
    using std::placeholders::_1;
    using std::placeholders::_2;

    server_ = rclcpp_action::create_server<ActionT>(
        node_,
        name_,  // action name
        std::bind(&GravityCompensation::handle_goal, this, _1, _2),
        std::bind(&GravityCompensation::handle_cancel, this, _1),
        std::bind(&GravityCompensation::handle_accepted, this, _1));

    robot_controller_ = std::make_unique<drc::Manipulator::RobotController>(model_updater_.getRobotData());

    RCLCPP_INFO(node_->get_logger(), "[%s] GravityCompensation created", name_.c_str());
}

bool GravityCompensation::isActive() const
{
    std::lock_guard<std::mutex> lk(mutex_);
    return static_cast<bool>(active_goal_);
}

int GravityCompensation::priority() const
{
    return 0;
}

bool GravityCompensation::compute(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    std::shared_ptr<GoalHandle> gh;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        gh = active_goal_;
    }
    if (!gh) return true;

    if(model_updater_.HasEffortCommandInterface())
    {
        // for gravity compensation using QPID, set desired acceleration & tracking weight as zero (dummy data)
        std::map<std::string, drc::TaskSpaceData> zero_data;
        std::map<std::string, Eigen::Vector6d> zero_qpid_tracking;
        for (const auto & ee_name : model_updater_.getEEName())
        {
            zero_data[ee_name] = drc::TaskSpaceData::Zero();
            zero_qpid_tracking[ee_name] = Eigen::Vector6d::Zero();
        }
        robot_controller_->setQPIDTrackingGain(zero_qpid_tracking);
    
        Eigen::VectorXd opt_torque;
        opt_torque.setZero(model_updater_.getManipulatorDOF());

        std::string time_verbose;
        const bool qp_ok = robot_controller_->QPID(zero_data, opt_torque, time_verbose);

        if(qp_ok)
        {
            model_updater_.getState().torque_desired_total = opt_torque - model_updater_.getState().g_total;
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "[%s] QPID solve failed! Gravity Compensation WO QPID", name_.c_str());
            time_verbose.clear();
            model_updater_.getState().torque_desired_total.setZero();
        }

        model_updater_.writeCommand(model_updater_.getState().torque_desired_total);

        auto fb = std::make_shared<ActionT::Feedback>();
        fb->is_qp_solved = qp_ok;
        fb->time_verbose = time_verbose;
        gh->publish_feedback(fb);
    }
    else
    {
        RCLCPP_WARN(node_->get_logger(), "[%s] Command Interface expected as [Effort], but [%s]! Halted Command", name_.c_str(), model_updater_.HasPositionCommandInterface() ? "Position" : "Velocity");
        model_updater_.haltCommands();

        auto fb = std::make_shared<ActionT::Feedback>();
        fb->is_qp_solved = false;
        fb->time_verbose = "";
        gh->publish_feedback(fb);
    }

    return true;
}

void GravityCompensation::onActivated()
{
    RCLCPP_INFO(node_->get_logger(), "[%s] activated(owner)", name_.c_str());
}

void GravityCompensation::onDeactivated()
{
    std::shared_ptr<GoalHandle> goal_to_finalize;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        goal_to_finalize = active_goal_;
        active_goal_.reset();
    }

    if (goal_to_finalize)
    {
        auto result = std::make_shared<ActionT::Result>();
        result->is_completed = true;

        // If a cancel request is in progress, finalize as canceled.
        if (goal_to_finalize->is_canceling())
        {
            goal_to_finalize->canceled(result);
        }
        else
        {
            goal_to_finalize->succeed(result);
        }
    }
    RCLCPP_INFO(node_->get_logger(), "[%s] deactivated(owner released)", name_.c_str());
}

rclcpp_action::GoalResponse GravityCompensation::handle_goal(const rclcpp_action::GoalUUID& /*uuid*/, std::shared_ptr<const ActionT::Goal> /*goal*/)
{
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GravityCompensation::handle_cancel(const std::shared_ptr<GoalHandle> /*goal_handle*/)
{
    // Callback thread -> controller thread
    requestCancel();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void GravityCompensation::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
    auto goal = goal_handle->get_goal();

    // enable=true: this goal becomes the active owner candidate.
    if (goal->enable)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        active_goal_ = goal_handle;
        requestActivate();
        return;
    }

    // enable=false: request owner release, but do not overwrite active_goal_.
    requestCancel();
    auto result = std::make_shared<ActionT::Result>();
    result->is_completed = true;
    goal_handle->succeed(result);
}

// Register this server into global registry (executed when this TU is linked)
REGISTER_FR3_ACTION_SERVER(GravityCompensation, "fr3_gravity_compensation")

}  // namespace fr3_husky_controller::servers
