#include <algorithm>
#include <numeric>
#include <sstream>
#include <fr3_husky_controller/utils/robot_utils.hpp>
#include <franka/lowpass_filter.h>

#include "fr3_husky_controller/test_single_fr3_controller.hpp"

namespace
{
bool poseIsFinite(const geometry_msgs::msg::Pose& pose)
{
    return std::isfinite(pose.position.x) &&
           std::isfinite(pose.position.y) &&
           std::isfinite(pose.position.z) &&
           std::isfinite(pose.orientation.x) &&
           std::isfinite(pose.orientation.y) &&
           std::isfinite(pose.orientation.z) &&
           std::isfinite(pose.orientation.w);
}

Eigen::Affine3d poseMsgToEigen(const geometry_msgs::msg::Pose& pose)
{
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    if (q.norm() < 1e-9)
    {
        q.setIdentity();
    }
    else
    {
        q.normalize();
    }
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.linear() = q.toRotationMatrix();
    transform.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    return transform;
}

geometry_msgs::msg::Pose eigenToPoseMsg(const Eigen::Affine3d& pose)
{
    geometry_msgs::msg::Pose msg;
    msg.position.x = pose.translation().x();
    msg.position.y = pose.translation().y();
    msg.position.z = pose.translation().z();
    Eigen::Quaterniond q(pose.linear());
    q.normalize();
    msg.orientation.x = q.x();
    msg.orientation.y = q.y();
    msg.orientation.z = q.z();
    msg.orientation.w = q.w();
    return msg;
}
}  // namespace

namespace FR3HuskyController 
{
// ========================================================================
// ============================ Core Functions ============================
// ========================================================================
controller_interface::InterfaceConfiguration TestSingleFR3Controller::command_interface_configuration() const 
{
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    conf.names.reserve(num_cmd_joints_ * params_.command_interfaces.size());
    for (const auto & joint_name : command_joint_names_)
    {
        for (const auto & interface_type : params_.command_interfaces)
        {
            conf.names.push_back(joint_name + "/" + interface_type);
        }
    }
    return conf;
}

controller_interface::InterfaceConfiguration TestSingleFR3Controller::state_interface_configuration() const 
{
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    conf.names.reserve(dof_ * params_.state_interfaces.size());
    for (const auto & joint_name : params_.joints)
    {
        for (const auto & interface_type : params_.state_interfaces)
        {
        conf.names.push_back(joint_name + "/" + interface_type);
        }
    }
    return conf;
}

CallbackReturn TestSingleFR3Controller::on_init() 
{
    try
    {
        // Create the parameter listener and get the parameters
        param_listener_ = std::make_shared<ParamListener>(get_node());
        params_ = param_listener_->get_params();
    }
    catch (const std::exception & e)
    {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return CallbackReturn::ERROR;
    }

    auto node = get_node();
    node->declare_parameter("arm_id", arm_id_);
    node->declare_parameter("dyros.task_link", default_task_link_);
    node->declare_parameter("dyros.joint_kp", std::vector<double>{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0});
    node->declare_parameter("dyros.joint_kv", std::vector<double>{30.0, 30.0, 30.0, 30.0, 10.0, 10.0, 5.0});
    node->declare_parameter("dyros.task_kp", std::vector<double>{100.0, 100.0, 100.0, 100.0, 100.0, 100.0});
    node->declare_parameter("dyros.task_kv", std::vector<double>{20.0, 20.0, 20.0, 20.0, 20.0, 20.0});
    node->declare_parameter("dyros.qpik_tracking", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    node->declare_parameter("dyros.qpik_damping", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    node->declare_parameter("dyros.qpid_tracking", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    node->declare_parameter("dyros.qpid_vel_damping", std::vector<double>{0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
    node->declare_parameter("dyros.qpid_acc_damping", std::vector<double>{0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01});

    const std::string & urdf = get_robot_description();
    if (!urdf.empty())
    {
        urdf::Model model;
        if (!model.initString(urdf))
        {
            LOGE(get_node(), "Failed to parse robot description!");
            return CallbackReturn::ERROR;
        }
        else
        {
        /// initialize the URDF model and update the joint angles wraparound vector
        // Configure joint position error normalization (angle_wraparound)
        joints_angle_wraparound_.resize(params_.joints.size(), false);
        for (size_t i = 0; i < params_.joints.size(); ++i)
        {
            auto urdf_joint = model.getJoint(params_.joints[i]);
            if (urdf_joint && urdf_joint->type == urdf::Joint::CONTINUOUS)
            {
                LOGI(get_node(), "joint '%s' is of type continuous, use angle_wraparound.", params_.joints[i].c_str());
                joints_angle_wraparound_[i] = true;
            }
            // do nothing if joint is not found in the URDF
        }
        RCLCPP_DEBUG(get_node()->get_logger(), "Successfully parsed URDF file");
        }
    }
    else
    {
        // empty URDF is used for some tests
        RCLCPP_DEBUG(get_node()->get_logger(), "No URDF file given");
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn TestSingleFR3Controller::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) 
{
    // update the dynamic map parameters
    param_listener_->refresh_dynamic_parameters();

    // get parameters from the listener in case they were updated
    params_ = param_listener_->get_params();

    // get degrees of freedom
    dof_ = params_.joints.size();
    if (dof_ != 7)
    {
        LOGE(get_node(), "Single FR3 controller expects 7 DoF, but got %zu.", dof_);
        return CallbackReturn::FAILURE;
    }

    get_node()->get_parameter("arm_id", arm_id_);

    command_joint_names_ = params_.command_joints;

    if (command_joint_names_.empty())
    {
        command_joint_names_ = params_.joints;
        LOGI(get_node(), "No specific joint names are used for command interfaces. Using 'joints' parameter.");
    }
    num_cmd_joints_ = command_joint_names_.size();

    if (num_cmd_joints_ > dof_)
    {
        LOGE(get_node(), "The 'command_joints' parameter must not exceed the size of the 'joints' parameter.");
        return CallbackReturn::FAILURE;
    }
    else if (num_cmd_joints_ < dof_)
    {
        // create a map for the command joints
        map_cmd_to_joints_ = robot_utils::mapping(command_joint_names_, params_.joints);
        if (map_cmd_to_joints_.size() != num_cmd_joints_)
        {
            LOGE(get_node(), "'command_joints' parameter must be a subset of 'joints' parameter, if their size is not equal.");
            return CallbackReturn::FAILURE;
        }
        for (size_t i = 0; i < command_joint_names_.size(); i++)
        {
            LOGI(get_node(), "Command joint %zu: '%s' maps to joint %zu: '%s'.", i,
                 command_joint_names_[i].c_str(), map_cmd_to_joints_[i],
                 params_.joints.at(map_cmd_to_joints_[i]).c_str());
        }
    }
    else
    {
        // create a map for the command joints, trivial if the size is the same
        map_cmd_to_joints_.resize(num_cmd_joints_);
        std::iota(map_cmd_to_joints_.begin(), map_cmd_to_joints_.end(), 0); // map_cmd_to_joints_ <- {0, 1, 2, ...} 
    }

    if (params_.command_interfaces.empty())
    {
        LOGE(get_node(), "'command_interfaces' parameter is empty.");
        return CallbackReturn::FAILURE;
    }

    // Check if only allowed interface types are used and initialize storage to avoid memory
    // allocation during activation
    joint_command_interface_.resize(allowed_interface_types_.size());
    for (auto & itf : joint_command_interface_)
    {
        itf.reserve(params_.joints.size());
    } 

    has_position_command_interface_     = robot_utils::contains_interface_type(params_.command_interfaces, hardware_interface::HW_IF_POSITION);
    has_velocity_command_interface_     = robot_utils::contains_interface_type(params_.command_interfaces, hardware_interface::HW_IF_VELOCITY);
    has_effort_command_interface_       = robot_utils::contains_interface_type(params_.command_interfaces, hardware_interface::HW_IF_EFFORT);


    if (params_.state_interfaces.empty())
    {
        LOGE(get_node(), "'state_interfaces' parameter is empty.");
        return CallbackReturn::FAILURE;
    }
    // Check if only allowed interface types are used and initialize storage to avoid memory
    // allocation during activation
    joint_state_interface_.resize(allowed_interface_types_.size());
    for (auto & itf : joint_state_interface_)
    {
        itf.reserve(params_.joints.size());
    }

    has_position_state_interface_     = robot_utils::contains_interface_type(params_.state_interfaces, hardware_interface::HW_IF_POSITION);
    has_velocity_state_interface_     = robot_utils::contains_interface_type(params_.state_interfaces, hardware_interface::HW_IF_VELOCITY);
    has_acceleration_state_interface_ = robot_utils::contains_interface_type(params_.state_interfaces, hardware_interface::HW_IF_ACCELERATION);
    has_effort_state_interface_       = robot_utils::contains_interface_type(params_.state_interfaces, hardware_interface::HW_IF_EFFORT);

    // Validation of combinations of state and velocity together have to be done
    // here because the parameter validators only deal with each parameter
    // separately.
    if (has_velocity_command_interface_ && 
        params_.command_interfaces.size() == 1 && 
        (!has_velocity_state_interface_ || !has_position_state_interface_))
    {
        LOGE(get_node(),
            "'velocity' command interface can only be used alone if 'velocity' and "
            "'position' state interfaces are present");
        return CallbackReturn::FAILURE;
    }

    // effort only or effort and position command interfaces require position and velocity state
    if (has_effort_command_interface_ &&
        (params_.command_interfaces.size() == 1 || (params_.command_interfaces.size() == 2 && has_position_command_interface_)) &&
        (!has_velocity_state_interface_ || !has_position_state_interface_))
    {
        LOGE(get_node(),
                "'effort' command interface can only be used alone or with 'position' command interface "
                "if 'velocity' and 'position' state interfaces are present");
        return CallbackReturn::FAILURE;
    }
    if (!has_effort_command_interface_)
    {
        LOGE(get_node(), "This controller requires an 'effort' command interface.");
        return CallbackReturn::FAILURE;
    }
    if (!has_position_state_interface_ || !has_velocity_state_interface_)
    {
        LOGE(get_node(), "This controller requires both 'position' and 'velocity' state interfaces.");
        return CallbackReturn::FAILURE;
    }

    auto get_interface_list = [](const std::vector<std::string> & interface_types)
    {
        std::stringstream ss_interfaces;
        for (size_t index = 0; index < interface_types.size(); ++index)
        {
            if (index != 0)
            {
                ss_interfaces << " ";
            }
            ss_interfaces << interface_types[index];
        }
        return ss_interfaces.str();
    };

    // Print output so users can be sure the interface setup is correct
    LOGI(get_node(), "Command interfaces are [%s] and state interfaces are [%s].",
        get_interface_list(params_.command_interfaces).c_str(),
        get_interface_list(params_.state_interfaces).c_str());

    if (params_.action_monitor_rate <= 0.0)
    {
        LOGW(get_node(), "Invalid action_monitor_rate %.2f Hz, falling back to 20 Hz.", params_.action_monitor_rate);
        action_monitor_period_ = rclcpp::Duration::from_seconds(0.05);
    }
    else
    {
        LOGI(get_node(), "Action status changes will be monitored at %.2f Hz.", params_.action_monitor_rate);
        action_monitor_period_ = rclcpp::Duration::from_seconds(1.0 / params_.action_monitor_rate);
    }

    using namespace std::placeholders;
    action_server_ = rclcpp_action::create_server<MoveLink>(
        get_node()->get_node_base_interface(), get_node()->get_node_clock_interface(),
        get_node()->get_node_logging_interface(), get_node()->get_node_waitables_interface(),
        std::string(get_node()->get_name()) + "/move_link",
        std::bind(&TestSingleFR3Controller::goal_received_callback, this, _1, _2),
        std::bind(&TestSingleFR3Controller::goal_cancelled_callback, this, _1),
        std::bind(&TestSingleFR3Controller::goal_accepted_callback, this, _1));

    if (get_update_rate() == 0)
    {
        throw std::runtime_error("Controller's update rate is set to 0. This should not happen!");
    }
    dt_ = 1.0 / static_cast<double>(get_update_rate());

    // initialize franka robot data
    franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
        arm_id_ + "/robot_model", arm_id_ + "/robot_state");
    
    const std::string robot_model_path = ament_index_cpp::get_package_share_directory("fr3_husky_description");
    robot_data_ = std::make_shared<drc::Manipulator::RobotData>(robot_model_path + "/robots/for_control/single_fr3_w_gripper.urdf",
                                                                robot_model_path + "/robots/for_control/single_fr3_w_gripper.srdf",
                                                                robot_model_path);
    if (robot_data_->getDof() != 7)
    {
        LOGE(get_node(), "Robot model DoF is %d, expected 7 for single FR3.", robot_data_->getDof());
        return CallbackReturn::FAILURE;
    }

    robot_controller_ = std::make_unique<drc::Manipulator::RobotController>(dt_, robot_data_);
    if (!loadDyrosGains())
    {
        return CallbackReturn::FAILURE;
    }

    q_init_.setZero();
    qdot_init_.setZero();
    qddot_init_.setZero();

    q_.setZero();
    qdot_.setZero();
    qddot_.setZero();
    torque_.setZero();

    q_desired_.setZero();
    qdot_desired_.setZero();
    torque_desired_.setZero();

    M_.setIdentity();
    M_inv_.setIdentity();
    c_.setZero();
    g_.setZero();

    x_init_.setIdentity();
    xdot_init_.setZero();

    x_.setIdentity();
    xdot_.setZero();
    J_.setZero();

    M_task_.setIdentity();
    g_task_.setZero();
    J_T_inv_.setZero();
    action_active_ = false;
    active_goal_handle_.reset();
    link_task_data_.clear();

    if (!compute_thread_.joinable())
    {
        std::lock_guard<std::mutex> lock(compute_cv_mutex_);
        stop_compute_thread_ = false;
        compute_requested_ = false;
        compute_completed_ = false;
        compute_thread_ = std::thread(&TestSingleFR3Controller::computeWorkerLoop, this);
    }

  return CallbackReturn::SUCCESS;
}

CallbackReturn TestSingleFR3Controller::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) 
{
    // update the dynamic map parameters
    param_listener_->refresh_dynamic_parameters();

    // get parameters from the listener in case they were updated
    params_ = param_listener_->get_params();

    // order all joints in the storage
    for (const auto & interface : params_.command_interfaces)
    {
        auto it = std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
        auto index = static_cast<size_t>(std::distance(allowed_interface_types_.begin(), it));
        if (!controller_interface::get_ordered_interfaces(command_interfaces_, command_joint_names_, interface, joint_command_interface_[index]))
        {
            LOGE(get_node(), "Expected %zu '%s' command interfaces, got %zu.", num_cmd_joints_, interface.c_str(), joint_command_interface_[index].size());
            return CallbackReturn::ERROR;
        }
    }
    for (const auto & interface : params_.state_interfaces)
    {
        auto it = std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
        auto index = static_cast<size_t>(std::distance(allowed_interface_types_.begin(), it));
        if (!controller_interface::get_ordered_interfaces(state_interfaces_, params_.joints, interface, joint_state_interface_[index]))
        {
            LOGE(get_node(), "Expected %zu '%s' state interfaces, got %zu.", dof_, interface.c_str(), joint_state_interface_[index].size());
            return CallbackReturn::ERROR;
        }
    }

    franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

    updateJointStates();
    updateRobotData();

    q_desired_      = q_;
    qdot_desired_   = qdot_;
    torque_desired_ = torque_;
    x_desired_      = x_;
    xdot_desired_   = xdot_;
    last_action_feedback_time_ = 0.0;
    action_active_ = false;
    active_goal_handle_.reset();
    link_task_data_.clear();

    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TestSingleFR3Controller::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) 
{
    {
        std::lock_guard<std::mutex> lock(compute_cv_mutex_);
        stop_compute_thread_ = true;
        compute_requested_ = false;
    }
    compute_cv_.notify_all();
    compute_done_cv_.notify_all();
    if (compute_thread_.joinable())
    {
        compute_thread_.join();
    }

    franka_robot_model_->release_interfaces();


    for (size_t index = 0; index < num_cmd_joints_; ++index)
    {
        if (has_position_command_interface_)
        {
            const auto joint_position_value_op = joint_command_interface_[0][index].get().get_optional();
            if (!joint_position_value_op.has_value())
            {
                LOGW(get_node(), "Unable to retrieve joint position value");
                return controller_interface::CallbackReturn::SUCCESS;
            }
            if (!joint_command_interface_[0][index].get().set_value(joint_position_value_op.value()))
            {
                LOGW(get_node(), "Unable to set the joint position to value: %f", joint_position_value_op.value());
                return controller_interface::CallbackReturn::SUCCESS;
            }
        }

        if (has_velocity_command_interface_ && !joint_command_interface_[1][index].get().set_value(0.0))
        {
            LOGW(get_node(), "Error while setting joint velocity to value 0.0");
            return controller_interface::CallbackReturn::SUCCESS;
        }

        // TODO(anyone): How to halt when using effort commands?
        if (has_effort_command_interface_ && !joint_command_interface_[3][index].get().set_value(0.0))
        {   
            LOGW(get_node(), "Error while setting joint effort to value 0.0");
            return controller_interface::CallbackReturn::SUCCESS;
        }
    }

    for (size_t index = 0; index < allowed_interface_types_.size(); ++index)
    {
        joint_command_interface_[index].clear();
        joint_state_interface_[index].clear();
    }

    std::shared_ptr<GoalHandleMoveLink> goal_handle;
    {
        std::lock_guard<std::mutex> lk(calculation_mutex_);
        goal_handle = active_goal_handle_;
    }
    if (goal_handle && goal_handle->is_active())
    {
        auto result = std::make_shared<MoveLink::Result>();
        result->success = false;
        result->message = "Controller deactivated";
        goal_handle->abort(result);
    }
    {
        std::lock_guard<std::mutex> lk(calculation_mutex_);
        action_active_ = false;
        active_goal_handle_.reset();
        link_task_data_.clear();
    }

    return CallbackReturn::SUCCESS;
}

controller_interface::return_type TestSingleFR3Controller::update(const rclcpp::Time& time, const rclcpp::Duration& /*period*/)
{
  Benchmark bench;

  updateJointStates();
  updateRobotData();

  {
    std::lock_guard<std::mutex> lk(calculation_mutex_);
    play_time_ = time.seconds();
  }
  const double current_time = time.seconds();

  const double spent_ms = bench.elapsed() * 1000.;
  double budget_ms = (dt_ * 1000.) - spent_ms - 0.05; // 0.05 ms for command to robot
  if (budget_ms < 0.0)
  {
    budget_ms = 0.0;
    LOGW(get_node(), "State update exceeded 1.0 ms (%.3f ms)", spent_ms);
  }

  constexpr double kWaitGuardMs = 0.2; // leave headroom for remaining work and comms
  const bool skip_guard = relax_wait_guard_.load(std::memory_order_acquire);
  if (!skip_guard && budget_ms > kWaitGuardMs)
  {
    budget_ms -= kWaitGuardMs;
  }
  else if (!skip_guard)
  {
    budget_ms = 0.0;
  }

  Eigen::Vector7d last_command;
  {
    std::lock_guard<std::mutex> lk(calculation_mutex_);
    last_command = torque_desired_;
  }

  bool used_new_solution = false;

  auto request_compute_and_wait =
      [this](double wait_ms, bool clear_relax_on_success) -> bool
      {
        {
          std::lock_guard<std::mutex> lock(compute_cv_mutex_);
          compute_requested_ = true;
          compute_completed_ = false;
        }
        compute_cv_.notify_one();

        const double clamped_wait = std::max(0.0, std::min(wait_ms, 0.2));
        const auto wait_dur = std::chrono::duration<double, std::milli>(clamped_wait);
        std::unique_lock<std::mutex> wait_lock(compute_cv_mutex_);
        if (compute_done_cv_.wait_for(wait_lock, wait_dur, [this]() { return compute_completed_; }))
        {
          if (clear_relax_on_success)
          {
            relax_wait_guard_.store(false, std::memory_order_release);
          }
          return true;
        }
        return false;
      };
  if (skip_guard)
  {
    if (!compute_inflight_.exchange(true, std::memory_order_acq_rel))
    {
      try
      {
        compute();
        used_new_solution = true;
        relax_wait_guard_.store(false, std::memory_order_release);
      }
      catch (const std::exception& e)
      {
        LOGE(get_node(), "Exception in compute(): %s", e.what());
      }
      catch (...)
      {
        LOGE(get_node(), "Unknown exception in compute()");
      }
      compute_inflight_.store(false, std::memory_order_release);
      {
        std::lock_guard<std::mutex> lock(compute_cv_mutex_);
        compute_completed_ = true;
        compute_requested_ = false;
      }
      compute_done_cv_.notify_all();
    }
  }
  else if (!compute_inflight_.exchange(true, std::memory_order_acq_rel))
  {
    if (request_compute_and_wait(budget_ms, false))
    {
      used_new_solution = true;
    }
  }

  Eigen::Vector7d command;
  if (used_new_solution)
  {
    std::lock_guard<std::mutex> lk(calculation_mutex_);
    command = torque_desired_;
    relax_wait_guard_.store(false, std::memory_order_release);
  }
  else
  {
    command = last_command;
  }

  static bool warned_no_effort = false;
  if (has_effort_command_interface_)
  {
    constexpr size_t kPositionIndex = 0;
    constexpr size_t kVelocityIndex = 1;
    constexpr size_t kEffortIndex = 3;
    for (size_t cmd_index = 0; cmd_index < num_cmd_joints_; ++cmd_index)
    {
      const size_t joint_index = map_cmd_to_joints_.empty() ? cmd_index : map_cmd_to_joints_[cmd_index];
      if (has_position_command_interface_)
      {
        joint_command_interface_[kPositionIndex][cmd_index].get().set_value(q_desired_(joint_index));
      }
      if (has_velocity_command_interface_)
      {
        joint_command_interface_[kVelocityIndex][cmd_index].get().set_value(qdot_desired_(joint_index));
      }
      joint_command_interface_[kEffortIndex][cmd_index].get().set_value(command(joint_index));
    }
  }
  else if (!warned_no_effort)
  {
    warned_no_effort = true;
    LOGE(get_node(), "No effort command interface available; controller will not send commands.");
  }

  // Action feedback + completion handling
  std::shared_ptr<GoalHandleMoveLink> goal_handle;
  std::string link_name;
  double start_time = 0.0;
  double duration = 0.0;
  double last_feedback_time = 0.0;
  bool action_active = false;
  {
    std::lock_guard<std::mutex> lk(calculation_mutex_);
    goal_handle = active_goal_handle_;
    link_name = active_link_name_;
    start_time = control_start_time_;
    duration = action_duration_;
    last_feedback_time = last_action_feedback_time_;
    action_active = action_active_;
  }

  if (goal_handle && action_active)
  {
    const double elapsed = std::max(0.0, current_time - start_time);
    if (goal_handle->is_canceling())
    {
      auto result = std::make_shared<MoveLink::Result>();
      result->success = false;
      result->message = "Goal canceled";
      goal_handle->canceled(result);
      std::lock_guard<std::mutex> lk(calculation_mutex_);
      action_active_ = false;
      active_goal_handle_.reset();
      link_task_data_.clear();
    }
    else if (elapsed >= duration)
    {
      auto result = std::make_shared<MoveLink::Result>();
      result->success = true;
      result->message = "Goal reached";
      goal_handle->succeed(result);
      std::lock_guard<std::mutex> lk(calculation_mutex_);
      action_active_ = false;
      active_goal_handle_.reset();
      link_task_data_.clear();
    }
    else if ((current_time - last_feedback_time) >= action_monitor_period_.seconds())
    {
      geometry_msgs::msg::PoseStamped current_pose;
      {
        std::lock_guard<std::mutex> lk(robot_data_mutex_);
        current_pose.header.stamp = time;
        if (robot_data_)
        {
          current_pose.header.frame_id = robot_data_->getRootLinkName();
          current_pose.pose = eigenToPoseMsg(robot_data_->getPose(link_name));
        }
      }
      auto feedback = std::make_shared<MoveLink::Feedback>();
      feedback->progress = duration > 1e-6 ? std::min(1.0, elapsed / duration) : 1.0;
      feedback->current_pose = current_pose;
      goal_handle->publish_feedback(feedback);
      std::lock_guard<std::mutex> lk(calculation_mutex_);
      last_action_feedback_time_ = current_time;
    }
  }

  return controller_interface::return_type::OK;
}

// ========================================================================
// ====================== Main Controller Functions =======================
// ========================================================================
void TestSingleFR3Controller::compute()
{
    std::scoped_lock(robot_data_mutex_, calculation_mutex_);
    if (!robot_controller_ || !robot_data_)
    {
        torque_desired_.setZero();
        return;
    }

    q_desired_ = q_;
    qdot_desired_.setZero();

    if (action_active_ && !active_link_name_.empty() && action_duration_ > 0.0)
    {
        auto it = link_task_data_.find(active_link_name_);
        if (it != link_task_data_.end())
        {
            it->second.x = robot_data_->getPose(active_link_name_);
            it->second.xdot = robot_data_->getVelocity(active_link_name_);
            it->second.xddot.setZero();
            const Eigen::VectorXd tau = robot_controller_->QPIDCubic(link_task_data_, play_time_, control_start_time_, action_duration_, false);
            if (tau.size() == static_cast<int>(dof_))
            {
                torque_desired_ = tau;
                return;
            }
            LOGE(get_node(), "QPIDCubic returned size %ld, expected %zu.", static_cast<long>(tau.size()), dof_);
        }
    }

    torque_desired_ = robot_data_->getGravity();
}

void TestSingleFR3Controller::updateJointStates() 
{
    auto assign_jvalue_from_state_interface = [&](Eigen::VectorXd & joint_value, const auto & joint_interface)
    {
        joint_value.setZero(dof_);
        for (size_t index = 0; index < dof_; ++index)
        {
            const auto joint_state_interface_value_op = joint_interface[index].get().get_optional();
            if (!joint_state_interface_value_op.has_value())
            {
                LOGW(get_node(), "Unable to retrieve joint state interface value for joint at index %zu", index);
            }
            else
            {
                joint_value(index) = joint_state_interface_value_op.value();
            }
        }
    };

    std::lock_guard<std::mutex> lk(robot_data_mutex_);
    Eigen::VectorXd qd_tmp;
    qd_tmp.setZero(dof_);
    if (has_position_state_interface_)  assign_jvalue_from_state_interface(q_,      joint_state_interface_[0]);
    if (has_velocity_state_interface_)  assign_jvalue_from_state_interface(qd_tmp,  joint_state_interface_[1]);
    if (has_effort_state_interface_)    assign_jvalue_from_state_interface(torque_, joint_state_interface_[3]);
    if (has_acceleration_state_interface_)
    {
        if (!has_velocity_state_interface_)
        {
            LOGW(get_node(), "Unable to retrieve joint acceleration. Set velocity as state_interface.");
        }
        else 
        {
            for(size_t i = 0; i < dof_; i++)
            {
                qddot_(i) = franka::lowpassFilter(dt_, (qd_tmp(i) - qdot_(i)) / dt_, qddot_(i), 150.0);
            }
        }
    }
    if (has_velocity_state_interface_)
    {
        qdot_ = qd_tmp;
    }
    else
    {
        qdot_.setZero();
    }
}

void TestSingleFR3Controller::updateRobotData()
{
    std::array<double, 49> mass = franka_robot_model_->getMassMatrix();
    std::array<double, 7> coriolis = franka_robot_model_->getCoriolisForceVector();
    std::array<double, 7> gravity = franka_robot_model_->getGravityForceVector();
    std::array<double, 16> pose = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
    std::array<double, 42> endeffector_jacobian_wrt_base = franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
    {
        std::lock_guard<std::mutex> lock(robot_data_mutex_);
        M_ = Eigen::Map<const Eigen::Matrix<double, 7, 7, Eigen::RowMajor>>(mass.data());
        c_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(coriolis.data());
        g_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(gravity.data());

        x_.matrix() = Eigen::Map<const Eigen::Matrix4d>(pose.data());

        Eigen::Map<const Eigen::Matrix<double,6,7,Eigen::ColMajor>> J_tmp(endeffector_jacobian_wrt_base.data());
        J_ = J_tmp;

        M_inv_ = M_.inverse();
        xdot_ = J_ * qdot_;
        // M_task_ = (J_ * M_inv_ * J_.transpose()).inverse();
        // J_T_inv_ = M_task_ * J_ * M_inv_;

        robot_data_->updateState(q_, qdot_);
    }
}

bool TestSingleFR3Controller::loadDyrosGains()
{
    if (!robot_controller_)
    {
        LOGE(get_node(), "RobotController is not initialized for gain setup.");
        return false;
    }

    auto read_vector_param = [this](const std::string& name, size_t expected, std::vector<double>& out) -> bool
    {
        if (!get_node()->get_parameter(name, out))
        {
            LOGE(get_node(), "Missing parameter '%s'.", name.c_str());
            return false;
        }
        if (out.size() != expected)
        {
            LOGE(get_node(), "Parameter '%s' expected %zu values, got %zu.", name.c_str(), expected, out.size());
            return false;
        }
        return true;
    };

    get_node()->get_parameter("dyros.task_link", default_task_link_);

    std::vector<double> joint_kp_vec;
    std::vector<double> joint_kv_vec;
    std::vector<double> task_kp_vec;
    std::vector<double> task_kv_vec;
    std::vector<double> qpik_tracking_vec;
    std::vector<double> qpik_damping_vec;
    std::vector<double> qpid_tracking_vec;
    std::vector<double> qpid_vel_damping_vec;
    std::vector<double> qpid_acc_damping_vec;

    if (!read_vector_param("dyros.joint_kp", 7, joint_kp_vec)) return false;
    if (!read_vector_param("dyros.joint_kv", 7, joint_kv_vec)) return false;
    if (!read_vector_param("dyros.task_kp", 6, task_kp_vec)) return false;
    if (!read_vector_param("dyros.task_kv", 6, task_kv_vec)) return false;
    if (!read_vector_param("dyros.qpik_tracking", 6, qpik_tracking_vec)) return false;
    if (!read_vector_param("dyros.qpik_damping", 7, qpik_damping_vec)) return false;
    if (!read_vector_param("dyros.qpid_tracking", 6, qpid_tracking_vec)) return false;
    if (!read_vector_param("dyros.qpid_vel_damping", 7, qpid_vel_damping_vec)) return false;
    if (!read_vector_param("dyros.qpid_acc_damping", 7, qpid_acc_damping_vec)) return false;

    joint_kp_ = Eigen::Map<Eigen::VectorXd>(joint_kp_vec.data(), joint_kp_vec.size());
    joint_kv_ = Eigen::Map<Eigen::VectorXd>(joint_kv_vec.data(), joint_kv_vec.size());
    qpik_damping_ = Eigen::Map<Eigen::VectorXd>(qpik_damping_vec.data(), qpik_damping_vec.size());
    qpid_vel_damping_ = Eigen::Map<Eigen::VectorXd>(qpid_vel_damping_vec.data(), qpid_vel_damping_vec.size());
    qpid_acc_damping_ = Eigen::Map<Eigen::VectorXd>(qpid_acc_damping_vec.data(), qpid_acc_damping_vec.size());

    default_task_kp_ << task_kp_vec[0], task_kp_vec[1], task_kp_vec[2], task_kp_vec[3], task_kp_vec[4], task_kp_vec[5];
    default_task_kv_ << task_kv_vec[0], task_kv_vec[1], task_kv_vec[2], task_kv_vec[3], task_kv_vec[4], task_kv_vec[5];
    default_qpik_tracking_ << qpik_tracking_vec[0], qpik_tracking_vec[1], qpik_tracking_vec[2], qpik_tracking_vec[3], qpik_tracking_vec[4], qpik_tracking_vec[5];
    default_qpid_tracking_ << qpid_tracking_vec[0], qpid_tracking_vec[1], qpid_tracking_vec[2], qpid_tracking_vec[3], qpid_tracking_vec[4], qpid_tracking_vec[5];

    link_task_kp_.clear();
    link_task_kv_.clear();
    link_qpik_tracking_.clear();
    link_qpid_tracking_.clear();
    link_task_kp_[default_task_link_] = default_task_kp_;
    link_task_kv_[default_task_link_] = default_task_kv_;
    link_qpik_tracking_[default_task_link_] = default_qpik_tracking_;
    link_qpid_tracking_[default_task_link_] = default_qpid_tracking_;

    if (robot_data_ && !robot_data_->hasLinkFrame(default_task_link_))
    {
        LOGW(get_node(), "Default task link '%s' not found in robot model.", default_task_link_.c_str());
    }

    robot_controller_->setJointGain(joint_kp_, joint_kv_);
    robot_controller_->setTaskGain(link_task_kp_, link_task_kv_);
    robot_controller_->setQPIKGain(link_qpik_tracking_, qpik_damping_);
    robot_controller_->setQPIDGain(link_qpid_tracking_, qpid_vel_damping_, qpid_acc_damping_);
    return true;
}

bool TestSingleFR3Controller::ensureLinkGains(const std::string& link_name)
{
    bool updated = false;
    if (link_task_kp_.find(link_name) == link_task_kp_.end())
    {
        link_task_kp_[link_name] = default_task_kp_;
        updated = true;
    }
    if (link_task_kv_.find(link_name) == link_task_kv_.end())
    {
        link_task_kv_[link_name] = default_task_kv_;
        updated = true;
    }
    if (link_qpik_tracking_.find(link_name) == link_qpik_tracking_.end())
    {
        link_qpik_tracking_[link_name] = default_qpik_tracking_;
        updated = true;
    }
    if (link_qpid_tracking_.find(link_name) == link_qpid_tracking_.end())
    {
        link_qpid_tracking_[link_name] = default_qpid_tracking_;
        updated = true;
    }

    if (updated && robot_controller_)
    {
        robot_controller_->setTaskGain(link_task_kp_, link_task_kv_);
        robot_controller_->setQPIKGain(link_qpik_tracking_, qpik_damping_);
        robot_controller_->setQPIDGain(link_qpid_tracking_, qpid_vel_damping_, qpid_acc_damping_);
    }
    return true;
}

void TestSingleFR3Controller::computeWorkerLoop()
{
    std::unique_lock<std::mutex> lock(compute_cv_mutex_);
    while (!stop_compute_thread_)
    {
    compute_cv_.wait(lock, [this]() { return compute_requested_ || stop_compute_thread_; });
    if (stop_compute_thread_)
    {
        break;
    }
    compute_requested_ = false;
    lock.unlock();
    try
    {
        compute();
    }
    catch (const std::exception& e)
    {
        LOGE(get_node(), "Exception in compute(): %s", e.what());
    }
    catch (...)
    {
        LOGE(get_node(), "Unknown exception in compute()");
    }
    lock.lock();
    compute_completed_ = true;
    compute_inflight_.store(false, std::memory_order_release);
    compute_done_cv_.notify_all();
    }
}

rclcpp_action::GoalResponse TestSingleFR3Controller::goal_received_callback(
    const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const MoveLink::Goal> goal)
{
    if (!goal)
    {
        LOGE(get_node(), "Received null goal.");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->link_name.empty())
    {
        LOGE(get_node(), "Goal link_name is empty.");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (!std::isfinite(goal->duration) || goal->duration <= 0.0)
    {
        LOGE(get_node(), "Goal duration must be positive.");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (!poseIsFinite(goal->target_pose.pose))
    {
        LOGE(get_node(), "Goal pose has non-finite values.");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (!robot_data_)
    {
        LOGE(get_node(), "Robot model is not initialized.");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (!robot_data_->hasLinkFrame(goal->link_name))
    {
        LOGE(get_node(), "Goal link '%s' not found in robot model.", goal->link_name.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (robot_data_ && !goal->target_pose.header.frame_id.empty() &&
        goal->target_pose.header.frame_id != robot_data_->getRootLinkName())
    {
        LOGW(get_node(), "Goal frame '%s' does not match root '%s'. Assuming root frame.",
             goal->target_pose.header.frame_id.c_str(), robot_data_->getRootLinkName().c_str());
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TestSingleFR3Controller::goal_cancelled_callback(
    const std::shared_ptr<GoalHandleMoveLink> goal_handle)
{
    if (!goal_handle)
    {
        return rclcpp_action::CancelResponse::REJECT;
    }
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TestSingleFR3Controller::goal_accepted_callback(std::shared_ptr<GoalHandleMoveLink> goal_handle)
{
    if (!goal_handle)
    {
        return;
    }
    auto goal = goal_handle->get_goal();
    if (!goal)
    {
        return;
    }

    std::shared_ptr<GoalHandleMoveLink> previous_goal;
    {
        std::scoped_lock lock(robot_data_mutex_, calculation_mutex_);
        previous_goal = active_goal_handle_;
        ensureLinkGains(goal->link_name);

        active_goal_handle_ = goal_handle;
        active_link_name_ = goal->link_name;
        action_duration_ = goal->duration;
        control_start_time_ = get_node()->now().seconds();
        last_action_feedback_time_ = control_start_time_;
        action_active_ = true;

        drc::TaskSpaceData task = drc::TaskSpaceData::Zero();
        task.x = robot_data_->getPose(active_link_name_);
        task.xdot = robot_data_->getVelocity(active_link_name_);
        task.setInit();
        task.x_desired = poseMsgToEigen(goal->target_pose.pose);
        task.xdot_desired.setZero();
        task.xddot_desired.setZero();

        link_task_data_.clear();
        link_task_data_[active_link_name_] = task;
    }

    if (previous_goal && previous_goal->is_active() && previous_goal != goal_handle)
    {
        auto result = std::make_shared<MoveLink::Result>();
        result->success = false;
        result->message = "Preempted by new goal";
        previous_goal->abort(result);
    }
}

}  // namespace FR3HuskyController
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(FR3HuskyController::TestSingleFR3Controller,
                       controller_interface::ControllerInterface)

        
