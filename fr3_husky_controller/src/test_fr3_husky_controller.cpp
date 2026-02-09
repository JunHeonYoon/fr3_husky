#include "fr3_husky_controller/test_fr3_husky_controller.hpp"
namespace fr3_husky_controller
{
controller_interface::InterfaceConfiguration TestFR3HuskyController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    // Manipulator joint state interfaces
    for (const auto & joint_name : params_.manipulator_joints)
    {
        for (const auto & interface_type : params_.manipulator_state_interfaces)
        {
            conf.names.push_back(joint_name + "/" + interface_type);
        }
    }

    // Franka semantic components (model/state)
    for (size_t i = 0; i < num_robots_; ++i)
    {
        for (const auto & name : franka_robot_model_[i]->get_state_interface_names())
        {
            conf.names.push_back(name);
        }
        conf.names.push_back(params_.robot_name[i] + "_" + arm_id_ + "/robot_time");
    }

    // Mobile base state interfaces (position/velocity)
    for (const auto & joint_name : params_.left_wheel_names)
    {
        conf.names.push_back(joint_name + "/" + allowed_interface_types_[0]);
        conf.names.push_back(joint_name + "/" + allowed_interface_types_[1]);
    }
    for (const auto & joint_name : params_.right_wheel_names)
    {
        conf.names.push_back(joint_name + "/" + allowed_interface_types_[0]);
        conf.names.push_back(joint_name + "/" + allowed_interface_types_[1]);
    }

    return conf;
}

controller_interface::InterfaceConfiguration TestFR3HuskyController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    // Manipulator command interfaces
    for (const auto & joint_name : params_.manipulator_joints)
    {
        conf.names.push_back(joint_name + "/" + params_.manipulator_command_interface);
    }

    // Mobile base command interfaces (velocity)
    for (const auto & joint_name : params_.left_wheel_names)
    {
        conf.names.push_back(joint_name + "/" + allowed_interface_types_[1]);
    }
    for (const auto & joint_name : params_.right_wheel_names)
    {
        conf.names.push_back(joint_name + "/" + allowed_interface_types_[1]);
    }

    return conf;
}

CallbackReturn TestFR3HuskyController::on_init()
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
    return CallbackReturn::SUCCESS;
}

CallbackReturn TestFR3HuskyController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    // update parameters if they have changed
    if (param_listener_->is_old(params_))
    {
        params_ = param_listener_->get_params();
        LOGI(get_node(), "Parameters were updated");
    }

    // number of FR3 robot used
    num_robots_ = params_.robot_name.size();
    if (num_robots_ < 1 || num_robots_ > 2)
    {
        LOGE(get_node(), "FR3 Husky controller expects one or two FR3 robots, but got %zu.", num_robots_);
        return CallbackReturn::FAILURE;
    }

    // check names in allowed name list & uniqueness 
    const std::unordered_set<std::string> allowed = {"left", "right"};
    std::unordered_set<std::string> seen;
    for (const auto& name : params_.robot_name) 
    {
        if (allowed.find(name) == allowed.end()) 
        {
            LOGE(get_node(), "Invalid robot_name '%s'. Allowed values are 'left' or 'right'.", name.c_str());
            return CallbackReturn::FAILURE;
        }

        if (!seen.insert(name).second) 
        {
            LOGE(get_node(), "Duplicate robot_name '%s' detected. robot_name entries must be unique.", name.c_str());
            return CallbackReturn::FAILURE;
        }
    }

    // get manipulator degrees of freedom
    manipulator_dof_ = params_.manipulator_joints.size();
    if (manipulator_dof_ != FR3_DOF * num_robots_)
    {
        LOGE(get_node(), "FR3 Husky controller expects %zu manipulator DoF, but got %zu.", FR3_DOF * num_robots_, manipulator_dof_);
        return CallbackReturn::FAILURE;
    }

    // number of mobile wheel checking
    if (params_.left_wheel_names.size() != params_.right_wheel_names.size())
    {
        LOGE(get_node(), "Number of left wheels (%zu) differs from right wheels (%zu).", params_.left_wheel_names.size(), params_.right_wheel_names.size());
        return CallbackReturn::FAILURE;
    }
    if (params_.left_wheel_names.size() != WHEEL_PER_SIDE)
    {
        LOGW(get_node(), "Expected %d wheels per side; got %zu. Continuing but commands will be duplicated.", WHEEL_PER_SIDE, params_.left_wheel_names.size());
    }

    if (params_.manipulator_command_interface.empty() || params_.manipulator_state_interfaces.empty())
    {
        LOGE(get_node(), "Command/state interface parameters are empty.");
        return CallbackReturn::FAILURE;
    }

    // command flags (exactly one will be true)
    has_position_command_interface_ = (params_.manipulator_command_interface == allowed_interface_types_[0]);
    has_velocity_command_interface_ = (params_.manipulator_command_interface == allowed_interface_types_[1]);
    has_effort_command_interface_   = (params_.manipulator_command_interface == allowed_interface_types_[2]);

    has_position_state_interface_ = robot_utils::contains_interface_type(params_.manipulator_state_interfaces, allowed_interface_types_[0]);
    has_velocity_state_interface_ = robot_utils::contains_interface_type(params_.manipulator_state_interfaces, allowed_interface_types_[1]);
    has_effort_state_interface_   = robot_utils::contains_interface_type(params_.manipulator_state_interfaces, allowed_interface_types_[2]);

    // Validation of combinations of state and velocity together have to be done
    // here because the parameter validators only deal with each parameter
    // separately.
    if (has_velocity_command_interface_ && (!has_velocity_state_interface_ || !has_position_state_interface_))
    {
        LOGE(get_node(),
            "'velocity' command interface can only be used alone if 'velocity' and "
            "'position' state interfaces are present");
        return CallbackReturn::FAILURE;
    }

    // effort interfaces require position and velocity state
    if (has_effort_command_interface_ && (!has_velocity_state_interface_ || !has_position_state_interface_))
    {
        LOGE(get_node(),
                "'effort' command interface can only be used alone or with 'position' command interface "
                "if 'velocity' and 'position' state interfaces are present");
        return CallbackReturn::FAILURE;
    }

    // get sampling time dt_
    if (get_update_rate() == 0)
    {
        throw std::runtime_error("Controller's update rate is set to 0. This should not happen!");
    }
    dt_ = 1.0 / static_cast<double>(get_update_rate());

    // Initialize Franka semantic components
    franka_robot_model_.clear();
    for (const auto & name : params_.robot_name)
    {
        franka_robot_model_.push_back(std::make_unique<franka_semantic_components::FrankaRobotModel>(name + "_" + arm_id_ + "/robot_model", 
                                                                                                     name + "_" + arm_id_ + "/robot_state"));
    }

    // initialize dyros_robot_data & controller
    const std::string description_pkg = ament_index_cpp::get_package_share_directory("fr3_husky_description");
    std::string xacro_path = description_pkg + "/robots/";
    std::ostringstream segmentation_args;
    segmentation_args << " with_sc:=true"
                      << " hand:=" << (params_.hand ? "true" : "false")
                      << " as_two_wheels:=true"
                      << " ros2_control:=false"
                      << " use_fake_hardware:=false"
                      << " fake_sensor_commands:=false"
                      << " virtual_joint:=true";
    std::string robot_segmentation_description_param = segmentation_args.str();
    std::string robot_description_param = robot_segmentation_description_param + " ros2_control:=false"
                                                                               + " use_fake_hardware:=false"
                                                                               + " fake_sensor_commands:=false"
                                                                               + " fix_finger:=true"
                                                                               + " virtual_joint:=true";

    if (num_robots_ == 1)
    {
        robot_segmentation_description_param += " side:=" + params_.robot_name[0];
        robot_description_param += " side:=" + params_.robot_name[0];
        xacro_path += "single_fr3_husky";
    }
    else
    {
        xacro_path += "dual_fr3_husky";
    }

    const std::string urdf_xml = robot_utils::execAndCaptureStdout("xacro " + xacro_path + ".urdf.xacro" + robot_description_param);
    const std::string srdf_xml = robot_utils::execAndCaptureStdout("xacro " + xacro_path + ".srdf.xacro" + robot_segmentation_description_param);

    drc::Mobile::KinematicParam p;
    p.type          = drc::Mobile::DriveType::Differential;
    p.wheel_radius  = params_.wheel_radius_multiplier * params_.wheel_radius;
    p.base_width    = params_.wheel_separation_multiplier * params_.wheel_separation;
    p.max_lin_speed = params_.linear.x.max_velocity;
    p.max_ang_speed = params_.angular.z.max_velocity;
    p.max_lin_acc   = params_.linear.x.max_acceleration;
    p.max_ang_acc   = params_.angular.z.max_acceleration;

    // check the indices of URDF by urdf_to_graphviz
    // its order depends on DFS from root link
    drc::MobileManipulator::JointIndex j;
    if(!setJointIndex(urdf_xml, j))
    {
        LOGE(get_node(), "Failed to set mobile manipulator JointIndex");
        return CallbackReturn::ERROR;
    }

    drc::MobileManipulator::ActuatorIndex a;
    a.mani_start = j.mani_start - (int)(virtual_dof_);
    a.mobi_start = j.mobi_start - (int)(virtual_dof_);

    try
    {
        robot_data_ = std::make_shared<drc::MobileManipulator::RobotData>(dt_, p, j, a, urdf_xml, srdf_xml, description_pkg, true);
        robot_controller_ = std::make_unique<drc::MobileManipulator::RobotController>(robot_data_);

        LOGI(get_node(), robot_data_->getVerbose().c_str());
    }
    catch (const std::exception & e)
    {
        LOGE(get_node(), "Failed to initialize mobile manipulator data/controller: %s", e.what());
        return CallbackReturn::ERROR;
    }
    if (!loadDRCGains())
    {
        return CallbackReturn::FAILURE;
    }

    // Allocate manipulator state buffers
    mani_state_.q_init.assign(num_robots_, Eigen::Matrix<double, FR3_DOF, 1>::Zero());
    mani_state_.qdot_init.assign(num_robots_, Eigen::Matrix<double, FR3_DOF, 1>::Zero());
    mani_state_.qddot_init.assign(num_robots_, Eigen::Matrix<double, FR3_DOF, 1>::Zero());
    mani_state_.q_total_init.setZero(manipulator_dof_);
    mani_state_.qdot_total_init.setZero(manipulator_dof_);
    mani_state_.qddot_total_init.setZero(manipulator_dof_);

    mani_state_.q.assign(num_robots_, Eigen::Matrix<double, FR3_DOF, 1>::Zero());
    mani_state_.qdot.assign(num_robots_, Eigen::Matrix<double, FR3_DOF, 1>::Zero());
    mani_state_.qddot.assign(num_robots_, Eigen::Matrix<double, FR3_DOF, 1>::Zero());
    mani_state_.torque.assign(num_robots_, Eigen::Matrix<double, FR3_DOF, 1>::Zero());
    mani_state_.q_total.setZero(manipulator_dof_);
    mani_state_.qdot_total.setZero(manipulator_dof_);
    mani_state_.qddot_total.setZero(manipulator_dof_);
    mani_state_.torque_total.setZero(manipulator_dof_);

    mani_state_.q_desired.assign(num_robots_, Eigen::Matrix<double, FR3_DOF, 1>::Zero());
    mani_state_.qdot_desired.assign(num_robots_, Eigen::Matrix<double, FR3_DOF, 1>::Zero());
    mani_state_.torque_desired.assign(num_robots_, Eigen::Matrix<double, FR3_DOF, 1>::Zero());
    mani_state_.q_desired_total.setZero(manipulator_dof_);
    mani_state_.qdot_desired_total.setZero(manipulator_dof_);
    mani_state_.torque_desired_total.setZero(manipulator_dof_);

    mani_state_.M.assign(num_robots_, Eigen::Matrix<double, FR3_DOF, FR3_DOF>::Zero());
    mani_state_.M_inv.assign(num_robots_, Eigen::Matrix<double, FR3_DOF, FR3_DOF>::Zero());
    mani_state_.c.assign(num_robots_, Eigen::Matrix<double, FR3_DOF, 1>::Zero());
    mani_state_.g.assign(num_robots_, Eigen::Matrix<double, FR3_DOF, 1>::Zero());
    mani_state_.M_total.setZero(manipulator_dof_, manipulator_dof_);
    mani_state_.M_inv_total.setZero(manipulator_dof_, manipulator_dof_);
    mani_state_.c_total.setZero(manipulator_dof_);
    mani_state_.g_total.setZero(manipulator_dof_);

    ee_name_.clear();
    for (const auto & name : params_.robot_name)
    {
        std::string ee_name = name + "_" + arm_id_ + "_";
        if (params_.hand) ee_name = ee_name + "hand_tcp";
        else              ee_name = ee_name + "link8";
        ee_name_.push_back(ee_name);
    }

    for (const auto & ee_name : ee_name_)
    {
        ee_data_[ee_name] = drc::TaskSpaceData::Zero();
    }

    // Mobile base init
    mobi_state_.base_pose_w = Eigen::Affine2d::Identity();
    mobi_state_.base_pose_w_desired = Eigen::Affine2d::Identity();
    mobi_state_.base_pose_w_init = Eigen::Affine2d::Identity();
    mobi_state_.base_vel_w.setZero();
    mobi_state_.base_vel_w_desired.setZero();
    mobi_state_.base_vel_w_init.setZero();
    mobi_state_.base_vel_b.setZero();
    mobi_state_.base_vel_b_desired.setZero();
    mobi_state_.base_vel_b_init.setZero();
    mobi_state_.wheel_pos.setZero();
    mobi_state_.wheel_vel.setZero();
    mobi_state_.wheel_vel_desired.setZero();

    // cmd_vel timeout
    cmd_vel_timeout_ = std::chrono::milliseconds{static_cast<int>(params_.cmd_vel_timeout * 1000.0)};

    // Subscribers and publishers
    const geometry_msgs::msg::TwistStamped empty_twist;
    received_velocity_msg_ptr_.set(std::make_shared<geometry_msgs::msg::TwistStamped>(empty_twist));

    velocity_command_unstamped_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
        "~/cmd_vel_unstamped", rclcpp::SystemDefaultsQoS(),
        [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void
        {
            if (!subscriber_is_active_)
            {
                LOGW(get_node(), "Can't accept new commands. subscriber is inactive");
                return;
            }
            std::shared_ptr<geometry_msgs::msg::TwistStamped> twist_stamped;
            received_velocity_msg_ptr_.get(twist_stamped);
            twist_stamped->twist = *msg;
            twist_stamped->header.stamp = get_node()->get_clock()->now();
        });

    // Odometry publishers
    odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>("~/odom", rclcpp::SystemDefaultsQoS());
    odometry_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::SystemDefaultsQoS());

    publish_rate_ = params_.publish_rate;

    mobi_state_pub_buf_.writeFromNonRT(mobi_state_);

    odom_timer_ = get_node()->create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate_),
        [this]()
        {
            this->publishFromMobileStateBuffer();
        }
    );

    return CallbackReturn::SUCCESS;
}

CallbackReturn TestFR3HuskyController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    // Register manipulator interfaces into JointHandle
    std::vector<std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>> state_by_type(allowed_interface_types_.size()); // [interface_type][joint_idx]
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> cmd_by_type; // [joint_idx]

    for (const auto & interface : params_.manipulator_state_interfaces)
    {
        auto it = std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
        if (it == allowed_interface_types_.end()) continue;
        size_t idx = static_cast<size_t>(std::distance(allowed_interface_types_.begin(), it));
        if (!controller_interface::get_ordered_interfaces(state_interfaces_, params_.manipulator_joints, interface, state_by_type[idx]))
        {
            LOGE(get_node(), "Expected %zu '%s' state interfaces, got %zu.", manipulator_dof_, interface.c_str(), state_by_type[idx].size());
            return CallbackReturn::ERROR;
        }
        if (state_by_type[idx].size() != manipulator_dof_)
        {
            LOGE(get_node(), "State interface '%s' not available for every manipulator joint (%zu of %zu).", interface.c_str(), state_by_type[idx].size(), manipulator_dof_);
            return CallbackReturn::ERROR;
        }
    }

    const auto & cmd_interface = params_.manipulator_command_interface;
    auto it = std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), cmd_interface);
    if (it == allowed_interface_types_.end())
    {
        LOGE(get_node(), "Invalid command_interface '%s'.", cmd_interface.c_str());
        return CallbackReturn::ERROR;
    }
    if (!controller_interface::get_ordered_interfaces(command_interfaces_, params_.manipulator_joints, cmd_interface, cmd_by_type))
    {
        LOGE(get_node(), "Expected %zu '%s' command interfaces, got %zu.",
            manipulator_dof_, cmd_interface.c_str(), cmd_by_type.size());
        return CallbackReturn::ERROR;
    }
    if (cmd_by_type.size() != manipulator_dof_)
    {
        LOGE(get_node(), "Command interface '%s' not available for every manipulator joint (%zu of %zu).",
            cmd_interface.c_str(), cmd_by_type.size(), manipulator_dof_);
        return CallbackReturn::ERROR;
    }


    registered_manipulator_joint_handles_.clear();
    for (size_t i = 0; i < manipulator_dof_; ++i)
    {
        registered_manipulator_joint_handles_.emplace_back(cmd_by_type[i]);
        auto & handle = registered_manipulator_joint_handles_.back();
        handle.state.clear();

        auto default_state_ref = state_by_type[kPositionIndex][i]; // position state is mandatory
        handle.state.assign(allowed_interface_types_.size(), default_state_ref);

        // NOTE: this one is redundant if position is always mandatory, but harmless
        if (has_position_state_interface_)   handle.state[kPositionIndex] = state_by_type[kPositionIndex][i];
        if (has_velocity_state_interface_)   handle.state[kVelocityIndex] = state_by_type[kVelocityIndex][i];
        if (has_effort_state_interface_ && state_by_type.size() > kEffortIndex && !state_by_type[kEffortIndex].empty())
            handle.state[kEffortIndex] = state_by_type[kEffortIndex][i];
    }


    // Register wheel interfaces (reuse Husky logic)
    auto configure_side = [this](const std::string & side, const std::vector<std::string> & wheel_names, std::vector<WheelHandle> & registered_handles)
    {
        (void)side;
        registered_handles.clear();
        registered_handles.reserve(wheel_names.size());
        for (const auto & wheel_name : wheel_names)
        {
            const auto position_handle = std::find_if(
                state_interfaces_.cbegin(), state_interfaces_.cend(),
                [this, &wheel_name](const auto & iface)
                {
                    return iface.get_prefix_name() == wheel_name && iface.get_interface_name() == allowed_interface_types_[0];
                });
            if (position_handle == state_interfaces_.cend())
            {
                LOGE(get_node(), "Unable to obtain joint position state handle for %s", wheel_name.c_str());
                return false;
            }
            const auto velocity_handle = std::find_if(
                state_interfaces_.cbegin(), state_interfaces_.cend(),
                [this, &wheel_name](const auto & iface)
                {
                    return iface.get_prefix_name() == wheel_name && iface.get_interface_name() == allowed_interface_types_[1];
                });
            if (velocity_handle == state_interfaces_.cend())
            {
                LOGE(get_node(), "Unable to obtain joint velocity state handle for %s", wheel_name.c_str());
                return false;
            }
            const auto command_handle = std::find_if(
                command_interfaces_.begin(), command_interfaces_.end(),
                [this, &wheel_name](const auto & iface)
                {
                    return iface.get_prefix_name() == wheel_name && iface.get_interface_name() == allowed_interface_types_[1];
                });
            if (command_handle == command_interfaces_.end())
            {
                LOGE(get_node(), "Unable to obtain joint command handle for %s", wheel_name.c_str());
                return false;
            }
            std::vector<std::reference_wrapper<const hardware_interface::LoanedStateInterface>> state_handles;
            state_handles.reserve(2);
            state_handles.emplace_back(std::ref(*position_handle));
            state_handles.emplace_back(std::ref(*velocity_handle));
            registered_handles.emplace_back(WheelHandle{std::move(state_handles), std::ref(*command_handle)});
        }
        return true;
    };

    if (!configure_side("left", params_.left_wheel_names, registered_left_wheel_handles_) ||
        !configure_side("right", params_.right_wheel_names, registered_right_wheel_handles_))
    {
        return CallbackReturn::ERROR;
    }

    // Assign Franka semantic component state interfaces
    try
    {
        for (const auto & model : franka_robot_model_)
        {
            model->assign_loaned_state_interfaces(state_interfaces_);
        }
    }
    catch (const std::exception & e)
    {
        LOGE(get_node(), "Failed to assign Franka state interfaces: %s", e.what());
        return CallbackReturn::ERROR;
    }

    is_halted_ = false;
    subscriber_is_active_ = true;

    updateJointStates();
    updateRobotData();
    setInitfromCurrent();

    return CallbackReturn::SUCCESS;
}

CallbackReturn TestFR3HuskyController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    subscriber_is_active_ = false;
    if (!is_halted_)
    {
        // stop wheels
        for (const auto & h : registered_left_wheel_handles_) h.command.get().set_value(0.0);
        for (const auto & h : registered_right_wheel_handles_) h.command.get().set_value(0.0);

        for (const auto & model : franka_robot_model_)
        {
            model->release_interfaces();
        }
        // hold manipulator position by commanding current values
        if (has_position_command_interface_)
        {
            for (auto & h : registered_manipulator_joint_handles_)
            {
                const double current = h.command.get().get_value();
                h.command.get().set_value(current);
            }
        }
        else
        {
            for (auto & h : registered_manipulator_joint_handles_) h.command.get().set_value(0.0);
        }
        is_halted_ = true;
    }
    registered_left_wheel_handles_.clear();
    registered_right_wheel_handles_.clear();
    registered_manipulator_joint_handles_.clear();

    odom_timer_.reset();

    return CallbackReturn::SUCCESS;
}

CallbackReturn TestFR3HuskyController::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
    if (!reset()) return CallbackReturn::ERROR;
    received_velocity_msg_ptr_.set(std::make_shared<geometry_msgs::msg::TwistStamped>());
    return CallbackReturn::SUCCESS;
}

CallbackReturn TestFR3HuskyController::on_error(const rclcpp_lifecycle::State & /*previous_state*/)
{
    if (!reset()) return CallbackReturn::ERROR;
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type TestFR3HuskyController::update(const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{

    if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
        if (!is_halted_)
        {
            for (const auto & h : registered_left_wheel_handles_) h.command.get().set_value(0.0);
            for (const auto & h : registered_right_wheel_handles_) h.command.get().set_value(0.0);
            // hold manipulator position by commanding current values
            if (has_position_command_interface_)
            {
                for (auto & h : registered_manipulator_joint_handles_)
                {
                    const double current = h.command.get().get_value();
                    h.command.get().set_value(current);
                }
            }
            else
            {
                for (auto & h : registered_manipulator_joint_handles_) h.command.get().set_value(0.0);
            }
            is_halted_ = true;
        }
        return controller_interface::return_type::OK;
    }

    updateJointStates();
    updateRobotData();

    mobi_state_pub_buf_.writeFromNonRT(mobi_state_);

    std::shared_ptr<geometry_msgs::msg::TwistStamped> cmd_msg;
    received_velocity_msg_ptr_.get(cmd_msg);
    if (!cmd_msg)
    {
        LOGW(get_node(), "Velocity message received was a nullptr.");
        return controller_interface::return_type::ERROR;
    }

    const bool cmd_valid = ((time - cmd_msg->header.stamp) <= cmd_vel_timeout_);
    const ControlMode prev_mode = control_mode_;

    if (cmd_valid) control_mode_ = ControlMode::JOY;
    else           control_mode_ = ControlMode::AUTONOMOUS;

    if(prev_mode != control_mode_)
    {
        LOGI(get_node(), "Control mode changed: %s -> %s", (prev_mode == ControlMode::JOY) ? "JOY" : "AUTONOMOUS", (control_mode_ == ControlMode::JOY) ? "JOY" : "AUTONOMOUS");
        setInitfromCurrent();
    }

    if (control_mode_ == ControlMode::JOY)
    {
        mani_state_.q_desired_total = mani_state_.q_total_init;
        mani_state_.qdot_desired_total.setZero();
        mani_state_.torque_desired_total = robot_controller_->moveManipulatorJointTorqueStep(mani_state_.q_desired_total,
                                                                                             mani_state_.qdot_desired_total,
                                                                                             params_.dyros_robot_controller.manipulator_joint_gains.use_mass_matrix);
        mani_state_.torque_desired_total = mani_state_.torque_desired_total - mani_state_.g_total;
        
        const Eigen::VectorXd& mani_cmd = has_position_command_interface_ ? mani_state_.q_desired_total :
                                          has_velocity_command_interface_ ? mani_state_.qdot_desired_total :
                                                                            mani_state_.torque_desired_total;

        mobi_state_.base_vel_b_desired << cmd_msg->twist.linear.x, 0.0, cmd_msg->twist.angular.z;
        mobi_state_.wheel_vel_desired = robot_controller_->VelocityCommand(mobi_state_.base_vel_b_desired);
               
        writeCommandInterfaces(mani_cmd, mobi_state_.wheel_vel_desired);
    }
    else if(control_mode_ == ControlMode::AUTONOMOUS)
    {
        // ------------ gravity compensation for manipulator & stop mobile ------------
        // mani_state_.q_desired_total = mani_state_.q_total_init;
        // mani_state_.qdot_desired_total.setZero();
        // mani_state_.torque_desired_total = robot_data_->getGravityActuated().segment(robot_data_->getActuatorIndex().mani_start, manipulator_dof_);
        // mani_state_.torque_desired_total = mani_state_.torque_desired_total - mani_state_.g_total;

        // const Eigen::VectorXd& mani_cmd = has_position_command_interface_ ? mani_state_.q_desired_total :
        //                                   has_velocity_command_interface_ ? mani_state_.qdot_desired_total :
        //                                                                     mani_state_.torque_desired_total;

        // mobi_state_.base_vel_b_desired.setZero();
        // mobi_state_.wheel_vel_desired = robot_controller_->VelocityCommand(mobi_state_.base_vel_b_desired);
                              
        // writeCommandInterfaces(mani_cmd, mobi_state_.wheel_vel_desired);

        // ------------ QPID for mobile manipulator ------------
        if (has_effort_command_interface_)
        {
            std::map<std::string, drc::TaskSpaceData> zero_data;
            std::map<std::string, Eigen::Vector6d> zero_qpid_tracking;
            for (const auto & link_name : params_.dyros_robot_controller.QPID_weight.tracking.link_names)
            {
                zero_data[link_name] = drc::TaskSpaceData::Zero();
                zero_qpid_tracking[link_name] = Eigen::Vector6d::Zero();
            }
            robot_controller_->setQPIDGain(zero_qpid_tracking, qpid_mani_vel_damping_, qpid_mani_acc_damping_, qpid_mobi_vel_damping_, qpid_mobi_acc_damping_);
    
            Eigen::VectorXd opt_torque_mani, opt_qddot_mobi;
            opt_torque_mani.setZero(manipulator_dof_);
            opt_qddot_mobi.setZero(robot_data_->getMobileDof());
    
            if(!robot_controller_->QPID(zero_data, opt_qddot_mobi, opt_torque_mani))
            {
                mani_state_.q_desired_total = mani_state_.q_total_init;
                mani_state_.qdot_desired_total.setZero();
                mani_state_.torque_desired_total = robot_data_->getGravityActuated().segment(robot_data_->getActuatorIndex().mani_start, manipulator_dof_);
                mani_state_.torque_desired_total = mani_state_.torque_desired_total - mani_state_.g_total;
        
                mobi_state_.base_vel_b_desired.setZero();
                mobi_state_.wheel_vel_desired = robot_controller_->VelocityCommand(mobi_state_.base_vel_b_desired);
                                    
                writeCommandInterfaces(mani_state_.torque_desired_total, mobi_state_.wheel_vel_desired);
    
            }
            else
            {
                mani_state_.torque_desired_total = opt_torque_mani - mani_state_.g_total;
                mobi_state_.wheel_vel_desired = mobi_state_.wheel_vel_desired + opt_qddot_mobi * dt_;

                writeCommandInterfaces(mani_state_.torque_desired_total, mobi_state_.wheel_vel_desired);
            }

        }
        else
        {
            mani_state_.q_desired_total = mani_state_.q_total_init;
            mani_state_.qdot_desired_total.setZero();
            mani_state_.torque_desired_total = robot_data_->getGravityActuated().segment(robot_data_->getActuatorIndex().mani_start, manipulator_dof_);
            mani_state_.torque_desired_total = mani_state_.torque_desired_total - mani_state_.g_total;

            const Eigen::VectorXd& mani_cmd = has_position_command_interface_ ? mani_state_.q_desired_total :
                                            has_velocity_command_interface_ ? mani_state_.qdot_desired_total :
                                                                                mani_state_.torque_desired_total;

            mobi_state_.base_vel_b_desired.setZero();
            mobi_state_.wheel_vel_desired = robot_controller_->VelocityCommand(mobi_state_.base_vel_b_desired);
                                
            writeCommandInterfaces(mani_cmd, mobi_state_.wheel_vel_desired);

        }


    }
    return controller_interface::return_type::OK;
}

void TestFR3HuskyController::writeCommandInterfaces(const Eigen::VectorXd& command_mani, const Eigen::VectorXd& command_mobi)
{
    // ---- Manipulator ----
    if (static_cast<size_t>(command_mani.size()) == manipulator_dof_)
    {
        for (size_t i = 0; i < registered_manipulator_joint_handles_.size(); ++i)
            registered_manipulator_joint_handles_[i].command.get().set_value(command_mani(i));
    }
    else
    {
        LOGW(get_node(), "Manipulator cmd size mismatch (expected %zu, got %zu). Holding/zeroing.", manipulator_dof_, command_mani.size());
        if (has_position_command_interface_)
        {
            for (size_t i = 0; i < registered_manipulator_joint_handles_.size(); ++i)
            {
                const double cur = registered_manipulator_joint_handles_[i].state[kPositionIndex].get().get_value();
                registered_manipulator_joint_handles_[i].command.get().set_value(cur);
            }
        }
        else
        {
            for (auto & h : registered_manipulator_joint_handles_) h.command.get().set_value(0.0);
        }
    }

    // ---- Mobile ----
    if (static_cast<size_t>(command_mobi.size()) == mobile_dof_)
    {
        for (size_t i = 0; i < registered_left_wheel_handles_.size(); ++i)
            registered_left_wheel_handles_[i].command.get().set_value(command_mobi(0));
        for (size_t i = 0; i < registered_right_wheel_handles_.size(); ++i)
            registered_right_wheel_handles_[i].command.get().set_value(command_mobi(1));
    }
    else
    {
        LOGW(get_node(), "Mobile cmd size mismatch (expected %zu, got %zu). Zeroing.", mobile_dof_, command_mobi.size());
        for (auto & h : registered_left_wheel_handles_)  h.command.get().set_value(0.0);
        for (auto & h : registered_right_wheel_handles_) h.command.get().set_value(0.0);
    }
}

void TestFR3HuskyController::updateJointStates()
{
    // Manipulator joint states
    const Eigen::VectorXd last_qdot_total = mani_state_.qdot_total;
    for (size_t i = 0; i < manipulator_dof_; ++i)
    {
        if (has_position_state_interface_) mani_state_.q_total(i)      = registered_manipulator_joint_handles_[i].state[kPositionIndex].get().get_value();
        if (has_velocity_state_interface_) mani_state_.qdot_total(i)   = registered_manipulator_joint_handles_[i].state[kVelocityIndex].get().get_value();
        if (has_effort_state_interface_)   mani_state_.torque_total(i) = registered_manipulator_joint_handles_[i].state[kEffortIndex].get().get_value();
    }
    mani_state_.qddot_total = (mani_state_.qdot_total - last_qdot_total) / dt_;
    

    // split per arm
    for (size_t r = 0; r < num_robots_; ++r)
    {
        mani_state_.q[r]      = mani_state_.q_total.segment(FR3_DOF * r, FR3_DOF);
        mani_state_.qdot[r]   = mani_state_.qdot_total.segment(FR3_DOF * r, FR3_DOF);
        mani_state_.qddot[r]  = mani_state_.qddot_total.segment(FR3_DOF * r, FR3_DOF);
        mani_state_.torque[r] = mani_state_.torque_total.segment(FR3_DOF * r, FR3_DOF);
    }

    // Wheels
    double left_pos = 0.0, right_pos = 0.0, left_vel = 0.0, right_vel = 0.0;
    for (size_t i = 0; i < registered_left_wheel_handles_.size(); ++i)
    {
        const auto & left_state = registered_left_wheel_handles_[i].state;
        const auto & right_state = registered_right_wheel_handles_[i].state;
        left_pos += left_state[kFeedbackPositionIndex].get().get_value();
        right_pos += right_state[kFeedbackPositionIndex].get().get_value();
        left_vel += left_state[kFeedbackVelocityIndex].get().get_value();
        right_vel += right_state[kFeedbackVelocityIndex].get().get_value();
    }
    if (!registered_left_wheel_handles_.empty())
    {
        left_pos /= static_cast<double>(registered_left_wheel_handles_.size());
        right_pos /= static_cast<double>(registered_right_wheel_handles_.size());
        left_vel /= static_cast<double>(registered_left_wheel_handles_.size());
        right_vel /= static_cast<double>(registered_right_wheel_handles_.size());
    }
    mobi_state_.wheel_pos = Eigen::Vector2d(left_pos, right_pos);
    mobi_state_.wheel_vel = Eigen::Vector2d(left_vel, right_vel);
}

void TestFR3HuskyController::updateRobotData()
{
    play_time_ = get_node()->now().seconds();

    mani_state_.M_total.setZero(manipulator_dof_, manipulator_dof_);
    for (size_t i = 0; i < num_robots_; ++i)
    {
        std::array<double, FR3_DOF*FR3_DOF> mass = franka_robot_model_[i]->getMassMatrix();
        std::array<double, FR3_DOF> coriolis = franka_robot_model_[i]->getCoriolisForceVector();
        std::array<double, FR3_DOF> gravity = franka_robot_model_[i]->getGravityForceVector();
        {
            std::lock_guard<std::mutex> lock(robot_data_mutex_);
            mani_state_.M[i] = Eigen::Map<const Eigen::Matrix<double, FR3_DOF, FR3_DOF, Eigen::RowMajor>>(mass.data());
            mani_state_.c[i] = Eigen::Map<const Eigen::Matrix<double, FR3_DOF, 1>>(coriolis.data());
            mani_state_.g[i] = Eigen::Map<const Eigen::Matrix<double, FR3_DOF, 1>>(gravity.data());
            // J_[i] = J_tmp;
            mani_state_.M_inv[i] = mani_state_.M[i].inverse();

            mani_state_.M_total.block(FR3_DOF*i, FR3_DOF*i, FR3_DOF, FR3_DOF) = mani_state_.M[i];
            mani_state_.M_inv_total.block(FR3_DOF*i, FR3_DOF*i, FR3_DOF, FR3_DOF) = mani_state_.M_inv[i];
            mani_state_.c_total.segment(FR3_DOF*i, FR3_DOF) = mani_state_.c[i];
            mani_state_.g_total.segment(FR3_DOF*i, FR3_DOF) = mani_state_.g[i];
        }
    }

    // Minimal placeholder: sync mobile base pose/vel from robot_data_ if available
    if (robot_data_)
    {
        try
        {
            // using odometry for getting virtual joint (you can use SLAM instead)
            mobi_state_.base_pose_w = robot_data_->computeBasePose(mobi_state_.wheel_pos, mobi_state_.wheel_vel);
            mobi_state_.base_vel_b  = robot_data_->computeBaseVel(mobi_state_.wheel_pos, mobi_state_.wheel_vel);
            mobi_state_.base_vel_w.head(2) = mobi_state_.base_pose_w.linear() * mobi_state_.base_vel_b.head(2);
            mobi_state_.base_vel_w(2) = mobi_state_.base_vel_b(2);

            const Eigen::Vector3d base_pose_w{mobi_state_.base_pose_w.translation()(0), 
                                              mobi_state_.base_pose_w.translation()(1),
                                              Eigen::Rotation2Dd(mobi_state_.base_pose_w.linear()).angle()};

            robot_data_->updateState(base_pose_w,             mobi_state_.wheel_pos, mani_state_.q_total,
                                     mobi_state_.base_vel_w,  mobi_state_.wheel_vel, mani_state_.qdot_total);
            
            
            for(auto & [ee_name, ee_data] : ee_data_)
            {
                ee_data.x = robot_data_->getPose(ee_name);
                ee_data.xdot = robot_data_->getVelocity(ee_name);
                ee_data.xddot.setZero();
            }


        }
        catch (const std::exception & e)
        {
            LOGW(get_node(), "RobotData update failed: %s", e.what());
        }
    }
}

bool TestFR3HuskyController::loadDRCGains()
{
    if (!robot_data_)
    {
        LOGE(get_node(), "RobotData is not initialized for gain setup.");
        return false;
    }
    if (!robot_controller_)
    {
        LOGE(get_node(), "RobotController is not initialized for gain setup.");
        return false;
    }

    constexpr size_t task_dof = 6;
    constexpr size_t mobile_base_dof = 3; // vx, vy, wz

    const auto & joint = params_.dyros_robot_controller.manipulator_joint_gains;
    const auto & task  = params_.dyros_robot_controller.task_gains;
    const auto & qpik  = params_.dyros_robot_controller.QPIK_weight;
    const auto & qpid  = params_.dyros_robot_controller.QPID_weight;

    auto check_vector_size = [this](const std::string & name, size_t expected, size_t got) -> bool
    {
        if (got != expected)
        {
            LOGE(get_node(), "Parameter '%s' expected %zu values, got %zu.", name.c_str(), expected, got);
            return false;
        }
        return true;
    };

    // manipulator joint gains
    if (!check_vector_size("dyros_robot_controller.manipulator_joint_gains.kp", manipulator_dof_, joint.kp.size())) return false;
    if (!check_vector_size("dyros_robot_controller.manipulator_joint_gains.kv", manipulator_dof_, joint.kv.size())) return false;

    // task-space gains
    if (task.link_names.empty())
    {
        LOGE(get_node(), "Parameter 'dyros_robot_controller.task_gains.link_names' must not be empty.");
        return false;
    }
    const size_t expected_task = task.link_names.size() * task_dof;
    if (!check_vector_size("dyros_robot_controller.task_gains.kp", expected_task, task.kp.size())) return false;
    if (!check_vector_size("dyros_robot_controller.task_gains.kv", expected_task, task.kv.size())) return false;

    // QPIK weights
    if (qpik.tracking.link_names.empty())
    {
        LOGE(get_node(), "Parameter 'dyros_robot_controller.QPIK_weight.tracking.link_names' must not be empty.");
        return false;
    }
    const size_t expected_qpik_tracking = qpik.tracking.link_names.size() * task_dof;
    if (!check_vector_size("dyros_robot_controller.QPIK_weight.tracking.weights", expected_qpik_tracking, qpik.tracking.weights.size())) return false;
    if (!check_vector_size("dyros_robot_controller.QPIK_weight.joint.velocity.manipulator", manipulator_dof_, qpik.joint.velocity.manipulator.size())) return false;
    if (!check_vector_size("dyros_robot_controller.QPIK_weight.joint.velocity.mobile", mobile_base_dof, qpik.joint.velocity.mobile.size())) return false;

    // QPID weights
    if (qpid.tracking.link_names.empty())
    {
        LOGE(get_node(), "Parameter 'dyros_robot_controller.QPID_weight.tracking.link_names' must not be empty.");
        return false;
    }
    const size_t expected_qpid_tracking = qpid.tracking.link_names.size() * task_dof;
    if (!check_vector_size("dyros_robot_controller.QPID_weight.tracking.weights", expected_qpid_tracking, qpid.tracking.weights.size())) return false;
    if (!check_vector_size("dyros_robot_controller.QPID_weight.joint.velocity.manipulator", manipulator_dof_, qpid.joint.velocity.manipulator.size())) return false;
    if (!check_vector_size("dyros_robot_controller.QPID_weight.joint.velocity.mobile", mobile_base_dof, qpid.joint.velocity.mobile.size())) return false;
    if (!check_vector_size("dyros_robot_controller.QPID_weight.joint.acceleration.manipulator", manipulator_dof_, qpid.joint.acceleration.manipulator.size())) return false;
    if (!check_vector_size("dyros_robot_controller.QPID_weight.joint.acceleration.mobile", mobile_base_dof, qpid.joint.acceleration.mobile.size())) return false;

    mani_joint_kp_ = Eigen::Map<const Eigen::VectorXd>(joint.kp.data(), joint.kp.size());
    mani_joint_kv_ = Eigen::Map<const Eigen::VectorXd>(joint.kv.data(), joint.kv.size());

    qpik_mani_damping_ = Eigen::Map<const Eigen::VectorXd>(qpik.joint.velocity.manipulator.data(), qpik.joint.velocity.manipulator.size());
    qpik_mobi_damping_ = Eigen::Map<const Eigen::Vector3d>(qpik.joint.velocity.mobile.data());

    qpid_mani_vel_damping_ = Eigen::Map<const Eigen::VectorXd>(qpid.joint.velocity.manipulator.data(), qpid.joint.velocity.manipulator.size());
    qpid_mani_acc_damping_ = Eigen::Map<const Eigen::VectorXd>(qpid.joint.acceleration.manipulator.data(), qpid.joint.acceleration.manipulator.size());
    qpid_mobi_vel_damping_ = Eigen::Map<const Eigen::Vector3d>(qpid.joint.velocity.mobile.data());
    qpid_mobi_acc_damping_ = Eigen::Map<const Eigen::Vector3d>(qpid.joint.acceleration.mobile.data());

    // task gains per ee
    task_kp_.clear();
    task_kv_.clear();
    qpik_tracking_.clear();
    qpid_tracking_.clear();

    for (size_t i = 0; i < task.link_names.size(); ++i)
    {
        const auto & link = task.link_names[i];
        const size_t base = i * task_dof;
        Eigen::Vector6d kp = Eigen::Map<const Eigen::Vector6d>(&task.kp[base]);
        Eigen::Vector6d kv = Eigen::Map<const Eigen::Vector6d>(&task.kv[base]);
        task_kp_[link] = kp;
        task_kv_[link] = kv;
    }

    for (size_t i = 0; i < qpik.tracking.link_names.size(); ++i)
    {
        const auto & link = qpik.tracking.link_names[i];
        const size_t base = i * task_dof;
        Eigen::Vector6d w = Eigen::Map<const Eigen::Vector6d>(&qpik.tracking.weights[base]);
        qpik_tracking_[link] = w;
    }

    for (size_t i = 0; i < qpid.tracking.link_names.size(); ++i)
    {
        const auto & link = qpid.tracking.link_names[i];
        const size_t base = i * task_dof;
        Eigen::Vector6d w = Eigen::Map<const Eigen::Vector6d>(&qpid.tracking.weights[base]);
        qpid_tracking_[link] = w;
    }

    robot_controller_->setManipulatorJointGain(mani_joint_kp_, mani_joint_kv_);
    robot_controller_->setTaskGain(task_kp_, task_kv_);
    robot_controller_->setQPIKGain(qpik_tracking_, qpik_mani_damping_, qpik_mobi_damping_);
    robot_controller_->setQPIDGain(qpid_tracking_, qpid_mani_vel_damping_, qpid_mani_acc_damping_, qpid_mobi_vel_damping_, qpid_mobi_acc_damping_);
    return true;
}

void TestFR3HuskyController::setInitfromCurrent()
{
    mani_state_.q_init = mani_state_.q;
    mani_state_.qdot_init = mani_state_.qdot;
    mani_state_.qddot_init = mani_state_.qddot;
    mani_state_.q_total_init = mani_state_.q_total;
    mani_state_.qdot_total_init = mani_state_.qdot_total;
    mani_state_.qddot_total_init = mani_state_.qddot_total;

    mobi_state_.base_pose_w_init = mobi_state_.base_pose_w;
    mobi_state_.base_vel_w_init = mobi_state_.base_vel_w;
    mobi_state_.base_vel_b_init = mobi_state_.base_vel_b;

    for(auto & [ee_name, ee_data] : ee_data_)
    {
        ee_data.setInit();
    }

    control_start_time_ = play_time_;
}

bool TestFR3HuskyController::reset()
{
    registered_left_wheel_handles_.clear();
    registered_right_wheel_handles_.clear();
    registered_manipulator_joint_handles_.clear();

    odom_timer_.reset();

    subscriber_is_active_ = false;
    velocity_command_unstamped_subscriber_.reset();

    received_velocity_msg_ptr_.set(nullptr);
    is_halted_ = false;
    return true;
}

bool TestFR3HuskyController::setJointIndex(const std::string& urdf_xml, drc::MobileManipulator::JointIndex& out_idx)
{
    // Init output
    out_idx.virtual_start = -1;
    out_idx.mani_start    = -1;
    out_idx.mobi_start    = -1;

    pinocchio::Model model;
    pinocchio::urdf::buildModelFromXML(urdf_xml, model);

    const int nj = (int)model.njoints;

    auto join_keys = [](const std::vector<std::string>& keys) -> std::string
    {
        std::ostringstream os;
        for (size_t i = 0; i < keys.size(); ++i)
        {
            if (i) os << ", ";
            os << "'" << keys[i] << "'";
        }
        return os.str();
    };

    auto preview_names = [&](const std::vector<int>& idxs) -> std::string
    {
        std::ostringstream os;
        const size_t limit = 6;
        for (size_t i = 0; i < idxs.size() && i < limit; ++i)
        {
            if (i) os << ", ";
            os << model.names[idxs[i]];
        }
        if (idxs.size() > limit) os << ", ...";
        return os.str();
    };

    // ---------- Common search lambda with detailed diagnostics ----------
    auto find_block = [&](const std::string& label,
                          const std::vector<std::string>& keys,
                          int N,
                          int& out_start_vs,
                          bool ignore_dummy = false) -> bool
    {
        if (N <= 0)
        {
            LOGE(get_node(), "[setJointIndex:%s] requested block size N=%d is invalid.", label.c_str(), N);
            return false;
        }

        std::vector<int> matches;
        matches.reserve(nj);
        for (int i = 1; i < nj; ++i) // skip universe joint (0)
        {
            const std::string& name = model.names[i];
            if (ignore_dummy && name.find("dummy") != std::string::npos) continue;
            bool match = true;
            for (const auto& key : keys)
            {
                if (name.find(key) == std::string::npos)
                {
                    match = false;
                    break;
                }
            }
            if (match) matches.push_back(i);
        }

        if ((int)matches.size() < N)
        {
            LOGE(get_node(),
                 "[setJointIndex:%s] Expected %d joints but found only %zu joints containing keys (%s). Example matches: %s",
                 label.c_str(), N, matches.size(), join_keys(keys).c_str(), preview_names(matches).c_str());
            return false;
        }

        std::string last_reason;
        int attempts = 0;
        for (int start = 1; start + N - 1 < nj; ++start)
        {
            ++attempts;
            std::string reason;

            // 1) keyword check
            for (int k = 0; k < N; ++k)
            {
                const std::string& name = model.names[start + k];
                if (ignore_dummy && name.find("dummy") != std::string::npos)
                {
                    reason = "Joint '" + name + "' marked as dummy; skipping.";
                    break;
                }
                for (const auto& key : keys)
                {
                    if (name.find(key) == std::string::npos)
                    {
                        reason = "Joint '" + name + "' does not contain keyword '" + key + "'";
                        break;
                    }
                }
                if (!reason.empty()) break;
            }
            if (!reason.empty()) { last_reason = reason; continue; }

            // 2) 1-DoF continuity check (nvs/nqs)
            for (int k = 0; k < N; ++k)
            {
                if (model.nvs[start + k] != 1 || model.nqs[start + k] != 1)
                {
                    std::ostringstream os;
                    os << "Joint '" << model.names[start + k] << "' is not 1-DoF (nv=" << model.nvs[start + k]
                       << ", nq=" << model.nqs[start + k] << ")";
                    reason = os.str();
                    break;
                }
            }
            if (!reason.empty()) { last_reason = reason; continue; }

            // 3) velocity index contiguity check
            const int v0 = model.idx_vs[start];
            for (int k = 1; k < N; ++k)
            {
                if (model.idx_vs[start + k] != v0 + k)
                {
                    std::ostringstream os;
                    os << "Contiguous index mismatch: '" << model.names[start + k] << "' expected=" << v0 + k
                       << ", actual=" << model.idx_vs[start + k];
                    reason = os.str();
                    break;
                }
            }
            if (!reason.empty()) { last_reason = reason; continue; }

            out_start_vs = model.idx_vs[start];
            return true;
        }

        LOGE(get_node(),
             "[setJointIndex:%s] Found %zu joints containing keys (%s) but no contiguous block of %d. Last failure: %s (checked %d positions)",
             label.c_str(), matches.size(), join_keys(keys).c_str(), N,
             last_reason.c_str(), attempts);
        return false;
    };

    // 1) Virtual joints (v_*_joint) : 3
    int virtual_vs = -1;
    if (!find_block("virtual", { "v_", "_joint" }, 3, virtual_vs)) return false;

    // 2) FR3 manipulator joints
    const int mani_N = FR3_DOF * (int)(num_robots_);
    int mani_vs = -1;
    if (!find_block("fr3", { arm_id_, "joint" }, mani_N, mani_vs)) return false;

    // 3) Wheel joints
    int wheel_vs = -1;
    if (!find_block("mobile", { "wheel" }, 2, wheel_vs, true)) return false;

    out_idx.virtual_start = virtual_vs;
    out_idx.mani_start    = mani_vs;
    out_idx.mobi_start    = wheel_vs;

    return true;
}

void TestFR3HuskyController::publishFromMobileStateBuffer()
{
    const MobileBaseState s = *mobi_state_pub_buf_.readFromRT();

    // yaw from pose
    const double yaw = Eigen::Rotation2Dd(s.base_pose_w.linear()).angle();

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);


    // Odometry publish 
    if (odometry_publisher_)
    {
        nav_msgs::msg::Odometry msg;
        msg.header.stamp = get_node()->now();   
        msg.header.frame_id = params_.odom_frame_id;
        msg.child_frame_id  = params_.base_frame_id;

        msg.pose.covariance.fill(0.0);
        msg.twist.covariance.fill(0.0);
        constexpr size_t N = 6;
        for (size_t i = 0; i < N; ++i)
        {
            const size_t diag = N * i + i;
            msg.pose.covariance[diag]  = params_.pose_covariance_diagonal[i];
            msg.twist.covariance[diag] = params_.twist_covariance_diagonal[i];
        }

        msg.pose.pose.position.x = s.base_pose_w.translation()(0);
        msg.pose.pose.position.y = s.base_pose_w.translation()(1);
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        msg.pose.pose.orientation.w = q.w();

        msg.twist.twist.linear.x  = s.base_vel_b(0);
        msg.twist.twist.angular.z = s.base_vel_b(2);

        odometry_publisher_->publish(msg);
    }

    // TF publish
    if (odometry_transform_publisher_)
    {
        tf2_msgs::msg::TFMessage tfm;
        tfm.transforms.resize(1);
        auto & t = tfm.transforms[0];

        t.header.stamp = get_node()->now();  // 위와 동일
        t.header.frame_id = params_.odom_frame_id;
        t.child_frame_id  = params_.base_frame_id;
        t.transform.translation.x = s.base_pose_w.translation()(0);
        t.transform.translation.y = s.base_pose_w.translation()(1);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        odometry_transform_publisher_->publish(tfm);
    }
}

}  // namespace fr3_husky_controller

#include "class_loader/register_macro.hpp"
CLASS_LOADER_REGISTER_CLASS(fr3_husky_controller::TestFR3HuskyController, controller_interface::ControllerInterface);
