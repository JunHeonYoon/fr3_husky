#include "fr3_husky_controller/test_fr3_controller.hpp"

namespace fr3_husky_controller 
{
controller_interface::InterfaceConfiguration TestFR3Controller::state_interface_configuration() const 
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
    
    return conf;
}

controller_interface::InterfaceConfiguration TestFR3Controller::command_interface_configuration() const 
{
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    // Manipulator command interfaces
    for (const auto & joint_name : params_.manipulator_joints)
    {
        conf.names.push_back(joint_name + "/" + params_.manipulator_command_interface);
    }

    return conf;
}

CallbackReturn TestFR3Controller::on_init() 
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

CallbackReturn TestFR3Controller::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) 
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

    // command flags (exactly one will be true)
    has_position_command_interface_ = (params_.manipulator_command_interface == allowed_interface_types_[0]);
    has_velocity_command_interface_ = (params_.manipulator_command_interface == allowed_interface_types_[1]);
    has_effort_command_interface_   = (params_.manipulator_command_interface == allowed_interface_types_[2]);

    has_position_state_interface_ = robot_utils::contains_interface_type(params_.manipulator_state_interfaces, allowed_interface_types_[0]);
    has_velocity_state_interface_ = robot_utils::contains_interface_type(params_.manipulator_state_interfaces, allowed_interface_types_[1]);
    has_effort_state_interface_   = robot_utils::contains_interface_type(params_.manipulator_state_interfaces, allowed_interface_types_[2]);

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
                      << " mobile:=" << (params_.mobile_base ? "true" : "false");
    std::string robot_segmentation_description_param = segmentation_args.str();
    std::string robot_description_param = robot_segmentation_description_param + " fix_finger:=true"
                                                                               + " ros2_control:=false"
                                                                               + " use_fake_hardware:=false"
                                                                               + " fake_sensor_commands:=false";

    if(num_robots_ == 1)
    {
        robot_segmentation_description_param += " side:=" + params_.robot_name[0];
        robot_description_param += " side:=" + params_.robot_name[0];
        xacro_path += "single_fr3";
    }
    else if(num_robots_ == 2)
    {
        xacro_path += "dual_fr3";
    }
    
    const std::string urdf_xml = robot_utils::execAndCaptureStdout("xacro " + xacro_path + ".urdf.xacro" + robot_description_param);
    const std::string srdf_xml = robot_utils::execAndCaptureStdout("xacro " + xacro_path + ".srdf.xacro" + robot_segmentation_description_param);

    try
    {
        robot_data_ = std::make_shared<drc::Manipulator::RobotData>(dt_, urdf_xml, srdf_xml, description_pkg, true);
        robot_controller_ = std::make_unique<drc::Manipulator::RobotController>(robot_data_);
        
        LOGI(get_node(), robot_data_->getVerbose().c_str());
    }
    catch (const std::exception& e)
    {
        LOGE(get_node(), "Failed to initialize RobotData: %s", e.what());
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

    return CallbackReturn::SUCCESS;
}

CallbackReturn TestFR3Controller::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) 
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

    updateJointStates();
    updateRobotData();
    setInitfromCurrent();

    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TestFR3Controller::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) 
{
    if (!is_halted_)
    {
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
    registered_manipulator_joint_handles_.clear();

    return CallbackReturn::SUCCESS;
}

controller_interface::return_type TestFR3Controller::update(const rclcpp::Time& time, const rclcpp::Duration& /*period*/)
{
    if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
        if (!is_halted_)
        {
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

    // ------------ QPID for manipulator ------------
    if (has_effort_command_interface_)
    {
        std::map<std::string, drc::TaskSpaceData> zero_data;
        std::map<std::string, Eigen::Vector6d> zero_qpid_tracking;
        for (const auto & link_name : params_.dyros_robot_controller.QPID_weight.tracking.link_names)
        {
            zero_data[link_name] = drc::TaskSpaceData::Zero();
            zero_qpid_tracking[link_name] = Eigen::Vector6d::Zero();
        }
        robot_controller_->setQPIDGain(zero_qpid_tracking, qpid_mani_vel_damping_, qpid_mani_acc_damping_);
    
        Eigen::VectorXd opt_torque;
        opt_torque.setZero(manipulator_dof_);

        robot_data_->getMinDistance(false, false, true);
    
        if(!robot_controller_->QPID(zero_data, opt_torque))
        {
            mani_state_.q_desired_total = mani_state_.q_total_init;
            mani_state_.qdot_desired_total.setZero();
            mani_state_.torque_desired_total = robot_data_->getGravity() - mani_state_.g_total;
    
            writeCommandInterfaces(mani_state_.torque_desired_total);
        }
        else
        {
            mani_state_.torque_desired_total = opt_torque - mani_state_.g_total;
    
            writeCommandInterfaces(mani_state_.torque_desired_total);
        }
    }
    else
    {
        LOGW(get_node(), "FR3 is not effort control interface, but %s. QPID did not operated.", has_position_command_interface_ ? "position" : "velocity");
        mani_state_.q_desired_total = mani_state_.q_total_init;
        mani_state_.qdot_desired_total.setZero();

        const Eigen::VectorXd& mani_cmd = has_position_command_interface_ ? mani_state_.q_desired_total :
                                                                            mani_state_.qdot_desired_total;
                            
        writeCommandInterfaces(mani_cmd);

    }


    return controller_interface::return_type::OK;
}

void TestFR3Controller::writeCommandInterfaces(const Eigen::VectorXd& command)
{
    // ---- Manipulator ----
    if (static_cast<size_t>(command.size()) == manipulator_dof_)
    {
        for (size_t i = 0; i < registered_manipulator_joint_handles_.size(); ++i)
            registered_manipulator_joint_handles_[i].command.get().set_value(command(i));
    }
    else
    {
        LOGW(get_node(), "Manipulator cmd size mismatch (expected %zu, got %zu). Holding/zeroing.", manipulator_dof_, command.size());
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

}

void TestFR3Controller::updateJointStates() 
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
}

void TestFR3Controller::updateRobotData()
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

    if (robot_data_)
    {
        try
        {
            robot_data_->updateState(mani_state_.q_total, mani_state_.qdot_total);
            
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

void TestFR3Controller::setInitfromCurrent()
{
    mani_state_.q_init = mani_state_.q;
    mani_state_.qdot_init = mani_state_.qdot;
    mani_state_.qddot_init = mani_state_.qddot;
    mani_state_.q_total_init = mani_state_.q_total;
    mani_state_.qdot_total_init = mani_state_.qdot_total;
    mani_state_.qddot_total_init = mani_state_.qddot_total;

    for(auto & [ee_name, ee_data] : ee_data_)
    {
        ee_data.setInit();
    }

    control_start_time_ = play_time_;
}

bool TestFR3Controller::loadDRCGains()
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

    // QPID weights
    if (qpid.tracking.link_names.empty())
    {
        LOGE(get_node(), "Parameter 'dyros_robot_controller.QPID_weight.tracking.link_names' must not be empty.");
        return false;
    }
    const size_t expected_qpid_tracking = qpid.tracking.link_names.size() * task_dof;
    if (!check_vector_size("dyros_robot_controller.QPID_weight.tracking.weights", expected_qpid_tracking, qpid.tracking.weights.size())) return false;
    if (!check_vector_size("dyros_robot_controller.QPID_weight.joint.velocity.manipulator", manipulator_dof_, qpid.joint.velocity.manipulator.size())) return false;
    if (!check_vector_size("dyros_robot_controller.QPID_weight.joint.acceleration.manipulator", manipulator_dof_, qpid.joint.acceleration.manipulator.size())) return false;

    mani_joint_kp_ = Eigen::Map<const Eigen::VectorXd>(joint.kp.data(), joint.kp.size());
    mani_joint_kv_ = Eigen::Map<const Eigen::VectorXd>(joint.kv.data(), joint.kv.size());

    qpik_mani_damping_ = Eigen::Map<const Eigen::VectorXd>(qpik.joint.velocity.manipulator.data(), qpik.joint.velocity.manipulator.size());

    qpid_mani_vel_damping_ = Eigen::Map<const Eigen::VectorXd>(qpid.joint.velocity.manipulator.data(), qpid.joint.velocity.manipulator.size());
    qpid_mani_acc_damping_ = Eigen::Map<const Eigen::VectorXd>(qpid.joint.acceleration.manipulator.data(), qpid.joint.acceleration.manipulator.size());

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

    robot_controller_->setJointGain(mani_joint_kp_, mani_joint_kv_);
    robot_controller_->setTaskGain(task_kp_, task_kv_);
    robot_controller_->setQPIKGain(qpik_tracking_, qpik_mani_damping_);
    robot_controller_->setQPIDGain(qpid_tracking_, qpid_mani_vel_damping_, qpid_mani_acc_damping_);
    return true;
}

}  // namespace fr3_husky_controller
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(fr3_husky_controller::TestFR3Controller,
                       controller_interface::ControllerInterface)

        
