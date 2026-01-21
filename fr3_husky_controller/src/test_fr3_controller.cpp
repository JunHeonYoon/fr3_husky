#include "fr3_husky_controller/test_fr3_controller.hpp"

namespace fr3_husky_controller 
{
// ========================================================================
// ============================ Core Functions ============================
// ========================================================================
controller_interface::InterfaceConfiguration TestFR3Controller::state_interface_configuration() const 
{
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint_name : params_.joints)
    {
        for (const auto & interface_type : params_.state_interfaces)
        {
            conf.names.push_back(joint_name + "/" + interface_type);
        }
    }
    for (size_t i = 0; i < num_robots_; ++i)
    {
        for (const auto& franka_robot_model_name : franka_robot_model_[i]->get_state_interface_names()) 
        {
            conf.names.push_back(franka_robot_model_name);
        }
        conf.names.push_back(params_.robot_name[i] + "_" + arm_id_ + "/robot_time");

    }

    return conf;
}

controller_interface::InterfaceConfiguration TestFR3Controller::command_interface_configuration() const 
{
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint_name : params_.joints)
    {
        for (const auto & interface_type : params_.command_interfaces)
        {
            conf.names.push_back(joint_name + "/" + interface_type);
        }
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
    // update the dynamic map parameters
    param_listener_->refresh_dynamic_parameters();

    // get parameters from the listener in case they were updated
    params_ = param_listener_->get_params();

    // get number of robots
    num_robots_ = params_.robot_name.size();
    if(num_robots_ < 1 || num_robots_ > 2)
    {
        LOGE(get_node(), "FR3 controller expects one or two FR3 robots, but got %zu.", num_robots_);
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

    // get degrees of freedom
    dof_ = params_.joints.size();
    if (dof_ != FR3_DOF*num_robots_)
    {
        LOGE(get_node(), "FR3 controller expects %zu DoF, but got %zu.", 7*num_robots_, dof_);
        return CallbackReturn::FAILURE;
    }

    if (params_.command_interfaces.empty())
    {
        LOGE(get_node(), "'command_interfaces' parameter is empty.");
        return CallbackReturn::FAILURE;
    }

    // Check if only allowed interface types are used and initialize storage to avoid memory allocation during activation
    joint_command_interface_.resize(allowed_interface_types_.size());
    for (auto & itf : joint_command_interface_)
    {
        itf.reserve(params_.joints.size());
    } 

    has_position_command_interface_ = robot_utils::contains_interface_type(params_.command_interfaces, hardware_interface::HW_IF_POSITION);
    has_velocity_command_interface_ = robot_utils::contains_interface_type(params_.command_interfaces, hardware_interface::HW_IF_VELOCITY);
    has_effort_command_interface_   = robot_utils::contains_interface_type(params_.command_interfaces, hardware_interface::HW_IF_EFFORT);

    if (params_.state_interfaces.empty())
    {
        LOGE(get_node(), "'state_interfaces' parameter is empty.");
        return CallbackReturn::FAILURE;
    }

    // Check if only allowed interface types are used and initialize storage to avoid memory allocation during activation
    joint_state_interface_.resize(allowed_interface_types_.size());
    for (auto & itf : joint_state_interface_)
    {
        itf.reserve(params_.joints.size());
    }

    has_position_state_interface_     = robot_utils::contains_interface_type(params_.state_interfaces, hardware_interface::HW_IF_POSITION);
    has_velocity_state_interface_     = robot_utils::contains_interface_type(params_.state_interfaces, hardware_interface::HW_IF_VELOCITY);
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

    if (get_update_rate() == 0)
    {
        throw std::runtime_error("Controller's update rate is set to 0. This should not happen!");
    }
    dt_ = 1.0 / static_cast<double>(get_update_rate());

    // initialize franka robot data
    franka_robot_model_.clear();
    for (const auto & name : params_.robot_name)
    {
        franka_robot_model_.push_back(
            std::make_unique<franka_semantic_components::FrankaRobotModel>(name + "_" + arm_id_ + "/robot_model", 
                                                                           name + "_" + arm_id_ + "/robot_state")
        );
    }
    
    // initialize dyros_robot_data & controller
    const std::string description_pkg = ament_index_cpp::get_package_share_directory("fr3_husky_description");
    std::string xacro_path = description_pkg + "/robots/";
    std::ostringstream segmentation_args;
    segmentation_args << " with_sc:=true"
                      << " hand:=" << (params_.hand ? "true" : "false")
                      << " mobile:=" << (params_.mobile_base ? "true" : "false");
    std::string robot_segmentation_description_param = segmentation_args.str();
    std::string robot_description_param = robot_segmentation_description_param + " fix_finger:=true";

    if(num_robots_ == 1)
    {
        robot_segmentation_description_param = robot_segmentation_description_param + " side:=" + params_.robot_name[0];
        robot_description_param = robot_description_param + " side:=" + params_.robot_name[0];
        xacro_path = xacro_path + "single_fr3";
    }
    else if(num_robots_ == 2)
    {
        xacro_path = xacro_path + "dual_fr3";
    }
    
    const std::string urdf_xml = robot_utils::execAndCaptureStdout("xacro " + xacro_path + ".urdf.xacro" + robot_description_param);
    const std::string srdf_xml = robot_utils::execAndCaptureStdout("xacro " + xacro_path + ".srdf.xacro" + robot_segmentation_description_param);

    try
    {
        robot_data_ = std::make_shared<drc::Manipulator::RobotData>(urdf_xml, srdf_xml, description_pkg, true);
    }
    catch (const std::exception& e)
    {
        LOGE(get_node(), "Failed to initialize RobotData: %s", e.what());
        return CallbackReturn::ERROR;
    }
    
    if (robot_data_->getDof() != dof_)
    {
        LOGE(get_node(), "Robot model DoF is %d, expected %d for FR3.", robot_data_->getDof(), dof_);
        return CallbackReturn::FAILURE;
    }

    try
    {
        robot_controller_ = std::make_unique<drc::Manipulator::RobotController>(dt_, robot_data_);
    }
    catch (const std::exception& e)
    {
        LOGE(get_node(), "Failed to initialize RobotController: %s", e.what());
        return CallbackReturn::ERROR;
    }
    
    if (!loadDRCGains())
    {
        return CallbackReturn::FAILURE;
    }

    q_init_.assign(num_robots_, Eigen::Vector7d::Zero());
    qdot_init_.assign(num_robots_, Eigen::Vector7d::Zero());
    qddot_init_.assign(num_robots_, Eigen::Vector7d::Zero());
    q_total_init_.setZero(dof_);
    qdot_total_init_.setZero(dof_);
    qddot_total_init_.setZero(dof_);

    q_.assign(num_robots_, Eigen::Vector7d::Zero());
    qdot_.assign(num_robots_, Eigen::Vector7d::Zero());
    qddot_.assign(num_robots_, Eigen::Vector7d::Zero());
    torque_.assign(num_robots_, Eigen::Vector7d::Zero());
    q_total_.setZero(dof_);
    qdot_total_.setZero(dof_);
    qddot_total_.setZero(dof_);
    torque_total_.setZero(dof_);

    q_desired_.assign(num_robots_, Eigen::Vector7d::Zero());
    qdot_desired_.assign(num_robots_, Eigen::Vector7d::Zero());
    torque_desired_.assign(num_robots_, Eigen::Vector7d::Zero());
    q_desired_total_.setZero(dof_);
    qdot_desired_total_.setZero(dof_);
    torque_desired_total_.setZero(dof_);

    M_.assign(num_robots_, Eigen::Matrix7d::Zero());
    M_inv_.assign(num_robots_, Eigen::Matrix7d::Zero());
    c_.assign(num_robots_, Eigen::Vector7d::Zero());
    g_.assign(num_robots_, Eigen::Vector7d::Zero());
    M_total_.setZero(dof_, dof_);
    M_inv_total_.setZero(dof_, dof_);
    c_total_.setZero(dof_);
    g_total_.setZero(dof_);

    x_init_.assign(num_robots_, Eigen::Affine3d::Identity());
    xdot_init_.assign(num_robots_, Eigen::Vector6d::Zero());
    
    x_.assign(num_robots_, Eigen::Affine3d::Identity());
    xdot_.assign(num_robots_, Eigen::Vector6d::Zero());
    J_.assign(num_robots_, Eigen::Matrix<double, 6, FR3_DOF>::Zero());
    
    x_desired_.assign(num_robots_, Eigen::Affine3d::Identity());
    xdot_desired_.assign(num_robots_, Eigen::Vector6d::Zero());
    
    ee_name_.clear();
    for (const auto & name : params_.robot_name)
    {
        std::string ee_name = name + "_" + arm_id_ + "_";
        if (params_.hand) ee_name = ee_name + "hand_tcp";
        else              ee_name = ee_name + "link8";
        ee_name_.push_back(ee_name);
    }

  return CallbackReturn::SUCCESS;
}

CallbackReturn TestFR3Controller::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) 
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
        if (!controller_interface::get_ordered_interfaces(command_interfaces_, params_.joints, interface, joint_command_interface_[index]))
        {
            LOGE(get_node(), "Expected %zu '%s' command interfaces, got %zu.", dof_, interface.c_str(), joint_command_interface_[index].size());
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

    for (const auto & model : franka_robot_model_)
    {
        model->assign_loaned_state_interfaces(state_interfaces_);
    }

    updateJointStates();
    updateRobotData();
    setInitfromCurrent();

    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TestFR3Controller::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) 
{
    for (const auto & model : franka_robot_model_)
    {
        model->release_interfaces();
    }

    for (size_t index = 0; index < dof_; ++index)
    {
        if (has_position_command_interface_)
        {
            const double current_position = joint_command_interface_[0][index].get().get_value();
            joint_command_interface_[0][index].get().set_value(current_position);
        }

        if (has_velocity_command_interface_)
        {
            joint_command_interface_[1][index].get().set_value(0.0);
        }

        // TODO(anyone): How to halt when using effort commands?
        if (has_effort_command_interface_)
        {   
            joint_command_interface_[3][index].get().set_value(0.0);
        }
    }

    for (size_t index = 0; index < allowed_interface_types_.size(); ++index)
    {
        joint_command_interface_[index].clear();
        joint_state_interface_[index].clear();
    }

    return CallbackReturn::SUCCESS;
}

controller_interface::return_type TestFR3Controller::update(const rclcpp::Time& time, const rclcpp::Duration& /*period*/)
{
    const auto update_start = std::chrono::steady_clock::now();

    updateJointStates();
    updateRobotData();

    Eigen::VectorXd command_position;
    Eigen::VectorXd command_velocity;
    Eigen::VectorXd command_effort;

    // example: Gravity Compensation with QPID
    std::map<std::string, drc::TaskSpaceData> tmp;
    tmp[ee_name_[0]] = drc::TaskSpaceData::Zero();
    command_effort = robot_controller_->QPID(tmp, false) -g_total_;
    // command_effort.setZero(dof_);
    command_position.setZero(dof_);
    command_velocity.setZero(dof_);

    writeCommandInterfaces(command_position, command_velocity, command_effort);

    const double update_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - update_start).count();
    if (update_ms > 0.9)
    {
        LOGW(get_node(), "update took %.3f ms", update_ms);
    }
    else
    {
        LOGI(get_node(), "update took %.3f ms", update_ms);
    }

    return controller_interface::return_type::OK;
}

void TestFR3Controller::writeCommandInterfaces(const Eigen::VectorXd& command_position,
                                               const Eigen::VectorXd& command_velocity,
                                               const Eigen::VectorXd& command_effort)
{
    const Eigen::VectorXd* cmd_pos = &command_position;
    const Eigen::VectorXd* cmd_vel = &command_velocity;
    const Eigen::VectorXd* cmd_eff = &command_effort;
    Eigen::VectorXd zero_pos;
    Eigen::VectorXd zero_vel;
    Eigen::VectorXd zero_eff;
    if (command_position.size() != dof_ || command_velocity.size() != dof_ || command_effort.size() != dof_)
    {
        LOGW(get_node(),
             "Command vector sizes must match dof_=%zu (pos %zu, vel %zu, eff %zu).",
             dof_,
             command_position.size(),
             command_velocity.size(),
             command_effort.size());

        zero_pos.setZero(dof_);
        zero_vel.setZero(dof_);
        zero_eff.setZero(dof_);
        cmd_pos = &zero_pos;
        cmd_vel = &zero_vel;
        cmd_eff = &zero_eff;
    }
    constexpr size_t kPositionIndex = 0;
    constexpr size_t kVelocityIndex = 1;
    constexpr size_t kEffortIndex = 3;

    static bool warned_no_effort = false;
    if (has_effort_command_interface_ || has_position_command_interface_ || has_velocity_command_interface_)
    {
        for (size_t cmd_index = 0; cmd_index < dof_; ++cmd_index)
        {
            if (has_position_command_interface_)
            {
                joint_command_interface_[kPositionIndex][cmd_index].get().set_value((*cmd_pos)(cmd_index));
            }
            if (has_velocity_command_interface_)
            {
                joint_command_interface_[kVelocityIndex][cmd_index].get().set_value((*cmd_vel)(cmd_index));
            }
            if (has_effort_command_interface_)
            {
                joint_command_interface_[kEffortIndex][cmd_index].get().set_value((*cmd_eff)(cmd_index));
            }
        }
    }
    else if (!warned_no_effort)
    {
        warned_no_effort = true;
        LOGE(get_node(), "No command interface available; controller will not send commands.");
    }
}

void TestFR3Controller::updateJointStates() 
{
    auto assign_jvalue_from_state_interface = [&](Eigen::VectorXd & joint_value, const auto & joint_interface)
    {
        joint_value.setZero(dof_);
        for (size_t index = 0; index < dof_; ++index)
        {
            joint_value(index) = joint_interface[index].get().get_value();
        }
    };

    std::lock_guard<std::mutex> lk(robot_data_mutex_);
    const Eigen::VectorXd last_qdot_total = qdot_total_;
    if (has_position_state_interface_)  assign_jvalue_from_state_interface(q_total_,      joint_state_interface_[0]);
    if (has_velocity_state_interface_)  assign_jvalue_from_state_interface(qdot_total_,   joint_state_interface_[1]);
    if (has_effort_state_interface_)    assign_jvalue_from_state_interface(torque_total_, joint_state_interface_[3]);
    if (has_velocity_state_interface_)  qddot_total_ = (qdot_total_ - last_qdot_total) / dt_;
    
    for (size_t i = 0; i < num_robots_; ++i)
    {
        q_[i]      = q_total_.segment(FR3_DOF*i, FR3_DOF);
        qdot_[i]   = qdot_total_.segment(FR3_DOF*i, FR3_DOF);
        qddot_[i]  = qddot_total_.segment(FR3_DOF*i, FR3_DOF);
        torque_[i] = torque_total_.segment(FR3_DOF*i, FR3_DOF);
    }
}

void TestFR3Controller::updateRobotData()
{
    M_total_.setZero(dof_, dof_);

    for (size_t i = 0; i < num_robots_; ++i)
    {
        std::array<double, FR3_DOF*FR3_DOF> mass = franka_robot_model_[i]->getMassMatrix();
        std::array<double, FR3_DOF> coriolis = franka_robot_model_[i]->getCoriolisForceVector();
        std::array<double, FR3_DOF> gravity = franka_robot_model_[i]->getGravityForceVector();
        std::array<double, 16> pose = franka_robot_model_[i]->getPoseMatrix(franka::Frame::kEndEffector);
        std::array<double, FR3_DOF*6> endeffector_jacobian_wrt_base = franka_robot_model_[i]->getZeroJacobian(franka::Frame::kEndEffector);
        {
            std::lock_guard<std::mutex> lock(robot_data_mutex_);
            M_[i] = Eigen::Map<const Eigen::Matrix<double, FR3_DOF, FR3_DOF, Eigen::RowMajor>>(mass.data());
            c_[i] = Eigen::Map<const Eigen::Matrix<double, FR3_DOF, 1>>(coriolis.data());
            g_[i] = Eigen::Map<const Eigen::Matrix<double, FR3_DOF, 1>>(gravity.data());
            x_[i].matrix() = Eigen::Map<const Eigen::Matrix4d>(pose.data());
            Eigen::Map<const Eigen::Matrix<double,6,FR3_DOF,Eigen::ColMajor>> J_tmp(endeffector_jacobian_wrt_base.data());
            J_[i] = J_tmp;
            M_inv_[i] = M_[i].inverse();
            xdot_[i] = J_[i] * qdot_[i];

            M_total_.block(FR3_DOF*i, FR3_DOF*i, FR3_DOF, FR3_DOF) = M_[i];
            M_inv_total_.block(FR3_DOF*i, FR3_DOF*i, FR3_DOF, FR3_DOF) = M_inv_[i];
            c_total_.segment(FR3_DOF*i, FR3_DOF) = c_[i];
            g_total_.segment(FR3_DOF*i, FR3_DOF) = g_[i];
        }
    }
    {
        std::lock_guard<std::mutex> lock(robot_data_mutex_);
        robot_data_->updateState(q_total_, qdot_total_);
        play_time_ = get_node()->now().seconds();
    }
}

void TestFR3Controller::setInitfromCurrent()
{
    std::lock_guard<std::mutex> lock(robot_data_mutex_);
    for (size_t i = 0; i < num_robots_; ++i)
    {
        q_init_[i] = q_[i];
        qdot_init_[i] = qdot_[i];
        qddot_init_[i] = qddot_[i];

        q_desired_[i] = q_[i];
        qdot_desired_[i] = qdot_[i];
        torque_desired_[i] = torque_[i];

        x_init_[i] = x_[i];
        xdot_init_[i] = xdot_[i];

        x_desired_[i] = x_[i];
        xdot_desired_[i] = xdot_[i];
    }

    q_total_init_ = q_total_;
    qdot_total_init_ = qdot_total_;
    qddot_total_init_ = qddot_total_;

    hold_init_valid_ = true;

    q_desired_total_ = q_total_;
    qdot_desired_total_ = qdot_total_;
    torque_desired_total_ = torque_total_;

    x_desired_ = x_;
    xdot_desired_ = xdot_;

    control_start_time_ = get_node()->now().seconds();
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

    auto check_vector_size = [this](const std::string& name, const std::vector<double>& values, size_t expected) -> bool
    {
        if (values.size() != expected)
        {
            LOGE(get_node(), "Parameter '%s' expected %zu values, got %zu.", name.c_str(), expected, values.size());
            return false;
        }
        return true;
    };

    const auto& joint_params = params_.dyros_robot_controller.joint;
    const auto& task_params = params_.dyros_robot_controller.task;

    const auto& joint_kp_vec = joint_params.joint_kp;
    const auto& joint_kv_vec = joint_params.joint_kv;
    const auto& qpik_damping_vec = joint_params.qpik_damping;
    const auto& qpid_vel_damping_vec = joint_params.qpid_vel_damping;
    const auto& qpid_acc_damping_vec = joint_params.qpid_acc_damping;

    const auto& link_names = task_params.link_names;
    const auto& task_kp_vec = task_params.task_kp;
    const auto& task_kv_vec = task_params.task_kv;
    const auto& qpik_tracking_vec = task_params.qpik_tracking;
    const auto& qpid_tracking_vec = task_params.qpid_tracking;

    if (link_names.empty())
    {
        LOGE(get_node(), "Parameter 'dyros_robot_controller.task.link_names' must not be empty.");
        return false;
    }

    if (!check_vector_size("dyros_robot_controller.joint.joint_kp", joint_kp_vec, dof_)) return false;
    if (!check_vector_size("dyros_robot_controller.joint.joint_kv", joint_kv_vec, dof_)) return false;
    if (!check_vector_size("dyros_robot_controller.joint.qpik_damping", qpik_damping_vec, dof_)) return false;
    if (!check_vector_size("dyros_robot_controller.joint.qpid_vel_damping", qpid_vel_damping_vec, dof_)) return false;
    if (!check_vector_size("dyros_robot_controller.joint.qpid_acc_damping", qpid_acc_damping_vec, dof_)) return false;

    const size_t task_dof = 6;
    const size_t expected_task_size = link_names.size() * task_dof;
    if (!check_vector_size("dyros_robot_controller.task.task_kp", task_kp_vec, expected_task_size)) return false;
    if (!check_vector_size("dyros_robot_controller.task.task_kv", task_kv_vec, expected_task_size)) return false;
    if (!check_vector_size("dyros_robot_controller.task.qpik_tracking", qpik_tracking_vec, expected_task_size)) return false;
    if (!check_vector_size("dyros_robot_controller.task.qpid_tracking", qpid_tracking_vec, expected_task_size)) return false;

    joint_kp_ = Eigen::Map<const Eigen::VectorXd>(joint_kp_vec.data(), joint_kp_vec.size());
    joint_kv_ = Eigen::Map<const Eigen::VectorXd>(joint_kv_vec.data(), joint_kv_vec.size());
    qpik_damping_ = Eigen::Map<const Eigen::VectorXd>(qpik_damping_vec.data(), qpik_damping_vec.size());
    qpid_vel_damping_ = Eigen::Map<const Eigen::VectorXd>(qpid_vel_damping_vec.data(), qpid_vel_damping_vec.size());
    qpid_acc_damping_ = Eigen::Map<const Eigen::VectorXd>(qpid_acc_damping_vec.data(), qpid_acc_damping_vec.size());

    link_task_kp_.clear();
    link_task_kv_.clear();
    link_qpik_tracking_.clear();
    link_qpid_tracking_.clear();

    for (size_t i = 0; i < link_names.size(); ++i)
    {
        const auto& link_name = link_names[i];
        if (robot_data_ && !robot_data_->hasLinkFrame(link_name))
        {
            LOGW(get_node(), "Task link '%s' not found in robot model.", link_name.c_str());
            continue;
        }
        const size_t base = i * task_dof;
        Eigen::Vector6d task_kp;
        Eigen::Vector6d task_kv;
        Eigen::Vector6d qpik_tracking;
        Eigen::Vector6d qpid_tracking;
        for (size_t j = 0; j < task_dof; ++j)
        {
            const size_t idx = base + j;
            task_kp[j] = task_kp_vec[idx];
            task_kv[j] = task_kv_vec[idx];
            qpik_tracking[j] = qpik_tracking_vec[idx];
            qpid_tracking[j] = qpid_tracking_vec[idx];
        }
        link_task_kp_[link_name] = task_kp;
        link_task_kv_[link_name] = task_kv;
        link_qpik_tracking_[link_name] = qpik_tracking;
        link_qpid_tracking_[link_name] = qpid_tracking;
    }

    robot_controller_->setJointGain(joint_kp_, joint_kv_);
    robot_controller_->setTaskGain(link_task_kp_, link_task_kv_);
    robot_controller_->setQPIKGain(link_qpik_tracking_, qpik_damping_);
    robot_controller_->setQPIDGain(link_qpid_tracking_, qpid_vel_damping_, qpid_acc_damping_);
    return true;
}

}  // namespace fr3_husky_controller
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(fr3_husky_controller::TestFR3Controller,
                       controller_interface::ControllerInterface)

        
