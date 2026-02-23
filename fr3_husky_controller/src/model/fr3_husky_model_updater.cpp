#include "fr3_husky_controller/model/fr3_husky_model_updater.hpp"

namespace fr3_husky_controller
{

bool FR3HuskyModelUpdater::initialize(size_t num_robots,
                                      size_t manipulator_dof,
                                      double dt,
                                      const std::vector<std::string>& ee_names)
{
    if (!ModelUpdaterBase::initialize(num_robots, manipulator_dof, dt, ee_names))
    {
        return false;
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

    ee_data_.clear();
    for (const auto& name : ee_names_)
    {
        ee_data_[name] = drc::TaskSpaceData::Zero();
    }

    // Mobile base init
    mobi_state_.base_pose_w.setIdentity();
    mobi_state_.base_pose_w_desired.setIdentity();
    mobi_state_.base_pose_w_init.setIdentity();
    mobi_state_.base_vel_w.setZero();
    mobi_state_.base_vel_w_desired.setZero();
    mobi_state_.base_vel_w_init.setZero();
    mobi_state_.base_vel_b.setZero();
    mobi_state_.base_vel_b_desired.setZero();
    mobi_state_.base_vel_b_init.setZero();
    mobi_state_.wheel_pos.setZero();
    mobi_state_.wheel_vel.setZero();
    mobi_state_.wheel_vel_desired.setZero();

    return true;
}

void FR3HuskyModelUpdater::updateJointStates()
{
    if (!is_configured_ || !getHandlesReady()) return;

    // update FR3 joint state 
    const Eigen::VectorXd last_qdot_total = mani_state_.qdot_total;
    for (size_t i = 0; i < manipulator_dof_; ++i)
    {
        if (has_position_state_interface_) mani_state_.q_total(i)      = robot_handle_.mani_joints[i].state[kPositionIndex].get().get_value();
        if (has_velocity_state_interface_) mani_state_.qdot_total(i)   = robot_handle_.mani_joints[i].state[kVelocityIndex].get().get_value();
        if (has_effort_state_interface_)   mani_state_.torque_total(i) = robot_handle_.mani_joints[i].state[kEffortIndex].get().get_value();
    }
    mani_state_.qddot_total = (mani_state_.qdot_total - last_qdot_total) / dt_;

    for (size_t r = 0; r < num_robots_; ++r)
    {
        mani_state_.q[r]      = mani_state_.q_total.segment(FR3_DOF * r, FR3_DOF);
        mani_state_.qdot[r]   = mani_state_.qdot_total.segment(FR3_DOF * r, FR3_DOF);
        mani_state_.qddot[r]  = mani_state_.qddot_total.segment(FR3_DOF * r, FR3_DOF);
        mani_state_.torque[r] = mani_state_.torque_total.segment(FR3_DOF * r, FR3_DOF);
    }

    // update Husky joint state
    double left_pos = 0.0, right_pos = 0.0, left_vel = 0.0, right_vel = 0.0;
    for (size_t i = 0; i < robot_handle_.left_wheels.size(); ++i)
    {
        const auto & left_state = robot_handle_.left_wheels[i].state;
        const auto & right_state = robot_handle_.right_wheels[i].state;
        left_pos += left_state[kFeedbackPositionIndex].get().get_value();
        right_pos += right_state[kFeedbackPositionIndex].get().get_value();
        left_vel += left_state[kFeedbackVelocityIndex].get().get_value();
        right_vel += right_state[kFeedbackVelocityIndex].get().get_value();
    }
    if (!robot_handle_.left_wheels.empty())
    {
        left_pos /= static_cast<double>(robot_handle_.left_wheels.size());
        right_pos /= static_cast<double>(robot_handle_.right_wheels.size());
        left_vel /= static_cast<double>(robot_handle_.left_wheels.size());
        right_vel /= static_cast<double>(robot_handle_.right_wheels.size());
    }
    mobi_state_.wheel_pos = Eigen::Vector2d(left_pos, right_pos);
    mobi_state_.wheel_vel = Eigen::Vector2d(left_vel, right_vel);
}

void FR3HuskyModelUpdater::updateRobotData()
{
    if (!is_configured_)
    {
        return;
    }

    if (!franka_robot_model_)
    {
        if (node_) RCLCPP_WARN(node_->get_logger(), "Franka robot model pointer is null; skipping model update.");
        return;
    }

    if (!robot_data_)
    {
        if (node_) RCLCPP_WARN(node_->get_logger(), "DRC robot data pointer is null; skipping model update.");
        return;
    }

    // FR3 model update
    mani_state_.M_total.setZero(manipulator_dof_, manipulator_dof_);
    for (size_t i = 0; i < num_robots_; ++i)
    {
        std::array<double, FR3_DOF * FR3_DOF> mass = (*franka_robot_model_)[i]->getMassMatrix();
        std::array<double, FR3_DOF> coriolis = (*franka_robot_model_)[i]->getCoriolisForceVector();
        std::array<double, FR3_DOF> gravity = (*franka_robot_model_)[i]->getGravityForceVector();

        {
            std::lock_guard<std::mutex> lock(robot_data_mutex_);
            mani_state_.M[i] = Eigen::Map<const Eigen::Matrix<double, FR3_DOF, FR3_DOF, Eigen::RowMajor>>(mass.data());
            mani_state_.c[i] = Eigen::Map<const Eigen::Matrix<double, FR3_DOF, 1>>(coriolis.data());
            mani_state_.g[i] = Eigen::Map<const Eigen::Matrix<double, FR3_DOF, 1>>(gravity.data());
            mani_state_.M_inv[i] = mani_state_.M[i].inverse();

            mani_state_.M_total.block(FR3_DOF * i, FR3_DOF * i, FR3_DOF, FR3_DOF) = mani_state_.M[i];
            mani_state_.M_inv_total.block(FR3_DOF * i, FR3_DOF * i, FR3_DOF, FR3_DOF) = mani_state_.M_inv[i];
            mani_state_.c_total.segment(FR3_DOF * i, FR3_DOF) = mani_state_.c[i];
            mani_state_.g_total.segment(FR3_DOF * i, FR3_DOF) = mani_state_.g[i];
        }
    }


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

void FR3HuskyModelUpdater::setInitFromCurrent()
{
    if (!is_configured_)
    {
        return;
    }

    mani_state_.q_init = mani_state_.q;
    mani_state_.qdot_init = mani_state_.qdot;
    mani_state_.qddot_init = mani_state_.qddot;
    mani_state_.q_total_init = mani_state_.q_total;
    mani_state_.qdot_total_init = mani_state_.qdot_total;
    mani_state_.qddot_total_init = mani_state_.qddot_total;

    mobi_state_.base_pose_w_init = mobi_state_.base_pose_w;
    mobi_state_.base_vel_w_init = mobi_state_.base_vel_w;
    mobi_state_.base_vel_b_init = mobi_state_.base_vel_b;

    for (auto& [ee_name, ee_data] : ee_data_)
    {
        ee_data.setInit();
    }
}

void FR3HuskyModelUpdater:: writeCommand(const Eigen::VectorXd& command_mani, const Eigen::Vector2d& command_mobi)
{
    if (!is_configured_ || !getHandlesReady())
    {
        return;
    }

    if (static_cast<size_t>(command_mani.size()) == manipulator_dof_)
    {
        halt_initialized_ = false;  // leave halt mode; next halt will re-capture pose

        // for FR3
        for (size_t i = 0; i < robot_handle_.mani_joints.size(); ++i)
        {
            const auto& h = robot_handle_.mani_joints[i];
            const int joint_idx = jointNameToIndex(h.command.get().get_name());
            Eigen::Index cmd_idx = static_cast<Eigen::Index>(i);
            if (joint_idx >= 0)
            {
                if (num_robots_ > 1)
                {
                    // Keep arm block offset in dual-arm mode (left/right jointN must not alias).
                    const Eigen::Index arm_block = static_cast<Eigen::Index>(i / FR3_DOF);
                    cmd_idx = arm_block * static_cast<Eigen::Index>(FR3_DOF) + static_cast<Eigen::Index>(joint_idx);
                }
                else
                {
                    cmd_idx = static_cast<Eigen::Index>(joint_idx);
                }
            }
            if (cmd_idx < 0 || cmd_idx >= command_mani.size())
            {
                if (node_) RCLCPP_ERROR(node_->get_logger(),
                                        "Computed invalid command index (%ld) for joint '%s' with command size %zu. Halting.",
                                        static_cast<long>(cmd_idx),
                                        h.command.get().get_name().c_str(),
                                        static_cast<size_t>(command_mani.size()));
                haltCommands();
                return;
            }
            robot_handle_.mani_joints[i].command.get().set_value(command_mani(cmd_idx));
        }
        
        // for Husky
        for (size_t i = 0; i < robot_handle_.left_wheels.size(); ++i) robot_handle_.left_wheels[i].command.get().set_value(command_mobi(0));
        for (size_t i = 0; i < robot_handle_.right_wheels.size(); ++i) robot_handle_.right_wheels[i].command.get().set_value(command_mobi(1));

    }
    else
    {
        if (node_) RCLCPP_WARN(node_->get_logger(),
                               "Manipulator cmd size mismatch (expected %zu, got %zu). Holding/zeroing.",
                               manipulator_dof_, static_cast<size_t>(command_mani.size()));
        haltCommands();
    }

}

void FR3HuskyModelUpdater::forceStopMobile()
{
    if (!is_configured_ || !getHandlesReady())
    {
        return;
    }

    mobi_state_.wheel_vel_desired.setZero();
    for (auto& h : robot_handle_.left_wheels)  h.command.get().set_value(0.0);
    for (auto& h : robot_handle_.right_wheels) h.command.get().set_value(0.0);
}

void FR3HuskyModelUpdater::haltCommands()
{
    if (!getHandlesReady())
    {
        return;
    }

    if (!halt_initialized_)
    {
        halt_position_.clear();
        for (size_t i = 0; i < manipulator_dof_; ++i)
        {
            const std::string jname = robot_handle_.mani_joints[i].command.get().get_name();
            if(jname.find(arm_id_) != std::string::npos && jname.find("joint") != std::string::npos)
            {
                halt_position_[jname] = robot_handle_.mani_joints[i].state[kPositionIndex].get().get_value();
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(), "Joint named [%s] does not enclude [%s_joint]. Halt command for [%s] is omitted!",
                            jname.c_str(), arm_id_.c_str(), jname.c_str());
            }
        }
        halt_initialized_ = true;
    }

    if (has_position_command_interface_)
    {
        for (size_t i = 0; i < robot_handle_.mani_joints.size(); ++i)
        {
            auto it = halt_position_.find(robot_handle_.mani_joints[i].command.get().get_name());
            if (it != halt_position_.end())
            {
                robot_handle_.mani_joints[i].command.get().set_value(it->second);
            }
        }
    }
    else if (has_velocity_command_interface_)
    {
        for (auto& h : robot_handle_.mani_joints) h.command.get().set_value(0.0);
    }
    else
    {
        const Eigen::Vector<double, FR3_DOF> kp{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0};
        const Eigen::Vector<double, FR3_DOF> kv{30.0,   30.0,  30.0,  30.0,  10.0,  10.0,  5.0};

        for (size_t i = 0; i < robot_handle_.mani_joints.size(); ++i)
        {
            auto it = halt_position_.find(robot_handle_.mani_joints[i].command.get().get_name());
            if (it != halt_position_.end())
            {
                const std::string jname = it->first;
                const double q_halted = it->second;
                
                int arm_idx = -1;
                for (int joint_idx = 1; joint_idx <= FR3_DOF; ++joint_idx)
                {
                    if (jname.find(std::to_string(joint_idx)) != std::string::npos)
                    {
                        arm_idx = joint_idx - 1;  // 0-based index
                    }
                }
                if(arm_idx < 0)
                {
                    RCLCPP_WARN(node_->get_logger(), "Joint named [%s] exceed %s joint index [0 to %zu]. Halt command for [%s] set as 0!",
                                jname.c_str(), arm_id_.c_str(), static_cast<size_t>(FR3_DOF - 1), jname.c_str());
                    robot_handle_.mani_joints[i].command.get().set_value(0.0);
                    continue;
                }

                const double q_curr = robot_handle_.mani_joints[i].state[kPositionIndex].get().get_value();
                const double qdot_curr = (has_velocity_state_interface_) ? robot_handle_.mani_joints[i].state[kVelocityIndex].get().get_value() : 0.0;

                robot_handle_.mani_joints[i].command.get().set_value(kp[arm_idx] * (q_halted - q_curr) - kv[arm_idx] * (qdot_curr));
            }
        }
    }

    for (auto & h : robot_handle_.left_wheels)  h.command.get().set_value(0.0);
    for (auto & h : robot_handle_.right_wheels) h.command.get().set_value(0.0);
}

}  // namespace fr3_husky_controller
