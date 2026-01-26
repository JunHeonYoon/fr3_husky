#include "fr3_husky_controller/test_husky_controller.hpp"


namespace fr3_husky_controller
{
namespace
{
constexpr size_t kFeedbackPositionIndex = 0;
constexpr size_t kFeedbackVelocityIndex = 1;
using lifecycle_msgs::msg::State;
}

using namespace std::chrono_literals;

controller_interface::InterfaceConfiguration TestHuskyController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint_name : params_.left_wheel_names)
    {
        conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
        conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
    }
    for (const auto & joint_name : params_.right_wheel_names)
    {
        conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
        conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
    }

    return conf;
}

controller_interface::InterfaceConfiguration TestHuskyController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint_name : params_.left_wheel_names)
    {
        conf.names.push_back(joint_name + "/" +  hardware_interface::HW_IF_VELOCITY);
    }
    for (const auto & joint_name : params_.right_wheel_names)
    {
        conf.names.push_back(joint_name + "/" +  hardware_interface::HW_IF_VELOCITY);
    }
    return conf;
}

CallbackReturn TestHuskyController::on_init() 
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

CallbackReturn TestHuskyController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    // update parameters if they have changed
    if (param_listener_->is_old(params_))
    {
        params_ = param_listener_->get_params();
        LOGI(get_node(), "Parameters were updated");
    }

    if (params_.left_wheel_names.size() != params_.right_wheel_names.size())
    {
        LOGE(get_node(), "The number of left wheels [%zu] and the number of right wheels [%zu] are different",
            params_.left_wheel_names.size(), params_.right_wheel_names.size());
        return CallbackReturn::ERROR;
    }

    if (params_.left_wheel_names.empty())
    {
        LOGE(get_node(), "Wheel names parameters are empty!");
        return CallbackReturn::ERROR;
    }

    if (get_update_rate() == 0)
    {
        throw std::runtime_error("Controller's update rate is set to 0. This should not happen!");
    }
    dt_ = 1.0 / static_cast<double>(get_update_rate());

    // initialize dyros_robot_data & controller
    drc::Mobile::KinematicParam p;
    p.type          = drc::Mobile::DriveType::Differential;
    p.wheel_radius  = params_.wheel_radius_multiplier * params_.wheel_radius;
    p.base_width    = params_.wheel_separation_multiplier * params_.wheel_separation;
    p.max_lin_speed = params_.linear.x.max_velocity;
    p.max_ang_speed = params_.angular.z.max_velocity;
    p.max_lin_acc   = params_.linear.x.max_acceleration;
    p.max_ang_acc   = params_.angular.z.max_acceleration;

    robot_data_ = std::make_shared<drc::Mobile::RobotData>(dt_, p);
    robot_controller_ = std::make_unique<drc::Mobile::RobotController>(robot_data_);

    cmd_vel_timeout_ = std::chrono::milliseconds{static_cast<int>(params_.cmd_vel_timeout * 1000.0)};

    if (!reset()) return CallbackReturn::ERROR;


    const Twist empty_twist;
    received_velocity_msg_ptr_.set(std::make_shared<Twist>(empty_twist));

    // initialize command subscriber
    if (params_.use_stamped_vel)
    {
        velocity_command_subscriber_ = get_node()->create_subscription<Twist>(
            "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
            [this](const std::shared_ptr<Twist> msg) -> void
            {
                if (!subscriber_is_active_)
                {
                    LOGW(get_node(), "Can't accept new commands. subscriber is inactive");
                    return;
                }
                if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
                {
                    LOGWO(get_node(),
                        "Received TwistStamped with zero timestamp, setting it to current "
                        "time, this message will only be shown once");
                    msg->header.stamp = get_node()->get_clock()->now();
                }
                received_velocity_msg_ptr_.set(std::move(msg));
            }
        );
    }
    else
    {
        velocity_command_unstamped_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
            "~/cmd_vel_unstamped", rclcpp::SystemDefaultsQoS(),
            [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void
            {
                if (!subscriber_is_active_)
                {
                    LOGW(get_node(), "Can't accept new commands. subscriber is inactive");
                    return;
                }

                // Write fake header in the stored stamped command
                std::shared_ptr<Twist> twist_stamped;
                received_velocity_msg_ptr_.get(twist_stamped);
                twist_stamped->twist = *msg;
                twist_stamped->header.stamp = get_node()->get_clock()->now();
            }
        );
    }

    // initialize odometry publisher and message
    odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>("~/odom", rclcpp::SystemDefaultsQoS());
    realtime_odometry_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odometry_publisher_);

    auto & odometry_message = realtime_odometry_publisher_->msg_;
    odometry_message.header.frame_id = params_.odom_frame_id;
    odometry_message.child_frame_id = params_.base_frame_id;

    // limit the publication on the topics /odom and /tf
    publish_rate_ = params_.publish_rate;
    publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);

    // initialize odom values zeros
    odometry_message.twist = geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

    constexpr size_t NUM_DIMENSIONS = 6;
    for (size_t index = 0; index < 6; ++index)
    {
        // 0, 7, 14, 21, 28, 35
        const size_t diagonal_index = NUM_DIMENSIONS * index + index;
        odometry_message.pose.covariance[diagonal_index] = params_.pose_covariance_diagonal[index];
        odometry_message.twist.covariance[diagonal_index] = params_.twist_covariance_diagonal[index];
    }

    // initialize transform publisher and message
    odometry_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::SystemDefaultsQoS());
    realtime_odometry_transform_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(odometry_transform_publisher_);

    // keeping track of odom and base_link transforms only
    auto & odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
    odometry_transform_message.transforms.resize(1);
    odometry_transform_message.transforms.front().header.frame_id = params_.odom_frame_id;
    odometry_transform_message.transforms.front().child_frame_id = params_.base_frame_id;

    return CallbackReturn::SUCCESS;
}

CallbackReturn TestHuskyController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) 
{
    auto configure_side = [this](const std::string & side, const std::vector<std::string> & wheel_names, std::vector<WheelHandle> & registered_handles)
    {
        if (wheel_names.empty())
        {
            LOGE(get_node(), "No '%s' wheel names specified", side.c_str());
            return false;
        }

        // register handles
        registered_handles.reserve(wheel_names.size());
        for (const auto & wheel_name : wheel_names)
        {
            const auto position_handle = std::find_if(
                state_interfaces_.cbegin(), state_interfaces_.cend(), 
                [&wheel_name](const auto & interface)
                {
                    return interface.get_prefix_name() == wheel_name &&
                           interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
                }
            );

            if (position_handle == state_interfaces_.cend())
            {
                LOGE(get_node(), "Unable to obtain joint position state handle for %s", wheel_name.c_str());
                return false;
            }

            const auto velocity_handle = std::find_if(
                state_interfaces_.cbegin(), state_interfaces_.cend(),
                [&wheel_name](const auto & interface)
                {
                    return interface.get_prefix_name() == wheel_name &&
                           interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
                }
            );

            if (velocity_handle == state_interfaces_.cend())
            {
                LOGE(get_node(), "Unable to obtain joint velocity state handle for %s", wheel_name.c_str());
                return false;
            }

            const auto command_handle = std::find_if(
                command_interfaces_.begin(), command_interfaces_.end(),
                [&wheel_name](const auto & interface)
                {
                    return interface.get_prefix_name() == wheel_name &&
                           interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
                }
            );

            if (command_handle == command_interfaces_.end())
            {
                LOGE(get_node(), "Unable to obtain joint command handle for %s", wheel_name.c_str());
                return false;
            }

            std::vector<std::reference_wrapper<const hardware_interface::LoanedStateInterface>> feedback_handles;
            feedback_handles.reserve(2);
            feedback_handles.emplace_back(std::ref(*position_handle));
            feedback_handles.emplace_back(std::ref(*velocity_handle));

            registered_handles.emplace_back(WheelHandle{std::move(feedback_handles), std::ref(*command_handle)});
        }

        return true;
    };

    const auto left_result = configure_side("left", params_.left_wheel_names, registered_left_wheel_handles_);
    const auto right_result = configure_side("right", params_.right_wheel_names, registered_right_wheel_handles_);

    if (!left_result || !right_result)
    {
        return CallbackReturn::ERROR;
    }

    if (registered_left_wheel_handles_.empty() || registered_right_wheel_handles_.empty())
    {
        LOGE(get_node(), "Either left wheel interfaces, right wheel interfaces are non existent");
        return CallbackReturn::ERROR;
    }

    is_halted = false;
    subscriber_is_active_ = true;

    LOGI(get_node(), "Subscriber and publisher are now active.");

    if (!updateJointStates())
    {
        return CallbackReturn::ERROR;
    }
    updateRobotData();
    setInitfromCurrent();

    return CallbackReturn::SUCCESS;
}

CallbackReturn TestHuskyController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) 
{
    subscriber_is_active_ = false;
    if (!is_halted)
    {
        for (const auto & wheel_handle : registered_left_wheel_handles_)  wheel_handle.input.get().set_value(0.0);
        for (const auto & wheel_handle : registered_right_wheel_handles_) wheel_handle.input.get().set_value(0.0);
        is_halted = true;
    }
    registered_left_wheel_handles_.clear();
    registered_right_wheel_handles_.clear();
    return CallbackReturn::SUCCESS;
}

CallbackReturn TestHuskyController::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) 
{
    if (!reset()) return CallbackReturn::ERROR;

    received_velocity_msg_ptr_.set(std::make_shared<Twist>());
    return CallbackReturn::SUCCESS;
}

CallbackReturn TestHuskyController::on_error(const rclcpp_lifecycle::State & /*previous_state*/) 
{
    if (!reset()) return CallbackReturn::ERROR;
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type TestHuskyController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
        if (!is_halted)
        {
            for (const auto & wheel_handle : registered_left_wheel_handles_)  wheel_handle.input.get().set_value(0.0);
            for (const auto & wheel_handle : registered_right_wheel_handles_) wheel_handle.input.get().set_value(0.0);
            is_halted = true;
        }
        return controller_interface::return_type::OK;
    }

    updateJointStates();
    updateRobotData();

    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, Eigen::Rotation2Dd(base_pose_w_.linear()).angle());

    bool should_publish = false;
    try
    {
        if (previous_publish_timestamp_ + publish_period_ < time)
        {
        previous_publish_timestamp_ += publish_period_;
        should_publish = true;
        }
    }
    catch (const std::runtime_error &)
    {
        // Handle exceptions when the time source changes and initialize publish timestamp
        previous_publish_timestamp_ = time;
        should_publish = true;
    }

    if (should_publish)
    {
        if (realtime_odometry_publisher_->trylock())
        {
            auto & odometry_message = realtime_odometry_publisher_->msg_;
            odometry_message.header.stamp = time;
            odometry_message.pose.pose.position.x = base_pose_w_.translation()(0);
            odometry_message.pose.pose.position.y = base_pose_w_.translation()(1);
            odometry_message.pose.pose.orientation.x = orientation.x();
            odometry_message.pose.pose.orientation.y = orientation.y();
            odometry_message.pose.pose.orientation.z = orientation.z();
            odometry_message.pose.pose.orientation.w = orientation.w();
            odometry_message.twist.twist.linear.x = base_vel_b_(0);
            odometry_message.twist.twist.angular.z = base_vel_b_(2);
            realtime_odometry_publisher_->unlockAndPublish();
        }

        if (params_.enable_odom_tf && realtime_odometry_transform_publisher_->trylock())
        {
            auto & transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
            transform.header.stamp = time;
            transform.transform.translation.x = base_pose_w_.translation()(0);
            transform.transform.translation.y = base_pose_w_.translation()(1);
            transform.transform.rotation.x = orientation.x();
            transform.transform.rotation.y = orientation.y();
            transform.transform.rotation.z = orientation.z();
            transform.transform.rotation.w = orientation.w();
            realtime_odometry_transform_publisher_->unlockAndPublish();
        }
    }

    std::shared_ptr<Twist> command_msg;
    received_velocity_msg_ptr_.get(command_msg);

    if (command_msg == nullptr)
    {
        LOGW(get_node(), "Velocity message received was a nullptr.");
        return controller_interface::return_type::ERROR;
    }

    const auto age_of_last_command = time - command_msg->header.stamp;
    // Brake if cmd_vel has timeout, override the stored command
    if (age_of_last_command > cmd_vel_timeout_)
    {
        command_msg->twist.linear.x = 0.0;
        command_msg->twist.angular.z = 0.0;
    }

    // command may be limited further by SpeedLimit,
    // without affecting the stored twist command
    Twist command = *command_msg;
    double & linear_command = command.twist.linear.x;
    double & angular_command = command.twist.angular.z;

    base_vel_desired_b_ << linear_command, 0, angular_command;
    wheel_vel_desired_ = robot_controller_->VelocityCommand(base_vel_desired_b_);
    writeCommandInterfaces(wheel_vel_desired_);

    return controller_interface::return_type::OK;
}

void TestHuskyController::writeCommandInterfaces(const Eigen::VectorXd& command_velocity)
{
    const Eigen::VectorXd* cmd_vel = &command_velocity;
    Eigen::VectorXd zero_vel;
    if (command_velocity.size() != 2)
    {
        LOGW(get_node(), "Command vector sizes must match 2 (vel %zu).", command_velocity.size());
        zero_vel.setZero(2);
        cmd_vel = &zero_vel;
    }

    // Set wheels velocities:
    for (size_t index = 0; index < WHEEL_PER_SIDE; ++index) // front, rear
    {
        registered_left_wheel_handles_[index].input.get().set_value((*cmd_vel)(0));
        registered_right_wheel_handles_[index].input.get().set_value((*cmd_vel)(1));
    }
}

bool TestHuskyController::updateJointStates()
{
    double left_position_mean = 0.0;
    double right_position_mean = 0.0;
    double left_velocity_mean = 0.0;
    double right_velocity_mean = 0.0;
    for (size_t index = 0; index < WHEEL_PER_SIDE; ++index) // front, rear
    {
        const auto & left_feedback = registered_left_wheel_handles_[index].feedback;
        const auto & right_feedback = registered_right_wheel_handles_[index].feedback;
        const double left_position = left_feedback[kFeedbackPositionIndex].get().get_value();
        const double right_position = right_feedback[kFeedbackPositionIndex].get().get_value();
        const double left_velocity = left_feedback[kFeedbackVelocityIndex].get().get_value();
        const double right_velocity = right_feedback[kFeedbackVelocityIndex].get().get_value();

        if (std::isnan(left_position) || std::isnan(right_position) ||
            std::isnan(left_velocity) || std::isnan(right_velocity))
        {
            LOGE(get_node(), "Either the left or right wheel is invalid for index [%zu]", index);
            return false;
        }

        left_position_mean += left_position;
        right_position_mean += right_position;
        left_velocity_mean += left_velocity;
        right_velocity_mean += right_velocity;
    }

    left_position_mean /= WHEEL_PER_SIDE;
    right_position_mean /= WHEEL_PER_SIDE;
    left_velocity_mean /= WHEEL_PER_SIDE;
    right_velocity_mean /= WHEEL_PER_SIDE;

    wheel_pos_ = Eigen::Vector2d(left_position_mean, right_position_mean);
    wheel_vel_ = Eigen::Vector2d(left_velocity_mean, right_velocity_mean);

    // LOGI(get_node(), "========================================");
    // LOGI(get_node(), "wheel_pos: %f, %f", wheel_pos_(0), wheel_pos_(1));
    // LOGI(get_node(), "wheel_vel: %f, %f", wheel_vel_(0), wheel_vel_(1));
    // LOGI(get_node(), "========================================");

    return true;
}

void TestHuskyController::updateRobotData()
{
    robot_data_->updateState(wheel_pos_, wheel_vel_);
    play_time_ = get_node()->now().seconds();

    base_vel_b_ = robot_data_->getBaseVel();
    base_pose_w_ = robot_data_->getBasePose();
}

void TestHuskyController::setInitfromCurrent()
{
    base_pose_init_w_ = base_pose_w_;
    // base_pose_init_b_ = base_pose_b_;
    base_vel_init_w_ = base_vel_w_;
    base_vel_init_b_ = base_vel_b_;

    control_start_time_ = get_node()->now().seconds();
}


bool TestHuskyController::reset()
{
    if(robot_data_)
    {
        robot_data_->initBasePose();
    }

    registered_left_wheel_handles_.clear();
    registered_right_wheel_handles_.clear();

    subscriber_is_active_ = false;
    velocity_command_subscriber_.reset();
    velocity_command_unstamped_subscriber_.reset();

    received_velocity_msg_ptr_.set(nullptr);
    is_halted = false;
    return true;
}

}  // namespace fr3_husky_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(fr3_husky_controller::TestHuskyController, 
                            controller_interface::ControllerInterface)
