#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <fr3_husky_controller/utils/model/fr3_model_updater.hpp>

namespace fr3_husky_controller::servers
{

class IdleControl
{
    public:
        using NodePtr = rclcpp_lifecycle::LifecycleNode::SharedPtr;

        IdleControl(const std::string& name, const NodePtr& node, FR3ModelUpdater& fr3_model_updater);
        ~IdleControl() = default;

        // active_server_가 없을 때만 호출되는 fallback
        bool compute(const rclcpp::Time& time, const rclcpp::Duration& period);

        void onActivated();   // idle 모드에 들어올 때(선택)
        void onDeactivated(); // idle 모드에서 나갈 때(선택)

    private:
        std::string name_;
        NodePtr node_;
        FR3ModelUpdater& model_updater_;

        bool was_idle_{false};
};

}  // namespace fr3_husky_controller::servers
