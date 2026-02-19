#pragma once

#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <fr3_husky_controller/utils/model/fr3_model_updater.hpp>

namespace fr3_husky_controller::servers
{

class ActionServerBase
{
    public:
        using NodePtr = rclcpp_lifecycle::LifecycleNode::SharedPtr;
        using Factory = std::function<std::shared_ptr<ActionServerBase>(const std::string& name, const NodePtr& node, FR3ModelUpdater& model_updater)>;

        // Registry
        static bool registerServer(const std::string& name, Factory factory) {registry()[name] = std::move(factory); return true;}
        static std::vector<std::shared_ptr<ActionServerBase>>
        createAll(const NodePtr& node, FR3ModelUpdater& model_updater);

        ActionServerBase(std::string name, const NodePtr& node, FR3ModelUpdater& fr3_model_updater);
        virtual ~ActionServerBase() = default;

        // Controller(owner) interface
        virtual bool compute(const rclcpp::Time& time, const rclcpp::Duration& period) = 0;
        virtual bool isActive() const = 0;
        virtual int priority() const { return 0; }

        bool consumeActivateRequest();
        bool consumeCancelRequest();

        virtual void onActivated() {};
        virtual void onDeactivated() {};

        const std::string& getName() const {return name_;}

    protected:
        void requestActivate();
        void requestCancel();

        std::string name_;
        NodePtr node_;
        FR3ModelUpdater& model_updater_;

    private:
        static std::map<std::string, Factory>& registry(){static std::map<std::string, Factory> reg; return reg;}

        std::atomic<bool> activate_requested_{false};
        std::atomic<bool> cancel_requested_{false};
};

#define REGISTER_FR3_ACTION_SERVER(ServerClass, server_name)                            \
    static bool _registered_##ServerClass =                                             \
        ::fr3_husky_controller::servers::ActionServerBase::registerServer(              \
            server_name,                                                                \
            [](const std::string& name,                                                 \
               const ::fr3_husky_controller::servers::ActionServerBase::NodePtr& node,  \
               ::fr3_husky_controller::FR3ModelUpdater& model_updater)                  \
               -> std::shared_ptr<                                                      \
                   ::fr3_husky_controller::servers::ActionServerBase> {                 \
              return std::make_shared<ServerClass>(name, node, model_updater);          \
            });

}  // namespace fr3_husky_controller::servers
