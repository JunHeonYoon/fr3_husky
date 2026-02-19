// src/servers/action_server_base.cpp
#include <fr3_husky_controller/servers/action_server_base.hpp>

namespace fr3_husky_controller::servers
{

std::vector<std::shared_ptr<ActionServerBase>> ActionServerBase::createAll(const NodePtr& node, FR3ModelUpdater& model_updater)
{
    std::vector<std::shared_ptr<ActionServerBase>> out;
    out.reserve(registry().size());
    for (auto& [name, fac] : registry())
    {
        out.push_back(fac(name, node, model_updater));
        RCLCPP_INFO(node->get_logger(), "[ActionServerRegistry] Created: %s", name.c_str());
    }
    return out;
}

ActionServerBase::ActionServerBase(std::string name, const NodePtr& node, FR3ModelUpdater& model_updater)
: name_(std::move(name)),
  node_(node),
  model_updater_(model_updater)
{

}

bool ActionServerBase::consumeActivateRequest()
{
    return activate_requested_.exchange(false, std::memory_order_acq_rel);
}

bool ActionServerBase::consumeCancelRequest()
{
    return cancel_requested_.exchange(false, std::memory_order_acq_rel);
}

void ActionServerBase::requestActivate()
{
    activate_requested_.store(true, std::memory_order_release);
}

void ActionServerBase::requestCancel()
{
    cancel_requested_.store(true, std::memory_order_release);
}

}  // namespace fr3_husky_controller::servers
