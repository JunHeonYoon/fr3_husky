#!/usr/bin/env python3
"""
generate_action_server.py

It generates:
- HPP: include/fr3_husky_controller/servers/<snake>_action_server.hpp
- CPP: src/servers/<snake>_action_server.cpp

It also patches CMakeLists.txt to add the new .cpp source (deduped).

Usage:
  python3 generate_action_server.py HoldPosition
  python3 generate_action_server.py fr3_hold_position --action-name fr3_hold_position
  python3 generate_action_server.py "My New Server" --force
"""

import argparse
import re
from pathlib import Path
from typing import Tuple


def script_pkg_root() -> Path:
    # This file lives in <pkg_root>/generate_action_server.py
    return Path(__file__).resolve().parent


def to_snake(name: str) -> str:
    s = name.strip()
    s = re.sub(r"[\-\s]+", "_", s)
    s = re.sub(r"[^0-9a-zA-Z_]", "_", s)
    s = re.sub(r"(.)([A-Z][a-z]+)", r"\1_\2", s)
    s = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", s)
    s = s.lower()
    s = re.sub(r"_+", "_", s).strip("_")
    if not s:
        raise ValueError("Name becomes empty after normalization.")
    if s[0].isdigit():
        s = f"server_{s}"
    return s


def to_camel(snake: str) -> str:
    parts = [p for p in snake.split("_") if p]
    out = "".join(p[:1].upper() + p[1:] for p in parts)
    if not out:
        raise ValueError("Cannot create CamelCase class name.")
    if out[0].isdigit():
        out = f"Server{out}"
    return out


def ensure_dir(p: Path) -> None:
    p.mkdir(parents=True, exist_ok=True)


def render_hpp(class_name: str, action_type_include: str, action_type: str) -> str:
    # Namespace and include paths are fixed to fr3_husky_controller per your layout.
    return f"""#pragma once

#include <mutex>
#include <memory>

#include <rclcpp_action/rclcpp_action.hpp>
#include <{action_type_include}>

#include <fr3_husky_controller/servers/action_server_base.hpp>

namespace fr3_husky_controller::servers
{{

class {class_name} final : public ActionServerBase
{{
public:
    using ActionT = {action_type};
    using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;

    {class_name}(const std::string& name, const NodePtr& node, FR3ModelUpdater& model_updater);
    ~{class_name}() override = default;

    // Controller(owner) queries
    bool isActive() const override;
    int priority() const override;

    // Called ONLY when this server is owner in controller update-loop
    bool compute(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    void onActivated() override;
    void onDeactivated() override;

private:
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const ActionT::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);

private:
    std::shared_ptr<rclcpp_action::Server<ActionT>> server_;

    mutable std::mutex mutex_;
    std::shared_ptr<GoalHandle> active_goal_;

    // ======================= USER TODO FRAMEWORK =======================
    // [SERVER CALLBACK THREAD] handle_goal / handle_cancel / handle_accepted
    //   - Validate incoming goal
    //   - Cache only small immutable goal parameters for controller thread
    //   - requestActivate()/requestCancel() only (no heavy robot computation)
    //
    // [CONTROLLER UPDATE THREAD] onActivated / compute / onDeactivated
    //   - Own robot command authority while selected as active owner
    //   - Read state from model_updater_ and write command in compute()
    //   - Finalize active goal in onDeactivated()
    // ==================================================================

    // TODO[SERVER->CONTROLLER BRIDGE]: Cache goal data needed by compute()
    // e.g., ActionT::Goal::some_field goal_param_;
}};

}}  // namespace fr3_husky_controller::servers
"""


def render_cpp(class_name: str, hpp_include: str, action_name: str) -> str:
    return f"""#include <{hpp_include}>

namespace fr3_husky_controller::servers
{{

{class_name}::{class_name}(const std::string& name, const NodePtr& node, FR3ModelUpdater& model_updater)
: ActionServerBase(name, node, model_updater)
{{
    using std::placeholders::_1;
    using std::placeholders::_2;

    server_ = rclcpp_action::create_server<ActionT>(
        node_,
        name_,  // action name
        std::bind(&{class_name}::handle_goal, this, _1, _2),
        std::bind(&{class_name}::handle_cancel, this, _1),
        std::bind(&{class_name}::handle_accepted, this, _1));

    RCLCPP_INFO(node_->get_logger(), "[%s] {class_name} created", name_.c_str());
}}

bool {class_name}::isActive() const
{{
    std::lock_guard<std::mutex> lk(mutex_);
    return static_cast<bool>(active_goal_);
}}

int {class_name}::priority() const
{{
    // TODO: Set priority if multiple servers request activation simultaneously.
    // Higher wins. Default 0.
    return 0;
}}

bool {class_name}::compute(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{{
    std::shared_ptr<GoalHandle> gh;
    {{
        std::lock_guard<std::mutex> lk(mutex_);
        gh = active_goal_;
    }}
    if (!gh) return true;

    // TODO[CONTROLLER THREAD]: Implement control logic here.
    // - Read current state from model_updater_
    // - Write commands through model_updater_
    //
    // Example placeholder:
    // const auto& st = model_updater_.getState();
    // model_updater_.writePositionCommand(st.q_total);

    // TODO[CONTROLLER THREAD]: Publish feedback/result as needed:
    // auto feedback = std::make_shared<ActionT::Feedback>();
    // gh->publish_feedback(feedback);

    return true;
}}

void {class_name}::onActivated()
{{
    // TODO[CONTROLLER THREAD]: optional one-time init when owner is granted.
    RCLCPP_INFO(node_->get_logger(), "[%s] activated(owner)", name_.c_str());
}}

void {class_name}::onDeactivated()
{{
    std::lock_guard<std::mutex> lk(mutex_);
    if (active_goal_)
    {{
        // TODO[CONTROLLER THREAD]: Decide how to end the goal when owner is released.
        // - abort(...) if you are stopping prematurely
        // - succeed(...) if completed
        auto result = std::make_shared<ActionT::Result>();
        active_goal_->abort(result);
        active_goal_.reset();
    }}
    RCLCPP_INFO(node_->get_logger(), "[%s] deactivated(owner released)", name_.c_str());
}}

rclcpp_action::GoalResponse {class_name}::handle_goal(const rclcpp_action::GoalUUID& /*uuid*/, std::shared_ptr<const ActionT::Goal> /*goal*/)
{{
    // TODO[SERVER CALLBACK THREAD]: Validate goal and decide ACCEPT/REJECT.
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}}

rclcpp_action::CancelResponse {class_name}::handle_cancel(const std::shared_ptr<GoalHandle> /*goal_handle*/)
{{
    // SERVER CALLBACK THREAD -> CONTROLLER THREAD signal
    requestCancel();
    return rclcpp_action::CancelResponse::ACCEPT;
}}

void {class_name}::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{{
    {{
        std::lock_guard<std::mutex> lk(mutex_);
        active_goal_ = goal_handle;

        // TODO[SERVER CALLBACK THREAD]: Cache goal fields needed for compute()
        // from goal_handle->get_goal()
        // auto goal = goal_handle->get_goal();
    }}
    // SERVER CALLBACK THREAD -> CONTROLLER THREAD signal
    requestActivate();
}}

}}  // namespace fr3_husky_controller::servers

// Register this server into global registry (executed when this TU is linked)
REGISTER_FR3_ACTION_SERVER(fr3_husky_controller::servers::{class_name}, "{action_name}")
"""


def patch_cmakelists(cmake_path: Path, new_cpp_rel: str) -> Tuple[bool, str]:
    """
    Insert new_cpp_rel into an add_library/add_executable block.

    Preference:
    - a block that already contains "src/servers/"
    - else first add_library/add_executable block

    Deduped: no-op if already present.
    """
    text = cmake_path.read_text(encoding="utf-8")
    if new_cpp_rel in text:
        return False, f"Already present: {new_cpp_rel}"

    lines = text.splitlines(True)

    def find_blocks():
        blocks = []
        i = 0
        while i < len(lines):
            if re.match(r"^\s*add_(library|executable)\s*\(", lines[i]):
                start = i
                depth = lines[i].count("(") - lines[i].count(")")
                i += 1
                while i < len(lines) and depth > 0:
                    depth += lines[i].count("(") - lines[i].count(")")
                    i += 1
                end = i
                blocks.append((start, end))
            else:
                i += 1
        return blocks

    blocks = find_blocks()
    chosen = None
    for s, e in blocks:
        if "src/servers/" in "".join(lines[s:e]):
            chosen = (s, e)
            break
    if chosen is None and blocks:
        chosen = blocks[0]

    if chosen is None:
        hint = (
            "\n# ---- Added by generate_action_server.py ----\n"
            f"# Please add this source to your target:\n"
            f"#   {new_cpp_rel}\n"
            "# ------------------------------------------\n"
        )
        cmake_path.write_text(text + hint, encoding="utf-8")
        return True, "No add_library/add_executable block found; appended hint."

    s, e = chosen

    # find insertion point: before last line containing ')'
    insert_at = None
    for j in range(e - 1, s - 1, -1):
        if ")" in lines[j]:
            insert_at = j
            break
    if insert_at is None:
        insert_at = e

    # infer indent
    indent = "  "
    for j in range(s, e):
        if "src/" in lines[j]:
            m = re.match(r"^(\s+)\S", lines[j])
            if m:
                indent = m.group(1)
                break

    lines.insert(insert_at, f"{indent}{new_cpp_rel}\n")
    cmake_path.write_text("".join(lines), encoding="utf-8")
    return True, f"Inserted: {new_cpp_rel}"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("name", help="Server name (any style). Used to derive snake_case file name and CamelCase class.")
    ap.add_argument("--action-name", default=None, help='Action name string for registration, e.g., "fr3_hold_position".')
    ap.add_argument(
        "--action-include",
        default="control_msgs/action/follow_joint_trajectory.hpp",
        help="Action type include path.",
    )
    ap.add_argument(
        "--action-type",
        default="control_msgs::action::FollowJointTrajectory",
        help="C++ action type name.",
    )
    ap.add_argument(
        "--suffix",
        default="_action_server",
        help='File suffix (default "_action_server"). Final files: <snake><suffix>.hpp/.cpp',
    )
    ap.add_argument("--force", action="store_true", help="Overwrite files if they already exist.")
    args = ap.parse_args()

    pkg_root = script_pkg_root()
    cmake_path = pkg_root / "CMakeLists.txt"
    if not cmake_path.exists():
        raise FileNotFoundError(f"CMakeLists.txt not found next to script: {cmake_path}")

    snake = to_snake(args.name)
    class_name = to_camel(snake)
    action_name = args.action_name or snake  # default action name: snake_case

    hpp_name = f"{snake}{args.suffix}.hpp"
    cpp_name = f"{snake}{args.suffix}.cpp"

    hpp_dir = pkg_root / "include" / "fr3_husky_controller" / "servers"
    cpp_dir = pkg_root / "src" / "servers"

    ensure_dir(hpp_dir)
    ensure_dir(cpp_dir)

    hpp_path = hpp_dir / hpp_name
    cpp_path = cpp_dir / cpp_name

    if (hpp_path.exists() or cpp_path.exists()) and not args.force:
        raise FileExistsError(
            f"Target file exists.\n- {hpp_path}\n- {cpp_path}\nUse --force to overwrite."
        )

    hpp_txt = render_hpp(class_name, args.action_include, args.action_type)
    hpp_include = f"fr3_husky_controller/servers/{hpp_name}"
    cpp_txt = render_cpp(class_name, hpp_include, action_name)

    hpp_path.write_text(hpp_txt, encoding="utf-8")
    cpp_path.write_text(cpp_txt, encoding="utf-8")

    new_cpp_rel = f"src/servers/{cpp_name}"
    changed, msg = patch_cmakelists(cmake_path, new_cpp_rel)

    print("Generated:")
    print(f"  - {hpp_path.relative_to(pkg_root)}")
    print(f"  - {cpp_path.relative_to(pkg_root)}")
    print("CMakeLists.txt:")
    print(f"  - {msg}")
    if not changed:
        print("  - No changes were needed.")

    print("\nNext steps:")
    print("  1) Fill TODOs by thread boundary:")
    print("     - [SERVER CALLBACK THREAD] handle_goal/handle_cancel/handle_accepted")
    print("     - [CONTROLLER THREAD] onActivated/compute/onDeactivated")
    print("  2) Ensure your controller creates servers via ActionServerBase::createAll(...)")
    print("  3) Read docs: fr3_husky_controller/docs/action_server_todo_framework.md")
    print("  4) Build: colcon build --packages-select fr3_husky_controller")


if __name__ == "__main__":
    main()
