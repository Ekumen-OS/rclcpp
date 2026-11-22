// Copyright 2026 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RCLCPP__REGION_SUBSCRIPTION_HPP_
#define RCLCPP__REGION_SUBSCRIPTION_HPP_

#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

#include "rcutils/env.h"
#include "rcpputils/scope_exit.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"

#include "rclcpp/detail/resolve_use_intra_process.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/subscription_options.hpp"

namespace rclcpp
{

/// Subscription whose callback borrows a region view for the callback duration.
template<typename ViewT>
class RegionSubscription : public SubscriptionBase
{
public:
  using Callback = std::function<void(const ViewT &, const rclcpp::MessageInfo &)>;

  RegionSubscription(
    node_interfaces::NodeBaseInterface * node_base, const std::string & topic,
    const rclcpp::QoS & qos, Callback callback, const rclcpp::SubscriptionOptions & options,
    const rosidl_runtime_c__message_context_t & context)
  : SubscriptionBase(
      node_base, type_support(), topic, subscription_options(options, qos),
      options.event_callbacks, options.use_default_callbacks, DeliveredMessageKind::ROS_MESSAGE,
      &context),
    callback_(std::move(callback)),
    options_(options)
  {
    if (rclcpp::detail::resolve_use_intra_process(options_, *node_base)) {
      throw std::invalid_argument(
          "Region subscriptions do not support intra-process communication");
    }
    if (!can_loan_messages()) {
      throw std::runtime_error("Middleware cannot loan region views for this subscription");
    }
  }

  void disable_callbacks() override
  {
    SubscriptionBase::disable_callbacks();
    std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
    callback_enabled_ = false;
    for (const auto & [_, event] : event_handlers_) {
      if (event) {
        event->disable();
      }
    }
  }

  void enable_callbacks() override
  {
    SubscriptionBase::enable_callbacks();
    std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
    callback_enabled_ = true;
    for (const auto & [_, event] : event_handlers_) {
      if (event) {
        event->enable();
      }
    }
  }

  void handle_loaned_message(void * message, const rclcpp::MessageInfo & info) override
  {
    auto handle = get_subscription_handle();
    auto return_loan = rcpputils::make_scope_exit([handle, message]() {
          auto ret = rcl_return_loaned_message_from_subscription(handle.get(), message);
          if (ret != RCL_RET_OK) {
            RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"), "Failed to return region subscription loan: %s",
            rcl_get_error_string().str);
            rcl_reset_error();
          }
      });
    std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
    if (callback_enabled_) {
      callback_(*static_cast<const ViewT *>(message), info);
    }
  }

  std::shared_ptr<void> create_message() override
  {
    throw std::runtime_error("Region subscriptions require middleware loans");
  }
  std::shared_ptr<rclcpp::SerializedMessage> create_serialized_message() override
  {
    throw std::runtime_error("Region subscriptions do not deliver serialized messages");
  }
  void handle_message(std::shared_ptr<void> &, const rclcpp::MessageInfo &) override
  {
    throw std::runtime_error("Region subscriptions require middleware loans");
  }
  void handle_serialized_message(
    const std::shared_ptr<rclcpp::SerializedMessage> &, const rclcpp::MessageInfo &) override
  {
    throw std::runtime_error("Region subscriptions do not deliver serialized messages");
  }
  void return_message(std::shared_ptr<void> &) override
  {
    throw std::runtime_error("Region subscriptions require middleware loans");
  }
  void return_serialized_message(std::shared_ptr<rclcpp::SerializedMessage> &) override
  {
    throw std::runtime_error("Region subscriptions do not deliver serialized messages");
  }
  rclcpp::dynamic_typesupport::DynamicMessageType::SharedPtr
  get_shared_dynamic_message_type() override
  {
    throw rclcpp::exceptions::UnimplementedError("Region subscription dynamic message type");
  }
  rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr get_shared_dynamic_message() override
  {
    throw rclcpp::exceptions::UnimplementedError("Region subscription dynamic message");
  }
  rclcpp::dynamic_typesupport::DynamicSerializationSupport::SharedPtr
  get_shared_dynamic_serialization_support() override
  {
    throw rclcpp::exceptions::UnimplementedError("Region subscription dynamic serialization");
  }
  rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr create_dynamic_message() override
  {
    throw rclcpp::exceptions::UnimplementedError("Region subscription dynamic message");
  }
  void return_dynamic_message(rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr &) override
  {
    throw rclcpp::exceptions::UnimplementedError("Region subscription dynamic message");
  }
  void handle_dynamic_message(
    const rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr &,
    const rclcpp::MessageInfo &) override
  {
    throw rclcpp::exceptions::UnimplementedError("Region subscription dynamic message");
  }

private:
  static rcl_subscription_options_t subscription_options(
    const rclcpp::SubscriptionOptions & options, const rclcpp::QoS & qos)
  {
    auto result = options.to_rcl_subscription_options(qos);
    const char * value = nullptr;
    const char * error = rcutils_get_env("ROS_DISABLE_LOANED_MESSAGES", &value);
    if (error) {
      throw std::runtime_error(std::string("Failed to read loan setting: ") + error);
    }
    if (value && value[0] == '\0') {
      result.disable_loaned_message = false;
    }
    return result;
  }

  static const rosidl_message_type_support_t & type_support()
  {
    auto * support = rosidl_typesupport_cpp::get_message_type_support_handle<ViewT>();
    if (!support) {
      throw std::runtime_error("Region type support is null");
    }
    return *support;
  }

  Callback callback_;
  const rclcpp::SubscriptionOptions options_;
  std::recursive_mutex callback_mutex_;
  bool callback_enabled_{true};
};

}  // namespace rclcpp

#endif  // RCLCPP__REGION_SUBSCRIPTION_HPP_
