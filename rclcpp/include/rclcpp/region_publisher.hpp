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

#ifndef RCLCPP__REGION_PUBLISHER_HPP_
#define RCLCPP__REGION_PUBLISHER_HPP_

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rcl/publisher.h"
#include "rosidl_runtime_cpp/views/traits.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"

#include "rclcpp/detail/resolve_use_intra_process.hpp"
#include "rclcpp/detail/region_fallback.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/publisher_base.hpp"
#include "rclcpp/publisher_options.hpp"

namespace rclcpp
{

template<typename ViewT>
class RegionPublisher;

/// Owns a publisher loan and returns it unless it is published.
template<typename ViewT>
class LoanedRegionMessage
{
public:
  LoanedRegionMessage(const LoanedRegionMessage &) = delete;
  LoanedRegionMessage & operator=(const LoanedRegionMessage &) = delete;

  LoanedRegionMessage(LoanedRegionMessage && other) noexcept
  : publisher_(std::move(other.publisher_)), owned_(std::move(other.owned_)),
    constraints_(std::move(other.constraints_)), message_(std::exchange(other.message_, nullptr))
  {}

  ~LoanedRegionMessage()
  {
    if (message_ && !owned_) {
      auto ret = rcl_return_loaned_message_from_publisher(publisher_.get(), message_);
      if (ret != RCL_RET_OK) {
        RCLCPP_ERROR(
          rclcpp::get_logger("rclcpp"), "Failed to return region publisher loan: %s",
          rcl_get_error_string().str);
        rcl_reset_error();
      }
    }
  }

  bool is_valid() const noexcept {return message_ != nullptr;}

  ViewT & get() const
  {
    if (!message_) {
      throw std::logic_error("Region publisher loan is no longer valid");
    }
    return *message_;
  }

private:
  friend class RegionPublisher<ViewT>;

  LoanedRegionMessage(std::shared_ptr<rcl_publisher_t> publisher, ViewT * message)
  : publisher_(std::move(publisher)), message_(message)
  {}

  LoanedRegionMessage(
    std::shared_ptr<rcl_publisher_t> publisher, std::unique_ptr<ViewT> message,
    const rosidl_runtime_cpp::views::MessageConstraints<ViewT> & constraints)
  : publisher_(std::move(publisher)), owned_(std::move(message)),
    constraints_(constraints), message_(owned_.get())
  {}

  std::shared_ptr<rcl_publisher_t> publisher_;
  std::unique_ptr<ViewT> owned_;
  rosidl_runtime_cpp::views::MessageConstraints<ViewT> constraints_{};
  ViewT * message_;
};

/// Publisher for generated region views, using ordinary-message copies when loans are unavailable.
template<typename ViewT>
class RegionPublisher : public PublisherBase
{
public:
  using Constraints = rosidl_runtime_cpp::views::MessageConstraints<ViewT>;

  RegionPublisher(
    node_interfaces::NodeBaseInterface * node_base, const std::string & topic,
    const rclcpp::QoS & qos, const rclcpp::PublisherOptions & options,
    const rosidl_runtime_c__message_context_t & context)
  : RegionPublisher(
      node_base, topic, qos, options, context,
      rclcpp::detail::region_loan_available(RMW_REGION_LOAN_PUBLISHER) &&
      rclcpp::detail::region_loan_type_available(
        RMW_REGION_LOAN_PUBLISHER, &type_support(), &context), nullptr)
  {}

private:
  RegionPublisher(
    node_interfaces::NodeBaseInterface * node_base, const std::string & topic,
    const rclcpp::QoS & qos, const rclcpp::PublisherOptions & options,
    const rosidl_runtime_c__message_context_t & context, bool native, std::nullptr_t)
  : PublisherBase(
      node_base, topic, native ? type_support() : plain_type_support(),
      publisher_options(options, qos, native),
      options.event_callbacks, options.use_default_callbacks, native ? &context : nullptr,
      native ? &plain_type_support() : nullptr),
    options_(options), native_(native && can_loan_messages()),
    constraints_(native_ ? Constraints{} :
      rclcpp::detail::copy_region_constraints<ViewT>(&context)),
    has_type_constraints_(context.constraints.type_specific != nullptr)
  {
    if (rclcpp::detail::resolve_use_intra_process(options_, *node_base)) {
      throw std::invalid_argument("Region publishers do not support intra-process communication");
    }
  }

public:
  LoanedRegionMessage<ViewT> borrow_loaned_message()
  {
    return borrow(nullptr);
  }

  LoanedRegionMessage<ViewT> borrow_loaned_message(const Constraints & constraints)
  {
    rosidl_runtime_c__message_context_t context{constraints.to_rosidl_message_type_constraints()};
    return borrow(&context);
  }

  void publish(LoanedRegionMessage<ViewT> && loan)
  {
    if (!loan.is_valid() || loan.publisher_.get() != this->get_publisher_handle().get()) {
      throw std::invalid_argument("Region loan does not belong to this publisher");
    }
    if (loan.owned_) {
      if (!constraints_.type_specific.CheckCompatibleView(*loan.message_) ||
        !loan.constraints_.type_specific.CheckCompatibleView(*loan.message_))
      {
        loan.owned_.reset();
        loan.message_ = nullptr;
        throw std::invalid_argument("Region message exceeds loan constraints");
      }
      auto plain = loan.message_->to_plain();
      auto ret = rcl_publish(loan.publisher_.get(), &plain, nullptr);
      if (ret != RCL_RET_PUBLISHER_INVALID) {
        loan.owned_.reset();
        loan.message_ = nullptr;
      }
      if (ret != RCL_RET_OK) {
        rclcpp::exceptions::throw_from_rcl_error(ret, "failed to publish region copy");
      }
      return;
    }
    auto * message = std::exchange(loan.message_, nullptr);
    auto ret = rcl_publish_loaned_message(loan.publisher_.get(), message, nullptr);
    if (ret == RCL_RET_PUBLISHER_INVALID) {
      loan.message_ = message;
    }
    if (ret != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to publish region loan");
    }
  }

private:
  static rcl_publisher_options_t publisher_options(
    const rclcpp::PublisherOptions & options, const rclcpp::QoS & qos, bool native)
  {
    auto result = options.template to_rcl_publisher_options<ViewT>(qos);
    if (!native) {
      result.disable_loaned_message = true;
    }
    return result;
  }

  static const rosidl_message_type_support_t & plain_type_support()
  {
    auto * support =
      rosidl_typesupport_cpp::get_message_type_support_handle<typename ViewT::PlainMessage>();
    if (!support) {
      throw std::runtime_error("Ordinary message type support is null");
    }
    return *support;
  }

  static const rosidl_message_type_support_t & type_support()
  {
    auto * support = rosidl_typesupport_cpp::get_message_type_support_handle<ViewT>();
    if (!support) {
      throw std::runtime_error("Region type support is null");
    }
    return *support;
  }

  LoanedRegionMessage<ViewT> borrow(const rosidl_runtime_c__message_context_t * context)
  {
    auto publisher = this->get_publisher_handle();
    if (!native_) {
      auto constraints = rclcpp::detail::refine_region_constraints<ViewT>(
        constraints_, context, has_type_constraints_);
      return LoanedRegionMessage<ViewT>(
        std::move(publisher), std::make_unique<ViewT>(), constraints);
    }
    void * message = nullptr;
    auto ret = rcl_borrow_loaned_message_with_context(
      publisher.get(), &type_support(), context, &message);
    if (ret != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to borrow region loan");
    }
    if (!message) {
      throw std::runtime_error("Middleware returned a null region loan");
    }
    return LoanedRegionMessage<ViewT>(std::move(publisher), static_cast<ViewT *>(message));
  }

  const rclcpp::PublisherOptions options_;
  const bool native_;
  const Constraints constraints_;
  const bool has_type_constraints_;
};

}  // namespace rclcpp

#endif  // RCLCPP__REGION_PUBLISHER_HPP_
