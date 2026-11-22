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
  : publisher_(std::move(other.publisher_)), message_(std::exchange(other.message_, nullptr))
  {}

  ~LoanedRegionMessage()
  {
    if (message_) {
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

  std::shared_ptr<rcl_publisher_t> publisher_;
  ViewT * message_;
};

/// Publisher for generated region views; region loans never fall back to copies.
template<typename ViewT>
class RegionPublisher : public PublisherBase
{
public:
  using Constraints = rosidl_runtime_cpp::views::MessageConstraints<ViewT>;

  RegionPublisher(
    node_interfaces::NodeBaseInterface * node_base, const std::string & topic,
    const rclcpp::QoS & qos, const rclcpp::PublisherOptions & options,
    const rosidl_runtime_c__message_context_t & context)
  : PublisherBase(
      node_base, topic, type_support(),
      options.template to_rcl_publisher_options<ViewT>(qos),
      options.event_callbacks, options.use_default_callbacks, &context),
    options_(options)
  {
    if (rclcpp::detail::resolve_use_intra_process(options_, *node_base)) {
      throw std::invalid_argument("Region publishers do not support intra-process communication");
    }
    if (!can_loan_messages()) {
      throw std::runtime_error("Middleware cannot loan region views for this publisher");
    }
  }

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
};

}  // namespace rclcpp

#endif  // RCLCPP__REGION_PUBLISHER_HPP_
