// Copyright 2026 Ekumen Inc.
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

#ifndef RCLCPP__DETAIL__REGION_FALLBACK_HPP_
#define RCLCPP__DETAIL__REGION_FALLBACK_HPP_

#include <stdexcept>
#include <string>

#include "rcl/node.h"
#include "rcutils/env.h"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"
#include "rosidl_runtime_cpp/views/traits.hpp"

#include "rclcpp/exceptions.hpp"

namespace rclcpp::detail
{

inline bool region_loan_available(rmw_region_loan_role_t role)
{
  bool disabled = false;
  auto ret = rcl_get_disable_loaned_message(&disabled);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to read loan setting");
  }
  if (role == RMW_REGION_LOAN_SUBSCRIPTION && !disabled) {
    const char * value = nullptr;
    const char * error = rcutils_get_env("ROS_DISABLE_LOANED_MESSAGES", &value);
    if (error) {
      throw std::runtime_error(std::string("failed to read loan setting: ") + error);
    }
    disabled = value && value[0] != '\0' && std::string(value) != "0";
  }
  bool available = false;
  auto rmw_ret = rmw_region_loan_entry_points_available(role, &available);
  if (rmw_ret != RMW_RET_OK) {
    const std::string message = rmw_get_error_string().str;
    rmw_reset_error();
    throw std::runtime_error("failed to query region loan support: " + message);
  }
  return available && !disabled;
}

inline bool region_loan_type_available(
  rmw_region_loan_role_t role, const rosidl_message_type_support_t * first_type,
  const rosidl_runtime_c__message_context_t * first_context,
  const rosidl_message_type_support_t * second_type = nullptr,
  const rosidl_runtime_c__message_context_t * second_context = nullptr)
{
  bool available = false;
  auto ret = rmw_region_loan_type_support_available(
    role, first_type, first_context, second_type, second_context, &available);
  if (ret != RMW_RET_OK) {
    const std::string message = rmw_get_error_string().str;
    rmw_reset_error();
    throw std::runtime_error("failed to query region loan type support: " + message);
  }
  return available;
}

template<typename ViewT>
rosidl_runtime_cpp::views::MessageConstraints<ViewT>
copy_region_constraints(const rosidl_runtime_c__message_context_t * context)
{
  using Constraints = rosidl_runtime_cpp::views::MessageConstraints<ViewT>;
  Constraints result;
  if (!context) {
    return result;
  }
  const auto & source = context->constraints;
  if (source.max_string_length || source.max_total_size) {
    throw std::invalid_argument("Copy fallback does not support blanket constraints");
  }
  if (source.strict && !source.type_specific) {
    throw std::invalid_argument("Strict constraints require type-specific constraints");
  }
  if (source.type_specific) {
    result.type_specific = *static_cast<const typename ViewT::Constraints *>(source.type_specific);
  }
  result.strict = source.strict;
  if (!result.type_specific.CheckCompatible(typename ViewT::Constraints{})) {
    throw std::invalid_argument("Invalid region constraints");
  }
  return result;
}

template<typename ViewT>
rosidl_runtime_cpp::views::MessageConstraints<ViewT>
refine_region_constraints(
  const rosidl_runtime_cpp::views::MessageConstraints<ViewT> & baseline,
  const rosidl_runtime_c__message_context_t * context, bool baseline_has_type_specific)
{
  if (!context) {
    return baseline;
  }
  auto merged = *context;
  if (!merged.constraints.type_specific && baseline_has_type_specific) {
    merged.constraints.type_specific = &baseline.type_specific;
  }
  auto result = copy_region_constraints<ViewT>(&merged);
  if (!merged.constraints.type_specific) {
    result.type_specific = baseline.type_specific;
  }
  result.strict = result.strict || baseline.strict;
  if (!result.type_specific.CheckCompatible(baseline.type_specific)) {
    throw std::invalid_argument("Loan constraints are looser than endpoint baseline");
  }
  return result;
}

}  // namespace rclcpp::detail

#endif  // RCLCPP__DETAIL__REGION_FALLBACK_HPP_
