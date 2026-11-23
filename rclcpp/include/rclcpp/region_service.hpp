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

#ifndef RCLCPP__REGION_SERVICE_HPP_
#define RCLCPP__REGION_SERVICE_HPP_

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

#include "rcpputils/scope_exit.hpp"
#include "rcl/client.h"
#include "rcl/error_handling.h"
#include "rcl/service.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"

#include "rclcpp/detail/region_fallback.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"

namespace rclcpp
{

template<typename ServiceT>
class RegionClient;
template<typename ServiceT>
class RegionService;

/// A service loan keeps its endpoint alive and returns the message on destruction.
template<
  typename ViewT, typename EndpointT, rcl_ret_t (*Return)(const EndpointT *, void **),
  bool ReadOnly = false>
class LoanedRegionServiceMessage
{
public:
  LoanedRegionServiceMessage(const LoanedRegionServiceMessage &) = delete;
  LoanedRegionServiceMessage & operator=(const LoanedRegionServiceMessage &) = delete;

  LoanedRegionServiceMessage(LoanedRegionServiceMessage && other) noexcept
  : endpoint_(std::move(other.endpoint_)), owned_(std::move(other.owned_)),
    constraints_(std::move(other.constraints_)), message_(std::exchange(other.message_, nullptr))
  {}

  ~LoanedRegionServiceMessage()
  {
    if (message_ && !owned_) {
      void * raw = message_;
      auto ret = Return(endpoint_.get(), &raw);
      if (ret != RCL_RET_OK) {
        RCLCPP_ERROR(
          rclcpp::get_logger("rclcpp"), "Failed to return region service loan: %s",
          rcl_get_error_string().str);
        rcl_reset_error();
      }
    }
  }

  bool is_valid() const noexcept {return message_ != nullptr;}

  std::conditional_t<ReadOnly, const ViewT, ViewT> & get() const
  {
    if (!message_) {
      throw std::logic_error("Region service loan is no longer valid");
    }
    return *message_;
  }

private:
  template<typename>
  friend class RegionClient;
  template<typename>
  friend class RegionService;

  LoanedRegionServiceMessage(std::shared_ptr<EndpointT> endpoint, void * message)
  : endpoint_(std::move(endpoint)), message_(static_cast<ViewT *>(message))
  {}

  LoanedRegionServiceMessage(
    std::shared_ptr<EndpointT> endpoint, std::unique_ptr<ViewT> message,
    const rosidl_runtime_cpp::views::MessageConstraints<ViewT> & constraints)
  : endpoint_(std::move(endpoint)), owned_(std::move(message)),
    constraints_(constraints), message_(owned_.get())
  {}

  std::shared_ptr<EndpointT> endpoint_;
  std::unique_ptr<ViewT> owned_;
  rosidl_runtime_cpp::views::MessageConstraints<ViewT> constraints_{};
  ViewT * message_;
};

/// Explicitly polled region client. It is not registered with an rclcpp executor.
template<typename ServiceT>
class RegionClient
{
public:
  using Request = typename ServiceT::Request;
  using Response = typename ServiceT::Response;
  using RequestLoan = LoanedRegionServiceMessage<
    Request, rcl_client_t, rcl_return_loaned_request_from_client>;
  using ResponseLoan = LoanedRegionServiceMessage<
    Response, rcl_client_t, rcl_return_loaned_response_from_client, true>;

  RegionClient(
    std::shared_ptr<rcl_node_t> node, const std::string & service_name,
    const rcl_client_options_t & options = rcl_client_get_default_options(),
    const rosidl_runtime_c__message_context_t * request_context = nullptr,
    const rosidl_runtime_c__message_context_t * response_context = nullptr)
  : native_(rclcpp::detail::region_loan_available(RMW_REGION_LOAN_CLIENT) &&
      rclcpp::detail::region_loan_type_available(
        RMW_REGION_LOAN_CLIENT, type_support()->request_typesupport, request_context,
        type_support()->response_typesupport, response_context)),
    request_constraints_(native_ ? RequestConstraints{} :
      rclcpp::detail::copy_region_constraints<Request>(request_context)),
    response_constraints_(native_ ? ResponseConstraints{} :
      rclcpp::detail::copy_region_constraints<Response>(response_context)),
    request_has_type_constraints_(request_context && request_context->constraints.type_specific),
    response_has_type_constraints_(response_context && response_context->constraints.type_specific)
  {
    if (!node) {
      throw std::invalid_argument("Region client node is null");
    }
    auto handle = std::make_unique<rcl_client_t>(rcl_get_zero_initialized_client());
    auto ret = native_ ?
      rcl_client_init_with_context(
      handle.get(), node.get(), type_support(), request_context, response_context,
      service_name.c_str(), &options) :
      rcl_client_init(handle.get(), node.get(), plain_type_support(), service_name.c_str(),
        &options);
    if (ret != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to initialize region client");
    }
    bool initialized = true;
    auto cleanup = rcpputils::make_scope_exit([&]() {
          if (initialized) {
            auto result = rcl_client_fini(handle.get(), node.get());
            if (result != RCL_RET_OK) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to finalize region client: %s",
                rcl_get_error_string().str);
              rcl_reset_error();
            }
          }
      });
    if (native_) {
      auto * rmw_handle = rcl_client_get_rmw_handle(handle.get());
      if (!rmw_handle) {
        rclcpp::exceptions::throw_from_rcl_error(RCL_RET_ERROR, "failed to get region client");
      }
      bool available = false;
      auto rmw_ret = rmw_region_client_can_loan_messages(rmw_handle, &available);
      if (rmw_ret != RMW_RET_OK) {
        const std::string message = rmw_get_error_string().str;
        rmw_reset_error();
        throw std::runtime_error("failed to query region client loan support: " + message);
      }
      if (!available) {
        request_constraints_ = rclcpp::detail::copy_region_constraints<Request>(request_context);
        response_constraints_ = rclcpp::detail::copy_region_constraints<Response>(response_context);
        auto * support = plain_type_support();
        ret = rcl_client_fini(handle.get(), node.get());
        if (ret != RCL_RET_OK) {
          rclcpp::exceptions::throw_from_rcl_error(ret, "failed to finalize region client");
        }
        initialized = false;
        *handle = rcl_get_zero_initialized_client();
        ret = rcl_client_init(handle.get(), node.get(), support, service_name.c_str(), &options);
        if (ret != RCL_RET_OK) {
          rclcpp::exceptions::throw_from_rcl_error(ret, "failed to initialize copy client");
        }
        initialized = true;
        native_ = false;
      }
    }
    cleanup.cancel();
    handle_ = std::shared_ptr<rcl_client_t>(
      handle.release(), [node](rcl_client_t * client) {
        auto result = rcl_client_fini(client, node.get());
        if (result != RCL_RET_OK) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to finalize region client: %s",
            rcl_get_error_string().str);
          rcl_reset_error();
        }
        delete client;
      });
  }

  RequestLoan borrow_request(const rosidl_runtime_c__message_context_t * context = nullptr)
  {
    if (!native_) {
      auto limits = rclcpp::detail::refine_region_constraints<Request>(
        request_constraints_, context, request_has_type_constraints_);
      return RequestLoan(handle_, std::make_unique<Request>(), limits);
    }
    void * raw = nullptr;
    auto ret = rcl_borrow_loaned_request(handle_.get(), context, &raw);
    if (ret != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to borrow region request");
    }
    if (!raw) {
      throw std::runtime_error("Middleware returned a null region request");
    }
    return RequestLoan(handle_, raw);
  }

  int64_t send_request(RequestLoan && loan)
  {
    if (!loan.is_valid() || loan.endpoint_.get() != handle_.get()) {
      throw std::invalid_argument("Region request does not belong to this client");
    }
    if (loan.owned_) {
      if (!request_constraints_.type_specific.CheckCompatibleView(*loan.message_) ||
        !loan.constraints_.type_specific.CheckCompatibleView(*loan.message_))
      {
        throw std::invalid_argument("Region request exceeds loan constraints");
      }
      auto plain = loan.message_->to_plain();
      int64_t sequence = 0;
      auto ret = rcl_send_request(handle_.get(), &plain, &sequence);
      if (ret != RCL_RET_OK) {
        rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send region request copy");
      }
      loan.owned_.reset();
      loan.message_ = nullptr;
      return sequence;
    }
    void * raw = loan.message_;
    int64_t sequence = 0;
    auto ret = rcl_send_loaned_request(handle_.get(), &raw, &sequence);
    loan.message_ = static_cast<Request *>(raw);
    if (ret != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send region request");
    }
    if (raw) {
      throw std::runtime_error("Middleware did not consume the region request");
    }
    return sequence;
  }

  std::optional<ResponseLoan> take_response(
    rmw_service_info_t & header, const rosidl_runtime_c__message_context_t * context = nullptr)
  {
    if (!native_) {
      auto limits = rclcpp::detail::refine_region_constraints<Response>(
        response_constraints_, context, response_has_type_constraints_);
      typename Response::PlainMessage plain;
      auto ret = rcl_take_response_with_info(handle_.get(), &header, &plain);
      if (ret == RCL_RET_CLIENT_TAKE_FAILED) {
        return std::nullopt;
      }
      if (ret != RCL_RET_OK) {
        rclcpp::exceptions::throw_from_rcl_error(ret, "failed to take region response copy");
      }
      if (!response_constraints_.type_specific.CheckCompatible(plain) ||
        !limits.type_specific.CheckCompatible(plain))
      {
        throw std::invalid_argument("Region response exceeds loan constraints");
      }
      auto view = std::make_unique<Response>();
      view->assign(plain);
      return ResponseLoan(handle_, std::move(view), limits);
    }
    void * raw = nullptr;
    auto ret = rcl_take_loaned_response(handle_.get(), context, &header, &raw);
    if (ret == RCL_RET_CLIENT_TAKE_FAILED) {
      return std::nullopt;
    }
    if (ret != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to take region response");
    }
    if (!raw) {
      throw std::runtime_error("Middleware returned a null region response");
    }
    return ResponseLoan(handle_, raw);
  }

  std::shared_ptr<rcl_client_t> get_client_handle() const {return handle_;}
  /// Report whether the endpoint uses middleware loans instead of copies.
  bool can_loan_messages() const noexcept {return native_;}

private:
  using RequestConstraints = rosidl_runtime_cpp::views::MessageConstraints<Request>;
  using ResponseConstraints = rosidl_runtime_cpp::views::MessageConstraints<Response>;

  static const rosidl_service_type_support_t * plain_type_support()
  {
    auto * support =
      rosidl_typesupport_cpp::get_service_type_support_handle<typename ServiceT::PlainService>();
    if (!support) {
      throw std::runtime_error("Ordinary service type support is null");
    }
    return support;
  }

  static const rosidl_service_type_support_t * type_support()
  {
    auto * support = rosidl_typesupport_cpp::get_service_type_support_handle<ServiceT>();
    if (!support) {
      throw std::runtime_error("Region service type support is null");
    }
    return support;
  }

  std::shared_ptr<rcl_client_t> handle_;
  bool native_;
  RequestConstraints request_constraints_;
  ResponseConstraints response_constraints_;
  const bool request_has_type_constraints_;
  const bool response_has_type_constraints_;
};

/// Explicitly polled region service. It is not registered with an rclcpp executor.
template<typename ServiceT>
class RegionService
{
public:
  using Request = typename ServiceT::Request;
  using Response = typename ServiceT::Response;
  using RequestLoan = LoanedRegionServiceMessage<
    Request, rcl_service_t, rcl_return_loaned_request_from_service, true>;
  using ResponseLoan = LoanedRegionServiceMessage<
    Response, rcl_service_t, rcl_return_loaned_response_from_service>;

  RegionService(
    std::shared_ptr<rcl_node_t> node, const std::string & service_name,
    const rcl_service_options_t & options = rcl_service_get_default_options(),
    const rosidl_runtime_c__message_context_t * request_context = nullptr,
    const rosidl_runtime_c__message_context_t * response_context = nullptr)
  : native_(rclcpp::detail::region_loan_available(RMW_REGION_LOAN_SERVICE) &&
      rclcpp::detail::region_loan_type_available(
        RMW_REGION_LOAN_SERVICE, type_support()->request_typesupport, request_context,
        type_support()->response_typesupport, response_context)),
    request_constraints_(native_ ? RequestConstraints{} :
      rclcpp::detail::copy_region_constraints<Request>(request_context)),
    response_constraints_(native_ ? ResponseConstraints{} :
      rclcpp::detail::copy_region_constraints<Response>(response_context)),
    request_has_type_constraints_(request_context && request_context->constraints.type_specific),
    response_has_type_constraints_(response_context && response_context->constraints.type_specific)
  {
    if (!node) {
      throw std::invalid_argument("Region service node is null");
    }
    auto handle = std::make_unique<rcl_service_t>(rcl_get_zero_initialized_service());
    auto * support = native_ ?
      rosidl_typesupport_cpp::get_service_type_support_handle<ServiceT>() :
      rosidl_typesupport_cpp::get_service_type_support_handle<typename ServiceT::PlainService>();
    if (!support) {
      throw std::runtime_error("Region service type support is null");
    }
    auto ret = native_ ? rcl_service_init_with_context(
      handle.get(), node.get(), support, request_context, response_context,
      service_name.c_str(), &options) :
      rcl_service_init(handle.get(), node.get(), support, service_name.c_str(), &options);
    if (ret != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to initialize region service");
    }
    bool initialized = true;
    auto cleanup = rcpputils::make_scope_exit([&]() {
          if (initialized) {
            auto result = rcl_service_fini(handle.get(), node.get());
            if (result != RCL_RET_OK) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to finalize region service: %s",
                rcl_get_error_string().str);
              rcl_reset_error();
            }
          }
      });
    if (native_) {
      auto * rmw_handle = rcl_service_get_rmw_handle(handle.get());
      if (!rmw_handle) {
        rclcpp::exceptions::throw_from_rcl_error(RCL_RET_ERROR, "failed to get region service");
      }
      bool available = false;
      auto rmw_ret = rmw_region_service_can_loan_messages(rmw_handle, &available);
      if (rmw_ret != RMW_RET_OK) {
        const std::string message = rmw_get_error_string().str;
        rmw_reset_error();
        throw std::runtime_error("failed to query region service loan support: " + message);
      }
      if (!available) {
        request_constraints_ = rclcpp::detail::copy_region_constraints<Request>(request_context);
        response_constraints_ = rclcpp::detail::copy_region_constraints<Response>(response_context);
        support = rosidl_typesupport_cpp::get_service_type_support_handle<
          typename ServiceT::PlainService>();
        if (!support) {
          throw std::runtime_error("Ordinary service type support is null");
        }
        ret = rcl_service_fini(handle.get(), node.get());
        if (ret != RCL_RET_OK) {
          rclcpp::exceptions::throw_from_rcl_error(ret, "failed to finalize region service");
        }
        initialized = false;
        *handle = rcl_get_zero_initialized_service();
        ret = rcl_service_init(handle.get(), node.get(), support, service_name.c_str(), &options);
        if (ret != RCL_RET_OK) {
          rclcpp::exceptions::throw_from_rcl_error(ret, "failed to initialize copy service");
        }
        initialized = true;
        native_ = false;
      }
    }
    cleanup.cancel();
    handle_ = std::shared_ptr<rcl_service_t>(
      handle.release(), [node](rcl_service_t * service) {
        auto result = rcl_service_fini(service, node.get());
        if (result != RCL_RET_OK) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to finalize region service: %s",
            rcl_get_error_string().str);
          rcl_reset_error();
        }
        delete service;
      });
  }

  std::optional<RequestLoan> take_request(
    rmw_service_info_t & header, const rosidl_runtime_c__message_context_t * context = nullptr)
  {
    if (!native_) {
      auto limits = rclcpp::detail::refine_region_constraints<Request>(
        request_constraints_, context, request_has_type_constraints_);
      typename Request::PlainMessage plain;
      auto ret = rcl_take_request_with_info(handle_.get(), &header, &plain);
      if (ret == RCL_RET_SERVICE_TAKE_FAILED) {
        return std::nullopt;
      }
      if (ret != RCL_RET_OK) {
        rclcpp::exceptions::throw_from_rcl_error(ret, "failed to take region request copy");
      }
      if (!request_constraints_.type_specific.CheckCompatible(plain) ||
        !limits.type_specific.CheckCompatible(plain))
      {
        throw std::invalid_argument("Region request exceeds loan constraints");
      }
      auto view = std::make_unique<Request>();
      view->assign(plain);
      return RequestLoan(handle_, std::move(view), limits);
    }
    void * raw = nullptr;
    auto ret = rcl_take_loaned_request(handle_.get(), context, &header, &raw);
    if (ret == RCL_RET_SERVICE_TAKE_FAILED) {
      return std::nullopt;
    }
    if (ret != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to take region request");
    }
    if (!raw) {
      throw std::runtime_error("Middleware returned a null region request");
    }
    return RequestLoan(handle_, raw);
  }

  ResponseLoan borrow_response(const rosidl_runtime_c__message_context_t * context = nullptr)
  {
    if (!native_) {
      auto limits = rclcpp::detail::refine_region_constraints<Response>(
        response_constraints_, context, response_has_type_constraints_);
      return ResponseLoan(handle_, std::make_unique<Response>(), limits);
    }
    void * raw = nullptr;
    auto ret = rcl_borrow_loaned_response(handle_.get(), context, &raw);
    if (ret != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to borrow region response");
    }
    if (!raw) {
      throw std::runtime_error("Middleware returned a null region response");
    }
    return ResponseLoan(handle_, raw);
  }

  void send_response(const rmw_request_id_t & header, ResponseLoan && loan)
  {
    if (!loan.is_valid() || loan.endpoint_.get() != handle_.get()) {
      throw std::invalid_argument("Region response does not belong to this service");
    }
    if (loan.owned_) {
      if (!response_constraints_.type_specific.CheckCompatibleView(*loan.message_) ||
        !loan.constraints_.type_specific.CheckCompatibleView(*loan.message_))
      {
        throw std::invalid_argument("Region response exceeds loan constraints");
      }
      auto plain = loan.message_->to_plain();
      auto response_header = header;
      auto ret = rcl_send_response(handle_.get(), &response_header, &plain);
      if (ret != RCL_RET_OK) {
        rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send region response copy");
      }
      loan.owned_.reset();
      loan.message_ = nullptr;
      return;
    }
    void * raw = loan.message_;
    auto ret = rcl_send_loaned_response(handle_.get(), &header, &raw);
    loan.message_ = static_cast<Response *>(raw);
    if (ret != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send region response");
    }
    if (raw) {
      throw std::runtime_error("Middleware did not consume the region response");
    }
  }

  std::shared_ptr<rcl_service_t> get_service_handle() const {return handle_;}
  /// Report whether the endpoint uses middleware loans instead of copies.
  bool can_loan_messages() const noexcept {return native_;}

private:
  using RequestConstraints = rosidl_runtime_cpp::views::MessageConstraints<Request>;
  using ResponseConstraints = rosidl_runtime_cpp::views::MessageConstraints<Response>;

  static const rosidl_service_type_support_t * type_support()
  {
    auto * support = rosidl_typesupport_cpp::get_service_type_support_handle<ServiceT>();
    if (!support) {
      throw std::runtime_error("Region service type support is null");
    }
    return support;
  }

  std::shared_ptr<rcl_service_t> handle_;
  bool native_;
  RequestConstraints request_constraints_;
  ResponseConstraints response_constraints_;
  const bool request_has_type_constraints_;
  const bool response_has_type_constraints_;
};

}  // namespace rclcpp

#endif  // RCLCPP__REGION_SERVICE_HPP_
