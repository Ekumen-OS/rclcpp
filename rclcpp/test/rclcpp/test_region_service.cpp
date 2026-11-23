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

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>

#include "gtest/gtest.h"
#include "rcpputils/scope_exit.hpp"
#include "rcutils/env.h"
#include "rclcpp/node.hpp"
#include "rclcpp/region_service.hpp"
#include "rclcpp/utilities.hpp"
#include "rmw/rmw.h"
#include "test_msgs/srv/basic_types.hpp"
#include "test_msgs/srv/views/basic_types__region.hpp"

using Region = test_msgs::srv::views::RegionBasicTypes;
using Client = rclcpp::RegionClient<Region>;
using Service = rclcpp::RegionService<Region>;

extern "C" int test_c_region_service(rcl_node_t * node);

static_assert(std::is_const_v<std::remove_reference_t<decltype(
    std::declval<const Service::RequestLoan &>().get())>>);
static_assert(std::is_const_v<std::remove_reference_t<decltype(
    std::declval<const Client::ResponseLoan &>().get())>>);

class TestRegionService : public ::testing::Test
{
protected:
  static void SetUpTestCase() {rclcpp::init(0, nullptr);}
  static void TearDownTestCase() {rclcpp::shutdown();}
};

TEST_F(TestRegionService, c_region_service_round_trip) {
  if (std::strcmp(rmw_get_implementation_identifier(), "rmw_fastrtps_cpp") != 0) {
    GTEST_SKIP() << "Fast DDS is required for region service loans";
  }
  auto node = std::make_shared<rclcpp::Node>("c_region_service_test");
  auto handle = node->get_node_base_interface()->get_shared_rcl_node_handle();
  EXPECT_EQ(0, test_c_region_service(handle.get())) << rcl_get_error_string().str;
}

TEST_F(TestRegionService, request_loan_keeps_client_alive) {
  if (std::strcmp(rmw_get_implementation_identifier(), "rmw_fastrtps_cpp") != 0) {
    GTEST_SKIP() << "Fast DDS is required for region service loans";
  }
  auto node = std::make_shared<rclcpp::Node>("region_service_lifetime");
  auto node_handle = node->get_node_base_interface()->get_shared_rcl_node_handle();
  Region::Request::Constraints limits;
  limits.string_value.size = 8;
  rosidl_runtime_c__message_context_t context{};
  context.constraints.type_specific = &limits;
  std::optional<Client::RequestLoan> loan;
  std::weak_ptr<rcl_client_t> handle;
  {
    Client client(
      node_handle, "region_lifetime", rcl_client_get_default_options(), &context, nullptr);
    handle = client.get_client_handle();
    loan.emplace(client.borrow_request());
  }
  EXPECT_FALSE(handle.expired());
  loan.reset();
  EXPECT_TRUE(handle.expired());
}

void region_service_round_trip(bool copy, bool disable_loans = false, bool bounded = true)
{
  if (!copy && std::strcmp(rmw_get_implementation_identifier(), "rmw_fastrtps_cpp") != 0) {
    GTEST_SKIP() << "Fast DDS is required for region service loans";
  }
  const char * previous = nullptr;
  ASSERT_EQ(nullptr, rcutils_get_env("ROS_DISABLE_LOANED_MESSAGES", &previous));
  const std::string saved = previous;
  auto restore = rcpputils::make_scope_exit([&saved]() {
        EXPECT_TRUE(rcutils_set_env("ROS_DISABLE_LOANED_MESSAGES", saved.c_str()));
    });
  ASSERT_TRUE(rcutils_set_env("ROS_DISABLE_LOANED_MESSAGES", disable_loans ? "1" : "0"));
  {
    auto node = std::make_shared<rclcpp::Node>("region_service_test");
    auto node_handle = node->get_node_base_interface()->get_shared_rcl_node_handle();
    Region::Request::Constraints request_limits;
    request_limits.string_value.size = 8;
    Region::Response::Constraints response_limits;
    response_limits.string_value.size = 8;
    rosidl_runtime_c__message_context_t request_context{};
    request_context.constraints.type_specific = &request_limits;
    rosidl_runtime_c__message_context_t response_context{};
    response_context.constraints.type_specific = &response_limits;
    Service service(
      node_handle, "region_service", rcl_service_get_default_options(),
      bounded ? &request_context : nullptr, bounded ? &response_context : nullptr);
    Client client(
      node_handle, "region_service", rcl_client_get_default_options(),
      bounded ? &request_context : nullptr, bounded ? &response_context : nullptr);
    EXPECT_EQ(!copy, service.can_loan_messages());
    EXPECT_EQ(!copy, client.can_loan_messages());
    bool available = false;
    for (int i = 0; i < 100 && !available; ++i) {
      ASSERT_EQ(
        RCL_RET_OK, rcl_service_server_is_available(
          node_handle.get(), client.get_client_handle().get(), &available));
      if (!available) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }

    ASSERT_TRUE(available);
    if (copy && bounded) {
      rosidl_runtime_c__message_context_t strict_refinement{};
      strict_refinement.constraints.strict = true;
      auto inherited = client.borrow_request(&strict_refinement);
      inherited.get().string_value().assign("inherited");
      EXPECT_THROW(client.send_request(std::move(inherited)), std::invalid_argument);

      Region::Request::Constraints partial_limits;
      rosidl_runtime_c__message_context_t partial{};
      partial.constraints.type_specific = &partial_limits;
      auto partial_loan = client.borrow_request(&partial);
      partial_loan.get().string_value().assign("too long!");
      EXPECT_THROW(client.send_request(std::move(partial_loan)), std::invalid_argument);

      Region::Request::Constraints tighter;
      tighter.string_value.size = 2;
      rosidl_runtime_c__message_context_t refinement{};
      refinement.constraints.type_specific = &tighter;
      auto rejected = client.borrow_request(&refinement);
      rejected.get().string_value().assign("bad");
      EXPECT_THROW(client.send_request(std::move(rejected)), std::invalid_argument);
      EXPECT_TRUE(rejected.is_valid());
      tighter.string_value.size = 9;
      EXPECT_THROW(client.borrow_request(&refinement), std::invalid_argument);
    }
    auto loan = client.borrow_request();
    loan.get().uint32_value() = 42;
    auto sequence = client.send_request(std::move(loan));
    EXPECT_FALSE(loan.is_valid());
    rmw_service_info_t header{};
    std::optional<Service::RequestLoan> request;
    for (int i = 0; i < 100 && !request; ++i) {
      auto candidate = service.take_request(header);
      if (candidate) {
        request.emplace(std::move(*candidate));
      }
      if (!request) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }
    ASSERT_TRUE(request.has_value());
    EXPECT_EQ(42u, request->get().uint32_value());
    EXPECT_EQ(sequence, header.request_id.sequence_number);
    request.reset();
    if (copy && bounded) {
      Region::Response::Constraints partial_limits;
      rosidl_runtime_c__message_context_t partial{};
      partial.constraints.type_specific = &partial_limits;
      auto partial_loan = service.borrow_response(&partial);
      partial_loan.get().string_value().assign("too long!");
      EXPECT_THROW(
        service.send_response(header.request_id, std::move(partial_loan)), std::invalid_argument);
    }
    auto response = service.borrow_response();
    response.get().uint32_value() = 43;
    service.send_response(header.request_id, std::move(response));
    EXPECT_FALSE(response.is_valid());
    std::optional<Client::ResponseLoan> received;
    for (int i = 0; i < 100 && !received; ++i) {
      auto candidate = client.take_response(header);
      if (candidate) {
        received.emplace(std::move(*candidate));
      }
      if (!received) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }
    ASSERT_TRUE(received.has_value());
    EXPECT_EQ(43u, received->get().uint32_value());
    EXPECT_EQ(sequence, header.request_id.sequence_number);
  }
}

TEST_F(TestRegionService, loan_round_trip) {
  region_service_round_trip(false);
}

TEST_F(TestRegionService, copy_round_trip) {
  region_service_round_trip(true, true);
}

TEST_F(TestRegionService, unbounded_endpoints_fall_back_to_copy) {
  if (std::strcmp(rmw_get_implementation_identifier(), "rmw_fastrtps_cpp") != 0) {
    GTEST_SKIP() << "Fast DDS is required to exercise endpoint capability detection";
  }
  region_service_round_trip(true, false, false);
}

TEST_F(TestRegionService, unsupported_request_does_not_hide_invalid_response_type) {
  if (std::strcmp(rmw_get_implementation_identifier(), "rmw_fastrtps_cpp") != 0) {
    GTEST_SKIP() << "Fast DDS is required to exercise type capability detection";
  }
  auto request =
    rosidl_typesupport_cpp::get_message_type_support_handle<test_msgs::srv::BasicTypes::Request>();
  rosidl_message_type_support_t invalid_response{};
  bool available = true;
  EXPECT_EQ(
    RMW_RET_ERROR,
    rmw_region_loan_type_support_available(
      RMW_REGION_LOAN_CLIENT, request, nullptr, &invalid_response, nullptr, &available));
  rmw_reset_error();
}

void injected_capability(const char * variable, const char * expected_error)
{
  const char * mode = std::getenv(variable);
  if (!mode) {
    GTEST_SKIP() << "Service capability injection is not enabled";
  }
  if (std::strcmp(mode, "unavailable") == 0) {
    region_service_round_trip(true);
    return;
  }
  ASSERT_STREQ(mode, "error");
  auto node = std::make_shared<rclcpp::Node>("region_service_capability_error");
  auto handle = node->get_node_base_interface()->get_shared_rcl_node_handle();
  Region::Request::Constraints request_limits;
  request_limits.string_value.size = 8;
  Region::Response::Constraints response_limits;
  response_limits.string_value.size = 8;
  rosidl_runtime_c__message_context_t request_context{};
  request_context.constraints.type_specific = &request_limits;
  rosidl_runtime_c__message_context_t response_context{};
  response_context.constraints.type_specific = &response_limits;
  try {
    Client client(
      handle, "region_service_error", rcl_client_get_default_options(),
      &request_context, &response_context);
    FAIL() << "Expected client capability error";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find(expected_error), std::string::npos);
  }
  try {
    Service service(
      handle, "region_service_error", rcl_service_get_default_options(),
      &request_context, &response_context);
    FAIL() << "Expected service capability error";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find(expected_error), std::string::npos);
  }
}

TEST_F(TestRegionService, injected_service_capability) {
  injected_capability("REGION_TEST_SERVICE_QUERY", "injected service capability error");
}

TEST_F(TestRegionService, injected_type_capability) {
  injected_capability("REGION_TEST_TYPE_QUERY", "injected type capability error");
}

TEST_F(TestRegionService, missing_service_loan_protocol_uses_copy) {
  bool available = true;
  ASSERT_EQ(RMW_RET_OK, rmw_region_loan_entry_points_available(RMW_REGION_LOAN_CLIENT, &available));
  if (available) {
    GTEST_SKIP() << "Selected middleware exposes region service loans";
  }
  region_service_round_trip(true);
}

TEST_F(TestRegionService, copy_service_interoperates_with_ordinary_client) {
  const char * previous = nullptr;
  ASSERT_EQ(nullptr, rcutils_get_env("ROS_DISABLE_LOANED_MESSAGES", &previous));
  const std::string saved = previous;
  auto restore = rcpputils::make_scope_exit([&saved]() {
        EXPECT_TRUE(rcutils_set_env("ROS_DISABLE_LOANED_MESSAGES", saved.c_str()));
    });
  ASSERT_TRUE(rcutils_set_env("ROS_DISABLE_LOANED_MESSAGES", "0"));
  auto node = std::make_shared<rclcpp::Node>("region_copy_service_peer");
  auto node_handle = node->get_node_base_interface()->get_shared_rcl_node_handle();
  Service service(node_handle, "region_copy_peer");
  EXPECT_FALSE(service.can_loan_messages());
  auto client = rcl_get_zero_initialized_client();
  auto options = rcl_client_get_default_options();
  auto support =
    rosidl_typesupport_cpp::get_service_type_support_handle<test_msgs::srv::BasicTypes>();
  ASSERT_EQ(RCL_RET_OK, rcl_client_init(
      &client, node_handle.get(), support, "region_copy_peer", &options));
  auto cleanup = rcpputils::make_scope_exit([&]() {
        EXPECT_EQ(RCL_RET_OK, rcl_client_fini(&client, node_handle.get()));
    });
  bool available = false;
  for (int i = 0; i < 100 && !available; ++i) {
    ASSERT_EQ(RCL_RET_OK, rcl_service_server_is_available(node_handle.get(), &client, &available));
    if (!available) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  ASSERT_TRUE(available);
  test_msgs::srv::BasicTypes::Request plain_request;
  plain_request.int32_value = 41;
  int64_t sequence = 0;
  ASSERT_EQ(RCL_RET_OK, rcl_send_request(&client, &plain_request, &sequence));
  rmw_service_info_t header{};
  std::optional<Service::RequestLoan> request;
  for (int i = 0; i < 100 && !request; ++i) {
    auto candidate = service.take_request(header);
    if (candidate) {
      request.emplace(std::move(*candidate));
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  ASSERT_TRUE(request.has_value());
  EXPECT_EQ(request->get().int32_value(), 41);
  EXPECT_EQ(header.request_id.sequence_number, sequence);
  auto response = service.borrow_response();
  response.get().int32_value() = 42;
  service.send_response(header.request_id, std::move(response));
  test_msgs::srv::BasicTypes::Response plain_response;
  rcl_ret_t ret = RCL_RET_CLIENT_TAKE_FAILED;
  for (int i = 0; i < 100 && ret == RCL_RET_CLIENT_TAKE_FAILED; ++i) {
    ret = rcl_take_response_with_info(&client, &header, &plain_response);
    if (ret == RCL_RET_CLIENT_TAKE_FAILED) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  ASSERT_EQ(RCL_RET_OK, ret);
  EXPECT_EQ(plain_response.int32_value, 42);
  EXPECT_EQ(header.request_id.sequence_number, sequence);
}

TEST_F(TestRegionService, copy_client_interoperates_with_ordinary_service) {
  const char * previous = nullptr;
  ASSERT_EQ(nullptr, rcutils_get_env("ROS_DISABLE_LOANED_MESSAGES", &previous));
  const std::string saved = previous;
  auto restore = rcpputils::make_scope_exit([&saved]() {
        EXPECT_TRUE(rcutils_set_env("ROS_DISABLE_LOANED_MESSAGES", saved.c_str()));
    });
  ASSERT_TRUE(rcutils_set_env("ROS_DISABLE_LOANED_MESSAGES", "0"));
  auto node = std::make_shared<rclcpp::Node>("region_copy_client_peer");
  auto node_handle = node->get_node_base_interface()->get_shared_rcl_node_handle();
  Client client(node_handle, "region_copy_peer");
  EXPECT_FALSE(client.can_loan_messages());
  auto service = rcl_get_zero_initialized_service();
  auto options = rcl_service_get_default_options();
  auto support =
    rosidl_typesupport_cpp::get_service_type_support_handle<test_msgs::srv::BasicTypes>();
  ASSERT_EQ(RCL_RET_OK, rcl_service_init(
      &service, node_handle.get(), support, "region_copy_peer", &options));
  auto cleanup = rcpputils::make_scope_exit([&]() {
        EXPECT_EQ(RCL_RET_OK, rcl_service_fini(&service, node_handle.get()));
    });
  bool available = false;
  for (int i = 0; i < 100 && !available; ++i) {
    ASSERT_EQ(RCL_RET_OK, rcl_service_server_is_available(
        node_handle.get(), client.get_client_handle().get(), &available));
    if (!available) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  ASSERT_TRUE(available);
  auto request = client.borrow_request();
  request.get().int32_value() = 41;
  auto sequence = client.send_request(std::move(request));
  test_msgs::srv::BasicTypes::Request plain_request;
  rmw_service_info_t header{};
  rcl_ret_t ret = RCL_RET_SERVICE_TAKE_FAILED;
  for (int i = 0; i < 100 && ret == RCL_RET_SERVICE_TAKE_FAILED; ++i) {
    ret = rcl_take_request_with_info(&service, &header, &plain_request);
    if (ret == RCL_RET_SERVICE_TAKE_FAILED) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  ASSERT_EQ(RCL_RET_OK, ret);
  EXPECT_EQ(plain_request.int32_value, 41);
  EXPECT_EQ(header.request_id.sequence_number, sequence);
  test_msgs::srv::BasicTypes::Response plain_response;
  plain_response.int32_value = 42;
  ASSERT_EQ(RCL_RET_OK, rcl_send_response(&service, &header.request_id, &plain_response));
  std::optional<Client::ResponseLoan> response;
  for (int i = 0; i < 100 && !response; ++i) {
    auto candidate = client.take_response(header);
    if (candidate) {
      response.emplace(std::move(*candidate));
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  ASSERT_TRUE(response.has_value());
  EXPECT_EQ(response->get().int32_value(), 42);
  EXPECT_EQ(header.request_id.sequence_number, sequence);
}
