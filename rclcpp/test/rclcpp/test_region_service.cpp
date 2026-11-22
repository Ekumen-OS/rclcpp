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
#include <cstring>
#include <memory>
#include <thread>
#include <type_traits>
#include <utility>

#include "gtest/gtest.h"
#include "rclcpp/node.hpp"
#include "rclcpp/region_service.hpp"
#include "rclcpp/utilities.hpp"
#include "rmw/rmw.h"
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

TEST_F(TestRegionService, loan_round_trip) {
  if (std::strcmp(rmw_get_implementation_identifier(), "rmw_fastrtps_cpp") != 0) {
    GTEST_SKIP() << "Fast DDS is required for region service loans";
  }
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
      &request_context, &response_context);
    Client client(
      node_handle, "region_service", rcl_client_get_default_options(),
      &request_context, &response_context);
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
