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

#include <time.h>

#include "rcl/client.h"
#include "rcl/graph.h"
#include "rcl/service.h"
#include "test_msgs/srv/views/basic_types.h"

#define REQUIRE_C(test) do {if (!(test)) {result = __LINE__; goto cleanup;}} while (0)

int test_c_region_service(rcl_node_t * node)
{
  int result = 0;
  bool client_initialized = false;
  bool service_initialized = false;
  bool available = false;
  rcl_client_t client = rcl_get_zero_initialized_client();
  rcl_service_t service = rcl_get_zero_initialized_service();
  rcl_client_options_t client_options = rcl_client_get_default_options();
  rcl_service_options_t service_options = rcl_service_get_default_options();
  test_msgs__srv__BasicTypes_Request__RegionConstraints * request_limits = NULL;
  test_msgs__srv__BasicTypes_Response__RegionConstraints * response_limits = NULL;
  rosidl_runtime_c__message_context_t request_context = {0};
  rosidl_runtime_c__message_context_t response_context = {0};
  void * writer_request = NULL;
  void * reader_request = NULL;
  void * writer_response = NULL;
  void * reader_response = NULL;
  int64_t sequence = 0;
  uint32_t value = 0;
  rmw_service_info_t header = {0};
  struct timespec delay = {0, 20000000};

  request_limits = test_msgs__srv__BasicTypes_Request__RegionConstraints__create();
  response_limits = test_msgs__srv__BasicTypes_Response__RegionConstraints__create();
  REQUIRE_C(request_limits && response_limits);
  REQUIRE_C(test_msgs__srv__BasicTypes_Request__RegionConstraints__string_value__set_string_size(
      request_limits, 8));
  REQUIRE_C(test_msgs__srv__BasicTypes_Response__RegionConstraints__string_value__set_string_size(
      response_limits, 8));
  request_context.constraints.type_specific =
    test_msgs__srv__BasicTypes_Request__RegionConstraints__data(request_limits);
  response_context.constraints.type_specific =
    test_msgs__srv__BasicTypes_Response__RegionConstraints__data(response_limits);
  const rosidl_service_type_support_t * support =
    test_msgs__srv__BasicTypes__RegionView__get_type_support();
  REQUIRE_C(support);
  REQUIRE_C(RCL_RET_OK == rcl_service_init_with_context(
      &service, node, support, &request_context, &response_context,
      "c_region_service", &service_options));
  service_initialized = true;
  REQUIRE_C(RCL_RET_OK == rcl_client_init_with_context(
      &client, node, support, &request_context, &response_context,
      "c_region_service", &client_options));
  client_initialized = true;
  for (int i = 0; i < 100 && !available; ++i) {
    REQUIRE_C(RCL_RET_OK == rcl_service_server_is_available(node, &client, &available));
    if (!available) {nanosleep(&delay, NULL);}
  }
  REQUIRE_C(available);
  REQUIRE_C(RCL_RET_OK == rcl_borrow_loaned_request(&client, NULL, &writer_request));
  REQUIRE_C(test_msgs__srv__BasicTypes_Request__RegionView__uint32_value__set(
      test_msgs__srv__BasicTypes_Request__RegionView__from_loan(writer_request), 72));
  REQUIRE_C(test_msgs__srv__BasicTypes_Request__RegionView__string_value__set(
      test_msgs__srv__BasicTypes_Request__RegionView__from_loan(writer_request), "test", 4));
  REQUIRE_C(RCL_RET_OK == rcl_send_loaned_request(&client, &writer_request, &sequence));
  REQUIRE_C(!writer_request);
  for (int i = 0; i < 100 && !reader_request; ++i) {
    rcl_ret_t ret = rcl_take_loaned_request(&service, NULL, &header, &reader_request);
    REQUIRE_C(ret == RCL_RET_OK || ret == RCL_RET_SERVICE_TAKE_FAILED);
    if (!reader_request) {nanosleep(&delay, NULL);}
  }
  REQUIRE_C(reader_request && header.request_id.sequence_number == sequence);
  REQUIRE_C(test_msgs__srv__BasicTypes_Request__ConstRegionView__uint32_value__get(
      test_msgs__srv__BasicTypes_Request__ConstRegionView__from_loan(reader_request), &value));
  REQUIRE_C(value == 72);
  REQUIRE_C(RCL_RET_OK == rcl_return_loaned_request_from_service(&service, &reader_request));
  REQUIRE_C(!reader_request);
  REQUIRE_C(RCL_RET_OK == rcl_borrow_loaned_response(&service, NULL, &writer_response));
  REQUIRE_C(test_msgs__srv__BasicTypes_Response__RegionView__uint32_value__set(
      test_msgs__srv__BasicTypes_Response__RegionView__from_loan(writer_response), 73));
  REQUIRE_C(RCL_RET_OK == rcl_send_loaned_response(&service, &header.request_id, &writer_response));
  REQUIRE_C(!writer_response);
  for (int i = 0; i < 100 && !reader_response; ++i) {
    rcl_ret_t ret = rcl_take_loaned_response(&client, NULL, &header, &reader_response);
    REQUIRE_C(ret == RCL_RET_OK || ret == RCL_RET_CLIENT_TAKE_FAILED);
    if (!reader_response) {nanosleep(&delay, NULL);}
  }
  REQUIRE_C(reader_response && header.request_id.sequence_number == sequence);
  REQUIRE_C(test_msgs__srv__BasicTypes_Response__ConstRegionView__uint32_value__get(
      test_msgs__srv__BasicTypes_Response__ConstRegionView__from_loan(reader_response), &value));
  REQUIRE_C(value == 73);
  REQUIRE_C(RCL_RET_OK == rcl_return_loaned_response_from_client(&client, &reader_response));
  REQUIRE_C(!reader_response);

cleanup:
  if (writer_request && RCL_RET_OK != rcl_return_loaned_request_from_client(
      &client, &writer_request)) {result = result ? result : __LINE__;}
  if (reader_request && RCL_RET_OK != rcl_return_loaned_request_from_service(
      &service, &reader_request)) {result = result ? result : __LINE__;}
  if (writer_response && RCL_RET_OK != rcl_return_loaned_response_from_service(
      &service, &writer_response)) {result = result ? result : __LINE__;}
  if (reader_response && RCL_RET_OK != rcl_return_loaned_response_from_client(
      &client, &reader_response)) {result = result ? result : __LINE__;}
  if (client_initialized && RCL_RET_OK != rcl_client_fini(&client, node)) {
    result = result ? result : __LINE__;
  }
  if (service_initialized && RCL_RET_OK != rcl_service_fini(&service, node)) {
    result = result ? result : __LINE__;
  }
  test_msgs__srv__BasicTypes_Request__RegionConstraints__destroy(request_limits);
  test_msgs__srv__BasicTypes_Response__RegionConstraints__destroy(response_limits);
  return result;
}

#undef REQUIRE_C
