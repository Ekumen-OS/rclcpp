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

#include <stdlib.h>
#include <string.h>

#include "rmw/error_handling.h"
#include "rmw/rmw.h"

static rmw_ret_t
service_capability(bool * available)
{
  const char * mode = getenv("REGION_TEST_SERVICE_QUERY");
  if (mode && strcmp(mode, "error") == 0) {
    RMW_SET_ERROR_MSG("injected service capability error");
    return RMW_RET_ERROR;
  }
  *available = false;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_region_client_can_loan_messages(const rmw_client_t * client, bool * available)
{
  (void)client;
  return service_capability(available);
}

rmw_ret_t
rmw_region_service_can_loan_messages(const rmw_service_t * service, bool * available)
{
  (void)service;
  return service_capability(available);
}
