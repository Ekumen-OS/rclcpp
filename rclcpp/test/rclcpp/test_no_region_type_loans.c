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

rmw_ret_t
rmw_region_loan_type_support_available(
  rmw_region_loan_role_t role, const rosidl_message_type_support_t * first_type,
  const rosidl_runtime_c__message_context_t * first_context,
  const rosidl_message_type_support_t * second_type,
  const rosidl_runtime_c__message_context_t * second_context, bool * available)
{
  (void)role;
  (void)first_type;
  (void)first_context;
  (void)second_type;
  (void)second_context;
  const char * mode = getenv("REGION_TEST_TYPE_QUERY");
  if (mode && strcmp(mode, "error") == 0) {
    RMW_SET_ERROR_MSG("injected type capability error");
    return RMW_RET_ERROR;
  }
  *available = false;
  return RMW_RET_OK;
}
