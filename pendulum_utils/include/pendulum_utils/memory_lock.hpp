// Copyright 2019 Carlos San Vicente
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

#ifndef PENDULUM_UTILS__MEMORY_LOCK_HPP_
#define PENDULUM_UTILS__MEMORY_LOCK_HPP_

#include <sys/types.h>
#include <sys/syscall.h>
#include <sys/mman.h>

#include <stdlib.h>
#include <limits.h>
#include <malloc.h>
#include <sys/resource.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#include <iostream>
#include <cerrno>
#include <cstring>
#include <memory>

namespace pendulum
{
namespace utils
{
/// \brief Lock currently paged memory using mlockall.
/// \return Error code to propagate to main
int lock_memory();

/// \brief Commit a pool of dynamic memory based on the memory already cached
/// by this process by checking the number of pagefaults.
/// \return Error code to propagate to main
int lock_and_prefault_dynamic();

/// \brief Commit a pool of dynamic memory based on a prefixed size
/// \return Error code to propagate to main
int lock_and_prefault_dynamic(size_t process_max_dynamic_memory);
}  // namespace utils
}  // namespace pendulum

#endif  // PENDULUM_UTILS__MEMORY_LOCK_HPP_
