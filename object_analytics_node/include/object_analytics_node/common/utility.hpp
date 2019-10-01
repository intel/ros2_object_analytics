// Copyright (c) 2018 Intel Corporation
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

#pragma once

#define __FILENAME__ \
  (strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1) : __FILE__)

#ifndef NDEBUG
#define TRACE_INFO(fmt, args...)                                  \
  printf("[INFO]: %s(%d)<%s>\t" fmt "\n", __FILENAME__, __LINE__, \
         __FUNCTION__, ##args)
#else
#define TRACE_INFO(fmt, args...)
#endif

#define TRACE_ERR(fmt, args...)                                   \
  printf("[ERR ]: %s(%d)<%s>\t" fmt "\n", __FILENAME__, __LINE__, \
         __FUNCTION__, ##args)
