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
#include "boost/format.hpp"


#define __FILENAME__ \
  (strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1) : __FILE__)

#ifndef NDEBUG

#define TRACE_INFO(fmt, args...)                                  \
  printf("[INFO]: %s(%d)<%s>\t" fmt "\n", __FILENAME__, __LINE__, \
         __FUNCTION__, ##args);                                   \
         std::cout << ""

#else
#define TRACE_INFO(fmt, args...)
#endif

#define TRACE_ERR(fmt, args...)                                   \
  printf("[ERR ]: %s(%d)<%s>\t" fmt "\n", __FILENAME__, __LINE__, \
         __FUNCTION__, ##args)



#if 0
#include "boost/format.hpp"
#include <cstdarg>



#define __FILENAME__ \
  (strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1) : __FILE__)

#ifndef NDEBUG
#if 0
#define TRACE_INFO(fmt, args...)                                  \
  printf("[INFO]: %s(%d)<%s>\t" fmt "\n", __FILENAME__, __LINE__, \
         __FUNCTION__, ##args); \
         std::cout <<""
#endif

#define TRACE_INFO(fmt, ...)                                  \
  logger log;\
  log.trace_info(__FILE__,__LINE__ ,__FUNCTION__, fmt, ##__VA_ARGS__)

#define TRACE_CLASS(value, )                                  \
  logger log;\
  log.trace_info(__FILE__,__LINE__ ,__FUNCTION__, fmt, ##__VA_ARGS__)

#else
#define TRACE_INFO(fmt, args...)
#endif

#define TRACE_ERR(fmt, args...)                                   \
  printf("[ERR ]: %s(%d)<%s>\t" fmt "\n", __FILENAME__, __LINE__, \
         __FUNCTION__, ##args)

class logger
{
public:
  void trace_info(const char* file, int line, const char* func, const char *format, ...)
  {
    fprintf(stdout, "%s(%d):%s:\t", file, line, func);

    va_list args;
    va_start(args, format);
    vfprintf(stdout, format, args);
    va_end(args);
  };

  template<typename T>
  void trace_class(T value, std::string desc="")
  {
    std::cout << desc << std::endl;
    std::cout << value << std::endl;
  };

};
#endif
