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

#ifndef OBJECT_ANALYTICS_NODE__UTIL__LOGGER_HPP_
#define OBJECT_ANALYTICS_NODE__UTIL__LOGGER_HPP_

#include <boost/type_erasure/any.hpp>
#include <boost/type_erasure/operators.hpp>

#include <iostream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <memory>

#include "common/utility.hpp"

#define REGISTER_LOGGER(LOGGER_NAME) \
  bool diag::LOGGER_NAME::registered = \
    diag::loggerFarm::registerLogger(#LOGGER_NAME, LOGGER_NAME::create())

#ifndef NDEBUG
#define TRACE_INFO(...) \
  { \
    auto log = diag::loggerFarm::getLogger("consoleLogger"); \
    if (log != nullptr) { \
      std::string header = __FILE__; \
      header += "("; \
      header += std::to_string(__LINE__); \
      header += ")"; \
      header += ":"; \
      header += __FUNCTION__; \
      log->log(header); \
      log->log(__VA_ARGS__); \
    } \
  }
#else
#define TRACE_INFO(...)
#endif

#define TRACE_ERR(...) \
  { \
    auto log = diag::loggerFarm::getLogger("consoleLogger"); \
    if (log != nullptr) { \
      std::string header = __FILE__; \
      header += "("; \
      header += std::to_string(__LINE__); \
      header += ")"; \
      header += ":"; \
      header += __FUNCTION__; \
      log->log(header); \
      log->log(__VA_ARGS__); \
    } \
  }

namespace diag
{
namespace te = boost::type_erasure;

typedef te::any<boost::mpl::vector<te::copy_constructible<>, te::destructible<>,
    te::ostreamable<>>>
  stream_any;

enum logLevel { TRACE = 0, INFO, DEBUG, WARN, ERR, FATAL };

class loggerConfig
{
public:
  bool validModule(std::string module)
  {
    auto search = moduleList_.find(module);
    if (search == moduleList_.end()) {
      return true;
    } else {
      return false;
    }
  }

  bool addModule(std::string module)
  {
    if (validModule(module)) {return false;}

    moduleList_.insert(module);

    return true;
  }

public:
  logLevel level_;
  std::unordered_set<std::string> moduleList_;
};

class loggerBase
{
public:
  loggerBase() {}
  virtual ~loggerBase() {}

  // @brief log format as printf()
  template<typename T, typename ... Args>
  void log(const char * s, const T & value, const Args & ... args)
  {
    if (!log_begin()) {return;}

    log_char(s, value, args ...);

    log_end();
  }

  template<typename T, typename ... Args>
  void log_char(const char * s, const T & value, const Args & ... args)
  {
    while (*s) {
      if (*s == '%') {
        ++s;
        if (*s != '%') {
          log_structure(value);
          log_char(s + 1, args ...);
          return;
        }
      }

      log_char(*s++);
    }
  }

  // @brief log format as std::cout for structure
  template<typename T, typename ... Args>
  void log(const T & value, const Args & ... args)
  {
    if (!log_begin()) {return;}

    log_dat(value, args ...);

    log_end();
  }

  template<typename T, typename ... Args>
  void log_dat(const T & value, const Args & ... args)
  {
    log_structure(value);
    log_dat(args ...);
  }

  template<typename T>
  void log_dat(const T & value)
  {
    log_structure(value);
  }

  // @brief sub function to be implemented in sub-class
  virtual void log_char(const char * s) = 0;
  virtual void log_char(const char s) = 0;
  virtual void log_structure(const stream_any & s) = 0;

  virtual bool log_begin() = 0;
  virtual void log_end() = 0;

  // @brief Flush loger
  virtual void flush() = 0;

public:
  std::string loggerName_;
};

class loggerFarm
{
public:
  /// @brief Gets existing logger
  static std::shared_ptr<loggerBase> getLogger(const std::string & name)
  {
    auto search = loggerBaseList_.find(name);
    if (search != loggerBaseList_.end()) {
      return search->second;
    } else {
      return NULL;
    }
  }

  // @brief loggerBase registration
  static bool registerLogger(
    const std::string & name,
    std::shared_ptr<loggerBase> logger)
  {
    auto search = loggerBaseList_.find(name);
    if (search != loggerBaseList_.end()) {return false;}

    loggerBaseList_.insert({name, logger});

    return true;
  }

  // @brief Whether or not loggerBase with id is registered
  static bool hasLogger(const std::string & name)
  {
    auto search = loggerBaseList_.find(name);
    if (search != loggerBaseList_.end()) {
      return true;
    } else {
      return false;
    }
  }

  // @brief Reconfigures specified loggerBase with configurations
  static void configLogger(
    const std::string & name,
    const loggerConfig & config)
  {
    UNUSED(name);
    UNUSED(config);
  }

  // @brief Flush all the logers
  static void flushAll(void) {}

  static std::unordered_map<std::string, std::shared_ptr<loggerBase>>
  loggerBaseList_;
};

class consoleLogger : public loggerBase
{
public:
  // @brief Create logger
  static std::shared_ptr<loggerBase> create()
  {
    std::shared_ptr<loggerBase> handle = std::make_shared<consoleLogger>();
    return handle;
  }

  virtual void log_char(const char * s)
  {
    while (*s) {
      std::cout << *s++;
    }
  }

  virtual void log_char(const char s) {std::cout << s;}

  virtual void log_structure(const stream_any & s) {std::cout << s;}

  virtual bool log_begin() {return true;}

  virtual void log_end() {std::cout << std::endl;}

  virtual void flush() {}

private:
  static bool registered;
};

class fileLogger : public loggerBase
{
public:
private:
  static bool registered;
};

class netLogger : public loggerBase
{
public:
private:
  static bool registered;
};

}  // namespace diag

#endif  // OBJECT_ANALYTICS_NODE__UTIL__LOGGER_HPP_
