/**
* Copyright (c) 2024 Rainbow Robotics
* Use of this source code is governed by the Apache 2.0, see LICENSE
*/

#pragma once

#include <algorithm>
#include <chrono>
#include <optional>
#include <string>

namespace rb::podo {

static inline const unsigned int kCommandPort = 5000;
static inline const unsigned int kDataPort = 5001;

static inline const std::string kACKMessage{"The command was executed"};
static inline const std::string kRequestDataCommand{"reqdata"};

inline void ltrim(std::string& s, bool newline = false) {
  const auto& check = [=](unsigned char ch) {
    return !std::isspace(ch) && (!newline || (ch != '\n' && ch != '\r'));
  };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), check));
}

inline void rtrim(std::string& s, bool newline = false) {
  const auto& check = [=](unsigned char ch) {
    return !std::isspace(ch) && (!newline || (ch != '\n' && ch != '\r'));
  };
  s.erase(std::find_if(s.rbegin(), s.rend(), check).base(), s.end());
}

inline void trim(std::string& s, bool newline = false) {
  ltrim(s, newline);
  rtrim(s, newline);
}

class Timeout {
 public:
  explicit Timeout(double duration) : duration_(duration) { stime_ = std::chrono::steady_clock::now(); }

  [[nodiscard]] bool is_elapsed() const {
    if (duration_ < 0)
      return false;

    return remain_time() <= std::numeric_limits<double>::epsilon();
  }

  [[nodiscard]] double remain_time() const {
    if (duration_ < 0) {
      return duration_;
    }

    auto etime = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(etime - stime_).count();
    double t = duration_ - ((double)duration / 1.e9);
    return t < 0. ? 0. : t;
  }

 private:
  double duration_;
  std::chrono::steady_clock::time_point stime_;
};

template <typename T, typename Enable = void>
struct is_optional : std::false_type {};

template <typename T>
struct is_optional<std::optional<T>> : std::true_type {};

struct Timer {
  Timer() { stime_ = std::chrono::steady_clock::now(); }

  double get_duration() {
    auto etime = std::chrono::steady_clock::now();
    double d = (double)std::chrono::duration_cast<std::chrono::nanoseconds>(etime - stime_).count();
    d /= 1e9;
    return d;
  }

 private:
  std::chrono::steady_clock::time_point stime_;
};

}  // namespace rb::podo