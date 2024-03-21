/**
* Copyright (c) 2024 Rainbow Robotics
* Use of this source code is governed by the Apache 2.0, see LICENSE
*/

#include <iostream>
#include <stdexcept>
#include <thread>
#include <utility>

#include "rbpodo/cobot.hpp"

using namespace std::chrono_literals;

namespace rb::podo {

ResponseCollector::ResponseCollector(unsigned int flag) {
  flag_ = flag;
}

void ResponseCollector::swap_responses(rb::podo::ResponseCollector& response_collector) {
  std::swap(static_cast<std::deque<Response>&>(*this), static_cast<std::deque<Response>&>(response_collector));
}

ResponseCollector ResponseCollector::type_filter(Response::Type type, bool remain) {
  const auto& pred = [=](const auto& r) {
    return r.type() == type;
  };
  return filter(pred, remain);
}

void ResponseCollector::add(Response&& response) {
  if (cb_) {
    cb_(response);
  }
  emplace_back(std::move(response));
}

void ResponseCollector::add(const Response& response) {
  if (cb_) {
    cb_(response);
  }
  push_back(response);
}

void ResponseCollector::set_callback(const std::function<void(const Response&)>& cb) {
  cb_ = cb;
}

void ResponseCollector::clear_callback() {
  cb_ = nullptr;
}

void ResponseCollector::clear() {
  std::deque<Response>::clear();
}

ResponseCollector ResponseCollector::info(bool remain) {
  return type_filter(Response::Type::Info, remain);
}

ResponseCollector ResponseCollector::warn(bool remain) {
  return type_filter(Response::Type::Warn, remain);
}

ResponseCollector ResponseCollector::error(bool remain) {
  return type_filter(Response::Type::Error, remain);
}

ResponseCollector ResponseCollector::ack(bool remain) {
  return type_filter(Response::Type::ACK, remain);
}

ResponseCollector ResponseCollector::category_filter(const std::string& category, bool remain) {
  const auto& pred = [=](const auto& r) {
    return r.category() == category;
  };
  return filter(pred, remain);
}

bool ResponseCollector::has_error() const {
  return std::find_if(begin(), end(), [=](const auto& r) { return r.type() == Response::Type::Error; }) != end();
}

std::ostream& operator<<(std::ostream& out, const Response& res) {
  out << R"({ "type": )" << to_string(res.type_) << R"(, "category": ")" << res.category_ << R"(", "msg": ")"
      << res.msg_ << R"(" })";
  return out;
}

std::ostream& operator<<(std::ostream& out, const ResponseCollector& response_collector) {
  out << "[";
  for (int i = 0; i < response_collector.size(); i++) {
    if (i != 0) {
      out << ", ";
    }
    out << response_collector[i];
  }
  out << "]";
  return out;
}

Response::Response(std::string raw) : raw_(std::move(raw)) {
  parse();
}

void Response::parse() {
  if (raw_ == kACKMessage) {
    type_ = Type::ACK;
    category_ = "";
    msg_ = "";
    return;
  }

  std::string tmp;
  int bracket_count = 0;

  auto it = raw_.begin();
  for (; it != raw_.end(); it++) {
    if (*it == '[')
      break;
    tmp.push_back(*it);
  }
  if (tmp == "info") {
    type_ = Type::Info;
  } else if (tmp == "warn") {
    type_ = Type::Warn;
  } else if (tmp == "error") {
    type_ = Type::Error;
  } else {
    type_ = Type::Unknown;
    return;
  }

  const auto& extract = [&]() {
    tmp.clear();
    bracket_count = 0;
    for (; it != raw_.end(); it++) {
      if (*it == '[') {
        it++;
        bracket_count++;
        break;
      }
    }
    for (; it != raw_.end(); it++) {
      if (*it == '[')
        bracket_count++;
      if (*it == ']')
        bracket_count--;
      if (bracket_count == 0)
        break;
      tmp.push_back(*it);
    }
  };

  extract();
  category_ = tmp;
  if (bracket_count != 0)
    return;

  extract();
  msg_ = tmp;
}

std::ostream& operator<<(std::ostream& out, const ReturnType& ret) {
  out << R"({ "type": )" << to_string(ret.type()) << R"(, "remain_time": )" << ret.remain_time() << " }";
  return out;
}

}  // namespace rb::podo