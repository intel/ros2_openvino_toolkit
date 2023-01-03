// Copyright (c) 2018-2022 Intel Corporation
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

/**
 * @brief An implementation file with implementation for BaseFilter Class
 * @file base_filter.cpp
 */

#include "openvino_wrapper_lib/inferences/base_filter.hpp"
#include <stack>
#include <string>
#include <vector>

openvino_wrapper_lib::BaseFilter::BaseFilter() {}

bool openvino_wrapper_lib::BaseFilter::isValidFilterConditions(
  const std::string & filter_conditions)
{
  return strip(filter_conditions) != "";
}

void openvino_wrapper_lib::BaseFilter::acceptFilterConditions(
  const std::string & filter_conditions)
{
  striped_conditions_ = strip(filter_conditions);
  std::vector<std::string> infix_conditions = split(striped_conditions_);
  infixToSuffix(infix_conditions);
}

bool openvino_wrapper_lib::BaseFilter::isRelationOperator(const std::string & str)
{
  if (std::find(relation_operators_.begin(), relation_operators_.end(), str) !=
    relation_operators_.end())
  {
    return true;
  }
  return false;
}

bool openvino_wrapper_lib::BaseFilter::isLogicOperator(const std::string & str)
{
  if (std::find(logic_operators_.begin(), logic_operators_.end(), str) !=
    logic_operators_.end())
  {
    return true;
  }
  return false;
}

bool openvino_wrapper_lib::BaseFilter::isPriorTo(
  const std::string & operator1, const std::string & operator2)
{
  if (isRelationOperator(operator1) && isLogicOperator(operator2)) {
    return true;
  }
  return false;
}

std::string openvino_wrapper_lib::BaseFilter::boolToStr(bool value)
{
  if (value) {return "true";}
  return "false";
}

bool openvino_wrapper_lib::BaseFilter::strToBool(const std::string & value)
{
  if (!value.compare("true")) {return true;} else if (!value.compare("false")) {
    return false;
  } else {
    slog::err << "Invalid string: " << value << " for bool conversion!" << slog::endl;
  }
  return false;
}

const std::vector<std::string> &
openvino_wrapper_lib::BaseFilter::getSuffixConditions() const
{
  return suffix_conditons_;
}

bool openvino_wrapper_lib::BaseFilter::logicOperation(
  const std::string & logic1, const std::string & op, const std::string & logic2)
{
  if (!op.compare("&&")) {
    return strToBool(logic1) && strToBool(logic2);
  } else if (!op.compare("||")) {
    return strToBool(logic1) || strToBool(logic2);
  } else {
    slog::err << "Invalid operator: " << op << " for logic operation!" << slog::endl;
    return false;
  }
}

bool openvino_wrapper_lib::BaseFilter::stringCompare(
  const std::string & candidate, const std::string & op, const std::string & target)
{
  if (!op.compare("==")) {
    return !target.compare(candidate);
  } else if (!op.compare("!=")) {
    return target.compare(candidate);
  } else {
    slog::err << "Invalid operator " << op << " for label comparsion" << slog::endl;
    return false;
  }
}

bool openvino_wrapper_lib::BaseFilter::floatCompare(
  float candidate, const std::string & op, float target)
{
  if (!op.compare("<=")) {
    return candidate <= target;
  } else if (!op.compare(">=")) {
    return candidate >= target;
  } else if (!op.compare("<")) {
    return candidate < target;
  } else if (!op.compare(">")) {
    return candidate > target;
  } else {
    slog::err << "Invalid operator " << op << " for confidence comparsion" << slog::endl;
    return false;
  }
}

float openvino_wrapper_lib::BaseFilter::stringToFloat(const std::string & candidate)
{
  float result = 0;
  try {
    result = std::stof(candidate);
  } catch (...) {
    slog::err << "Failed when converting string " << candidate << " to float" << slog::endl;
  }
  return result;
}

std::vector<std::string> openvino_wrapper_lib::BaseFilter::split(
  const std::string & filter_conditions)
{
  std::vector<std::string> seperators;
  seperators.insert(seperators.end(), relation_operators_.begin(), relation_operators_.end());
  seperators.insert(seperators.end(), logic_operators_.begin(), logic_operators_.end());
  seperators.push_back("(");
  seperators.push_back(")");
  std::vector<std::string> infix_conditions;
  int last_pos = 0, pos = 0;
  for (pos = 0; pos < filter_conditions.length(); pos++) {
    for (auto sep : seperators) {
      if (!sep.compare(filter_conditions.substr(pos, sep.length()))) {
        std::string elem = filter_conditions.substr(last_pos, pos - last_pos);
        if (!elem.empty()) {
          infix_conditions.push_back(elem);
        }
        elem = filter_conditions.substr(pos, sep.length());
        infix_conditions.push_back(elem);
        last_pos = pos + sep.length();
        pos = last_pos - 1;
        break;
      }
    }
  }
  if (last_pos != pos) {
    infix_conditions.push_back(filter_conditions.substr(last_pos, pos - last_pos));
  }
  return infix_conditions;
}

void openvino_wrapper_lib::BaseFilter::infixToSuffix(
  std::vector<std::string> & infix_conditions)
{
  std::stack<std::string> operator_stack;
  for (auto elem : infix_conditions) {
    if (!elem.compare("(")) {
      operator_stack.push(elem);
    } else if (!elem.compare(")")) {
      while (!operator_stack.empty() && operator_stack.top().compare("(")) {
        suffix_conditons_.push_back(operator_stack.top());
        operator_stack.pop();
      }
      if (operator_stack.empty()) {
        slog::err << "Brackets mismatch in filter_conditions!" << slog::endl;
      }
      operator_stack.pop();
    } else if (isRelationOperator(elem) || isLogicOperator(elem)) {
      while (!operator_stack.empty() && isPriorTo(operator_stack.top(), elem)) {
        suffix_conditons_.push_back(operator_stack.top());
        operator_stack.pop();
      }
      operator_stack.push(elem);
    } else {
      suffix_conditons_.push_back(elem);
    }
  }
  while (!operator_stack.empty()) {
    suffix_conditons_.push_back(operator_stack.top());
    operator_stack.pop();
  }
}

std::string openvino_wrapper_lib::BaseFilter::strip(const std::string & str)
{
  std::string stripped_string = "";
  for (auto character : str) {
    if (character != ' ') {
      stripped_string += character;
    }
  }
  return stripped_string;
}
