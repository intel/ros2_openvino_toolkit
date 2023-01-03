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
 * @brief A header file with declaration for BaseFilter Class
 * @file base_filter.hpp
 */
#ifndef OPENVINO_WRAPPER_LIB__INFERENCES__BASE_FILTER_HPP_
#define OPENVINO_WRAPPER_LIB__INFERENCES__BASE_FILTER_HPP_

#include <string>
#include <vector>
#include <utility>
#include <stack>
#include "openvino_wrapper_lib/inferences/base_inference.hpp"

namespace openvino_wrapper_lib
{

/**
 * @class BaseFilter
 * @brief Base class for result filter.
 */
class BaseFilter
{
public:
  BaseFilter();
  /**
   * @brief Initiate a result filter.
   */
  virtual void init() = 0;

  /**
   * @brief Get the filtered results' ROIs.
   * @return The filtered ROIs.
   */
  virtual std::vector<cv::Rect> getFilteredLocations() = 0;

  /**
   * @brief Check if the filter conditions is valid for filtering.
   * @param[in] Filter conditions.
   * @return true if some of the conditions are valid, otherwise false.
   */
  bool isValidFilterConditions(const std::string &);

  /**
   * @brief Accept the filter conditions for filtering.
   * @param[in] Filter conditions.
   */
  void acceptFilterConditions(const std::string &);

  /**
   * @brief Decide whether the input string is a relational operator or not.
   * @param[in] A string to be decided.
   * @return True if the input string is a relational operator, false if not.
   */
  bool isRelationOperator(const std::string &);

  /**
   * @brief Decide whether the input string is a logic operator or not.
   * @param[in] A string to be decided.
   * @return True if the input string is a logic operator, false if not.
   */
  bool isLogicOperator(const std::string &);

  /**
   * @brief Decide whether the an operator has a higher priority than anthor.
   * @param[in] The two operators.
   * @return True if the first operator has higher priority, false if not.
   */
  bool isPriorTo(const std::string &, const std::string &);

  /**
   * @brief Convert the input bool variable to a string type.
   * @param[in] A bool type to be converted.
   * @return A converted string result.
   */
  std::string boolToStr(bool);

  /**
   * @brief Convert the input string variable to a bool type.
   * @param[in] A string type to be converted.
   * @return A converted bool result.
   */
  bool strToBool(const std::string &);

  /**
   * @brief Get the filter conditions in the suffix order.
   * @return A vector with suffix-order filter conditions.
   */
  const std::vector<std::string> & getSuffixConditions() const;

  /**
   * @brief Do logic operation with the given bool values and the operator.
   * @param[in] A bool string, an logic operator, the other bool string.
   * @return The logic operation result.
   */
  bool logicOperation(const std::string &, const std::string &, const std::string &);

  /**
   * @brief Compare two strings with a given relational operator.
   * @param[in] A string, an relational operator, the other string.
   * @return True if valid, false if not.
   */
  static bool stringCompare(const std::string &, const std::string &, const std::string &);

  /**
   * @brief Compare two floats with a given relational operator.
   * @param[in] A float number, an relational operator, the other float number.
   * @return True if valid, false if not.
   */
  static bool floatCompare(float, const std::string &, float);

  /**
   * @brief Convert a string into a float number.
   * @param[in] A string to be converted.
   * @return The converted float number, 0 if string is invalid.
   */
  static float stringToFloat(const std::string &);

  /**
   * @brief A macro to decide whether a given result satisfies the filter condition.
   * @param[in] A key to function mapping, a given result.
   * @return True if valid, false if not.
   */
  #define ISVALIDRESULT(key_to_function, result) \
  { \
    std::vector<std::string> suffix_conditons = getSuffixConditions(); \
    std::stack<std::string> result_stack; \
    for (auto elem : suffix_conditons) { \
      if (!isRelationOperator(elem) && !isLogicOperator(elem)) { \
        result_stack.push(elem); \
      } else { \
        try { \
          std::string str1 = result_stack.top(); \
          result_stack.pop(); \
          std::string str2 = result_stack.top(); \
          result_stack.pop(); \
          if (key_to_function.count(str2)) { \
            result_stack.push(boolToStr(key_to_function[str2](result, elem, str1))); \
          } else { \
            result_stack.push(boolToStr(logicOperation(str1, elem, str2))); \
          } \
        } \
        catch (...) { \
          slog::err << "Invalid filter conditions format!" << slog::endl; \
        } \
      } \
    } \
    if (result_stack.empty()) { \
      return true; \
    } \
    return strToBool(result_stack.top()); \
  }

private:
  /**
   * @brief Parse the filter conditions and stores it into a vector.
   * @param[in] A string form filter conditions.
   * @return The vector form filter conditions.
   */
  std::vector<std::string> split(const std::string & filter_conditions);

  /**
   * @brief Convert the infix expression into suffix expression.
   * @param[in] The infix form filter conditions.
   */
  void infixToSuffix(std::vector<std::string>&infix_conditions);

  /**
   * @brief Strip the extra space in a string.
   * @param[in] A string to be striped.
   * @return The striped string.
   */
  std::string strip(const std::string & str);

  std::string striped_conditions_ = "";
  std::vector<std::string> suffix_conditons_;
  std::vector<std::string> relation_operators_ = {"==", "!=", "<=", ">=", "<", ">"};
  std::vector<std::string> logic_operators_ = {"&&", "||"};
};
}  // namespace openvino_wrapper_lib

#endif  // OPENVINO_WRAPPER_LIB__INFERENCES__BASE_FILTER_HPP_
