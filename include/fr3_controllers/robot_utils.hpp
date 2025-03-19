#pragma once

#include <tinyxml2.h>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace robot_utils {
using namespace std::chrono_literals;

inline std::string getRobotNameFromDescription(const std::string& robot_description,
                                               const rclcpp::Logger& logger) {
  std::string robot_name;
  tinyxml2::XMLDocument doc;

  if (doc.Parse(robot_description.c_str()) != tinyxml2::XML_SUCCESS) {
    RCLCPP_ERROR(logger, "Failed to parse robot_description.");
    return "";
  }

  tinyxml2::XMLElement* robot_xml = doc.FirstChildElement("robot");
  if (robot_xml) {
    robot_name = robot_xml->Attribute("name");
    if (robot_name.empty()) {
      RCLCPP_ERROR(logger, "Failed to get robot name from XML.");
      return "";
    }
    RCLCPP_INFO(logger, "Extracted Robot Name: %s", robot_name.c_str());
  } else {
    RCLCPP_ERROR(logger, "Robot element not found in XML.");
  }
  return robot_name;
}

const auto time_out = 1000ms;

}  // namespace robot_utils
