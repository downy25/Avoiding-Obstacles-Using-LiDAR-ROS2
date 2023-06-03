// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#ifndef CAMERA_HPP_
#define CAMERA_HPP_

#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/rclcpp.hpp"

std::string mat_type2encoding(int mat_type);
//void convert_frame_to_message(const cv::Mat& frame, size_t frame_id, sensor_msgs::msg::Image& msg);
void convert_frame_to_message(const cv::Mat& frame, sensor_msgs::msg::Image& msg);
int encoding2mat_type(const std::string& encoding);

#endif  // CAMERA_HPP_

