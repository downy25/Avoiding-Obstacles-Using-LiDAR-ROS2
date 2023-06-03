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

//cam2image.cpp
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <sstream>
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "camera_ros2/camera.hpp"

/// Convert an OpenCV matrix encoding type to a string format recognized by sensor_msgs::Image.
/**
 * \param[in] mat_type The OpenCV encoding type.
 * \return A string representing the encoding type.
 */
std::string mat_type2encoding(int mat_type)
{
    switch (mat_type) {
    case CV_8UC1:
        return "mono8";
    case CV_8UC3:
        return "bgr8";
    case CV_16SC1:
        return "mono16";
    case CV_8UC4:
        return "rgba8";
    default:
        throw std::runtime_error("Unsupported encoding type");
    }
}

/// Convert an OpenCV matrix (cv::Mat) to a ROS Image message.
/**
 * \param[in] frame The OpenCV matrix/image to convert.
 * \param[in] frame_id ID for the ROS message.
 * \param[out] Allocated shared pointer for the ROS Image message.
 */
//void convert_frame_to_message(const cv::Mat& frame, size_t frame_id, sensor_msgs::msg::Image& msg)
void convert_frame_to_message(const cv::Mat& frame, sensor_msgs::msg::Image& msg)
{
    // copy cv information into ros message
    msg.height = frame.rows;
    msg.width = frame.cols;
    msg.encoding = mat_type2encoding(frame.type());
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    size_t size = frame.step * frame.rows;
    msg.data.resize(size);
    memcpy(&msg.data[0], frame.data, size);
    //msg.header.frame_id = std::to_string(frame_id);
    msg.header.frame_id = "camera";
}

/// Convert a sensor_msgs::Image encoding type (stored as a string) to an OpenCV encoding type.
/**
 * \param[in] encoding A string representing the encoding type.
 * \return The OpenCV encoding type.
 */
int encoding2mat_type(const std::string& encoding)
{
    if (encoding == "mono8") {
        return CV_8UC1;
    }
    else if (encoding == "bgr8") {
        return CV_8UC3;
    }
    else if (encoding == "mono16") {
        return CV_16SC1;
    }
    else if (encoding == "rgba8") {
        return CV_8UC4;
    }
    else if (encoding == "bgra8") {
        return CV_8UC4;
    }
    else if (encoding == "32FC1") {
        return CV_32FC1;
    }
    else if (encoding == "rgb8") {
        return CV_8UC3;
    }
    else {
        throw std::runtime_error("Unsupported encoding type");
    }
}

