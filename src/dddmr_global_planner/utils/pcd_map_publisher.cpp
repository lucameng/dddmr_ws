/*
* BSD 3-Clause License

* Copyright (c) 2024, DDDMobileRobot

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:

* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.

* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.

* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.

* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "rclcpp/rclcpp.hpp"

#include "pcl/common/transforms.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <filesystem>

#include <sensor_msgs/msg/point_cloud2.hpp>

class PcdMapPublisher : public rclcpp::Node {
public:
  PcdMapPublisher() : Node("pcd_map_publisher") {
    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "mapcloud",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "mapground",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    this->declare_parameter("map_dir", rclcpp::ParameterValue(""));
    this->declare_parameter("ground_dir", rclcpp::ParameterValue(""));
    this->declare_parameter("global_frame", rclcpp::ParameterValue("map"));
    this->declare_parameter("map_rotate_around_x", rclcpp::ParameterValue(0.0));
    this->declare_parameter("map_rotate_around_y", rclcpp::ParameterValue(0.0));
    this->declare_parameter("map_rotate_around_z", rclcpp::ParameterValue(0.0));
    this->declare_parameter("map_translate_x", rclcpp::ParameterValue(0.0));
    this->declare_parameter("map_translate_y", rclcpp::ParameterValue(0.0));
    this->declare_parameter("map_translate_z", rclcpp::ParameterValue(0.0));
    this->declare_parameter("ground_rotate_around_x",
                            rclcpp::ParameterValue(0.0));
    this->declare_parameter("ground_rotate_around_y",
                            rclcpp::ParameterValue(0.0));
    this->declare_parameter("ground_rotate_around_z",
                            rclcpp::ParameterValue(0.0));
    this->declare_parameter("ground_translate_x", rclcpp::ParameterValue(0.0));
    this->declare_parameter("ground_translate_y", rclcpp::ParameterValue(0.0));
    this->declare_parameter("ground_translate_z", rclcpp::ParameterValue(0.0));

    this->get_parameter("map_dir", map_dir_);
    this->get_parameter("ground_dir", ground_dir_);
    this->get_parameter("global_frame", global_frame_);

    if (!loadAndPublish(map_dir_, "mapcloud", *map_pub_,
                        buildTransform("map"))) {
      rclcpp::shutdown();
      return;
    }
    if (!loadAndPublish(ground_dir_, "mapground", *ground_pub_,
                        buildTransform("ground"))) {
      rclcpp::shutdown();
      return;
    }

    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&PcdMapPublisher::publishCachedClouds, this));
  }

private:
  Eigen::Affine3f buildTransform(const std::string &prefix) {
    double rotate_x = 0.0;
    double rotate_y = 0.0;
    double rotate_z = 0.0;
    double translate_x = 0.0;
    double translate_y = 0.0;
    double translate_z = 0.0;

    this->get_parameter(prefix + "_rotate_around_x", rotate_x);
    this->get_parameter(prefix + "_rotate_around_y", rotate_y);
    this->get_parameter(prefix + "_rotate_around_z", rotate_z);
    this->get_parameter(prefix + "_translate_x", translate_x);
    this->get_parameter(prefix + "_translate_y", translate_y);
    this->get_parameter(prefix + "_translate_z", translate_z);

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << translate_x, translate_y, translate_z;

    if (std::fabs(rotate_x) > 0.01) {
      transform.rotate(Eigen::AngleAxisf(rotate_x, Eigen::Vector3f::UnitX()));
    }
    if (std::fabs(rotate_y) > 0.01) {
      transform.rotate(Eigen::AngleAxisf(rotate_y, Eigen::Vector3f::UnitY()));
    }
    if (std::fabs(rotate_z) > 0.01) {
      transform.rotate(Eigen::AngleAxisf(rotate_z, Eigen::Vector3f::UnitZ()));
    }

    return transform;
  }

  bool
  loadAndPublish(const std::string &pcd_path, const std::string &topic_name,
                 rclcpp::Publisher<sensor_msgs::msg::PointCloud2> &publisher,
                 const Eigen::Affine3f &transform) {
    if (pcd_path.empty() || !std::filesystem::exists(pcd_path)) {
      RCLCPP_ERROR(this->get_logger(), "%s file does not exist: %s",
                   topic_name.c_str(), pcd_path.c_str());
      return false;
    }

    pcl::PointCloud<pcl::PointXYZI> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path, cloud) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load %s from: %s",
                   topic_name.c_str(), pcd_path.c_str());
      return false;
    }

    pcl::transformPointCloud(cloud, cloud, transform);

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = global_frame_;
    msg.header.stamp = this->get_clock()->now();
    publisher.publish(msg);

    if (topic_name == "mapcloud") {
      map_msg_ = msg;
    } else {
      ground_msg_ = msg;
    }

    RCLCPP_INFO(this->get_logger(),
                "Published %s from %s with %zu points in frame %s",
                topic_name.c_str(), pcd_path.c_str(), cloud.points.size(),
                global_frame_.c_str());
    return true;
  }

  void publishCachedClouds() {
    map_msg_.header.stamp = this->get_clock()->now();
    ground_msg_.header.stamp = this->get_clock()->now();
    map_pub_->publish(map_msg_);
    ground_pub_->publish(ground_msg_);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::PointCloud2 map_msg_;
  sensor_msgs::msg::PointCloud2 ground_msg_;
  std::string map_dir_;
  std::string ground_dir_;
  std::string global_frame_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PcdMapPublisher>();
  if (rclcpp::ok()) {
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
