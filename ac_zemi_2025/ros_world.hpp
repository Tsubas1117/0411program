#error "お気持ちコード。動かないのでちゃんと実装してね"

#pragma once

#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

namespace ac_zemi_2025::ros_world::impl {
	using Eigen::Matrix2Xd;

	struct RosWorld final : rclcpp::Node {
		Matrix2Xd laserscan{};
		rclcpp::Publisher<void>::SharePtr robot_speed_pub;
		rclcpp::Subscriber<sensor_msgs::msg::LaserScan>::SharePtr lidar_sub;

		Ros2Node():
			rclcpp::Node()
			, robot_speed_pub{}
			, lidar_sub{}
		{}

		auto update(const auto& robot_speed) const noexcept -> Matrix2Xd {
			const auto robot_speed_msg = robot_speed;
			this->robot_speed_pub->publish(robot_speed_msg);
			return this->laserscan;
		}

		void laserscan_callback(const sensor_msgs::msg::laserScan::ConstSharePtr msg) noexcept {
			this->laserscan = msg;
		}
	};
}

namespace ac_zemi_2025::ros_world {
	using impl::RosWorld;
}