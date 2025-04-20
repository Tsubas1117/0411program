#pragma once

#include <eigen3/Eigen/Dense>

#include "geometry.hpp"
#include "diff2_pure_pursuit.hpp"

namespace ac_zemi_2025::simulator::impl {
	using Eigen::Matrix2Xd;

	using geometry::Pose2d;
	using diff2_carrot_pursuit::Diff2wheelSpeed;

	/// @todo 実装
	struct SimulatorConstant final {};
	struct SimulatorState final {};

	struct SensorOutput final {
		Matrix2Xd laserscan;
		Pose2d odometry;
	};

	inline auto sim_update(const SimulatorConstant& cons, SimulatorState& state, const Diff2wheelSpeed speed) -> SensorOutput {
		/// @todo 実装
		(void) cons;
		(void) state;
		(void) speed;
		return {};
	}
}

namespace ac_zemi_2025::simulator {
	using impl::SimulatorConstant;
	using impl::SimulatorState;
	using impl::SensorOutput;
	using impl::sim_update;
}