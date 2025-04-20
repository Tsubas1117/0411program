#pragma once

#include <optional>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "utility.hpp"
#include "geometry.hpp"

namespace ac_zemi_2025::diff2_carrot_pursuit::impl {
	using Eigen::Vector2d;

	using geometry::Pose2d;

	struct Diff2wheelSpeed final {
		double ahead;
		double rotate;

		constexpr auto to_pose2d_velocity(const Pose2d& pose) const noexcept -> Pose2d {
			return Pose2d{pose.homogeneus_transform() * Vector2d{ahead, 0.0}, rotate};
		}
	};

	struct Diff2CarrotPursuit final {
		/// @todo 実装。configurableな項目を入れておく

		auto update(const std::vector<Vector2d>& route, const Pose2d& current_pose, i64& closest_ever_idx) const noexcept -> std::optional<Diff2wheelSpeed> {
			/// @todo 実装
			(void) route;
			(void) current_pose;
			(void) closest_ever_idx;
			return {};
		}
	};
}

namespace ac_zemi_2025::diff2_carrot_pursuit {
	using impl::Diff2wheelSpeed;
	using impl::Diff2CarrotPursuit;
}