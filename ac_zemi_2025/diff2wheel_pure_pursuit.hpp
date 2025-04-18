

#pragma once

#include <utility>
#include <numbers>
#include <optional>
#include <ostream>

#include <eigen3/Eigen/Dense>

#include "utility.hpp"

namespace ac_zemi_2025::diff2wheel_pure_pursuit::impl {
	using std::numbers::pi;

	using Eigen::Matrix2Xd;
	using Eigen::Vector2d;

	using utility::Pose2d;

	/// @brief [-pi, pi)を返すfmod。
	inline constexpr auto modpi(const double x) noexcept -> double {
		double result = std::fmod(x, 2 * pi);
		if (result < -pi)
		{
			result += 2 * pi;
		}
		else if (result >= pi)
		{
			result -= 2 * pi;
		}
		return result;
	}
	
	struct Config final {
		// 以下で示される扇状の範囲にあるマイルストーンを選ぶ
		std::pair<double, double> accept_distance;
		double accept_theta;
		// 速度の方向と位置ズレのどちらを重要視するか
		double param;
		// 前進速度
		double speed;
	};

	// fromからtoへのベクトルと、その偏角と機体の姿勢角の差角を返す
	inline auto calc_diff(const Pose2d& from, const Vector2d& to) noexcept -> std::pair<Vector2d, double> {
		const Vector2d diff = to - from.xy;
		const double diff_angle = std::abs(modpi(std::atan2(diff(1), diff(2)) - from.th));
		return std::pair{diff, diff_angle};
	}

	// 差動二輪の速度指令
	struct Diff2wheelSpeed final {
		double ahead;
		double rotate;
	};

	// 差動二輪のpure_pursuit
	inline auto pure_pursuit(const Config& conf, const Matrix2Xd& route, const i64 current_index, const Pose2d& current_pose) -> std::optional<Diff2wheelSpeed> {
		// current_index + 1以降の要素を舐め、良いマイルストーンの添え字を取得する
		// nearist_indexには、最初は"無効"を意味する値が入っている
		std::optional<i64> best_index{std::nullopt};
		std::optional<i64> better_index{std::nullopt};
		for(i64 i = current_index + 1; i < route.rows(); i++) {
			// 十分に近くにあり、またロボットの正面+-accept_thetaに入ってる点を探す
			const auto [diff, diff_angle] = calc_diff(current_pose, route.col(i));
			const double distance = diff.norm();
			if(conf.accept_distance.first < distance && distance < conf.accept_distance.second) {
				better_index = i;
				if(diff_angle < conf.accept_theta) {
					best_index = i;
					break;
				}
			}
		}

		// 近くにマイルストーンがなければ諦め
		if(!better_index.has_value()) {
			return std::nullopt;
		}
		
		const auto [diff, diff_angle] = calc_diff(current_pose, best_index.has_value() ? route.col(*best_index) : route.col(*better_index));

		return std::optional<Diff2wheelSpeed>{std::in_place, Diff2wheelSpeed{conf.speed, diff_angle}};
	}
}