/**
 * @file icp_p2l.hpp
 * @author tsubasa 23S, tamaki 21T
 * @brief Iterative Closest Point(ICP)による2D LiDAR + オドメトリを用いた自己位置推定コード。マップ情報を線分で持っているので軽量高速。
 * @version 0.1
 * @date 2025-04-17
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include <cmath>
#include <cstdint>
#include <concepts>

#include <eigen3/Eigen/Dense>

namespace crs_lib::icp_on_svd::main::impl {
	using i64 = std::int64_t;
	using std::numbers::pi;
	using Eigen::Matrix2Xd;
	using Eigen::Matrix2d;
	using Eigen::Vector2d;
	using Eigen::Rotation2Dd;
	using Eigen::Translation2d;
	using Eigen::Transform;
	using Eigen::Isometry;
	using Eigen::JacobiSVD;

	struct Pose2d final {
		double x;
		double y;
		double th;

		constexpr auto operator-() const noexcept -> Pose2d {
			return Pose2d{this->x, this->y, this->th};
		}
	};

	// 端点2つによる線分表現
	struct Line2d final {
		Vector2d p1;
		Vector2d p2;
	};

	inline constexpr auto rot90(const Vector2d& v) -> Vector2d {
		return Vector2d{-v.y(), v.x()};
	}

	inline constexpr auto distance_p2l(const Vector2d& part, const Line2d& whole) -> std::pair<Vector2d, double> {
		const auto segment = whole.p2 - whole.p1;
		const auto from_p1 = part - whole.p1;
		const auto to_p2 = whole.p2 - part;

		if(from_p1.dot(segment) <= 0.0) {
			return std::pair{whole.p1, from_p1.norm()};
		}
		else if(to_p2.dot(segment) <= 0.0) {
			return std::pair{whole.p2, to_p2.norm()};
		}
		else {
			const auto point = from_p1.dot(segment) / segment.squaredNorm() * segment + whole.p1;
			return std::pair{point, (part - point).norm()};
		}
	}

	inline constexpr auto distance_l2l(const Line2d& part, const Line2d& whole) -> double {
		/// @todo
		/// partを含む直線にwholeの端点からwholeに直交する向きにおろした2直線とpartを含む直線との交点p, qを考え、
		/// pからqの範囲内では「wholeを含む直線との距離」を、範囲外では「wholeの端点との距離」をpartに沿って線積分する
		/// 直線との距離の部分は1次関数の積分なので簡単
		/// 問題は端点との距離のほう。積分すると\int_a^b \sqrt{x^2 + h^2} dxとなる。
		/// ここでxはwholeの端点からpartに落とした垂線の足を原点にとったpartに沿う数直線で、a, bはpartの端点、pかqのx軸上の位置。
		/// 積分は計算すると\frac{h \sqrt{x^2 + h^2}}{2} + \frac{h^2}{4} \log{\frac{\sqrt{x^2 + h^2} + x}{\sqrt{x^2 + h^2} - x}}
		/// となる、はず
		(void) part;
		(void) whole;
		return 0.0;  // 仮
	}
	
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

	/// @brief 同次変換行列を作成
	inline auto homogeneus_transform(const Pose2d& pose) -> Transform<double, 2, Isometry> {
		auto trans = Transform<double, 2, Isometry>::Identity();
		trans.rotate(Rotation2Dd{pose.th}).pretranslate(Vector2d{pose.x, pose.y});
		return trans;
	}

	/// @brief グローバル座標系でのマップ線分から、ローカル座標系での可視なマップ線分を計算
	/// マップ線分は重複せず、またそれぞれの線分は交点を端点以外に持たないようにしておく
	/// -> マップ線分は各頂点がグローバル座標を持つような、無向グラフ構造で持っておくほうがいい？
	///    -> そもそもこの処理は頂点座標付きグラフ構造のメンバ関数にするべきかも
	/// 計算量はO(頂点数)、平均計算量はずっと小さくて済みそう
	inline auto make_visible_lines(const auto& map, const Pose2d& pose) -> std::vector<Line2d> {
		// 各端点を同次変換し、極座標にしてthetaでソート
		// 末尾には先頭の点を入れておく
		const auto map_local = map.to_local_rtheta(pose);

		// 可視な線分を見つけていく
		// 途中途中、thetaの増減で向きつけしたDAGを考えることがある
		std::vector<Line2d> ret{};
		std::vector<Line2d> connected_vertices{};
		/// @todo ret.reserve(??);
		/// @todo connected_vertices.reserve(??);
		auto iter = map_local.vertices.begin();
		while(iter != map.vertices.end()) {
			// DAGの隣接頂点の中で最も原点に近いものを選んでいく
			auto v = map_local.get_vertex(iter->idx);
			while(true) {
				connected_vertices.emplace_back(v);
				if(const auto next_opt = map_local.next(v)) {
					v = *next_opt;
				}
				else break;
			}

			// 繋がってる点をretに追加していく
			for(int i = 0; i < int(connected_vertices.size()) - 1; ++i) {
				ret.emplace_back(map_local.vertices[i].xy, map_local.vertices[i + 1].xy);
			}

			iter = map_local.next_by_r(iter);
		}

		return ret;
	}

	/// @brief センサ由来の点群をマップ由来の線分にfittingする ICP on SVD
	inline auto icp_p2l(const std::vector<Line2d>& map_local_lines, const Matrix2Xd& laserscan_local_xy) -> Pose2d {
		std::vector<Line2d> map = map_local_lines;

		// laserscan_local_xyを、重心を原点とする座標系に直す
		auto laserscan = laserscan_local_xy;
		const auto laserscan_mean = laserscan_local_xy.rowwise().mean();
		static_assert(decltype(laserscan_mean)::RowsAtCompileTime == 2);
		laserscan.colwise() -= laserscan_mean;  // これ以降laserscanは変更されない

		// laserscanをclosest_pointsに合わせる変換を計算し、その変換を合成、mapに適用していく
		auto closest_points = Matrix2Xd{laserscan_local_xy.rows(), laserscan_local_xy.cols()};
		auto total_transform = Transform<double, 2, Isometry>::Identity();
		for(i64 iloop = 0; iloop < 50; iloop++) {  // とりあえず50回
			// laserscanの各点の最近接点を求める
			for(i64 ip = 0; ip < i64(laserscan_local_xy.cols()); ++ip) {
				Vector2d closest_point{};
				double closest_distance = std::numeric_limits<double>::infinity();
				for (i64 iq = 0; iq < i64(map.size()); iq++) {
					const auto [point, distance] = distance_p2l(laserscan_local_xy.col(ip), map[iq]);
					if(distance < closest_distance) {
						closest_distance = distance;
						closest_point = point;
					}
				}

				closest_points.col(ip) = closest_point;
			}

			// closest_pointsを、重心を原点とする座標系に直す
			const auto closest_points_mean = closest_points.rowwise().mean();
			static_assert(decltype(closest_points_mean)::RowsAtCompileTime == 2);
			closest_points.colwise() -= closest_points_mean;

			// SVDで最適な剛体変換を求める
			const auto cross_covariance = (laserscan) * closest_points.transpose();
			static_assert(decltype(cross_covariance)::RowsAtCompileTime == 2 && decltype(cross_covariance)::ColsAtCompileTime == 2);

			// SVD分解
			JacobiSVD<Matrix2d> svd(cross_covariance, Eigen::ComputeThinU | Eigen::ComputeThinV);
			const auto u = svd.matrixU();
			const auto v = svd.matrixV();

			// 最適な回転移動を計算(if文内処理の詳細はjres.124.028.pdfなどを参照)
			auto optimized_rotation = (v * u.transpose()).eval();
			if(optimized_rotation.determinant() < 0.0) {
				optimized_rotation(1, 1) = -optimized_rotation(1, 1);
			}

			// 最適な平行移動を計算
			auto optimized_translation = closest_points_mean - optimized_rotation * laserscan_mean;
			static_assert(decltype(optimized_translation)::RowsAtCompileTime == 2 && decltype(optimized_translation)::ColsAtCompileTime == 1);

			// 最適な剛体変換に合わせ、それを蓄積する
			auto optimized_transform = Transform<double, 2, Isometry>::Identity();
			optimized_transform.rotate(optimized_rotation).pretranslate(optimized_translation);

			total_transform = optimized_transform * total_transform;

			// 計算が少なくて済むので、mapのほうを動かしてやる。適用すべきは逆変換である事に注意
			const auto inv_transform = optimized_transform.inverse();
			for(auto& mapi : map) {
				mapi.p1 = inv_transform * mapi.p1;
				mapi.p2 = inv_transform * mapi.p2;
			}
		}

		// Pose2Dにして返す
		const auto translation = total_transform.translation();
		const auto rotation = total_transform.rotation();
		return Pose2d{translation(0), translation(1), std::atan2(rotation(1, 0), rotation(0, 0))};
	}
}
