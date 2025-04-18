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
#include <utility>
#include <vector>
#include <limits>

#include <eigen3/Eigen/Dense>

#include "utility.hpp"

namespace ac_zemi_2025::icp_on_svd::impl {
	using Eigen::Matrix2Xd;
	using Eigen::Matrix2d;
	using Eigen::Vector2d;
	using Eigen::Transform;
	using Eigen::Isometry;
	using Eigen::JacobiSVD;

	using utility::Line2d;
	using utility::Pose2d;
	using utility::distance_l2l;
	using utility::distance_p2l;

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

	/// @brief 点群を線分群にfittingする ICP on SVD
	/// 線分数は点数に比べ十分少ないとする
	inline auto icp_p2l(Matrix2Xd from, std::vector<Line2d> to) -> Pose2d {

		// fromを、重心を原点とする座標系に直す
		const auto from_mean = from.rowwise().mean();
		static_assert(decltype(from_mean)::RowsAtCompileTime == 2);
		from.colwise() -= from_mean;  // これ以降fromは変更されない

		// fromをclosest_pointsに合わせる変換を計算し、その変換を合成、toに適用していく
		auto closest_points = Matrix2Xd{from.rows(), from.cols()};
		auto total_transform = Transform<double, 2, Isometry>::Identity();
		for(i64 iloop = 0; iloop < 50; iloop++) {  // とりあえず50回
			// fromの各点の最近接点を求める
			for(i64 ip = 0; ip < i64(from.cols()); ++ip) {
				Vector2d closest_point{};
				double closest_distance = std::numeric_limits<double>::infinity();
				for (i64 iq = 0; iq < i64(to.size()); iq++) {
					const auto [point, distance] = distance_p2l(from.col(ip), to[iq]);
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
			const auto cross_covariance = (from) * closest_points.transpose();
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
			auto optimized_translation = closest_points_mean - optimized_rotation * from_mean;
			static_assert(decltype(optimized_translation)::RowsAtCompileTime == 2 && decltype(optimized_translation)::ColsAtCompileTime == 1);

			// 最適な剛体変換に合わせ、それを蓄積する
			auto optimized_transform = Transform<double, 2, Isometry>::Identity();
			optimized_transform.rotate(optimized_rotation).pretranslate(optimized_translation);

			total_transform = optimized_transform * total_transform;

			// 線分数が十分に小さいため計算が少なくて済むよう、toのほうを動かしてやる。適用すべきは逆変換である事に注意
			const auto inv_transform = optimized_transform.inverse();
			for(auto& toi : to) {
				toi.p1 = inv_transform * toi.p1;
				toi.p2 = inv_transform * toi.p2;
			}
		}

		// Pose2Dにして返す
		const auto translation = total_transform.translation();
		const auto rotation = total_transform.rotation();
		return Pose2d{translation(0), translation(1), std::atan2(rotation(1, 0), rotation(0, 0))};
	}
}

namespace ac_zemi_2025::icp_on_svd {
	using impl::Pose2d;
	using impl::Line2d;
	using impl::distance_l2l;
	using impl::distance_p2l;
	using impl::icp_p2l;
}