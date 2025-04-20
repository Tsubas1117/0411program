#pragma once

#include <cmath>

#include <vector>
#include <tuple>

#include <eigen3/Eigen/Dense>

#include "utility.hpp"
#include "sparse_matrix.hpp"

namespace ac_zemi_2025::global_map::impl {
	using Eigen::Vector2d;
	
	using utility::Line2d;
	using utility::Pose2d;
	using sparse_matrix::Csr;

	template<class Edge_> requires
	requires(Edge_ edge) {
		/// @todo
		requires true;
	}
	struct GlobalMap final {
		Csr<Edge_> edges;
		std::vector<Vector2d> positions;

		/// @brief グローバル座標系でのマップから、ローカル座標系での可視なマップを計算
		/// マップの各曲線は重複せず、またそれぞれ曲線は交点を端点以外に持たないようにしておく
		/// -> @todo: 上の制約はどうせ時々凡ミスで破られるので、適宜assertやstd::expectedなどを入れる必要あり
		///   -> や、上の制約を満たすようにGlobalMapを構築する関数を書くべきか
		/// make_visible_linesの計算量は、端点数をNとして O(NlogN)。ソート分となる
		auto make_visible_lines(const Pose2d& pose) const noexcept -> std::vector<Line2d> {
			const i64 n = this->positions.size();
			const auto global2local = pose.homogeneus_transform().inverse();

			// マップ曲線をローカル極座標系に持ってくる
			struct RThetaVertex final {
				double r2;
				double th;
				i64 idx;

				friend constexpr auto operator<=>(const RThetaVertex&, const RThetaVertex&) = default;
				friend constexpr auto operator==(const RThetaVertex&, const RThetaVertex&) -> bool = default;
			};
			std::vector<RThetaVertex> local_vertices{};
			local_vertices.reserve(n);
			for(i64 i = 0; i < n; ++i) {
				const Vector2d pos = global2local * this->positions[i];
				const double r2 = pos.squaredNorm();
				const double th = std::atan2(pos(1), pos(0));
				local_vertices.emplace_back(r2, th, i);
			}
			
			// theta, r2, idxの順の辞書式ソート
			std::ranges::sort(local_vertices, [](const RThetaVertex& l, const RThetaVertex& r) -> bool {
				return l.th != r.th ? l.th < r.th :
					l.r2 != r.r2 ? l.r2 < r.r2 :
					l.idx < r.idx
				;
			});

			// idxから頂点を取得するための配列
			std::vector<i64> inv_indices(n);
			for(i64 i = 0; i < n; ++i) {
				inv_indices[local_vertices[i].idx] = i;
			}
			constexpr auto get = [&local_vertices, &inv_indices](const i64 idx) noexcept -> RThetaVertex {
				return local_vertices[inv_indices[idx]];
			};

			std::vector<Edge_> ret{};
			ret.reserve(n);

			// thの小さいほうから、可視な曲線を見つけていく
			i64 leftmost_idx = 0;
			while(leftmost_idx < n) {
				// DAGの隣接頂点の中で最も原点に近いものを次々選んでいく
				i64 last_idx = leftmost_idx;
				auto nexts = this->edges.c_row(last_idx);
				while(nexts.size() > 1) {
					std::optional<std::pair<usize_t, const Edge_&>> next{std::nullopt};
					for(const auto& [idx, edge] : nexts) {
					// 一つ前やthetaが減る向きに戻ろうとしないよう注意
					if(idx == last_idx || get(idx).th < get(last_idx).th) continue;
						const auto& [next_idx, _] = *next;
						if(!next.has_value() || get(idx).r < get(next_idx).r) {
							next = {idx, edge};
						}
					}
					if(!next.has_value()) break;

					const auto [next_idx, edge] = *next;
					ret.emplace_back(edge);
					last_idx = next_idx;
					nexts = this->edges.c_row(next_idx);
				}

				leftmost_idx = last_idx + 1;
			}

			return ret;
		}
	};
}