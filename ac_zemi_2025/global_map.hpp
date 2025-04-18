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

	/// @brief グローバル座標系でのマップから、ローカル座標系での可視なマップを計算
	/// マップの各曲線は重複せず、またそれぞれ曲線は交点を端点以外に持たないようにしておく
	/// -> @todo: 上の制約はどうせ時々凡ミスで破られるので、適宜assertやstd::expectedなどを入れる必要あり
	/// make_visible_linesの計算量はO(NlogN)、ソート分となる
	/// 今後、マップにあるのが線分だけで無くなった場合、Csrをテンプレートにし、各辺に線分情報を持たせると良い
	struct GlobalMap final {
		Csr edges;
		std::vector<Vector2d> positions;

		auto make_visible_lines(const Pose2d& pose) const noexcept -> std::vector<Line2d> {
			const i64 n = this->positions.size();
			
			std::vector<std::tuple<double, double, i64>> local_rtheta{};
			local_rtheta.reserve(n);
			for(i64 i = 0; i < n; ++i) {
				const Vector2d& pos = this->positions[i];
				local_rtheta.emplace_back(pos.squaredNorm(), std::atan2(pos(1), pos(2)), i);
			}
			
			std::ranges::sort(local_rtheta, [](const auto& l, const auto& r) -> bool {
				const auto [r_l, th_l, idx_l] = l;
				const auto [r_r, th_r, idx_r] = r;
				return th_l != th_r ? th_l < th_r :
					r_l != r_r ? r_l < r_r :
					idx_l < idx_r
				;
			});

			std::vector<Line2d> ret{};
			std::vector<i64> connected_vertices{};
			ret.reserve(n);
			connected_vertices.reserve(n);

			auto iter = local_rtheta.cbegin();
			while(iter != local_rtheta.cend()) {
				i64 idx = std::get<2>(*iter);
				auto nexts = this->edges.c_row(idx);
				connected_vertices.emplace_back(idx);
				while(!nexts.empty()) {
					const auto next_idx = nexts.front();
					if(std::get<1>(local_rtheta[next_idx]) < std::get<1>(local_rtheta[idx])) break;
					connected_vertices.emplace_back(next_idx);
					nexts = this->edges.c_row(next_idx);
				}
				
				/// @todo: ここはCsr<E>にしたなら、各辺に持たせた曲線情報を使ってやるといい
				for(i64 i = 0; i < connected_vertices.size() - 1; ++i) {
					ret.emplace_back(Line2d{this->positions[connected_vertices[i]], this->positions[connected_vertices[i + 1]]});
				}

				iter = connected_vertices.back()
				connected_vertices.clear();
			}
		}
	};

	
	inline auto make_visible_lines(const auto& map, const Pose2d& pose) noexcept -> std::vector<Line2d> {
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
}