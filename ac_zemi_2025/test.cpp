#include <thread>
#include <atomic>
#include <iostream>
#include <vector>
#include <chrono>

#include "utility.hpp"
#include "geometry.hpp"
#include "sparse_matrix.hpp"
#include "icp_on_svd.hpp"
#include "global_map.hpp"
#include "diff2_pure_pursuit.hpp"
#include "simulator.hpp"
// #include "ros_world.hpp"

namespace test {
	using Eigen::Matrix2Xd;
	using Eigen::Vector2d;

	using namespace ac_zemi_2025::integer_type;
	using namespace ac_zemi_2025::geometry;
	using namespace ac_zemi_2025::icp_on_svd;
	using namespace ac_zemi_2025::global_map;
	using namespace ac_zemi_2025::diff2_carrot_pursuit;
	using namespace ac_zemi_2025::simulator;
	// using namespace ac_zemi_2025::ros_world;

	// 差動二輪ロボの定数と状態
	struct RobotConstant final {
		GlobalMap<Line2d> map;
		std::vector<Vector2d> route;
		Diff2CarrotPursuit carrot;
		i64 number_of_iteration;
	};
	struct RobotState final {
		Pose2d pose;
		i64 closest_milestone_index;
	};

	// ロボの更新式
	inline auto robot_update(const RobotConstant& cons, RobotState& state, const Matrix2Xd& laserscan, const double dt) noexcept(false) -> Diff2wheelSpeed {
		// read state /////////////////////////////////////////////////////////////////////////////
		const auto pose = state.pose;

		// ICP on SVD /////////////////////////////////////////////////////////////////////////////
		const auto visible_edges = cons.map.make_visible_lines(pose);
		const auto new_pose = icp_p2l(laserscan, visible_edges, cons.number_of_iteration);

		// calc control input /////////////////////////////////////////////////////////////////////
		const auto speed = cons.carrot.update(cons.route, new_pose, state.closest_milestone_index);
		if(!speed.has_value()) {
			throw std::runtime_error{"Panic: cannot calc speed by carrot pursuit."};
		}

		// update state ///////////////////////////////////////////////////////////////////////////
		state.pose = new_pose + speed->to_pose2d_velocity(new_pose) * dt;

		return *speed;
	}

	struct MyClock final {
		std::chrono::time_point<std::chrono::steady_clock> last;

		static auto make() noexcept -> MyClock {
			return MyClock{std::chrono::steady_clock::now()};
		}

		auto watch() const noexcept -> std::chrono::duration<double> {
			const auto now = std::chrono::steady_clock::now();
			return now - this->last;
		}

		auto lap() noexcept -> std::chrono::duration<double> {
			const auto last = this->last;
			this->last = std::chrono::steady_clock::now();
			return this->last - last;
		}
	};

	void main() {
		/// @todo グローバルな図形情報を読み出し
		const auto shapes = std::vector<Line2d>{};

		/// @todo ルート情報を読み出し
		const auto route = std::vector<Vector2d>{};

		// グローバルマップを生成
		const auto map = GlobalMap<Line2d>::from_shapes(shapes);

		// 終了用のキー受付
		std::atomic_bool stop_flag{};
		std::jthread{[&stop_flag] {
			char dummy;
			std::cin >> dummy;
			stop_flag.store(true);
		}};

		/// @todo 外界の初期化
		SimulatorConstant sim_cons{};
		SimulatorState sim_state{};
		// RosWorld node{};

		/// @todo ロボットの初期化
		RobotConstant rb_cons{};
		RobotState rb_state{};

		Diff2wheelSpeed control_input{0.0, 0.0};

		auto sim_clock = MyClock::make();
		auto robo_clock = MyClock::make();
		// メインループ
		while(stop_flag.load()) {
			// calc world /////////////////////////////////////////////////////////////////////////

			const auto sensor_data = sim_update(sim_cons, sim_state, control_input, 0.1);
			// const auto sensor_data = node.update(control_input);

			// calc robot /////////////////////////////////////////////////////////////////////////
			control_input = robot_update(rb_cons, rb_state, sensor_data.laserscan, 0.1);

			// snapshot ///////////////////////////////////////////////////////////////////////////
			// sim_state.snap(logger);
			// rb_state.snap(logger);
		}
	}
}

int main() {
	test::main();
}