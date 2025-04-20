#pragma once

#include <concepts>
#include <vector>
#include <utility>

#include <eigen3/Eigen/Dense>

namespace ac_zemi_2025::geometry::impl {
	using Eigen::Vector2d;
	using Eigen::Isometry2d;

	template<class T_>
	concept edge = requires(const T_ e1, const T_ e2, const Vector2d p) {
		{intersect(e1, e2)} -> std::same_as<std::vector<Vector2d>>;
		{dis_p2e(p, e1)} -> std::same_as<double>;
		{dis_e2e(e1, e2)} -> std::same_as<double>;
	};

	// 姿勢
	struct Pose2d final {
		Vector2d xy;
		double th;

		constexpr auto homogeneus_transform() const noexcept -> Isometry2d {
			auto ret = Isometry2d::Identity();
			ret.rotate(this->th).pretranslate(this->xy);
			return ret;
		}
	};

	// 端点2つによる線分表現
	struct Line2d final {
		Vector2d p1;
		Vector2d p2;
	};

	inline constexpr auto dis_p2e(const Vector2d& part, const Line2d& whole) noexcept -> std::pair<Vector2d, double> {
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

	inline constexpr auto dis_e2e(const Line2d& part, const Line2d& whole) noexcept -> double {
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

	struct Polyline final {
		std::vector<Vector2d> vertices;
	};
}

namespace ac_zemi_2025::geometry {
	using impl::Line2d;
	using impl::Pose2d;
	using impl::dis_p2e;
	using impl::dis_e2e;
}