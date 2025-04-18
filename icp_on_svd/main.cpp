#include <numbers>
#include <random>
#include "main.hpp"

using std::numbers::pi;

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

int main() {
	// 乱数生成器の初期化
	std::random_device rd;							 // シードを生成
	std::mt19937 gen(rd());							 // メルセンヌ・ツイスタ法で乱数を生成
	std::uniform_real_distribution<> dis(-1.0, 1.0); // -1.0 から 1.0 の範囲で一様分布

	double random_value = dis(gen); // -1.0 から 1.0 の範囲で乱数を生成

	double xp, yp, thp, xr, yr, thr, xc, yc, thc;

	xp = 2000;
	yp = 500;
	thp = 0;

	xr = xp;
	yr = yp;
	thr = thp;

	MatrixXd lasermap = lidarlist(xr, yr, thr, lines);
	VectorXd Posi = icp(xp, yp, thp, lasermap);

// 	xc = Posi(0);
// 	yc = Posi(1);
// 	thc = Posi(2);
// 	cout << route.rows() << endl;
// 	int N = 100;
// 	for (int t = 0; t < N; t++)
// 	{
// 		auto start = chrono::high_resolution_clock::now();

// 		double head = 500;
// 		double dhead = head / 4;
// 		double near = 20000; // std::numeric_limits<double>::infinity();
// 		VectorXd distances(route.rows());
// 		int nearindex = 0;
// 		int cn = 0;
// 		double thnear, alpha;
// 		for (int i = 0; i < route.rows(); i++)
// 		{
// 			distances(i) = sqrt(pow((route(i, 0) - xc), 2) + pow((route(i, 1) - yc), 2));

// 			if ((head < distances(i)) && (distances(i) < near) && (abs(modpi(atan2(route(i, 1) - yc, route(i, 0) - xc) - thc)) < 1))
// 			{
// 				near = distances(i);
// 				nearindex = i;
// 				thnear = atan2(route(i, 1) - yc, route(i, 0) - xc);
// 				alpha = thnear - thc;
// 				cn++;
// 			}
// 		}
// 		if (cn == 0)
// 		{
// 			cout << "out" << endl;
// 			for (int i = 0; i < route.rows(); i++)
// 			{
// 				distances(i) = sqrt(pow((route(i, 0) - xc), 2) + pow((route(i, 1) - yc), 2));
// 				if ((head < distances(i)) && (distances(i) < near))
// 				{
// 					near = distances(i);
// 					nearindex = i;
// 					thnear = atan2(route(i, 1) - yc, route(i, 0) - xc);
// 					alpha = thnear - thc;
// 					cn += 1;
// 				}
// 			}
// 		}

// 		double dr, dth, dx, dy, drr, dthr, dxr, dyr;

// 		dr = dhead;
// 		dth = alpha;

// 		dx = dr * cos(dth);
// 		dy = dr * sin(dth);
// 		MatrixXd dvec(2, 1);
// 		dvec << dx, dy;
// 		MatrixXd dav = ITM(xc, yc, thc, dvec);

// 		xp = (dav(0, 0));
// 		yp = (dav(1, 0));
// 		thp = (thc + dth);

// 		drr = dr * (1.3);
// 		dthr = dth * (1.005) + 0.2;

// 		dxr = drr * cos(dthr);
// 		dyr = drr * sin(dthr);
// 		MatrixXd dvecr(2, 1);
// 		dvecr << dxr, dyr;
// 		MatrixXd davr = ITM(xr, yr, thr, dvecr);

// 		xr = (davr(0, 0));
// 		yr = (davr(1, 0));
// 		thr = (thr + dthr);

// 		lasermap = lidarlist(xr, yr, thr, lines);
// 		Posi = icp(xp, yp, thp, lasermap);

// 		xc = Posi(0);
// 		yc = Posi(1);
// 		thc = Posi(2);

// 		auto end = chrono::high_resolution_clock::now();
// 		auto duration = chrono::duration_cast<chrono::milliseconds>(end - start);
// 		cout << "実行時間: " << duration.count() << " milliseconds" << endl;
// 		cout << t << " " << Posi.transpose() << endl;
// 		cout << "real " << xr << " ," << yr << ", " << thr << endl;
// 		cout << endl;
// 	}

// 	////////////////////////////////////////////////////////

// 	cout << "robot" << endl;
// 	int rn = 7;
// 	RowVectorXd robox(7);
// 	robox << 0, -1, -3, -3, -3, -1, 0;
// 	RowVectorXd roboy(7);
// 	roboy << 0, 1., 1, 0, -1, -1, 0;
// 	robox = 400 * robox / 3;
// 	roboy = 400 * roboy / 2;
// 	double avx = robox.mean();
// 	double avy = roboy.mean();
// 	robox -= avx * VectorXd::Ones(rn);
// 	roboy -= avy * VectorXd::Ones(rn);
// 	MatrixXd robovec(2, 7);
// 	robovec << robox, roboy;
// 	////////////////////////////////////////////////////
// 	return 0;
}
