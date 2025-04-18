#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include <ctime>
#include <fstream>
#include <sstream>
#include <chrono>
#include <random>
#include <numbers>

using namespace std;
using namespace Eigen;

using numbers::pi;

// CSV ファイルを読み込んで MatrixXd に変換する関数
MatrixXd readCSV(const string &filename)
{
	ifstream file(filename); // ファイルを開く
	string line;
	vector<vector<double>> data;

	// ファイルが正常に開けた場合
	if (file.is_open())
	{
		while (getline(file, line))
		{ // 行ごとに読み込む
			stringstream ss(line);
			string value;
			vector<double> row;
			while (getline(ss, value, ','))
			{								// カンマ区切りで分割
				row.push_back(stod(value)); // 値を double に変換して行に追加
			}
			data.push_back(row); // 行をデータに追加
		}
		file.close(); // ファイルを閉じる
	}
	else
	{
		cerr << "Unable to open file: " << filename << endl;
		return MatrixXd(); // 空の行列を返す
	}

	// 2次元ベクトルから MatrixXd を作成
	MatrixXd matrix(data.size(), data[0].size());
	for (size_t i = 0; i < data.size(); ++i)
	{
		for (size_t j = 0; j < data[i].size(); ++j)
		{
			matrix(i, j) = data[i][j]; // 行列に値をセット
		}
	}

	return matrix;
}

double modpi(double x)
{
	double result = fmod(x, 2 * pi);
	if (result < -pi)
	{
		result += 2 * pi;
	}
	else if (result > pi)
	{
		result -= 2 * pi;
	}
	return result;
}

MatrixXd ITM(double x, double y, double th, MatrixXd veclist)
{
	Matrix3d itm;
	itm << cos(th), -sin(th), x,
		sin(th), cos(th), y,
		0, 0, 1;
	int cols = veclist.cols();
	int rows = veclist.rows();
	veclist.conservativeResize(rows + 1, cols); // 新しい行を追加
	veclist.row(rows) = VectorXd::Ones(cols);	// 1

	MatrixXd itmv = itm * veclist;
	itmv.conservativeResize(rows, cols);
	return itmv;
}

VectorXd linspace(double start, double stop, int num)
{
	Eigen::VectorXd result(num);
	double step = (stop - start) / (num - 1); // 各値の間隔を計算

	for (int i = 0; i < num; ++i)
	{
		result[i] = start + i * step; // 各値を計算してEigenのベクトルに格納
	}
	return result;
}

MatrixXd lidarlist(double x, double y, double th, MatrixXd lines0)
{
	int J = lines0.rows();
	int beemn = 100;
	MatrixXd thetas = linspace(-1.5, 1.5, beemn);
	RowVectorXd lidarxlist = RowVectorXd::Zero(beemn);
	RowVectorXd lidarylist = RowVectorXd::Zero(beemn);
	double R = 20000;
	VectorXd closestlist = VectorXd::Zero(beemn);
	for (int i = 0; i < beemn; i++)
	{
		double xrobo, yrobo, xbeem, ybeem;
		xrobo = x;
		yrobo = y;
		xbeem = x + R * cos(th + thetas(i));
		ybeem = y + R * sin(th + thetas(i));
		double closest_distance0 = sqrt(pow((xbeem - x), 2) + pow((ybeem - y), 2));
		for (int j = 0; j < J; j++)
		{
			double x1, y1, x2, y2;
			x1 = lines0(j, 0);
			y1 = lines0(j, 1);
			x2 = lines0(j, 2);
			y2 = lines0(j, 3);
			double denominator = (x1 - x2) * (yrobo - ybeem) - (y1 - y2) * (xrobo - xbeem);

			if (denominator == 0)
			{
				continue;
			}
			double t = ((x1 - xrobo) * (yrobo - ybeem) - (y1 - yrobo) * (xrobo - xbeem)) / denominator;
			double u = -((x1 - x2) * (y1 - yrobo) - (y1 - y2) * (x1 - xrobo)) / denominator;

			if ((0 <= t) && (t <= 1) && (0 <= u) && (u <= 1))
			{
				double intersection_x = x1 + t * (x2 - x1);
				double intersection_y = y1 + t * (y2 - y1);
				double robo_wall_distance = sqrt(pow(intersection_x - xrobo, 2) + pow(intersection_y - yrobo, 2));
				if (robo_wall_distance <= closest_distance0)
				{
					closest_distance0 = robo_wall_distance;
				}
				else
				{
					continue;
				}
			}
			else
			{
				continue;
			}
		}

		double Lnoise = 1.0;
		closestlist(i) = (Lnoise * closest_distance0);
		lidarxlist(i) = closestlist(i) * cos(thetas(i));
		lidarylist(i) = closestlist(i) * sin(thetas(i));
	}

	MatrixXd LiDAR(2, beemn);
	LiDAR << lidarxlist, lidarylist;
	return LiDAR;
}

MatrixXd Lines2(MatrixXd LINES)
{
	int Linerow = LINES.rows();
	MatrixXd LINES_2(2 * Linerow, 3);
	for (int i = 0; i < Linerow; i++)
	{
		LINES_2(2 * i, 0) = LINES(i, 0);
		LINES_2(2 * i, 1) = LINES(i, 1);
		LINES_2(2 * i, 2) = 1;
		LINES_2(2 * i + 1, 0) = LINES(i, 2);
		LINES_2(2 * i + 1, 1) = LINES(i, 3);
		LINES_2(2 * i + 1, 2) = 1;
	}
	return LINES_2;
}

// spring25.csv ファイルの読み込み
string filename1 = "spring25.csv";
MatrixXd lines = readCSV(filename1);
MatrixXd LINES2 = Lines2(lines);

string filename2 = "spring25route1.csv";
MatrixXd route = readCSV(filename2);

VectorXd icp(double x, double y, double th, MatrixXd LiDAR)
{

	Matrix3d transmatrix_field;
	transmatrix_field << cos(th), sin(th), -x * cos(th) - y * sin(th),
		-sin(th), cos(th), x * sin(th) - y * cos(th),
		0, 0, 1;

	MatrixXd lines2t(LINES2.rows(), 3);
	lines2t = LINES2 * (transmatrix_field.transpose());
	// cout<<lines2t<<endl;
	MatrixXd A = LiDAR.transpose();
	int Arows = A.rows();
	int Acols = A.cols();
	MatrixXd totalTrans = MatrixXd::Identity(3, 3);
	MatrixXd totalTrans0;
	for (int k = 0; k < 50; k++)
	{
		// MatrixXd Nearlist(A.rows(),A.cols());
		MatrixXd B(A.rows(), A.cols());
		for (int j = 0; j < A.rows(); j++)
		{
			double x0, y0;
			x0 = A(j, 0);
			y0 = A(j, 1);
			double x1, y1, x2, y2, xi, yi, Xnear, Ynear;
			double Short = std::numeric_limits<double>::infinity();
			for (int i = 0; i < LINES2.rows() / 2; i++)
			{

				x1 = lines2t(2 * i, 0);
				y1 = lines2t(2 * i, 1);
				x2 = lines2t(2 * i + 1, 0);
				y2 = lines2t(2 * i + 1, 1);

				if (((x0 - x1) * (x2 - x1) + (y0 - y1) * (y2 - y1)) < 0)
				{
					xi = x1;
					yi = y1;
				}
				else if ((x0 - x2) * (x2 - x1) + (y0 - y2) * (y2 - y1) > 0)
				{
					xi = x2;
					yi = y2;
				}
				else
				{
					double AI = ((x0 - x1) * (x2 - x1) + (y0 - y1) * (y2 - y1)) / ((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
					xi = x1 + AI * (x2 - x1);
					yi = y1 + AI * (y2 - y1);
				}
				double di2 = (x0 - xi) * (x0 - xi) + (y0 - yi) * (y0 - yi);
				if (di2 < Short)
				{
					Short = di2;
					Xnear = xi;
					Ynear = yi;
				}
			}
			B(j, 0) = Xnear;
			B(j, 1) = Ynear;
		}

		// AとBをそれぞれ平均を取ってひく

		// // 各列の平均を計算
		// Eigen::VectorXd colMeans = mat.colwise().mean();
		// // 各列からその平均を引く
		// Eigen::MatrixXd result = mat.rowwise() - colMeans.transpose();

		VectorXd Amean = A.colwise().mean();
		VectorXd Bmean = B.colwise().mean();
		MatrixXd Aresult = A.rowwise() - Amean.transpose();
		MatrixXd Bresult = B.rowwise() - Bmean.transpose();

		MatrixXd H = (Aresult.transpose()) * Bresult;

		JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
		MatrixXd U = svd.matrixU();
		VectorXd Sigma = svd.singularValues(); // 特異値はベクトルとして格納//使わんけど
		MatrixXd V = svd.matrixV();

		MatrixXd Rm = (U * (V.transpose())).transpose();
		// cout<<Rm<<endl;
		// cout<<endl;
		if (Rm(0, 1) * Rm(1, 0) > 0)
		{
			Rm = MatrixXd::Identity(2, 2);
		}

		VectorXd xy = Bmean - Rm * Amean;

		MatrixXd Transmatrix(3, 3);
		Transmatrix << Rm(0, 0), Rm(0, 1), xy(0),
			Rm(1, 0), Rm(1, 1), xy(1),
			0, 0, 1;

		totalTrans0 = Transmatrix * totalTrans;

		totalTrans = totalTrans0;

		// veclist.conservativeResize(Arows, 3);  // 新しい行を追加
		// veclist.row(rows) = VectorXd::Ones(cols); //1
		// MatrixXd itmv=itm*veclist;
		// itmv.conservativeResize(rows,cols);

		A.conservativeResize(Arows, Acols + 1); // 新しい行を追加
		A.col(Acols) = VectorXd::Ones(Arows);	// 1

		MatrixXd newA = A * (Transmatrix.transpose());
		A = newA;
		A.conservativeResize(Arows, Acols);
	}

	double dx, dy, dth;
	dx = totalTrans(0, 2);
	dy = totalTrans(1, 2);
	dth = atan2(totalTrans(1, 0), totalTrans(0, 0));

	VectorXd calc(3);
	calc << x + dx * cos(th) - dy * sin(th), y + dx * sin(th) + dy * cos(th), (th + dth);

	return calc;
}

int main()
{
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

	xc = Posi(0);
	yc = Posi(1);
	thc = Posi(2);
	cout << route.rows() << endl;
	int N = 100;
	for (int t = 0; t < N; t++)
	{
		auto start = chrono::high_resolution_clock::now();

		double head = 500;
		double dhead = head / 4;
		double near = 20000; // std::numeric_limits<double>::infinity();
		VectorXd distances(route.rows());
		int nearindex = 0;
		int cn = 0;
		double thnear, alpha;
		for (int i = 0; i < route.rows(); i++)
		{
			distances(i) = sqrt(pow((route(i, 0) - xc), 2) + pow((route(i, 1) - yc), 2));

			if ((head < distances(i)) && (distances(i) < near) && (abs(modpi(atan2(route(i, 1) - yc, route(i, 0) - xc) - thc)) < 1))
			{
				near = distances(i);
				nearindex = i;
				thnear = atan2(route(i, 1) - yc, route(i, 0) - xc);
				alpha = thnear - thc;
				cn++;
			}
		}
		if (cn == 0)
		{
			cout << "out" << endl;
			for (int i = 0; i < route.rows(); i++)
			{
				distances(i) = sqrt(pow((route(i, 0) - xc), 2) + pow((route(i, 1) - yc), 2));
				if ((head < distances(i)) && (distances(i) < near))
				{
					near = distances(i);
					nearindex = i;
					thnear = atan2(route(i, 1) - yc, route(i, 0) - xc);
					alpha = thnear - thc;
					cn += 1;
				}
			}
		}

		double dr, dth, dx, dy, drr, dthr, dxr, dyr;

		dr = dhead;
		dth = alpha;

		dx = dr * cos(dth);
		dy = dr * sin(dth);
		MatrixXd dvec(2, 1);
		dvec << dx, dy;
		MatrixXd dav = ITM(xc, yc, thc, dvec);

		xp = (dav(0, 0));
		yp = (dav(1, 0));
		thp = (thc + dth);

		drr = dr * (1.3);
		dthr = dth * (1.005) + 0.2;

		dxr = drr * cos(dthr);
		dyr = drr * sin(dthr);
		MatrixXd dvecr(2, 1);
		dvecr << dxr, dyr;
		MatrixXd davr = ITM(xr, yr, thr, dvecr);

		xr = (davr(0, 0));
		yr = (davr(1, 0));
		thr = (thr + dthr);

		lasermap = lidarlist(xr, yr, thr, lines);
		Posi = icp(xp, yp, thp, lasermap);

		xc = Posi(0);
		yc = Posi(1);
		thc = Posi(2);

		auto end = chrono::high_resolution_clock::now();
		auto duration = chrono::duration_cast<chrono::milliseconds>(end - start);
		cout << "実行時間: " << duration.count() << " milliseconds" << endl;
		cout << t << " " << Posi.transpose() << endl;
		cout << "real " << xr << " ," << yr << ", " << thr << endl;
		cout << endl;
	}

	////////////////////////////////////////////////////////

	cout << "robot" << endl;
	int rn = 7;
	RowVectorXd robox(7);
	robox << 0, -1, -3, -3, -3, -1, 0;
	RowVectorXd roboy(7);
	roboy << 0, 1., 1, 0, -1, -1, 0;
	robox = 400 * robox / 3;
	roboy = 400 * roboy / 2;
	double avx = robox.mean();
	double avy = roboy.mean();
	robox -= avx * VectorXd::Ones(rn);
	roboy -= avy * VectorXd::Ones(rn);
	MatrixXd robovec(2, 7);
	robovec << robox, roboy;
	////////////////////////////////////////////////////
	return 0;
}
