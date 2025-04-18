#pragma once

// inline auto linspace(const double start, const double stop, const int num) -> Vector
// {
// 	Eigen::VectorXd result(num);
// 	double step = (stop - start) / (num - 1); // 各値の間隔を計算

// 	for (int i = 0; i < num; ++i)
// 	{
// 		result[i] = start + i * step; // 各値を計算してEigenのベクトルに格納
// 	}
// 	return result;
// }

// inline MatrixXd lidarlist(double x, double y, double th, MatrixXd lines0)
// {
// 	int J = lines0.rows();
// 	int beemn = 100;
// 	MatrixXd thetas = linspace(-1.5, 1.5, beemn);
// 	RowVectorXd lidarxlist = RowVectorXd::Zero(beemn);
// 	RowVectorXd lidarylist = RowVectorXd::Zero(beemn);
// 	double R = 20000;
// 	VectorXd closestlist = VectorXd::Zero(beemn);
// 	for (int i = 0; i < beemn; i++)
// 	{
// 		double xrobo, yrobo, xbeem, ybeem;
// 		xrobo = x;
// 		yrobo = y;
// 		xbeem = x + R * cos(th + thetas(i));
// 		ybeem = y + R * sin(th + thetas(i));
// 		double closest_distance0 = sqrt(pow((xbeem - x), 2) + pow((ybeem - y), 2));
// 		for (int j = 0; j < J; j++)
// 		{
// 			double x1, y1, x2, y2;
// 			x1 = lines0(j, 0);
// 			y1 = lines0(j, 1);
// 			x2 = lines0(j, 2);
// 			y2 = lines0(j, 3);
// 			double denominator = (x1 - x2) * (yrobo - ybeem) - (y1 - y2) * (xrobo - xbeem);

// 			if (denominator == 0)
// 			{
// 				continue;
// 			}
// 			double t = ((x1 - xrobo) * (yrobo - ybeem) - (y1 - yrobo) * (xrobo - xbeem)) / denominator;
// 			double u = -((x1 - x2) * (y1 - yrobo) - (y1 - y2) * (x1 - xrobo)) / denominator;

// 			if ((0 <= t) && (t <= 1) && (0 <= u) && (u <= 1))
// 			{
// 				double intersection_x = x1 + t * (x2 - x1);
// 				double intersection_y = y1 + t * (y2 - y1);
// 				double robo_wall_distance = sqrt(pow(intersection_x - xrobo, 2) + pow(intersection_y - yrobo, 2));
// 				if (robo_wall_distance <= closest_distance0)
// 				{
// 					closest_distance0 = robo_wall_distance;
// 				}
// 				else
// 				{
// 					continue;
// 				}
// 			}
// 			else
// 			{
// 				continue;
// 			}
// 		}

// 		double Lnoise = 1.0;
// 		closestlist(i) = (Lnoise * closest_distance0);
// 		lidarxlist(i) = closestlist(i) * cos(thetas(i));
// 		lidarylist(i) = closestlist(i) * sin(thetas(i));
// 	}

// 	MatrixXd LiDAR(2, beemn);
// 	LiDAR << lidarxlist, lidarylist;
// 	return LiDAR;
// }