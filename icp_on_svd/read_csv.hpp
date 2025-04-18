#pragma once

#include <eigen3/Eigen/Dense>

using namespace Eigen;

namespace crs_lib::icp_on_svd::read_csv {
	// CSV ファイルを読み込んで MatrixXd に変換する関数
    MatrixXd readCSV(const string& filename) {
        ifstream file(filename);  // ファイルを開く
        string line;
        vector<vector<double>> data;

        // ファイルが正常に開けた場合
        if (file.is_open()) {
            while (getline(file, line)) {  // 行ごとに読み込む
                stringstream ss(line);
                string value;
                vector<double> row;
                while (getline(ss, value, ',')) {  // カンマ区切りで分割
                    row.push_back(stod(value));  // 値を double に変換して行に追加
                }
                data.push_back(row);  // 行をデータに追加
            }
            file.close();  // ファイルを閉じる
        } else {
            cerr << "Unable to open file: " << filename << endl;
            return MatrixXd();  // 空の行列を返す
        }

        // 2次元ベクトルから MatrixXd を作成
        MatrixXd matrix(data.size(), data[0].size());
        for (size_t i = 0; i < data.size(); ++i) {
            for (size_t j = 0; j < data[i].size(); ++j) {
                matrix(i, j) = data[i][j];  // 行列に値をセット
            }
        }

        return matrix;
    }

// MatrixXd Lines2(MatrixXd LINES)
	// {
	// 	int Linerow = LINES.rows();
	// 	MatrixXd LINES_2(2 * Linerow, 3);
	// 	for (int i = 0; i < Linerow; i++)
	// 	{
	// 		LINES_2(2 * i, 0) = LINES(i, 0);
	// 		LINES_2(2 * i, 1) = LINES(i, 1);
	// 		LINES_2(2 * i, 2) = 1;
	// 		LINES_2(2 * i + 1, 0) = LINES(i, 2);
	// 		LINES_2(2 * i + 1, 1) = LINES(i, 3);
	// 		LINES_2(2 * i + 1, 2) = 1;
	// 	}
	// 	return LINES_2;
	// }

	// // spring25.csv ファイルの読み込み
	// string filename1 = "spring25.csv";
	// MatrixXd lines = readCSV(filename1);
	// MatrixXd LINES2 = Lines2(lines);

	// string filename2 = "spring25route1.csv";
	// MatrixXd route = readCSV(filename2);
}