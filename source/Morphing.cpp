#include "Morphing.h"

/*
 * 测试函数
 */

// 测试 isInTriangle 函数是否正确
void triangleTest(Morphing &temp, vector<vector<int>> &tran, vector<Point> &goalPoints) {
	// 染色图
	CImg<float> result(temp.disImg.width(), temp.disImg.height(), 1, 3, 0);
	int color[][3] = { {255, 0, 0}, {0, 255, 0}, {0, 0, 255},
					   {255, 255, 0}, {0, 255, 255}, {255, 255, 255} };
	int x, y, i, count = 0;
	// 为每个三角形配色
	vector<int> tran_color;
	for (i = 0; i < tran.size(); ++i) {
		tran_color.push_back(count);
		count = ++count % 6;
	}
	// 为染色图染色
	cimg_forXY(result, x, y) {
		for (i = 0; i < tran.size(); ++i) {
			if (temp.isInTriangle(tran[i], goalPoints, y, x)) {
				result(x, y, 0) = color[tran_color[i]][0];
				result(x, y, 1) = color[tran_color[i]][1];
				result(x, y, 2) = color[tran_color[i]][2];
			}
		}
	}
	result.display("triangleTest");
}

// 测试函数
void test(Morphing &temp, vector<vector<int>> &tran, vector<Point> &goalPoints) {
	// 测试 isInTriangle 函数是否正确
	triangleTest(temp, tran, goalPoints);
}

/*
 * 类方法实现
 */

void Morphing::init(string &srcImg, string &disImg, string &srcPoints, string &disPoints, int &times) {
	ifstream finSrc1, finSrc2;
	finSrc1.open(srcPoints.c_str());
	finSrc2.open(disPoints.c_str());
	if (finSrc1.fail() || finSrc2.fail()){
		cout << "文件打开错误" << endl;
		exit(1);
	}
	int col, row;
	while (!finSrc1.eof() && !finSrc2.eof()) {
		finSrc1 >> row >> col;
		this->srcPoints.push_back(Point(col, row));
		finSrc2 >> row >> col;
		this->disPoints.push_back(Point(col, row));
	}
	finSrc1.close();
	finSrc2.close();
	if (this->srcPoints.size() != this->disPoints.size()) {
		cout << "起始图像特征点和终止图像特征点数量不一致！" << endl;
		exit(1);
	}
	this->srcImg.load(srcImg.c_str());
	this->disImg.load(disImg.c_str());
	this->times = times;
}

void Morphing::start(string srcImg, string disImg, string srcPoints, string disPoints, int times) {
	// [1] 初始化
	this->init(srcImg, disImg, srcPoints, disPoints, times);
	// [2] 三角剖分
	vector<vector<int>> tran;
	delaunay(this->disImg.width(), this->disImg.height(), this->disPoints, tran);
	// [3] 图形变换
	calcTargetImg(tran, times);
}

void Morphing::calcTargetImg(vector<vector<int>> &tran, int &times) {
	vector<vector<Point>> pointsList;
	vector<Point> points;

	vector<double> factors;
	double factor;

	int index = 0, x, y, i, j;
	// [1] 计算中间特征点——形状变形
	for (i = 0; i <= times; ++i) {
		points.clear();
		factor = i / double(times);
		for (j = 0; j < this->disPoints.size(); ++j) {
			x = (this->disPoints[j].x - this->srcPoints[j].x) * factor + this->srcPoints[j].x;
			y = (this->disPoints[j].y - this->srcPoints[j].y) * factor + this->srcPoints[j].y;
			points.push_back(Point(x, y));
		}
		factors.push_back(factor);
		pointsList.push_back(points);
	}
	// [2] 计算中间仿射变换矩阵
	vector<CImg<float>> matrixs; // 仿射变换矩阵
	for (i = 0; i < times; ++i) {
		matrixs.clear();
		// [2.1] 计算仿射变换矩阵
		calcMatrixs(tran, matrixs, pointsList[i], pointsList[i + 1]);
		// [2.2] 图形变换
		calcTargetImgHelper(tran, matrixs, pointsList[i + 1], factors[i]);
		// // 测试
		// test(*this, tran, pointsList[i + 1]);
	}
}

// M*ABC=abc -> M=abc/ABC
void Morphing::calcMatrixs(vector<vector<int>> &tran, vector<CImg<float>> &matrixs,
						   vector<Point> &sourcePoints, vector<Point> &goalPoints) {
	CImg<float> ABC(3, 3, 1, 1, 0); // 起始三角形矩阵
	CImg<float> abc(3, 3, 1, 1, 0); // 终止三角形矩阵
	// 对每个三角形计算其变换矩阵
	for (int i = 0; i < tran.size(); ++i) {
		// 矩阵赋值
		for (int col = 0; col < 3; ++col) {
			// img(col,row) 这里尤其注意，否则发生非法访问内存，还会对后面的运算结果产生重大影响！！
			ABC(col, 0) = goalPoints[tran[i][col]].x;
			ABC(col, 1) = goalPoints[tran[i][col]].y;
			ABC(col, 2) = 1;
			
			abc(col, 0) = sourcePoints[tran[i][col]].x;
			abc(col, 1) = sourcePoints[tran[i][col]].y;
			abc(col, 2) = 1;
		}
		// 计算 M 并保存到 matrixs 中
		matrixs.push_back(abc / ABC);
	}
	return;
}

void Morphing::calcTargetImgHelper(vector<vector<int>> &tran, vector<CImg<float>> &matrixs,
								   vector<Point> &goalPoints, double factor) {
	CImg<float> result(this->disImg.width(), this->disImg.height(), 1, 3, 0); // 目标图片
	CImg<char> where(this->disImg.width(), this->disImg.height(), 1, 1, 0);   // 保存像素点(x,y)在哪个三角形中
	int x, y, i;
	// 找出所有像素点所属哪个三角形
	cimg_forXY(this->disImg, x, y) {
		for (i = 0; i < tran.size(); ++i) {
			if (isInTriangle(tran[i], goalPoints, y, x)) {
				where(x, y) = i;
				break;
			} else if (i == tran.size() - 1) {
				where(x, y) = -1;
			}
		}
	}
	// 计算目标图
	CImg<float> X(1, 3, 1, 1, 0), // 目标图上的点
				U(1, 3, 1, 1, 0); // 目标图投影到起始图上的点
	cimg_forXY(this->disImg, x, y) {
		X(0, 0) = x; X(0, 1) = y; X(0, 2) = 1;
		if ((int)where(x, y) != -1) {
			U = matrixs[(int)where(x, y)] * X;
			U(0, 0) /= U(0, 2);
			U(0, 1) /= U(0, 2);
		} else {
			U(0, 0) = x; U(0, 1) = y;
		}
		U(0, 0) = U(0, 0) < 0 ? 0 : U(0, 0);
		U(0, 1) = U(0, 1) < 0 ? 0 : U(0, 1);
		result(x, y, 0) = this->srcImg(U(0, 0), U(0, 1), 0) * (1 - factor) + this->disImg(x, y, 0) * factor;
		result(x, y, 1) = this->srcImg(U(0, 0), U(0, 1), 1) * (1 - factor) + this->disImg(x, y, 1) * factor;
		result(x, y, 2) = this->srcImg(U(0, 0), U(0, 1), 2) * (1 - factor) + this->disImg(x, y, 2) * factor;
	}
	result.display("result");
	this->srcImg = result;
}

/* 重心法 判断点是否在三角形内 */
bool Morphing::isInTriangle(vector<int> &ABC, vector<Point> &goalPoints, int &row, int &col) {
	int v0_x = goalPoints[ABC[1]].x - goalPoints[ABC[0]].x,
		v0_y = goalPoints[ABC[1]].y - goalPoints[ABC[0]].y,
		
		v1_x = goalPoints[ABC[2]].x - goalPoints[ABC[0]].x,
		v1_y = goalPoints[ABC[2]].y - goalPoints[ABC[0]].y,
		
		v2_x = col - goalPoints[ABC[0]].x,
		v2_y = row - goalPoints[ABC[0]].y;

	int s12 = v1_x * v2_x + v1_y * v2_y,
		s00 = v0_x * v0_x + v0_y * v0_y,
		s02 = v0_x * v2_x + v0_y * v2_y,
		s01 = v0_x * v1_x + v0_y * v1_y,
		s11 = v1_x * v1_x + v1_y * v1_y;

	double inverDeno = 1 / double(s11 * s00 - s01 * s01);

	double x = (s02 * s11 - s12 * s01) * inverDeno,
	       y = (s12 * s00 - s02 * s01) * inverDeno;

	if (x < 0 || y < 0 || x > 1 || y > 1) return false;
	return x + y <= 1;
}
