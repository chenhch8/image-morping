#ifndef MORPHING_H
#define MORPHING_H

#include "CImg.h"
#include "Delaunay.h"
#include <iostream>
#include <string>
#include <sstream> 

using namespace std;
using namespace cimg_library;

class Morphing {
 private:
 	CImg<float> srcImg;      // 起始图像
 	CImg<float> disImg;      // 终止图像
 	CImg<float> midImg;      // 中间图像
 	vector<Point> srcPoints; // 起始图像特征点集
 	vector<Point> disPoints; // 终止图像特征点集
 	int times;               // 帧数
 	// 初始化各种变量
 	void init(string &srcImg, string &disImg, string &srcPoints,
 			  string &disPoints, int &times);
 	// 三角变换矩阵求解
 	void calcMatrixs(vector<vector<int>> &tran, vector<CImg<float>> *matrixs,
					 vector<Point> &midPoints);
 	// 三角变换矩阵求解辅助函数
 	void calcMatrixHelper(vector<vector<int>> &tran, vector<CImg<float>> &matrix,
						  vector<Point> &sourcePoints, vector<Point> &goalPoints);
 	// 图形变换
 	void calcTargetImgHelper(vector<vector<int>> &tran, vector<CImg<float>> *matrixs,
							 vector<Point> &midPoints, double factor);
	void calcTargetImg(vector<vector<int>> &tran, int &times);
	// 判断点(x, y)是否在三角形 ABC 内
	bool isInTriangle(vector<int> &ABC, vector<Point> &goalPoints, int &row, int &col);
	// 图像保存
	void saveImg(int &i);
 public:
	Morphing() {}
	~Morphing() {}

	/*
	 * srcImg    起始图像名
	 * disImg	 终止图像名
	 * srcPoints 保存起始图像的特征点的文件名
	 * disPoints 保存终止图像的特征点的文件名
	 * times	 起始图像过渡到终止图像的帧数
	 */
	void start(string srcImg, string disImg, string srcPoints, string disPoints, int times);

	/* 测试函数 */
	friend void triangleTest(Morphing &temp, vector<vector<int>> &tran, vector<Point> &goalPoints);
};

#endif