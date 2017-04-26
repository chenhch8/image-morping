/*
 * 使用 opencv 进行三角剖分
 */
#include "Delaunay.h"

/*
 * 获取三角剖分结果
 */
void disTriangles(int cols, int rows, Subdiv2D& subdiv, vector<vector<Point>> &triangles) {
    vector<Vec6f> triangleList;
    subdiv.getTriangleList(triangleList);
	vector<Point> pt(3);

    bool ok;
    Vec6f t;

    for(size_t i = 0; i < triangleList.size(); ++i) {
    	t = triangleList[i];
        pt[0] = Point(cvRound(t[0]), cvRound(t[1]));
        pt[1] = Point(cvRound(t[2]), cvRound(t[3]));
        pt[2] = Point(cvRound(t[4]), cvRound(t[5]));
		ok = true;
		for(int i = 0; i < 3; ++i) {
		 	if(pt[i].x > cols || pt[i].y > rows || pt[i].x < 0 || pt[i].y < 0)
				ok = false;
		}
		if (ok) {
			triangles.push_back(pt);
		}
    }
}

void indexMap(vector<vector<Point>> &triangles, vector<Point> &srcPoints, vector<vector<int>> &result) {
	vector<int> index(3);
	int i, j, k;
	for (i = 0; i < triangles.size(); ++i) {
		for (j = 0; j < 3; ++j) {
			for (k = 0; k < srcPoints.size(); ++k) {
				if (triangles[i][j] == srcPoints[k]) {
					index[j] = k;
					break;
				}
			}
		}
		result.push_back(index);
	}
}

void debug(vector<vector<Point>> &triangles, vector<Point> &srcPoints, vector<vector<int>> &result) {
	Scalar active_facet_color(0, 0, 255), delaunay_color(255,255,255);
	Mat img1 = imread("../input/2.bmp");
	Mat img2 = img1.clone();
	for (int i = 0; i < triangles.size(); ++i) {
		line(img1, triangles[i][0], triangles[i][1], active_facet_color);
		line(img1, triangles[i][0], triangles[i][2], active_facet_color);
		line(img1, triangles[i][2], triangles[i][1], active_facet_color);
	}
	for (int i = 0; i < result.size(); ++i) {
		line(img2, srcPoints[result[i][0]], srcPoints[result[i][1]], delaunay_color);
		line(img2, srcPoints[result[i][0]], srcPoints[result[i][2]], delaunay_color);
		line(img2, srcPoints[result[i][2]], srcPoints[result[i][1]], delaunay_color);
	}
	imshow("img1", img1);
	imshow("img2", img2);
	waitKey(5000);
}

/*
 * cols     [图像列数]
 * rows		[图像行数]
 * filename [特征点文件]
 * 返回每个三角形的顶点对应的下标
 */
void delaunay(int cols, int rows, vector<Point> &srcPoints, vector<vector<int>> &result) {
	Rect rect(0, 0, cols, rows);
    Subdiv2D subdiv(rect);	

	// [1] 读入特征点坐标
	for (int i = 0; i < srcPoints.size(); ++i) {
		subdiv.insert(srcPoints[i]);
	}
	// [2] 三角剖分
	vector<vector<Point>> triangles;
	disTriangles(cols, rows, subdiv, triangles);
	// [3] 找出三角行的三个顶点原先的下标
	indexMap(triangles, srcPoints, result);

	debug(triangles, srcPoints, result);
}
