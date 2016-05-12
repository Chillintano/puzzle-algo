#ifndef CV_TEST_H
#define CV_TEST_H

#include <QtWidgets/QWidget>
#include "ui_cv_test.h"
#include <opencv2\imgproc\imgproc.hpp>
#include <vector>

struct Boundary{
	enum Type {IN,OUT,FLAT};
	Type type;
	std::vector<cv::Point> boundary;
	cv::Point corners[2];
	//metrics
	int length;
	int lengthToPip = -1;
	int lengthFromPip = -1;
	//TODO: color features
	std::vector<cv::Point3i> color;
};

struct PuzzlePiece{
	cv::Mat piece;
	cv::Point corners[4];
	int cornerCount;
	bool valid = false;
	bool processed = false;
	cv::Point massCenter;
	Boundary boundaries[4];
	std::vector<cv::Point> contour;
};

struct PuzzleScan{
	cv::Mat sourceImage;
	std::vector<PuzzlePiece*> pieces;
	int piecesCount = 0;
};

class CV_test : public QWidget
{
	Q_OBJECT

public:
	CV_test(QWidget *parent = 0);
	~CV_test();
	std::vector<PuzzleScan*> images;
	cv::Mat tmpRGB;
	QImage* shown;
	int clickedOn=-1;

	void redrawContours(int,int);
	cv::Point getCurrentSelection();
	void CV_test::getColorComparisonScore(std::vector<cv::Point> boundary, std::vector<cv::Point> corners, int height, int width, cv::Point cur);
protected:
	void mouseReleaseEvent(QMouseEvent*) Q_DECL_OVERRIDE;
	void mouseMoveEvent(QMouseEvent*) Q_DECL_OVERRIDE;
	void mousePressEvent(QMouseEvent*) Q_DECL_OVERRIDE;
private:
	void rebuildList();
	Ui::CV_testClass ui;
	cv::Point LineIntersection(cv::Point, cv::Point, cv::Point, cv::Point);
	double rectangularity(cv::Point a, cv::Point b, cv::Point c, cv::Point d);
public slots:
	void loadImage();
	void switchImage();
	void parseImage();
	void postProcess();
	void getSolverData();
	std::vector<std::vector<cv::Point3i>> solve();
	void generateVisualSolution();
};

#endif // CV_TEST_H
