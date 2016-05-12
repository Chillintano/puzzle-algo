#include "cv_test.h"
#include <qfiledialog.h>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\core\core.hpp>
#include <memory>
#include <iostream>
#include <fstream>
#include <qdebug.h>
#include <qprogressbar.h>
#include <qevent.h>
#include <qpainter.h>
#include <qpaintdevice.h>
#include <qpixmap.h>
#include <Mathematics\GteMinimumAreaBox2.h>
#include <ctime>

CV_test::CV_test(QWidget *parent)
: QWidget(parent)
{
	ui.setupUi(this);
	connect(ui.loadButton, SIGNAL(clicked()), this, SLOT(loadImage()));
	connect(ui.listWidget, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(switchImage()));
	connect(ui.parseButton, SIGNAL(clicked()), this, SLOT(parseImage()));
	connect(ui.processButton, SIGNAL(clicked()), this, SLOT(postProcess()));
	connect(ui.pushButton, SIGNAL(clicked()), this, SLOT(getSolverData()));
	connect(ui.solverButton, SIGNAL(clicked()), this, SLOT(generateVisualSolution()));
}

double CV_test::rectangularity(cv::Point a, cv::Point b, cv::Point c, cv::Point d){
	gte::MinimumAreaBox2<double, double> mab;
	gte::Vector2<double>* gtePoints = new gte::Vector2<double>[4];
	gtePoints[0][0] = d.x; gtePoints[0][1] = d.y;
	gtePoints[1][0] = c.x; gtePoints[1][1] = c.y;
	gtePoints[2][0] = b.x; gtePoints[2][1] = b.y;
	gtePoints[3][0] = a.x; gtePoints[3][1] = a.y;
	double area = abs(0.5*((b.x - a.x)*(c.y - a.y) - (c.x - a.x)*(b.y - a.y))) + abs(0.5*((d.x - a.x)*(c.y - a.y) - (c.x - a.x)*(d.y - a.y)));
	gte::OrientedBox2<double> box = mab(4, gtePoints);
	return area / mab.GetArea();
}

void ClosestPtPointLine(cv::Point a, cv::Point b, cv::Point c, cv::Point &d)
{
	cv::Point ab = b - a;
	cv::Point ca = c - a;
	// Project c onto ab, computing parameterized position d(t) = a + t*(b – a)
	double t = (double)(ca.x*ab.x + ca.y*ab.y) / (ab.x*ab.x + ab.y*ab.y);
	// Compute projected position
	d.x = a.x + t * ab.x;
	d.y = a.y + t * ab.y;
}

double angle(cv::Point a, cv::Point b)
{
	double t = (a.x*b.x + a.y*b.y) / (sqrt((double)a.x*a.x + a.y*a.y)*sqrt((double)b.x*b.x + b.y*b.y));
	if (t<-1) t = -1;
	else if (t> 1) t = 1;
	if (acos(t) < acos(0)){
		if (a.x >= 0 && a.y <= 0) return 3*acos(0)-acos(t);
		else if (a.x >= 0 && a.y>0) return  acos(t) - acos(0);
		else if (a.x < 0 && a.y <= 0) return acos(0) + acos(t);
		else return -(3*acos(0) + acos(t));
	}
	else
	{
		if (a.x >= 0 && a.y <= 0) return acos(t) - 3*acos(0);
		else if (a.x >= 0 && a.y>0) return  acos(0) - acos(t);
		else if (a.x < 0 && a.y <= 0) return (3 * acos(0) - acos(t));
		else return -(acos(0) - acos(t));
	}
}

double simpleAngle(cv::Point a, cv::Point b)
{
	double t = (a.x*b.x + a.y*b.y) / (sqrt((double)a.x*a.x + a.y*a.y)*sqrt((double)b.x*b.x + b.y*b.y));
	if (t<-1) t = -1;
	else if (t> 1) t = 1;
	return acos(t);
}

cv::Point CV_test::LineIntersection(cv::Point pe1, cv::Point pe2, cv::Point ps1, cv::Point ps2){

	// Get A,B,C of first line - points : ps1 to pe1
	double A1 = pe1.y - ps1.y;
	double B1 = ps1.x - pe1.x;
	double C1 = A1*ps1.x + B1*ps1.y;

	// Get A,B,C of second line - points : ps2 to pe2
	double A2 = pe2.y - ps2.y;
	double B2 = ps2.x - pe2.x;
	double C2 = A2*ps2.x + B2*ps2.y;

	// Get delta and check if the lines are parallel
	double delta = A1*B2 - A2*B1;
	if (delta == 0)
		return cv::Point(-1, -1);

	// now return the Vector2 intersection point
	return cv::Point(
		(B2*C1 - B1*C2) / delta,
		(A1*C2 - A2*C1) / delta
		);
}

void CV_test::rebuildList(){
	ui.listWidget->clear();
	for (int i = 0; i < images.size(); i++){
		QString a;
		a.append("Picture ");
		char buf[10];
		itoa(i + 1, buf, 10);
		a.append(buf);
		ui.listWidget->addItem(a);
		for (int j = 0; j < images[i]->pieces.size(); j++){
			QString b;
			b.append("      Piece ");
			char buf2[10];
			itoa(j + 1, buf2, 10);
			b.append(buf2);

			if (rectangularity(images[i]->pieces[j]->corners[0], images[i]->pieces[j]->corners[1], images[i]->pieces[j]->corners[2], images[i]->pieces[j]->corners[3]) < 0.9){
				b.append(" [!]");
			}

			ui.listWidget->addItem(b);
		}
	}
}

void CV_test::loadImage(){
	QStringList filename = QFileDialog::getOpenFileNames(this, tr("Open Image"), "/", tr("Image files (*.png *.jpg *.bmp)"));
	if (filename.empty()) return;
	for (int i = 0; i < filename.size(); i++){
		images.push_back(new PuzzleScan());
		images.back()->sourceImage = cvLoadImage(filename.at(i).toUtf8().data());
	}
	cv::cvtColor(images.back()->sourceImage, tmpRGB, CV_BGR2RGB);
	shown = new QImage((quint8*)tmpRGB.data, tmpRGB.cols, tmpRGB.rows, tmpRGB.step, QImage::Format::Format_RGB888);
	int width = shown->size().width();
	int height = shown->size().height();
	if (shown->size().height()>ui.label->height() || shown->size().width() > ui.label->width()){
		if (width > ui.label->width()){
			int tmp = ui.label->width()*((double)height / width);
			width = ui.label->width();
			height = tmp;
		}
		if (height > ui.label->height()){
			int tmp = ui.label->height()*((double)width / height);
			width = tmp;
			height = ui.label->height();
		}
	}
	rebuildList();
	ui.label->setPixmap(QPixmap::fromImage(*shown).scaled(QSize(width, height)));
	qDebug() << getCurrentSelection().x << "  " << getCurrentSelection().y;
}

void CV_test::parseImage(){
	ui.progressBar->setEnabled(true);
	ui.progressBar->setValue(0);
	int count = -1;
	int imageToParse = 0;
	cv::Mat src, src_gray, src_median, src_edges, src_dilated;

	for (int i = 0; i < images.size(); i++){
		if (count + images[i]->piecesCount + 1 >= ui.listWidget->currentRow()){
			int pos = ui.listWidget->currentRow() - count - 2;
			if (pos < 0){
				imageToParse = i;
				src = images[i]->sourceImage.clone();
				images[i]->pieces.clear();
			}
			break;
		}
		else{
			count += images[i]->piecesCount + 1;
		}
	}
	int medianKernelSize = 13;

	medianBlur(src, src_median, medianKernelSize);
	qDebug() << "Median filter applied";
	ui.progressBar->setValue(ui.progressBar->value() + 30);
	std::vector<cv::Mat> bgr_images(3);
	split(src_median, bgr_images);
	std::vector<cv::Mat> bgr_edges(3);
	cv::Canny(bgr_images[0], bgr_edges[0], 0, 60);
	cv::Canny(bgr_images[1], bgr_edges[1], 0, 60);
	cv::Canny(bgr_images[2], bgr_edges[2], 0, 60);

	cv::Mat in[] = { bgr_edges[0], bgr_edges[1], bgr_edges[2] };
	src_edges = src_median.clone();
	cv::Mat out[] = { src_edges };
	int from_to[] = { 0, 0, 1, 1, 2, 2 };
	cv::mixChannels(in, 3, out, 1, from_to, 3);


	/* cvtColor(src_median, src_gray, CV_BGR2GRAY);
	cout << "Converted to grayscale" << endl;
	imwrite("2gray.jpg", src_gray);
	Canny(src_gray, src_edges, 40, 60);*/
	qDebug() << "Edged calculated" << endl;
	ui.progressBar->setValue(ui.progressBar->value() + 5);
	cv::cvtColor(src_edges, src_edges, CV_BGR2GRAY);
	cv::dilate(src_edges, src_dilated, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9)));
	qDebug() << "Dilated" << endl;
	ui.progressBar->setValue(ui.progressBar->value() + 5);
	cv::Mat src_dilated_copy = src_dilated.clone();
	cv::floodFill(src_dilated, cv::Point(1, 1), cv::Scalar(255, 255, 255));
	qDebug() << "Floodfilled" << endl;
	ui.progressBar->setValue(ui.progressBar->value() + 5);
	cv::Mat src_dilated2;
	cv::subtract(src_dilated, src_dilated_copy, src_dilated2);
	qDebug() << "Combined" << endl;
	ui.progressBar->setValue(ui.progressBar->value() + 5);
	cv::dilate(src_dilated2, src_dilated2, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9)));
	qDebug() << "Eroded" << endl;

	/*char* source_window = "Source";
	namedWindow(source_window, CV_WINDOW_FREERATIO);
	imshow(source_window, src_dilated);*/

	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(src_dilated2, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
	std::vector<cv::Rect> boundRect(contours.size());
	std::vector<std::vector<cv::Point> > contours_poly(contours.size());

	int xTotal = 0;
	int yTotal = 0;

	//get average contour dimensions to filter out noise
	for (int i = 0; i < contours.size(); i++)
	{
		cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);
		boundRect[i] = cv::boundingRect(cv::Mat(contours_poly[i]));
		if (i>0){
			xTotal += boundRect[i].width;
			yTotal += boundRect[i].height;
		}
	}

	xTotal /= contours.size();
	yTotal /= contours.size();
	int avg_size = xTotal * yTotal;
	cv::Mat drawing = cv::Mat::zeros(src_edges.size(), CV_8UC3);
	for (int i = 1; i< contours.size(); i++)
	{
		int size = boundRect[i].width * boundRect[i].height;
		if (size>avg_size*0.5){
			//Mat cutout = Mat::zeros(cv::Size((boundRect[i].br() - boundRect[i].tl()).x, (boundRect[i].br() - boundRect[i].tl()).y), CV_8UC3);

			//contours contain coorinates from src_edges image, so we create an empty canvas of src_edges.size() and draw only needed contour on it
			cv::Mat cutout = cv::Mat::zeros(src_edges.size(), CV_8UC3);
			cv::floodFill(cutout, cv::Point(1, 1), cv::Scalar(255, 255, 255));
			cv::drawContours(cutout, contours, i, cv::Scalar(0, 0, 0), 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
			//cutout(boundRect[i]).copyTo(cutout);

			//cut required piece with 35px border
			int left = boundRect[i].tl().x - 35; left = left > 0 ? left : 1;
			int top = boundRect[i].tl().y - 35; top = top > 0 ? top : 1;
			int width = (boundRect[i].br() - boundRect[i].tl()).x + 70; width = (left + width) < src.size().width ? width : src.size().width - left - 1;
			int height = (boundRect[i].br() - boundRect[i].tl()).y + 70; height = (top + height) < src.size().height ? height : src.size().height - top - 1;
			qDebug() << i << endl;
			qDebug() << width << "  " << height << endl;

			//erode to smooth edges and sharpen corners
			cutout(cv::Rect(left, top, width, height)).copyTo(cutout);
			cv::erode(cutout, cutout, cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(5, 5)));
			cv::floodFill(cutout, cv::Point(1, 1), cv::Scalar(0, 0, 0));
			cv::erode(cutout, cutout, cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(5, 5)));
			cv::erode(cutout, cutout, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(cv::max(width, height)*0.025, cv::max(width, height)*0.025)));
			
			cv::rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(0, 200, 0), 2, 8, 0);
			cv::Mat tmp = src_median(cv::Rect(left, top, width, height)).clone();
			cv::Mat tmp2, tmp3;
			
			//use corner detection
			cv::cvtColor(cutout, tmp3, CV_BGR2GRAY);
			std::vector<cv::Point> corners;
			goodFeaturesToTrack(tmp3, corners, 4, 0.01, std::min((boundRect[i].br() - boundRect[i].tl()).x, (boundRect[i].br() - boundRect[i].tl()).y)*0.6, cv::noArray(), 11, false);
			cv::Point* top_corner = &corners[0];
			if (corners.size() > 0){
				for (int i = 1; i < corners.size(); i++){
					if (corners[i].y < top_corner->y) top_corner = &corners[i];
				}
			}

			cv::Point* corner2 = new cv::Point(9999, 9999);
			if (corners.size()>1){
				for (int i = 0; i < corners.size(); i++){
					if (corners[i].y < corner2->y && corners[i].x>(top_corner->x - 0.1*width) && &corners[i] != top_corner) corner2 = &corners[i];
				}
				if (corner2->x == 9999 && corner2->y == 9999){
					for (auto x : corners){
						if (x.x != top_corner->x && x.y != top_corner->y) corner2 = &x;
					}
				}
			}

			cv::Point* corner3 = new cv::Point(0, 9999);
			if (corners.size() > 2){
				for (int i = 0; i < corners.size(); i++){
					if (corners[i].y >(corner2->y - 0.1*height) && corners[i].x > (corner3->x - 0.1*width) && &corners[i] != top_corner && &corners[i] != corner2) corner3 = &corners[i];
				}
				if (corner3->x == 0 && corner3->y == 9999){
					for (auto x : corners){
						if (x.x != top_corner->x && x.y != top_corner->y && x.x != corner2->x && x.y != corner2->y) corner3 = &x;
					}
				}
			}

			cv::Point* corner4 = new cv::Point(0, 9995);
			if (corners.size() > 3){
				for (int i = 0; i < corners.size(); i++){
					if (&corners[i] != corner3 && &corners[i] != top_corner && &corners[i] != corner2) corner4 = &corners[i];
				}
				if (corner4->x == 0 && corner4->y == 9995){
					for (auto x : corners){
						if (x.x != top_corner->x && x.y != top_corner->y && x.x != corner2->x && x.y != corner2->y && x.x != corner3->x && x.y != corner3->y) corner4 = &x;
					}
				}
			}


			std::vector<cv::Point> saved_corners;

			double max_rect = rectangularity(*top_corner, *corner2, *corner3, *corner4);
			saved_corners.swap(corners);

			//if rectangularity score is low, there is high probability that some corners are detected incorrectly: perform a grid search in a range of detector parameters
			if (rectangularity(*top_corner, *corner2, *corner3, *corner4) < 0.9){
				double param = 0.1;
				double param2 = 0.7;
				bool exit = false;
				while (!exit){

					corners.clear();
					goodFeaturesToTrack(tmp3, corners, 4, param, std::min((boundRect[i].br() - boundRect[i].tl()).x, (boundRect[i].br() - boundRect[i].tl()).y)*param2, cv::noArray(), cv::max(width, height)*0.01, false);
					param *= 0.6;

					top_corner = &corners[0];
					if (corners.size() > 0){
						for (int i = 1; i < corners.size(); i++){
							if (corners[i].y < top_corner->y) top_corner = &corners[i];
						}
					}

					corner2 = new cv::Point(9999, 9997);
					if (corners.size()>1){
						for (int i = 0; i < corners.size(); i++){
							if (corners[i].y < corner2->y && corners[i].x>(top_corner->x - 0.1*width) && &corners[i] != top_corner) corner2 = &corners[i];
						}
						if (corner2->x == 9999 && corner2->y == 9997){
							for (auto x : corners){
								if (x.x != top_corner->x && x.y != top_corner->y) corner2 = &x;
							}
						}
					}

					corner3 = new cv::Point(0, 9999);
					if (corners.size() > 2){
						for (int i = 0; i < corners.size(); i++){
							if (corners[i].y >(corner2->y - 0.1*height) && corners[i].x > (corner3->x - 0.1*width) && &corners[i] != top_corner && &corners[i] != corner2) corner3 = &corners[i];
						}
						if (corner3->x == 0 && corner3->y == 9999){
							for (auto x : corners){
								if (x.x != top_corner->x && x.y != top_corner->y && x.x != corner2->x && x.y != corner2->y) corner3 = &x;
							}
						}
					}

					corner4 = new cv::Point(0, 9995);
					if (corners.size() > 3){
						for (int i = 0; i < corners.size(); i++){
							if (&corners[i] != corner3 && &corners[i] != top_corner && &corners[i] != corner2) corner4 = &corners[i];
						}
						if (corner4->x == 0 && corner4->y == 9995){
							for (auto x : corners){
								if (x.x != top_corner->x && x.y != top_corner->y && x.x != corner2->x && x.y != corner2->y && x.x != corner3->x && x.y != corner3->y) corner4 = &x;
							}
						}
					}

					double rect = rectangularity(*top_corner, *corner2, *corner3, *corner4);

					if (rect > max_rect) {
						saved_corners.swap(corners);
					}

					if (param < 0.0001){
						param2 -= 0.03;
						param = 0.15;
					}
					if (param2 < 0.55)
						exit = true;

				}
			}
			/*for (auto x : corners){
			cv::circle(tmp2, cv::Point(x.x, x.y), 5, cv::Scalar(0, 0, 255), 2);
			}*/

			//eroded version required to get color comparison metrics; perfect way to follow the direction of boundaries
			cv::Mat cutout_edges1, cutout_binary1;
			cv::cvtColor(cutout, cutout_binary1, cv::COLOR_BGR2GRAY);
			std::vector<std::vector<cv::Point>> cutout_contour1;
			cv::threshold(cutout_binary1, cutout_binary1, 200, 255, cv::THRESH_BINARY);
			cv::Canny(cutout, cutout_edges1, 0, 60);
			try{
				cv::findContours(cutout_binary1, cutout_contour1, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
			}
			catch (cv::Exception e){
				qDebug() << e.err.data();
			}

			//dilate back to save original dimensions as much as possible
			cv::dilate(cutout, cutout, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(cv::max(width, height)*0.025, cv::max(width, height)*0.025)));
			cv::dilate(cutout, cutout, cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(5, 5)));
			tmp.copyTo(tmp2, cutout);

			images[imageToParse]->pieces.push_back(new PuzzlePiece());
			images[imageToParse]->piecesCount++;
			images[imageToParse]->pieces.back()->piece = tmp2;
			getColorComparisonScore(cutout_contour1[0], saved_corners, height, width, cv::Point(imageToParse, images[imageToParse]->pieces.size() - 1));

			cv::Mat cutout_edges, cutout_binary;
			cv::cvtColor(cutout, cutout_binary, cv::COLOR_BGR2GRAY);
			std::vector<std::vector<cv::Point>> cutout_contour;
			cv::threshold(cutout_binary, cutout_binary, 200, 255, cv::THRESH_BINARY);
			cv::Canny(cutout, cutout_edges, 0, 60);
			try{
				cv::findContours(cutout_binary, cutout_contour, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
			}
			catch (cv::Exception e){
				qDebug() << e.err.data();
			}
			/*drawContours(tmp2, cutout_contour, 0, Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), 3);*/

			if (cutout_contour.size() > 1){
				int biggest = 0, num = 0;
				for (int i = 0; i<cutout_contour.size(); i++){
					if (cutout_contour[i].size()>biggest){
						biggest = cutout_contour[i].size();
						num = i;
					}
				}
				if (num != 0) {
					cutout_contour.insert(cutout_contour.begin(), cutout_contour[num]);
					cutout_contour.erase(cutout_contour.begin() + (num + 1));
				}
			}


			int numContours = cutout_contour.size();
			while (numContours > 1){
				bool changed = false;
				for (int r = 1; r < cutout_contour.size(); r++){
					if ((pow(cutout_contour[0][cutout_contour[0].size() - 1].x - cutout_contour[r][0].x, 2) + pow(cutout_contour[0][cutout_contour[0].size() - 1].y - cutout_contour[r][0].y, 2)) <= 16 ||
						(pow(cutout_contour[0][cutout_contour[0].size() - 1].x - cutout_contour[r][cutout_contour[r].size() - 1].x, 2) + pow(cutout_contour[0][cutout_contour[0].size() - 1].y - cutout_contour[r][cutout_contour[r].size() - 1].y, 2)) <= 16){

						if ((pow(cutout_contour[0][cutout_contour[0].size() - 1].x - cutout_contour[r][0].x, 2) + pow(cutout_contour[0][cutout_contour[0].size() - 1].y - cutout_contour[r][0].y, 2)) < (pow(cutout_contour[0][cutout_contour[0].size() - 1].x - cutout_contour[r][cutout_contour[r].size() - 1].x, 2) + pow(cutout_contour[0][cutout_contour[0].size() - 1].y - cutout_contour[r][cutout_contour[r].size() - 1].y, 2))){
							for (int s = 0; s < cutout_contour[r].size(); s++){
								cutout_contour[0].push_back(cutout_contour[r][s]);
							}
						}
						else{
							for (int s = cutout_contour[r].size() - 1; s >= 0; s--){
								cutout_contour[0].push_back(cutout_contour[r][s]);
							}
						}

						cutout_contour.erase(cutout_contour.begin() + r);
						numContours--;
						changed = true;
					}
				}
				if (!changed) break;
			}
			//cv::drawContours(tmp2, cutout_contour, 0, cv::Scalar(0, 0, 255),2);

			cv::Moments a = cv::moments(cutout_contour[0], false);
			double mu20 = a.m20 / a.m00 - (a.m10 / a.m00)*(a.m10 / a.m00);
			double mu02 = a.m02 / a.m00 - (a.m01 / a.m00)*(a.m01 / a.m00);
			double mu11 = a.m11 / a.m00 - (a.m10 / a.m00)*(a.m01 / a.m00);

			double xc = a.m10 / a.m00;
			double yc = a.m01 / a.m00;
			double l1 = sqrt(2 * (a.mu20 + a.mu02 + sqrt((a.mu20 - a.mu02)*(a.mu20 - a.mu02) + 4 * a.mu11*a.mu11)) / a.m00);
			double l2 = sqrt(2 * (a.mu20 + a.mu02 - sqrt((a.mu20 - a.mu02)*(a.mu20 - a.mu02) + 4 * a.mu11*a.mu11)) / a.m00);
			double lambda1 = sqrt(((mu20 + mu02) / 2) + sqrt(4 * mu11*mu11 - (mu20 - mu02) * (mu20 - mu02)));
			double lambda2 = sqrt(((mu20 + mu02) / 2) - sqrt(4 * mu11*mu11 - (mu20 - mu02) * (mu20 - mu02)));
			double theta = 0.5*std::atan(2 * mu11 / (mu20 - mu02));
			cv::circle(tmp2, cv::Point(xc, yc), 10, cv::Scalar(0, 0, 255), 5);
			images[imageToParse]->pieces.back()->massCenter = cv::Point(xc, yc);

			while (saved_corners.size()<4){
				saved_corners.push_back(cv::Point(0, 0));
			}

			top_corner = &saved_corners[0];
			if (saved_corners.size() > 0){
				for (int i = 1; i < saved_corners.size(); i++){
					if (saved_corners[i].y < top_corner->y) top_corner = &saved_corners[i];
				}
				cv::circle(tmp2, cv::Point(top_corner->x, top_corner->y), 7, cv::Scalar(0, 255, 255), 2);
				//images[imageToParse]->pieces.back()->corners[0] = *top_corner;
			}

			corner2 = new cv::Point(9999, 9999);
			if (saved_corners.size()>1){
				for (int i = 0; i < saved_corners.size(); i++){
					if (saved_corners[i].y < corner2->y && saved_corners[i].x>(top_corner->x - 0.1*width) && &saved_corners[i] != top_corner) corner2 = &saved_corners[i];
				}
				if (corner2->x == 9999 && corner2->y == 9999){
					for (auto x : saved_corners){
						if (x.x != top_corner->x && x.y != top_corner->y) corner2 = &x;
					}
				}
				cv::circle(tmp2, cv::Point(corner2->x, corner2->y), 7, cv::Scalar(255, 0, 255), 2);
				//images[imageToParse]->pieces.back()->corners[1] = *corner2;
			}

			corner3 = new cv::Point(0, 9999);
			if (saved_corners.size()>2){
				for (int i = 0; i < saved_corners.size(); i++){
					if (saved_corners[i].y >(corner2->y - 0.1*height) && saved_corners[i].x > (corner3->x - 0.1*width) && &saved_corners[i] != top_corner && &saved_corners[i] != corner2) corner3 = &saved_corners[i];
				}
				if (corner3->x == 0 && corner3->y == 9999){
					for (auto x : saved_corners){
						if (x.x != top_corner->x && x.y != top_corner->y && x.x != corner2->x && x.y != corner2->y) corner3 = &x;
					}
				}
				cv::circle(tmp2, cv::Point(corner3->x, corner3->y), 7, cv::Scalar(255, 255, 0), 2);
				//images[imageToParse]->pieces.back()->corners[2] = *corner3;
			}

			corner4 = new cv::Point(0, 9999);
			if (saved_corners.size() > 3){
				for (int i = 0; i < saved_corners.size(); i++){
					if (&saved_corners[i] != corner3 && &saved_corners[i] != top_corner && &saved_corners[i] != corner2) corner4 = &saved_corners[i];
				}
				if (corner4->x == 0 && corner4->y == 9999){
					for (auto x : saved_corners){
						if (x.x != top_corner->x && x.y != top_corner->y && x.x != corner2->x && x.y != corner2->y && x.x != corner3->x && x.y != corner3->y) corner4 = &x;
					}
				}
				//images[imageToParse]->pieces.back()->corners[3] = *corner4;
			}

			qDebug() << "RECT:" << rectangularity(*top_corner, *corner2, *corner3, *corner4);


			
			cv::Point propagation_vector = images[imageToParse]->pieces.back()->massCenter - *top_corner;
			propagation_vector.x = ((cv::max(width, height)*0.025 + 5))*propagation_vector.x / sqrt(propagation_vector.x*propagation_vector.x + propagation_vector.y*propagation_vector.y);
			propagation_vector.y = ((cv::max(width, height)*0.025 + 5))*propagation_vector.y / sqrt(propagation_vector.x*propagation_vector.x + propagation_vector.y*propagation_vector.y);
			(*top_corner).x = (*top_corner).x - propagation_vector.x;
			(*top_corner).y = (*top_corner).y - propagation_vector.y;

			propagation_vector = images[imageToParse]->pieces.back()->massCenter - *corner2;
			propagation_vector.x = ((cv::max(width, height)*0.025 + 5))*propagation_vector.x / sqrt(propagation_vector.x*propagation_vector.x + propagation_vector.y*propagation_vector.y);
			propagation_vector.y = ((cv::max(width, height)*0.025 + 5))*propagation_vector.y / sqrt(propagation_vector.x*propagation_vector.x + propagation_vector.y*propagation_vector.y);
			(*corner2).x = (*corner2).x - propagation_vector.x;
			(*corner2).y = (*corner2).y - propagation_vector.y;

			propagation_vector = images[imageToParse]->pieces.back()->massCenter - *corner3;
			propagation_vector.x = ((cv::max(width, height)*0.025 + 5))*propagation_vector.x / sqrt(propagation_vector.x*propagation_vector.x + propagation_vector.y*propagation_vector.y);
			propagation_vector.y = ((cv::max(width, height)*0.025 + 5))*propagation_vector.y / sqrt(propagation_vector.x*propagation_vector.x + propagation_vector.y*propagation_vector.y);
			(*corner3).x = (*corner3).x - propagation_vector.x;
			(*corner3).y = (*corner3).y - propagation_vector.y;

			propagation_vector = images[imageToParse]->pieces.back()->massCenter - *corner4;
			propagation_vector.x = ((cv::max(width, height)*0.025 + 5))*propagation_vector.x / sqrt(propagation_vector.x*propagation_vector.x + propagation_vector.y*propagation_vector.y);
			propagation_vector.y = ((cv::max(width, height)*0.025 + 5))*propagation_vector.y / sqrt(propagation_vector.x*propagation_vector.x + propagation_vector.y*propagation_vector.y);
			(*corner4).x = (*corner4).x - propagation_vector.x;
			(*corner4).y = (*corner4).y - propagation_vector.y;
			
			int c1, c2, c3, c4;
			double dist1, dist2, dist3, dist4;
			dist1 = dist2 = dist3 = dist4 = height;
			c1 = c2 = c3 = c4 = 0;
			for (int i = 0; i < cutout_contour[0].size(); i++){
				if (sqrt(pow((cutout_contour[0][i] - (*top_corner)).x, 2) + pow((cutout_contour[0][i] - (*top_corner)).y, 2)) < dist1){
					c1 = i;
					dist1 = sqrt(pow((cutout_contour[0][i] - (*top_corner)).x, 2) + pow((cutout_contour[0][i] - (*top_corner)).y, 2));
				}
				if (sqrt(pow((cutout_contour[0][i] - (*corner2)).x, 2) + pow((cutout_contour[0][i] - (*corner2)).y, 2)) < dist2){
					c2 = i;
					dist2 = sqrt(pow((cutout_contour[0][i] - (*corner2)).x, 2) + pow((cutout_contour[0][i] - (*corner2)).y, 2));
				}
				if (sqrt(pow((cutout_contour[0][i] - (*corner3)).x, 2) + pow((cutout_contour[0][i] - (*corner3)).y, 2)) < dist3){
					c3 = i;
					dist3 = sqrt(pow((cutout_contour[0][i] - (*corner3)).x, 2) + pow((cutout_contour[0][i] - (*corner3)).y, 2));
				}
				if (sqrt(pow((cutout_contour[0][i] - (*corner4)).x, 2) + pow((cutout_contour[0][i] - (*corner4)).y, 2)) < dist4){
					c4 = i;
					dist4 = sqrt(pow((cutout_contour[0][i] - (*corner4)).x, 2) + pow((cutout_contour[0][i] - (*corner4)).y, 2));
				}
			}

			std::vector<std::vector<cv::Point>> edges;
			edges.push_back(std::vector<cv::Point>());
			edges.push_back(std::vector<cv::Point>());
			edges.push_back(std::vector<cv::Point>());
			edges.push_back(std::vector<cv::Point>());
			std::vector<int> tst;
			tst.push_back(c1); tst.push_back(c2); tst.push_back(c3); tst.push_back(c4);
			qSort(tst);

			qDebug() << "---------------";

			images[imageToParse]->pieces.back()->corners[0] = cutout_contour[0][tst[0]];
			images[imageToParse]->pieces.back()->corners[1] = cutout_contour[0][tst[1]];
			images[imageToParse]->pieces.back()->corners[2] = cutout_contour[0][tst[2]];
			images[imageToParse]->pieces.back()->corners[3] = cutout_contour[0][tst[3]];

			images[imageToParse]->pieces.back()->boundaries[0].corners[0] = cutout_contour[0][tst[0]];
			images[imageToParse]->pieces.back()->boundaries[0].corners[1] = cutout_contour[0][tst[1]];
			images[imageToParse]->pieces.back()->boundaries[1].corners[0] = cutout_contour[0][tst[1]];
			images[imageToParse]->pieces.back()->boundaries[1].corners[1] = cutout_contour[0][tst[2]];
			images[imageToParse]->pieces.back()->boundaries[2].corners[0] = cutout_contour[0][tst[2]];
			images[imageToParse]->pieces.back()->boundaries[2].corners[1] = cutout_contour[0][tst[3]];
			images[imageToParse]->pieces.back()->boundaries[3].corners[0] = cutout_contour[0][tst[3]];
			images[imageToParse]->pieces.back()->boundaries[3].corners[1] = cutout_contour[0][tst[0]];
			for (int i = tst[0]; i != tst[1]; i = (i + 1) < cutout_contour[0].size() ? i + 1 : 0){
				images[imageToParse]->pieces.back()->boundaries[0].boundary.push_back(cutout_contour[0][i]);
				edges[0].push_back(cutout_contour[0][i]);
			}
			for (int i = tst[1]; i != tst[0]; i = (i - 1) >= 0 ? i - 1 : cutout_contour[0].size() - 1){
				edges[0].push_back(cutout_contour[0][i]);
			}
			for (int i = tst[1]; i != tst[2]; i = (i + 1) < cutout_contour[0].size() ? i + 1 : 0){
				images[imageToParse]->pieces.back()->boundaries[1].boundary.push_back(cutout_contour[0][i]);
				edges[1].push_back(cutout_contour[0][i]);
			}
			for (int i = tst[2]; i != tst[1]; i = (i - 1) >= 0 ? i - 1 : cutout_contour[0].size() - 1){
				edges[1].push_back(cutout_contour[0][i]);
			}
			for (int i = tst[2]; i != tst[3]; i = (i + 1) < cutout_contour[0].size() ? i + 1 : 0){
				images[imageToParse]->pieces.back()->boundaries[2].boundary.push_back(cutout_contour[0][i]);
				edges[2].push_back(cutout_contour[0][i]);
			}
			for (int i = tst[3]; i != tst[2]; i = (i - 1) >= 0 ? i - 1 : cutout_contour[0].size() - 1){
				edges[2].push_back(cutout_contour[0][i]);
			}
			for (int i = tst[3]; i != tst[0]; i = (i + 1) < cutout_contour[0].size() ? i + 1 : 0){
				images[imageToParse]->pieces.back()->boundaries[3].boundary.push_back(cutout_contour[0][i]);
				edges[3].push_back(cutout_contour[0][i]);
			}
			for (int i = tst[0]; i != tst[3]; i = (i - 1) >= 0 ? i - 1 : cutout_contour[0].size() - 1){
				edges[3].push_back(cutout_contour[0][i]);
			}
			//cv::drawContours(tmp2, edges, 0, cv::Scalar(0, 0, 255), 3);
			//cv::drawContours(tmp2, edges, 1, cv::Scalar(0, 255, 255), 3);
			//cv::drawContours(tmp2, edges, 2, cv::Scalar(255, 0, 255), 3);
			//cv::drawContours(tmp2, edges, 3, cv::Scalar(0, 255, 0), 3);

			images[imageToParse]->pieces.back()->contour = cutout_contour[0];

			//strcat(buf, ".jpg");
			//imwrite(buf, tmp2);
		}
		ui.progressBar->setValue(ui.progressBar->value() + cvCeil((double)50 / contours.size()));
	}
	ui.progressBar->setValue(ui.progressBar->value() + (double)50 / contours.size());
	ui.progressBar->setEnabled(false);
	rebuildList();
}

void CV_test::redrawContours(int pic, int piece){
	cv::Moments a = cv::moments(images[pic]->pieces[piece]->contour, false);
	double xc = a.m10 / a.m00;
	double yc = a.m01 / a.m00;

	std::vector<int> c;
	c.push_back(0); c.push_back(0); c.push_back(0); c.push_back(0);
	double dist[4] = { shown->height(), shown->height(), shown->height(), shown->height() };
	double dist1, dist2, dist3, dist4;
	dist1 = dist2 = dist3 = dist4 = shown->height();
	for (int i = 0; i < images[pic]->pieces[piece]->contour.size(); i++){
		for (int j = 0; j < 4; j++){
			if (sqrt(pow((images[pic]->pieces[piece]->contour[i] - images[pic]->pieces[piece]->corners[j]).x, 2) + pow((images[pic]->pieces[piece]->contour[i] - images[pic]->pieces[piece]->corners[j]).y, 2)) < dist[j]){
				c[j] = i;
				dist[j] = sqrt(pow((images[pic]->pieces[piece]->contour[i] - images[pic]->pieces[piece]->corners[j]).x, 2) + pow((images[pic]->pieces[piece]->contour[i] - images[pic]->pieces[piece]->corners[j]).y, 2));
			}
		}
	}
	qSort(c);
	for (int j = 0; j < 4; j++){
		images[pic]->pieces[piece]->corners[j].x = images[pic]->pieces[piece]->contour[c[j]].x;
		images[pic]->pieces[piece]->corners[j].y = images[pic]->pieces[piece]->contour[c[j]].y;
	}
	int row = 0;
	for (int i = 0; i < pic; i++){
		++row;
		row += images[i]->piecesCount;
	}
	row += piece;
	qDebug() << ui.listWidget->item(row)->data(Qt::DisplayRole).value<QString>();
	QString t = "      Piece ";
	char buf[10];
	itoa(piece + 1, buf, 10);
	t.append(buf);
	if (rectangularity(images[pic]->pieces[piece]->corners[0], images[pic]->pieces[piece]->corners[1], images[pic]->pieces[piece]->corners[2], images[pic]->pieces[piece]->corners[3]) < 0.9){
		t.append(" [!]");
		images[pic]->pieces[piece]->valid = false;
	}
	else{
		images[pic]->pieces[piece]->valid = true;
	}
	ui.listWidget->currentItem()->setText(t);

	std::vector<std::vector<cv::Point>> edges;
	edges.push_back(std::vector<cv::Point>());
	edges.push_back(std::vector<cv::Point>());
	edges.push_back(std::vector<cv::Point>());
	edges.push_back(std::vector<cv::Point>());
	std::vector<int> tst;
	for (int j = 0; j < 4; j++){
		tst.push_back(c[j]);
	}
	qSort(tst);
	qDebug() << tst[0] << " " << tst[1] << " " << tst[2] << " " << tst[3];
	qDebug() << "---------------";
	images[pic]->pieces[piece]->boundaries[0].corners[0] = images[pic]->pieces[piece]->contour[tst[0]];
	images[pic]->pieces[piece]->boundaries[0].corners[1] = images[pic]->pieces[piece]->contour[tst[1]];
	images[pic]->pieces[piece]->boundaries[1].corners[0] = images[pic]->pieces[piece]->contour[tst[1]];
	images[pic]->pieces[piece]->boundaries[1].corners[1] = images[pic]->pieces[piece]->contour[tst[2]];
	images[pic]->pieces[piece]->boundaries[2].corners[0] = images[pic]->pieces[piece]->contour[tst[2]];
	images[pic]->pieces[piece]->boundaries[2].corners[1] = images[pic]->pieces[piece]->contour[tst[3]];
	images[pic]->pieces[piece]->boundaries[3].corners[0] = images[pic]->pieces[piece]->contour[tst[3]];
	images[pic]->pieces[piece]->boundaries[3].corners[1] = images[pic]->pieces[piece]->contour[tst[0]];
	for (int i = tst[0]; i != tst[1]; i = (i + 1) < images[pic]->pieces[piece]->contour.size() ? i + 1 : 0){
		images[pic]->pieces[piece]->boundaries[0].boundary.push_back(images[pic]->pieces[piece]->contour[i]);
		edges[0].push_back(images[pic]->pieces[piece]->contour[i]);
	}
	for (int i = tst[1]; i != tst[0]; i = (i - 1) >= 0 ? i - 1 : images[pic]->pieces[piece]->contour.size() - 1){
		edges[0].push_back(images[pic]->pieces[piece]->contour[i]);
	}
	for (int i = tst[1]; i != tst[2]; i = (i + 1) < images[pic]->pieces[piece]->contour.size() ? i + 1 : 0){
		images[pic]->pieces[piece]->boundaries[1].boundary.push_back(images[pic]->pieces[piece]->contour[i]);
		edges[1].push_back(images[pic]->pieces[piece]->contour[i]);
	}
	for (int i = tst[2]; i != tst[1]; i = (i - 1) >= 0 ? i - 1 : images[pic]->pieces[piece]->contour.size() - 1){
		edges[1].push_back(images[pic]->pieces[piece]->contour[i]);
	}
	for (int i = tst[2]; i != tst[3]; i = (i + 1) < images[pic]->pieces[piece]->contour.size() ? i + 1 : 0){
		images[pic]->pieces[piece]->boundaries[2].boundary.push_back(images[pic]->pieces[piece]->contour[i]);
		edges[2].push_back(images[pic]->pieces[piece]->contour[i]);
	}
	for (int i = tst[3]; i != tst[2]; i = (i - 1) >= 0 ? i - 1 : images[pic]->pieces[piece]->contour.size() - 1){
		edges[2].push_back(images[pic]->pieces[piece]->contour[i]);
	}
	for (int i = tst[3]; i != tst[0]; i = (i + 1) < images[pic]->pieces[piece]->contour.size() ? i + 1 : 0){
		images[pic]->pieces[piece]->boundaries[3].boundary.push_back(images[pic]->pieces[piece]->contour[i]);
		edges[3].push_back(images[pic]->pieces[piece]->contour[i]);
	}
	for (int i = tst[0]; i != tst[3]; i = (i - 1) >= 0 ? i - 1 : images[pic]->pieces[piece]->contour.size() - 1){
		edges[3].push_back(images[pic]->pieces[piece]->contour[i]);
	}

	//cv::drawContours((images[pic]->pieces[piece]->piece), edges, 0, cv::Scalar(0, 0, 255), 3);
	//cv::drawContours((images[pic]->pieces[piece]->piece), edges, 1, cv::Scalar(0, 255, 255), 3);
	//cv::drawContours((images[pic]->pieces[piece]->piece), edges, 2, cv::Scalar(255, 0, 255), 3);
	//cv::drawContours((images[pic]->pieces[piece]->piece), edges, 3, cv::Scalar(0, 255, 0), 3);

	
}

void CV_test::postProcess(){
	cv::Point cur = getCurrentSelection();

	
	for (int x = 0; x < images.size(); ++x){
		for (int y = 0; y < images[x]->pieces.size(); ++y){
			if (images[x]->pieces[y]->processed == false){
				for (int j = 0; j < 4; j++){
					//find intersection between line connecting corners and line perpendicular to it, given that second line contains mass center
					cv::Point intersection;
					ClosestPtPointLine(images[x]->pieces[y]->boundaries[j].corners[0], images[x]->pieces[y]->boundaries[j].corners[1], images[x]->pieces[y]->massCenter, intersection);
					//find rotation angle 
					double ang = angle(images[x]->pieces[y]->massCenter - intersection, cv::Point(1, 0));
					//TODO:add checks that boudary is correctly aligned
					double tt = cos(ang);
					//rotate intersection point to check alignment
					intersection.x = intersection.x * cos(ang) + intersection.y * sin(ang) + (1 - cos(ang))*images[x]->pieces[y]->massCenter.x - sin(ang)*images[x]->pieces[y]->massCenter.y;
					intersection.y = -intersection.x * sin(ang) + intersection.y * cos(ang) + sin(ang)*images[x]->pieces[y]->massCenter.x + (1 - cos(ang))*images[x]->pieces[y]->massCenter.y;
					//if (intersection.y<1)

					//rotate each point of boundary
					for (int i = 0; i < images[x]->pieces[y]->boundaries[j].boundary.size(); i++){
						cv::Point tmp;
						tmp.x = images[x]->pieces[y]->boundaries[j].boundary[i].x * cos(ang) + images[x]->pieces[y]->boundaries[j].boundary[i].y * sin(ang) + (1 - cos(ang))*images[x]->pieces[y]->massCenter.x - sin(ang)*images[x]->pieces[y]->massCenter.y;
						tmp.y = -images[x]->pieces[y]->boundaries[j].boundary[i].x * sin(ang) + images[x]->pieces[y]->boundaries[j].boundary[i].y * cos(ang) + sin(ang)*images[x]->pieces[y]->massCenter.x + (1 - cos(ang))*images[x]->pieces[y]->massCenter.y;
						images[x]->pieces[y]->boundaries[j].boundary[i] = tmp;
					}

					//translate the leftmost point to (0,0) and at the same time calculate max deviation from X axis
					cv::Point target(std::min(images[x]->pieces[y]->boundaries[j].boundary[0].x, images[x]->pieces[y]->boundaries[j].boundary.back().x), images[x]->pieces[y]->boundaries[j].boundary[0].y);
					int maxY = 0;
					for (int i = 0; i < images[x]->pieces[y]->boundaries[j].boundary.size(); i++){
						images[x]->pieces[y]->boundaries[j].boundary[i].x -= target.x;
						images[x]->pieces[y]->boundaries[j].boundary[i].y -= target.y;
						if (std::abs(images[x]->pieces[y]->boundaries[j].boundary[i].y)>std::abs(maxY)){
							maxY = images[x]->pieces[y]->boundaries[j].boundary[i].y;
						}
					}

					images[x]->pieces[y]->boundaries[j].length = abs(images[x]->pieces[y]->boundaries[j].boundary[0].x - images[x]->pieces[y]->boundaries[j].boundary.back().x);

					//determine boundary type
					if (std::abs(maxY) > 0.03*std::abs(images[x]->pieces[y]->boundaries[j].boundary[0].x - images[x]->pieces[y]->boundaries[j].boundary.back().x)){
						if (maxY > 0)
							images[x]->pieces[y]->boundaries[j].type = Boundary::IN;
						else{
							images[x]->pieces[y]->boundaries[j].type = Boundary::OUT;
							//rotate by 180 degrees for easy further matching
							for (int q = 0; q < images[x]->pieces[y]->boundaries[j].boundary.size(); q++){
								cv::Point tmp;
								tmp.x = images[x]->pieces[y]->boundaries[j].boundary[q].x * (-1) + images[x]->pieces[y]->boundaries[j].length;
								tmp.y = images[x]->pieces[y]->boundaries[j].boundary[q].y * (-1);
								images[x]->pieces[y]->boundaries[j].boundary[q] = tmp;
							}
							//translate the leftmost point to (0,0) again
							cv::Point target(std::min(images[x]->pieces[y]->boundaries[j].boundary[0].x, images[x]->pieces[y]->boundaries[j].boundary.back().x), images[x]->pieces[y]->boundaries[j].boundary[0].y);
							if (target.x != 0 || target.y != 0){
								for (int i = 0; i < images[x]->pieces[y]->boundaries[j].boundary.size(); i++){
									images[x]->pieces[y]->boundaries[j].boundary[i].x -= target.x;
									images[x]->pieces[y]->boundaries[j].boundary[i].y -= target.y;
									if (std::abs(images[x]->pieces[y]->boundaries[j].boundary[i].y)>std::abs(maxY)){
										maxY = images[x]->pieces[y]->boundaries[j].boundary[i].y;
									}
								}
							}
						}
					}
					else
						images[x]->pieces[y]->boundaries[j].type = Boundary::FLAT;

					
					//if boundary[0] is boundary end, reverse point order
					if (images[x]->pieces[y]->boundaries[j].boundary[0].x != 0){
						std::vector<cv::Point>::iterator first = images[x]->pieces[y]->boundaries[j].boundary.begin();
						std::vector<cv::Point>::iterator last = images[x]->pieces[y]->boundaries[j].boundary.end();
						while ((first != last) && (first != --last)){
							int tmp_int = (*first).x;
							(*first).x = (*last).x;
							(*last).x = tmp_int;
							tmp_int = (*first).y;
							(*first).y = (*last).y;
							(*last).y = tmp_int;
							++first;
						}
					}

					//find approximate distances of pip start/end on X axis
					if (images[x]->pieces[y]->boundaries[j].type != Boundary::FLAT){
						for (int i = 0; i < images[x]->pieces[y]->boundaries[j].boundary.size(); i++){
							cv::Point tmp = images[x]->pieces[y]->boundaries[j].boundary[i];

							//images[x]->pieces[y]->boundaries[j].distance_function.push_back(sqrt(tmp.x*tmp.x + tmp.y*tmp.y));
							//if (abs(tmp.y)>0.05*images[x]->pieces[y]->boundaries[j].length && images[x]->pieces[y]->boundaries[j].lengthToPip == -1){
							if (abs(tmp.y)>0.05*images[x]->pieces[y]->boundaries[j].length && images[x]->pieces[y]->boundaries[j].lengthToPip == -1){
								images[x]->pieces[y]->boundaries[j].lengthToPip = tmp.x;
							}
							//
							if (abs(tmp.y) < 0.045*images[x]->pieces[y]->boundaries[j].length && images[x]->pieces[y]->boundaries[j].lengthToPip != -1 && images[x]->pieces[y]->boundaries[j].lengthFromPip == -1){
								images[x]->pieces[y]->boundaries[j].lengthFromPip = images[x]->pieces[y]->boundaries[j].length - tmp.x;
							}
						}
					}
				}
				images[x]->pieces[y]->processed = true;
			}
		}
	}
			//prepare picture for display
			if (cur.y < 0){
				cv::cvtColor(images[cur.x]->sourceImage, tmpRGB, CV_BGR2RGB);
			}
			else{
				cv::cvtColor(images[cur.x]->pieces[cur.y]->piece, tmpRGB, CV_BGR2RGB);
			}
			
			std::vector<std::vector<cv::Point>> draw_boundary;
			int color = 0;
			for (auto b : images[cur.x]->pieces[cur.y]->boundaries){
				draw_boundary.push_back(std::vector<cv::Point>());
				for (int i = 0; i < b.boundary.size(); i++){
					draw_boundary.back().push_back(b.boundary[i]+cv::Point(0,150 + 100*color));
				}
				for (int i = b.boundary.size() - 1; i >= 0; i--){
					draw_boundary.back().push_back(b.boundary[i] + cv::Point(0,150 + 100*color));
				}
				
					cv::drawContours(tmpRGB, draw_boundary, color, cv::Scalar(255 * (b.type == Boundary::OUT), 255 * (b.type == Boundary::IN), 255 * (b.type == Boundary::FLAT)), 5);
					/*cv::line(tmpRGB, images[cur.x]->pieces[cur.y]->massCenter, cv::Point(b.lengthToPip, 100), cv::Scalar(70 * color, 70 * color, 255), 5);
					cv::line(tmpRGB, images[cur.x]->pieces[cur.y]->massCenter, cv::Point(b.length - b.lengthFromPip, 100), cv::Scalar(70 * color, 70 * color, 255), 5);*/
				
				color++;
			}
			
			
			shown = new QImage((quint8*)tmpRGB.data, tmpRGB.cols, tmpRGB.rows, tmpRGB.step, QImage::Format::Format_RGB888);
			int width = shown->size().width();
			int height = shown->size().height();
			if (shown->size().height()>ui.label->height() || shown->size().width() > ui.label->width()){
				if (width > ui.label->width()){
					int tmp = ui.label->width()*((double)height / width);
					width = ui.label->width();
					height = tmp;
				}
				if (height > ui.label->height()){
					int tmp = ui.label->height()*((double)width / height);
					width = tmp;
					height = ui.label->height();
				}
			}
			QPixmap p = QPixmap::fromImage(*shown).scaled(QSize(width, height));
			QPainter *painter = new QPainter(&p);
			ui.label->setPixmap(p);
}

std::vector<std::vector<cv::Point3i>> CV_test::solve(){
	std::vector<std::vector<cv::Point3i>> grid;
	std::vector<cv::Point> used;
	int maxLength = 0;
	for (auto x : images){
		for (auto y : x->pieces){
			for (auto z : y->boundaries){
				if (z.length > maxLength)
					maxLength = z.length;
			}
		}
	}

	int fastStageThreshold = 4*pow((int)(0.03*maxLength), 2);

	for (int i = 0; i < images.size(); i++){
		bool startingPieceFound = false;
		int orientation = -1;
		for (int j = 0; j < images[i]->pieces.size(); j++){
			for (int k = 0; k < 4; k++){
				int num = (k + 1)>3 ? 0 : k + 1;
				if (images[i]->pieces[j]->boundaries[k].type == Boundary::FLAT && images[i]->pieces[j]->boundaries[num].type == Boundary::FLAT){
					startingPieceFound = true;
					orientation = k;
				}
			}
			if (startingPieceFound){
				grid.push_back(std::vector<cv::Point3i>());
				grid[0].push_back(cv::Point3i(i,j,orientation));
				used.push_back(cv::Point(i, j));
				break;
			}
		}
		if (startingPieceFound){
			break;
		}
	}

	bool exitCondition = false;
	//top frame row
	while (!exitCondition){
		int x = grid[grid.size() - 1][0].x;
		int y = grid[grid.size() - 1][0].y;
		int z = grid[grid.size() - 1][0].z-1;
		z = z >= 0 ? z : 3;
		Boundary::Type target = images[x]->pieces[y]->boundaries[z].type;
		if (target != Boundary::FLAT){
			target = target == Boundary::IN ? Boundary::OUT : Boundary::IN;
			std::vector<cv::Point3i> candidates;
			for (int i = 0; i < images.size(); i++){
				bool nextPieceFound = false;
				for (int j = 0; j < images[i]->pieces.size(); j++){
					bool alreadyUsed = false;
					for (cv::Point u : used){
						if (u.x == i && u.y == j){
							alreadyUsed = true;
						}
					}

					if (alreadyUsed)
						continue;

					for (int k = 0; k < 4; k++){
						int num = (k + 1)>3 ? 0 : k + 1;
						if (images[i]->pieces[j]->boundaries[k].type == Boundary::FLAT && images[i]->pieces[j]->boundaries[num].type == target){
							//fast metrics here
							int fastScore = 0;
							fastScore += pow(images[i]->pieces[j]->boundaries[num].length - images[x]->pieces[y]->boundaries[z].length, 2);
							fastScore += pow(images[i]->pieces[j]->boundaries[num].lengthToPip - images[x]->pieces[y]->boundaries[z].lengthToPip, 2);
							fastScore += pow(images[i]->pieces[j]->boundaries[num].lengthFromPip - images[x]->pieces[y]->boundaries[z].lengthFromPip, 2);
							if (fastScore < fastStageThreshold){
								candidates.push_back(cv::Point3i(i, j, num));
							}
						}
					}
				}
			}
			if (candidates.size() < 1){
				qDebug() << "No matches, horizontal size grew to " << grid.size();
				return grid;
			}
			if (candidates.size() > 1){
				//slow metrics here
				//taking the best here, implement proper scoring cutoff later
				int best = -1;
				double bestScore = images[x]->pieces[y]->boundaries[z].length;
				std::vector<double> scores;

				for (int i = 0; i<candidates.size();i++){
					double totalDist = 0;
					for (auto p : images[x]->pieces[y]->boundaries[z].boundary){
						int min = images[x]->pieces[y]->boundaries[z].length;
						for (auto pp : images[candidates[i].x]->pieces[candidates[i].y]->boundaries[candidates[i].z].boundary){
							if (abs(pp.x - p.x) > min && pp.x > p.x) break;
							if (abs(pp.x - p.x) > min) continue;
							int dist = (p.x - pp.x)*(p.x - pp.x) + (p.y - pp.y)*(p.y - pp.y);
							if (dist < min)
								min = dist;
						}
						totalDist += sqrt(min);
					}
					totalDist /= images[x]->pieces[y]->boundaries[z].boundary.size();
					scores.push_back(totalDist);
					if (totalDist < bestScore){
						bestScore = totalDist;
						best = i;
					}
				}

				for (int i = scores.size() - 1; i >= 0; i--){
					if (scores.at(i) > bestScore*1.2) {
						candidates.erase(candidates.begin() + i);
						if (i<best){ best--; }
					}
				}

				if (candidates.size() > 1){
					best = -1;
					bestScore = INT_MAX;
					double color_dist1 = 0;
					double color_dist2 = 0;
					for (int j = 0; j < candidates.size(); j++){
						for (int i = 0; i < images[x]->pieces[y]->boundaries[z].color.size(); ++i){
							color_dist1 += pow(((images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color[i] - images[x]->pieces[y]->boundaries[z].color[i]).x + (images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color[i] - images[x]->pieces[y]->boundaries[z].color[i]).y + (images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color[i] - images[x]->pieces[y]->boundaries[z].color[i]).z), 2);
							color_dist2 += pow(((images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color[i] - images[x]->pieces[y]->boundaries[z].color[images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color.size() - 1 - i]).x + (images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color[i] - images[x]->pieces[y]->boundaries[z].color[images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color.size() - 1 - i]).y + (images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color[i] - images[x]->pieces[y]->boundaries[z].color[images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color.size() - 1 - i]).z), 2);
						}
						if (std::min(color_dist1, color_dist2) < bestScore){
							bestScore = std::min(color_dist1, color_dist2);
							best = j;
						}
					}
				}

				if (best != -1){
					grid.push_back(std::vector<cv::Point3i>());
					grid.back().push_back(cv::Point3i(candidates[best].x, candidates[best].y, (candidates[best].z - 1) >= 0 ? candidates[best].z - 1 : 3));
					used.push_back(cv::Point(candidates[best].x, candidates[best].y));
				}
			}
		}
		else{
			exitCondition = true;
		}
	}


	//left frame row
	exitCondition = false;
	while (!exitCondition){
		int x = grid[0][grid[0].size() - 1].x;
		int y = grid[0][grid[0].size() - 1].y;
		int z = grid[0][grid[0].size() - 1].z - 2;
		z = z >= 0 ? z : 4+z;
		Boundary::Type target = images[x]->pieces[y]->boundaries[z].type;
		if (target != Boundary::FLAT){
			target = target == Boundary::IN ? Boundary::OUT : Boundary::IN;
			std::vector<cv::Point3i> candidates;
			for (int i = 0; i < images.size(); i++){
				bool nextPieceFound = false;
				for (int j = 0; j < images[i]->pieces.size(); j++){
					bool alreadyUsed = false;
					for (cv::Point u : used){
						if (u.x == i && u.y == j){
							alreadyUsed = true;
						}
					}

					if (alreadyUsed)
						continue;

					for (int k = 0; k < 4; k++){
						int num = (k + 1)>3 ? 0 : k + 1;
						if (images[i]->pieces[j]->boundaries[k].type == target && images[i]->pieces[j]->boundaries[num].type == Boundary::FLAT){
							//fast metrics here
							int fastScore = 0;
							fastScore += pow(images[i]->pieces[j]->boundaries[k].length - images[x]->pieces[y]->boundaries[z].length, 2);
							fastScore += pow(images[i]->pieces[j]->boundaries[k].lengthToPip - images[x]->pieces[y]->boundaries[z].lengthToPip, 2);
							fastScore += pow(images[i]->pieces[j]->boundaries[k].lengthFromPip - images[x]->pieces[y]->boundaries[z].lengthFromPip, 2);
							if (fastScore < fastStageThreshold){
								candidates.push_back(cv::Point3i(i, j, k));
							}
						}
					}
				}
			}
			if (candidates.size() < 1){
				qDebug() << "No matches, left size grew to " << grid.size();
				return grid;
			}
			if (candidates.size() >= 1){
				//slow metrics here
				//taking the best here, implement proper scoring cutoff later
				int best = -1;
				double bestScore = images[x]->pieces[y]->boundaries[z].length;
				std::vector<double> scores;

				for (int i = 0; i<candidates.size(); i++){
					double totalDist = 0;
					for (auto p : images[x]->pieces[y]->boundaries[z].boundary){
						int min = images[x]->pieces[y]->boundaries[z].length;
						for (auto pp : images[candidates[i].x]->pieces[candidates[i].y]->boundaries[candidates[i].z].boundary){
							if (abs(pp.x - p.x) > min && pp.x > p.x) break;
							if (abs(pp.x - p.x) > min) continue;
							int dist = (p.x - pp.x)*(p.x - pp.x) + (p.y - pp.y)*(p.y - pp.y);
							if (dist < min)
								min = dist;
						}
						totalDist += sqrt(min);
					}
					totalDist /= images[x]->pieces[y]->boundaries[z].boundary.size();
					scores.push_back(totalDist);
					if (totalDist < bestScore){
						bestScore = totalDist;
						best = i;
					}
				}

				for (int i = scores.size() - 1; i >= 0; i--){
					if (scores.at(i) > bestScore*1.2) {
						candidates.erase(candidates.begin() + i);
						if (i<best){ best--; }
					}
				}

				if (candidates.size() > 1){
					best = -1;
					bestScore = INT_MAX;
					double color_dist1 = 0;
					double color_dist2 = 0;
					for (int j = 0; j < candidates.size(); j++){
						for (int i = 0; i < images[x]->pieces[y]->boundaries[z].color.size(); ++i){
							color_dist1 += pow(((images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color[i] - images[x]->pieces[y]->boundaries[z].color[i]).x + (images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color[i] - images[x]->pieces[y]->boundaries[z].color[i]).y + (images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color[i] - images[x]->pieces[y]->boundaries[z].color[i]).z), 2);
							color_dist2 += pow(((images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color[i] - images[x]->pieces[y]->boundaries[z].color[images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color.size() - 1 - i]).x + (images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color[i] - images[x]->pieces[y]->boundaries[z].color[images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color.size() - 1 - i]).y + (images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color[i] - images[x]->pieces[y]->boundaries[z].color[images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color.size() - 1 - i]).z), 2);
						}
						if (std::min(color_dist1, color_dist2) < bestScore){
							bestScore = std::min(color_dist1, color_dist2);
							best = j;
						}
					}
				}

				if (best != -1){
					grid[0].push_back(cv::Point3i(candidates[best].x, candidates[best].y, candidates[best].z));
					used.push_back(cv::Point(candidates[best].x, candidates[best].y));
				}
			}
		}
		else{
			exitCondition = true;
		}
	}

	//right frame row
	exitCondition = false;
	while (!exitCondition){
		int x = grid.back()[grid.back().size() - 1].x;
		int y = grid.back()[grid.back().size() - 1].y;
		int z = grid.back()[grid.back().size() - 1].z - 2;
		z = z >= 0 ? z : 4 + z;
		Boundary::Type target = images[x]->pieces[y]->boundaries[z].type;
		if (target != Boundary::FLAT){
			target = target == Boundary::IN ? Boundary::OUT : Boundary::IN;
			std::vector<cv::Point3i> candidates;
			for (int i = 0; i < images.size(); i++){
				bool nextPieceFound = false;
				for (int j = 0; j < images[i]->pieces.size(); j++){
					bool alreadyUsed = false;
					for (cv::Point u : used){
						if (u.x == i && u.y == j){
							alreadyUsed = true;
						}
					}

					if (alreadyUsed)
						continue;

					for (int k = 0; k < 4; k++){
						int num = (k + 1)>3 ? 0 : k + 1;
						if (images[i]->pieces[j]->boundaries[k].type == Boundary::FLAT && images[i]->pieces[j]->boundaries[num].type == target){
							//fast metrics here
							int fastScore = 0;
							fastScore += pow(images[i]->pieces[j]->boundaries[num].length - images[x]->pieces[y]->boundaries[z].length, 2);
							fastScore += pow(images[i]->pieces[j]->boundaries[num].lengthToPip - images[x]->pieces[y]->boundaries[z].lengthToPip, 2);
							fastScore += pow(images[i]->pieces[j]->boundaries[num].lengthFromPip - images[x]->pieces[y]->boundaries[z].lengthFromPip, 2);
							if (fastScore < fastStageThreshold){
								candidates.push_back(cv::Point3i(i, j, num));
							}
						}
					}
				}
			}
			if (candidates.size() < 1){
				qDebug() << "No matches, right size grew to " << grid.size();
				return grid;
			}
			if (candidates.size() >= 1){
				//slow metrics here
				//taking the best here, implement proper scoring cutoff later
				int best = -1;
				double bestScore = images[x]->pieces[y]->boundaries[z].length;
				std::vector<double> scores;

				for (int i = 0; i<candidates.size(); i++){
					double totalDist = 0;
					for (auto p : images[x]->pieces[y]->boundaries[z].boundary){
						int min = images[x]->pieces[y]->boundaries[z].length;
						for (auto pp : images[candidates[i].x]->pieces[candidates[i].y]->boundaries[candidates[i].z].boundary){
							if (abs(pp.x - p.x) > min && pp.x > p.x) break;
							if (abs(pp.x - p.x) > min) continue;
							int dist = (p.x - pp.x)*(p.x - pp.x) + (p.y - pp.y)*(p.y - pp.y);
							if (dist < min)
								min = dist;
						}
						totalDist += sqrt(min);
					}
					totalDist /= images[x]->pieces[y]->boundaries[z].boundary.size();
					scores.push_back(totalDist);
					if (totalDist < bestScore){
						bestScore = totalDist;
						best = i;
					}
				}

				for (int i = scores.size() - 1; i >= 0; i--){
					if (scores.at(i) > bestScore*1.2) {
						candidates.erase(candidates.begin() + i);
						if (i<best){ best--; }
					}
				}

				if (candidates.size() > 1){
					best = -1;
					bestScore = INT_MAX;
					double color_dist1 = 0;
					double color_dist2 = 0;
					for (int j = 0; j < candidates.size(); j++){
						for (int i = 0; i < images[x]->pieces[y]->boundaries[z].color.size(); ++i){
							color_dist1 += pow(((images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color[i] - images[x]->pieces[y]->boundaries[z].color[i]).x + (images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color[i] - images[x]->pieces[y]->boundaries[z].color[i]).y + (images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color[i] - images[x]->pieces[y]->boundaries[z].color[i]).z), 2);
							color_dist2 += pow(((images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color[i] - images[x]->pieces[y]->boundaries[z].color[images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color.size() - 1 - i]).x + (images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color[i] - images[x]->pieces[y]->boundaries[z].color[images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color.size() - 1 - i]).y + (images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color[i] - images[x]->pieces[y]->boundaries[z].color[images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color.size() - 1 - i]).z), 2);
						}
						if (std::min(color_dist1,color_dist2) < bestScore){
							bestScore = std::min(color_dist1, color_dist2);
							best = j;
						}
					}
				}

				if (best != -1){
					grid.back().push_back(cv::Point3i(candidates[best].x, candidates[best].y, candidates[best].z));
					used.push_back(cv::Point(candidates[best].x, candidates[best].y));
				}
			}
		}
		else{
			exitCondition = true;
		}
	}

	//bottom frame row
	exitCondition = false;
	for (int i = 1; i < grid.size() - 1; ++i){
		for (int j = 1; j < grid[0].size(); ++j){
			grid[i].push_back(cv::Point3i(-1, -1, -1));
		}
	}
	int pos = 0;
	while (!exitCondition){		
		int x = grid[pos][grid.back().size() - 1].x;
		int y = grid[pos][grid.back().size() - 1].y;
		int z = grid[pos][grid.back().size() - 1].z - 1;
		z = z >= 0 ? z : 4 + z;
		Boundary::Type target = images[x]->pieces[y]->boundaries[z].type;
		if (target != Boundary::FLAT && pos<grid.size()-2){
			target = target == Boundary::IN ? Boundary::OUT : Boundary::IN;
			std::vector<cv::Point3i> candidates;
			for (int i = 0; i < images.size(); i++){
				bool nextPieceFound = false;
				for (int j = 0; j < images[i]->pieces.size(); j++){
					bool alreadyUsed = false;
					for (cv::Point u : used){
						if (u.x == i && u.y == j){
							alreadyUsed = true;
						}
					}

					if (alreadyUsed)
						continue;

					for (int k = 0; k < 4; k++){
						int num = (k + 1)>3 ? 0 : k + 1;
						if (images[i]->pieces[j]->boundaries[k].type == target && images[i]->pieces[j]->boundaries[num].type == Boundary::FLAT){
							//fast metrics here
							int fastScore = 0;
							fastScore += pow(images[i]->pieces[j]->boundaries[k].length - images[x]->pieces[y]->boundaries[z].length, 2);
							fastScore += pow(images[i]->pieces[j]->boundaries[k].lengthToPip - images[x]->pieces[y]->boundaries[z].lengthToPip, 2);
							fastScore += pow(images[i]->pieces[j]->boundaries[k].lengthFromPip - images[x]->pieces[y]->boundaries[z].lengthFromPip, 2);
							if (fastScore < fastStageThreshold){
								candidates.push_back(cv::Point3i(i, j, k));
							}
						}
					}
				}
			}
			if (candidates.size() < 1){
				qDebug() << "No matches at bottom";
				return grid;
			}
			if (candidates.size() >= 1){
				//slow metrics here
				//taking the best here, implement proper scoring cutoff later
				int best = -1;
				double bestScore = images[x]->pieces[y]->boundaries[z].length;
				std::vector<double> scores;

				for (int i = 0; i<candidates.size(); i++){
					double totalDist = 0;
					for (auto p : images[x]->pieces[y]->boundaries[z].boundary){
						int min = images[x]->pieces[y]->boundaries[z].length;
						for (auto pp : images[candidates[i].x]->pieces[candidates[i].y]->boundaries[candidates[i].z].boundary){
							if (abs(pp.x - p.x) > min && pp.x > p.x) break;
							if (abs(pp.x - p.x) > min) continue;
							int dist = (p.x - pp.x)*(p.x - pp.x) + (p.y - pp.y)*(p.y - pp.y);
							if (dist < min)
								min = dist;
						}
						totalDist += sqrt(min);
					}
					totalDist /= images[x]->pieces[y]->boundaries[z].boundary.size();
					scores.push_back(totalDist);
					if (totalDist < bestScore){
						bestScore = totalDist;
						best = i;
					}
				}

				for (int i = scores.size() - 1; i >= 0; i--){
					if (scores.at(i) > bestScore*1.2) {
						candidates.erase(candidates.begin() + i);
						if (i<best){ best--; }
					}
				}

				if (candidates.size() > 1){
					best = -1;
					bestScore = INT_MAX;
					double color_dist1 = 0;
					double color_dist2 = 0;
					for (int j = 0; j < candidates.size(); j++){
						for (int i = 0; i < images[x]->pieces[y]->boundaries[z].color.size(); ++i){
							color_dist1 += pow(((images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color[i] - images[x]->pieces[y]->boundaries[z].color[i]).x + (images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color[i] - images[x]->pieces[y]->boundaries[z].color[i]).y + (images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color[i] - images[x]->pieces[y]->boundaries[z].color[i]).z), 2);
							color_dist2 += pow(((images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color[i] - images[x]->pieces[y]->boundaries[z].color[images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color.size() - 1 - i]).x + (images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color[i] - images[x]->pieces[y]->boundaries[z].color[images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color.size() - 1 - i]).y + (images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color[i] - images[x]->pieces[y]->boundaries[z].color[images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color.size() - 1 - i]).z), 2);
						}
						if (std::min(color_dist1, color_dist2) < bestScore){
							bestScore = std::min(color_dist1, color_dist2);
							best = j;
						}
					}
				}

				if (best != -1){
					grid[++pos][grid[0].size() - 1] = cv::Point3i(candidates[best].x, candidates[best].y, (candidates[best].z - 1) >= 0 ? candidates[best].z - 1 : 3);
					used.push_back(cv::Point(candidates[best].x, candidates[best].y));
				}
			}
		}
		else{
			exitCondition = true;
		}
	}

	for (int i = 1; i < grid.size() - 1; ++i){
		for (int j = 1; j < grid[i].size() - 1; ++j){
			int x = grid[i-1][j].x;
			int y = grid[i-1][j].y;
			int z = grid[i-1][j].z - 1;
			int x2 = grid[i][j-1].x;
			int y2 = grid[i][j-1].y;
			int z2 = grid[i][j-1].z - 2;
			z = z >= 0 ? z : 4 + z;
			z2 = z2 >= 0 ? z2 : 4 + z2;
			Boundary::Type target = images[x]->pieces[y]->boundaries[z].type;
			Boundary::Type target2 = images[x2]->pieces[y2]->boundaries[z2].type;
			target = target == Boundary::IN ? Boundary::OUT : Boundary::IN;
			target2 = target2 == Boundary::IN ? Boundary::OUT : Boundary::IN;
			std::vector<cv::Point3i> candidates;
			for (int i = 0; i < images.size(); i++){
				bool nextPieceFound = false;
				for (int j = 0; j < images[i]->pieces.size(); j++){
					bool alreadyUsed = false;
					for (cv::Point u : used){
						if (u.x == i && u.y == j){
							alreadyUsed = true;
						}
					}

					if (alreadyUsed)
						continue;

					for (int k = 0; k < 4; k++){
						int num = (k + 1)>3 ? 0 : k + 1;
						if (images[i]->pieces[j]->boundaries[k].type == target2 && images[i]->pieces[j]->boundaries[num].type == target){
							//fast metrics here
							int fastScore = 0;
							fastScore += pow(images[i]->pieces[j]->boundaries[k].length - images[x2]->pieces[y2]->boundaries[z2].length, 2);
							fastScore += pow(images[i]->pieces[j]->boundaries[k].lengthToPip - images[x2]->pieces[y2]->boundaries[z2].lengthToPip, 2);
							fastScore += pow(images[i]->pieces[j]->boundaries[k].lengthFromPip - images[x2]->pieces[y2]->boundaries[z2].lengthFromPip, 2);
							fastScore += pow(images[i]->pieces[j]->boundaries[num].length - images[x]->pieces[y]->boundaries[z].length, 2);
							fastScore += pow(images[i]->pieces[j]->boundaries[num].lengthToPip - images[x]->pieces[y]->boundaries[z].lengthToPip, 2);
							fastScore += pow(images[i]->pieces[j]->boundaries[num].lengthFromPip - images[x]->pieces[y]->boundaries[z].lengthFromPip, 2);
							if (fastScore < fastStageThreshold*2.5f){
								candidates.push_back(cv::Point3i(i, j, k));
							}
						}
					}
				}
			}

			if (candidates.size() < 1){
				qDebug() << "No matches";
				return grid;
			}
			if (candidates.size() >= 1){
				//slow metrics here
				//taking the best here, implement proper scoring cutoff later
				int best = -1;
				double bestScore = images[x]->pieces[y]->boundaries[z].length;
				std::vector<double> scores;

				for (int i = 0; i<candidates.size(); i++){
					double totalDist = 0;
					double totalDist2 = 0;
					int num = (candidates[i].z + 1)>3 ? 0 : candidates[i].z + 1;
					for (auto p : images[x]->pieces[y]->boundaries[z].boundary){
						int min = images[x]->pieces[y]->boundaries[z].length;
						for (auto pp : images[candidates[i].x]->pieces[candidates[i].y]->boundaries[num].boundary){
							if (abs(pp.x - p.x) > min && pp.x > p.x) break;
							if (abs(pp.x - p.x) > min) continue;
							int dist = (p.x - pp.x)*(p.x - pp.x) + (p.y - pp.y)*(p.y - pp.y);
							if (dist < min)
								min = dist;
						}
						totalDist += sqrt(min);
					}
					totalDist /= images[x]->pieces[y]->boundaries[z].boundary.size();
					
					for (auto p : images[x2]->pieces[y2]->boundaries[z2].boundary){
						int min = images[x2]->pieces[y2]->boundaries[z2].length;
						for (auto pp : images[candidates[i].x]->pieces[candidates[i].y]->boundaries[candidates[i].z].boundary){
							if (abs(pp.x - p.x) > min && pp.x > p.x) break;
							if (abs(pp.x - p.x) > min) continue;
							int dist = (p.x - pp.x)*(p.x - pp.x) + (p.y - pp.y)*(p.y - pp.y);
							if (dist < min)
								min = dist;
						}
						totalDist2 += sqrt(min);
					}
					totalDist2 /= images[x2]->pieces[y2]->boundaries[z2].boundary.size();
					scores.push_back(totalDist + totalDist2);
					if ((totalDist + totalDist2) < bestScore){
						bestScore = (totalDist + totalDist2);
						best = i;
					}
				}

				for (int i = scores.size() - 1; i >= 0; i--){
					if (scores.at(i) > bestScore*1.2) {
						candidates.erase(candidates.begin() + i);
						if (i<best){ best--; }
					}
				}

				if (candidates.size() > 1){
					best = -1;
					bestScore = INT_MAX;
					double color_dist1 = 0;
					double color_dist2 = 0;
					for (int j = 0; j < candidates.size(); j++){
						for (int i = 0; i < images[x]->pieces[y]->boundaries[z].color.size(); ++i){
							color_dist1 += pow(((images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color[i] - images[x]->pieces[y]->boundaries[z].color[i]).x + (images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color[i] - images[x]->pieces[y]->boundaries[z].color[i]).y + (images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color[i] - images[x]->pieces[y]->boundaries[z].color[i]).z), 2);
							color_dist2 += pow(((images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color[i] - images[x]->pieces[y]->boundaries[z].color[images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color.size() - 1 - i]).x + (images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color[i] - images[x]->pieces[y]->boundaries[z].color[images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color.size() - 1 - i]).y + (images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color[i] - images[x]->pieces[y]->boundaries[z].color[images[candidates[j].x]->pieces[candidates[j].y]->boundaries[candidates[j].z].color.size() - 1 - i]).z), 2);
						}
						if (std::min(color_dist1, color_dist2) < bestScore){
							bestScore = std::min(color_dist1, color_dist2);
							best = j;
						}
					}
				}

				if (best != -1){
					grid[i][j] = cv::Point3i(candidates[best].x, candidates[best].y, candidates[best].z);
					used.push_back(cv::Point(candidates[best].x, candidates[best].y));
				}
			}
		}
	}

	std::ofstream sol("solution.txt");
	for (int i = 0; i < grid[0].size(); i++){
		for (int j = 0; j < grid.size(); j++){
			sol << grid[j][i].x << "," << grid[j][i].y << "," << grid[j][i].z << " | ";
		}
		sol << "\n";
	}
	sol.close();
	return grid;
}

void CV_test::generateVisualSolution(){
	std::vector<std::vector<cv::Point3i>> grid = solve();
	int maxX = 0;
	int maxY = 0;
	for (auto x : images){
		for (auto y : x->pieces){
			if (y->massCenter.x > maxX){
				maxX = y->massCenter.x;
			}
			if (y->massCenter.y > maxY){
				maxY = y->massCenter.y;
			}
		}
	}

	cv::Mat blank_canvas(cv::Size(grid.size()*maxX * 2, grid.size()*maxY * 2), images[0]->pieces[0]->piece.type(), cv::Scalar(0,0,0));
	cv::Mat canvas = cv::Mat::zeros(cv::Size(grid.size()*maxX * 2, grid.size()*maxY * 2), CV_8UC3);
	int horplus = images[grid[0][0].x]->pieces[grid[0][0].y]->piece.cols;
	int vertplus = images[grid[0][0].x]->pieces[grid[0][0].y]->piece.rows;
	cv::Mat canvasRoi(canvas.colRange(50, 50 + horplus).rowRange(50, 50 + vertplus));

	for (int j = 0; j < grid[0].size(); j++){
		int maxSize = std::max(images[grid[0][j].x]->pieces[grid[0][j].y]->piece.cols, images[grid[0][j].x]->pieces[grid[0][j].y]->piece.rows) * 2;
		cv::Mat tmp = cv::Mat::zeros(cv::Size(maxSize, maxSize), CV_8UC3);
		int left = maxSize / 2 - images[grid[0][j].x]->pieces[grid[0][j].y]->piece.cols / 2;
		int top = maxSize / 2 - images[grid[0][j].x]->pieces[grid[0][j].y]->piece.rows / 2;
		images[grid[0][j].x]->pieces[grid[0][j].y]->piece.copyTo(tmp.colRange(left, left + images[grid[0][j].x]->pieces[grid[0][j].y]->piece.cols).rowRange(top, top + images[grid[0][j].x]->pieces[grid[0][j].y]->piece.rows));
		double targetAngle;
		cv::Point targetVector;
			targetVector.x = 1; targetVector.y = 0;
		targetAngle = (180 / 3.1415) * simpleAngle(cv::Point(1, 0), images[grid[0][j].x]->pieces[grid[0][j].y]->boundaries[grid[0][j].z].corners[0] - images[grid[0][j].x]->pieces[grid[0][j].y]->boundaries[grid[0][j].z].corners[1]);
		cv::warpAffine(tmp, tmp, cv::getRotationMatrix2D(cv::Point(maxSize / 2, maxSize / 2), targetAngle, 1.0), cv::Size(maxSize, maxSize));
		cv::Point boundaryTest1 = images[grid[0][j].x]->pieces[grid[0][j].y]->boundaries[grid[0][j].z].corners[0];
		cv::Point boundaryTest2 = images[grid[0][j].x]->pieces[grid[0][j].y]->boundaries[grid[0][j].z].corners[1];
		targetAngle = targetAngle * (3.1415 / 180);
		boundaryTest1.x = boundaryTest1.x * cos(targetAngle) + boundaryTest1.y * sin(targetAngle) + (1 - cos(targetAngle))*images[grid[0][j].x]->pieces[grid[0][j].y]->massCenter.x - sin(targetAngle)*images[grid[0][j].x]->pieces[grid[0][j].y]->massCenter.y;
		boundaryTest1.y = -boundaryTest1.x * sin(targetAngle) + boundaryTest1.y * cos(targetAngle) + sin(targetAngle)*images[grid[0][j].x]->pieces[grid[0][j].y]->massCenter.x + (1 - cos(targetAngle))*images[grid[0][j].x]->pieces[grid[0][j].y]->massCenter.y;
		boundaryTest2.x = boundaryTest2.x * cos(targetAngle) + boundaryTest2.y * sin(targetAngle) + (1 - cos(targetAngle))*images[grid[0][j].x]->pieces[grid[0][j].y]->massCenter.x - sin(targetAngle)*images[grid[0][j].x]->pieces[grid[0][j].y]->massCenter.y;
		boundaryTest2.y = -boundaryTest2.x * sin(targetAngle) + boundaryTest2.y * cos(targetAngle) + sin(targetAngle)*images[grid[0][j].x]->pieces[grid[0][j].y]->massCenter.x + (1 - cos(targetAngle))*images[grid[0][j].x]->pieces[grid[0][j].y]->massCenter.y;
		if (boundaryTest1.y > images[grid[0][j].x]->pieces[grid[0][j].y]->massCenter.y || boundaryTest2.y > images[grid[0][j].x]->pieces[grid[0][j].y]->massCenter.y) {
			cv::warpAffine(tmp, tmp, cv::getRotationMatrix2D(cv::Point(maxSize / 2, maxSize / 2), 180, 1.0), cv::Size(maxSize, maxSize));
		}

		cv::Mat tempCanvas;
		cv::cvtColor(tmp, tempCanvas, cv::COLOR_BGR2GRAY);
		cv::threshold(tempCanvas, tempCanvas, 1, 255, cv::THRESH_BINARY);
		std::vector<std::vector<cv::Point>> edge;
		cv::findContours(tempCanvas, edge, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		cv::Rect brect = cv::boundingRect(edge[0]);
		cv::rectangle(tempCanvas, brect, cv::Scalar(255),5);
		//tmp(brect).copyTo(canvas.colRange(50, 50 + horplus).rowRange(50, 50 + vertplus));

		std::string s = "piece_";
		char numstr[21]; // enough to hold all numbers up to 64-bits
		sprintf(numstr, "%d_%d", grid[0][j].x, grid[0][j].y);
		s += numstr;
		s += ".jpg";
		cv::imwrite(s, tmp);
		cv::imwrite("gray_" + s, tempCanvas);
	}

	for (int i = 1; i < grid.size(); i++){
		std::vector<int> restrictions;
		for (int j = 0; j < grid[i].size(); j++){
			int maxSize = std::max(images[grid[i][j].x]->pieces[grid[i][j].y]->piece.cols, images[grid[i][j].x]->pieces[grid[i][j].y]->piece.rows) * 2;
			cv::Mat tmp = cv::Mat::zeros(cv::Size(maxSize, maxSize), CV_8UC3);
			int left = maxSize / 2 - images[grid[i][j].x]->pieces[grid[i][j].y]->piece.cols / 2;
			int top = maxSize / 2 - images[grid[i][j].x]->pieces[grid[i][j].y]->piece.rows / 2;
			images[grid[i][j].x]->pieces[grid[i][j].y]->piece.copyTo(tmp.colRange(left, left + images[grid[i][j].x]->pieces[grid[i][j].y]->piece.cols).rowRange(top, top + images[grid[i][j].x]->pieces[grid[i][j].y]->piece.rows));
			double targetAngle;
			cv::Point targetVector;
			if (j == 0){
				targetVector.x = 1; targetVector.y = 0;
			}
			else{
				int boundaryToPositionAgainst = grid[i][j - 1].z - 2;
				boundaryToPositionAgainst = boundaryToPositionAgainst < 0 ? 4 - boundaryToPositionAgainst : boundaryToPositionAgainst;
				targetVector = images[grid[i][j - 1].x]->pieces[grid[i][j - 1].y]->boundaries[boundaryToPositionAgainst].corners[0] - images[grid[i][j - 1].x]->pieces[grid[i][j - 1].y]->boundaries[boundaryToPositionAgainst].corners[1];
			}
			targetAngle = (180/3.1415) * simpleAngle(cv::Point(1, 0), images[grid[i][j].x]->pieces[grid[i][j].y]->boundaries[grid[i][j].z].corners[0] - images[grid[i][j].x]->pieces[grid[i][j].y]->boundaries[grid[i][j].z].corners[1]);
			cv::warpAffine(tmp, tmp, cv::getRotationMatrix2D(cv::Point(maxSize/2,maxSize/2), targetAngle, 1.0), cv::Size(maxSize, maxSize));
			cv::imwrite("tst.jpg", tmp);
			cv::Point boundaryTest1 = images[grid[i][j].x]->pieces[grid[i][j].y]->boundaries[grid[i][j].z].corners[0];
			cv::Point boundaryTest2 = images[grid[i][j].x]->pieces[grid[i][j].y]->boundaries[grid[i][j].z].corners[1];
			targetAngle = targetAngle * (3.1415/180);
			boundaryTest1.x = boundaryTest1.x * cos(targetAngle) + boundaryTest1.y * sin(targetAngle) + (1 - cos(targetAngle))*images[grid[i][j].x]->pieces[grid[i][j].y]->massCenter.x - sin(targetAngle)*images[grid[i][j].x]->pieces[grid[i][j].y]->massCenter.y;
			boundaryTest1.y = -boundaryTest1.x * sin(targetAngle) + boundaryTest1.y * cos(targetAngle) + sin(targetAngle)*images[grid[i][j].x]->pieces[grid[i][j].y]->massCenter.x + (1 - cos(targetAngle))*images[grid[i][j].x]->pieces[grid[i][j].y]->massCenter.y;
			boundaryTest2.x = boundaryTest2.x * cos(targetAngle) + boundaryTest2.y * sin(targetAngle) + (1 - cos(targetAngle))*images[grid[i][j].x]->pieces[grid[i][j].y]->massCenter.x - sin(targetAngle)*images[grid[i][j].x]->pieces[grid[i][j].y]->massCenter.y;
			boundaryTest2.y = -boundaryTest2.x * sin(targetAngle) + boundaryTest2.y * cos(targetAngle) + sin(targetAngle)*images[grid[i][j].x]->pieces[grid[i][j].y]->massCenter.x + (1 - cos(targetAngle))*images[grid[i][j].x]->pieces[grid[i][j].y]->massCenter.y;
			if (boundaryTest1.y > images[grid[i][j].x]->pieces[grid[i][j].y]->massCenter.y || boundaryTest2.y > images[grid[i][j].x]->pieces[grid[i][j].y]->massCenter.y) {
				cv::warpAffine(tmp, tmp, cv::getRotationMatrix2D(cv::Point(maxSize / 2, maxSize / 2), 180, 1.0), cv::Size(maxSize, maxSize));
			}

			std::string s = "piece_";
			char numstr[21]; // enough to hold all numbers up to 64-bits
			sprintf(numstr, "%d_%d", grid[i][j].x, grid[i][j].y);
			s += numstr;
			s += ".jpg";
			cv::imwrite(s, tmp);
		}
	}
	int maxSize = std::max(images[grid[0][0].x]->pieces[grid[0][0].y]->piece.cols, images[grid[0][0].x]->pieces[grid[0][0].y]->piece.rows)*2;
	cv::Mat tmp = cv::Mat::zeros(cv::Size(maxSize, maxSize), CV_8UC3);
	double targetAngle;
	targetAngle = -180*simpleAngle(cv::Point(1, 0), images[grid[0][0].x]->pieces[grid[0][0].y]->boundaries[grid[0][0].z].corners[0] - images[grid[0][0].x]->pieces[grid[0][0].y]->boundaries[grid[0][0].z].corners[1]);
	cv::warpAffine(images[grid[0][0].x]->pieces[grid[0][0].y]->piece, tmp, cv::getRotationMatrix2D(images[grid[0][0].x]->pieces[grid[0][0].y]->massCenter, targetAngle, 1.0), cv::Size(maxSize, maxSize));
	cv::Point boundaryTest1 = images[grid[0][0].x]->pieces[grid[0][0].y]->boundaries[grid[0][0].z].corners[0];
	cv::Point boundaryTest2 = images[grid[0][0].x]->pieces[grid[0][0].y]->boundaries[grid[0][0].z].corners[1];
	boundaryTest1.x = boundaryTest1.x * cos(targetAngle) + boundaryTest1.y * sin(targetAngle) + (1 - cos(targetAngle))*images[grid[0][0].x]->pieces[grid[0][0].y]->massCenter.x - sin(targetAngle)*images[grid[0][0].x]->pieces[grid[0][0].y]->massCenter.y;
	boundaryTest1.y = -boundaryTest1.x * sin(targetAngle) + boundaryTest1.y * cos(targetAngle) + sin(targetAngle)*images[grid[0][0].x]->pieces[grid[0][0].y]->massCenter.x + (1 - cos(targetAngle))*images[grid[0][0].x]->pieces[grid[0][0].y]->massCenter.y;
	boundaryTest2.x = boundaryTest2.x * cos(targetAngle) + boundaryTest2.y * sin(targetAngle) + (1 - cos(targetAngle))*images[grid[0][0].x]->pieces[grid[0][0].y]->massCenter.x - sin(targetAngle)*images[grid[0][0].x]->pieces[grid[0][0].y]->massCenter.y;
	boundaryTest2.y = -boundaryTest2.x * sin(targetAngle) + boundaryTest2.y * cos(targetAngle) + sin(targetAngle)*images[grid[0][0].x]->pieces[grid[0][0].y]->massCenter.x + (1 - cos(targetAngle))*images[grid[0][0].x]->pieces[grid[0][0].y]->massCenter.y;
	if (boundaryTest1.y < images[grid[0][0].x]->pieces[grid[0][0].y]->massCenter.y || boundaryTest2.y < images[grid[0][0].x]->pieces[grid[0][0].y]->massCenter.y) {
		cv::warpAffine(tmp, tmp, cv::getRotationMatrix2D(images[grid[0][0].x]->pieces[grid[0][0].y]->massCenter, 180, 1.0), cv::Size(maxSize, maxSize));
	}
	try{
		tmp.copyTo(canvas.colRange(50, 50 + horplus).rowRange(50, 50 + vertplus));
	}
	catch(std::exception e){
		qDebug() << e.what();
	}
	
	cv::imwrite("piece.jpg", images[grid[0][0].x]->pieces[grid[0][0].y]->piece);
	cv::imwrite("tmp.jpg", tmp);
	cv::imwrite("canvas.jpg", canvas);
}

void CV_test::getColorComparisonScore(std::vector<cv::Point> boundary, std::vector<cv::Point> saved_corners, int height, int width, cv::Point cur){
	while (saved_corners.size()<4){
		saved_corners.push_back(cv::Point(0, 0));
	}

	cv::Point* top_corner = &saved_corners[0];
	if (saved_corners.size() > 0){
		for (int i = 1; i < saved_corners.size(); i++){
			if (saved_corners[i].y < top_corner->y) top_corner = &saved_corners[i];
		}
	}

	cv::Point* corner2 = new cv::Point(9999, 9999);
	if (saved_corners.size()>1){
		for (int i = 0; i < saved_corners.size(); i++){
			if (saved_corners[i].y < corner2->y && saved_corners[i].x>(top_corner->x - 0.1*width) && &saved_corners[i] != top_corner) corner2 = &saved_corners[i];
		}
		if (corner2->x == 9999 && corner2->y == 9999){
			for (auto x : saved_corners){
				if (x.x != top_corner->x && x.y != top_corner->y) corner2 = &x;
			}
		}
	}

	cv::Point* corner3 = new cv::Point(0, 9999);
	if (saved_corners.size()>2){
		for (int i = 0; i < saved_corners.size(); i++){
			if (saved_corners[i].y >(corner2->y - 0.1*height) && saved_corners[i].x > (corner3->x - 0.1*width) && &saved_corners[i] != top_corner && &saved_corners[i] != corner2) corner3 = &saved_corners[i];
		}
		if (corner3->x == 0 && corner3->y == 9999){
			for (auto x : saved_corners){
				if (x.x != top_corner->x && x.y != top_corner->y && x.x != corner2->x && x.y != corner2->y) corner3 = &x;
			}
		}
	}

	cv::Point* corner4 = new cv::Point(0, 9999);
	if (saved_corners.size() > 3){
		for (int i = 0; i < saved_corners.size(); i++){
			if (&saved_corners[i] != corner3 && &saved_corners[i] != top_corner && &saved_corners[i] != corner2) corner4 = &saved_corners[i];
		}
		if (corner4->x == 0 && corner4->y == 9999){
			for (auto x : saved_corners){
				if (x.x != top_corner->x && x.y != top_corner->y && x.x != corner2->x && x.y != corner2->y && x.x != corner3->x && x.y != corner3->y) corner4 = &x;
			}
		}
	}

	int c1, c2, c3, c4;
	double dist1, dist2, dist3, dist4;
	dist1 = dist2 = dist3 = dist4 = height;
	c1 = c2 = c3 = c4 = 0;
	for (int i = 0; i < boundary.size(); i++){
		if (sqrt(pow((boundary[i] - (*top_corner)).x, 2) + pow((boundary[i] - (*top_corner)).y, 2)) < dist1){
			c1 = i;
			dist1 = sqrt(pow((boundary[i] - (*top_corner)).x, 2) + pow((boundary[i] - (*top_corner)).y, 2));
		}
		if (sqrt(pow((boundary[i] - (*corner2)).x, 2) + pow((boundary[i] - (*corner2)).y, 2)) < dist2){
			c2 = i;
			dist2 = sqrt(pow((boundary[i] - (*corner2)).x, 2) + pow((boundary[i] - (*corner2)).y, 2));
		}
		if (sqrt(pow((boundary[i] - (*corner3)).x, 2) + pow((boundary[i] - (*corner3)).y, 2)) < dist3){
			c3 = i;
			dist3 = sqrt(pow((boundary[i] - (*corner3)).x, 2) + pow((boundary[i] - (*corner3)).y, 2));
		}
		if (sqrt(pow((boundary[i] - (*corner4)).x, 2) + pow((boundary[i] - (*corner4)).y, 2)) < dist4){
			c4 = i;
			dist4 = sqrt(pow((boundary[i] - (*corner4)).x, 2) + pow((boundary[i] - (*corner4)).y, 2));
		}
	}

	std::vector<std::vector<cv::Point>> edges;
	edges.push_back(std::vector<cv::Point>());
	edges.push_back(std::vector<cv::Point>());
	edges.push_back(std::vector<cv::Point>());
	edges.push_back(std::vector<cv::Point>());
	std::vector<int> tst;
	tst.push_back(c1); tst.push_back(c2); tst.push_back(c3); tst.push_back(c4);
	qSort(tst);

	qDebug() << "---------------";

	std::vector<cv::Point> corners_sorted;

	corners_sorted.push_back(boundary[tst[0]]);
	corners_sorted.push_back(boundary[tst[1]]);
	corners_sorted.push_back(boundary[tst[2]]);
	corners_sorted.push_back(boundary[tst[3]]);

	for (int i = tst[0]; i != tst[1]; i = (i + 1) < boundary.size() ? i + 1 : 0){
		edges[0].push_back(boundary[i]);
	}
	for (int i = tst[1]; i != tst[2]; i = (i + 1) < boundary.size() ? i + 1 : 0){
		edges[1].push_back(boundary[i]);
	}
	for (int i = tst[2]; i != tst[3]; i = (i + 1) < boundary.size() ? i + 1 : 0){
		edges[2].push_back(boundary[i]);
	}
	for (int i = tst[3]; i != tst[0]; i = (i + 1) < boundary.size() ? i + 1 : 0){
		edges[3].push_back(boundary[i]);
	}

	//add color extraction here
	for (int i = 0; i < 4; i++){
		for (int q = 0; q < edges[i].size(); q += (edges[i].size() / 10)+1){
			cv::Point3i avg(0, 0, 0);
			for (int row = -1; row <= 1; ++row){
				for (int col = -1; col <= 1; ++col){
					cv::Vec3b intensity = images[cur.x]->pieces[cur.y]->piece.at<cv::Vec3b>(edges[i][q].y + col, edges[i][q].x + row);
					avg.x += intensity.val[0]; avg.y += intensity.val[1]; avg.z += intensity.val[2];
				}
			}
			avg.x /= 9; avg.y /= 9; avg.z /= 9;
			images[cur.x]->pieces[cur.y]->boundaries[i].color.push_back(avg);
		}
	}
}

cv::Point CV_test::getCurrentSelection(){
	int count = -1;
	for (int i = 0; i < images.size(); i++){
		if (count + images[i]->piecesCount + 1 >= ui.listWidget->currentRow()){
			int pos = ui.listWidget->currentRow() - count - 2;
			return cv::Point(i, pos);
		}
		else{
			count += images[i]->piecesCount + 1;
		}
	}
	return cv::Point(-1, -1);
}

void CV_test::switchImage(){
	cv::Point cur = getCurrentSelection();
	if (cur.y < 0){
		cv::cvtColor(images[cur.x]->sourceImage, tmpRGB, CV_BGR2RGB);
		
	}
	else{
		cv::cvtColor(images[cur.x]->pieces[cur.y]->piece, tmpRGB, CV_BGR2RGB);
	}
	if (cur.y >= 0 && images[cur.x]->pieces[cur.y]->processed == true){
		std::vector<std::vector<cv::Point>> draw_boundary;
		int color = 0;
		for (auto b : images[cur.x]->pieces[cur.y]->boundaries){
			draw_boundary.push_back(std::vector<cv::Point>());
			for (int i = 0; i < b.boundary.size(); i++){
				draw_boundary.back().push_back(b.boundary[i] + cv::Point(0,150 + 100*color));
			}
			for (int i = b.boundary.size() - 1; i >= 0; i--){
				draw_boundary.back().push_back(b.boundary[i] + cv::Point(0,150 + 100*color));
			}
			cv::drawContours(tmpRGB, draw_boundary, color, cv::Scalar(255 * (b.type == Boundary::OUT), 255 * (b.type == Boundary::IN), 255 * (b.type == Boundary::FLAT)), 5);
			color++;
		}
	}
		shown = new QImage((quint8*)tmpRGB.data, tmpRGB.cols, tmpRGB.rows, tmpRGB.step, QImage::Format::Format_RGB888);
		int width = shown->size().width();
		int height = shown->size().height();
		if (shown->size().height() > ui.label->height() || shown->size().width() > ui.label->width()){
			if (width > ui.label->width()){
				int tmp = ui.label->width()*((double)height / width);
				width = ui.label->width();
				height = tmp;
			}
			if (height > ui.label->height()){
				int tmp = ui.label->height()*((double)width / height);
				width = tmp;
				height = ui.label->height();
			}
		}
		QPixmap p = QPixmap::fromImage(*shown).scaled(QSize(width, height));
		QPainter *painter = new QPainter(&p);
		if (cur.y >= 0 && images[cur.x]->pieces[cur.y]->processed == false){
			painter->setBrush(Qt::red);
			int x = (double)images[cur.x]->pieces[cur.y]->corners[0].x / ((double)images[cur.x]->pieces[cur.y]->piece.size().width / (double)p.width());
			int y = (double)images[cur.x]->pieces[cur.y]->corners[0].y / ((double)images[cur.x]->pieces[cur.y]->piece.size().height / (double)p.height());
			painter->drawEllipse(QPoint(x, y), 8, 8);
			painter->setBrush(Qt::green);
			x = (double)images[cur.x]->pieces[cur.y]->corners[1].x / ((double)images[cur.x]->pieces[cur.y]->piece.size().width / (double)p.width());
			y = (double)images[cur.x]->pieces[cur.y]->corners[1].y / ((double)images[cur.x]->pieces[cur.y]->piece.size().height / (double)p.height());
			painter->drawEllipse(QPoint(x, y), 8, 8);
			painter->setBrush(Qt::blue);
			x = (double)images[cur.x]->pieces[cur.y]->corners[2].x / ((double)images[cur.x]->pieces[cur.y]->piece.size().width / (double)p.width());
			y = (double)images[cur.x]->pieces[cur.y]->corners[2].y / ((double)images[cur.x]->pieces[cur.y]->piece.size().height / (double)p.height());
			painter->drawEllipse(QPoint(x, y), 8, 8);
			painter->setBrush(Qt::cyan);
			x = (double)images[cur.x]->pieces[cur.y]->corners[3].x / ((double)images[cur.x]->pieces[cur.y]->piece.size().width / (double)p.width());
			y = (double)images[cur.x]->pieces[cur.y]->corners[3].y / ((double)images[cur.x]->pieces[cur.y]->piece.size().height / (double)p.height());
			painter->drawEllipse(QPoint(x, y), 8, 8);
		}


		ui.label->setPixmap(p);
	
	//qDebug() << getCurrentSelection().x << " " << getCurrentSelection().y;
}

void CV_test::getSolverData(){
	cv::Point3i current(1, 1, 1);
	std::ofstream out("out.txt");
	for (auto x : images){
		out << "Image " << current.x << "\n";
		for (auto y : x->pieces){
			out << "    Piece " << current.x << "." << current.y <<"\n";
			for (Boundary b : y->boundaries){
				qDebug() << current.x << "." << current.y;
				out << "        Boundary " << current.x << "." << current.y << "." << current.z << "   (" << b.type << ")\n";
				//for each boundary, test it against each other boundary
				cv::Point3i current_test(1, 1, 1);
				for (auto xx : images){
					for (auto yy : xx->pieces){
						for (Boundary bb : yy->boundaries){
							if (b.type == Boundary::IN && bb.type == Boundary::OUT || b.type == Boundary::OUT && bb.type == Boundary::IN){
								out << "            Boundary " << current.x << "." << current.y << "." << current.z
									<< " against " << current_test.x << "." << current_test.y << "." << current_test.z << "\n";
								out << "                Length diff squared: " << (b.length - bb.length)*(b.length - bb.length) << "\n";
								out << "                Pip length diff squared: " << (b.lengthFromPip - bb.lengthFromPip)*(b.lengthFromPip - bb.lengthFromPip) << "   ,   "
									<< (b.lengthToPip - bb.lengthToPip)*(b.lengthToPip - bb.lengthToPip) << "   ,   "
									<< ((b.lengthFromPip - b.lengthToPip) - (bb.lengthFromPip - bb.lengthToPip))*((b.lengthFromPip - b.lengthToPip) - (bb.lengthFromPip - bb.lengthToPip)) << "\n";

								double total_dist = 0;
								for (auto p : b.boundary){
									int min = b.length;
									for (auto pp : bb.boundary){
										if (abs(pp.x - p.x) > min && pp.x > p.x) break;
										if (abs(pp.x - p.x) > min) continue;
										int dist = (p.x - pp.x)*(p.x - pp.x) + (p.y - pp.y)*(p.y - pp.y);
										if (dist < min)
											min = dist;
									}
									total_dist += sqrt(min);
								}
								out << "                Boundary distance metric: " << total_dist / b.boundary.size() << "\n";

								double color_dist1 = 0;
								double color_dist2 = 0;
								for (int i = 0; i < bb.color.size(); ++i){
									color_dist1 += pow(((bb.color[i] - b.color[i]).x + (bb.color[i] - b.color[i]).y + (bb.color[i] - b.color[i]).z), 2);
									color_dist2 += pow(((bb.color[i] - b.color[bb.color.size() - 1 - i]).x + (bb.color[i] - b.color[bb.color.size() - 1 - i]).y + (bb.color[i] - b.color[bb.color.size() - 1 - i]).z), 2);
								}
								out << "                Color distance metric: " << std::min(color_dist1,color_dist2)<< "\n";

							}
							++current_test.z;
						}
						++current_test.y;
						current_test.z = 1;
					}
					++current_test.x;
					current_test.y = 1;
				}
				++current.z;
			}
			++current.y;
			current.z = 1;
		}
		++current.x;
		current.y = 1;
	}
	out.close();
}

void CV_test::mouseMoveEvent(QMouseEvent *e){
	if (ui.listWidget->currentRow() == -1) return;
	cv::Point cur = getCurrentSelection();
	int width = shown->size().width();
	int height = shown->size().height();
	if (shown->size().height()>ui.label->height() || shown->size().width() > ui.label->width()){
		if (width > ui.label->width()){
			int tmp = ui.label->width()*((double)height / width);
			width = ui.label->width();
			height = tmp;
		}
		if (height > ui.label->height()){
			int tmp = ui.label->height()*((double)width / height);
			width = tmp;
			height = ui.label->height();
		}
	}
	QPixmap p = QPixmap::fromImage(*shown).scaled(QSize(width, height));

	images[cur.x]->pieces[cur.y]->corners[clickedOn].x = (e->pos() - ui.label->pos()).x() * ((double)shown->width() / width);
	images[cur.x]->pieces[cur.y]->corners[clickedOn].y = ((e->pos() - ui.label->pos()).y() - (ui.label->size().height() - p.height()) / 2) * ((double)shown->height() / height);
	switchImage();
}

void CV_test::mouseReleaseEvent(QMouseEvent *e){
	if (ui.listWidget->currentRow() == -1) return;
	cv::Point cur = getCurrentSelection();
	if (images[cur.x]->pieces[cur.y]->processed == false){
		redrawContours(cur.x, cur.y);
	}
	switchImage();
}

void CV_test::mousePressEvent(QMouseEvent *e){

	if (ui.listWidget->currentRow() == -1) return;

	cv::Point cur = getCurrentSelection();

	int width = shown->size().width();
	int height = shown->size().height();
	if (shown->size().height()>ui.label->height() || shown->size().width() > ui.label->width()){
		if (width > ui.label->width()){
			int tmp = ui.label->width()*((double)height / width);
			width = ui.label->width();
			height = tmp;
		}
		if (height > ui.label->height()){
			int tmp = ui.label->height()*((double)width / height);
			width = tmp;
			height = ui.label->height();
		}
	}
	QPixmap p = QPixmap::fromImage(*shown).scaled(QSize(width, height));

	if ((e->pos() - ui.label->pos()).x() >= 0 && (e->pos() - ui.label->pos()).y() - (ui.label->size().height() - p.height()) / 2 >= 0 && (e->pos() - ui.label->pos()).x() < ui.label->pixmap()->size().width() && (e->pos() - ui.label->pos()).y() - (ui.label->size().height() - p.height()) / 2 < ui.label->pixmap()->size().height()){
		//clicked on picture

		if (cur.y >= 0){
			for (int i = 0; i < 4; i++){
				int x = (double)images[cur.x]->pieces[cur.y]->corners[i].x / ((double)images[cur.x]->pieces[cur.y]->piece.size().width / (double)p.width());
				int y = (double)images[cur.x]->pieces[cur.y]->corners[i].y / ((double)images[cur.x]->pieces[cur.y]->piece.size().height / (double)p.height());

				qDebug() << "Corner " << i + 1 << ":";
				qDebug() << (pow((e->pos().x() - ui.label->pos().x()) - x, 2) + pow((e->pos().y() - ui.label->pos().y() - (ui.label->size().height() - p.height()) / 2) - y, 2));
				if ((pow((e->pos().x() - ui.label->pos().x()) - x, 2) + pow((e->pos().y() - ui.label->pos().y() - (ui.label->size().height() - p.height()) / 2) - y, 2)) < 64){
					clickedOn = i;
					qDebug() << i;
				}
			}
		}
	}
}

CV_test::~CV_test()
{

}
