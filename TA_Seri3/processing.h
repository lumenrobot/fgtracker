#pragma once
#ifndef __processing_h__
#define __processing_h__

#include <C:\opencv\build\include\opencv2\core\core.hpp>
#include <C:\opencv\build\include\opencv\cvaux.h>
#include <C:\opencv\build\include\opencv2\imgproc\imgproc.hpp>
#include <C:\opencv\build\include\opencv2\highgui\highgui.hpp>
#include "stdafx.h"
#include <cv.h>
#include <C:\opencv\build\include\opencv2\opencv.hpp>
#include <iostream>
#include <vector>

#include <stdio.h>
#include "stdlib.h"
#include <iostream>

#include <string.h>
#include <ctype.h>
#include <string>

#include "bgfg_cb.h";
#include "cvaux.h";
#include "cxmisc.h";

using namespace cv;
using namespace std;

typedef struct ujikalman{
	clock_t kalman_start,kalman_end;
	int cap;
	int max;
	int objcnt;
};

typedef struct ujikoresponden{
	clock_t korespond_start,korespond_end;
	int korespondCount;
	int max;
	int objcnt;
	bool stimulus;
};

typedef struct ujisinglecam{
	clock_t begin_time,end_time;
	int captureCount;
	int max;
	int objcnt;
};

typedef struct ujimulticam{
	clock_t multi_start,multi_end;
	int MulticamCount;
	int max;
	int objcnt;
};

typedef struct ujitotal{
	clock_t total_start,total_end;
	int totalCount;
	int max;
	int objcnt;
};

typedef struct FrameElement{
	bool stat[2];
	bool lock;
};

typedef struct choosenCam{
	int i;
	int j;
};

typedef struct ObjectSubstraction{
	Point pusat;
	Point bottom;
	Point previous;
	Point labelpoint;
	int indexobj;
};

typedef struct Line{
	Point c11;
	Point c12;
	Point test;
	bool learn;
	bool lock11;
	bool lock12;
	int mark;
};

typedef struct KObject{
	KalmanFilter KF;
	int width;
	int height;
	Point pusat;
	Point bottom;
	bool trackStat;
	bool switchon;
	Mat_<float> measurement;
	String Label;
	Point LabelPoint;
	bool lock;
	bool locklama;
	int framelewat;
};

bool cekKanan(Line ls, ObjectSubstraction obj);
vector<vector <Point>> findKonturMOG(cv::Mat fore);
void MOGForLine(BackgroundSubtractorMOG2 bg,Mat &fore,Mat frame);
Mat skipNFramesMOG(VideoCapture capture, int n);
IplImage* skipNFrames(CvCapture* capture, int n);
void activateUjiSistem(bool &totalTrack);
void activateTrackMulticam(bool &trackMulticam);
void activateCapture(bool &cap);
void captureKalman(bool &trackKalman);
void activateMatching(bool &cap);
void DrawHasil(Mat duplicate, String path,String filename,String type);
double diffMilisecond(clock_t start, clock_t end );
double hitungErrorKalman(ObjectSubstraction obs, KObject KF);
void unLockFOV(Line &Ls, int i, int j);
void LockFOV(Line &Ls, int i, int j);
void setAutoPointLine(Line &Ls,int in1, int in2, CvSize sz, int &iterate);
bool bandingGarisBottom(Line Ls, CvSize frame, ObjectSubstraction O);
int binarySearch(vector<ObjectSubstraction> &Obj, ObjectSubstraction x);
void PilihTerkecil(vector<ObjectSubstraction> &Obj, Line Ls,ObjectSubstraction &obs, int &ind);
void SelectionSort(vector<ObjectSubstraction> &Obj, Line Ls);
void pilihPointCam(Line Ls, vector<ObjectSubstraction> &allbottom, CvSize frame_size, int idx);
double cekGrad(Line Ls, Point bottom);
void gambarFOVLine(Mat &duplicate, ObjectSubstraction &obs, Point &labelpoint, Line Ls, int i, int j);
void gambarPoint(Mat &duplicate, Point a);
double gradientLine(Line Ls);
void resetPointLine(Line &Ls, int &iterate);
void setPointLine(Line &Ls,Point x, int in1, int in2, int &iterate);
vector<Mat> objectmatching(Mat duplicate[], vector<Point> objbottom[], Point c11[],Point c12[],Point c21[],Point c22[], bool trackStat[], int cam );
void buatFOVLx(Mat &duplicate, Point &labelpoint, Point &bottom, Line Ls);
void buatFOVLine(Mat &duplicate, Point &bottom, Point &labelpoint, Point start1, Point end1, Point start2, Point end2);
void resetKalman(KalmanFilter &KF,Mat_<float> &measurement, Point pusat);
void jarakDeteksiTrack(Mat &duplicate,vector<ObjectSubstraction> &ObjAll, KObject &KF);
void kalmanTracker(KObject &KF, Mat &duplicate, Point &pusat);
void kumpulanKontur(vector<vector <Point>> contours, vector<vector<Point> > &contours_poly, vector<Rect> &boundRect, vector<Point2f> &center, vector<float> &radius);
void buatBSTracker(cv::Mat &duplicate, ObjectSubstraction &Obj, KObject &KF, vector<vector <Point>> contours, vector<vector<Point> > contours_poly, vector<int> idx,vector<int> idxdiff, vector<Rect> &boundRect, vector<ObjectSubstraction> &ObjAll);
//void buatBSTracker(cv::Mat &duplicate, Point &labelpoint, Point &pusat, Point &bottom, int &trakwin_w, int &trakwin_h, vector<vector <Point>> contours, vector<vector<Point> > contours_poly,int idx, vector<Rect> &boundRect);
vector<vector <Point>> cariKontur(cv::Mat ImaskCB);
double distance_to_Line(cv::Point line_start, cv::Point line_end, cv::Point point);
double distance_to_point(cv::Point line_start, cv::Point line_end);
int tipotY(int x,int x1,int x2,int y1,int y2);
int tipotX(int y,int x1,int x2,int y1,int y2);
void removePepperNoise(cv::Mat &mask);
void help(void);
void find_human(const vector<vector<Point> >& squares, vector<int>& index, vector<int> &idxdiff);
void find_largest_square(const vector<vector<Point> >& squares, vector<int>& index);
void setIsTrack(bool& tk);
bool getIsTrack();
void LearnFindBGR(CvBGCodeBookModel *model, IplImage *yuvImage, IplImage *ImaskCodeBook, IplImage *ImaskCodeBookCC);
void AlokasiCodeBook(IplImage *rawImage, IplImage *yuvImage, IplImage *ImaskCodeBook, IplImage *ImaskCodeBookCC);

#endif