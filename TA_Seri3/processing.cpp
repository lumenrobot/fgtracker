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
#include <cmath>

#include <string.h>
#include <ctype.h>
#include <string>
#include "bgfg_cb.h";
#include "cvaux.h";
#include "cxmisc.h";
#include "processing.h";

using namespace cv;
using namespace std;

bool isTrack = false;

bool cekKanan(Line ls, ObjectSubstraction obj){
	bool kanan;
	if (gradientLine(ls)>=0){
		if (cekGrad(ls,obj.bottom)>=0){
			kanan = true;
		} else {
			kanan = false;
		}
	} else {
		if (cekGrad(ls,obj.bottom)>=0){
			kanan = false;
		} else {
			kanan = true;
		}
	}
	return kanan;
}

vector<vector <Point>> findKonturMOG(cv::Mat fore){
	cv::Mat kontur;
	vector<vector <Point>> fixkontur;
	cv::Canny(fore,fore,1,2,3,false);
	cv::findContours( fore, fixkontur, CV_RETR_LIST, CV_CHAIN_APPROX_NONE );
	return fixkontur;
}

void MOGForLine(BackgroundSubtractorMOG2 bg, Mat &fore,Mat frame){
	Mat back;
	cv::Mat thresholded2;
	Mat kernel5x5 = getStructuringElement(MORPH_RECT, Size(10, 10));
	cv::medianBlur(frame,back,3);
	bg.operator ()(frame,fore);
    cv::threshold(fore,thresholded2,70.0f,255,CV_THRESH_BINARY);
	cv::cvtColor(thresholded2,fore,CV_GRAY2RGB);
    cv::erode(fore,fore,cv::Mat());
    cv::dilate(fore,fore,cv::Mat());
	cv::dilate(fore,fore,cv::Mat());
	cv::morphologyEx(fore,fore,cv::MORPH_CLOSE,kernel5x5);
}

Mat skipNFramesMOG(VideoCapture capture, int n)
{
	Mat image;
    for(int i = 0; i < n; ++i)
    {
		capture >> image;
    }

	return (image);
}

IplImage* skipNFrames(CvCapture* capture, int n)
{
    for(int i = 0; i < n; ++i)
    {
        if(cvQueryFrame(capture) == NULL)
        {
            return NULL;
        }
    }

    return cvQueryFrame(capture);
}


void activateUjiSistem(bool &totalTrack){
	if (totalTrack == true){
		totalTrack = false;
		printf("Capture SISTEM false \n");
	}else{
		totalTrack = true;
		printf("Capture SISTEM true \n");
	}
}

void activateTrackMulticam(bool &trackMulticam){
	if (trackMulticam == true){
		trackMulticam = false;
		printf("Capture MULTICAM false \n");
	}else{
		trackMulticam = true;
		printf("Capture MULTICAM true \n");
	}
}

void activateCapture(bool &cap){
	if (cap == true){
		cap = false;
		printf("Capture SINGLECAM false \n");
	}else{
		cap = true;
		printf("Capture SINGLECAM true \n");
	}
}

void captureKalman(bool &trackKalman){
	if (trackKalman==true){
		trackKalman =false;
		printf("Kalman capture false \n");
	}else{
		trackKalman =true;
		printf("Kalman capture true \n");
	}
}

void activateMatching(bool &cap){
	if (cap==true){
		cap = false;
		printf("Matching capture false \n");
	}else{
		cap = true;
		printf("Matching capture true \n");
	}
}


void DrawHasil(Mat duplicate, String path,String filename,String type){
	path = path+filename+type;
	vector<int> compression_params; //vector that stores the compression parameters of the image
	compression_params.push_back(CV_IMWRITE_JPEG_QUALITY); //specify the compression technique
	compression_params.push_back(98); //specify the compression quality
	imwrite(path,duplicate,compression_params);
}

double diffMilisecond(clock_t clock1, clock_t clock2){
	double diffticks = clock2-clock1;
    double diffms    = diffticks / ( CLOCKS_PER_SEC / 1000 );
	return diffms;
}

double hitungErrorKalman(ObjectSubstraction obs, KObject KF){
	return abs(distance_to_point(obs.pusat,KF.pusat));
}

void unLockFOV(Line &Ls, int i, int j){
	if (Ls.lock12==true){
		Ls.lock12 = false;
		printf("FOV Dot c12 [%d][%d] unlocked\n",i,j);
	}else if (Ls.lock11==true){
		Ls.lock11 = false;
		printf("FOV Dot c11 [%d][%d] unlocked\n",i,j);
	}
}

void LockFOV(Line &Ls, int i, int j){
	if (Ls.lock11==false){
		Ls.lock11 = true;
		printf("FOV Dot c11 [%d][%d] locked\n",i,j);
	}else if (Ls.lock12==false){
		Ls.lock12 = true;
		printf("FOV Dot c12 [%d][%d] locked\n",i,j);
	}
}

void setAutoPointLine(Line &Ls,int in1, int in2, CvSize sz, int &iterate){
	int x1,x2,y1,y2;
	if (Ls.c11.x == Ls.c12.x){
		x1 = Ls.c11.x;
		y1 = 0;
		x2 = Ls.c12.x;
		y2 = sz.height;
	}else if(Ls.c11.y == Ls.c12.y){
		x1 = 0;
		y1 = Ls.c11.y;
		x2 = sz.width;
		y2 = Ls.c12.y;
	}else{
		x1 = in1;
		y1 = tipotY(x1,Ls.c11.x,Ls.c11.y,Ls.c12.x,Ls.c12.y);
		x2 = in2;
		y2 = tipotY(x2,Ls.c11.x,Ls.c11.y,Ls.c12.x,Ls.c12.y);
	}
	Ls.c11 = Point(x1,y1);
	Ls.c12 = Point(x2,y2);
	printf("c11 %d %d \n",Ls.c11.x,Ls.c11.y);
	printf("c12 %d %d \n",Ls.c12.x,Ls.c12.y);
	Ls.mark = 0;
	Ls.learn = false;
	iterate++;
	printf("nilai iterate = %d \n",iterate);
	printf("\n");		
}


bool bandingGarisBottom(Line Ls, CvSize frame, ObjectSubstraction O){
	bool terdekat;
	Line tepi;
	if (gradientLine(Ls)>=0){
		tepi.c11=Point(frame.width,0);
		tepi.c12=Point(frame.width,frame.height);
	}else{
		tepi.c11=Point(0,0);
		tepi.c12=Point(0,frame.height);
	}
	if(abs(distance_to_Line(Ls.c11,Ls.c12,O.bottom))>abs(distance_to_Line(tepi.c11,tepi.c12,O.bottom))){
		terdekat = true;
	}else{
		terdekat = false;
	}
	return terdekat;
}

int binarySearch(vector<ObjectSubstraction> &Obj, ObjectSubstraction x)
{   
	int elemen = x.indexobj;
	int size = Obj.size();
	int posisi = 0;   
	for (int i = 0; i < size ; i++){
		if (Obj.at(i).indexobj == x.indexobj){
			posisi = i;
		}
	}
	return posisi;
}

void PilihTerkecil(vector<ObjectSubstraction> &Obj, Line Ls,ObjectSubstraction &obs,int &ind)
{
	int Size = Obj.size();
	int i; 
	int kecil;
	ObjectSubstraction temp = Obj.at(0);
	double dist = abs(distance_to_Line(Ls.c11,Ls.c12,Obj.at(0).bottom));
	kecil = 0;
	for(i=0;i<Size;i++){
		if (abs(distance_to_Line(Ls.c11,Ls.c12,Obj.at(i).bottom))<dist) {
			temp = Obj.at(i);
			kecil = i;
		}
	}
	obs = temp;
	ind = kecil;
}

void SelectionSort(vector<ObjectSubstraction> &Obj, Line Ls)// membuat fungsi untuk memanggil perhitungan dengan selection
{
	int Size = Obj.size();
	int i,j ; 
	int kecil;
	ObjectSubstraction temp;//mendeklarasikan variabel
	for(i=0;i<Size-1;i++){
		// melakukan perulangan sebanyak nilai array -1
		kecil=i;
		//menyimpan nilai index dalam variabel baru kecil
		for(j=i+1;j<Size;j++)
		//perulangan untuk menghitung index i+1 atau sebelahnya.
		{
			if(abs(distance_to_Line(Ls.c11,Ls.c12,Obj.at(kecil).bottom))>abs(distance_to_Line(Ls.c11,Ls.c12,Obj.at(j).bottom)))
			//mengecek kondisi dimana nilai array kecil lebih besar dari j. maka di ubah pivotnya
			{
				kecil=j;
				//mengeset nilai kecil dengan nilai j
			}
		}

		if(kecil != i){//mengecek dimana nilai kecil tidak sama dengan i
			temp=Obj.at(i);// dilakukan pertukaran nilai dengan variabel bantu temp
            Obj.at(i)=Obj.at(kecil);// mengisi nilai arrai index ke i dengan array index kecil
            Obj.at(kecil)=temp;//mengisi nilai array index kecil dengan temporari
		}
	}
}

void pilihPointCam(Line Ls, vector<ObjectSubstraction> &allbottom, CvSize frame_size, int posI){
	
	Line Lbts;
	if (gradientLine(Ls)>=0){
		Lbts.c11 = Point(0,0);
		Lbts.c12 = Point(0,frame_size.height);
	}else{
		Lbts.c11 = Point(frame_size.width,0);
		Lbts.c12 = Point(frame_size.width,frame_size.height);
	}
	//ObjectSubstraction temp;
	//PilihTerkecil(allbottom,Lbts,temp,posI);
	SelectionSort(allbottom,Lbts);
}

double cekGrad(Line Ls, Point bottom){
	return ((((bottom.x)-(Ls.c11.x))*(Ls.c12.y-Ls.c11.y))-((Ls.c12.x-Ls.c11.x)*(bottom.y-Ls.c11.y)));
}

void gambarFOVLine(Mat &duplicate, ObjectSubstraction &obs, Point &labelpoint, Line Ls,int i, int j){
	long double dtc = 0; //penghitung jarak
	String dist; //penghitung jarak string
	if((i<j && cekKanan(Ls,obs)) && isTrack){
		cv::line(duplicate,Ls.c11,Ls.c12,Scalar(255,0,0),1,8,0);
		dtc = distance_to_Line(Ls.c11,Ls.c12,obs.bottom);
		dist = "detected "+std::to_string(dtc);
		cv::putText(duplicate,dist, labelpoint,1,1,CV_RGB(255, 255, 255),1,8,false);
	}else if((i>j && !cekKanan(Ls,obs) && isTrack)){
		cv::line(duplicate,Ls.c11,Ls.c12,Scalar(255,0,0),1,8,0);
		dtc = distance_to_Line(Ls.c11,Ls.c12,obs.bottom);
		dist = "detected "+std::to_string(dtc);
		cv::putText(duplicate,dist, labelpoint,1,1,CV_RGB(255, 255, 255),1,8,false);
	}else{
		cv::line(duplicate,Ls.c11,Ls.c12,Scalar(0,255,255),1,8,0);
	}
}

void gambarPoint(Mat &duplicate, Point a){
	cv::circle(duplicate,a,2,Scalar(255,0,255),2,8,0);
}

double gradientLine(Line Ls){
	double a;
	a = (Ls.c12.y-Ls.c11.y)+0.0001/(Ls.c12.x-Ls.c11.x)+0.00001;
	return a;
}

void resetPointLine(Line &Ls, int &iterate){
	Ls.c11 = Point (0,0);
	Ls.c12 = Point (0,0);
	Ls.mark = 0;
	Ls.test = Point(0,0);
	iterate = 0;
}

void setPointLine(Line &Ls,Point x, int in1, int in2, int &iterate){
	int x1,x2,y1,y2;
	if (Ls.mark==0){
		Ls.c11 = x;
		Ls.mark = Ls.mark+1;
		printf("Ls.mark %d \n",Ls.mark);
		Ls.learn = true;
	}else if (Ls.mark==1){
		Ls.c12 = x;
		Ls.mark = Ls.mark+1;
		printf("Ls.mark %d \n",Ls.mark);
		Ls.learn = true;
	}else if (Ls.mark==2){
		x1 = in1;
		y1 = tipotY(x1,Ls.c11.x,Ls.c11.y,Ls.c12.x,Ls.c12.y);
		x2 = in2;
		y2 = tipotY(x2,Ls.c11.x,Ls.c11.y,Ls.c12.x,Ls.c12.y);
		Ls.c11 = Point(x1,y1);
		Ls.c12 = Point(x2,y2);
		printf("c11 %d %d \n",Ls.c11.x,Ls.c11.y);
		printf("c12 %d %d \n",Ls.c12.x,Ls.c12.y);
		Ls.mark = 0;
		Ls.learn = false;
		iterate++;
		printf("nilai iterate = %d \n",iterate);
		printf("\n");		
	}
}

void AlokasiCodeBook(IplImage *rawImage, IplImage *yuvImage, IplImage *ImaskCodeBook, IplImage *ImaskCodeBookCC){
	yuvImage = cvCloneImage(rawImage);
    ImaskCodeBook = cvCreateImage( cvGetSize(rawImage), IPL_DEPTH_8U, 1 );
    ImaskCodeBookCC = cvCreateImage( cvGetSize(rawImage), IPL_DEPTH_8U, 1 );
    cvSet(ImaskCodeBook,cvScalar(255));
}

void LearnFindBGR(CvBGCodeBookModel *model, IplImage *yuvImage, IplImage *ImaskCodeBook, IplImage *ImaskCodeBookCC){
	//Image Processing..... Buat ngurangin noise
	int filterSize = 30;
	IplConvKernel *convKernel = cvCreateStructuringElementEx(filterSize, filterSize, (filterSize - 1) / 2, (filterSize - 1) / 2, CV_SHAPE_RECT, NULL);
	// Find foreground by codebook method
	//yang ini  asli CODEBOOK....
    cvBGCodeBookDiff( model, yuvImage, ImaskCodeBook );
	//---------------------------
	cvErode(ImaskCodeBook,ImaskCodeBook,NULL,1);
	//cvSmooth(ImaskCodeBook,ImaskCodeBook,CV_MEDIAN,7,7);//indoor

	//yang ini  asli CODEBOOK....
    cvCopy(ImaskCodeBook,ImaskCodeBookCC);		
	//---------------------------

	cvMorphologyEx(ImaskCodeBookCC,ImaskCodeBookCC, NULL, convKernel, CV_MOP_CLOSE);
	cvMorphologyEx(ImaskCodeBook,ImaskCodeBook, NULL, convKernel, CV_MOP_CLOSE);
	
	//yang ini  asli CODEBOOK....
    cvSegmentFGMask( ImaskCodeBookCC );
	//---------------------------
	cvSmooth(ImaskCodeBookCC,ImaskCodeBookCC,CV_MEDIAN,7,7);
}

void buatFOVLx(Mat &duplicate, Point &labelpoint, Point &bottom, Line Ls){
	long double dtc = 0; //penghitung jarak
	String dist; //penghitung jarak string
	if (((((bottom.x)-(Ls.c11.x))*(Ls.c12.y-Ls.c11.y))-((Ls.c12.x-Ls.c11.x)*(bottom.y-Ls.c11.y)))>0 && isTrack){
		cv::line(duplicate,Ls.c11,Ls.c12,Scalar(255,0,0),2,8,0);
		dtc = distance_to_Line(Ls.c11,Ls.c11,bottom);
		dist = "detected "+std::to_string(dtc);
		cv::putText(duplicate,dist, labelpoint,1,1,CV_RGB(255, 255, 255),1,8,false);
	}else{
		cv::line(duplicate,Ls.c11,Ls.c12,Scalar(0,255,255),2,8,0);
	}
}

void buatFOVLine(Mat &duplicate, Point &bottom, Point &labelpoint, Point start1, Point end1, Point start2, Point end2){
	long double dtc = 0; //penghitung jarak
	String dist; //penghitung jarak string
	if (((((bottom.x)-(start1.x))*(end1.y-start1.y))-((end1.x-start1.x)*(bottom.y-start1.y)))>0 && isTrack){
		cv::line(duplicate,start1,end1,Scalar(255,0,0),1,8,0);
		dtc = distance_to_Line(start1,end1,bottom);
		dist = "detected "+std::to_string(dtc);
		cv::putText(duplicate,dist, labelpoint,1,1,CV_RGB(255, 255, 255),1,8,false);
	}else{
		cv::line(duplicate,start1,end1,Scalar(0,255,255),1,8,0);
	}
	if (((((bottom.x)-(start2.x))*(end2.y-start2.y))-((end2.x-start2.x)*(bottom.y-start2.y)))<0 && isTrack){
		cv::line(duplicate,start2,end2,Scalar(255,0,0),1,8,0);
		dtc = distance_to_Line(start2,end2,bottom);
		dist = "detected "+std::to_string(dtc);
		cv::putText(duplicate,dist, labelpoint,1,1,CV_RGB(255, 255, 255),1,8,false);
	}else{
		cv::line(duplicate,start2,end2,Scalar(0,255,255),1,8,0);
	}
}

void resetKalman(KalmanFilter &KF,Mat_<float> &measurement, Point pusat){
	KF = cv::KalmanFilter(4, 2, 0);
	KF.measurementMatrix;
	KF.controlMatrix;
	KF.transitionMatrix;
	//state[i](4, 1); // (x, y, Vx, Vy)
	Mat processNoise(4, 1, CV_32F);
	measurement = Mat_<float>(2,1);
	measurement.setTo(Scalar(0));
	KF.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,2,0,   0,1,0,2,  0,0,1,0,  0,0,0,1);
	setIdentity(KF.measurementMatrix);
	//setIdentity(KF.processNoiseCov, Scalar::all(1));
	setIdentity(KF.processNoiseCov, Scalar::all(1e-4));//default
	//setIdentity(KF.processNoiseCov, Scalar::all(1e-2));
	//setIdentity(KF.processNoiseCov, Scalar::all(0));
	//setIdentity(KF.measurementNoiseCov, Scalar::all(1));
	//setIdentity(KF.measurementNoiseCov, Scalar::all(1e-2));
	setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));//default
	//setIdentity(KF.measurementNoiseCov, Scalar::all(0));
	setIdentity(KF.errorCovPost, Scalar::all(.1));
	KF.statePre.at<float>(0) = pusat.x;
	KF.statePre.at<float>(1) = pusat.y;
	KF.statePre.at<float>(2) = 0;
	KF.statePre.at<float>(3) = 0;
	KF.statePost.at<float>(0) = pusat.x;
	KF.statePost.at<float>(1) = pusat.y;
	KF.statePost.at<float>(2) = 0;
	KF.statePost.at<float>(3) = 0;
}

void jarakDeteksiTrack(Mat &duplicate,vector<ObjectSubstraction> &ObjAll, KObject &KF){
	Scalar color = Scalar(255,255,255);
	String dist;
	for (int i = 0 ; i < ObjAll.size() ; i++){
		float dst = 0;
		dst = abs(distance_to_point(ObjAll.at(i).pusat,KF.pusat));
		dst = std::floor(dst * 100 + 0.5)/100;
		std::ostringstream buff;
		buff<<dst;
		dist = "e "+buff.str();
		cv::putText(duplicate,dist,ObjAll.at(i).labelpoint,1,0.8,color,1,8,false);
	}
}

void kalmanTracker(KObject &KF, Mat &duplicate, Point &pusat){
	if (!isTrack){
		KF.KF.statePre.at<float>(0) = pusat.x;
		KF.KF.statePre.at<float>(1) = pusat.y;
		KF.KF.statePre.at<float>(2) = 0;
		KF.KF.statePre.at<float>(3) = 0;
		KF.KF.statePost.at<float>(0) = pusat.x;
		KF.KF.statePost.at<float>(1) = pusat.y;
		KF.KF.statePost.at<float>(2) = 0;
		KF.KF.statePost.at<float>(3) = 0;
	}else{
		Mat prediction = KF.KF.predict();
		Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
		KF.measurement(0) = pusat.x;
		KF.measurement(1) = pusat.y;
		Point measPt(KF.measurement(0),KF.measurement(1));
		Mat estimated = KF.KF.correct(KF.measurement);
		Point statePt(estimated.at<float>(0),estimated.at<float>(1));
		KF.pusat = predictPt; //estimasi
		cv::circle(duplicate,predictPt,2,Scalar(0,255,0),2,8,0);
		cv::Point tl(statePt.x-(KF.width/2),statePt.y-(KF.height/2));
		cv::Point br(statePt.x+(KF.width/2),statePt.y+(KF.height/2));
		rectangle( duplicate,tl, br, Scalar(0,255,0), 2, 8, 0 );
		KF.bottom = Point((KF.pusat.x),(KF.pusat.y+(KF.height/2)));
	}
}

void buatBSTracker(cv::Mat &duplicate, ObjectSubstraction &Obj, KObject &KF, vector<vector <Point>> contours, vector<vector<Point> > contours_poly,vector<int> idx,vector<int> idxdiff, vector<Rect> &boundRect, vector<ObjectSubstraction> &ObjAll){
	Point oldPusat = Obj.pusat;
	Obj.previous = oldPusat;
	ObjectSubstraction temporary;
	bool wtrak = false;
	setIsTrack(wtrak);
	for( int i = 0; i < contours.size(); i++ )
	{
		for ( int j = 0; j < idx.size(); j++ )
		{
			if (i == idx.at(j)){
				wtrak=true;
				setIsTrack(wtrak);
				Scalar color = Scalar(255,0,255);
				//drawContours( duplicate, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
				//cv::putText(duplicate,"true",boundRect[i].tl(),1,1,color,1,8,false);
				rectangle( duplicate, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
				Obj.labelpoint = boundRect[i].tl();
				Obj.pusat = Point((boundRect[i].x+ (boundRect[i].width/2)),(boundRect[i].y+ (boundRect[i].height/2)));
				KF.width = boundRect[i].width;
				KF.height = boundRect[i].height;
				cv::circle(duplicate,Obj.pusat,2,color,2,8,0);
				Obj.bottom = Point((boundRect[i].x+ (boundRect[i].width/2)),(boundRect[i].y+ (boundRect[i].height)));
				cv::circle(duplicate,Obj.bottom,2,color,2,8,0);
				temporary.pusat = Obj.pusat;
				temporary.indexobj = i;
				temporary.bottom = Obj.bottom;
				temporary.labelpoint = Obj.labelpoint;
				ObjAll.push_back(temporary);
				break;
			}
		}
	}

	for( int i = 0; i < contours.size(); i++ )
	{
		for ( int j = 0; j < idxdiff.size(); j++ )
		{
			if( i == idxdiff.at(j)){
				wtrak=true;
				setIsTrack(wtrak);
				Scalar color = Scalar(0,255,255);
				//drawContours( duplicate, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
				//cv::putText(duplicate,"false",boundRect[i].tl(),1,1,color,1,8,false);
				rectangle( duplicate, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
				Obj.labelpoint = boundRect[i].tl();
				Obj.pusat = Point((boundRect[i].x+ (boundRect[i].width/2)),(boundRect[i].y+ (boundRect[i].height/2)));
				KF.width = boundRect[i].width;
				KF.height = boundRect[i].height;
				cv::circle(duplicate,Obj.pusat,2,color,2,8,0);
				Obj.bottom = Point((boundRect[i].x+ (boundRect[i].width/2)),(boundRect[i].y+ (boundRect[i].height)));
				cv::circle(duplicate,Obj.bottom,2,color,2,8,0);
				temporary.pusat = Obj.pusat;
				temporary.indexobj = i;
				temporary.bottom = Obj.bottom;
				temporary.labelpoint = Obj.labelpoint;
				ObjAll.push_back(temporary);
				break;
			}
		}
	}

	if(ObjAll.size()>=1){
		double min = abs(distance_to_point(oldPusat,ObjAll.at(0).pusat));
		Obj.indexobj = ObjAll.at(0).indexobj;
		Obj.pusat = ObjAll.at(0).pusat;
		for (int i = 0; i < ObjAll.size() ; i++){
			double distance = abs(distance_to_point(oldPusat,ObjAll.at(i).pusat));
			if (distance<min){
				min = distance;
				Obj.indexobj = ObjAll.at(i).indexobj;
				Obj.pusat = ObjAll.at(i).pusat;
			}
		}
		KF.width = boundRect[Obj.indexobj].width;
		KF.height = boundRect[Obj.indexobj].height;
		Obj.labelpoint = boundRect[Obj.indexobj].tl();
		Obj.bottom = Point((boundRect[Obj.indexobj].x+ (boundRect[Obj.indexobj].width/2)),(boundRect[Obj.indexobj].y+ (boundRect[Obj.indexobj].height)));
	}
}

vector<vector <Point>> cariKontur(cv::Mat ImaskCB){
	cv::Mat kontur;
	vector<vector <Point>> fixkontur;
	//cv::Canny(ImaskCB,kontur,1,2,3,false);
	//cv::findContours(kontur,fixkontur,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);//simple
	//Mat dup = ImaskCB;
	cv::findContours( ImaskCB, fixkontur, CV_RETR_LIST, CV_CHAIN_APPROX_NONE );
	return fixkontur;
}

void kumpulanKontur(vector<vector <Point>> contours, vector<vector<Point> > &contours_poly, vector<Rect> &boundRect, vector<Point2f> &center, vector<float> &radius){
	//Kasih bounding box di bagian yang sudah ditentukan
	for( int i = 0; i < contours.size(); i++ )
	{ 
		approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
		boundRect[i] = boundingRect( Mat(contours_poly[i]) );
		minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
	}
}


void help(void)
{
    printf("\nLearn background and find foreground using simple average and average difference learning method:\n"
        "\nUSAGE:\nbgfg_codebook [--nframes=300] [movie filename, else from camera]\n"
        "***Keep the focus on the video windows, NOT the consol***\n\n"
        "INTERACTIVE PARAMETERS:\n"
        "\tESC,q,Q  - quit the program\n"
        "\th	- print this help\n"
        "\tp	- pause toggle\n"
        "\ts	- single step\n"
        "\tr	- run mode (single step off)\n"
        "=== AVG PARAMS ===\n"
        "\t-    - bump high threshold UP by 0.25\n"
        "\t=    - bump high threshold DOWN by 0.25\n"
        "\t[    - bump low threshold UP by 0.25\n"
        "\t]    - bump low threshold DOWN by 0.25\n"
        "=== CODEBOOK PARAMS ===\n"
        "\ty,u,v- only adjust channel 0(y) or 1(u) or 2(v) respectively\n"
        "\ta	- adjust all 3 channels at once\n"
        "\tb	- adjust both 2 and 3 at once\n"
        "\ti,o	- bump upper threshold up,down by 1\n"
        "\tk,l	- bump lower threshold up,down by 1\n"
        "\tSPACE - reset the model\n"
		"=== FOV PARAMS ===\n"
		"\tx    - learn FOV mode \n"
		"\tz    - lock/unlock FOV \n"
		"=== FILE FRAME SAVE ===\n"
		"\t5    - Uji Kalman \n"
		"\t6	- Uji Korespondensi Obyek \n"
		"\t7	- Uji Layer SingleCam \n"
		"\t8	- Uji Layer MultiCam \n"
		"\t9	- Uji Sistem \n"
        );
}


void removePepperNoise(cv::Mat &mask)
{
    for ( int y=2; y<mask.rows-2; y++ ) {
        uchar *pUp2 = mask.ptr(y-2);
        uchar *pUp1 = mask.ptr(y-1);
        uchar *pThis = mask.ptr(y);
        uchar *pDown1 = mask.ptr(y+1);
        uchar *pDown2 = mask.ptr(y+2);
        pThis += 2;
        pUp1 += 2;
        pUp2 += 2;
        pDown1 += 2;
        pDown2 += 2;

        for (int x=2; x<mask.cols-2; x++) {
            uchar value = *pThis; // Get this pixel value (0 or 255). // Check if this is a black pixel that is surrounded by white pixels
            if (value == 0) {
                bool above, left, below, right, surroundings;
                above = *(pUp2 - 2) && *(pUp2 - 1) && *(pUp2) && *(pUp2 + 1) && *(pUp2 + 2);
                left = *(pUp1 - 2) && *(pThis - 2) && *(pDown1 - 2);
                below = *(pDown2 - 2) && *(pDown2 - 1) && *(pDown2) && *(pDown2 + 1) && *(pDown2 + 2);
                right = *(pUp1 + 2) && *(pThis + 2) && *(pDown1 + 2);
                surroundings = above && left && below && right;
                if (surroundings == true) {
                    // Fill the whole 5x5 block as white. Since we know
                    // the 5x5 borders are already white, we just need to
                    // fill the 3x3 inner region.
                    *(pUp1 - 1) = 255;
                    *(pUp1 + 0) = 255;
                    *(pUp1 + 1) = 255;
                    *(pThis - 1) = 255;
                    *(pThis + 0) = 255;
                    *(pThis + 1) = 255;
                    *(pDown1 - 1) = 255;
                    *(pDown1 + 0) = 255;
                    *(pDown1 + 1) = 255;
                    // Since we just covered the whole 5x5 block with
                    // white, we know the next 2 pixels won't be black,
                    // so skip the next 2 pixels on the right.
                    pThis += 2;
                    pUp1 += 2;
                    pUp2 += 2;
                    pDown1 += 2;
                    pDown2 += 2;
                }
            }
            // Move to the next pixel on the right.
            pThis++;
            pUp1++;
            pUp2++;
            pDown1++;
            pDown2++;
        }
    }
}

void setIsTrack(bool& tk){
	isTrack = tk;
}

bool getIsTrack(){
	return (isTrack);
}

void find_human(const vector<vector<Point> >& squares, vector<int>& index, vector<int> &idxdiff)
{
	index.clear();
	idxdiff.clear();
	if (!squares.size())
    {
		// no squares detected
        return;
    }
    const int n_points = 4;
    for (size_t i = 0; i < squares.size(); i++)
    {
        // Convert a set of 4 unordered Points into a meaningful cv::Rect structure.
        Rect rectangle = boundingRect(Mat(squares[i]));

        // Store the index position of the biggest square found
		if ((rectangle.width < rectangle.height))
        {		
			index.push_back(i);
		}else{
			idxdiff.push_back(i);
		}
    }
    
}


void find_largest_square(const vector<vector<Point> >& squares, vector<int>& index)
{
	if (!squares.size())
    {
		// no squares detected
        return;
    }

    int max_width = 0;
    int max_height = 0;
    int max_square_idx = 0;
    const int n_points = 4;

    for (size_t i = 0; i < squares.size(); i++)
    {
        // Convert a set of 4 unordered Points into a meaningful cv::Rect structure.
        Rect rectangle = boundingRect(Mat(squares[i]));

		//cout << "find_largest_square: #" << i << " rectangle x:" << rectangle.x << " y:" << rectangle.y << " " << rectangle.width << "x" << rectangle.height << endl;

        // Store the index position of the biggest square found
        if ((rectangle.width >= max_width) && (rectangle.height >= max_height))
        {
			max_width = rectangle.width;
            max_height = rectangle.height;
            max_square_idx = i;
		}
    }
	index.push_back(max_square_idx);
}

double distance_to_Line(cv::Point line_start, cv::Point line_end, cv::Point point)
{
	double normalLength = _hypot(line_end.x - line_start.x, line_end.y - line_start.y);
	double distance = (double)((point.x - line_start.x) * (line_end.y - line_start.y) - (point.y - line_start.y) * (line_end.x - line_start.x)) / normalLength;
	return distance;
}

double distance_to_point(cv::Point line_start, cv::Point line_end){
	double normalLength = _hypot(line_end.x - line_start.x, line_end.y - line_start.y);
	//double normalLength = cv::norm(line_start-line_end);
	double distance = normalLength;
	return distance;
}

int tipotY(int x,int x1,int y1,int x2,int y2){
	return ((x-x1)*(y2-y1)/((x2-x1))+y1);
}
int tipotX(int y,int x1,int x2,int y1,int y2){
	return ((y-y1)*(x2-x1)/((y2-y1))+x1);
}

