// TA_Start.cpp : Defines the entry point for the console application.
//

// Background average sample code done with averages and done with codebooks
// (adapted from the OpenCV book sample)
//
// NOTE: To get the keyboard to work, you *have* to have one of the video windows be active
//       and NOT the consule window.
//
// Gary Bradski Oct 3, 2008.
//
/* *************** License:**************************
   Oct. 3, 2008
   Right to use this code in any way you want without warrenty, support or any guarentee of it working.

   BOOK: It would be nice if you cited it:
   Learning OpenCV: Computer Vision with the OpenCV Library
     by Gary Bradski and Adrian Kaehler
     Published by O'Reilly Media, October 3, 2008

   AVAILABLE AT:
     http://www.amazon.com/Learning-OpenCV-Computer-Vision-Library/dp/0596516134
     Or: http://oreilly.com/catalog/9780596516130/
     ISBN-10: 0596516134 or: ISBN-13: 978-0596516130
************************************************** */
//LISENSI DARI CODEBOOK.....
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
#include <fstream>

#include <string.h>
#include <ctype.h>
#include <string>
#include <time.h>

#include "bgfg_cb.h";
#include "cvaux.h";
#include "cxmisc.h";
#include "processing.h";


using namespace cv;
using namespace std;

//mouse callback event
void mouseEvent(int evt, int x, int y, int flags, void* param);
void mouseEvent1(int event, int x, int y, int flags, void* param);
void mouseEvent2(int event, int x, int y, int flags, void* param);

//INISIALISASI N camera
const int cam = 3;
bool stilllearn = false;

//Mat Draw
Mat drawVG[cam];
Mat duplicate[cam];
vector<Point2f> grs[cam][cam];

//VARIABLES for CODEBOOK METHOD:
CvBGCodeBookModel* model[cam];
const int NCHANNELS = 3;
bool ch[NCHANNELS]={true,true,true}; // This sets what channels should be adjusted for background bounds

//USAGE:  ch9_background startFrameCollection# endFrameCollection# [movie filename, else from camera]
//If from AVI, then optionally add HighAvg, LowAvg, HighCB_Y LowCB_Y HighCB_U LowCB_U HighCB_V LowCB_V

//global variable yang lain
CvSize frame_size;

//POINT BUAT GARIS FOV
Line Ls[cam][cam];
FrameElement fe[cam][cam];
int iterategrs[cam];

//FOV Related Bool
bool learnFOV = false;
choosenCam pilih;
int activewindow = 0;

//File
int startFrame;
int allFrame;
String pathcsv = "E:/SMT 8 OTW/TA1/BEWARE TA CODE HERE/Edited LinerFresh/TA_Seri3/FileObs/DeltaKalman.csv";

String pathcsvUjiKalman = "E:/SMT 8 OTW/TA1/BEWARE TA CODE HERE/Edited LinerFresh/TA_Seri3/FileObs/KalmanR.csv";
String pathcsvUjiKoresponden = "E:/SMT 8 OTW/TA1/BEWARE TA CODE HERE/Edited LinerFresh/TA_Seri3/FileObs/Koresponden.csv";
String pathcsvUjiSingleCam = "E:/SMT 8 OTW/TA1/BEWARE TA CODE HERE/Edited LinerFresh/TA_Seri3/FileObs/SingleCam.csv";
String pathcsvUjiMultiCam = "E:/SMT 8 OTW/TA1/BEWARE TA CODE HERE/Edited LinerFresh/TA_Seri3/FileObs/MultiCam.csv";
String pathcsvUjiSistem = "E:/SMT 8 OTW/TA1/BEWARE TA CODE HERE/Edited LinerFresh/TA_Seri3/FileObs/Sistem.csv";

String pth = "E:/SMT 8 OTW/TA1/BEWARE TA CODE HERE/Edited LinerFresh/TA_Seri3/Observed/";


//lama waktu proses
ujikalman u_kalman[cam]; //Uji waktu kalman track kalman filter 
bool trackKalman = false;

ujikoresponden u_koresponden; //Uji waktu korespondensi + deteksi single camera layer
bool matchingobj = false;

//uji ketepatan korespondensi

ujisinglecam u_singlecam[cam];//Uji waktu layer single camera
bool captureFile = false;

ujimulticam u_multicam; //Uji waktu sistem multi-camera
bool trackMulticam = false;

ujitotal u_total; //Uji waktu total sistem
bool totalTrack = false;

bool capls = false;

int main(int argc, char** argv)
{	
	//inisiasi 1---------------------
	u_koresponden.korespondCount = 0;
	u_koresponden.max = 80;
	u_multicam.MulticamCount = 0;
	u_multicam.max = 300;
	u_total.totalCount = 0;
	u_total.max = 300;
	for (int i =0; i<cam ; i++){
		u_kalman[i].cap = 0;
		u_kalman[i].max = 150;
		u_singlecam[i].captureCount = 0;
		u_singlecam[i].max = 300;
	}
	//--------------------------------------------------------------

	ofstream myfile,ujikal,ujikor,ujisingcam,ujimulcam,ujisis;
	myfile.open(pathcsv);
	ujikal.open(pathcsvUjiKalman);
	ujikor.open(pathcsvUjiKoresponden);
	ujisingcam.open(pathcsvUjiSingleCam);
	ujimulcam.open(pathcsvUjiMultiCam);
	ujisis.open(pathcsvUjiSistem);

	//--------------------------------------------------------------
	pilih.i = 0;
	pilih.j = 0;
	//Line Inisiator 

	for ( int i = 0 ; i < cam ; i++ ){
		iterategrs[i]=0;
		for ( int j = 0 ; j < cam ; j++ ){
			fe[i][j].stat[0] = true;
			fe[i][j].stat[1] = true;
			fe[i][j].lock = false;
			Ls[i][j].mark=0;
			Ls[i][j].c11 = Point(0,0);
			Ls[i][j].c12 = Point(0,0);
			Ls[i][j].lock11 = false;
			Ls[i][j].lock12 = false;
		}
	}
	//Label need
	cv::Point labelpoint[cam];

	//int idx[cam]; //index largest_contour
	vector<int> idx[cam],idxdiff[cam];
	ObjectSubstraction Obj[cam],Objtemp[cam],Objobs[cam]; // Objek dipilih terdeteksi
	vector<ObjectSubstraction> ObjAll[cam]; // Kumpulan Objek terdeteksi

	//cv::Point pusat[cam];
	RNG rng(12345);

	//global inisiator tracker buat cek tracker lewat FOV
	vector<vector<Point>> contours_poly[cam];
	vector<Rect> boundRect[cam];
	vector<Point2f>center[cam];
	vector<float>radius[cam];

	//Inisiasi Kalman Filter
	KObject KF[cam];
	
	for(int i = 0; i<cam;i++){
		KF[i].KF = cv::KalmanFilter(4, 2, 0);
		KF[i].KF.measurementMatrix;
		KF[i].KF.controlMatrix;
		KF[i].KF.transitionMatrix;
		//state[i](4, 1); // (x, y, Vx, Vy)
		Mat processNoise(4, 1, CV_32F);
		KF[i].measurement = Mat_<float>(2,1);
		KF[i].measurement.setTo(Scalar(0));
		KF[i].KF.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,0,0,   0,1,0,0,  0,0,1,0,  0,0,0,1);
		setIdentity(KF[i].KF.measurementMatrix);
		//setIdentity(KF[i].KF.processNoiseCov, Scalar::all(1));
		setIdentity(KF[i].KF.processNoiseCov, Scalar::all(1e-4));//default
		//setIdentity(KF[i].KF.processNoiseCov, Scalar::all(1e-2));
		//setIdentity(KF[i].KF.processNoiseCov, Scalar::all(0));
		//setIdentity(KF[i].KF.measurementNoiseCov, Scalar::all(1));
		//setIdentity(KF[i].KF.measurementNoiseCov, Scalar::all(1e-2));
		setIdentity(KF[i].KF.measurementNoiseCov, Scalar::all(1e-1));//default
		//setIdentity(KF[i].KF.measurementNoiseCov, Scalar::all(0));
		

		setIdentity(KF[i].KF.errorCovPost, Scalar::all(.1));

		KF[i].trackStat=false;
		KF[i].switchon = false;
		KF[i].lock = false;
		KF[i].Label = "null";
		KF[i].framelewat = 0;
	}
	
	//Vektor
	vector<KeyPoint> keyPoints[cam];
    vector<vector <Point>> contours[cam];
	vector<Vec4i> hierarchy[cam];
	vector<Point2f> VekGr[cam];

	//Set Window....
	cv::namedWindow("RawImage");
	//cv::namedWindow("VektorGaris[0]");
	cv::namedWindow( "ForegroundCodeBook");
	//cv::namedWindow( "CodeBook_ConnectComp");
	cv::setMouseCallback("RawImage",mouseEvent,0);	

	cv::namedWindow("RawImage1");
	//cv::namedWindow("VektorGaris1[1]");
	cv::namedWindow( "ForegroundCodeBook1");
	//cv::namedWindow( "CodeBook_ConnectComp1");
	cv::setMouseCallback("RawImage1",mouseEvent1,0);

	cv::namedWindow("RawImage2");
	//cv::namedWindow("VektorGaris2[2]");
	cv::namedWindow( "ForegroundCodeBook2");
	//cv::namedWindow( "CodeBook_ConnectComp2");
	cv::setMouseCallback("RawImage2",mouseEvent2,0);

	//Set Variabel CV
	cv::Mat rawim[cam];
	cv::Mat ImaskC[cam];
	cv::Mat ImaskCB[cam];
	cv::Mat kontur[cam];

	cv::Mat kernel5x5 = getStructuringElement(MORPH_RECT, Size(6, 6));

	//---------------------------- cam init
    IplImage* rawImage[cam], *yuvImage[cam]; //yuvImage is for codebook method
    IplImage *ImaskCodeBook[cam],*ImaskCodeBookCC[cam];
	CvCapture* capture[cam];
	int nframes[cam];

	cv::BackgroundSubtractorMOG2 bg[cam];
	for (int i=0;i<cam;i++){
		rawImage[i]=0;
		yuvImage[i]=0;
		ImaskCodeBook[i]=0;
		ImaskCodeBookCC[i]=0;
		capture[i]=0;
		nframes[i]=0;

		model[i] = cvCreateBGCodeBookModel();
		//Set warna thresholds ke nilai default
		model[i]->modMin[0] = 3;
		model[i]->modMin[1] = model[i]->modMin[2] = 3;
		model[i]->modMax[0] = 10;
		model[i]->modMax[1] = model[i]->modMax[2] = 10;
		model[i]->cbBounds[0] = model[i]->cbBounds[1] = model[i]->cbBounds[2] = 10;
		bg[i]=  BackgroundSubtractorMOG2(200,30,false);
	}

    int c, n;
	bool learnFOV = false;
    bool pause = false;
    bool singlestep = false;
	
	//Ngambil gambar dari webCam....
	//capture[0] = cvCreateFileCapture("http://192.168.1.115:80/img/video.asf");
	//capture[0] = cvCreateFileCapture("http://192.168.1.115:80/img/mjpeg.cgi");
	/*
	for (int i = 0; i < cam ; i++){
		capture[i] = cvCaptureFromCAM(i+1);
		cvSetCaptureProperty(capture[i],CV_CAP_PROP_FRAME_HEIGHT,288);
		cvSetCaptureProperty(capture[i],CV_CAP_PROP_FRAME_WIDTH,352);
	}
	*/
	int skenario;
	
	/*
	skenario = 1;
	int nframesToLearnBG = 400;
	capture[0] = cvCreateFileCapture("E:/SMT 8 OTW/TA1/Buku TA/DataSet/Indoor/camera1edited.avi");
	capture[1] = cvCreateFileCapture("E:/SMT 8 OTW/TA1/Buku TA/DataSet/Indoor/camera2edited.avi");
	capture[2] = cvCreateFileCapture("E:/SMT 8 OTW/TA1/Buku TA/DataSet/Indoor/camera3edited.avi");
	rawImage[0] = skipNFrames(capture[0], 600);
	rawImage[1] = skipNFrames(capture[1], 572);
	rawImage[2] = skipNFrames(capture[2], 564);
	*/

	
	skenario = 2;
	int nframesToLearnBG = 350;
	capture[0] = cvCreateFileCapture("E:/SMT 8 OTW/TA1/Buku TA/DataSet/Outdoor/Cam0.avi");
	capture[1] = cvCreateFileCapture("E:/SMT 8 OTW/TA1/Buku TA/DataSet/Outdoor/Cam1.avi");
	capture[2] = cvCreateFileCapture("E:/SMT 8 OTW/TA1/Buku TA/DataSet/Outdoor/Cam2.avi");
	rawImage[0] = skipNFrames(capture[0], 100);
	rawImage[1] = skipNFrames(capture[1], 100);
	rawImage[2] = skipNFrames(capture[2], 66);
	

	for (int i = 0; i < cam ; i++){
		cvSetCaptureProperty(capture[i],CV_CAP_PROP_FRAME_HEIGHT,240);
		cvSetCaptureProperty(capture[i],CV_CAP_PROP_FRAME_WIDTH,320);
	}
	
	//MOG Drawer
	Mat mogfore[cam];
	cv::Mat mogframe[cam];

	frame_size.height = (int) cvGetCaptureProperty( capture[0], CV_CAP_PROP_FRAME_HEIGHT );
	frame_size.width = (int) cvGetCaptureProperty( capture[0], CV_CAP_PROP_FRAME_WIDTH );
	for (int i = 0; i < cam; i++){
		drawVG[i] = Mat(frame_size.height,frame_size.width, CV_64F, cvScalar(0.));
		mogfore[i] = Mat(frame_size.height,frame_size.width, CV_64F, cvScalar(0.));
	}
	
    //PROSES LOOPING BUAT AMBIL GAMBAR KAMERA
	allFrame = 0;
    for(;;)
    {
		if(!pause){
			allFrame++;
		}
		u_total.total_start = clock();
		for (int i = 0; i < cam;i++){
		if(!pause){
			u_singlecam[i].begin_time = clock();
			if( !pause )
			{
				//rawImage[i] = cvQueryFrame(capture[i]);
				rawImage[i] = skipNFrames(capture[i], 3);
				mogframe[i] = rawImage[i];
				++nframes[i];
				if(!rawImage[i]) 
					break;
			}

			if( singlestep )
				pause = true;
        
			//First time:
			if( nframes[i] == 1 && rawImage[i])
			{
				// CODEBOOK METHOD ALLOCATION cam 0
				yuvImage[i] = cvCloneImage(rawImage[i]);
				ImaskCodeBook[i] = cvCreateImage( cvGetSize(rawImage[i]), IPL_DEPTH_8U, 1 );
				ImaskCodeBookCC[i] = cvCreateImage( cvGetSize(rawImage[i]), IPL_DEPTH_8U, 1 );
				cvSet(ImaskCodeBook[i],cvScalar(255));
			}

			if( rawImage[i]) // If we've got an rawImage and are good to go:                
			{
				cvCvtColor( rawImage[i], yuvImage[i], CV_BGR2YCrCb );//YUV For codebook method
				if( !pause && nframes[i]-1 < nframesToLearnBG  ){ //This is where we build our background model
					cvBGCodeBookUpdate( model[i], yuvImage[i] );
					stilllearn = true;
				}else{
					stilllearn = false;
				}

				if( nframes[i]-1 == nframesToLearnBG  ){
					cvBGCodeBookClearStale( model[i], model[i]->t/2 );
					
				}

				if( nframes[i]-1 >= nframesToLearnBG  && (!learnFOV || skenario!=2)) //Find the foreground if any
				{
					LearnFindBGR(model[i],yuvImage[i],ImaskCodeBook[i],ImaskCodeBookCC[i]);
				}

				if( nframes[i]-1 == nframesToLearnBG )
				{
					printf("cam %d ready \n",i);
				}
				//trans C++ ----------------------------------------------------------------------------
				
				rawim[i] = rawImage[i];
				if (learnFOV && (skenario == 2)){
					Mat back;
					Mat thresholded2;
					cv::medianBlur(mogframe[i],back,3);
					//cv::blur(frame,back,cv::Size(10,10));
					bg[i].operator ()(mogframe[i],mogfore[i]);
					cv::threshold(mogfore[i],thresholded2,65.0f,255,CV_THRESH_BINARY);
					cv::cvtColor(thresholded2,mogfore[i],CV_GRAY2RGB);
					cv::erode(mogfore[i],mogfore[i],cv::Mat());
					removePepperNoise(mogfore[i]);
					cv::dilate(mogfore[i],mogfore[i],cv::Mat());
					cv::dilate(mogfore[i],mogfore[i],cv::Mat());
					cv::dilate(mogfore[i],mogfore[i],cv::Mat());
					cv::morphologyEx(mogfore[i],mogfore[i],cv::MORPH_CLOSE,kernel5x5);
					contours[i] = findKonturMOG(mogfore[i]);
				}else{
					ImaskC[i] = ImaskCodeBook[i];
					contours[i] = cariKontur(ImaskC[i]);
				}
				duplicate[i] = rawim[i];
				ImaskCB[i] = ImaskCodeBookCC[i];

				//BIKIN TRACKER cam --------------------------------------------------------------------
				contours_poly[i]=vector<vector<Point>>(contours[i].size());
				boundRect[i]=vector<Rect>(contours[i].size());
				center[i]=vector<Point2f>(contours[i].size());
				radius[i]=vector<float>(contours[i].size());

				kumpulanKontur(contours[i],contours_poly[i],boundRect[i],center[i],radius[i]);
				find_human(contours[i],idx[i],idxdiff[i]);
				//Menampilkan object yang terdeteksi
				if(!stilllearn){
					Objtemp[i] = Obj[i];
					buatBSTracker(duplicate[i], Obj[i], KF[i], contours[i], contours_poly[i], idx[i],idxdiff[i], boundRect[i], ObjAll[i]);
				}
				//Pakai Kalman Filter
				Point reseter(Obj[i].pusat.x,Obj[i].pusat.y);
				
				//
				KF[i].switchon = false;
				int rad = 1.5*abs(distance_to_point(Obj[i].labelpoint,Obj[i].pusat));
				if (rad < abs(distance_to_point(Obj[i].pusat,Objtemp[i].pusat))){
					//KF[i].switchon = true;
					KF[i].Label = "null";
					Point reseter(Obj[i].pusat.x,Obj[i].pusat.y);
					resetKalman(KF[i].KF,KF[i].measurement,reseter);
					KF[i].lock = false;
					KF[i].framelewat = 0;
				}
				//
				u_kalman[i].kalman_start=clock();
				if (!stilllearn){
					kalmanTracker(KF[i],duplicate[i], Obj[i].pusat);
					jarakDeteksiTrack(duplicate[i],ObjAll[i], KF[i]);
					Objobs[i] = Obj[i];
					Obj[i].pusat = KF[i].pusat;
				}
				u_kalman[i].kalman_end=clock();

				KF[i].trackStat = getIsTrack();
				if (KF[i].trackStat==false || KF[i].switchon ==true){
					resetKalman(KF[i].KF,KF[i].measurement,reseter);
					KF[i].Label = "null";
					KF[i].LabelPoint = Point(0,0);
					KF[i].lock = false;
					KF[i].framelewat = 0;
				}
				
				// Draw FOV + matching..
				if (cam > 1){
					for (int j = 0; j<cam ; j++ ){
						if (i!=j){
							gambarPoint(duplicate[i],Ls[i][j].test);
							gambarFOVLine(duplicate[i],Obj[i],labelpoint[i],Ls[i][j],i,j);
						}
					}
				}else{
					gambarPoint(duplicate[i],Ls[i][2].test);
					gambarPoint(duplicate[i],Ls[i][1].test);
					gambarFOVLine(duplicate[i],Obj[i],labelpoint[i],Ls[i][2],i,2);
					gambarFOVLine(duplicate[i],Obj[i],labelpoint[i],Ls[i][1],i,1);
				}
			}
			u_singlecam[i].end_time = clock();
		}
		}

		//MULTICAM LAYER MULAI DARI SINI KEBAWAH......
		u_multicam.multi_start = clock();

		//START USE FOV---------------------------------------------------------------------------------------------------
		vector<ObjectSubstraction> greaterLs[cam][cam];
		
		//matching object untuk beberapa kamera yang berbeda....
		//perulangan buat dicamera i,j 
		
		u_koresponden.stimulus = false;
		if (!learnFOV){
			u_koresponden.korespond_start= clock();
			for (int i=0 ; i<cam ; i++){
				for (int j=0 ; j<cam ; j++){
					if (i<j){
						for (int sz = 0; sz < ObjAll[i].size() ;  sz++){
							if (cekKanan(Ls[i][j],ObjAll[i].at(sz))==true){
								greaterLs[i][j].push_back(ObjAll[i].at(sz));
							}
						}
					}
					if (i>j){
						for (int sz = 0; sz < ObjAll[i].size() ;  sz++){
							if (cekKanan(Ls[i][j],ObjAll[i].at(sz))==false){
								greaterLs[i][j].push_back(ObjAll[i].at(sz));
							}
						}
					}
				}
			}
			//INISIASI KANDIDAT IRISAN---------------------------------------------
			
			//LOCK MODE-----------------
			int max = KF[0].framelewat;
			int indexmax = 0;
			for (int i=0 ; i<cam ; i++){
				KF[i].lock = false;
			}
			for (int i=0 ; i<cam ; i++){
				if (KF[i].framelewat>=max){
					max = KF[i].framelewat;
					indexmax = i;
				}
			}
			KF[indexmax].lock = true;
			for (int i=0 ; i<cam ; i++){
				if (KF[indexmax].framelewat==KF[i].framelewat){
					KF[i].lock = true;
				}
			}
			//--------------------------
			
			//PASANGIN X sama Y-----------------------------------------------------
			if (!learnFOV)
			for (int x=0 ; x<cam ; x++){
				for (int y=0 ; y<cam ; y++){
					if ((greaterLs[x][y].size())==(greaterLs[y][x].size()) && greaterLs[x][y].size()==1){
					//if ((x<y) && (KF[x].trackStat && KF[y].trackStat) && ((greaterLs[x][y].size()==1 && ObjAll[y].size()>0 && cekGrad(Ls[x][y],Obj[x].bottom)>0) || (greaterLs[y][x].size()==1 && ObjAll[x].size()>0 && cekGrad(Ls[y][x],Obj[y].bottom)>0))){
						if ((x<y) && KF[x].lock && KF[y].lock==false && cekKanan(Ls[x][y],Obj[x])==true){
							SelectionSort(greaterLs[x][y],Ls[x][y]);
							int posisi = binarySearch(greaterLs[x][y],Obj[x]);
							
							pilihPointCam(Ls[x][y],greaterLs[y][x],frame_size,posisi);
							Obj[y] = greaterLs[y][x].at(posisi);
							
							KF[y].framelewat = KF[x].framelewat;
							resetKalman(KF[y].KF,KF[y].measurement,Obj[y].pusat);
							std::stringstream out;
							std::stringstream out1;
							out << x;
							out1 << y;
							KF[y].Label = "Obj"+out.str()+""+out1.str();
							KF[x].Label = "Obj"+out.str()+""+out1.str();
						}
						if((x>y) && KF[x].lock && KF[y].lock==false && cekKanan(Ls[x][y],Obj[x])==false){
							SelectionSort(greaterLs[x][y],Ls[x][y]);
							int posisi = binarySearch(greaterLs[x][y],Obj[x]);
							
							pilihPointCam(Ls[x][y],greaterLs[y][x],frame_size,posisi);
							Obj[y] = greaterLs[y][x].at(posisi);
							
							KF[y].framelewat = KF[x].framelewat;
							resetKalman(KF[y].KF,KF[y].measurement,Obj[y].pusat);
							std::stringstream out;
							std::stringstream out1;
							out << x;
							out1 << y;
							KF[y].Label = "Obj"+out1.str()+""+out.str();
							KF[x].Label = "Obj"+out1.str()+""+out.str();
						}
						if ((x!=y) && KF[x].lock == true && KF[y].lock == true && ((cekKanan(Ls[x][y],Obj[x])==true && cekKanan(Ls[y][x],Obj[y])==false) || (cekKanan(Ls[y][x],Obj[y])==true && cekKanan(Ls[x][y],Obj[x])==false))){
							std::stringstream out;
							std::stringstream out1;
							out << x;
							out1 << y;
							KF[y].Label = "Obj"+out1.str()+""+out.str();
							KF[x].Label = "Obj"+out1.str()+""+out.str();
						}
						if ((x<y) && ((KF[y].lock == true) && (KF[x].lock == true)) && ((cekKanan(Ls[x][y],Obj[x])==true && cekKanan(Ls[y][x],Obj[y])==true) || (cekKanan(Ls[y][x],Obj[y])==false && cekKanan(Ls[x][y],Obj[x])==false))){
							KF[x].Label = "null";
							KF[y].Label = "null";
							KF[x].framelewat = 1;
							KF[y].framelewat = 0;
						}
					}

					if ((x<y) && (greaterLs[x][y].size())==(greaterLs[y][x].size()) && greaterLs[x][y].size()>1){
						if (((KF[y].lock == true) && (KF[x].lock == true)) && ((cekKanan(Ls[x][y],Obj[x])==true && cekKanan(Ls[y][x],Obj[y])==true) || (cekKanan(Ls[y][x],Obj[y])==false && cekKanan(Ls[x][y],Obj[x])==false))){
							KF[x].Label = "null";
							KF[y].Label = "null";
							KF[x].framelewat = 1;
							KF[y].framelewat = 0;
						}
						if (((KF[x].lock == true) && (KF[y].lock == false)) && ((cekKanan(Ls[x][y],Obj[x])==true && cekKanan(Ls[y][x],Obj[y])==false) || (cekKanan(Ls[y][x],Obj[y])==true && cekKanan(Ls[x][y],Obj[x])==false))){
							SelectionSort(greaterLs[x][y],Ls[x][y]);
							int posisiA = binarySearch(greaterLs[x][y],Obj[x]);
							SelectionSort(greaterLs[y][x],Ls[y][x]);
							int posisiB = binarySearch(greaterLs[y][x],Obj[y]);
							Obj[y] = greaterLs[y][x].at((greaterLs[y][x].size()-1)-posisiA);
							resetKalman(KF[y].KF,KF[y].measurement,Obj[y].pusat);
							std::stringstream out;
							std::stringstream out1;
							out << x;
							out1 << y;
							KF[y].Label = "Obj"+out1.str()+""+out.str();
							KF[x].Label = "Obj"+out1.str()+""+out.str();
							KF[y].framelewat = KF[x].framelewat;
						}
						if (((KF[y].lock == true) && (KF[x].lock == false)) && ((cekKanan(Ls[x][y],Obj[x])==true && cekKanan(Ls[y][x],Obj[y])==false) || (cekKanan(Ls[y][x],Obj[y])==true && cekKanan(Ls[x][y],Obj[x])==false))){
							SelectionSort(greaterLs[y][x],Ls[y][x]);
							int posisiA = binarySearch(greaterLs[y][x],Obj[y]);
							SelectionSort(greaterLs[x][y],Ls[x][y]);
							int posisiB = binarySearch(greaterLs[x][y],Obj[x]);
							Obj[x] = greaterLs[x][y].at((greaterLs[x][y].size()-1)-posisiA);
							resetKalman(KF[x].KF,KF[x].measurement,Obj[x].pusat);
							std::stringstream out;
							std::stringstream out1;
							out << x;
							out1 << y;
							KF[y].Label = "Obj"+out1.str()+""+out.str();
							KF[x].Label = "Obj"+out1.str()+""+out.str();
							KF[x].framelewat = KF[y].framelewat;
						}
						if ((KF[x].lock == true) && (KF[y].lock == true) && ((cekKanan(Ls[x][y],Obj[x])==true && cekKanan(Ls[y][x],Obj[y])==false) || (cekKanan(Ls[y][x],Obj[y])==true && cekKanan(Ls[x][y],Obj[x])==false))){
							SelectionSort(greaterLs[x][y],Ls[x][y]);
							int posisiA = binarySearch(greaterLs[x][y],Obj[x]);
							SelectionSort(greaterLs[y][x],Ls[y][x]);
							int posisiB = binarySearch(greaterLs[y][x],Obj[y]);
							if (posisiB!=(greaterLs[y][x].size()-1)-posisiA){
								KF[x].Label = "null";
								KF[y].Label = "null";
								KF[x].framelewat = 0;
								KF[y].framelewat = 1;
							}else{
								std::stringstream out;
								std::stringstream out1;
								out << x;
								out1 << y;
								KF[y].Label = "Obj"+out1.str()+""+out.str();
								KF[x].Label = "Obj"+out1.str()+""+out.str();
								KF[x].framelewat = KF[y].framelewat;
							}
						}
					}
				}
			}
			
			//clear semua calon kandidat beririsan-----------------------------------
			for (int i=0 ; i<cam ; i++){
				for (int j = 0 ; j < cam ; j++){ 
					greaterLs[i][j].clear();
				}
			}
			u_koresponden.korespond_end = clock();
		}
		//END USE FOV--------------------------------------------------------------------------------------------------
		u_multicam.multi_end = clock();
		u_total.total_end = clock();


		//START LEARN FOV----------------------------------------------------------------------------------------------
		//Insert Vektor FOV
		if (learnFOV || !learnFOV){
			int i = pilih.i;
			int j = pilih.j;
			if (i!=j){
				fe[i][j].stat[0]=fe[i][j].stat[1];
				fe[i][j].stat[1]=KF[j].trackStat;
			}
		}

		//Gambar Vektor FOV
		bool AutomaticDetermineFOV = true;
		if (learnFOV && AutomaticDetermineFOV){
			int i = pilih.i;
			int j = pilih.j;
			if ((i!=j) && fe[i][j].lock==false){
				if (((fe[i][j].stat[0]==false) && (fe[i][j].stat[1]==true)) || ((fe[i][j].stat[0]==true) && (fe[i][j].stat[1]==false))){
					if (Ls[i][j].lock11==false){
						Ls[i][j].c11 = Obj[i].bottom;
						printf("Ls[%d][%d] Point c11 (%d,%d) \n",i,j,Ls[i][j].c11.x,Ls[i][j].c11.y);
						Ls[i][j].mark = Ls[i][j].mark+1;
					}else if (Ls[i][j].lock12==false){
						Ls[i][j].c12 = Obj[i].bottom;
						printf("Ls[%d][%d] Point c12 (%d,%d) \n",i,j,Ls[i][j].c11.x,Ls[i][j].c11.y);
						Ls[i][j].mark = Ls[i][j].mark+1;
					}
					cv::circle(drawVG[i],Ls[i][j].c11,1,Scalar(255,255,255),2,8,0);
					cv::circle(drawVG[i],Ls[i][j].c12,1,Scalar(255,255,255),2,8,0);

					//Print capture
					String path = "E:/SMT 8 OTW/TA1/BEWARE TA CODE HERE/Edited LinerFresh/TA_Seri3/Observed/";
					String filename = "FOVLearn_Line[";
					String type = ".jpg";
					std::stringstream numcami,numcamj;
					numcami << i;
					filename = filename+numcami.str()+"][";
					numcamj << j;
					filename = filename+numcamj.str()+"]";
					std::stringstream fre;
					fre << allFrame;
					filename = filename+"_frame_"+fre.str();
					DrawHasil(drawVG[i],path,filename,type);
				}
			}
		}
		//END LEARN FOV-----------------------------------------------------------------------------------

		
		
		//PART PENGUJIAN---------------------------------------------------------------------------------

		//UJI KALMAN Tracker.............
		if(trackKalman){
			for (int i = 0; i<cam ; i++){
				String in = "";
				
				std::stringstream numcami;
				numcami << i;

				in = in+numcami.str()+",";

				std::stringstream fre;
				fre << allFrame;

				in = in+fre.str()+",";

				std::stringstream timecap;
				timecap << diffMilisecond(u_kalman[i].kalman_start,u_kalman[i].kalman_end);

				in = in+timecap.str()+",";

				std::stringstream errkalman;
				errkalman << hitungErrorKalman(Objobs[i],KF[i]);
				
				in = in+errkalman.str()+",";

				std::stringstream xreal,yreal;
				std::stringstream xpredict,ypredict;
				std::stringstream xdiff,ydiff;

				if (KF[i].trackStat){
					xpredict<<KF[i].pusat.x;
					ypredict<<KF[i].pusat.y;
					xreal<<Objobs[i].pusat.x;
					yreal<<Objobs[i].pusat.y;
					xdiff<<abs(KF[i].pusat.x-Objobs[i].pusat.x);
					ydiff<<abs(KF[i].pusat.y-Objobs[i].pusat.y);
				}else{
					xpredict<<0;
					ypredict<<0;
					xreal<<0;
					yreal<<0;
					xdiff<<0;
					ydiff<<0;
				}

				std::stringstream luas;
				luas << (KF[i].height*KF[i].width);
				in = in+xreal.str()+","+yreal.str()+","+xpredict.str()+","+ypredict.str()+","+xdiff.str()+","+ydiff.str()+","+luas.str();
				
				ujikal<<in<<endl;
				u_kalman[i].cap++;
			}
		}

		//UJI KORESPONDENSI................
		if(!learnFOV && matchingobj && u_koresponden.stimulus)
		{
			if (u_koresponden.korespondCount<=u_koresponden.max){
				for (int i=0;i<cam;i++){
					for (int j=0;j<cam;j++){
						if (i!=j){
							String path = "E:/SMT 8 OTW/TA1/BEWARE TA CODE HERE/Edited LinerFresh/TA_Seri3/Observed/koresponden/";
							String filename = "Korespond_cam";
							String type = ".jpg";
							String in = "";

							std::stringstream numcami;
							numcami << i;
							filename = filename+numcami.str();

							in = in+numcami.str()+",";
							
							std::stringstream numcamj;
							numcamj << j;
							filename = filename+numcamj.str();

							in = in+numcamj.str()+",";
							
							std::stringstream fre;
							fre << allFrame;
							filename = filename+"_frame_"+fre.str();

							in = in+fre.str()+",";

							std::stringstream candidate;
							candidate << ObjAll[j].size();

							in = in+candidate.str()+",";
							
							std::stringstream timecap;
							timecap << diffMilisecond(u_koresponden.korespond_start,u_koresponden.korespond_start);
							filename = filename+"_time_"+timecap.str();
							String filenames = filename+" second";
							in = in+timecap.str();
							
							DrawHasil(duplicate[i],path,filename,type);
							DrawHasil(duplicate[j],path,filenames,type);
							ujikor<<in<<endl;
						}
					}
				}
				u_koresponden.korespondCount++;
			}else{
				printf("Uji KORESPONDENSI selesai \n");
			}
		}

		//UJI LAYER Single CAMERA
		if (captureFile){
			for (int i = 0 ; i < cam ; i++){
				if (u_singlecam[i].captureCount<=u_singlecam[i].max){
					String path = "E:/SMT 8 OTW/TA1/BEWARE TA CODE HERE/Edited LinerFresh/TA_Seri3/Observed/singlecam/";
					String filename = "SingleCam";
					String type = ".jpg";
					String in = "";
					
					std::stringstream numcam;
					numcam << i;
					filename = filename+numcam.str();

					in =  numcam.str()+",";

					std::stringstream fre;
					fre << allFrame;
					filename = filename+"_frame_"+fre.str();
				
					in = in+fre.str()+",";

					std::stringstream objcon;
					objcon << ObjAll[i].size();

					in = in+objcon.str()+",";

					std::stringstream timecap;
					timecap << diffMilisecond(u_singlecam[i].begin_time,u_singlecam[i].end_time);
					filename = filename+"_time_"+timecap.str();

					in = in+timecap.str();
				
					DrawHasil(duplicate[i],path,filename,type);
					ujisingcam<<in<<endl;
					u_singlecam[i].captureCount++;
				}else{
					printf("Uji SINGLECAM[%d] selesai \n",i);
				}
			}
		}

		//UJI Layer Multicamera
		if (trackMulticam){
			if (u_multicam.MulticamCount<=u_multicam.max){
				int sumObj = 0;
				for (int i = 0; i < cam ; i++){
					sumObj = sumObj+ObjAll[i].size();
				}
				
				String in = "";

				std::stringstream fre;
				fre << allFrame;
				
				in = in+fre.str()+",";

				std::stringstream oby;
				oby << sumObj;

				in = in+oby.str()+",";

				std::stringstream timecap;
				timecap << diffMilisecond(u_multicam.multi_start,u_multicam.multi_end);

				in = in+timecap.str();

				ujimulcam<<in<<endl;
				u_multicam.MulticamCount++;
			}else{
				printf("Uji MultiCAM selesai \n");
			}
		}

		//Uji TOTAL Sistem
		if (totalTrack){
			if (u_total.totalCount<=u_total.max){
				int sumObj = 0;
				for (int i = 0; i < cam ; i++){
					sumObj = sumObj+ObjAll[i].size();
				}
				
				String in = "";

				std::stringstream fre;
				fre << allFrame;
				
				in = in+fre.str()+",";

				std::stringstream oby;
				oby << sumObj;

				in = in+fre.str()+",";

				std::stringstream timecap;
				timecap << diffMilisecond(u_total.total_start,u_total.total_end);

				in = in+timecap.str();

				ujisis<<in<<endl;
				u_multicam.MulticamCount++;
				u_total.totalCount++;
			}else{
				printf("Uji SISTEM selesai \n");
			}
		}

		//END PENGUJIAN-------------------------------------------------------------------------------------

		std::stringstream fps;
		fps << 1/((diffMilisecond(u_total.total_start,u_total.total_end))/1000);

		std::stringstream ft;
		ft << allFrame;

		//Penamaan KAMERA sudut kiri bawah & properti Kamera................................................
		if(!pause)
		for (int k = 0 ; k < cam ; k++){	
			KF[k].LabelPoint = Point(KF[k].pusat.x, (KF[k].pusat.y+(KF[k].height/2)));
			cv::putText(duplicate[k],KF[k].Label, KF[k].LabelPoint,1,1,CV_RGB(255, 0, 0),1,8,false);
			KF[k].framelewat++;
			std::stringstream outp,outk;
			String boli;
			outp << k;
			outk << KF[k].framelewat++;//abs(distance_to_point(Objtemp[k].pusat,Obj[k].pusat));
			String dist = outk.str();
			if (KF[k].lock){boli = "true";}else{boli="false";}
			String camname = "cam "+outp.str()+" "+boli+" "+ft.str();//+" proc "+fps.str()+" fps, frame "+ft.str();
			cv::putText(duplicate[k], dist,KF[k].pusat,1,1,CV_RGB(255,0,255),2,8,false); 
			cv::putText(duplicate[k], camname,Point(1,frame_size.height),1,1,CV_RGB(255,0,0),1,8,false); 
			ObjAll[k].clear();
		}
	
		//....................................................................................................

		//Capture Proses pembentukan FOVLINE------------------------------------------------------------------
		if (capls) {
			String filename = "Ls[";
			String filename2 = "";
			std::stringstream numcami,numcamj;
			numcami << pilih.i;
			numcamj << pilih.j;
			filename = filename+numcami.str()+"][";
			filename = filename+numcamj.str()+"]";
			filename2 = filename+"second";
			DrawHasil(duplicate[pilih.i],pth,filename,".jpg");	
			DrawHasil(duplicate[pilih.j],pth,filename2,".jpg");	
			capls = false;
		}
		//----------------------------------------------------------------------------------------------------

		cv::imshow("RawImage",duplicate[0]);
		cv::imshow("RawImage1",duplicate[1]);
		cv::imshow("RawImage2",duplicate[2]);

		//cv::imshow("VektorGaris[0]",drawVG[0]);
		//cv::imshow("VektorGaris1[1]",drawVG[1]);
		//cv::imshow("VektorGaris2[2]",drawVG[2]);

		//cv::imshow("CodeBook_ConnectComp",ImaskCB[0]);
		//cv::imshow("CodeBook_ConnectComp1",ImaskCB[1]);
		//cv::imshow("CodeBook_ConnectComp2",ImaskCB[2]);

		if (!learnFOV){
			cv::imshow("ForegroundCodeBook",ImaskC[0]);
			cv::imshow("ForegroundCodeBook1",ImaskC[1]);
			cv::imshow("ForegroundCodeBook2",ImaskC[2]);
		}
		else{
			cv::imshow("ForegroundCodeBook",mogfore[0]);
			cv::imshow("ForegroundCodeBook1",mogfore[1]);
			cv::imshow("ForegroundCodeBook2",mogfore[2]);
		}
        // User input:
        c = cvWaitKey(30)&0xFF;
        c = tolower(c);
        // End processing on ESC, q or Q
        if(c == 27 || c == 'q')
            break;
        //Else check for user input
        switch( c )
        {
        case 'h':
            help();
            break;
        case 'p':
            pause = !pause;
            break;
        case 's':
            singlestep = !singlestep;
            pause = false;
            break;
        case 'r':
            pause = false;
            singlestep = false;
            break;
		case 'x':
			unLockFOV(Ls[pilih.i][pilih.j], pilih.i, pilih.j);
			break;
		case 'z':
			LockFOV(Ls[pilih.i][pilih.j], pilih.i, pilih.j);
			break;
		case 'c':
			if (learnFOV==true){
				printf("Learn FOV Mode : false \n");
				learnFOV = false;
			}else{
				printf("Learn FOV Mode : true \n");
				learnFOV = true;
			}
			break;
		case '7':
			activateCapture(captureFile);
			for (int i = 0; i<cam ; i++){
				u_singlecam[i].captureCount = 0;
			}
			break;
		case '8':
			activateTrackMulticam(trackMulticam);
			u_multicam.MulticamCount=0;
			break;
		case '9':
			activateUjiSistem(totalTrack);
			u_total.totalCount=0;
			break;
		case '5':
			captureKalman(trackKalman);
			for (int a=0; a<cam ; a++){
				u_kalman[a].cap=0;
			}
			break;
		case '6':
			activateMatching(matchingobj);
			u_koresponden.korespondCount=0;
			break;
        case ' ':
			printf("RESET CODEBOOK MODEL \n");
			for (int i = 0;i<cam;i++){
				cvBGCodeBookClearStale( model[i], 0 );
				nframes[i] = 0;
			}
            break;
            //CODEBOOK PARAMS
        case 'y': case '0':
        case 'u': case '1':
        case 'v': case '2':
        case 'a': case '3':
        case 'b': 
            ch[0] = c == 'y' || c == '0' || c == 'a' || c == '3';
            ch[1] = c == 'u' || c == '1' || c == 'a' || c == '3' || c == 'b';
            ch[2] = c == 'v' || c == '2' || c == 'a' || c == '3' || c == 'b';
            printf("Aktivasi Channel Codebook : %d, %d, %d\n", ch[0], ch[1], ch[2] );
            break;
        case 'i': //modify max classification bounds (max bound goes higher)
        case 'o': //modify max classification bounds (max bound goes lower)
        case 'k': //modify min classification bounds (min bound goes lower)
        case 'l': //modify min classification bounds (min bound goes higher)
            for (int i = 0 ; i < cam; i++)
			{
				if (activewindow==i){
					uchar* ptr = c == 'i' || c == 'o' ? model[i]->modMax : model[i]->modMin;
					for(n=0; n<NCHANNELS; n++)
					{
						if( ch[n] )
						{
							int v = ptr[n] + (c == 'i' || c == 'l' ? 1 : -1);
							ptr[n] = CV_CAST_8U(v);
						}
						printf("%d,", ptr[n]);
					}
					printf(" Cam %d CodeBook %s Side\n",i, c == 'i' || c == 'o' ? "High" : "Low" );
				}
			}
            break;
        }
    }	

    for (int i = 0; i<cam;i++){
		cvReleaseCapture( &capture[i] );
	}
	myfile.close();
	ujikal.close();
	ujikor.close();
	ujisingcam.close();
	ujimulcam.close();
	ujisis.close();
    return 0;
}


/*
void mouseEvent(int event, int x, int y, int flags, void* param){
	if (iterategrs[0]%2 == 0){
		switch( event ){
			case CV_EVENT_LBUTTONDOWN:  //start drawing
				Ls[0][1].test = Point(x,y);
				printf("Gambar GARIS FOV Line cam [0] ke [1] \n");
				break;
			case CV_EVENT_LBUTTONUP:  //draw what we created with Lbuttondown
				Ls[0][1].test = Point(x,y);
				pilih.i = 0;
				pilih.j = 1;
				setPointLine(Ls[0][1],Ls[0][1].test,0,frame_size.width,iterategrs[0]);
				capls = true;
				printf("Lewat Ls[0][1] \n");
				break;
			case CV_EVENT_LBUTTONDBLCLK:
				resetPointLine(Ls[0][1],iterategrs[0]);
				resetPointLine(Ls[0][2],iterategrs[0]);
				break;
			case CV_EVENT_MOUSEMOVE:
				activewindow = 0;
				break;
		}
	}else if(iterategrs[0]%2 == 1){
		switch( event ){
			case CV_EVENT_LBUTTONDOWN:  //start drawing
				Ls[0][2].test = Point(x,y);
				printf("Gambar GARIS FOV Line cam [0] ke [2] \n");
				break;
			case CV_EVENT_LBUTTONUP:  //draw what we created with Lbuttondown
				Ls[0][2].test = Point(x,y);
				printf("Lewat Ls[0][2] \n");
				pilih.i = 0;
				pilih.j = 2;
				setPointLine(Ls[0][2],Ls[0][2].test,0,frame_size.width,iterategrs[0]);
				capls = true;
				break;
			case CV_EVENT_LBUTTONDBLCLK:
				resetPointLine(Ls[0][2],iterategrs[0]);
				resetPointLine(Ls[0][1],iterategrs[0]);
				break;
			case CV_EVENT_MOUSEMOVE:
				activewindow = 0;
				break;
		}
	}
}

void mouseEvent1(int event, int x, int y, int flags, void* param){
	if (iterategrs[1]%2 == 0){
		switch( event ){
			case CV_EVENT_LBUTTONDOWN:  //start drawing
				Ls[1][2].test = Point(x,y);
				printf("Gambar GARIS FOV Line cam [1] ke [2] \n");
				break;
			case CV_EVENT_LBUTTONUP:  //draw what we created with Lbuttondown
				Ls[1][2].test = Point(x,y);
				setPointLine(Ls[1][2],Ls[1][2].test,0,frame_size.width,iterategrs[1]);
				pilih.i = 1;
				pilih.j = 2;
				capls = true;
				break;
			case CV_EVENT_LBUTTONDBLCLK:
				resetPointLine(Ls[1][2],iterategrs[1]);
				resetPointLine(Ls[1][0],iterategrs[1]);
				break;
			case CV_EVENT_MOUSEMOVE:
				activewindow = 1;
				break;
		}
	}else if(iterategrs[1]%2 == 1){
		switch( event ){
			case CV_EVENT_LBUTTONDOWN:  //start drawing
				Ls[1][0].test = Point(x,y);
				printf("Gambar GARIS FOV Line cam [1] ke [0] \n");
				break;
			case CV_EVENT_LBUTTONUP:  //draw what we created with Lbuttondown
				Ls[1][0].test = Point(x,y);
				setPointLine(Ls[1][0],Ls[1][0].test,0,frame_size.width,iterategrs[1]);
				pilih.i = 1;
				pilih.j = 0;
				capls = true;
				break;
			case CV_EVENT_LBUTTONDBLCLK:
				resetPointLine(Ls[1][0],iterategrs[1]);
				resetPointLine(Ls[1][2],iterategrs[1]);
				break;
			case CV_EVENT_MOUSEMOVE:
				activewindow = 1;
				break;
		}
	}
}


void mouseEvent2(int event, int x, int y, int flags, void* param){
	if (iterategrs[2]%2 == 0){
		switch( event ){
			case CV_EVENT_LBUTTONDOWN:  //start drawing
				Ls[2][0].test = Point(x,y);
				printf("Gambar GARIS FOV Line cam [2] ke [0] \n");
				break;
			case CV_EVENT_LBUTTONUP:  //draw what we created with Lbuttondown
				Ls[2][0].test = Point(x,y);
				setPointLine(Ls[2][0],Ls[2][0].test,0,frame_size.width,iterategrs[2]);
				pilih.i = 2;
				pilih.j = 0;
				capls = true;
				break;
			case CV_EVENT_LBUTTONDBLCLK:
				resetPointLine(Ls[2][0],iterategrs[2]);
				resetPointLine(Ls[2][1],iterategrs[2]);
				break;
			case CV_EVENT_MOUSEMOVE:
				activewindow = 2;
				break;
		}
	}else if(iterategrs[2]%2 == 1){
		switch( event ){
			case CV_EVENT_LBUTTONDOWN:  //start drawing
				Ls[2][1].test = Point(x,y);
				printf("Gambar GARIS FOV Line cam [2] ke [1] \n");
				break;
			case CV_EVENT_LBUTTONUP:  //draw what we created with Lbuttondown
				Ls[2][1].test = Point(x,y);
				setPointLine(Ls[2][1],Ls[2][1].test,0,frame_size.width,iterategrs[2]);
				pilih.i = 2;
				pilih.j = 1;
				capls = true;
				break;
			case CV_EVENT_LBUTTONDBLCLK:
				resetPointLine(Ls[2][1],iterategrs[2]);
				resetPointLine(Ls[2][0],iterategrs[2]);
				break;
			case CV_EVENT_MOUSEMOVE:
				activewindow = 2;
				break;
		}
	}
}
*/


void mouseEvent(int event, int x, int y, int flags, void* param){
	if (iterategrs[0]%2 == 0){
		switch( event ){
			case CV_EVENT_LBUTTONDOWN:  
				printf("Gambar GARIS FOV Line cam [0] ke [1] \n");
				break;
			case CV_EVENT_LBUTTONUP:  
				pilih.i = 0;
				pilih.j = 1;
				break;
			case CV_EVENT_RBUTTONUP:  
				setAutoPointLine(Ls[0][1],0,frame_size.width,frame_size,iterategrs[0]);
				capls = true;
				printf("Lewat Ls[0][1] \n");
				break;
			case CV_EVENT_LBUTTONDBLCLK:
				resetPointLine(Ls[0][1],iterategrs[0]);
				resetPointLine(Ls[0][2],iterategrs[0]);
				pilih.i = 0;
				pilih.j = 0;
				break;
			case CV_EVENT_MOUSEMOVE:
				activewindow = 0;
				pilih.i = 0;
				pilih.j = 1;
				break;
		}
	}else if(iterategrs[0]%2 == 1){
		switch( event ){
			case CV_EVENT_LBUTTONDOWN:
				//printf("Gambar GARIS FOV Line cam [0] ke [2] \n");
				break;
			case CV_EVENT_LBUTTONUP: 
				//pilih.i = 0;
				//pilih.j = 2;
				break;
			case CV_EVENT_RBUTTONUP:  
				//setAutoPointLine(Ls[0][2],0,frame_size.width,frame_size,iterategrs[0]);
				//capls = true;
				//printf("Lewat Ls[0][2] \n");
				break;
			case CV_EVENT_LBUTTONDBLCLK:
				//resetPointLine(Ls[0][2],iterategrs[0]);
				//resetPointLine(Ls[0][1],iterategrs[0]);
				//pilih.i = 0;
				//pilih.j = 0;
				break;
			case CV_EVENT_MOUSEMOVE:
				activewindow = 0;
				break;
		}
	}
}

void mouseEvent1(int event, int x, int y, int flags, void* param){
	if (iterategrs[1]%2 == 0){
		switch( event ){
			case CV_EVENT_LBUTTONDOWN:  
				printf("Gambar GARIS FOV Line cam [1] ke [2] \n");
				break;
			case CV_EVENT_LBUTTONUP:  
				pilih.i = 1;
				pilih.j = 2;
				break;
			case CV_EVENT_RBUTTONUP:
				setAutoPointLine(Ls[1][2],0,frame_size.width,frame_size,iterategrs[1]);
				capls = true;
				printf("Lewat Ls[1][2] \n");
				break;
			case CV_EVENT_LBUTTONDBLCLK:
				resetPointLine(Ls[1][2],iterategrs[1]);
				resetPointLine(Ls[1][0],iterategrs[1]);
				pilih.i = 0;
				pilih.j = 0;
				break;
			case CV_EVENT_MOUSEMOVE:
				activewindow = 1;
				pilih.i = 1;
				pilih.j = 2;
				break;
			
		}
	}else if(iterategrs[1]%2 == 1){
		switch( event ){
			case CV_EVENT_LBUTTONDOWN:  
				printf("Gambar GARIS FOV Line cam [1] ke [0] \n");
				break;
			case CV_EVENT_LBUTTONUP:
				pilih.i = 1;
				pilih.j = 0;
				break;
			case CV_EVENT_RBUTTONUP:  
				setAutoPointLine(Ls[1][0],0,frame_size.width,frame_size,iterategrs[1]);
				capls = true;
				printf("Lewat Ls[1][0] \n");
				break;
			case CV_EVENT_LBUTTONDBLCLK:
				resetPointLine(Ls[1][0],iterategrs[1]);
				resetPointLine(Ls[1][2],iterategrs[1]);
				pilih.i = 0;
				pilih.j = 0;
				break;
			case CV_EVENT_MOUSEMOVE:
				activewindow = 1;
				pilih.i = 1;
				pilih.j = 0;
				break;
		}
	}
}

void mouseEvent2(int event, int x, int y, int flags, void* param){
	if (iterategrs[2]%2 == 0){
		switch( event ){
			case CV_EVENT_LBUTTONDOWN:  
				printf("Gambar GARIS FOV Line cam [2] ke [1] \n");
				break;
			case CV_EVENT_LBUTTONUP:  
				pilih.i = 2;
				pilih.j = 1;
				break;
			case CV_EVENT_RBUTTONUP:
				setAutoPointLine(Ls[2][1],0,frame_size.width,frame_size,iterategrs[2]);
				capls = true;
				printf("Lewat Ls[1][0] \n");
				break;
			case CV_EVENT_LBUTTONDBLCLK:
				resetPointLine(Ls[2][1],iterategrs[2]);
				resetPointLine(Ls[2][0],iterategrs[2]);
				pilih.i = 0;
				pilih.j = 0;
				break;
			case CV_EVENT_MOUSEMOVE:
				activewindow = 2;
				pilih.i = 2;
				pilih.j = 1;
				break;
		}
	}else if(iterategrs[2]%2 == 1){
		switch( event ){
			case CV_EVENT_LBUTTONDOWN:
				//printf("Gambar GARIS FOV Line cam [2] ke [0] \n");
				break;
			case CV_EVENT_LBUTTONUP:
				//pilih.i = 2;
				//pilih.j = 0;
				break;
			case CV_EVENT_RBUTTONUP:  
				//setAutoPointLine(Ls[2][0],0,frame_size.width,frame_size,iterategrs[2]);
				//capls = true;
				//printf("Lewat Ls[1][0] \n");
				break;
			case CV_EVENT_LBUTTONDBLCLK:
				//resetPointLine(Ls[2][0],iterategrs[2]);
				//resetPointLine(Ls[2][1],iterategrs[2]);
				//pilih.i = 0;
				//pilih.j = 0;
				break;
			case CV_EVENT_MOUSEMOVE:
				activewindow = 2;
				break;
		}
	}
}
