#ifndef __bgfg_cb_h__
#define __bgfg_cb_h__

//http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.148.9778&rep=rep1&type=pdf
#include <C:\opencv\build\include\opencv2\core\core.hpp>
#include <C:\opencv\build\include\opencv2\imgproc\imgproc.hpp>
#include <C:\opencv\build\include\opencv2\highgui\highgui.hpp>
#include "stdafx.h";
#include <iostream>
#include <time.h>
using namespace cv;
using namespace std;


struct codeword {
    float min;
    float max; 
    float f;
    float l;
    int first;
    int last;
    bool isStale;
};
extern int alpha ;
extern float beta ;
extern int Tdel ,Tadd , Th;


void initializeCodebook(int w,int h);
void update_cb(Mat& frame);
void fg_cb(Mat& frame,Mat& fg);
#endif