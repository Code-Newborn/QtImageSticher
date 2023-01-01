#ifndef _SURFMETHOD_H_
#define _SURFMETHOD_H_

#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"
#include <iostream>

void CalcCorners(const cv::Mat &H, const cv::Mat &src);

void OptimizeSeam(cv::Mat &img1, cv::Mat &trans, cv::Mat &dst);

void SurfStich(cv::Mat &img1, cv::Mat &img2, cv::Mat &dst);

#endif // _SURFMETHOD_H_