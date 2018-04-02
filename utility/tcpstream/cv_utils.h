#pragma once

#include <opencv2/opencv.hpp>

cv::Rect scale_rect(const cv::Rect& rect,float scale);
void crop_image(const cv::Mat& img, cv::Mat& out, const cv::Rect& rect);
void insert_image(const cv::Mat& src, cv::Mat& tar, const cv::Rect& rect);

    