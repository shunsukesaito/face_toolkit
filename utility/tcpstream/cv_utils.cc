#include "cv_utils.h"

cv::Rect scale_rect(const cv::Rect& rect,float scale)
{
    int cx = rect.x + rect.width/2;
    int cy = rect.y + rect.height/2;
    int length = std::max(rect.width,rect.height);
    
    return cv::Rect(cx - 0.5*scale*length,cy - 0.5*scale*length,scale*length,scale*length);
}

void crop_image(const cv::Mat& img, cv::Mat& out, const cv::Rect& rect)
{
    cv::Rect inter = (rect & cv::Rect(0, 0, img.cols, img.rows));
    if( inter != rect)
	{
        out = cv::Mat::zeros(rect.size(), img.type());
        img(inter).copyTo(out(inter - cv::Point(rect.tl())));
    }
    else
	{
        out = img(rect).clone();
    }
}

void insert_image(const cv::Mat& src, cv::Mat& tar, const cv::Rect& rect)
{
    cv::Rect inter = (rect & cv::Rect(0, 0, tar.cols, tar.rows));
    if( inter != rect)
	{
        src(inter - cv::Point(rect.tl())).copyTo(tar(inter));
    }
    else
	{
        src.copyTo(tar(inter));
    }
}
    