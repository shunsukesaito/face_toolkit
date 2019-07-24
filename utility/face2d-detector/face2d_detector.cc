/*
 MIT License
 
 Copyright (c) 2018 Shunsuke Saito
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */
#include "face2d_detector.h"

#include <gflags/gflags.h>

DEFINE_string(cpm_ip, "csloadbalancer-dev-746798469.us-east-1.elb.amazonaws.com", "IP for CPM net");
DEFINE_string(sp_name, DLIB_68_FACEALIGNMENT_MODEL, "facial landmark model name");
cv::Rect ScaleRect(const cv::Rect& rect,float scale)
{
    int cx = rect.x + rect.width/2;
    int cy = rect.y + rect.height/2;
    int length = std::max(rect.width,rect.height);
    
    return cv::Rect(cx - scale*length,cy - scale*length,2.0*scale*length,2.0*scale*length);
}

void DrawLandmarks(cv::Mat& img, const std::vector<Eigen::Vector3f>& p2d)
{
    for(const Eigen::Vector3f& p : p2d)
    {
        cv::circle(img, cv::Point(p[0],p[1]), 2, cv::Scalar(0,255,0), -1);
    }
}

cv::Rect GetBBoxFromLandmarks(std::vector<Eigen::Vector3f>& shape)
{
    float max_x = 0, min_x = 1e4, max_y = 0, min_y = 1e4;
    for(int i = 0; i < shape.size(); ++i)
    {
        if(shape[i](0)>max_x) max_x = shape[i](0);
        if(shape[i](0)<min_x) min_x = shape[i](0);
        if(shape[i](1)>max_y) max_y = shape[i](1);
        if(shape[i](1)<min_y) min_y = shape[i](1);
    }

    float cx = 0.5*(max_x + min_x);
    float cy = 0.5*(max_y + min_y);
    float length = std::max(max_x-min_x,max_y-min_y);
    
    return cv::Rect(cx-0.5*length,cy-0.5*length,length,length);
}

Face2DDetector::Face2DDetector(std::string data_dir){
    if(!FLAGS_cpm_ip.empty())
        cpm_tcp_ = std::make_shared<LandmarkCpmTCPStream>(FLAGS_cpm_ip);

    dlib::deserialize(data_dir + "facedetector/" + FLAGS_sp_name) >> sp_;
    gab_detector_.LoadModel(data_dir + "facedetector/" + NPD_MODEL);
    gab_detector_.DetectSize = 200;
    
    detector_ = dlib::get_frontal_face_detector();
}

bool Face2DDetector::GetFaceRect(const cv::Mat &img,
                                  cv::Rect& rect,
                                  bool enable_dlib)
{
    std::vector<dlib::rectangle> dets;
    dlib::array2d<dlib::rgb_pixel> img_dlib, img_sml_dlib;
    cv::Mat img_sml;
    float scale = 150.0/(float)std::max(img.rows,img.cols);
    float invscale = 1.0/scale;
    cv::resize(img, img_sml, cv::Size(),scale,scale);
    assign_image(img_dlib, dlib::cv_image<dlib::bgr_pixel>(img));
    assign_image(img_sml_dlib, dlib::cv_image<dlib::bgr_pixel>(img_sml));
    
    if(enable_dlib){
        dets = detector_(img_sml_dlib,-0.2);
        
        if(dets.size()!=0){
            dets[0].set_bottom(invscale*(float)dets[0].bottom());
            dets[0].set_top(invscale*(float)dets[0].top());
            dets[0].set_left(invscale*(float)dets[0].left());
            dets[0].set_right(invscale*(float)dets[0].right());
        }
    }

    if(dets.size()==0)
    {
        cv::Mat frame_gray;
        cv::cvtColor( img_sml, frame_gray, cv::COLOR_BGR2GRAY );

        std::vector<float> scores;
        std::vector<int> index;
        std::vector<cv::Rect> new_rects;
        index = gab_detector_.DetectFace(frame_gray,new_rects,scores);
        
        std::vector<cv::Rect> rects;
        for (int i = 0; i < index.size(); i++) {
            if(scores[index[i]]>9)
                rects.push_back(new_rects[index[i]]);
        }
        
        dets.resize(rects.size());
        if(rects.size()==0) return false;
        if(rects.size()>0)
        {
            int face_id = 0;
            rects[face_id].x = invscale*(float)rects[face_id].x;
            rects[face_id].y = invscale*(float)rects[face_id].y;
            rects[face_id].width = invscale*(float)rects[face_id].width;
            rects[face_id].height = invscale*(float)rects[face_id].height;
            
            dets[0].set_bottom(rects[face_id].y+rects[face_id].height);
            dets[0].set_top(rects[face_id].y);
            dets[0].set_left(rects[face_id].x);
            dets[0].set_right(rects[face_id].x + rects[face_id].width);
        }
    }
    
    if(dets.size() == 0){
        return false;
    }
    
    rect.x = dets[0].left();
    rect.y = dets[0].top();
    rect.width = dets[0].width();
    rect.height = dets[0].height();
    
    return true;
}

void Face2DDetector::GetFaceRects(const cv::Mat &img,
                                  std::vector<cv::Rect>& rects,
                                  bool enable_dlib,
                                  int max_size,
                                  int min_size)
{
    if(enable_dlib){
        std::vector<dlib::rectangle> dets;
        
        if(img.channels() == 1){
            dlib::array2d<uchar> img_gray;
            img_gray.set_size(img.rows, img.cols);
            for(int i = 0; i < img.rows; ++i)
            {
                for(int j = 0; j < img.cols; ++j)
                {
                    img_gray[i][j] = img.at<uchar>(i,j);
                }
            }
            dets = detector_(img_gray,-0.2);
        }
        else if(img.channels() == 3)
        {
            dlib::array2d<dlib::rgb_pixel> img_dlib;
            img_dlib.set_size(img.rows, img.cols);
            for(int i = 0; i < img.rows; ++i)
            {
                const cv::Vec3b* ptr = img.ptr<cv::Vec3b>(i);
                for(int j = 0; j < img.cols; ++j)
                {
                    img_dlib[i][j].blue = ptr[j](0);
                    img_dlib[i][j].green = ptr[j](1);
                    img_dlib[i][j].red = ptr[j](2);
                }
            }
            dets = detector_(img_dlib,-0.2);
        }
        
        rects.clear();
        for(int i = 0; i < dets.size(); ++i)
        {
            cv::Rect rect;
            rect.x = (int)dets[i].left();
            rect.y = (int)dets[i].top();
            rect.width = (int)dets[i].width();
            rect.height = (int)dets[i].height();
            rects.push_back(rect);
        }
    }
    else{
        cv::Mat frame_gray;
        
        cv::cvtColor( img, frame_gray, cv::COLOR_BGR2GRAY );
        
        std::vector<float> scores;
        std::vector<int> index;
        std::vector<cv::Rect> new_rects;
        index = gab_detector_.DetectFace(frame_gray,new_rects,scores);
        
        rects.clear();
        
        cv::Rect rect;
        float max_score = -1.0f;
        for (int i = 0; i < index.size(); i++) {
            if(scores[index[i]]>10 && scores[index[i]] > max_score){
                rect = new_rects[index[i]];
                max_score = scores[index[i]];
            }
        }
        
        if(max_score != -1.0f)
            rects.push_back(rect);
    }
    
}

bool Face2DDetector::GetFaceLandmarks(const cv::Mat &img,
                                      std::vector<Eigen::Vector3f>& p2d,
                                      cv::Rect& rect,
                                      bool enable_dlib,
                                      bool enable_cpm)
{
    std::vector<dlib::rectangle> dets;
    dlib::array2d<dlib::rgb_pixel> img_dlib, img_sml_dlib;
    cv::Mat img_sml;
    float scale = 150.0/(float)std::max(img.rows,img.cols);
    float invscale = 1.0/scale;
    cv::resize(img, img_sml, cv::Size(),scale,scale);
    assign_image(img_dlib, dlib::cv_image<dlib::bgr_pixel>(img));
    assign_image(img_sml_dlib, dlib::cv_image<dlib::bgr_pixel>(img_sml));
    
    if(enable_dlib){
        dets = detector_(img_sml_dlib,-0.2);
        if(dets.size()!=0){
            dets[0].set_bottom(invscale*(float)dets[0].bottom());
            dets[0].set_top(invscale*(float)dets[0].top());
            dets[0].set_left(invscale*(float)dets[0].left());
            dets[0].set_right(invscale*(float)dets[0].right());
        }
    }

    if(dets.size()==0)
    {
        cv::Mat frame_gray;
        cv::cvtColor( img_sml, frame_gray, cv::COLOR_BGR2GRAY );

        std::vector<float> scores;
        std::vector<int> index;
        std::vector<cv::Rect> new_rects;
        index = gab_detector_.DetectFace(frame_gray,new_rects,scores);
        
        std::vector<cv::Rect> rects;
        for (int i = 0; i < index.size(); i++) {
            if(scores[index[i]]>9)
                rects.push_back(new_rects[index[i]]);
        }
        
        dets.resize(rects.size());
        if(rects.size()==0) return false;
        if(rects.size()>0)
        {
            int face_id = 0;
            rects[face_id].x = invscale*(float)rects[face_id].x;
            rects[face_id].y = invscale*(float)rects[face_id].y;
            rects[face_id].width = invscale*(float)rects[face_id].width;
            rects[face_id].height = invscale*(float)rects[face_id].height;
            
            dets[0].set_bottom(rects[face_id].y+rects[face_id].height);
            dets[0].set_top(rects[face_id].y);
            dets[0].set_left(rects[face_id].x);
            dets[0].set_right(rects[face_id].x + rects[face_id].width);
        }
    }
    
    if(dets.size() == 0){
        return false;
    }
    
    rect.x = dets[0].left();
    rect.y = dets[0].top();
    rect.width = dets[0].width();
    rect.height = dets[0].height();
    
    if(enable_cpm){
        if(rect.width != rect.height){
            int len = 1.4*std::max(rect.width,rect.height);
            int cx = rect.x + rect.width/2;
            int cy = rect.y + rect.height/2;
            rect.x = cx - len/2;
            rect.y = cy - len/2;
            rect.width = len;
            rect.height = len;
        }
        cpm_tcp_->sendImage(img, rect);
        p2d = cpm_tcp_->getLandmarks();
        return true;
    }
    
    dlib::full_object_detection shape = sp_(img_dlib, dets[0]);
    
    p2d.clear();
    for(int i = 0; i < shape.num_parts(); ++i)
    {
        p2d.push_back(Eigen::Vector3f(shape.part(i)(0),shape.part(i)(1),1.0));
    }
    
    return true;
}

bool Face2DDetector::GetFaceLandmarks(const cv::Mat &img,
                                      const cv::Rect &rect,
                                      std::vector<Eigen::Vector3f>& p2d,
                                      bool enable_cpm)
{
    if(enable_cpm && cpm_tcp_ != nullptr){
        cpm_tcp_->sendImage(img, rect);
        p2d = cpm_tcp_->getLandmarks();
        return true;
    }
    dlib::array2d<dlib::rgb_pixel> img_dlib;
    assign_image(img_dlib, dlib::cv_image<dlib::bgr_pixel>(img));
    
    dlib::rectangle det;
    det.set_bottom(rect.y+rect.height);
    det.set_top(rect.y);
    det.set_left(rect.x);
    det.set_right(rect.x + rect.width);
    
    dlib::full_object_detection shape = sp_(img_dlib, det);
    
    
    p2d.clear();
    for(int i = 0; i < shape.num_parts(); ++i)
    {
        p2d.push_back(Eigen::Vector3f(shape.part(i)(0),shape.part(i)(1),1.0));
    }
    
    return true;
}

void Face2DDetector::detect(cv::Mat& frame, cv::Rect& r, cv::Mat& output)
{
    cv::Point2f nose(r.x+r.width/2, r.y+2*r.height/3);
    r.x=nose.x-r.height*0.7;
    r.y=nose.y-r.height*0.7;
    r.width=r.height*1.4;
    r.height=r.height*1.4;
    cout<<r<<" "<<frame.rows<<" "<<frame.cols<<endl;
    int borders[4];
    for(int i=0; i<4; i++)
        borders[i]=0;
    if(r.y<0)
    {
        borders[0]=r.y*-1;
        r.y=0;
    }
    if(r.y+r.height>frame.rows)
    {
        borders[1]=r.y+r.height-frame.rows;
    }
    if(r.x<0)
    {
        borders[2]=r.x*-1;
        r.x=0;
    }
    if(r.x+r.width>frame.cols)
    {
        borders[3]=r.x+r.width-frame.cols;
    }
    cv::copyMakeBorder( frame, frame, borders[0], borders[1], borders[2], borders[3], cv::BORDER_CONSTANT, cv::Scalar(0,0,0) );
    output=frame(r);
    resize(output,output, cv::Size(256,256));
}

bool Face2DDetector::CropFaceImage(cv::Mat& out,
                                   cv::Rect& rect,
                                   cv::Rect& rectCPM,
                                   const cv::Mat &img,
                                   cv::Mat& imgCPM,
                                   float cpm_width,
                                   float cpm_crop_pos,
                                   float cpm_crop_width,
                                   float width)
{
    std::vector<cv::Rect> rects;
    GetFaceRects(img, rects, true);
    
    cv::Mat imgInput = out.clone();
    cv::Rect r = rects[0];
    
    if(rects.size() == 0){
        std::cout << "Warning: Face is not detected." << std::endl;
        return false;
    }
    else{
        rect = ScaleRect(rects[0],1.25f);
    }
    
    float scale = 1.0f;
    cv::Rect rect_old = rect;
    if(width > 0){
        scale = width/rect.width;
        rect.x *= scale;
        rect.y *= scale;
        rect.width = width;
        rect.height = width; // to make sure, it's always rectangle
    }
    cv::Mat img_resize;
    cv::resize(img, img_resize, cv::Size(),scale,scale);
    cv::Rect inter = (rect & cv::Rect(0,0,img_resize.cols,img_resize.rows));
    if( inter != rect){
        out = cv::Mat::zeros(rect.size(), img_resize.type());
        img_resize(inter).copyTo(out(inter - cv::Point(rect.tl())));
    }
    else{
        out = img_resize(rect).clone();
    }
    
    cv::Rect crop_rect(cpm_crop_pos, cpm_crop_pos, cpm_crop_width, cpm_crop_width);
    imgCPM = out(crop_rect);
    cv::resize(imgCPM,imgCPM, cv::Size(cpm_width,cpm_width));
    
    rect = rect_old;
    
    return true;
}

bool  Face2DDetector::CropFaceImage(cv::Mat& out,
                                    cv::Rect& rect,
                                    const cv::Mat &img,
                                    float width)
{
    std::vector<cv::Rect> rects;
    GetFaceRects(img, rects, false);
    if(rects.size() == 0){
        std::cout << "Warning: Face is not detected." << std::endl;
        return false;
    }
    else{
        rect = ScaleRect(rects[0],1.25f);
    }
    
    float scale = 1.0f;
    if(width > 0){
        scale = width/rect.width;
        rect.x *= scale;
        rect.y *= scale;
        rect.width = width;
        rect.height = width; // to make sure, it's always rectangle
    }
    cv::Mat img_resize;
    cv::resize(img, img_resize, cv::Size(),scale,scale);
    cv::Rect inter = (rect & cv::Rect(0,0,img_resize.cols,img_resize.rows));
    if( inter != rect){
        out = cv::Mat::zeros(rect.size(), img_resize.type());
        img_resize(inter).copyTo(out(inter - cv::Point(rect.tl())));
    }
    else{
        out = img_resize(rect).clone();
    }

    return true;
}


cv::Rect Face2DBoxTracker::doubleRectSize(const cv::Rect &inputRect, const cv::Rect &frameSize) const
{
    cv::Rect outputRect;
    // Double rect size
    outputRect.width = inputRect.width * 2;
    outputRect.height = inputRect.height * 2;
    
    // Center rect around original center
    outputRect.x = inputRect.x - inputRect.width / 2;
    outputRect.y = inputRect.y - inputRect.height / 2;
    
    return outputRect & frameSize;
}

cv::Point Face2DBoxTracker::centerOfRect(const cv::Rect &rect) const
{
    return cv::Point(rect.x + rect.width / 2, rect.y + rect.height / 2);
}

cv::Rect Face2DBoxTracker::biggestFace(std::vector<cv::Rect> &faces) const
{
    assert(!faces.empty());
    
    cv::Rect *biggest = &faces[0];
    for (auto &face : faces) {
        if (face.area() < biggest->area())
            biggest = &face;
    }
    return *biggest;
}

cv::Rect Face2DBoxTracker::getFaceROI()
{
    cv::Rect rect;
    rect.x = (int)(faceRoi_.x / scale_);
    rect.y = (int)(faceRoi_.y / scale_);
    rect.width = (int)(faceRoi_.width / scale_);
    rect.height = (int)(faceRoi_.height / scale_);
    
    return rect;
}

/*
 * Face template is small patch in the middle of detected face.
 */
cv::Mat Face2DBoxTracker::getFaceTemplate(const cv::Mat &frame, cv::Rect face)
{
    face.x += face.width / 4;
    face.y += face.height / 4;
    face.width /= 2;
    face.height /= 2;
    
    cv::Mat faceTemplate = frame(face).clone();
    return faceTemplate;
}

bool Face2DBoxTracker::detectFaceAllSizes(const cv::Mat &frame)
{
    std::vector<cv::Rect> rects;
    cv::Mat frame_gray;
    
    cv::cvtColor( frame, frame_gray, cv::COLOR_BGR2GRAY );
    cv::equalizeHist( frame_gray, frame_gray );
    
    cv::Mat small_gray_image(cvRound(frame.size().height), cvRound(frame.size().width), CV_8UC1);
    cv::Mat flipped_small_gray_image;
    
    cv::cvtColor(frame, frame_gray, cv::COLOR_RGB2GRAY);
    cv::resize(frame_gray, small_gray_image, small_gray_image.size(), 0, 0, cv::INTER_LINEAR);
    cv::equalizeHist(small_gray_image, small_gray_image);

    int cascade_flags = 0;
    cascade_flags |= cv::CASCADE_FIND_BIGGEST_OBJECT;
    cascade_flags |= cv::CASCADE_DO_ROUGH_SEARCH;
    cascade_flags |= cv::CASCADE_SCALE_IMAGE;
    
    std::vector<cv::Rect> faces;
    
    std::vector<float> scores;
    std::vector<int> index;
    std::vector<cv::Rect> new_rects;
    index = faceDetector_->gab_detector_.DetectFace(small_gray_image,new_rects,scores);
    
    cv::Rect rect;
    float max_score = -1.0f;
    for (int i = 0; i < index.size(); i++) {
        if(scores[index[i]]>10 && scores[index[i]] > max_score){
            rect = new_rects[index[i]];
            max_score = scores[index[i]];
        }
    }
    
    if(max_score != -1.0f)
        rects.push_back(rect);
    
    if(rects.size() == 0) return false;
    
    
    foundFace_ = true;
    
    // Locate biggest face
    trackedFace_ = rects[0];
    
    // Copy face template
    faceTemplate_ = getFaceTemplate(frame, trackedFace_);
    
    // Calculate roi
    faceRoi_ = doubleRectSize(trackedFace_, cv::Rect(0, 0, frame.cols, frame.rows));
    
    // Update face position
    facePosition_ = centerOfRect(trackedFace_);
    
    return true;
}

bool Face2DBoxTracker::detectFaceAroundRoi(const cv::Mat &frame)
{
    std::vector<cv::Rect> rects;
    
    cv::Mat frame_gray;
    
    cv::cvtColor( frame, frame_gray, cv::COLOR_BGR2GRAY );
    cv::equalizeHist( frame_gray, frame_gray );
    
    std::vector<float> scores;
    std::vector<int> index;
    std::vector<cv::Rect> new_rects;
    index = faceDetector_->gab_detector_.DetectFace(frame_gray(faceRoi_),new_rects,scores);
    
    cv::Rect rect;
    float max_score = -1.0f;
    for (int i = 0; i < index.size(); i++) {
        if(scores[index[i]]>10 && scores[index[i]] > max_score){
            rect = new_rects[index[i]];
            max_score = scores[index[i]];
        }
    }

    if(max_score != -1.0f)
        rects.push_back(rect);
    
    if (rects.empty())
    {
        // Activate template matching if not already started and start timer
        templateMatchingRunning_ = true;
        if (templateMatchingStartTime_ == 0)
            templateMatchingStartTime_ = cv::getTickCount();
        return false;
    }
    
    // Turn off template matching if running and reset timer
    templateMatchingRunning_ = false;
    templateMatchingCurrentTime_ = templateMatchingStartTime_ = 0;
    
    // Get detected face
    trackedFace_ = rects[0];
    
    // Add roi offset to face
    trackedFace_.x += faceRoi_.x;
    trackedFace_.y += faceRoi_.y;
    
    // Get face template
    faceTemplate_ = getFaceTemplate(frame, trackedFace_);
    
    // Calculate roi
    faceRoiOld_ = faceRoi_;
    faceRoi_ = doubleRectSize(trackedFace_, cv::Rect(0, 0, frame.cols, frame.rows));
    
    // Update face position
    facePosition_ = centerOfRect(trackedFace_);
    
    return true;
}

bool Face2DBoxTracker::detectFacesTemplateMatching(const cv::Mat &frame)
{
    templateMatchingCurrentTime_ = cv::getTickCount();
    double duration = (double)(templateMatchingCurrentTime_ - templateMatchingStartTime_) / cv::getTickFrequency();
    // If template matching lasts for more than 2 seconds face is possibly lost
    // so disable it and redetect using cascades
    if (duration > templateMatchingMaxDuration_) {
        foundFace_ = false;
        templateMatchingRunning_ = false;
        templateMatchingStartTime_ = templateMatchingCurrentTime_ = 0;
        
        return false;
    }
    
    // Template matching with last known face
    cv::matchTemplate(frame(faceRoi_), faceTemplate_, matchingResult_, CV_TM_SQDIFF_NORMED);
    cv::normalize(matchingResult_, matchingResultNorm_, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
    double min, max;
    cv::Point minLoc, maxLoc;
    cv::minMaxLoc(matchingResultNorm_, &min, &max, &minLoc, &maxLoc);
    
    if(matchingResult_.at<float>(minLoc) > 0.04) return false;
    
    // Add roi offset to face position
    minLoc.x += faceRoi_.x;
    minLoc.y += faceRoi_.y;
    
    // Get detected face
    trackedFace_ = cv::Rect(minLoc.x, minLoc.y, faceTemplate_.cols, faceTemplate_.rows);
    trackedFace_ = doubleRectSize(trackedFace_, cv::Rect(0, 0, frame.cols, frame.rows));
    // Get new face template
    faceTemplate_ = getFaceTemplate(frame, trackedFace_);
    
    // Calculate face roi
    faceRoiOld_ = faceRoi_;
    faceRoi_ = doubleRectSize(trackedFace_, cv::Rect(0, 0, frame.cols, frame.rows));
    
    // Update face position
    facePosition_ = centerOfRect(trackedFace_);
    
    return true;
}

bool Face2DBoxTracker::getFrameAndDetect(cv::Mat &frame,cv::Rect& rect)
{
    assert(faceDetector_ != NULL);
    
    // Downscale frame to m_resizedWidth width - keep aspect ratio
    scale_ = (double) std::min(resizedWidth_, frame.cols) / frame.cols;
    cv::Size resizedFrameSize = cv::Size((int)(scale_*frame.cols), (int)(scale_*frame.rows));
    
    cv::Mat resizedFrame;
    cv::resize(frame, resizedFrame, resizedFrameSize);
    
    if (!foundFace_){
        if(!detectFaceAllSizes(resizedFrame)) // Detect using cascades over whole image
            return false;
    }
    else {
        detectFaceAroundRoi(resizedFrame); // Detect using cascades only in ROI
        if (templateMatchingRunning_) {
            if(!detectFacesTemplateMatching(resizedFrame)) // Detect using template matching
                return false;
        }
    }
    
    faceROIImage_ = resizedFrame(trackedFace_).clone();
    trackedFaceOld_ = trackedFace_;
    
    rect.x = (int)(trackedFace_.x / scale_);
    rect.y = (int)(trackedFace_.y / scale_);
    rect.width = (int)(trackedFace_.width / scale_);
    rect.height = (int)(trackedFace_.height / scale_);
    
    return true;
}


bool Face2DBoxTracker::getFrameAndDetect(cv::Mat &frame,const std::vector<Eigen::Vector2f>& Pcur,cv::Point& dp,cv::Rect& rect)
{
    assert(faceDetector_ != NULL);
    
    // Downscale frame to m_resizedWidth width - keep aspect ratio
    scale_ = (double) std::min(resizedWidth_, frame.cols) / frame.cols;
    cv::Size resizedFrameSize = cv::Size((int)(scale_*frame.cols), (int)(scale_*frame.rows));
    
    cv::Mat resizedFrame;
    cv::resize(frame, resizedFrame, resizedFrameSize);
    
    if (!foundFace_){
        if(!detectFaceAllSizes(resizedFrame)) // Detect using cascades over whole image
            return false;
    }
    else {
        detectFaceAroundRoi(resizedFrame); // Detect using cascades only in ROI
        if (templateMatchingRunning_) {
            if(!detectFacesTemplateMatching(resizedFrame)){ // Detect using template matching
                dp.x = 0.0; dp.y = 0.0;
                faceTemplate_ = getFaceTemplate(resizedFrame, trackedFace_);
                faceROIImage_ = resizedFrame(faceRoi_).clone();
                return true;
            }
        }
        
        // obtain face patch
        cv::Rect rectface;
        cv::Point p_for;
        cv::Point p_tmp;
        float min_dist = 1.e10;
        const int max_iter = 3;
        for(int i = 0; i < max_iter; ++i){
            //cv::Rect recttmp = getFaceTemplateRect(Pcur);
            cv::Rect recttmp = getFaceTemplateRect(trackedFaceOld_);
            
            recttmp = cv::Rect(0,0,faceROIImage_.cols,faceROIImage_.rows) & recttmp;
            if(recttmp.width == 0 || recttmp.height == 0) return false;
            if(recttmp.width > trackedFace_.width || recttmp.height > trackedFace_.height) return false;
            float dist = detectFaceTemplateMatching(faceROIImage_(recttmp), resizedFrame(trackedFace_),p_tmp);
            
            if(dist < min_dist){
                min_dist = dist;
                p_for = p_tmp;
                rectface = recttmp;
            }
        }
        //        std::cout << min_dist << std::endl;
        
        if(min_dist < 0.05){
            dp.x = (int)((p_for.x-rectface.x+trackedFace_.x-trackedFaceOld_.x) / scale_);
            dp.y = (int)((p_for.y-rectface.y+trackedFace_.y-trackedFaceOld_.y) / scale_);
        }
        else{
            dp.x = 0.0;
            dp.y = 0.0;
        }
        
        // for debug
//        {
//            rectface.x = (int)((rectface.x + trackedFaceOld_.x)/ scale_);
//            rectface.y = (int)((rectface.y + trackedFaceOld_.y)/ scale_);
//            rectface.width = (int)(rectface.width / scale_);
//            rectface.height = (int)(rectface.height / scale_);
//
//            cv::rectangle(frame, rectface, cv::Scalar(255,0,0));
//
//            rectface.x = p_for.x / scale_ + trackedFace_.x / scale_;
//            rectface.y = p_for.y / scale_ + trackedFace_.y / scale_;
//
//            cv::rectangle(frame, rectface, cv::Scalar(0,0,255));
//        }
    }
    
    faceROIImage_ = resizedFrame(trackedFace_).clone();
    trackedFaceOld_ = trackedFace_;
    
    rect.x = (int)(trackedFace_.x / scale_);
    rect.y = (int)(trackedFace_.y / scale_);
    rect.width = (int)(trackedFace_.width / scale_);
    rect.height = (int)(trackedFace_.height / scale_);
    
    return true;
}

void Face2DBoxTracker::setInitialFace(const cv::Mat &frame,const cv::Rect& rect)
{
    scale_ = (double) std::min(resizedWidth_, frame.cols) / frame.cols;
    cv::Size resizedFrameSize = cv::Size((int)(scale_*frame.cols), (int)(scale_*frame.rows));
    
    cv::Mat resizedFrame;
    cv::resize(frame, resizedFrame, resizedFrameSize);
    
    foundFace_ = true;
    
    // Locate biggest face
    trackedFace_ = rect;
    
    trackedFace_.x = (int)(trackedFace_.x * scale_);
    trackedFace_.y = (int)(trackedFace_.y * scale_);
    trackedFace_.width = (int)(trackedFace_.width * scale_);
    trackedFace_.height = (int)(trackedFace_.height * scale_);
    
    // Copy face template
    faceTemplate_ = getFaceTemplate(resizedFrame, trackedFace_);
    
    // Calculate roi
    faceRoi_ = doubleRectSize(trackedFace_, cv::Rect(0, 0, resizedFrame.cols, resizedFrame.rows));
    
    // Update face position
    facePosition_ = centerOfRect(trackedFace_);
    
    faceROIImage_ = resizedFrame(trackedFace_).clone();
    trackedFaceOld_ = trackedFace_;
}

cv::Rect Face2DBoxTracker::getFaceTemplateRect(const std::vector<Eigen::Vector2f>& p2d)
{
    float width = scale_*(p2d[10][0]-p2d[4][0]);
    float height = scale_*(p2d[4][1]-p2d[1][1]);// std::min(landmarks[11][1],landmarks[5][1]));
    
    float x = scale_*p2d[4][0]-faceRoi_.x;
    float y = scale_*p2d[1][1]-faceRoi_.y;
    
    return cv::Rect(x,y,width,height);
}

cv::Rect Face2DBoxTracker::getFaceTemplateRect(const cv::Rect& rect)
{
    float ratioX = cv::theRNG().uniform(0.3, 0.8);
    float ratioY = cv::theRNG().uniform(0.3, 0.8);
    float width = ratioX*rect.width;
    float height = ratioY*rect.height;
    
    float x = cv::theRNG().uniform(0.0, 1.0-ratioX)*rect.width;
    float y = cv::theRNG().uniform(0.0, 1.0-ratioY)*rect.height;
    
    return cv::Rect(x,y,width,height);
}

float Face2DBoxTracker::detectFaceTemplateMatching(const cv::Mat &faceTemplate,
                                                   const cv::Mat &frame,
                                                   cv::Point& p)
{
    cv::matchTemplate(frame,faceTemplate, matchingResultFace_, CV_TM_SQDIFF_NORMED);
    cv::normalize(matchingResultFace_, matchingResultFaceNorm_, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
    double min, max;
    cv::Point maxLoc;
    cv::minMaxLoc(matchingResultFaceNorm_, &min, &max, &p, &maxLoc);
    
    return matchingResultFace_.at<float>(p);
}
