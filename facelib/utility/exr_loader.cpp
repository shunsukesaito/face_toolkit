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
#include "exr_loader.h"

void loadEXRToCV(std::string filename, cv::Mat& mat)
{
    TinyExrImage tmp;
    const char* err;
    int ret = LoadEXR(&tmp.buf, &tmp.width, &tmp.height, filename.c_str(), &err);
    if (ret != 0)
    {
        std::cout << "Error: exr file isn't loaded correctly... " << filename << " " << err << std::endl;
        throw std::runtime_error("Error: exr file isn't loaded correctly...");
    }
    mat = cv::Mat(tmp.width,tmp.height,CV_32FC4,(float*)tmp.buf).clone();
    cv::flip(mat, mat, 0);
    cv::cvtColor(mat,mat,CV_RGBA2BGRA);
}

void saveEXRFromCV(std::string filename, const cv::Mat& mat)
{
    cv::Mat tmp = mat.clone();
    cv::flip(tmp,tmp,-1);
    if(mat.channels() == 4)
        cv::cvtColor(tmp,tmp,CV_BGRA2RGBA);
    else if(mat.channels() == 3)
        cv::cvtColor(tmp,tmp,CV_BGR2RGB);
    int ret = SaveEXR((float*)tmp.ptr(), tmp.cols, tmp.rows, mat.channels(), 0, filename.c_str());
    if (ret != 0)
    {
        std::cout << "Error: exr file isn't saved correctly... " << filename << std::endl;
        throw std::runtime_error("Error: exr file isn't saved correctly...");
    }
}
