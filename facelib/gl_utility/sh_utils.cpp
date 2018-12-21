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

#include "sh_utils.h"

#include <utility/sh_basic.h>

static glm::vec3 PixelToDirection_LatitudeLongitude(glm::vec2 v)
{
    float radial = (v.x - 0.5) * M_PI * 2.0f;
    float nradial = (v.y) * M_PI;
    
    return glm::vec3(sin(nradial) * sin(radial), cos(nradial), -sin(nradial) * cos(radial));
}

int CreateSphericalHarmonics(int M, int L, TinyExrImage &dest)
{
    glm::vec2 d, s;
    for (int y = 0; y < dest.height; y++)
    {
        d.y = (y + 0.5) / (dest.height);
        for (int x = 0; x < dest.width; x++)
        {
            d.x = (x + 0.5) / (dest.width);
            glm::vec3 a = PixelToDirection_LatitudeLongitude(d);
            
            
            if (a.x <= 1.0f)
            {
                float val = SphericalHarmonic(M, L, (float)acos(a.y), (float)atan2(a.x, -a.z));
                dest.buf[4 * (dest.width*y + x) + 0] = val;
                dest.buf[4 * (dest.width*y + x) + 1] = val;
                dest.buf[4 * (dest.width*y + x) + 2] = val;
                dest.buf[4 * (dest.width*y + x) + 3] = val;
            }
            else
            {
                dest.buf[4 * (dest.width*y + x) + 0] = 0.0f;
                dest.buf[4 * (dest.width*y + x) + 1] = 0.0f;
                dest.buf[4 * (dest.width*y + x) + 2] = 0.0f;
                dest.buf[4 * (dest.width*y + x) + 3] = 1.0f;
            }
        }
    }
    
    return true;
}

bool ReadSHCoefficients(std::string filepath, int order, Eigen::Matrix3Xf& SHCoeff)
{
    SHCoeff.resize(3, 9);
    std::ifstream infile(filepath);
    std::string line;
    glm::vec3 tmp;
    int L = 0;
    int cnt = 0;
    while (std::getline(infile, line))
    {
        {
            std::istringstream iss(line);
            
            for (int M = 0; M < 2*L+1; M++)
            {
                if (!(iss >> SHCoeff(0,cnt) >> SHCoeff(1,cnt) >> SHCoeff(2,cnt)))
                {
                    printf("Error reading file\n");
                    break;
                } // error
                cnt++;
            }
            L++;
            if (L > order)
            break;
        }
    }
    return true;
}

void ReconstructSHfromSHImage(const int order, Eigen::Matrix3Xf& SHCoeff, const TinyExrImage * SHBasis, TinyExrImage & result)
{
    int w = SHBasis[0].width;
    int h = SHBasis[0].height;
    result.AllocateWithClear(w, h);
    for (int l = 0; l <= order; l++)
    {
        for (int m = 0; m < 2 * l + 1; m++)
        {
            //[FIXME]: probably good to add "+=" operator
            for (int py = 0; py < h; py++)
            {
                for (int px = 0; px < w; px++)
                {
                    result.buf[4 * (w*py + px) + 0] += SHCoeff(0,l*l + m)*SHBasis[l*l + m].buf[4 * (w*py + px) + 0];//R
                    result.buf[4 * (w*py + px) + 1] += SHCoeff(1,l*l + m)*SHBasis[l*l + m].buf[4 * (w*py + px) + 1];//G
                    result.buf[4 * (w*py + px) + 2] += SHCoeff(2,l*l + m)*SHBasis[l*l + m].buf[4 * (w*py + px) + 2];//B
                    result.buf[4 * (w*py + px) + 3] = 1.0 * SHBasis[l*l + m].buf[4 * (w*py + px) + 3];//A
                }
            }
        }
    }
}

void PanoramaSphericalHarmonicsBlurFromSHImage(const int order, const TinyExrImage* SH, TinyExrImage& source, TinyExrImage& result)
{
    TinyExrImage sh, temp;
    int w = SH[0].width;
    int h = SH[0].height;
    result = source;
    result.AllocateWithClear(w, h);
    temp.AllocateWithClear(w, h);
    
    float diffcoef[3] = { 3.14159265358979323f, 2.094395f, 0.785398f }; // 0.0f, -0.130900f, 0.0f, 0.049087f
    
    for (int l = 0; l <= order; l++)
    {
        for (int m = 0; m < 2 * l + 1; m++)
        {
            //CreateSphericalHarmonics(M, L, sh, panorama_format);
            sh = SH[l*l + m];
            //temp.AllocateWithClear(w, h);
            temp.Multiply(sh, source);			// temp = dot product sh, src
            
            glm::vec3 coeff(0.0f, 0.0f, 0.0f);
            
            for (int y = 0; y < temp.height; y++)
            {
                float theta = (y + 0.5) * 3.14159265358979323f / temp.height;
                float sintheta = sin( theta );
                
                for (int x = 0; x < temp.width; x++)
                {
                    coeff.x += temp.buf[4 * (w*y + x) + 0] * sintheta;//R
                    coeff.y += temp.buf[4 * (w*y + x) + 1] * sintheta;//R
                    coeff.z += temp.buf[4 * (w*y + x) + 2] * sintheta;//R
                }
                
            }
            
            coeff *= 3.1415926535f / temp.height;		// dtheta const term
            coeff *= 2 * 3.1415926535f / temp.width;	// dphi const term
            
            sh.Scale(coeff.x * diffcoef[l] / 3.14159265358979323f,
                     coeff.y * diffcoef[l] / 3.14159265358979323f,
                     coeff.z * diffcoef[l] / 3.14159265358979323f,
                     1.0);		// apply diffuse coefficients and scale SH image
            
            result.Add(sh);					// accumulate scaled SH
        }
    }
}
