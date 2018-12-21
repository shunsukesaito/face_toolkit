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

#include <random>
#include "sampler.h"
#include "sh_basic.h"

float random(int n)
{
    return (rand() % (n+1)) / (float)n;
}

Sampler::Sampler(unsigned n)
{
    srand((unsigned)time(NULL));

    for(int i = 0; i < n; ++i)
    {
        for(int j = 0; j < n; ++j)
        {
            Eigen::Vector3f dir;
            Eigen::Vector2f sph;

            float x = ((float)i + random(n*n))/(float)n;
            float y = ((float)j + random(n*n))/(float)n;
            sph[0] = 2.0f*acos(sqrt(1.0f-x));
            sph[1] = 2.0f*(float)M_PI*y;

            // NOTE: this somehow matches with analytical solution (not sure...)
            dir[0] = -sin(sph[0])*cos(sph[1]);
            dir[1] = -sin(sph[0])*sin(sph[1]);
            dir[2] = cos(sph[0]);

            samples_.push_back(Sample(dir,sph));
        }
    }
}

void Sampler::computeSH(int band)
{
    int band2 = band * band;
    unsigned size = samples_.size();
    for(unsigned i = 0; i < size; ++i)
    {
        samples_[i].shValue_.resize(band2);
        for(int l = 0; l < band; ++l)
        {
            for(int m = -l; m <= l;++m)
            {
                int index = l*(l + 1) + m;
                samples_[i].shValue_[index] = SphericalHarmonic(m,l,samples_[i].spCoord_[0],samples_[i].spCoord_[1]);
                
                if(std::isnan(samples_[i].shValue_[index]))
                {
                    std::cerr << "Sampler::computeSH - get NaN." << std::endl;
                }

                if(fabs(samples_[i].shValue_[index]) > 1)
                {
                    std::cerr << "Sampler::computeSH - SH value exceeds 1." << std::endl;
                }
            }
        }
    }
}
