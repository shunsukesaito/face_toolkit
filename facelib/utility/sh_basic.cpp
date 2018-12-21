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

#include "sh_basic.h"

const static float s_c3 = 0.94617469575; // (3*sqrt(5))/(4*sqrt(pi))
const static float s_c4 = -0.31539156525;// (-sqrt(5))/(4*sqrt(pi))
const static float s_c5 = 0.54627421529; // (sqrt(15))/(4*sqrt(pi))

const static float s_c_scale = 1.0/0.91529123286551084;
const static float s_c_scale_inv = 0.91529123286551084;

const static float s_rc2 = 1.5853309190550713*s_c_scale;
const static float s_c4_div_c3 = s_c4/s_c3;
const static float s_c4_div_c3_x2 = (s_c4/s_c3)*2.0;

const static float s_scale_dst2 = s_c3 * s_c_scale_inv;
const static float s_scale_dst4 = s_c5 * s_c_scale_inv;

// 0 multiplies
static void OptRotateBand0(float dst[1], const float src[1], const Eigen::Matrix3f& R)
{
    dst[0] = src[0];
}

// 9 multiplies
static void OptRotateBand1(float dst[3], const float src[3], const Eigen::Matrix3f& R)
{
    // derived from  SlowRotateBand1
    dst[0] = ( R(1,1))*src[0] + (-R(1,2))*src[1] + ( R(1,0))*src[2];
    dst[1] = (-R(2,1))*src[0] + ( R(2,2))*src[1] + (-R(2,0))*src[2];
    dst[2] = ( R(0,1))*src[0] + (-R(0,2))*src[1] + ( R(0,0))*src[2];
}

// 48 multiplies
static void OptRotateBand2(float dst[5], const float x[5],
                           float m00, float m01, float m02,
                           float m10, float m11, float m12,
                           float m20, float m21, float m22)
{
    // Sparse matrix multiply
    float sh0 =  x[3] + x[4] + x[4] - x[1];
    float sh1 =  x[0] + s_rc2*x[2] +  x[3] + x[4];
    float sh2 =  x[0];
    float sh3 = -x[3];
    float sh4 = -x[1];
    
    // Rotations.  R0 and R1 just use the raw matrix columns
    float r2x = m00 + m01;
    float r2y = m10 + m11;
    float r2z = m20 + m21;
    
    float r3x = m00 + m02;
    float r3y = m10 + m12;
    float r3z = m20 + m22;
    
    float r4x = m01 + m02;
    float r4y = m11 + m12;
    float r4z = m21 + m22;
    
    // dense matrix multiplication one column at a time
    
    // column 0
    float sh0_x = sh0 * m00;
    float sh0_y = sh0 * m10;
    float d0 = sh0_x * m10;
    float d1 = sh0_y * m20;
    float d2 = sh0 * (m20 * m20 + s_c4_div_c3);
    float d3 = sh0_x * m20;
    float d4 = sh0_x * m00 - sh0_y * m10;
    
    // column 1
    float sh1_x = sh1 * m02;
    float sh1_y = sh1 * m12;
    d0 += sh1_x * m12;
    d1 += sh1_y * m22;
    d2 += sh1 * (m22 * m22 + s_c4_div_c3);
    d3 += sh1_x * m22;
    d4 += sh1_x * m02 - sh1_y * m12;
    
    // column 2
    float sh2_x = sh2 * r2x;
    float sh2_y = sh2 * r2y;
    d0 += sh2_x * r2y;
    d1 += sh2_y * r2z;
    d2 += sh2 * (r2z * r2z + s_c4_div_c3_x2);
    d3 += sh2_x * r2z;
    d4 += sh2_x * r2x - sh2_y * r2y;
    
    // column 3
    float sh3_x = sh3 * r3x;
    float sh3_y = sh3 * r3y;
    d0 += sh3_x * r3y;
    d1 += sh3_y * r3z;
    d2 += sh3 * (r3z * r3z + s_c4_div_c3_x2);
    d3 += sh3_x * r3z;
    d4 += sh3_x * r3x - sh3_y * r3y;
    
    // column 4
    float sh4_x = sh4 * r4x;
    float sh4_y = sh4 * r4y;
    d0 += sh4_x * r4y;
    d1 += sh4_y * r4z;
    d2 += sh4 * (r4z * r4z + s_c4_div_c3_x2);
    d3 += sh4_x * r4z;
    d4 += sh4_x * r4x - sh4_y * r4y;
    
    // extra multipliers
    dst[0] = d0;
    dst[1] = -d1;
    dst[2] = d2 * s_scale_dst2;
    dst[3] = -d3;
    dst[4] = d4 * s_scale_dst4;
}

static float factratio(int N, int D)        // ratio of N! / D!
{
    if (N >= D)
    {
        float prod = 1.0f;
        for (int x = D + 1; x <= N; x++)
            prod *= x;
        return prod;
    }
    else    // D > N
    {
        float prod = 1.0f;
        for (int x = N + 1; x <= D; x++)
            prod *= x;
        return 1.0f / prod;
    }
}

static float K(int M, int L)
{
    return sqrt(((2 * L + 1) / (4 * M_PI)) * (factratio(L - M, L + M)));
}

static float AssociatedLegendre(int M, int L, float x)        // computes the associated legendre polynomial Pm,l(x)   x in [0, 1] and 0 <= M <= L
{
    if ((M < 0) || (M > L) || (fabs(x) > 1.0f))        // error
        return 0.0f;
    
    float pmm = 1.0f;                                // compute Pm,m(x) = (-1)^M * (2m-1)!! * (1-x^2)^(M/2)
    if (M > 0)
    {
        float somx2 = sqrt((1.0f - x) * (1.0f + x));
        float fact = 1.0f;
        for (int i = 1; i <= M; i++)
        {
            pmm = -pmm * fact * somx2;
            fact = fact + 2;
        }
    }
    
    if (L == M)                                        // if L == M then Pl,m(x) == Pm,m(x)
        return pmm;
    else                                            // otherwise L > M
    {
        float pmmp1 = x * (2 * M + 1) * pmm;        // calc Pm,m+1(x)
        if (L == M+1)
            return pmmp1;
        else                                        // otherwise, L > M+1,  use recurrence
        {
            float pll = 0.0;
            for (int i = M+2; i <= L; i++)
            {
                pll = (x * (2 * i - 1) * pmmp1 - (i + M - 1) * pmm) / (i - M);
                pmm = pmmp1;
                pmmp1 = pll;
            }
            return pll;
        }
    }
}

float SphericalHarmonic(int M, int L, float theta, float phi)
{
    if (M > 0)
    {
        return sqrt(2.0f) * K(M, L) * cos(M * phi) * AssociatedLegendre(M, L, cos(theta));
    }
    else if (M < 0)
    {
        return sqrt(2.0f) * K(-M, L) * sin(-M * phi) * AssociatedLegendre(-M, L, cos(theta));
    }
    else
        return K(0, L) * AssociatedLegendre(0, L, cos(theta));
}

void RotateSHCoefficients(const Eigen::Matrix3Xf &src, Eigen::Matrix3Xf &tar, float x, float y, float z)
{
    assert(src.cols() == 9);
    if(tar.cols() != 9)
        tar = src;
    
    Eigen::Matrix3f R;
    R = Eigen::AngleAxisf(x, Eigen::Vector3f::UnitX())
    * Eigen::AngleAxisf(y,  Eigen::Vector3f::UnitY())
    * Eigen::AngleAxisf(z, Eigen::Vector3f::UnitZ());
    
    //const float* sh = src.data();
    float sh[27], sh_rotated[27];
    for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 9; ++j)
            sh[i*9+j] = src(i,j);
    
    // R
    OptRotateBand0(&sh_rotated[0],&sh[0],R);
    OptRotateBand1(&sh_rotated[1],&sh[1],R);
    OptRotateBand2(&sh_rotated[4],&sh[4],
                   R(0,0),R(0,1),R(0,2),
                   R(1,0),R(1,1),R(1,2),
                   R(2,0),R(2,1),R(2,2));
    // G
    OptRotateBand0(&sh_rotated[0+9],&sh[0+9],R);
    OptRotateBand1(&sh_rotated[1+9],&sh[1+9],R);
    OptRotateBand2(&sh_rotated[4+9],&sh[4+9],
                   R(0,0),R(0,1),R(0,2),
                   R(1,0),R(1,1),R(1,2),
                   R(2,0),R(2,1),R(2,2));
    // B
    OptRotateBand0(&sh_rotated[0+18],&sh[0+18],R);
    OptRotateBand1(&sh_rotated[1+18],&sh[1+18],R);
    OptRotateBand2(&sh_rotated[4+18],&sh[4+18],
                   R(0,0),R(0,1),R(0,2),
                   R(1,0),R(1,1),R(1,2),
                   R(2,0),R(2,1),R(2,2));
    
    for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 9; ++j)
            tar(i,j) = sh_rotated[i*9+j];
}

