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
#include "pts_loader.h"

std::vector<Eigen::Vector3f> load_pts(std::string file)
{
    std::vector<Eigen::Vector3f> shape;
    std::ifstream fin;
    std::string temp;
    
    fin.open(file);
    if(!fin.is_open()){
        std::cout << "Warning: failed parsing pts from " << file << std::endl;
        return std::vector<Eigen::Vector3f>();
    }
    
    std::getline(fin, temp);
    std::getline(fin, temp, ' ');
    std::getline(fin, temp, ' ');
    std::getline(fin, temp);
    int n_land = std::stoi(temp);
    std::getline(fin, temp);
    for(int i = 0; i < n_land; ++i)
    {
        float x, y;
        fin >> x >> y;
        shape.push_back(Eigen::Vector3f(x,y,1.0));
    }
    fin.close();
    
    return shape;
}

bool write_pts(std::string file, const std::vector<Eigen::Vector3f>& pts)
{
    std::ofstream fout(file);
    if(!fout.is_open()){
        std::cout << "Warning: failed writing pts to " << file << std::endl;
        return false;
    }
    
    fout << "version: 1" << std::endl;
    fout << "n_points:  " << pts.size() << std::endl;
    fout << "{" << std::endl;
    for(int i = 0; i < pts.size(); ++i)
    {
        fout << pts[i][0] << " " << pts[i][1] << std::endl;
    }
    fout << "}";
    fout.close();
    
    return true;
}
