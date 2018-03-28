//
//  pts_loader.h
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 8/25/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//
#pragma once

#include <iostream>
#include <fstream>
#include <vector>

#include <Eigen/Core>

inline std::vector<Eigen::Vector3f> load_pts(std::string file)
{
    std::vector<Eigen::Vector3f> shape;
    std::ifstream fin;
    std::string temp;
    
    fin.open(file);
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
