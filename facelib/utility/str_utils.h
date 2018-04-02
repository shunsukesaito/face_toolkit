//
//  str_utils.h
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 8/25/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//
#pragma once

#include <iostream>
#include <iterator>
#include <fstream>
#include <vector>

inline std::vector<int> string2array(std::string str)
{
    std::istringstream buf(str);
    std::istream_iterator<std::string> beg(buf), end;
    
    std::vector<std::string> tokens(beg, end);
    std::vector<int> out;
    for(auto&& s : tokens)
    {
        out.push_back(std::stoi(s));
    }
    return out;
}
