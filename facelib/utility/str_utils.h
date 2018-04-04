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
#include <map>
#include <vector>

inline void replace_str_all(std::string& str, const std::string& from, const std::string& to) {
    if(from.empty())
        return;
    size_t start_pos = 0;
    while((start_pos = str.find(from, start_pos)) != std::string::npos) {
        str.replace(start_pos, from.length(), to);
        start_pos += to.length(); // In case 'to' contains 'from', like replacing 'x' with 'yx'
    }
}

inline void find_erase_all(std::string& str, const std::string & toErase)
{
    size_t pos = std::string::npos;
    
    // Search for the substring in string in a loop untill nothing is found
    while ((pos  = str.find(toErase) )!= std::string::npos)
    {
        // If found then erase it from string
        str.erase(pos, toErase.length());
    }
}

inline std::vector<std::string> split_str(const std::string& str, const std::string& delim)
{
    std::vector<std::string> tokens;
    size_t prev = 0, pos = 0;
    do
    {
        pos = str.find(delim, prev);
        if (pos == std::string::npos) pos = str.length();
        std::string token = str.substr(prev, pos-prev);
        if (!token.empty()) tokens.push_back(token);
        prev = pos + delim.length();
    }
    while (pos < str.length() && prev < str.length());

    return tokens;
}

inline std::map<std::string, bool> vector2map(const std::vector<std::string>& vec)
{
    std::map<std::string, bool> map;
    for(auto&& s : vec)
        map[s] = true;
    
    return map;
}

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
