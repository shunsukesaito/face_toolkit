//
//  str_utils.h
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 8/25/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//
#pragma once

#include <iostream>
#include <sstream>
#include <iterator>
#include <fstream>
#include <map>
#include <vector>

void replace_str_all(std::string& str, const std::string& from, const std::string& to);
void find_erase_all(std::string& str, const std::string & toErase);
std::vector<std::string> split_str(const std::string& str, const std::string& delim);
std::map<std::string, bool> vector2map(const std::vector<std::string>& vec);

std::vector<int> string2arrayi(std::string str);
std::vector<float> string2arrayf(std::string str);


