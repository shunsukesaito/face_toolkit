//
//  main.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 6/10/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include <iostream>

#include "gui.h"

#include <gflags/gflags.h>
DEFINE_uint32(win_w, 800, "window width");
DEFINE_uint32(win_h, 800, "window height");

int main(int argc, char **argv) try
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    // insert code here...
    GUI *gui = GUI::getInstance();
    
    gui->init(FLAGS_win_w, FLAGS_win_h);
    gui->loop();
    
    return 0;
}
catch (const std::exception& e)
{
    std::cout << "Halted: " << e.what() << std::endl;
    return EXIT_FAILURE;
}
