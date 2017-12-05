//
//  main.cpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 6/10/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#include <iostream>

#include "gui.h"

int main(int argc, const char * argv[]) try
{
    // insert code here...
    GUI *gui = GUI::getInstance();
    
    gui->init(1280, 720);
    gui->loop();
    
    return 0;
}
catch (const std::exception& e)
{
    std::cout << "Halted: " << e.what() << std::endl;
    return EXIT_FAILURE;
}
