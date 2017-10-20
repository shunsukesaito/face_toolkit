//
//  fps.hpp
//  RenderingTemplate
//
//  Created by Shunsuke Saito on 9/13/17.
//  Copyright © 2017 Shunsuke Saito. All rights reserved.
//

#ifndef fps_hpp
#define fps_hpp

#include <iostream>
#include <chrono>

// using moving average
struct FPSCounter
{
    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    std::chrono::system_clock::time_point lastFrame = std::chrono::system_clock::now();

    inline float count(){
        now = std::chrono::system_clock::now();
        std::chrono::duration<double, std::milli> work_time = now - lastFrame;
        fps = decay*(1000.0/work_time.count())+(1.0-decay)*fps;
        lastFrame = now;
        return fps;
    }
    
    float fps = 0.0;
    float decay = 0.2;
};


#endif /* fps_hpp */
