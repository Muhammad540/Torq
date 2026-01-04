#include "openmanip/RobotSystem.hpp"
#include "openmanip/Visualizer.hpp"
#include <iostream>

int main(int argc, char** argv){
    openmanip::RobotSystem robot;
    std::string modelPath = "/home/ambi/OpenManip/workspace/models/SO101/so101_new_calib.xml";

    if (!robot.initialize(modelPath)){
        std::cerr << "Failed to load model" << std::endl;
        return 1;
    }

    openmanip::Visualizer vis;
    vis.initialize(&robot, "SO-100 Control");

    while (vis.windowIsOpen()){
        for (int i=0; i<10; ++i){
            robot.update();
        }
        vis.render();
    }

    return 0;
}