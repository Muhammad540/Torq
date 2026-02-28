#include "openmanip/RobotSystem.hpp"
#include "openmanip/Gui.hpp"
#include "openmanip/logger.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <filesystem>
#include <iostream>

static Logger logger;

int main(int argc, char** argv){
    openmanip::RobotSystem robot;
    std::filesystem::path root(PROJECT_ROOT);
    std::string modelPath_xml = (root / "workspace/models/SO101/scene.xml").string();
    std::string modelPath_urdf = (root / "workspace/models/SO101/so101_new_calib.urdf").string();
    if (!robot.initialize(modelPath_xml, modelPath_urdf)){
        logger.error() << "Failed to load model";
        return 1;
    }

    openmanip::Gui gui;
    gui.initialize(&robot, "Robot Gui");

    while (gui.windowIsOpen()){
        for (int i=0; i<10; ++i){
            robot.update();
        }
        gui.render();
    }

    return 0;
}