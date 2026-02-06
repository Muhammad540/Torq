#include "openmanip/RobotSystem.hpp"
#include "openmanip/Visualizer.hpp"
#include "openmanip/logger.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <iostream>

static Logger logger;

int main(int argc, char** argv){
    openmanip::RobotSystem robot;
    std::string modelPath_xml = "/home/ambi/OpenManip/workspace/models/SO101/so101_new_calib.xml";
    std::string modelPath_urdf = "/home/ambi/OpenManip/workspace/models/SO101/so101_new_calib.urdf";
    if (!robot.initialize(modelPath_xml, modelPath_urdf)){
        logger.error() << "Failed to load model";
        return 1;
    }

    openmanip::Visualizer vis;
    vis.initialize(&robot, "SO-100 Control");

    // TEST: IK
    Eigen::Matrix4d target = Eigen::Matrix4d::Identity();
    target.block<3,1>(0,3) << 0.5, 0.3, 0.5;
    
    robot.setTaskSpaceTarget(target, "gripper_frame_link");

    while (vis.windowIsOpen()){
        for (int i=0; i<10; ++i){
            robot.update();
        }
        vis.render();
    }

    return 0;
}