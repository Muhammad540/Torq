#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include <Eigen/Dense>
#include "openmanip/logger.hpp"
#include <vector>
#include <memory>
#include <string>

struct GLFWwindow;
struct mjModel_;
struct mjData_;
struct mjvCamera_;
struct mjvOption_;
struct mjvScene_;
struct mjrContext_;

using mjvCamera  = struct mjvCamera_;
using mjvOption  = struct mjvOption_;
using mjvScene   = struct mjvScene_;
using mjrContext = struct mjrContext_;
using mjModel = struct mjModel_;
using mjData = struct mjData_;


class Logger; 

namespace openmanip {
    class RobotSystem;
    
    #ifdef ENABLE_TRACKING_POINTS
    struct TrackingPoint{
        Eigen::Vector3d position;
        double radius;
        Eigen::Vector3d color;
    };
    #endif

    class Visualizer {
        public:
            Visualizer();
            ~Visualizer();

            bool initialize(RobotSystem* robot_system, const std::string& title = "OpenManip");
            bool windowIsOpen() const;
            /* NOTE(AHMED): user application is responsible to call this */
            void render();

            void toggleOption(int flag);
            void toggleFrame(int frameType);

            mjvCamera* getCamera() { return cam_.get(); }

            // Methods for debugging
            #ifdef ENABLE_TRACKING_POINTS
            // Draws tracking point (internally updates the mjv Scene), user should call this method to specific the tracking points.
            void drawtp(const Eigen::Vector3d& pos, double radius, const Eigen::Vector3d& color);
            #endif
        private:
            mjModel *model_ = nullptr;
            mjData *data_ = nullptr;

            GLFWwindow *window_ = nullptr;
            std::unique_ptr<mjvCamera>  cam_;
            std::unique_ptr<mjvOption>  opt_;
            // NOTE: mjvscene holds list of all visual objects that need to be drawn in a single frame (This can be used to add decorative points like contact points, force vectors etc)
            std::unique_ptr<mjvScene>   scn_;
            std::unique_ptr<mjrContext> ctx_;

            bool buttonLeft_ = false;
            bool buttonRight_ = false;
            bool buttonMiddle_ = false;
            double lastX_ = 0;
            double lastY_ = 0;

            static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
            static void cursorPosCallback(GLFWwindow* window, double xpos, double ypos);
            static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);
            static Visualizer* getVis(GLFWwindow* window);

            // Debugging Specific
            mutable Logger logger;
            #ifdef ENABLE_TRACKING_POINTS
            std::vector<openmanip::TrackingPoint> tps;
            #endif
    };
}


#endif // VISUALIZER_HPP