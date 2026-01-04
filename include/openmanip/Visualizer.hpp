#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include <memory>
#include <string>

struct GLFWwindow;
struct mjvCamera_;
struct mjvOption_;
struct mjvScene_;
struct mjrContext_;

using mjvCamera  = struct mjvCamera_;
using mjvOption  = struct mjvOption_;
using mjvScene   = struct mjvScene_;
using mjrContext = struct mjrContext_;

namespace openmanip {
    class RobotSystem;

    class Visualizer {
        public:
            Visualizer();
            ~Visualizer();

            bool initialize(RobotSystem* robot_system, const std::string& title = "OpenManip");
            bool windowIsOpen() const;
            void render();

        private:
            void *model_ = nullptr;
            void *data_ = nullptr;

            GLFWwindow *window_ = nullptr;
            std::unique_ptr<mjvCamera>  cam_;
            std::unique_ptr<mjvOption>  opt_;
            std::unique_ptr<mjvScene>   scn_;
            std::unique_ptr<mjrContext> ctx_;

            bool buttonLeft_ = false;
            bool buttonRight_ = false;
            double lastX_ = 0;
            double lastY_ = 0;

            static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
            static void cursorPosCallback(GLFWwindow* window, double xpos, double ypos);
            static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);
            static Visualizer* getVis(GLFWwindow* window);
    };
}


#endif // VISUALIZER_HPP