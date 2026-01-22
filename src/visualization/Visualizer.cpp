#include "openmanip/Visualizer.hpp"
#include "mujoco/mjdata.h"
#include "mujoco/mjmodel.h"
#include "mujoco/mjrender.h"
#include "mujoco/mjvisualize.h"
#include "openmanip/RobotSystem.hpp"
#include "openmanip/MujocoDriver.hpp"
#include "openmanip/logger.hpp"

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <memory>

namespace openmanip {
    Visualizer::Visualizer(): 
        cam_(std::make_unique<mjvCamera>()),
        opt_(std::make_unique<mjvOption>()),
        scn_(std::make_unique<mjvScene>()),
        ctx_(std::make_unique<mjrContext>()){
    }
    
    Visualizer::~Visualizer() {
        if (ctx_) mjr_freeContext(ctx_.get());
        if (scn_) mjv_freeScene(scn_.get());
        if (window_) {
            glfwDestroyWindow(window_);
            glfwTerminate();
        }
        window_ = nullptr;
        logger.info() << "[Visualizer] cleaned up";
    }

    bool Visualizer::initialize(RobotSystem* robot, const std::string& title) {
        if (!robot) return false;

        model_ = robot->getPhysics()->getModel();
        data_ = robot->getPhysics()->getData();

        if (!glfwInit()) return false;
        window_ = glfwCreateWindow(1200, 900, title.c_str(), NULL, NULL);
        if (!window_) { glfwTerminate(); return false; }

        glfwMakeContextCurrent(window_);
        glfwSwapInterval(1);
        glfwSetWindowUserPointer(window_, this);

        mjv_defaultCamera(cam_.get());
        mjv_defaultOption(opt_.get());
        mjv_defaultScene(scn_.get());
        mjr_defaultContext(ctx_.get());

        mjv_makeScene((mjModel*)model_, scn_.get(), 2000);
        mjr_makeContext((mjModel*)model_, ctx_.get(), mjFONTSCALE_150);

        glfwSetMouseButtonCallback(window_, mouseButtonCallback);
        glfwSetCursorPosCallback(window_, cursorPosCallback);
        glfwSetScrollCallback(window_, scrollCallback);
    
        return true;
    }
    
    bool Visualizer::windowIsOpen() const {
        return (window_ && !glfwWindowShouldClose(window_));
    }

    void Visualizer::render(){
        if (!window_) return;

        mjrRect viewport = {0,0,0,0};
        glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

        mjv_updateScene((mjModel*)model_, (mjData*)data_, opt_.get(), NULL, cam_.get(), mjCAT_ALL, scn_.get());
        mjr_render(viewport,scn_.get(),ctx_.get());

        glfwSwapBuffers(window_);
        glfwPollEvents();
    }

    Visualizer* Visualizer::getVis(GLFWwindow* window) {
        return static_cast<Visualizer*>(glfwGetWindowUserPointer(window));
    }

    void Visualizer::mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
        Visualizer* vis = getVis(window);
        vis->buttonLeft_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
        vis->buttonRight_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
        glfwGetCursorPos(window, &vis->lastX_, &vis->lastY_);
    }
    void Visualizer::cursorPosCallback(GLFWwindow* window, double xpos, double ypos){
        Visualizer* vis = getVis(window);
        if (!vis->buttonLeft_ && !vis->buttonRight_) return;

        double dx = xpos - vis->lastX_;
        double dy = ypos - vis->lastY_;
        vis->lastX_ = xpos;
        vis->lastY_ = ypos;

        int width, height;
        glfwGetWindowSize(window, &width, &height);
        if (height < 1) height = 1;
        int action = vis->buttonLeft_  ? mjMOUSE_ROTATE_H : mjMOUSE_MOVE_V;
        mjv_moveCamera((mjModel*)vis->model_, action,  dx / height, dy / height, vis->scn_.get(), vis->cam_.get());
    }

    void Visualizer::scrollCallback(GLFWwindow* window, double xoffset, double yoffset){
        Visualizer* vis = getVis(window);
        mjv_moveCamera((mjModel*)vis->model_, mjMOUSE_ZOOM, 0, -0.05 * yoffset, vis->scn_.get(), vis->cam_.get());
    }
}