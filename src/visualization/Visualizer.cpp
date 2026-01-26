#include "openmanip/Visualizer.hpp"
#include "mujoco/mjdata.h"
#include "mujoco/mjmodel.h"
#include "mujoco/mjrender.h"
#include "mujoco/mjtnum.h"
#include "mujoco/mjvisualize.h"
#include "openmanip/RobotSystem.hpp"
#include "openmanip/MujocoDriver.hpp"
#include "openmanip/logger.hpp"

#include <cstdio>
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

        // NOTE(AHMED): Viusalizer class doesnot own the model_ or data_ (RobotSystem Class does)
        model_ = static_cast<mjModel*>(robot->getPhysics()->getModel());
        data_ = static_cast<mjData*>(robot->getPhysics()->getData());

        if (!glfwInit()) return false;
        glfwWindowHint(GLFW_SAMPLES, 4);
        window_ = glfwCreateWindow(1200, 900, title.c_str(), NULL, NULL);
        if (!window_) { glfwTerminate(); return false; }

        glfwMakeContextCurrent(window_);
        glfwSwapInterval(1);
        glfwSetWindowUserPointer(window_, this);

        model_->vis.headlight.active = 1;
        
        mjv_defaultCamera(cam_.get());
        mjv_defaultOption(opt_.get());
        mjv_defaultScene(scn_.get());
        mjr_defaultContext(ctx_.get());

        mjv_makeScene(model_, scn_.get(), 2000);
        mjr_makeContext(model_, ctx_.get(), mjFONTSCALE_150);

        glfwSetMouseButtonCallback(window_, mouseButtonCallback);
        glfwSetCursorPosCallback(window_, cursorPosCallback);
        glfwSetScrollCallback(window_, scrollCallback);
        
        cam_->type = mjCAMERA_FREE;
        return true;
    }
    
    bool Visualizer::windowIsOpen() const {
        return (window_ && !glfwWindowShouldClose(window_));
    }

    void Visualizer::render(){
        if (!window_) return;

        //NOTE(AHMED) origin is bottom left
        mjrRect viewport = {0,0,0,0};
        glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

        mjv_updateScene(model_, data_, opt_.get(), NULL, cam_.get(), mjCAT_ALL, scn_.get());
        
        #ifdef ENABLE_TRACKING_POINTS
        for (const auto& point: tps){
            if (scn_->ngeom < scn_->maxgeom){
                mjvGeom* next_geom = scn_->geoms + scn_->ngeom;
                float color[4] = {(float)point.color[0], (float)point.color[1], (float)point.color[2], 1.0f};
                double pos[3] = {(double)point.position[0],(double)point.position[1],(double)point.position[2]};
                double size[3] = {(double)point.radius, 0.0f, 0.0f};
                
                mjv_initGeom(next_geom, mjGEOM_SPHERE,size,pos,NULL,color);
                next_geom->category = mjCAT_DECOR;
                next_geom->emission = 0.5f;
                scn_->ngeom += 1;
            } else {
                logger.warning() << "[VISUALIZER] Max geom exceeded!"; 
                break;
            }
        }

        tps.clear();
        #endif
        mjr_render(viewport,scn_.get(),ctx_.get());

        // Keeping track of simulation time 
        // TODO(AHMED): Add wall clock time
        char timestr[50];
        std::snprintf(timestr, 50, "Time: %.3f", data_->time);
        mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, viewport, timestr, NULL, ctx_.get());

        glfwSwapBuffers(window_);
        glfwPollEvents();
    }

    void Visualizer::toggleOption(int flag){
        opt_->flags[(mjtVisFlag)flag] = !opt_->flags[(mjtVisFlag)flag]; 
    }

    void Visualizer::toggleFrame(int frameType){
        opt_->frame = (opt_->frame == (mjtFrame)frameType) ? mjFRAME_NONE : frameType; 
    }

    Visualizer* Visualizer::getVis(GLFWwindow* window) {
        return static_cast<Visualizer*>(glfwGetWindowUserPointer(window));
    }

    void Visualizer::mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
        Visualizer* vis = getVis(window);
        vis->buttonLeft_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
        vis->buttonRight_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
        vis->buttonMiddle_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
        glfwGetCursorPos(window, &vis->lastX_, &vis->lastY_);
    }
    void Visualizer::cursorPosCallback(GLFWwindow* window, double xpos, double ypos){
        Visualizer* vis = getVis(window);
        if (!vis->buttonLeft_ && !vis->buttonRight_  && !vis->buttonMiddle_) return;

        double dx = xpos - vis->lastX_;
        double dy = ypos - vis->lastY_;
        vis->lastX_ = xpos;
        vis->lastY_ = ypos;

        int width, height;
        glfwGetWindowSize(window, &width, &height);
        if (height < 1) height = 1;
        int action = vis->buttonLeft_  ? mjMOUSE_ROTATE_H : mjMOUSE_MOVE_V;
        mjv_moveCamera(vis->model_, action,  dx / height, dy / height, vis->scn_.get(), vis->cam_.get());
    }

    void Visualizer::scrollCallback(GLFWwindow* window, double xoffset, double yoffset){
        Visualizer* vis = getVis(window);
        mjv_moveCamera(vis->model_, mjMOUSE_ZOOM, 0, -0.05 * yoffset, vis->scn_.get(), vis->cam_.get());
    }
    #ifdef ENABLE_TRACKING_POINTS
    void Visualizer::drawtp(const Eigen::Vector3d &pos, double radius, const Eigen::Vector3d &color){
        tps.push_back({pos, radius, color});
    }
    #endif
}