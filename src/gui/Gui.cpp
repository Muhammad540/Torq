#include "torq/Gui.hpp"
#include "imgui_internal.h"
#include "mujoco/mjrender.h"
#include "mujoco/mjvisualize.h"
#include "torq/RobotSystem.hpp"
#include "torq/MujocoDriver.hpp"
#include "torq/logger.hpp"
#include <cstring>
#include <cmath>

#include <GL/glew.h>
#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glext.h>
#else
#include <GL/gl.h>
#include <GL/glext.h>
#endif
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"

namespace torq {
    Gui::Gui(): 
        cam_(std::make_unique<mjvCamera>()),
        opt_(std::make_unique<mjvOption>()),
        scn_(std::make_unique<mjvScene>()),
        ctx_(std::make_unique<mjrContext>()){
    }
    
    Gui::~Gui() {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImPlot::DestroyContext();
        ImGui::DestroyContext();

        if (!pip_cameras_.empty() && display_pip){
            for (auto& pip : pip_cameras_) {
                if (pip.texture) glDeleteTextures(1, &pip.texture);
                if (pip.rbo_depth) glDeleteRenderbuffers(1, &pip.rbo_depth);
                if (pip.fbo) glDeleteFramebuffers(1, &pip.fbo);
            }
        }

        destroyFBO();

        if (ctx_) mjr_freeContext(ctx_.get());
        if (scn_) mjv_freeScene(scn_.get());
        if (window_) {
            glfwDestroyWindow(window_);
            glfwTerminate();
        }
        window_ = nullptr;
        logger.info() << "[Gui] cleaned up";
    }

    bool Gui::initialize(RobotSystem* robot, const std::string& title) {
        if (!robot) return false;
        robot_ = robot;
        if (!robot->mujocoVisualizationDriver()) {
            logger.error() << "[Gui] No MuJoCo model available for rendering. "
                              "Use driver_type=\"mujoco\", or provide scene_path with driver_type=\"serial_servo\" for display mirror.";
            return false;
        }
        MujocoDriver* mj = robot->mujocoVisualizationDriver();
        model_ = static_cast<mjModel*>(mj->getModel());
        data_ = static_cast<mjData*>(mj->getData());

        if (!glfwInit()) return false;
        // Valid on GLFW 3.3+ only
        float main_scale = ImGui_ImplGlfw_GetContentScaleForMonitor(glfwGetPrimaryMonitor());
        window_ = glfwCreateWindow((int)(1280 * main_scale), (int)(800 * main_scale), title.c_str(), nullptr, nullptr);
        if (!window_) { glfwTerminate(); return false; }

        glfwMakeContextCurrent(window_);
        glewExperimental = GL_TRUE;
        glewInit();
        glfwSwapInterval(1); // to enable vsync
        glfwSetWindowUserPointer(window_, this);

        model_->vis.headlight.active = 1;
        
        mjv_defaultCamera(cam_.get());
        mjv_defaultOption(opt_.get());
        mjv_defaultScene(scn_.get());
        mjr_defaultContext(ctx_.get());
        mjv_makeScene(model_, scn_.get(), 2000);
        mjr_makeContext(model_, ctx_.get(), mjFONTSCALE_150);
        cam_->type = mjCAMERA_FREE;

        createFBO(viewport_width_, viewport_height_);
        
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImPlot::CreateContext();
        ImGuiIO& io = ImGui::GetIO(); (void)io;
        io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

        ImGui::StyleColorsDark();
        // Setup Platform/Renderer backends
        ImGui_ImplGlfw_InitForOpenGL(window_, true);
#ifdef __APPLE__
        ImGui_ImplOpenGL3_Init("#version 120");
#else
        ImGui_ImplOpenGL3_Init("#version 330");
#endif
        
        int nj = model_->nu;
        joint_targets_.resize(nj, 0.0f);
        pos_history_.resize(nj);
        vel_history_.resize(nj);

        lin_step_ = static_cast<float>(robot_->jogLinearStep());
        ang_step_ = static_cast<float>(robot_->jogAngularStep());


        if (display_pip){
            for (int i = 0; i < model_->ncam; ++i) {
                PiPCamera pip;
                pip.name = mj_id2name(model_, mjOBJ_CAMERA, i);
                pip.camera_id = i;
                pip.cam = std::make_unique<mjvCamera>();
                mjv_defaultCamera(pip.cam.get());
                pip.cam->type = mjCAMERA_FIXED;
                pip.cam->fixedcamid = i;
                pip_cameras_.push_back(std::move(pip));
            }
            if (!pip_cameras_.empty()) {
                logger.info() << "[Gui] Discovered " << pip_cameras_.size() << " PiP camera(s)";
            }
        }

        return true;
    }
    
    bool Gui::windowIsOpen() const {
        return (window_ && !glfwWindowShouldClose(window_));
    }

    void Gui::render(){
        if (!window_) return;

        glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
        glViewport(0, 0, viewport_width_, viewport_height_);
        mjrRect mjr_viewport = {0, 0, viewport_width_, viewport_height_};
        mjv_updateScene(model_, data_, opt_.get(), NULL, cam_.get(), mjCAT_ALL, scn_.get());

        mjr_render(mjr_viewport, scn_.get(), ctx_.get());
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        if (display_pip){
            // pip size is scaled down version of the full viewport
            int pip_w = std::max(1, static_cast<int>(viewport_width_ * pip_scale_));
            int pip_h = std::max(1, static_cast<int>(viewport_height_ * pip_scale_));
            for (auto& pip : pip_cameras_) {
                if (pip.width != pip_w || pip.height != pip_h) {
                    if (pip.fbo) {
                        glDeleteFramebuffers(1, &pip.fbo);
                        glDeleteTextures(1, &pip.texture);
                        glDeleteRenderbuffers(1, &pip.rbo_depth);
                    }
                    pip.width = pip_w;
                    pip.height = pip_h;

                    glGenFramebuffers(1, &pip.fbo);
                    glBindFramebuffer(GL_FRAMEBUFFER, pip.fbo);

                    glGenTextures(1, &pip.texture);
                    glBindTexture(GL_TEXTURE_2D, pip.texture);
                    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, pip_w, pip_h, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, pip.texture, 0);

                    glGenRenderbuffers(1, &pip.rbo_depth);
                    glBindRenderbuffer(GL_RENDERBUFFER, pip.rbo_depth);
                    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, pip_w, pip_h);
                    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, pip.rbo_depth);
                    glBindFramebuffer(GL_FRAMEBUFFER, 0);
                }

                glBindFramebuffer(GL_FRAMEBUFFER, pip.fbo);
                glViewport(0, 0, pip_w, pip_h);
                mjrRect pip_rect = {0, 0, pip_w, pip_h};
                mjv_updateScene(model_, data_, opt_.get(), NULL, pip.cam.get(), mjCAT_ALL, scn_.get());
                mjr_render(pip_rect, scn_.get(), ctx_.get());
                glBindFramebuffer(GL_FRAMEBUFFER, 0);
            }

            if (!pip_cameras_.empty()) {
                mjv_updateScene(model_, data_, opt_.get(), NULL, cam_.get(), mjCAT_ALL, scn_.get());
            }
        }
        glfwPollEvents();
        
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGuiID dockspaceid = ImGui::DockSpaceOverViewport();

        if (first_frame_){
            setupDockLayout(dockspaceid);
            first_frame_ = false;
        }

        drawViewport();
        drawJointControlPanel();
        drawJointPlots();
        drawCartesianPanel();
        drawIKTuningPanel();
        
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window_, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color_.at(0) * clear_color_.at(3), clear_color_.at(1) * clear_color_.at(3), clear_color_.at(2) * clear_color_.at(3), clear_color_.at(3));
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // Keeping track of simulation time 
        // TODO(AHMED): Add wall clock time
        // char timestr[50];
        // std::snprintf(timestr, 50, "Time: %.3f", data_->time);
        // mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, viewport, timestr, NULL, ctx_.get());

        glfwSwapBuffers(window_);
    }

    void Gui::createFBO(int width, int height){
        glGenFramebuffers(1, &fbo_);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo_);

        glGenTextures(1, &texture_color_);
        glBindTexture(GL_TEXTURE_2D, texture_color_);
        glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA16,width,height,0,GL_RGB,GL_UNSIGNED_BYTE,NULL);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture_color_, 0);

        glGenRenderbuffers(1, &rbo_depth_);
        glBindRenderbuffer(GL_RENDERBUFFER, rbo_depth_);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, width, height);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo_depth_);

        GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
        if (status != GL_FRAMEBUFFER_COMPLETE) {
            logger.error() << "[Gui] FBO incomplete, status: " << status;
        }
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    void Gui::setupDockLayout(unsigned int dockerspace_id){
        ImGui::DockBuilderRemoveNodeChildNodes(dockerspace_id);
        ImGui::DockBuilderAddNode(dockerspace_id, ImGuiDockNodeFlags_DockSpace);
        ImGui::DockBuilderSetNodeSize(dockerspace_id, ImGui::GetMainViewport()->Size);
        
        ImGuiID dock_left, dock_center, dock_right;
        ImGui::DockBuilderSplitNode(dockerspace_id, ImGuiDir_Left, 0.25f, &dock_left, &dock_center);
        ImGui::DockBuilderSplitNode(dock_center, ImGuiDir_Right, 0.33f, &dock_right, &dock_center);

        ImGuiID dock_right_top, dock_right_bottom;
        ImGui::DockBuilderSplitNode(dock_right, ImGuiDir_Down, 0.55f, &dock_right_bottom, &dock_right_top);

        ImGuiID dock_left_top, dock_left_bottom;
        ImGui::DockBuilderSplitNode(dock_left, ImGuiDir_Down, 0.55f, &dock_left_bottom, &dock_left_top);

        ImGui::DockBuilderDockWindow("Viewport", dock_center);
        ImGui::DockBuilderDockWindow("Joint Control", dock_left_top);
        ImGui::DockBuilderDockWindow("Joint Plots", dock_left_bottom);
        ImGui::DockBuilderDockWindow("Cartesian Control", dock_right_top);
        ImGui::DockBuilderDockWindow("IK Tuning", dock_right_bottom);
        ImGui::DockBuilderFinish(dockerspace_id);
    }

    void Gui::drawViewport(){
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
        ImGui::Begin("Viewport", nullptr, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        
        ImVec2 avail = ImGui::GetContentRegionAvail();
        int new_w = std::max(1, static_cast<int>(avail.x));
        int new_h = std::max(1, static_cast<int>(avail.y));

        if (new_w != viewport_width_ || new_h != viewport_height_) {
            viewport_width_ = new_w;
            viewport_height_ = new_h;
            destroyFBO();
            createFBO(viewport_width_, viewport_height_);
        }
    
        ImVec2 img_size = ImVec2(static_cast<float>(viewport_width_), static_cast<float>(viewport_height_));

        ImGui::Image(
            (ImTextureID)(intptr_t)texture_color_,
            img_size,
            ImVec2(0.0f, 1.0f), // UV top left     (flipped)
            ImVec2(1.0f, 0.0f)  // UV bottom right (flipped)
        );

        bool viewport_hovered = ImGui::IsItemHovered();
        ImVec2 img_origin = ImGui::GetItemRectMin();
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        {
            int pip_pad = 6;
            for (int i = 0; i < static_cast<int>(pip_cameras_.size()); ++i) {
                auto& pip = pip_cameras_[i];
                if (!pip.texture || pip.width <= 0 || pip.height <= 0) continue;

                float fx = static_cast<float>(viewport_width_ - pip.width - pip_pad);
                float fy = static_cast<float>(i * (pip.height + pip_pad + 16) + pip_pad);
                ImVec2 p0(img_origin.x + fx, img_origin.y + fy);
                ImVec2 p1(p0.x + pip.width, p0.y + pip.height);

                draw_list->AddRectFilled(p0, p1, IM_COL32(0, 0, 0, 200));
                draw_list->AddImage(
                    (ImTextureID)(intptr_t)pip.texture,
                    p0, p1,
                    ImVec2(0.0f, 1.0f), ImVec2(1.0f, 0.0f)
                );
                draw_list->AddRect(p0, p1, IM_COL32(255, 255, 255, 200), 0.0f, 0, 2.0f);

                ImVec2 label_pos(p0.x + 4.0f, p1.y + 1.0f);
                draw_list->AddText(label_pos, IM_COL32(255, 255, 255, 220), pip.name.c_str());
            }
        }

        {
            mjvGLCamera* glcam = scn_->camera;
            mjtNum fwd[3], up[3], right[3];
            mju_f2n(fwd, glcam[0].forward, 3);
            mju_f2n(up, glcam[0].up, 3);
            mju_cross(right, fwd, up);
            double cx = (glcam[0].pos[0] + glcam[1].pos[0]) * 0.5;
            double cy = (glcam[0].pos[1] + glcam[1].pos[1]) * 0.5;
            double cz = (glcam[0].pos[2] + glcam[1].pos[2]) * 0.5;

            ImVec2 img_end = ImGui::GetItemRectMax();
            ImVec2 btn_pos(img_origin.x + 8.0f, img_end.y - 28.0f);
            ImGui::SetCursorScreenPos(btn_pos);
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(6, 4));
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.15f, 0.15f, 0.85f));
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1, 1, 1, 0.9f));
            if (ImGui::SmallButton("Copy Camera Spec")) {
                char spec[512];
                std::snprintf(spec, sizeof(spec),
                    "<camera pos=\"%.4f %.4f %.4f\" xyaxes=\"%.4f %.4f %.4f %.4f %.4f %.4f\"/>",
                    cx, cy, cz,
                    right[0], right[1], right[2],
                    up[0], up[1], up[2]);
                ImGui::SetClipboardText(spec);
                logger.info() << "[Gui] Copied to clipboard: " << spec;
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("pos=(%.3f, %.3f, %.3f)\nxyaxes=(%.3f %.3f %.3f, %.3f %.3f %.3f)",
                    cx, cy, cz, right[0], right[1], right[2], up[0], up[1], up[2]);
            }
            ImGui::PopStyleColor(2);
            ImGui::PopStyleVar();
        }

        if (viewport_hovered){
            float wheel = ImGui::GetIO().MouseWheel;
            if (wheel != 0.0f){
                mjv_moveCamera(model_, mjMOUSE_ZOOM, 0, -0.05 * wheel, scn_.get(), cam_.get());
            }

            bool left_down   =  ImGui::IsMouseDown(ImGuiMouseButton_Left);
            bool right_down  =  ImGui::IsMouseDown(ImGuiMouseButton_Right);
            bool middle_down =  ImGui::IsMouseDown(ImGuiMouseButton_Middle);
            
            ImVec2 mouse_delta = ImGui::GetIO().MouseDelta;

            if ((left_down || right_down || middle_down) && (mouse_delta.x != 0 || mouse_delta.y != 0)){
                int action = mjMOUSE_NONE;
                if (left_down){
                    if (ImGui::GetIO().KeyShift) {
                        action = mjMOUSE_MOVE_H;
                    } else {
                        action = mjMOUSE_ROTATE_H;
                    }
                } else if (right_down) {
                    action = mjMOUSE_MOVE_V;
                } else if (middle_down) {
                    action = mjMOUSE_ZOOM;
                }

                mjv_moveCamera(model_, action, mouse_delta.x/(double)viewport_height_, mouse_delta.y/(double)viewport_height_, scn_.get(), cam_.get());
            }
        }

        ImGui::End();
        ImGui::PopStyleVar();
    }

    void Gui::drawJointControlPanel(){
        ImGui::Begin("Joint Control");


        ImGui::Text("Forward Kinematics - Joint Jog");
        ImGui::Separator();
        
        for (int i = 0; i < static_cast<int>(joint_targets_.size()); i++) {
            joint_targets_[i] = static_cast<float>(data_->ctrl[i]);
        }

        bool changed = false;
        for (int i=0; i<static_cast<int>(joint_targets_.size()); i++){
            const char* jname = mj_id2name(model_, mjOBJ_ACTUATOR, i); 
            std::string label = jname ? jname : ("Joint "+std::to_string(i));

            double low  = model_->actuator_ctrlrange[2*i];
            double high = model_->actuator_ctrlrange[2*i+1];
            
            changed |= ImGui::SliderFloat(label.c_str(), &joint_targets_[i], low, high, "%.2f rad", ImGuiSliderFlags_ClampZeroRange);
        }

        if (changed){
            size_t num_joints = joint_targets_.size();
            Eigen::VectorXd q(num_joints);
            for (size_t i=0;i < num_joints; i++){
                q[i]=joint_targets_[i];
            }
            robot_->setJointSpaceTarget(q);
        }

	ImGui::Separator();
	if (ImGui::Button("Set Home")) {
	    robot_->setHomePosition();
	}
	ImGui::SameLine();
	if (ImGui::Button("Move to Home")) {
	    robot_->moveToHome();
	}
	if (!robot_->hasHomePosition()) {
	    ImGui::TextColored(ImVec4(1,1,0,1), "Home not set");
	}

        ImGui::End();    
    }

    void Gui::drawCartesianPanel() {
        ImGui::Begin("Cartesian Control");
        ImGui::Text("Inverse Kinematics - Task Space Jog");
        ImGui::Separator();

        static bool frame_initialized = false;
        static char frame_buf[128] = {};
        if (!frame_initialized) {
            std::strncpy(frame_buf, robot_->endEffectorFrame().c_str(), sizeof(frame_buf) - 1);
            frame_initialized = true;
        }
        ImGui::InputText("Frame", frame_buf, sizeof(frame_buf));
        std::string frame(frame_buf);

        ImGui::Separator();

        bool step_changed = false;
        step_changed |= ImGui::DragFloat("Linear Step (m)", &lin_step_, 0.001f, 0.001f, 0.1f, "%.3f");
        step_changed |= ImGui::DragFloat("Angular Step (rad)", &ang_step_, 0.005f, 0.01f, 0.5f, "%.3f");
        if (step_changed) {
            robot_->setJogStep(lin_step_, ang_step_);
        }

        ImGui::Separator();
        ImGui::Text("Position");

        float bw = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;

        auto jogButtons = [&](const char* neg, const char* pos, int axis) {
            if (ImGui::Button(neg, ImVec2(bw, 0))) {}
            bool negHeld = ImGui::IsItemActive();
            ImGui::SameLine();
            if (ImGui::Button(pos, ImVec2(bw, 0))) {}
            bool posHeld = ImGui::IsItemActive();
            if (negHeld)robot_->jogCartesian(axis, -1.0, frame);
            if (posHeld)robot_->jogCartesian(axis, +1.0, frame);
        };

        jogButtons("-X", "+X", 0);
        jogButtons("-Y", "+Y", 1);
        jogButtons("-Z", "+Z", 2);

        ImGui::Separator();
        ImGui::Text("Orientation");

        jogButtons("-Roll",  "+Roll",  3);
        jogButtons("-Pitch", "+Pitch", 4);
        jogButtons("-Yaw",   "+Yaw",   5);

        ImGui::Separator();
        const char* gripper_label = robot_->isGripperOpen() ? "Close Gripper" : "Open Gripper";
        if (ImGui::Button(gripper_label, ImVec2(-1, 0))) {
            robot_->toggleGripper();
        }
        ImGui::End();
    }
    void Gui::drawIKTuningPanel() {
        ImGui::Begin("IK Tuning");

        if (!robot_->ikReady()) {
            ImGui::TextColored(ImVec4(1, 1, 0, 1), "IK not active (use Cartesian jog to initialize)");
            ImGui::End();
            return;
        }

        if (!ik_panel_initialized_) {
            const auto& cfg = robot_->ikConfig();
            ik_frame_pos_cost_     = static_cast<float>(cfg.frame_position_cost);
            ik_frame_ori_cost_     = static_cast<float>(cfg.frame_orientation_cost);
            ik_frame_gain_         = static_cast<float>(cfg.frame_gain);
            ik_frame_lm_damping_   = static_cast<float>(cfg.frame_lm_damping);
            ik_posture_cost_       = static_cast<float>(cfg.posture_cost);
            ik_posture_gain_       = static_cast<float>(cfg.posture_gain);
            ik_posture_lm_damping_ = static_cast<float>(cfg.posture_lm_damping);
            ik_damping_cost_       = static_cast<float>(cfg.damping_cost);
            ik_solver_damping_     = static_cast<float>(cfg.solver_damping);
            ik_config_limit_gain_  = static_cast<float>(cfg.config_limit_gain);
            ik_panel_initialized_ = true;
        }

        if (ImGui::CollapsingHeader("Frame Task", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::DragFloat("Position Cost##frame", &ik_frame_pos_cost_, 0.01f, 0.0f, 100.0f, "%.4f"))
                robot_->setFrameTaskPositionCost(ik_frame_pos_cost_);
            if (ImGui::DragFloat("Orientation Cost##frame", &ik_frame_ori_cost_, 0.01f, 0.0f, 100.0f, "%.4f"))
                robot_->setFrameTaskOrientationCost(ik_frame_ori_cost_);
            if (ImGui::SliderFloat("Gain##frame", &ik_frame_gain_, 0.0f, 1.0f, "%.3f"))
                robot_->setFrameTaskGain(ik_frame_gain_);
            if (ImGui::DragFloat("LM Damping##frame", &ik_frame_lm_damping_, 0.001f, 0.0f, 10.0f, "%.4f"))
                robot_->setFrameTaskLMDamping(ik_frame_lm_damping_);
        }

        if (ImGui::CollapsingHeader("Posture Task", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::DragFloat("Cost##posture", &ik_posture_cost_, 0.0001f, 0.0f, 1.0f, "%.6f"))
                robot_->setPostureTaskCost(ik_posture_cost_);
            if (ImGui::SliderFloat("Gain##posture", &ik_posture_gain_, 0.0f, 1.0f, "%.3f"))
                robot_->setPostureTaskGain(ik_posture_gain_);
            if (ImGui::DragFloat("LM Damping##posture", &ik_posture_lm_damping_, 0.001f, 0.0f, 10.0f, "%.4f"))
                robot_->setPostureTaskLMDamping(ik_posture_lm_damping_);
        }

        if (ImGui::CollapsingHeader("Damping Task", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::DragFloat("Cost##damping", &ik_damping_cost_, 0.00001f, 0.0f, 0.1f, "%.6f"))
                robot_->setDampingTaskCost(ik_damping_cost_);
        }

        if (ImGui::CollapsingHeader("Solver", ImGuiTreeNodeFlags_DefaultOpen)) {
            float log_damping = std::log10(std::max(ik_solver_damping_, 1e-20f));
            if (ImGui::SliderFloat("Tikhonov Damping (log10)", &log_damping, -15.0f, -3.0f, "%.1f")) {
                ik_solver_damping_ = std::pow(10.0f, log_damping);
                robot_->setSolverDamping(ik_solver_damping_);
            }
            ImGui::SameLine();
            ImGui::Text("= %.2e", static_cast<double>(ik_solver_damping_));
        }

        if (ImGui::CollapsingHeader("Limits", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::SliderFloat("Config Limit Gain", &ik_config_limit_gain_, 0.01f, 1.0f, "%.3f"))
                robot_->setConfigLimitGain(ik_config_limit_gain_);
        }

        ImGui::End();
    }

    void Gui::drawJointPlots(){
        ImGui::Begin("Joint Plots");

        float t = static_cast<float>(ImGui::GetTime());
        Eigen::VectorXd q  = robot_->getJointPositions();
        Eigen::VectorXd qd = robot_->getJointVelocities();

        int nj = static_cast<int>(pos_history_.size());
        int nq = static_cast<int>(q.size());
        int nv = static_cast<int>(qd.size());

        for (int i = 0; i < nj && i < nq; ++i)
            pos_history_[i].addPoint(t, static_cast<float>(q[i]));
        for (int i = 0; i < nj && i < nv; ++i)
            vel_history_[i].addPoint(t, static_cast<float>(qd[i]));

        ImGui::SliderFloat("History (s)", &plot_history_s_, 1.0f, 30.0f, "%.1f");

        float plot_h = (ImGui::GetContentRegionAvail().y - ImGui::GetFrameHeightWithSpacing()) * 0.5f;

        if (ImPlot::BeginPlot("Positions", ImVec2(-1, plot_h))) {
            ImPlot::SetupAxes("time (s)", "rad");
            ImPlot::SetupAxisLimits(ImAxis_X1, t - plot_history_s_, t, ImPlotCond_Always);
            for (int i = 0; i < nj; ++i) {
                const char* name = mj_id2name(model_, mjOBJ_ACTUATOR, i);
                std::string label = name ? name : ("j" + std::to_string(i));
                auto& buf = pos_history_[i];
                if (buf.size() > 0) {
                    ImPlotSpec spec;
                    spec.Offset = buf.offset;
                    ImPlot::PlotLine(label.c_str(), buf.time.data(), buf.data.data(), buf.size(), spec);
                }
            }
            ImPlot::EndPlot();
        }

        if (ImPlot::BeginPlot("Velocities", ImVec2(-1, plot_h))) {
            ImPlot::SetupAxes("time (s)", "rad/s");
            ImPlot::SetupAxisLimits(ImAxis_X1, t - plot_history_s_, t, ImPlotCond_Always);
            for (int i = 0; i < nj; ++i) {
                const char* name = mj_id2name(model_, mjOBJ_ACTUATOR, i);
                std::string label = name ? name : ("j" + std::to_string(i));
                auto& buf = vel_history_[i];
                if (buf.size() > 0) {
                    ImPlotSpec spec;
                    spec.Offset = buf.offset;
                    ImPlot::PlotLine(label.c_str(), buf.time.data(), buf.data.data(), buf.size(), spec);
                }
            }
            ImPlot::EndPlot();
        }

        ImGui::End();
    }

    void Gui::destroyFBO(){ 
        if (texture_color_){
            glDeleteTextures(1, &texture_color_);
            texture_color_ = 0;
        } if (rbo_depth_) {
            glDeleteRenderbuffers(1, &rbo_depth_);
            rbo_depth_ = 0;
        } if (fbo_) {
            glDeleteFramebuffers(1, &fbo_);
            fbo_ = 0;
        }
    }

    Gui* Gui::getGui(GLFWwindow* window) {
        return static_cast<Gui*>(glfwGetWindowUserPointer(window));
    }

    // #ifdef ENABLE_TRACKING_POINTS
    // void Gui::drawtp(const Eigen::Vector3d &pos, double radius, const Eigen::Vector3d &color){
    //     tps.push_back({pos, radius, color});
    // }
    // #endif
}
