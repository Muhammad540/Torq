#include "openmanip/Gui.hpp"
#include "imgui_internal.h"
#include "mujoco/mjrender.h"
#include "mujoco/mjvisualize.h"
#include "openmanip/RobotSystem.hpp"
#include "openmanip/MujocoDriver.hpp"
#include "openmanip/logger.hpp"

#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

namespace openmanip {
    Gui::Gui(): 
        cam_(std::make_unique<mjvCamera>()),
        opt_(std::make_unique<mjvOption>()),
        scn_(std::make_unique<mjvScene>()),
        ctx_(std::make_unique<mjrContext>()){
    }
    
    Gui::~Gui() {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();

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
        model_ = static_cast<mjModel*>(robot->getPhysics()->getModel());
        data_ = static_cast<mjData*>(robot->getPhysics()->getData());

        if (!glfwInit()) return false;
        // Valid on GLFW 3.3+ only
        float main_scale = ImGui_ImplGlfw_GetContentScaleForMonitor(glfwGetPrimaryMonitor()); 
        window_ = glfwCreateWindow((int)(1280 * main_scale), (int)(800 * main_scale), title.c_str(), nullptr, nullptr);
        if (!window_) { glfwTerminate(); return false; }

        glfwMakeContextCurrent(window_);
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
        ImGuiIO& io = ImGui::GetIO(); (void)io;
        io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

        ImGui::StyleColorsDark();
        // Setup Platform/Renderer backends
        ImGui_ImplGlfw_InitForOpenGL(window_, true);
        ImGui_ImplOpenGL3_Init("#version 330");
        
        int nj = model_->nu;
        joint_targets_.resize(nj, 0.0f);

        lin_step_ = static_cast<float>(robot_->jogLinearStep());
        ang_step_ = static_cast<float>(robot_->jogAngularStep());
        
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

        // #ifdef ENABLE_TRACKING_POINTS
        //     for (const auto& point: tps){
        //         if (scn_->ngeom < scn_->maxgeom){
        //             mjvGeom* next_geom = scn_->geoms + scn_->ngeom;
        //             float color[4] = {(float)point.color[0], (float)point.color[1], (float)point.color[2], 1.0f};
        //             double pos[3] = {(double)point.position[0],(double)point.position[1],(double)point.position[2]};
        //             double size[3] = {(double)point.radius, 0.0f, 0.0f};
                    
        //             mjv_initGeom(next_geom, mjGEOM_SPHERE,size,pos,NULL,color);
        //             next_geom->category = mjCAT_DECOR;
        //             next_geom->emission = 0.5f;
        //             scn_->ngeom += 1;
        //         } else {
        //             logger.warning() << "[Gui] Max geom exceeded!"; 
        //             break;
        //         }
        //     }

        //     tps.clear();
        // #endif
        mjr_render(mjr_viewport,scn_.get(),ctx_.get());
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glfwPollEvents();
        
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGuiID dockspaceid = ImGui::DockSpaceOverViewport();

        if (first_frame_){
            setupDockLayout(dockspaceid);
            first_frame_ = false;
        }

        // drawMenuBar();
        drawViewport();
        drawJointControlPanel();
        drawCartesianPanel();
        // drawInfoPanel();    
        
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
        ImGui::DockBuilderSplitNode(dockerspace_id, ImGuiDir_Right, 0.25f, &dock_right, &dock_center);

        ImGui::DockBuilderDockWindow("Viewport", dock_center);
        ImGui::DockBuilderDockWindow("Joint Control", dock_left);
        ImGui::DockBuilderDockWindow("Cartesian Control", dock_right);
        ImGui::DockBuilderFinish(dockerspace_id);
    }

    void Gui::drawViewport(){
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
        ImGui::Begin("Viewport", nullptr, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        
        ImVec2 img_size = ImVec2(static_cast<float>(viewport_width_),static_cast<float>(viewport_height_));

        ImGui::Image(
            (ImTextureID)(intptr_t)texture_color_,
            img_size,
            ImVec2(0.0f, 1.0f), // UV top left     (flipped)
            ImVec2(1.0f, 0.0f)  // UV bottom right (flipped)
        );

        if (ImGui::IsItemHovered()){
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

	static char frame_buf[128] = "gripper_frame_link";
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
	    if (ImGui::Button(neg, ImVec2(bw, 0)))
		robot_->jogCartesian(axis, -1.0, frame);
	    ImGui::SameLine();
	    if (ImGui::Button(pos, ImVec2(bw, 0)))
		robot_->jogCartesian(axis, +1.0, frame);
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
