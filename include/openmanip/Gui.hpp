#ifndef OPENMANIP_GUI_HPP
#define OPENMANIP_GUI_HPP
#include <Eigen/Core>
#include "openmanip/logger.hpp"
#include <Eigen/Dense>
#include <vector>
#include <array>
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


namespace openmanip {
    class RobotSystem;
    
    #ifdef ENABLE_TRACKING_POINTS
    struct TrackingPoint{
        Eigen::Vector3d position;
        double radius;
        Eigen::Vector3d color;
    };
    #endif

    class Gui {
        public:
            Gui();
            ~Gui();

            bool initialize(RobotSystem* robot_system, const std::string& title = "OpenManip GUI");
            bool windowIsOpen() const;
            /* NOTE(AHMED): user application is responsible to call this */
            void render();

            mjvCamera* getCamera() { return cam_.get(); }

            // Methods for debugging
            #ifdef ENABLE_TRACKING_POINTS
            // Draws tracking point (internally updates the mjv Scene), user should call this method to add specific stracking points.
            void drawtp(const Eigen::Vector3d& pos, double radius, const Eigen::Vector3d& color);
            #endif
        private:
            // Non owning ptrs
            RobotSystem* robot_ = nullptr;
            mjModel *model_ = nullptr;
            mjData *data_ = nullptr;

            GLFWwindow *window_ = nullptr;
            // NOTE: mjvscene holds list of all visual objects that need to be drawn in a single frame (This can be used to add decorative points like contact points, force vectors etc)
            std::unique_ptr<mjvCamera>  cam_;
            std::unique_ptr<mjvOption>  opt_;
            std::unique_ptr<mjvScene>   scn_;
            std::unique_ptr<mjrContext> ctx_;

            unsigned int fbo_ = 0;
            unsigned int rbo_depth_ = 0;
            unsigned int texture_color_ = 0;
            int viewport_width_ = 1200;
            int viewport_height_ = 900;
            std::array<float, 4> clear_color_ = {0.45f, 0.55f, 0.60f, 1.00f};
            void createFBO(int width, int height);
            void destroyFBO();
            bool first_frame_ = true;

            void setupDockLayout(unsigned int dockerspace_id);
            // void drawMenuBar();
            void drawViewport();
            void drawJointControlPanel();
            void drawCartesianPanel();
            void drawIKTuningPanel();
            // void drawInfoPanel();

            std::vector<float> joint_targets_;
            std::string ik_frame_name_;
            int selected_frame_idx_ = 0;
            float lin_step_;
            float ang_step_;

            // IK tuning panel state (synced from IKConfig on first frame)
            bool ik_panel_initialized_ = false;
            float ik_frame_pos_cost_ = 1.0f;
            float ik_frame_ori_cost_ = 1.0f;
            float ik_frame_gain_ = 1.0f;
            float ik_frame_lm_damping_ = 0.0f;
            float ik_posture_cost_ = 1e-3f;
            float ik_posture_gain_ = 1.0f;
            float ik_posture_lm_damping_ = 0.0f;
            float ik_damping_cost_ = 1e-4f;
            float ik_solver_damping_ = 1e-12f;
            float ik_config_limit_gain_ = 0.5f;
	
            static Gui* getGui(GLFWwindow* window);

            // Debugging Specific
            mutable Logger logger;
            // #ifdef ENABLE_TRACKING_POINTS
            // std::vector<openmanip::TrackingPoint> tps;
            // #endif
    };
}

#endif // OPENMANIP_GUI_HPP
