#ifndef TORQ_GUI_HPP
#define TORQ_GUI_HPP
#include <Eigen/Core>
#include "torq/logger.hpp"
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


namespace torq {
    class RobotSystem;

    struct PiPCamera {
        std::string name;
        int camera_id = -1;
        std::unique_ptr<mjvCamera> cam;
        unsigned int fbo = 0;
        unsigned int texture = 0;
        unsigned int rbo_depth = 0;
        int width = 0;
        int height = 0;
    };
    
    #ifdef ENABLE_TRACKING_POINTS
    /**
     * @brief A debug visualisation point rendered in the 3D viewport.
     */
    struct TrackingPoint{
        Eigen::Vector3d position; ///< World-frame position.
        double radius;            ///< Sphere radius [m].
        Eigen::Vector3d color;    ///< RGB colour (0–1).
    };
    #endif

    struct ScrollingBuffer {
        int max_size;
        int offset;
        std::vector<float> time;
        std::vector<float> data;
        ScrollingBuffer(int max_sz = 2000) : max_size(max_sz), offset(0) {
            time.reserve(max_sz);
            data.reserve(max_sz);
        }
        void addPoint(float t, float v) {
            if (static_cast<int>(time.size()) < max_size) {
                time.push_back(t);
                data.push_back(v);
            } else {
                time[offset] = t;
                data[offset] = v;
                offset = (offset + 1) % max_size;
            }
        }
        void clear() { time.clear(); data.clear(); offset = 0; }
        int size() const { return static_cast<int>(time.size()); }
    };

    /**
     * @brief ImGui + GLFW + OpenGL3 docking GUI for Torq.
     *
     * Provides a 3D MuJoCo viewport, joint-control panel, Cartesian jog panel,
     * real-time joint plots, and an IK tuning panel.  The GUI does
     * not own the RobotSystem — it only holds a non-owning pointer and
     * reads/writes through the public API.
     *
     * @see RobotSystem
     */
    class Gui {
        public:
            Gui();
            ~Gui();

            /**
             * @brief Initialise the GUI window and attach to a RobotSystem.
             * @param robot_system  Non-owning pointer to the robot (must outlive the GUI).
             * @param title         Window title.
             * @return True on success.
             */
            bool initialize(RobotSystem* robot_system, const std::string& title = "Torq GUI");

            /** @brief True while the window has not been closed. */
            bool windowIsOpen() const;

            /**
             * @brief Render one frame.
             * @note The user application is responsible for calling this in its main loop.
             */
            void render();

            /** @brief Direct access to the MuJoCo camera (for programmatic view control). */
            mjvCamera* getCamera() { return cam_.get(); }

            #ifdef ENABLE_TRACKING_POINTS
            /**
             * @brief Draw a tracking point in the 3D viewport.
             * @param pos     World-frame position.
             * @param radius  Sphere radius [m].
             * @param color   RGB colour (0–1).
             */
            void drawtp(const Eigen::Vector3d& pos, double radius, const Eigen::Vector3d& color);
            #endif
        private:
            RobotSystem* robot_ = nullptr;
            mjModel *model_ = nullptr;
            mjData *data_ = nullptr;

            GLFWwindow *window_ = nullptr;
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

            void ensureMujocoOffscreenAtLeast(int min_w, int min_h);
            void renderMujocoBlitToFbo(unsigned int dst_fbo, int w, int h, mjvCamera* cam);
            bool first_frame_ = true;

            void setupDockLayout(unsigned int dockerspace_id);
            void drawViewport();
            void drawJointControlPanel();
            void drawCartesianPanel();
            void drawIKTuningPanel();
            void drawJointPlots();

            std::vector<PiPCamera> pip_cameras_;
            float pip_scale_ = 0.25f;
            bool display_pip = true;

            std::vector<float> joint_targets_;
            std::string ik_frame_name_;
            int selected_frame_idx_ = 0;
            float lin_step_;
            float ang_step_;

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
	
            std::vector<ScrollingBuffer> pos_history_;
            std::vector<ScrollingBuffer> vel_history_;
            float plot_history_s_ = 5.0f;

            static Gui* getGui(GLFWwindow* window);

            mutable Logger logger;
    };
}

#endif // TORQ_GUI_HPP
