/*
 @licstart  The following is the entire license notice for the JavaScript code in this file.

 The MIT License (MIT)

 Copyright (C) 1997-2020 by Dimitri van Heesch

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 and associated documentation files (the "Software"), to deal in the Software without restriction,
 including without limitation the rights to use, copy, modify, merge, publish, distribute,
 sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or
 substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 @licend  The above is the entire license notice for the JavaScript code in this file
*/
var NAVTREE =
[
  [ "Torq", "index.html", [
    [ "Key features", "index.html#autotoc_md131", null ],
    [ "Quick start", "index.html#autotoc_md133", null ],
    [ "Documentation", "index.html#autotoc_md135", null ],
    [ "Building the documentation", "index.html#autotoc_md137", null ],
    [ "Embedding videos in documentation", "index.html#autotoc_md138", null ],
    [ "Conventions and Notation", "conventions.html", [
      [ "Notation table", "conventions.html#autotoc_md56", null ],
      [ "Coordinate frames", "conventions.html#autotoc_md58", [
        [ "World frame", "conventions.html#autotoc_md59", null ],
        [ "Body (local) frame", "conventions.html#autotoc_md60", null ],
        [ "Jacobian reference frames", "conventions.html#autotoc_md61", null ],
        [ "SE(3) transform composition", "conventions.html#autotoc_md62", null ]
      ] ],
      [ "Manifold operations", "conventions.html#autotoc_md64", [
        [ "Difference (\\f$\\ominus\\f$)", "conventions.html#autotoc_md65", null ],
        [ "Integration (\\f$\\oplus\\f$)", "conventions.html#autotoc_md66", null ],
        [ "Why this matters", "conventions.html#autotoc_md67", null ]
      ] ],
      [ "Task convention", "conventions.html#autotoc_md69", null ],
      [ "Limit convention", "conventions.html#autotoc_md71", null ],
      [ "Barrier convention", "conventions.html#autotoc_md73", null ],
      [ "Units", "conventions.html#autotoc_md75", null ]
    ] ],
    [ "Getting Started", "quickstart_page.html", [
      [ "Prerequisites", "quickstart_page.html#autotoc_md168", null ],
      [ "Step 1: Configure the robot", "quickstart_page.html#autotoc_md170", null ],
      [ "Step 2: Create the GUI", "quickstart_page.html#autotoc_md172", null ],
      [ "Step 3: Run the control loop", "quickstart_page.html#autotoc_md174", null ],
      [ "Step 4: Drive the end-effector", "quickstart_page.html#autotoc_md176", null ],
      [ "What happens under the hood", "quickstart_page.html#autotoc_md178", null ],
      [ "Step 5: Tune the parameters", "quickstart_page.html#autotoc_md180", null ],
      [ "Complete example", "quickstart_page.html#autotoc_md182", null ],
      [ "Next steps", "quickstart_page.html#autotoc_md184", null ]
    ] ],
    [ "Tutorials", "tutorials_page.html", [
      [ "Tutorial 2: Adding regularisation (PostureTask + DampingTask)", "tutorials_page.html#autotoc_md285", [
        [ "Key concepts", "tutorials_page.html#autotoc_md282", null ],
        [ "Code", "tutorials_page.html#autotoc_md283", null ],
        [ "What goes wrong without regularisation", "tutorials_page.html#autotoc_md286", null ],
        [ "Built-in regularisation", "tutorials_page.html#autotoc_md287", null ],
        [ "Tuning", "tutorials_page.html#autotoc_md288", null ]
      ] ],
      [ "Tutorial 3: Working with limits", "tutorials_page.html#autotoc_md290", [
        [ "Built-in limits", "tutorials_page.html#autotoc_md291", null ],
        [ "Adding an acceleration limit", "tutorials_page.html#autotoc_md292", null ],
        [ "Effect of configuration limit gain", "tutorials_page.html#autotoc_md293", null ]
      ] ],
      [ "Tutorial 4: Barrier functions for safety", "tutorials_page.html#autotoc_md295", [
        [ "Workspace bounds with PositionBarrier", "tutorials_page.html#autotoc_md296", null ],
        [ "Collision avoidance with BodySphericalBarrier", "tutorials_page.html#autotoc_md297", null ],
        [ "Self-collision avoidance", "tutorials_page.html#autotoc_md298", null ]
      ] ],
      [ "</blockquote>", "tutorials_page.html#autotoc_md299", null ],
      [ "Tutorial 5: Custom tasks for complex robots", "tutorials_page.html#autotoc_md300", [
        [ "Center of mass regulation", "tutorials_page.html#autotoc_md301", null ],
        [ "Relative frame tracking (foot relative to pelvis)", "tutorials_page.html#autotoc_md302", null ],
        [ "Joint coupling (symmetric knees)", "tutorials_page.html#autotoc_md303", null ]
      ] ],
      [ "Tutorial 6: Sim-to-real deployment", "tutorials_page.html#autotoc_md305", null ],
      [ "Recording and attaching tutorial videos", "tutorials_page.html#autotoc_md307", null ]
    ] ],
    [ "Architecture", "architecture.html", [
      [ "System diagram", "architecture.html#autotoc_md5", null ],
      [ "Public API and ownership", "architecture.html#autotoc_md6", [
        [ "Ownership hierarchy", "architecture.html#autotoc_md7", null ]
      ] ],
      [ "Class hierarchy", "architecture.html#autotoc_md8", [
        [ "Core", "architecture.html#autotoc_md9", null ],
        [ "Kinematics", "architecture.html#autotoc_md10", null ],
        [ "Tasks (QP objective) — see @ref tasks_page", "architecture.html#autotoc_md11", null ],
        [ "Limits (QP inequality constraints) — see @ref limits_page", "architecture.html#autotoc_md12", null ],
        [ "Barriers (CBF inequality + optional objective) — see @ref barriers_page", "architecture.html#autotoc_md13", null ],
        [ "Hardware abstraction", "architecture.html#autotoc_md14", null ],
        [ "GUI", "architecture.html#autotoc_md15", null ]
      ] ],
      [ "Ownership diagram", "architecture.html#autotoc_md16", null ],
      [ "Control loop frequencies", "architecture.html#autotoc_md17", null ],
      [ "Data flow per tick", "architecture.html#autotoc_md18", null ],
      [ "Adding a new robot", "architecture.html#autotoc_md19", null ]
    ] ],
    [ "QP Formulation", "qp_formulation.html", [
      [ "Overview", "qp_formulation.html#autotoc_md140", null ],
      [ "Per-task contribution", "qp_formulation.html#autotoc_md142", [
        [ "Weight matrix", "qp_formulation.html#autotoc_md143", null ],
        [ "Weighted quantities", "qp_formulation.html#autotoc_md144", null ],
        [ "Levenberg-Marquardt damping", "qp_formulation.html#autotoc_md145", null ],
        [ "Per-task Hessian and gradient", "qp_formulation.html#autotoc_md146", null ]
      ] ],
      [ "Composing multiple tasks", "qp_formulation.html#autotoc_md148", null ],
      [ "Tikhonov (solver) damping", "qp_formulation.html#autotoc_md150", null ],
      [ "Inequality constraints from limits", "qp_formulation.html#autotoc_md152", null ],
      [ "Barrier contributions", "qp_formulation.html#autotoc_md154", [
        [ "Barrier inequality rows", "qp_formulation.html#autotoc_md155", null ],
        [ "Barrier objective terms", "qp_formulation.html#autotoc_md156", null ]
      ] ],
      [ "The complete QP", "qp_formulation.html#autotoc_md158", null ],
      [ "OSQP solver", "qp_formulation.html#autotoc_md160", null ],
      [ "Manifold-aware integration", "qp_formulation.html#autotoc_md162", null ],
      [ "Velocity output", "qp_formulation.html#autotoc_md164", null ],
      [ "Summary of the assembly pipeline", "qp_formulation.html#autotoc_md166", null ]
    ] ],
    [ "Tasks", "tasks_page.html", [
      [ "Task overview", "tasks_page.html#autotoc_md196", null ],
      [ "Common parameters", "tasks_page.html#task_common_params", [
        [ "Generic QP contribution", "tasks_page.html#task_qp_generic", null ]
      ] ],
      [ "FrameTask", "tasks_page.html#frametask", [
        [ "Error (6-D)", "tasks_page.html#autotoc_md199", null ],
        [ "Jacobian (6 x \\f$n_v\\f$)", "tasks_page.html#autotoc_md200", null ],
        [ "Weight vector", "tasks_page.html#autotoc_md201", null ],
        [ "QP contribution", "tasks_page.html#autotoc_md202", null ],
        [ "Setters", "tasks_page.html#autotoc_md203", null ]
      ] ],
      [ "PostureTask", "tasks_page.html#posturetask", [
        [ "Error (\\f$n_v\\f$-D)", "tasks_page.html#autotoc_md205", null ],
        [ "Jacobian (\\f$n_v \\times n_v\\f$)", "tasks_page.html#autotoc_md206", null ],
        [ "QP contribution", "tasks_page.html#autotoc_md207", null ],
        [ "Velocity interpretation", "tasks_page.html#autotoc_md208", null ],
        [ "Setters", "tasks_page.html#autotoc_md209", null ]
      ] ],
      [ "DampingTask", "tasks_page.html#dampingtask", [
        [ "Error", "tasks_page.html#autotoc_md211", null ],
        [ "Jacobian", "tasks_page.html#autotoc_md212", null ],
        [ "QP contribution", "tasks_page.html#autotoc_md213", null ],
        [ "Connection to Tikhonov regularisation", "tasks_page.html#autotoc_md214", null ]
      ] ],
      [ "ComTask", "tasks_page.html#comtask", [
        [ "Error (3-D)", "tasks_page.html#autotoc_md216", null ],
        [ "Jacobian (3 x \\f$n_v\\f$)", "tasks_page.html#autotoc_md217", null ],
        [ "QP contribution", "tasks_page.html#autotoc_md218", null ],
        [ "Setters", "tasks_page.html#autotoc_md219", null ],
        [ "Example", "tasks_page.html#autotoc_md220", null ]
      ] ],
      [ "RelativeFrameTask", "tasks_page.html#relativeframetask", [
        [ "Error (6-D)", "tasks_page.html#autotoc_md222", null ],
        [ "Jacobian (6 x \\f$n_v\\f$)", "tasks_page.html#autotoc_md223", null ],
        [ "QP contribution", "tasks_page.html#autotoc_md224", null ],
        [ "Setters", "tasks_page.html#autotoc_md225", null ],
        [ "Example", "tasks_page.html#autotoc_md226", null ]
      ] ],
      [ "LinearHolonomicTask", "tasks_page.html#linearholonomictask", [
        [ "Error (\\f$p\\f$-D)", "tasks_page.html#autotoc_md228", null ],
        [ "Jacobian (\\f$p \\times n_v\\f$)", "tasks_page.html#autotoc_md229", null ],
        [ "QP contribution", "tasks_page.html#autotoc_md230", null ],
        [ "Example", "tasks_page.html#autotoc_md231", null ]
      ] ],
      [ "JointCouplingTask", "tasks_page.html#jointcouplingtask", [
        [ "Formulation", "tasks_page.html#autotoc_md233", null ],
        [ "Use cases", "tasks_page.html#autotoc_md234", null ],
        [ "Example", "tasks_page.html#autotoc_md235", null ]
      ] ],
      [ "JointVelocityTask", "tasks_page.html#jointvelocitytask", [
        [ "Error (\\f$n_v\\f$-D)", "tasks_page.html#autotoc_md237", null ],
        [ "Jacobian (\\f$n_v \\times n_v\\f$)", "tasks_page.html#autotoc_md238", null ],
        [ "QP contribution", "tasks_page.html#autotoc_md239", null ],
        [ "Setters", "tasks_page.html#autotoc_md240", null ],
        [ "Example", "tasks_page.html#autotoc_md241", null ]
      ] ],
      [ "LowAccelerationTask", "tasks_page.html#lowaccelerationtask", [
        [ "Error (\\f$n_v\\f$-D)", "tasks_page.html#autotoc_md243", null ],
        [ "Jacobian (\\f$n_v \\times n_v\\f$)", "tasks_page.html#autotoc_md244", null ],
        [ "QP contribution", "tasks_page.html#autotoc_md245", null ],
        [ "Setters", "tasks_page.html#autotoc_md246", null ],
        [ "Example", "tasks_page.html#autotoc_md247", null ]
      ] ],
      [ "How tasks compose", "tasks_page.html#autotoc_md249", null ]
    ] ],
    [ "Limits", "limits_page.html", [
      [ "Limit overview", "limits_page.html#autotoc_md102", null ],
      [ "Common interface", "limits_page.html#autotoc_md104", null ],
      [ "VelocityLimit", "limits_page.html#velocitylimit", [
        [ "Formulation", "limits_page.html#autotoc_md106", null ],
        [ "How it works in code", "limits_page.html#autotoc_md107", null ],
        [ "Parameters", "limits_page.html#autotoc_md108", null ],
        [ "Key property", "limits_page.html#autotoc_md109", null ]
      ] ],
      [ "ConfigurationLimit", "limits_page.html#configurationlimit", [
        [ "Formulation", "limits_page.html#autotoc_md111", null ],
        [ "How the gain works", "limits_page.html#autotoc_md112", null ],
        [ "How it works in code", "limits_page.html#autotoc_md113", null ],
        [ "Parameters", "limits_page.html#autotoc_md114", null ]
      ] ],
      [ "FloatingBaseVelocityLimit", "limits_page.html#floatingbasevelocitylimit", [
        [ "Formulation", "limits_page.html#autotoc_md116", null ],
        [ "Requirements", "limits_page.html#autotoc_md117", null ],
        [ "How it works in code", "limits_page.html#autotoc_md118", null ],
        [ "Parameters", "limits_page.html#autotoc_md119", null ],
        [ "Example", "limits_page.html#autotoc_md120", null ]
      ] ],
      [ "AccelerationLimit", "limits_page.html#accelerationlimit", [
        [ "Formulation", "limits_page.html#autotoc_md122", null ],
        [ "Requirements", "limits_page.html#autotoc_md123", null ],
        [ "Parameters", "limits_page.html#autotoc_md124", null ],
        [ "Example", "limits_page.html#autotoc_md125", null ]
      ] ],
      [ "How limits stack", "limits_page.html#autotoc_md127", null ],
      [ "Planned: equality constraints", "limits_page.html#autotoc_md129", null ]
    ] ],
    [ "Barriers", "barriers_page.html", [
      [ "Barrier overview", "barriers_page.html#autotoc_md21", null ],
      [ "CBF theory", "barriers_page.html#cbf_theory", [
        [ "The CBF condition", "barriers_page.html#autotoc_md23", null ],
        [ "QP inequality form", "barriers_page.html#autotoc_md24", null ],
        [ "Safe-displacement objective", "barriers_page.html#autotoc_md25", null ]
      ] ],
      [ "Common parameters", "barriers_page.html#barrier_common_params", null ],
      [ "PositionBarrier", "barriers_page.html#positionbarrier", [
        [ "Barrier function", "barriers_page.html#autotoc_md28", null ],
        [ "Jacobian", "barriers_page.html#autotoc_md29", null ],
        [ "Use cases", "barriers_page.html#autotoc_md30", null ],
        [ "Parameters", "barriers_page.html#autotoc_md31", null ],
        [ "Example", "barriers_page.html#autotoc_md32", null ]
      ] ],
      [ "BodySphericalBarrier", "barriers_page.html#bodysphericalbarrier", [
        [ "Barrier function (1-D)", "barriers_page.html#autotoc_md34", null ],
        [ "Jacobian (1 x \\f$n_v\\f$)", "barriers_page.html#autotoc_md35", null ],
        [ "Gain function", "barriers_page.html#autotoc_md36", null ],
        [ "Parameters", "barriers_page.html#autotoc_md37", null ],
        [ "Example", "barriers_page.html#autotoc_md38", null ]
      ] ],
      [ "SelfCollisionBarrier", "barriers_page.html#selfcollisionbarrier", [
        [ "Barrier function (\\f$N\\f$-D)", "barriers_page.html#autotoc_md40", null ],
        [ "Jacobian (\\f$N \\times n_v\\f$)", "barriers_page.html#autotoc_md41", null ],
        [ "Requirements", "barriers_page.html#autotoc_md42", null ],
        [ "Parameters", "barriers_page.html#autotoc_md43", null ],
        [ "Example", "barriers_page.html#autotoc_md44", null ]
      ] ],
      [ "How barriers fit into the QP", "barriers_page.html#autotoc_md46", [
        [ "1. Inequality constraints (hard safety)", "barriers_page.html#autotoc_md47", null ],
        [ "2. Objective terms (active avoidance)", "barriers_page.html#autotoc_md48", null ]
      ] ],
      [ "Tuning barriers", "barriers_page.html#autotoc_md50", [
        [ "Gain too low (\\f$\\alpha \\ll 1\\f$)", "barriers_page.html#autotoc_md51", null ],
        [ "Gain too high (\\f$\\alpha \\gg 1\\f$)", "barriers_page.html#autotoc_md52", null ],
        [ "Safe displacement gain (\\f$\\kappa\\f$)", "barriers_page.html#autotoc_md53", null ],
        [ "Practical recommendations", "barriers_page.html#autotoc_md54", null ]
      ] ]
    ] ],
    [ "Extending Torq", "extending_page.html", [
      [ "Adding a custom Task", "extending_page.html#autotoc_md77", [
        [ "Step 1: Subclass <tt>torq::Task</tt>", "extending_page.html#autotoc_md78", null ],
        [ "Step 2: Choose constructor parameters", "extending_page.html#autotoc_md79", null ],
        [ "Step 3: Register with RobotSystem", "extending_page.html#autotoc_md80", null ],
        [ "Step 4: Update targets in the control loop (if needed)", "extending_page.html#autotoc_md81", null ],
        [ "Example: Elbow height task", "extending_page.html#autotoc_md82", null ]
      ] ],
      [ "Adding a custom Limit", "extending_page.html#autotoc_md84", [
        [ "Step 1: Subclass <tt>torq::Limit</tt>", "extending_page.html#autotoc_md85", null ],
        [ "Step 2: Implement the constraint", "extending_page.html#autotoc_md86", null ],
        [ "Step 3: Register", "extending_page.html#autotoc_md87", null ],
        [ "Guidelines", "extending_page.html#autotoc_md88", null ]
      ] ],
      [ "Adding a custom Barrier", "extending_page.html#autotoc_md90", [
        [ "Step 1: Subclass <tt>torq::Barrier</tt>", "extending_page.html#autotoc_md91", null ],
        [ "Step 2: Choose constructor parameters", "extending_page.html#autotoc_md92", null ],
        [ "Optional: Override <tt>computeSafeDisplacement()</tt>", "extending_page.html#autotoc_md93", null ],
        [ "Optional: Set a custom gain function", "extending_page.html#autotoc_md94", null ],
        [ "Step 3: Register", "extending_page.html#autotoc_md95", null ],
        [ "Example: Orientation barrier", "extending_page.html#autotoc_md96", null ]
      ] ],
      [ "Registration and ownership", "extending_page.html#autotoc_md98", null ],
      [ "Testing your extension", "extending_page.html#autotoc_md100", null ]
    ] ],
    [ "Parameter Tuning Guide", "tuning_guide.html", [
      [ "Quick reference", "tuning_guide.html#autotoc_md251", [
        [ "Built-in task parameters", "tuning_guide.html#autotoc_md252", null ],
        [ "Solver and limit parameters", "tuning_guide.html#autotoc_md253", null ],
        [ "Barrier parameters (set per-barrier on the object)", "tuning_guide.html#autotoc_md254", null ]
      ] ],
      [ "The IKConfig struct", "tuning_guide.html#autotoc_md256", null ],
      [ "Parameter details", "tuning_guide.html#autotoc_md258", [
        [ "Frame task costs", "tuning_guide.html#autotoc_md259", null ],
        [ "Frame gain (\\f$\\alpha\\f$)", "tuning_guide.html#autotoc_md260", null ],
        [ "Frame LM damping (\\f$\\mu\\f$)", "tuning_guide.html#autotoc_md261", null ],
        [ "Posture cost", "tuning_guide.html#autotoc_md262", null ],
        [ "Posture gain and LM damping", "tuning_guide.html#autotoc_md263", null ],
        [ "Damping cost", "tuning_guide.html#autotoc_md264", null ],
        [ "Solver (Tikhonov) damping (\\f$\\lambda\\f$)", "tuning_guide.html#autotoc_md265", null ],
        [ "Configuration limit gain (\\f$\\gamma\\f$)", "tuning_guide.html#autotoc_md266", null ]
      ] ],
      [ "Barrier tuning", "tuning_guide.html#autotoc_md268", [
        [ "Barrier gain (\\f$\\alpha\\f$)", "tuning_guide.html#autotoc_md269", null ],
        [ "Safe displacement gain (\\f$\\kappa\\f$)", "tuning_guide.html#autotoc_md270", null ],
        [ "Minimum distance (\\f$d_{\\min}\\f$)", "tuning_guide.html#autotoc_md271", null ]
      ] ],
      [ "Tuning recipes", "tuning_guide.html#autotoc_md273", [
        [ "Robot oscillates near target", "tuning_guide.html#autotoc_md274", null ],
        [ "Arm barely moves", "tuning_guide.html#autotoc_md275", null ],
        [ "Weird elbow configurations", "tuning_guide.html#autotoc_md276", null ],
        [ "Jerky motions", "tuning_guide.html#autotoc_md277", null ],
        [ "Solver reports infeasible", "tuning_guide.html#autotoc_md278", null ],
        [ "Robot stops before reaching target (barrier too aggressive)", "tuning_guide.html#autotoc_md279", null ]
      ] ],
      [ "GUI: IK Tuning panel", "tuning_guide.html#autotoc_md281", null ]
    ] ],
    [ "Sim-to-real with Torq", "sim_to_real.html", [
      [ "Architecture", "sim_to_real.html#autotoc_md185", null ],
      [ "Real robot: ServoDriver (ST3215 / ST/STS/SMS)", "sim_to_real.html#autotoc_md186", [
        [ "Connection", "sim_to_real.html#autotoc_md187", null ],
        [ "SO101 real robot example", "sim_to_real.html#autotoc_md188", null ],
        [ "GUI with real robot (display mirror)", "sim_to_real.html#autotoc_md189", null ],
        [ "Passive mode (display only, move by hand)", "sim_to_real.html#autotoc_md190", null ],
        [ "Calibration and joint/velocity limits", "sim_to_real.html#autotoc_md191", null ],
        [ "ST/STS/SMS protocol (SCSPort + SMS_STS_Port)", "sim_to_real.html#autotoc_md192", null ]
      ] ],
      [ "Limits and barriers for real hardware", "sim_to_real.html#autotoc_md193", null ],
      [ "Checklist for real robot", "sim_to_real.html#autotoc_md194", null ]
    ] ],
    [ "Namespaces", "namespaces.html", [
      [ "Namespace List", "namespaces.html", "namespaces_dup" ],
      [ "Namespace Members", "namespacemembers.html", [
        [ "All", "namespacemembers.html", null ],
        [ "Functions", "namespacemembers_func.html", null ],
        [ "Variables", "namespacemembers_vars.html", null ],
        [ "Typedefs", "namespacemembers_type.html", null ],
        [ "Enumerations", "namespacemembers_enum.html", null ]
      ] ]
    ] ],
    [ "Classes", "annotated.html", [
      [ "Class List", "annotated.html", "annotated_dup" ],
      [ "Class Index", "classes.html", null ],
      [ "Class Hierarchy", "hierarchy.html", "hierarchy" ],
      [ "Class Members", "functions.html", [
        [ "All", "functions.html", "functions_dup" ],
        [ "Functions", "functions_func.html", "functions_func" ],
        [ "Variables", "functions_vars.html", null ]
      ] ]
    ] ],
    [ "Files", "files.html", [
      [ "File List", "files.html", "files_dup" ],
      [ "File Members", "globals.html", [
        [ "All", "globals.html", null ],
        [ "Typedefs", "globals_type.html", null ],
        [ "Macros", "globals_defs.html", null ]
      ] ]
    ] ]
  ] ]
];

var NAVTREEINDEX =
[
"Barriers_8hpp.html",
"classtorq_1_1InverseKinematics.html#af01b34934313ab1b1a3c3f56135d47ab",
"functions_f.html",
"tasks_page.html#jointcouplingtask"
];

var SYNCONMSG = 'click to disable panel synchronisation';
var SYNCOFFMSG = 'click to enable panel synchronisation';