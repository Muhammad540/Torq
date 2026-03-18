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
    [ "Quick start", "index.html#autotoc_md47", null ],
    [ "Documentation topics", "index.html#autotoc_md48", null ],
    [ "Key design choices", "index.html#autotoc_md49", null ],
    [ "Architecture", "architecture.html", [
      [ "System diagram", "architecture.html#autotoc_md5", null ],
      [ "Public API and ownership", "architecture.html#autotoc_md6", [
        [ "Ownership hierarchy", "architecture.html#autotoc_md7", null ]
      ] ],
      [ "Class hierarchy", "architecture.html#autotoc_md8", [
        [ "Core", "architecture.html#autotoc_md9", null ],
        [ "Kinematics", "architecture.html#autotoc_md10", null ],
        [ "Tasks (QP objective)", "architecture.html#autotoc_md11", null ],
        [ "Limits (QP inequality constraints)", "architecture.html#autotoc_md12", null ],
        [ "Barriers (CBF inequality + optional objective)", "architecture.html#autotoc_md13", null ],
        [ "Hardware abstraction", "architecture.html#autotoc_md14", null ],
        [ "GUI", "architecture.html#autotoc_md15", null ]
      ] ],
      [ "Ownership diagram", "architecture.html#autotoc_md16", null ],
      [ "Control loop frequencies", "architecture.html#autotoc_md17", null ],
      [ "Data flow per tick", "architecture.html#autotoc_md18", null ],
      [ "Adding a new robot", "architecture.html#autotoc_md19", null ]
    ] ],
    [ "QP Formulation", "qp_formulation.html", [
      [ "Overview", "qp_formulation.html#autotoc_md50", null ],
      [ "Per-task contribution", "qp_formulation.html#autotoc_md52", [
        [ "Weight matrix", "qp_formulation.html#autotoc_md53", null ],
        [ "Weighted quantities", "qp_formulation.html#autotoc_md54", null ],
        [ "Levenberg–Marquardt damping", "qp_formulation.html#autotoc_md55", null ],
        [ "Per-task Hessian and gradient", "qp_formulation.html#autotoc_md56", null ]
      ] ],
      [ "Composing multiple tasks", "qp_formulation.html#autotoc_md58", null ],
      [ "Tikhonov (solver) damping", "qp_formulation.html#autotoc_md60", null ],
      [ "Inequality constraints from limits", "qp_formulation.html#autotoc_md62", null ],
      [ "The complete QP", "qp_formulation.html#autotoc_md64", null ],
      [ "OSQP solver", "qp_formulation.html#autotoc_md66", null ],
      [ "Manifold-aware integration", "qp_formulation.html#autotoc_md68", null ]
    ] ],
    [ "Tasks", "tasks_page.html", [
      [ "Common parameters", "tasks_page.html#autotoc_md80", null ],
      [ "FrameTask", "tasks_page.html#autotoc_md82", [
        [ "Error (6-D)", "tasks_page.html#autotoc_md83", null ],
        [ "Jacobian (6×n_v)", "tasks_page.html#autotoc_md84", null ],
        [ "Weight vector", "tasks_page.html#autotoc_md85", null ],
        [ "QP contribution", "tasks_page.html#autotoc_md86", null ],
        [ "Setters", "tasks_page.html#autotoc_md87", null ]
      ] ],
      [ "PostureTask", "tasks_page.html#autotoc_md89", [
        [ "Error (n_v-D)", "tasks_page.html#autotoc_md90", null ],
        [ "Jacobian (n_v×n_v)", "tasks_page.html#autotoc_md91", null ],
        [ "Weight", "tasks_page.html#autotoc_md92", null ],
        [ "QP contribution", "tasks_page.html#autotoc_md93", null ]
      ] ],
      [ "DampingTask", "tasks_page.html#autotoc_md95", [
        [ "Error", "tasks_page.html#autotoc_md96", null ],
        [ "Jacobian", "tasks_page.html#autotoc_md97", null ],
        [ "QP contribution", "tasks_page.html#autotoc_md98", null ]
      ] ],
      [ "How tasks compose", "tasks_page.html#autotoc_md100", null ]
    ] ],
    [ "Limits", "limits_page.html", [
      [ "VelocityLimit", "limits_page.html#autotoc_md34", [
        [ "Formulation", "limits_page.html#autotoc_md35", null ],
        [ "Parameters", "limits_page.html#autotoc_md36", null ]
      ] ],
      [ "ConfigurationLimit", "limits_page.html#autotoc_md38", [
        [ "Formulation", "limits_page.html#autotoc_md39", null ],
        [ "Effect of <tt>config_limit_gain</tt>", "limits_page.html#autotoc_md40", null ],
        [ "Parameters", "limits_page.html#autotoc_md41", null ]
      ] ],
      [ "How limits stack", "limits_page.html#autotoc_md43", null ],
      [ "Planned: equality constraints", "limits_page.html#autotoc_md45", null ]
    ] ],
    [ "Barriers", "barriers_page.html", [
      [ "</blockquote>", "barriers_page.html#autotoc_md20", null ],
      [ "Control Barrier Functions (CBFs)", "barriers_page.html#autotoc_md21", [
        [ "CBF inequality", "barriers_page.html#autotoc_md22", null ],
        [ "Safe displacement objective", "barriers_page.html#autotoc_md23", null ]
      ] ],
      [ "Planned barrier types", "barriers_page.html#autotoc_md25", [
        [ "PositionBarrier", "barriers_page.html#autotoc_md26", null ],
        [ "BodySphericalBarrier", "barriers_page.html#autotoc_md27", null ],
        [ "SelfCollisionBarrier", "barriers_page.html#autotoc_md28", null ]
      ] ],
      [ "How barriers fit into the QP", "barriers_page.html#autotoc_md30", null ],
      [ "Common barrier parameters", "barriers_page.html#autotoc_md32", null ]
    ] ],
    [ "Parameter Tuning Guide", "tuning_guide.html", [
      [ "Quick reference", "tuning_guide.html#autotoc_md102", null ],
      [ "The IKConfig struct", "tuning_guide.html#autotoc_md104", null ],
      [ "Parameter details", "tuning_guide.html#autotoc_md106", [
        [ "Frame task costs", "tuning_guide.html#autotoc_md107", null ],
        [ "Frame gain (α)", "tuning_guide.html#autotoc_md108", null ],
        [ "Frame LM damping (μ)", "tuning_guide.html#autotoc_md109", null ],
        [ "Posture cost", "tuning_guide.html#autotoc_md110", null ],
        [ "Posture gain and LM damping", "tuning_guide.html#autotoc_md111", null ],
        [ "Damping cost", "tuning_guide.html#autotoc_md112", null ],
        [ "Solver (Tikhonov) damping (λ)", "tuning_guide.html#autotoc_md113", null ],
        [ "Configuration limit gain (γ)", "tuning_guide.html#autotoc_md114", null ]
      ] ],
      [ "Tuning recipes", "tuning_guide.html#autotoc_md116", [
        [ "Robot oscillates near target", "tuning_guide.html#autotoc_md117", null ],
        [ "Arm barely moves", "tuning_guide.html#autotoc_md118", null ],
        [ "Weird elbow configurations", "tuning_guide.html#autotoc_md119", null ],
        [ "Jerky motions", "tuning_guide.html#autotoc_md120", null ],
        [ "Solver reports infeasible", "tuning_guide.html#autotoc_md121", null ]
      ] ],
      [ "GUI: IK Tuning panel", "tuning_guide.html#autotoc_md123", null ]
    ] ],
    [ "Sim-to-real with Torq", "sim_to_real.html", [
      [ "Architecture", "sim_to_real.html#autotoc_md69", null ],
      [ "Real robot: ServoDriver (ST3215 / ST/STS/SMS)", "sim_to_real.html#autotoc_md70", [
        [ "Connection", "sim_to_real.html#autotoc_md71", null ],
        [ "SO101 real robot example", "sim_to_real.html#autotoc_md72", null ],
        [ "GUI with real robot (display mirror)", "sim_to_real.html#autotoc_md73", null ],
        [ "Passive mode (display only, move by hand)", "sim_to_real.html#autotoc_md74", null ],
        [ "Calibration and joint/velocity limits", "sim_to_real.html#autotoc_md75", null ],
        [ "ST/STS/SMS protocol (SCSPort + SMS_STS_Port)", "sim_to_real.html#autotoc_md76", null ]
      ] ],
      [ "Limits and barriers for real hardware", "sim_to_real.html#autotoc_md77", null ],
      [ "Checklist for real robot", "sim_to_real.html#autotoc_md78", null ]
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
"classtorq_1_1KinematicsEngine.html#aec6d8aca289a3f11e0dba2f2bab4c43d",
"limits_page.html#autotoc_md39"
];

var SYNCONMSG = 'click to disable panel synchronisation';
var SYNCOFFMSG = 'click to enable panel synchronisation';