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
    [ "Quick start", "index.html#autotoc_md42", null ],
    [ "Documentation topics", "index.html#autotoc_md43", null ],
    [ "Key design choices", "index.html#autotoc_md44", null ],
    [ "Architecture", "architecture.html", [
      [ "System diagram", "architecture.html#autotoc_md0", null ],
      [ "Public API and ownership", "architecture.html#autotoc_md1", [
        [ "Ownership hierarchy", "architecture.html#autotoc_md2", null ]
      ] ],
      [ "Class hierarchy", "architecture.html#autotoc_md3", [
        [ "Core", "architecture.html#autotoc_md4", null ],
        [ "Kinematics", "architecture.html#autotoc_md5", null ],
        [ "Tasks (QP objective)", "architecture.html#autotoc_md6", null ],
        [ "Limits (QP inequality constraints)", "architecture.html#autotoc_md7", null ],
        [ "Barriers (CBF inequality + optional objective)", "architecture.html#autotoc_md8", null ],
        [ "Hardware abstraction", "architecture.html#autotoc_md9", null ],
        [ "GUI", "architecture.html#autotoc_md10", null ]
      ] ],
      [ "Ownership diagram", "architecture.html#autotoc_md11", null ],
      [ "Control loop frequencies", "architecture.html#autotoc_md12", null ],
      [ "Data flow per tick", "architecture.html#autotoc_md13", null ],
      [ "Adding a new robot", "architecture.html#autotoc_md14", null ]
    ] ],
    [ "QP Formulation", "qp_formulation.html", [
      [ "Overview", "qp_formulation.html#autotoc_md45", null ],
      [ "Per-task contribution", "qp_formulation.html#autotoc_md47", [
        [ "Weight matrix", "qp_formulation.html#autotoc_md48", null ],
        [ "Weighted quantities", "qp_formulation.html#autotoc_md49", null ],
        [ "Levenberg–Marquardt damping", "qp_formulation.html#autotoc_md50", null ],
        [ "Per-task Hessian and gradient", "qp_formulation.html#autotoc_md51", null ]
      ] ],
      [ "Composing multiple tasks", "qp_formulation.html#autotoc_md53", null ],
      [ "Tikhonov (solver) damping", "qp_formulation.html#autotoc_md55", null ],
      [ "Inequality constraints from limits", "qp_formulation.html#autotoc_md57", null ],
      [ "The complete QP", "qp_formulation.html#autotoc_md59", null ],
      [ "OSQP solver", "qp_formulation.html#autotoc_md61", null ],
      [ "Manifold-aware integration", "qp_formulation.html#autotoc_md63", null ]
    ] ],
    [ "Tasks", "tasks_page.html", [
      [ "Common parameters", "tasks_page.html#autotoc_md65", null ],
      [ "FrameTask", "tasks_page.html#autotoc_md67", [
        [ "Error (6-D)", "tasks_page.html#autotoc_md68", null ],
        [ "Jacobian (6×n_v)", "tasks_page.html#autotoc_md69", null ],
        [ "Weight vector", "tasks_page.html#autotoc_md70", null ],
        [ "QP contribution", "tasks_page.html#autotoc_md71", null ],
        [ "Setters", "tasks_page.html#autotoc_md72", null ]
      ] ],
      [ "PostureTask", "tasks_page.html#autotoc_md74", [
        [ "Error (n_v-D)", "tasks_page.html#autotoc_md75", null ],
        [ "Jacobian (n_v×n_v)", "tasks_page.html#autotoc_md76", null ],
        [ "Weight", "tasks_page.html#autotoc_md77", null ],
        [ "QP contribution", "tasks_page.html#autotoc_md78", null ]
      ] ],
      [ "DampingTask", "tasks_page.html#autotoc_md80", [
        [ "Error", "tasks_page.html#autotoc_md81", null ],
        [ "Jacobian", "tasks_page.html#autotoc_md82", null ],
        [ "QP contribution", "tasks_page.html#autotoc_md83", null ]
      ] ],
      [ "How tasks compose", "tasks_page.html#autotoc_md85", null ]
    ] ],
    [ "Limits", "limits_page.html", [
      [ "VelocityLimit", "limits_page.html#autotoc_md29", [
        [ "Formulation", "limits_page.html#autotoc_md30", null ],
        [ "Parameters", "limits_page.html#autotoc_md31", null ]
      ] ],
      [ "ConfigurationLimit", "limits_page.html#autotoc_md33", [
        [ "Formulation", "limits_page.html#autotoc_md34", null ],
        [ "Effect of <tt>config_limit_gain</tt>", "limits_page.html#autotoc_md35", null ],
        [ "Parameters", "limits_page.html#autotoc_md36", null ]
      ] ],
      [ "How limits stack", "limits_page.html#autotoc_md38", null ],
      [ "Planned: equality constraints", "limits_page.html#autotoc_md40", null ]
    ] ],
    [ "Barriers", "barriers_page.html", [
      [ "</blockquote>", "barriers_page.html#autotoc_md15", null ],
      [ "Control Barrier Functions (CBFs)", "barriers_page.html#autotoc_md16", [
        [ "CBF inequality", "barriers_page.html#autotoc_md17", null ],
        [ "Safe displacement objective", "barriers_page.html#autotoc_md18", null ]
      ] ],
      [ "Planned barrier types", "barriers_page.html#autotoc_md20", [
        [ "PositionBarrier", "barriers_page.html#autotoc_md21", null ],
        [ "BodySphericalBarrier", "barriers_page.html#autotoc_md22", null ],
        [ "SelfCollisionBarrier", "barriers_page.html#autotoc_md23", null ]
      ] ],
      [ "How barriers fit into the QP", "barriers_page.html#autotoc_md25", null ],
      [ "Common barrier parameters", "barriers_page.html#autotoc_md27", null ]
    ] ],
    [ "Parameter Tuning Guide", "tuning_guide.html", [
      [ "Quick reference", "tuning_guide.html#autotoc_md87", null ],
      [ "The IKConfig struct", "tuning_guide.html#autotoc_md89", null ],
      [ "Parameter details", "tuning_guide.html#autotoc_md91", [
        [ "Frame task costs", "tuning_guide.html#autotoc_md92", null ],
        [ "Frame gain (α)", "tuning_guide.html#autotoc_md93", null ],
        [ "Frame LM damping (μ)", "tuning_guide.html#autotoc_md94", null ],
        [ "Posture cost", "tuning_guide.html#autotoc_md95", null ],
        [ "Posture gain and LM damping", "tuning_guide.html#autotoc_md96", null ],
        [ "Damping cost", "tuning_guide.html#autotoc_md97", null ],
        [ "Solver (Tikhonov) damping (λ)", "tuning_guide.html#autotoc_md98", null ],
        [ "Configuration limit gain (γ)", "tuning_guide.html#autotoc_md99", null ]
      ] ],
      [ "Tuning recipes", "tuning_guide.html#autotoc_md101", [
        [ "Robot oscillates near target", "tuning_guide.html#autotoc_md102", null ],
        [ "Arm barely moves", "tuning_guide.html#autotoc_md103", null ],
        [ "Weird elbow configurations", "tuning_guide.html#autotoc_md104", null ],
        [ "Jerky motions", "tuning_guide.html#autotoc_md105", null ],
        [ "Solver reports infeasible", "tuning_guide.html#autotoc_md106", null ]
      ] ],
      [ "GUI: IK Tuning panel", "tuning_guide.html#autotoc_md108", null ]
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
        [ "Typedefs", "globals_type.html", null ]
      ] ]
    ] ]
  ] ]
];

var NAVTREEINDEX =
[
"Barriers_8hpp.html",
"classtorq_1_1MujocoDriver.html#a9e977dbabcec3afa0e1c4fe617121562",
"tasks_page.html#autotoc_md68"
];

var SYNCONMSG = 'click to disable panel synchronisation';
var SYNCOFFMSG = 'click to enable panel synchronisation';