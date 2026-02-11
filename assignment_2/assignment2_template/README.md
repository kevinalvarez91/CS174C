# CS 174C Assignment 2: Inverse Kinematics

This project implements an Inverse Kinematics (IK) engine where an articulated human character draws a figure-8 spline on a blackboard.

## Implementation Details

### 1. Classroom Scene
**Requirement:** Display a "classroom" with a floor plane and a blackboard on a planar wall.
* **Implementation:** Inside `Assignment2.render_animation`.
* **Location:**
    * **Floor:** Drawn using `shapes.box` with yellow plastic material (Lines 398-399).
    * **Wall:** Drawn behind the blackboard (Lines 438-439).
    * **Blackboard:** Drawn as a dark gray box (Lines 440-441).

### 2. Spline Display
**Requirement:** Display the spline on the blackboard.
* **Implementation:** The spline is generated using the `HermiteSpline` class and drawn as a `Polyline`.
* **Location:**
    * **Class Definition:** `HermiteSpline` class defines the math for cubic Hermite interpolation (Lines 259-318).
    * **Setup:** The Control Points and Tangents for the "Figure-8" shape are defined in `init()` (Lines 358-372).
    * **Drawing:** The spline is sampled into a `Polyline` strip and drawn in `render_animation` (Lines 403-409).

### 3. Human Character Model
**Requirement:** Model the human character using ellipsoids (Spheres with non-uniform scaling).
* **Implementation:** The `Articulated_Human` class constructs a hierarchy of nodes connected by arcs.
* **Location:**
    * **Geometry:** Uses `shapes.sphere` (Line 10, 15).
    * **Hierarchy Construction:** The `constructor` builds the torso, head, arms, and legs (Lines 14-142).
    * **Ellipsoids:** Non-uniform scaling is applied to spheres to create limbs (e.g., `Mat4.scale(1.2, .2, .2)` for the upper arm at Line 39).
    * **Drawing:** The `_rec_draw` method traverses the hierarchy to render the character (Lines 149-169).

### 4. Inverse Kinematics Solver
**Requirement:** Implement the inverse kinematics solver using the pseudoinverse approach.
* **Implementation:** The solver uses a Jacobian-based approach with Damped Least Squares (a robust form of pseudoinverse).
* **Location:**
    * **Degrees of Freedom:** The right arm is defined with 7 DOFs (Shoulder: 3, Elbow: 2, Wrist: 2) inside `solve_ik` (Lines 185-192).
    * **Jacobian Computation:** The 3xN Jacobian matrix `J` is built by computing the cross product of rotation axes and the distance vector to the end effector (Lines 199-215).
    * **Pseudoinverse/Linear Solve:** The function `solve_linear_system` computes `dtheta` using the Damped Least Squares formula `J^T * (J * J^T + lambda^2 * I)^-1 * dx` (Lines 333-376).
    * **Integration:** The calculated angular velocities (`dtheta`) are applied to the `articulation_matrix` of the shoulder, elbow, and wrist joints (Lines 220-239).

### 5. Animation Phase 1: Reaching
**Requirement:** Move the right hand from its initial position to touch the board.
* **Implementation:** Logic checks a boolean flag `hand_reached_board`.
* **Location:**
    * Inside `render_animation` (Lines 415-422).
    * The IK solver targets `initial_target` (the start of the spline). Once the distance is < 0.1, the state switches to the drawing phase.

### 6. Animation Phase 2: Drawing
**Requirement:** Move the hand to draw the spline, looping repeatedly.
* **Implementation:** The target position moves along the spline curve over time.
* **Location:**
    * Inside `render_animation` (Lines 423-435).
    * **Spline Evaluation:** `this.spline_u` is incremented by `dt`, and `spline.evaluate(u)` calculates the specific 3D coordinate on the curve.
    * **Stability:** The IK solver runs multiple iterations (10 times) per frame to ensure the hand tracks the moving target tightly (Lines 430-434).