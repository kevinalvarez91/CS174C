# Assignment 3: Mass-Spring-Damper Systems and Chains

This project implements a physical simulation framework using **tiny-graphics.js**. [cite_start]It consists of a scriptable mass-spring-damper simulator and a secondary simulation of a viscoelastic chain driven by a Hermite spline[cite: 10, 63].

---

## Part 1: Mass-Spring-Damper System
**File:** `part_one_spring.js` [cite: 11]

This component allows for the creation and simulation of arbitrary particle-spring networks via a text-based interface[cite: 23, 12].

### Implementation Details
* **Particle System & Spring Classes**: Defined in lines 5–22 to store mass, kinematic state, and spring constants[cite: 56].
* **Script-based Commands**: The `parse_commands()` function (Lines 173–221) implements all 8 required commands: `create particles`, `particle`, `all_velocities`, `create springs`, `link`, `integration`, `ground`, and `gravity`.
* **Integration Methods**:
    * **Forward Euler**: (Lines 95–97) Explicit update of position and velocity[cite: 58].
    * **Symplectic Euler**: (Lines 98–100) Semi-implicit update for improved energy stability[cite: 59].
    * **Verlet**: (Lines 101–105) Position-based integration using previous states to maintain momentum[cite: 60].
* **Forces**:
    * **Gravity**: Applied as a constant downward acceleration (Line 44)[cite: 24, 61].
    * **Spring Forces**: Damped spring-damper forces calculated based on Hooke's Law and linear damping (Lines 47–61)[cite: 23, 44].
    * **Ground Collision**: Implemented using a penalty method on the $xz$-plane ($y=0$)[cite: 25, 61]. The normal force is calculated as:
      $$f_n = \max(0, k_{s\_ground} \cdot \text{penetration} - k_{d\_ground} \cdot v_n)$$
    * **Friction**: Added basic Coulomb friction (Lines 74–88) to prevent particles from sliding infinitely on the floor.
* **Drawing**:
    * **Particles**: Rendered as blue spheres (Lines 148–151)[cite: 62].
    * **Springs**: Rendered as red lines using a dynamic `Polyline` shape (Lines 154–163)[cite: 62].

---

## Part 2: Viscoelastic Chain
**File:** `part_two_chain.js` [cite: 65]

This part simulates a chain of 8 particles where the top particle follows a predefined 3D Hermite spline[cite: 80, 71].

### Implementation Details
* **Hermite Spline**: (Lines 28–64) A custom class that interpolates positions and tangents using Hermite basis functions[cite: 93].
* **Trajectory**: The spline is hardcoded with **4 control points** and specific tangents to create an interesting 3D path (Lines 159–163)[cite: 77, 78].
* **Sinusoidal Movement**: The top particle (index 0) is driven along the spline. The parameter $t$ varies sinusoidally over time: `0.5 + 0.5 * Math.sin(this.t)` (Lines 185–186)[cite: 71, 79, 76].
* **Chain Physics**:
    * The chain consists of **8 particles** (including the top one) linked by springs with $k_s = 500$ and $k_d = 15$ (Lines 149–156)[cite: 80, 70].
    * The physics are updated using the system from Part 1, modified to skip the integration of the "driven" top particle[cite: 69, 72].
* **Ground Interaction**: The spline path and chain length are configured so that the bottom particle collides with the yellow ground plane during its cycle[cite: 84].
* **Drawing**:
    * **Spline**: Displayed as a gray path (Line 182)[cite: 76].
    * **Chain**: Particles are blue spheres and springs are rendered as a continuous line strip (Lines 191–208)[cite: 80, 85].

---

## How to Run

1.  **Part 1**:
    * Enter configuration commands into the **Input** text box.
    * Click the **Config** button to apply the setup[cite: 27].
    * Click the **Run** button to start the simulation[cite: 27].
2.  **Part 2**:
    *  The chain simulation runs automatically upon loading the scene; no manual input is required[cite: 74].