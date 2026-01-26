# Assignment 1 – Implementation Notes (Grader Guide)

**File:** `assign_one_hermite.js`

This file contains all required functionality for the assignment. Below are the **non-obvious parts** and where to find them.

---

## Core spline implementation
- Implemented a full **cubic Hermite spline** in `class HermiteSpline`.
- Global parameterization `t := [0,1]` is handled in `evaluate(t)` by:
  - mapping `t` to a segment index
  - converting to local `u := [0,1]`
  - scaling tangents correctly so interpolation is consistent across segments

See `HermiteSpline.evaluate(t)`

---

## Drawing the spline
- The spline is sampled and drawn **only when the Draw button is pressed**.
- Sampling is done in `update_scene()` using `this.spline.sample(40)`.
- The curve is rendered as a continuous **LINE_STRIP** via a custom `Polyline` shape.

See `update_scene()` and `class Polyline`

---

## Command parsing (text input)
- All text-based commands are handled in `parse_commands()`.
- Supports:
  - adding control points
  - editing points and tangents
  - computing arc length and printing a lookup table
- Errors are caught and reported in the output textbox.

See `parse_commands()`

---

## Arc length parameterization
- Arc length is approximated using piecewise linear sampling.
- A lookup table mapping `s -> t` is generated and printed.
- Used only for reporting (not for motion).

See `HermiteSpline.arc_length_table()`  
Triggered by `get_arc_length` inside `parse_commands()`

---

## Load / Export
- Implemented a simple text format loader/exporter for splines.
- Used to save and restore control points and tangents.

See:
- `HermiteSpline.load_from_string()`
- `HermiteSpline.export_to_string()`

---

## Preset shapes
- **Straight!** creates a straight Hermite spline with consistent tangents.
- **Circle!** creates a smooth closed loop using many control points and correctly scaled tangents.
- Both presets immediately redraw the spline.

See `preset_straight()` and `preset_circle()`

---

## IMPORTANT NOTE 
To avoid unintended interpolation artifacts:
> **Please reload the page before testing a new shape or input set.**

Because the spline state persists across button presses, mixing presets and manual commands without reloading can lead to confusing intermediate results. A fresh reload ensures clean, expected behavior.

---

## How to quickly test
1. Reload page
2. Click **Straight!** or **Circle!**
3. Click **Draw**
4. **Or** Export -> Reload -> Load -> Draw
