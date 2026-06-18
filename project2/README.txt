Programming Assignment #2 - Cow Roller Coaster

Files Made or Changed
[Changed]
SimpleScene.py
Implemented the user interface for specifying six control points.
Implemented horizontal cow movement using mouse movement.
Implemented vertical cow movement using left-button dragging.
Implemented a cyclic uniform cubic B-spline.
Implemented the three-lap cow animation.
Implemented yaw and pitch orientation along the spline trajectory.
Implemented the reset behavior after the animation finishes.
[Added]
README.txt
Describes the modified files, execution method, controls, and implementation.
[Provided Files Used Without Modification]
OBJ.py
Ray.py
cow.obj
camera.obj
bricks.bmp

Requirements
The program requires the following Python packages:
Python 3
NumPy
GLFW
PyOpenGL
Pillow
Example installation command:
pip install numpy glfw PyOpenGL Pillow

How to Run
Place all source codes, OBJ files, and the texture image in the same directory.
Run the program with:
python SimpleScene.py

User Controls
[Camera Control]
Press the Space key or C key to switch the camera viewpoint.
[Selecting the Cow]
Move the mouse cursor over the cow.
A white bounding box appears when the cursor is on the cow.
Left-click the cow to start control-point specification.
[Horizontal Positioning]
After selecting the cow, move the mouse without pressing a button.
The cow follows the mouse cursor horizontally while maintaining its current height.
[Vertical Positioning]
Press and hold the left mouse button.
Drag the mouse upward or downward to adjust the cow's height.
Release the left mouse button to fix the current control point.
[Control-Point Specification]
Six control points must be specified.
Whenever a control point is fixed, a duplicated cow remains at that position.
After the sixth control point is fixed, the animation starts automatically.

Cyclic B-Spline Implementation
The trajectory is generated using a cyclic uniform cubic B-spline.
Each spline segment is evaluated using four neighboring control points:
Previous control point
Current control point
Next control point
Second next control point
The control-point indices are calculated using modulo operations. Therefore, the last control points are connected smoothly to the first control points, forming a cyclic trajectory.
The cubic B-spline is an approximating spline. Therefore, the generated curve does not necessarily pass directly through every control point.
The position of one spline segment is calculated as:
P(u) =
(
(-u^3 + 3u^2 - 3u + 1)P(i-1)
+ (3u^3 - 6u^2 + 4)P(i)
+ (-3u^3 + 3u^2 + 3u + 1)P(i+1)
+ u^3 P(i+2)
) / 6
where 0 <= u < 1.

Animation
The animation time is calculated using glfw.get_time().
The current spline segment and local parameter u are determined from the elapsed time.
The cow follows the complete cyclic B-spline trajectory three times.
After completing three laps:
The cow stops at the final animation position.
The cow's direction is restored to the initial direction.
The control points are cleared.
The cow does not immediately follow the mouse.
The user must click the cow again before specifying a new trajectory.

Cow Orientation
The derivative of the cubic B-spline is used as the tangent vector of the trajectory.
[Yaw Orientation]
The horizontal XZ component of the tangent vector is used.
The yaw angle is calculated so that the cow faces the forward direction of movement.
The cow turns left or right according to the direction of the trajectory.
[Pitch Orientation]
The vertical component of the tangent vector is used.
The pitch angle is calculated from the vertical direction and horizontal tangent length.
The cow faces upward while moving upward.
The cow faces downward while moving downward.
The translation, yaw rotation, and pitch rotation are combined into the cow-to-world transformation matrix.

Main Implemented Functions
cubicBSplinePoint()
Calculates a position on a cubic B-spline segment.
cubicBSplineTangent()
Calculates the derivative and movement direction of the spline.
splinePositionAndTangent()
Evaluates a cyclic spline segment using modulo-indexed control points.
makeCowTransform()
Creates the cow transformation matrix using its position, yaw, and pitch.
startVerticalDrag()
Creates a screen-facing dragging plane for vertical movement.
setHorizontalPickAnchor()
Sets the reference point used for horizontal mouse movement.
onMouseButton()
Handles cow selection, vertical dragging, and control-point registration.
onMouseDrag()
Handles horizontal positioning, vertical positioning, and cow picking.
display()
Updates and displays the three-lap spline animation.