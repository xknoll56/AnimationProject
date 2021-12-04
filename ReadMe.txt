Xavier Knoll - 40132134

To build the project, simply open the .pro file up with qtcreator and build, after building, 
place the all the shaders from AnimationProject/Shaders into the build folder and run. I have provided
the standalone exacutables, the windows version requires a few dependencies also included, the
linux version seems to work for me on any of the lab computers without any issues.

Controls:

-The project will open up with the command line interface open. Press escape to close and start
the scene.

-W and S keys will move the camera in the forward/back direcitons.

-A and D keys will move the camera in the right/left directions.

-left clicking on the screen and then moving the mouse will rotate the camera.

-Right clicking on a rigid body in specific scenes will select a specifice rigid body for modification
or reading values in the command line.

Commands:

"set scene value": will set the scene to a specific value. There are 3 main values for the demo such as:
demo, collision, and vaccume.

"get position": will return the x, y and z positions in that order.

"get rotation": will return the w, x, y, and z quaternion rotations values in that order.

"get velocity": will return the x, y and z velocity values in that order.

"get angular_velocity": will return the x, y and z components of the angular velocity in that order.

"get mass": returns the mass.

"get inertia": returns the inertia.

"set position x y z": sets position to x in the x-coordinate, y in the y-coordinate and z in the z-coordinate.

"set euler x y z": sets euler angle rotation to x in the x-coordinate, y in the y-coordinate and z in the z-coordinate.

"set velocity x y z": sets velocity to x in the x-coordinate, y in the y-coordinate and z in the z-coordinate.

"set angular_velocity x y z": sets the angluar velocity to x in the x-coordinate, y in the y-coordinate and z in the z-coordinate.