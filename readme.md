Rapier-Macroquad-Interface
==========================

Have you ever tried to use Rapier in Macroquad and thought "there has to be a better way"?
Wouldn't it it be nice if you could step the simulation without passing in 13 parameters?
Wouldn't it be nice to have a function to show your simulation in camera space for debugging?
Wouldn't it be nice to have example code?

Well now you can!

Features:
---------

* `physics.rs` single file containing all library functions (See note)
* `PhysicsSimulation` struct to keep all of your important state in one place
* `draw_debug` function to see what's going on
* `get_bodies_at_point` function to sample a point in a single line of code
* `step` function that only takes one f32 argument
* `load_svg_physics_sprite` No more counting pixels to outline your sprites.
  Just open the image in inkscape, draw an outline, and save with the image
  embedded. This function will load the texture, and use all of the paths it
  finds as the physical outline.

In general, `physics.rs` attempts to make simple things simple to do, while
presenting no obstruction to doing more complex things.
Rapier2D is confusing. If nothing else, this can work as a reference for how
to do a few basic things.

Note: When rendering, this library matches screen units to camera units, and
if the y axis of the camera is down, rapier's rotation will be inverted
(because it uses a coordinate system with the y axis up).
