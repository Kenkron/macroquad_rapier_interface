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

In general, `physics.rs` attempts to make simple things simple to do, while
presenting no obstruction to doing more complex things.
Rapier2D is confusing. If nothing else, this can work as a reference for how
to do a few basic things.

Note: I'm currently going through dependency hell in nodejs, so until the
butthurt subsides, I'm going to leave the interface as a single file instead
of a proper library.