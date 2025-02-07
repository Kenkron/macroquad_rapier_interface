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

Simple Example
==============

```
use macroquad::prelude::*;
use rapier2d::prelude::*;
extern crate macroquad_rapier_interface as physics;
use physics::*;

#[macroquad::main("physics")]
async fn main() {
    let brick_texture = load_texture("assets/brick_texture.png").await.unwrap();
    let mut simulation = PhysicsSimulation::new(vec2(0.0, 980.0));

    // Add sloped floor
    let shape = vec![[0., 400.], [800., 500.], [800., 600.], [0., 600.]];
    simulation.create_body(&RigidBodyBuilder::fixed(), &[polygon_collider(&shape)]);

    // Make block
    let block = PhysicsSprite{
        texture: brick_texture,
        body: simulation.create_body(
            &RigidBodyBuilder::dynamic().translation([350., 150.].into()),
            &vec![ColliderBuilder::cuboid(60., 60.)]),
        size: vec2(120., 120.)};

    // Game loop
    loop {
        simulation.step(get_frame_time().min(0.05));

        clear_background(LIGHTGRAY);
        block.draw(&simulation, true);
        simulation.draw_colliders(GREEN, 2.0);

        next_frame().await
    }
}
```

Breakdown
---------

* `PhysicsSimulation::new` Creates a simulation with a given gravity.
  This simulation assumes 1 pixel = 1 cm, so 9.8m/ss becomes 980m/ss.
* `simulation::create_body` Creates a body for a RigidBodyBuilder
  and a list of ColliderBuilders, and adds it to the simulation.
  It returns a `RigidBodyHandle`, which can be used to reference the
  rigid body with `simulation.rigid_body_set.get(handle)`.
* `RigidBodyBuilder::fixed()` Creates a rigid body for the floor that
  won't move. This holds physical properties for the rigid body that can
  be chained together.
  (eg. `RigidBodyBuilder::fixed().translation(...).rotation(...)`)
* `polygon_collider(&shape)` Creates a polygon `ColliderBuilder` defined
  by a list of vertices, which may be of type `[f32; 2]`, `Vec2` or anything
  that can be cast to an `[f32; 2]`. In addition to shapes, colliders
  have physical properties that can be chained together.
  (eg. `polygon_collider(&shape).friction(...).restitution(...)`)
* `PhysicsSprite { texture, body, size }` This is a structure for rendering
  a texture relative to a body, in this case, a brick block. The texture
  will be centered at the body's origin. Since not all bodies are
  rectangles, size is used to determin how big the texture should be to line
  it up with its colliders.
* `simulation.step(get_frame_time().min(0.05));` Runs the simulation for the
  duration of the frame, but at most 0.05 seconds to prevent sudden jumps in
  time.
  It's often best practice to use a fixed time step for simulation rather
  than frame time.
* `block.draw(&simulation, true);` Draws the PhysicsSprite onto the screen.
  Physics coordinates will match camera coordinates, so
  `macroquad::camera::set_camera(...)` can be used to specify the region of
  the physics world that should apper on screen.
  The second parameter is used to indicate whether the camera's y axis
  increases towards the top of the screen (false for the default camera).
  Note: Y axis direction doesn't affect how the physics engine runs, it just
  indicates if the texture should be flipped.
* `simulation.draw_colliders(GREEN, 2.0);` Outlines the physics colliders with
  a given color and line weight. Physics coordinates will match camera
  coordinates, so `macroquad::camera::set_camera(...)` can be used to specify
  the region of the physics world that should apper on screen.

SVG
===

Optional feature for storing a rasterized image and a path for a physics
outline in an svg. This makes it easy to open a sprite in inkscape, draw an
outline for it, then save the result to an svg file that can be imported
directly.

The truck example makes use of this with `assets/arch.svg`. Note that, while
bezier curves are supported, arches are not.

Example
-------

```
let arch_builder = load_svg_physics_sprite(
    &ColliderBuilder::default(),
    include_str!("../../assets/arch.svg"),
    vec2(2.0, 1.0),
    true).unwrap();
let arch = arch_builder.build(
    &mut simulation,
    &RigidBodyBuilder::dynamic()
      .translation([5.0,0.0].into())
      .rotation(f32::consts::PI));
```

### Breakdown

* `load_svg_physics_sprite` Loads the texture and shape of the object.
  * `&ColliderBuilder::default(),` The physical properties of this collider
    will be applied to the colliders from the SVG.
  * `include_str!("../../assets/arch.svg"),` The SVG, passed in as a string.
  * `vec2(2.0, 1.0),` The size of the svg canvas in physics dimensions.
  * `true` Indicates the coordinates should be flipped so the y axis is up
    (because, in this case, the camera has its y axis up.)
* `arch_builder.build` Creates a new physics body based on the provided
  RigidBodyBuilder. This can be used repeatedly to create multiple bodies
  without reloadind the svg and its embedded texture.