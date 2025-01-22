use macroquad::prelude::*;
use rapier2d::prelude::*;
extern crate macroquad_rapier_interface as physics;
use physics::*;

#[macroquad::main("physics")]
async fn main() {
    let brick_texture = load_texture("assets/brick_texture.png").await.unwrap();
    let mut simulation = PhysicsSimulation::new(vec2(0.0, 980.0));

    // Add sloped floor
    let shape = vec![vec2(0., 400.), vec2(800., 500.), vec2(800., 600.), vec2(0., 600.)];
    simulation.create_body(&RigidBodyBuilder::fixed(), &vec![polygon_collider(&shape)]);

    // Make block
    let block = PhysicsSprite{
        texture: brick_texture,
        body: simulation.create_body(
            &RigidBodyBuilder::dynamic().translation(to_physics_vector(vec2(350., 150.))),
            &vec![ColliderBuilder::cuboid(60., 60.)]),
        size: vec2(120., 120.)};

    // Game loop
    loop {
        simulation.step(get_frame_time());

        clear_background(LIGHTGRAY);
        block.draw(&simulation, true);
        simulation.draw_debug(GREEN, 2.0);

        next_frame().await
    }
}