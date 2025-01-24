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