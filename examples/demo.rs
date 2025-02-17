use macroquad::prelude::*;
use physics::{to_physics_vector, PhysicsSimulation, PhysicsSprite};
use rapier2d::prelude::*;
extern crate macroquad_rapier_interface as physics;

#[macroquad::main("physics")]
async fn main() {
    let ball_texture = load_texture("assets/tire.png").await.unwrap();
    let brick_texture = load_texture("assets/brick_texture.png").await.unwrap();
    let mut debug = true;
    let mut simulation = PhysicsSimulation::new(vec2(0.0, -9.8));
    let mut sprites: Vec<PhysicsSprite> = vec![];
    let mut elevators: Vec<PhysicsSprite> = vec![];

    // Scale camera to world coordinates (8x6 with the y axis up)
    let physics_camera = Camera2D::from_display_rect(Rect::new(0.0, 0.0, 8.0, 6.0));

    // Add floor
    sprites.push(
        PhysicsSprite {
            texture: brick_texture.clone(),
            size: vec2(8.0, 1.0),
            body: simulation.create_body(
                &RigidBodyBuilder::fixed().translation([4.0, -0.5].into()),
                &vec![ColliderBuilder::cuboid(4.0, 0.5)]
            )});

    // Add elevators
    elevators.push(
        PhysicsSprite {
            texture: brick_texture.clone(),
            size: vec2(1.0, 1.0),
            body: simulation.create_body(
                &RigidBodyBuilder::kinematic_velocity_based().translation([7.0, -0.5].into()),
                &vec![ColliderBuilder::cuboid(0.5, 0.5)]
        )});
    for elevator in &elevators {
        if let Some(elevator) = simulation.rigid_body_set.get_mut(elevator.body) {
            elevator.set_linvel([0.0, 1.0].into(), true);
        }
    }

    loop {
        //////// Input ////////
        // Left click to add sprites
        if is_mouse_button_pressed(MouseButton::Left) {
            let location = physics_camera.screen_to_world(mouse_position().into());
            if is_key_down(KeyCode::LeftShift) || is_key_down(KeyCode::RightShift) {
                sprites.push(spawn_block(&mut simulation, location, brick_texture.clone()));
            } else {
                sprites.push(spawn_ball(&mut simulation, location, ball_texture.clone()));
            }
        }

        // Right click to remove sprites
        if is_mouse_button_pressed(MouseButton::Right) {
            let location = physics_camera.screen_to_world(mouse_position().into());
            let intersecting_bodies = simulation.get_bodies_at_point(location, true);
            for body in intersecting_bodies {
                let index = sprites.iter().enumerate().find(|(_, physics)| physics.body == body);
                if index.is_some() {
                    sprites.remove(index.unwrap().0);
                    simulation.destroy_body(body);
                }
            }
        }

        // Show/Hide debug
        if is_key_pressed(KeyCode::I) { debug = !debug }

        //////// Simulate ///////
        simulation.step(get_frame_time().min(0.05));
        for elevator in &elevators {
            if let Some (body) = simulation.rigid_body_set.get_mut(elevator.body) {
                let mut translation = body.position().translation;
                if translation.y > 7.0 {translation.y = -1.0}
                body.set_translation([translation.x, translation.y].into(), true);
            }
        }

        //////// Render ////////
        clear_background(LIGHTGRAY);
        set_camera(&physics_camera);
        for elevator in &elevators {
            elevator.draw(&simulation, true);
        }
        for sprite in &sprites {
            sprite.draw(&simulation, true);
        }
        if debug {
            simulation.draw_colliders(GREEN, 0.03125);
        }

        // Reset to the default camera for the HUD
        set_default_camera();
        draw_text(&format!("FPS: {}", get_fps()), 100.0, 20.0, 20.0, DARKBLUE);
        draw_text("Left Click to add a circle", 100.0, 60.0, 20.0, DARKBLUE);
        draw_text("Shift + Left Click to add a square", 100.0, 80.0, 20.0, DARKBLUE);
        draw_text("Right Click to remove a shape", 100.0, 100.0, 20.0, DARKBLUE);
        draw_text("I to toggle debug drawing", 100.0, 120.0, 20.0, DARKBLUE);

        next_frame().await
    }
}

fn spawn_ball(simulation: &mut PhysicsSimulation, location: Vec2, texture: Texture2D) -> PhysicsSprite {
    PhysicsSprite{
        texture,
        body: simulation.create_body(
            &RigidBodyBuilder::dynamic().translation(to_physics_vector(location)),
            &vec![ColliderBuilder::ball(0.5)]),
        size: vec2(1.0, 1.0)}
}

fn spawn_block(simulation: &mut PhysicsSimulation, location: Vec2, texture: Texture2D) -> PhysicsSprite {
    PhysicsSprite{
        texture,
        body: simulation.create_body(
            &RigidBodyBuilder::dynamic().translation(to_physics_vector(location)),
            &vec![ColliderBuilder::cuboid(0.5, 0.5)]),
        size: vec2(1.0, 1.0)}
}