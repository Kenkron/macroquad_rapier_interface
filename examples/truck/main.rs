use core::f32;
use std::collections::{HashMap, HashSet};

use macroquad::{audio::{load_sound, play_sound_once, set_sound_volume}, prelude::*};
use rapier2d::prelude::*;
extern crate macroquad_rapier_interface as physics;
use physics::*;
use svg::load_svg_physics_sprite;
mod truck;

fn make_box(physics: &mut PhysicsSimulation, texture: &Texture2D, rect: Rect, dynamic: bool) -> PhysicsSprite {
    let body_type = if dynamic {RigidBodyType::Dynamic} else {RigidBodyType::Fixed};
    let body_builder =
        RigidBodyBuilder::new(body_type)
        .translation(vector![rect.x + rect.w * 0.5, rect.y + rect.h * 0.5]);
    let collider_builders = vec![
        ColliderBuilder::cuboid(rect.w * 0.5, rect.h * 0.5)
            .friction(0.75)
            .active_events(ActiveEvents::CONTACT_FORCE_EVENTS)
    ];
    return PhysicsSprite{
        texture: texture.clone(),
        body: physics.create_body(&body_builder, &collider_builders),
        size: vec2(rect.w, rect.h)
    }
}

fn make_ball(physics: &mut PhysicsSimulation, texture: &Texture2D, circle: Circle) -> PhysicsSprite {
    let body_builder =
        RigidBodyBuilder::new(RigidBodyType::Dynamic)
        .translation(vector![circle.x, circle.y]);
    let collider_builders = vec![
        ColliderBuilder::ball(circle.radius())
            .friction(2.0)
            .density(2.0)
            .active_events(ActiveEvents::CONTACT_FORCE_EVENTS)
    ];
    return PhysicsSprite{
        texture: texture.clone(),
        body: physics.create_body(&body_builder, &collider_builders),
        size: Vec2::splat(circle.radius() * 2.0)
    }
}

struct GameState {
    pub simulation: PhysicsSimulation,
    pub sprites: HashMap<RigidBodyHandle, PhysicsSprite>,
    pub truck: truck::Truck
}

impl GameState {
    pub async fn new() -> Self {
        let mut simulation = PhysicsSimulation::new(vec2(0.0, -9.8));
        let brick_texture = load_texture("assets/brick_texture.png").await.unwrap();
        let truck = truck::Truck::new(&mut simulation, vec2(0.0, 0.0)).await;
        let mut sprites: HashMap<RigidBodyHandle, PhysicsSprite> = HashMap::new();
        // make floor
        simulation.create_body(
            &RigidBodyBuilder::fixed()
                .translation([0.0, -10.0].into()),
            &[ColliderBuilder::cuboid(16.0, 1.0)
            .friction(0.75)]);
        // make walls
        for i in 0..18 {
            let left_wall = make_box(
                    &mut simulation,
                    &brick_texture,
                    Rect { x: -17.0, y: i as f32 - 9.0, w: 1.0, h: 1.0 },
                    false);
            sprites.insert(left_wall.body, left_wall);
            let right_wall =
                make_box(
                    &mut simulation,
                    &brick_texture,
                    Rect { x: 16.0, y: i as f32 - 9.0, w: 1.0, h: 1.0 },
                    false);
            sprites.insert(right_wall.body, right_wall);
        }
        // make arch
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
        sprites.insert(arch.body, arch);
        Self { simulation, sprites, truck }
    }
    pub fn update(&mut self) -> Vec<ContactForceEvent>{
        self.truck.update(&mut self.simulation);
        let (_, contact_force_events) = self.simulation.step(get_frame_time().min(1.0/20.0));
        contact_force_events
    }
    pub fn draw(&self) {
        for (_, sprite) in &self.sprites {
            sprite.draw(&self.simulation, true);
        }
        self.truck.draw(&self.simulation);
    }
}

fn window_conf() -> Conf {
    Conf {
        window_title: "Physics".to_string(),
        window_width: 1200,
        window_height: 675,
        ..Default::default()
    }
}
#[macroquad::main(window_conf)]
async fn main() {
    let ball_texture = load_texture("assets/tire.png").await.unwrap();
    let brick_texture = load_texture("assets/brick_texture.png").await.unwrap();
    let collision_sound = load_sound("assets/collision.wav").await.unwrap();
    let mut game_state = GameState::new().await;
    let mut already_touching: HashSet<ColliderPair> = HashSet::new();
    let mut debug = false;
    loop {
        clear_background(LIGHTGRAY);

        // Use a scaled, y-axis-up camera for the world
        let cam_height = 18.0;
        let cam_width = cam_height * screen_width() / screen_height();
        let camera: Camera2D = Camera2D::from_display_rect(
            Rect::new(-cam_width * 0.5, -cam_height * 0.5, cam_width, cam_height)
        );
        set_camera(&camera);

        // Update and render
        let contact_force_events = game_state.update();
        game_state.draw();

        // Play audio for collisions
        let mut touching: HashSet<ColliderPair> = HashSet::new();
        for collision_event in contact_force_events {
            let pair = ColliderPair::new(collision_event.collider1, collision_event.collider2);
            touching.insert(pair);
            // Handle the collision event.
            if collision_event.max_force_magnitude > 100.0 &&
                !already_touching.contains(&pair) &&
                !already_touching.contains(&pair.swap()) &&
                get_frame_time() > 0.0 {
                let impulse = collision_event.max_force_magnitude / get_frame_time();
                play_sound_once(&collision_sound);
                set_sound_volume(&collision_sound, 1.0_f32.min(impulse/1000000.0));
            }
        }
        already_touching = touching;

        // Left click to add sprites
        if is_mouse_button_pressed(MouseButton::Left) {
            let mut mouse = vec2(mouse_position().0, mouse_position().1);
            mouse = camera.screen_to_world(mouse);
            // Use shift to add a square, otherwise adds a circle
            if is_key_down(KeyCode::LeftShift) || is_key_down(KeyCode::RightShift) {
                let rect = Rect::new(mouse.x - 0.5, mouse.y - 0.5, 0.9, 0.9);
                let phys_box = make_box(&mut game_state.simulation, &brick_texture, rect, true);
                game_state.sprites.insert(phys_box.body, phys_box);
            } else {
                let circle = Circle::new(mouse.x, mouse.y, 0.5);
                let ball = make_ball(&mut game_state.simulation, &ball_texture, circle);
                game_state.sprites.insert(ball.body, ball);
            }
        }
        // Right click to remove sprites
        if is_mouse_button_pressed(MouseButton::Right) {
            let mut mouse = vec2(mouse_position().0, mouse_position().1);
            mouse = camera.screen_to_world(mouse);
            let intersecting_bodies = game_state.simulation.get_bodies_at_point(mouse, true);
            for body in intersecting_bodies {
                let removed = game_state.sprites.remove(&body);
                if removed.is_some() {
                    game_state.simulation.destroy_body(body);
                }
            }
        }
        // Press r to reset the simulation
        if is_key_pressed(KeyCode::R) {
            game_state = GameState::new().await;
        }
        // Press i to toggle debug
        if is_key_pressed(KeyCode::I) {
            debug = !debug;
        }
        if debug {
            game_state.simulation.draw_colliders(GREEN, 0.0625);
        }

        // Reset to the default camera for the HUD
        set_default_camera();
        draw_text(&format!("FPS: {}", get_fps()), 100.0, 20.0, 20.0, DARKBLUE);
        draw_text("Press W and S to drive the truck forward and backwards", 100.0, 40.0, 20.0, DARKBLUE);
        draw_text("Left Click to add a circle", 100.0, 60.0, 20.0, DARKBLUE);
        draw_text("Shift + Left Click to add a square", 100.0, 80.0, 20.0, DARKBLUE);
        draw_text("Right Click to remove a shape", 100.0, 100.0, 20.0, DARKBLUE);
        draw_text("I to toggle debug drawing", 100.0, 120.0, 20.0, DARKBLUE);

        next_frame().await
    }
}