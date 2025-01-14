use macroquad::prelude::*;
use rapier2d::prelude::*;
mod physics;
use physics::*;
mod truck;
use truck::*;

fn make_box(physics: &mut PhysicsSimulation, texture: &Texture2D, rect: Rect, dynamic: bool) -> PhysicsSprite {
    let body_type = if dynamic {RigidBodyType::Dynamic} else {RigidBodyType::Fixed};
    let mut properties = PhysicalProperties::new(body_type);
    properties.colliders.push(ColliderBuilder::cuboid(rect.w * 0.5, rect.h * 0.5));
    properties.set_location(vec2(rect.x + rect.w * 0.5, rect.y + rect.h * 0.5));
    let physics_handle = physics.create_body(&properties);
    return PhysicsSprite{
        texture: texture.clone(),
        physics: physics_handle,
        size: vec2(rect.w, rect.h)
    }
}

fn make_ball(physics: &mut PhysicsSimulation, texture: &Texture2D, circle: Circle) -> PhysicsSprite {
    let mut properties = PhysicalProperties::new(RigidBodyType::Dynamic);
    properties.colliders.push(
        ColliderBuilder::ball(circle.radius())
        .friction(2.0)
        .density(2.0));
    properties.set_location(vec2(circle.x, circle.y));
    let physics_handle = physics.create_body(&properties);
    return PhysicsSprite{
        texture: texture.clone(),
        physics: physics_handle,
        size: Vec2::splat(circle.radius() * 2.0)
    }
}

struct GameState {
    pub simulation: PhysicsSimulation,
    pub sprites: Vec<PhysicsSprite>,
    pub truck: Truck
}

impl GameState {
    pub async fn new() -> Self {
        let mut simulation = PhysicsSimulation::new(vec2(0.0, -9.8));
        let brick_texture = load_texture("assets/brick_texture.png").await.unwrap();
        let truck = Truck::new(&mut simulation, vec2(0.0, 0.0)).await;
        let mut sprites: Vec<PhysicsSprite> = Vec::new();
        // make floor
        for i in 0..32 {
            sprites.push(
                make_box(
                    &mut simulation,
                    &brick_texture,
                    Rect { x: i as f32 - 16.0, y: -9.0, w: 1.0, h: 1.0 },
                    false));
        }
        // make walls
        for i in 0..18 {
            sprites.push(
                make_box(
                    &mut simulation,
                    &brick_texture,
                    Rect { x: -17.0, y: i as f32 - 9.0, w: 1.0, h: 1.0 },
                    false));
            sprites.push(
                make_box(
                    &mut simulation,
                    &brick_texture,
                    Rect { x: 16.0, y: i as f32 - 9.0, w: 1.0, h: 1.0 },
                    false));
        }
        Self { simulation, sprites, truck }
    }
    pub fn update(&mut self) {
        self.truck.update(&mut self.simulation);
        self.simulation.step(get_frame_time().min(1.0/20.0));
    }
    pub fn draw(&self) {
        for sprite in &self.sprites {
            sprite.draw(&self.simulation, false);
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
    let mut game_state = GameState::new().await;
    let mut debug = false;
    loop {
        clear_background(LIGHTGRAY);

        let aspect_ratio = screen_width()/screen_height();
        let cam_height = 18.0;
        let cam_width = cam_height * aspect_ratio;
        let camera: Camera2D = Camera2D::from_display_rect(
            Rect::new(-cam_width * 0.5, -cam_height * 0.5, cam_width, cam_height)
        );
        set_camera(&camera);

        // Update and render
        game_state.update();
        game_state.draw();

        // Left click to add sprites
        if is_mouse_button_pressed(MouseButton::Left) {
            let mut mouse = vec2(mouse_position().0, mouse_position().1);
            mouse = camera.screen_to_world(mouse);
            // Use shift to add a square, otherwise adds a circle
            if is_key_down(KeyCode::LeftShift) || is_key_down(KeyCode::RightShift) {
                let rect = Rect::new(mouse.x - 0.5, mouse.y - 0.5, 1.0, 1.0);
                let phys_box = make_box(&mut game_state.simulation, &brick_texture, rect, true);
                game_state.sprites.push(phys_box);
            } else {
                let circle = Circle::new(mouse.x, mouse.y, 0.5);
                let ball = make_ball(&mut game_state.simulation, &ball_texture, circle);
                game_state.sprites.push(ball);
            }
        }
        // Right click to remove sprites
        if is_mouse_button_pressed(MouseButton::Right) {
            let mut mouse = vec2(mouse_position().0, mouse_position().1);
            mouse = camera.screen_to_world(mouse);
            let intersecting_bodies = game_state.simulation.get_bodies_at_point(mouse, true);
            let mut removed_sprites: Vec<PhysicsSprite> = Vec::new();
            for body in intersecting_bodies {
                let mut i = 0;
                while i < game_state.sprites.len() {
                    if game_state.sprites[i].physics.body == body {
                        removed_sprites.push(game_state.sprites.remove(i));
                    } else {
                        i += 1;
                    }
                }
            }
            for sprite in removed_sprites {
                game_state.simulation.destroy_body(sprite.physics);
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
            game_state.simulation.draw_debug(GREEN, 0.0625);
        }

        next_frame().await
    }
}