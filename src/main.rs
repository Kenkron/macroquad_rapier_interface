use macroquad::prelude::*;
mod physics;
use physics::*;
use rapier2d::prelude::*;
mod truck;
use truck::*;

fn make_fixed_rect(physics: &mut Physics, texture: &Texture2D, rect: Rect) -> PhysicsSprite {
    let mut properties = PhysicalProperties::new(RigidBodyType::Fixed);
    properties.colliders.push(ColliderBuilder::cuboid(rect.w * 0.5, rect.h * 0.5));
    properties.set_location(vec2(rect.x + rect.w * 0.5, rect.y + rect.h * 0.5));
    let physics_handle = physics.create_body(&properties);
    return PhysicsSprite{
        texture: texture.clone(),
        physics: physics_handle,
        size: vec2(rect.w, rect.h)
    }
}

fn make_box(physics: &mut Physics, texture: &Texture2D, rect: Rect) -> PhysicsSprite {
    let mut properties = PhysicalProperties::new(RigidBodyType::Dynamic);
    properties.colliders.push(ColliderBuilder::cuboid(rect.w * 0.5, rect.h * 0.5));
    properties.set_location(vec2(rect.x + rect.w * 0.5, rect.y + rect.h * 0.5));
    let physics_handle = physics.create_body(&properties);
    return PhysicsSprite{
        texture: texture.clone(),
        physics: physics_handle,
        size: vec2(rect.w, rect.h)
    }
}

fn make_ball(physics: &mut Physics, texture: &Texture2D, circle: Circle) -> PhysicsSprite {
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
    pub physics_world: Physics,
    pub sprites: Vec<PhysicsSprite>,
    pub truck: Truck
}

impl GameState {
    pub async fn new() -> Self {
        let mut physics_world = Physics::new(1.0/60.0, vec2(0.0, -9.8));
        let brick_texture = load_texture("assets/brick_texture.png").await.unwrap();
        let truck = Truck::new(&mut physics_world, vec2(0.0, 0.0)).await;
        let mut sprites: Vec<PhysicsSprite> = Vec::new();
        // make floor
        for i in 0..32 {
            sprites.push(make_fixed_rect(&mut physics_world, &brick_texture, Rect { x: i as f32 - 16.0, y: -9.0, w: 1.0, h: 1.0 }));
        }
        // make walls
        for i in 0..18 {
            sprites.push(make_fixed_rect(&mut physics_world, &brick_texture, Rect { x: -17.0, y: i as f32 - 9.0, w: 1.0, h: 1.0 }));
            sprites.push(make_fixed_rect(&mut physics_world, &brick_texture, Rect { x: 16.0, y: i as f32 - 9.0, w: 1.0, h: 1.0 }));
        }
        Self { physics_world, sprites, truck }
    }
    pub fn update(&mut self) {
        self.truck.update(&mut self.physics_world);
        self.physics_world.step();
    }
    pub fn draw(&self) {
        for sprite in &self.sprites {
            sprite.draw(&self.physics_world, false);
        }
        self.truck.draw(&self.physics_world);
    }
}

#[macroquad::main("Physics")]
async fn main() {

    let ball_texture = load_texture("assets/tire.png").await.unwrap();
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

        game_state.update();
        game_state.draw();

        if is_mouse_button_pressed(MouseButton::Left) {
            let mut mouse = vec2(mouse_position().0, mouse_position().1);
            mouse = camera.screen_to_world(mouse);
            let circle = Circle::new(mouse.x, mouse.y, 0.4);
            let ball = make_ball(&mut game_state.physics_world, &ball_texture, circle);
            game_state.sprites.push(ball);
        }
        if is_key_pressed(KeyCode::I) {
            debug = !debug;
        }
        if debug {
            game_state.physics_world.draw_debug(GREEN, 0.125);
        }
        if is_key_pressed(KeyCode::R) {
            game_state = GameState::new().await;
        }

        next_frame().await
    }
}