use macroquad::prelude::*;
use rapier2d::prelude::*;
use crate::physics::*;

/// The same as [truck::Truck], except this version uses an explicit
/// png image and manual vectors, instead of loading both from an svg,
/// and uses a simple rotation joint for the wheels, rather than
/// a joint with springs as well.
pub struct Truck {
    front_wheel: PhysicsSprite,
    rear_wheel: PhysicsSprite,
    frame: PhysicsSprite,
    front_wheel_drive: ImpulseJointHandle,
    rear_wheel_drive: ImpulseJointHandle,
}

const FRONT_WHEEL_POSITION: Vec2 = vec2(2.0, -1.0);
const REAR_WHEEL_POSITION: Vec2 = vec2(-2.0, -1.0);

impl Truck {
    /// Create a truck with its frame's center at a given location.
    pub async fn new(simulation: &mut PhysicsSimulation, location: Vec2) -> Self {
        // Texture and shape of a wheel
        const WHEEL_RADIUS: f32 = 0.75;
        let wheel_texture = load_texture("assets/tire.png").await.unwrap();
        let wheel_colliders = vec![
            ColliderBuilder::ball(WHEEL_RADIUS)
                .friction(2.0)
        ];

        // Rear wheel
        let rear_wheel_location = to_physics_vector(location + REAR_WHEEL_POSITION);
        let rear_wheel_body = RigidBodyBuilder::new(RigidBodyType::Dynamic).translation(rear_wheel_location);
        let rear_wheel = PhysicsSprite {
            texture: wheel_texture.clone(),
            body: simulation.create_body(&rear_wheel_body, &wheel_colliders),
            size: Vec2::splat(WHEEL_RADIUS * 2.0)
        };

        // Front wheel
        let front_wheel_location = to_physics_vector(location + FRONT_WHEEL_POSITION);
        let front_wheel_body =
            RigidBodyBuilder::new(RigidBodyType::Dynamic)
            .translation(front_wheel_location);
        let front_wheel = PhysicsSprite {
            texture: wheel_texture.clone(),
            body: simulation.create_body(&front_wheel_body, &wheel_colliders),
            size: Vec2::splat(WHEEL_RADIUS * 2.0)
        };

        // Texture and properties of the frame
        let frame_texture = load_texture("assets/truck_frame.png").await.unwrap();
        let frame_shape = [
            vec2(3.0, -0.75), vec2(3.0, 0.15), vec2(2.0, 0.25), vec2(1.0, 0.75),
            vec2(0.0, 0.75), vec2(0.0, 0.05), vec2(-3.0, 0.05), vec2(-3.0, -0.75)
        ];
        let collider_builders = vec![
            polygon_collider(&frame_shape)
        ];
        let frame_builder =
            RigidBodyBuilder::new(RigidBodyType::Dynamic)
            .translation(to_physics_vector(location));
        let frame = PhysicsSprite {
            texture: frame_texture,
            body: simulation.create_body(&frame_builder, &collider_builders),
            size: vec2(6.0, 1.5)
        };

        // Make front wheel drive
        let joint = RevoluteJointBuilder::new()
            .local_anchor1(to_physics_point(REAR_WHEEL_POSITION))
            .local_anchor2(Point::new(0., 0.))
            .contacts_enabled(false);
        let front_wheel_drive = simulation.impulse_joint_set.insert(frame.body, rear_wheel.body, joint, false);

        // Make rear wheel drive
        let joint = RevoluteJointBuilder::new()
                .local_anchor1(to_physics_point(FRONT_WHEEL_POSITION))
                .local_anchor2(Point::new(0., 0.))
                .contacts_enabled(false);
        let rear_wheel_drive = simulation.impulse_joint_set.insert(frame.body, front_wheel.body, joint, false);

        Self {
            front_wheel: front_wheel,
            rear_wheel: rear_wheel,
            frame: frame,
            front_wheel_drive: front_wheel_drive,
            rear_wheel_drive: rear_wheel_drive
        }
    }

    /// Process inputs for truck movement, and apply the corresponding
    /// movements in the simulation.
    pub fn update(&self, simulation: &mut PhysicsSimulation) {
        let torque = 40.0;
        let mut speed = 0.0;
        if is_key_down(KeyCode::W) {
            speed -= 15.0;
        }
        if is_key_down(KeyCode::S) {
            speed += 15.0;
        }
        simulation.set_motor(self.front_wheel_drive, torque, speed, 0.0);
        simulation.set_motor(self.rear_wheel_drive, torque, speed, 0.0);
    }

    pub fn draw(&self, simulation: &PhysicsSimulation) {
        self.frame.draw(simulation, false);
        self.front_wheel.draw(simulation, false);
        self.rear_wheel.draw(simulation, false);
    }
}