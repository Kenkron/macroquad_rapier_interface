use macroquad::{prelude::*, telemetry::Frame};
use rapier2d::prelude::*;
use crate::physics::{self, *};

/// A Truck is a relatively complex physical structure.
///
/// It contains a front wheel, rear wheel, and a body, as well as a motor
/// to turn the wheels.
///
/// The front and rear wheels must be able to overlap the body without
/// colliding. This is accomplished with collision groups.
///
/// Additionally, because of the truck bed, the body of the truck is
/// convex, and must be split into multiplie colliders.
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
    pub async fn new(simulation: &mut PhysicsSimulation, location: Vec2) -> Self {
        // To keep the wheels from colliding with the frame of the truck,
        // the wheel and frame must be in separate groups that do not
        // interact with each other
        let wheel_group = isolate_physics_group(Group::GROUP_1);
        let frame_group = isolate_physics_group(Group::GROUP_2);

        // Texture and properties of a wheel
        const WHEEL_RADIUS: f32 = 0.75;
        let wheel_texture = load_texture("assets/tire.png").await.unwrap();
        let mut wheel_properties = PhysicalProperties::new(RigidBodyType::Dynamic);
        wheel_properties.colliders.push(
            ColliderBuilder::ball(WHEEL_RADIUS)
                .collision_groups(wheel_group)
                .friction(2.0)
        );

        // Rear wheel
        wheel_properties.set_location(location - REAR_WHEEL_POSITION);
        let rear_wheel = PhysicsSprite {
            texture: wheel_texture.clone(),
            physics: simulation.create_body(&wheel_properties),
            size: Vec2::splat(WHEEL_RADIUS * 2.0)
        };

        // Front wheel
        wheel_properties.set_location(location - FRONT_WHEEL_POSITION);
        let front_wheel = PhysicsSprite {
            texture: wheel_texture.clone(),
            physics: simulation.create_body(&wheel_properties),
            size: Vec2::splat(WHEEL_RADIUS * 2.0)
        };

        // Texture and properties of the frame
        let frame_texture = load_texture("assets/truck_frame.png").await.unwrap();
        let frame_shape = [
            vec2(3.0, -0.75),
            vec2(3.0, 0.15),
            vec2(2.0, 0.25),
            vec2(1.0, 0.75),
            vec2(0.0, 0.75),
            vec2(0.0, 0.05),
            vec2(-3.0, 0.05),
            vec2(-3.0, -0.75)
        ];
        let mut frame_properties = PhysicalProperties::new(RigidBodyType::Dynamic);
        frame_properties.colliders.push(polygon_collider(&frame_shape).collision_groups(frame_group));
        let frame = PhysicsSprite {
            texture: frame_texture,
            physics: simulation.create_body(&frame_properties),
            size: vec2(6.0, 1.5)
        };

        // Make front wheel drive
        let joint_properties = RevoluteJointBuilder::new()
                .local_anchor1(vec_to_point(FRONT_WHEEL_POSITION))
                .local_anchor2(Point::new(0., 0.));
        let front_wheel_drive = simulation.create_joint(&joint_properties, &frame.physics, &front_wheel.physics);
        //physics.set_motor(front_wheel_drive, 10.0, 0.0, 1.0);

        // Make front wheel drive
        let joint_properties = RevoluteJointBuilder::new()
                .local_anchor1(vec_to_point(REAR_WHEEL_POSITION))
                .local_anchor2(Point::new(0., 0.));
        let rear_wheel_drive = simulation.create_joint(&joint_properties, &frame.physics, &rear_wheel.physics);
        //physics.set_motor(front_wheel_drive, 10.0, 0.0, 1.0);

        Self {
            front_wheel: front_wheel,
            rear_wheel: rear_wheel,
            frame: frame,
            front_wheel_drive: front_wheel_drive,
            rear_wheel_drive: rear_wheel_drive
        }
    }

    pub fn update(&self, simulation: &mut PhysicsSimulation) {
        let torque = 30.0;
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