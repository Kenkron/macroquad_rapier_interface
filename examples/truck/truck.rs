use macroquad::prelude::*;
use rapier2d::prelude::*;
use crate::physics::*;
use crate::svg::*;

/// A Truck is a relatively complex physical structure.
///
/// It contains a front wheel, rear wheel, and a body, as well as a motor
/// to turn the wheels.
///
/// The front and rear wheels must be able to overlap the body without
/// colliding. This is accomplished with collision groups.
///
/// Additionally, because of the truck bed, the body of the truck is
/// convex, and must be use a convex decomposition.
///
/// Finally, the wheels are attached to the frame of the truck, and while
/// this could be done with a RevoluteJoint, using a GenericJoint will
/// allow the joint to simulate suspension on the x and y axis.
pub struct Truck {
    front_wheel: PhysicsSprite,
    rear_wheel: PhysicsSprite,
    frame: PhysicsSprite,
    front_wheel_drive: ImpulseJointHandle,
    rear_wheel_drive: ImpulseJointHandle,
}

const FRONT_WHEEL_POSITION: Vec2 = vec2(2.0, -1.0);
const REAR_WHEEL_POSITION: Vec2 = vec2(-2.0, -1.0);

/// Build suspension for a tire.
///
/// Creates a builder for a joint that works similar to
/// RevoluteJointBuilder::new(), but also allows movement on the X and Y
/// axis with springs/damping.
fn suspension() -> GenericJointBuilder {
    GenericJointBuilder::new(JointAxesMask::empty())
        .motor_position(JointAxis::LinX, 0.0, 500.0, 20.0)
        .motor_max_force(JointAxis::LinX, 500.0)
        .motor_position(JointAxis::LinY, 0.0, 500.0, 20.0)
        .motor_max_force(JointAxis::LinY, 500.0)
}

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

        // Frame
        let frame_builder =
            RigidBodyBuilder::new(RigidBodyType::Dynamic)
            .translation(to_physics_vector(location));
        let frame = load_svg_physics_sprite(
            simulation,
            &frame_builder,
            &ColliderBuilder::default(),
            include_str!("../../assets/truck_frame.svg"),
            vec2(6.0, 1.5),
            false).unwrap();

        // Make front wheel drive
        let joint = suspension()
            .local_anchor1(to_physics_point(REAR_WHEEL_POSITION))
            .local_anchor2(Point::new(0., 0.))
            .contacts_enabled(false);
        let front_wheel_drive = simulation.impulse_joint_set.insert(frame.body, rear_wheel.body, joint, false);

        // Make rear wheel drive
        let joint = suspension()
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