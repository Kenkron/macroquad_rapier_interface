use macroquad::prelude::*;
use rapier2d::na::UnitVector2;
use rapier2d::parry::transformation::vhacd::VHACDParameters;
use rapier2d::prelude::*;
use rapier2d::crossbeam::channel;

pub fn to_physics_point(source: Vec2) -> Point<Real> {
    Point::new(source.x, source.y)
}

pub fn to_physics_vector(source: Vec2) -> Vector<Real> {
    vector![source.x, source.y]
}

/// State of physics simulation
pub struct PhysicsSimulation {
    // Needed for step function
    pub gravity: Vector<Real>,
    pub rigid_body_set: RigidBodySet,
    pub collider_set: ColliderSet,
    pub physics_pipeline: PhysicsPipeline,
    pub integration_parameters: IntegrationParameters,
    pub island_manager: IslandManager,
    pub broad_phase: Box<dyn BroadPhase>,
    pub narrow_phase: NarrowPhase,
    pub impulse_joint_set: ImpulseJointSet,
    pub multibody_joint_set: MultibodyJointSet,
    pub ccd_solver: CCDSolver,
    pub event_handler: ChannelEventCollector,
    // Check for non-simulation intersections
    pub query_pipeline: QueryPipeline,
    // Anchor for friction
    pub friction_anchor: RigidBodyHandle,
    collision_reciever: channel::Receiver<CollisionEvent>,
    contact_force_reciever: channel::Receiver<ContactForceEvent>
}

impl PhysicsSimulation {
    pub fn new(gravity: Vec2) -> Self {
        // Initialize the event collector.
        let (collision_send, collision_recv) = channel::unbounded();
        let (contact_force_send, contact_force_recv) = channel::unbounded();
        let event_handler = ChannelEventCollector::new(collision_send, contact_force_send);
        let mut rigid_body_set = RigidBodySet::new();
        let friction_anchor = rigid_body_set.insert(RigidBodyBuilder::fixed().build());
        Self {
            gravity: vector![gravity.x, gravity.y],
            rigid_body_set,
            collider_set: ColliderSet::new(),
            physics_pipeline: PhysicsPipeline::new(),
            integration_parameters: IntegrationParameters::default(),
            island_manager: IslandManager::new(),
            broad_phase: Box::new(BroadPhaseMultiSap::new()),
            narrow_phase: NarrowPhase::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            event_handler,
            query_pipeline: QueryPipeline::new(),
            friction_anchor,
            collision_reciever: collision_recv,
            contact_force_reciever: contact_force_recv
        }
    }

    /// Adds a body to the physics simulation with colliders.
    pub fn create_body(&mut self, body_builder: &RigidBodyBuilder, collider_builders: &[ColliderBuilder]) -> RigidBodyHandle {
        let body_handle = self.rigid_body_set.insert(body_builder.build());
        let mut collider_handles: Vec<ColliderHandle> = Vec::new();
        for collider_builder in collider_builders {
            let handle =
                self.collider_set.insert_with_parent(
                    collider_builder.build(),
                    body_handle,
                    &mut self.rigid_body_set);
            collider_handles.push(handle);
        }
        body_handle
    }

    /// Removes a body from the simulation
    ///
    /// Also removes its colliders and joints.
    pub fn destroy_body(&mut self, body: RigidBodyHandle) {
        self.rigid_body_set.remove(
            body,
            &mut self.island_manager,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            true);
    }

    /// Configures a rotating motor on a rotating joint
    pub fn set_motor(
        &mut self,
        joint: ImpulseJointHandle,
        force: f32,
        max_velocity: f32,
        damping: f32) {
        if let Some(joint) = self.impulse_joint_set.get_mut(joint, true) {
            joint.data.set_motor_model(JointAxis::AngX, MotorModel::ForceBased);
            joint.data.set_motor_max_force(JointAxis::AngX, force);
            joint.data.set_motor(JointAxis::AngX, 0.0, max_velocity, 0.0, damping);
        }
    }

    /// Configures a motor that acts like a spring rotating to a specific position
    pub fn set_angular_spring(
        &mut self,
        joint: ImpulseJointHandle,
        max_force: f32,
        stifness: f32,
        angle: f32,
        damping: f32) {
        if let Some(joint) = self.impulse_joint_set.get_mut(joint, true) {
            joint.data.set_motor_model(JointAxis::AngX, MotorModel::ForceBased);
            joint.data.set_motor_max_force(JointAxis::AngX, max_force);
            joint.data.set_motor(JointAxis::AngX, angle, 0.0, stifness, damping);
        }
    }

    /// Find all bodies with colliders that intersect a given point
    ///
    /// Uses collider data from the last `step` call. If things have changed
    /// since then, `self.query_pipeline.update(&self.collider_set);` must
    /// be called.
    pub fn get_bodies_at_point(&self, location: Vec2, exclude_fixed: bool) -> Vec<RigidBodyHandle> {
        let query_filter = if exclude_fixed {QueryFilter::exclude_fixed()} else {QueryFilter::new()};
        let mut intersections: Vec<ColliderHandle> = Vec::new();
        self.query_pipeline.intersections_with_point(
            &self.rigid_body_set,
            &self.collider_set,
            &Point::new(location.x, location.y),
            query_filter,
            |collider| {
                intersections.push(collider);
                // Return true would break early
                false
            });
        intersections.iter()
            .filter_map(|collider_handle| self.collider_set.get(*collider_handle))
            .filter_map(|collider| collider.parent())
            .collect()
    }

    /// Move a rigid body to a new location and rotation.
    ///
    /// Does not affect bodies attached by joints.
    pub fn teleport(&mut self, body: RigidBodyHandle, location: &Vec2, angle: f32) {
        if let Some(body) = self.rigid_body_set.get_mut(body) {
            body.set_position(Isometry::translation(location.x, location.y), false);
            body.set_rotation(Rotation::from_angle(angle), true);
        }
    }

    /// Move the simulation forward a given amount of time.
    /// For consistency, its usually best to use a constant timestep.
    pub fn step(&mut self, time: f32) -> (Vec<CollisionEvent>, Vec<ContactForceEvent>){
        self.integration_parameters.dt = time;

        // Distribute world friction to be axis-independent
        let friction_joints: Vec<_> = self.impulse_joint_set.attached_joints(self.friction_anchor)
            .map(|joint_info| (joint_info.0, joint_info.1, joint_info.2)).collect();
        for joint in friction_joints {
            if let (Some(impulse_joint), Some(body)) =
                (self.impulse_joint_set.get_mut(joint.2, false), self.rigid_body_set.get(joint.1))
            {
                let velocity = body.velocity_at_point(&impulse_joint.data.local_anchor2());
                if velocity.magnitude_squared() > 0.0 {
                    let axis = UnitVector2::new_normalize(velocity);
                    // AngX is turned off, but its max_force is still used to store the friction force
                    let max_force_ang = impulse_joint.data.motors[JointAxis::AngX as usize].max_force;
                    let total_max_force = max_force_ang;
                    impulse_joint.data.set_motor_max_force(JointAxis::LinX, total_max_force * axis.x.abs());
                    impulse_joint.data.set_motor_velocity(JointAxis::LinX, 0.0, 0.0);
                    impulse_joint.data.set_motor_max_force(JointAxis::LinY, total_max_force * axis.y.abs());
                    impulse_joint.data.set_motor_velocity(JointAxis::LinY, 0.0, 0.0);
                }
            }
        }

        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut *self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            None,
            &(),
            &self.event_handler,
          );
        self.query_pipeline.update(&self.collider_set);

        // Collect events
        let mut collision_events: Vec<CollisionEvent> = Vec::new();
        let mut contact_force_events: Vec<ContactForceEvent> = Vec::new();
        while let Ok(collision_event) = self.collision_reciever.try_recv() {
            collision_events.push(collision_event);
        }
        while let Ok(contact_force_event) = self.contact_force_reciever.try_recv() {
            contact_force_events.push(contact_force_event);
        }
        (collision_events, contact_force_events)
    }

    /// Add friction between the object and the world
    /// Useful for top-down perspective
    ///
    /// - body: The body to which friction will be added
    /// - coefficient: The coefficient of friction between the body and the world.
    ///                This will apply force scaled by the body's mass.
    /// - torque: Affects how strongly spinning is stopped.
    pub fn add_world_friction(&mut self, body: RigidBodyHandle, coefficient: f32, torque: f32)
    {
        if let Some(rigid_body) = self.rigid_body_set.get(body) {
            let force = coefficient * rigid_body.mass();
            let com = rigid_body.center_of_mass() - rigid_body.translation();
            self.add_world_friction_point(body, vec2(com.x - torque, com.y), force / 2.0);
            self.add_world_friction_point(body, vec2(com.x + torque, com.y), force / 2.0);
        }
    }

    // Adds a single point of friction. This does not affect rotation
    fn add_world_friction_point(&mut self, body: RigidBodyHandle, point: Vec2, force: f32) {
        let joint = GenericJointBuilder::new(JointAxesMask::empty())
            .motor_velocity(JointAxis::LinX, 0.0, 0.0)
            .motor_max_force(JointAxis::LinX, force)
            .motor_velocity(JointAxis::LinY, 0.0, 0.0)
            .motor_max_force(JointAxis::LinY, force)
            // AngX isn't set, but storing the force there prevents drift
            .motor_max_force(JointAxis::AngX, force)
            .local_anchor2([point.x, point.y].into());
        self.impulse_joint_set.insert(self.friction_anchor, body, joint, false);
    }

    /// Draw the colliders of the simulation onto camera space
    ///
    /// This will match physics space to camera space.
    pub fn draw_colliders(&self, color: Color, stroke: f32) {
        for (_, collider) in self.collider_set.iter() {
            let shape = collider.shape();
            let position = collider.position();
            draw_shape(shape, position, color, stroke);
        }
    }
}

/// Utility structure that associates a texture with a rigid body
///
/// Designed to render such that camera units match physics units
#[derive(Debug, Clone)]
pub struct PhysicsSprite {
    /// The appearance of the sprite, centered on the origin
    pub texture: Texture2D,
    /// The size of the sprite, irrespective of texture resolution
    pub size: Vec2,
    /// The physical body for this sprite
    pub body: RigidBodyHandle
}

impl PhysicsSprite {
    /// Draws self as a texture centered on the origin
    ///
    /// * `simulation` the physical world in which this object resides
    /// * `y_down` set true if the y axis increases downwards
    pub fn draw(&self, simulation: &PhysicsSimulation, y_up: bool) {
        if let Some(body) = simulation.rigid_body_set.get(self.body) {
            let body_position = body.position();
            let body_translation = body_position.translation;
            let mut size = self.size;
            let body_rotation = body_position.rotation.angle();
            if y_up {size *= vec2(1.0, -1.0)}
            draw_texture_ex(
                &self.texture,
                body_translation.x - size.x * 0.5,
                body_translation.y - size.y * 0.5,
                WHITE,
                DrawTextureParams{
                    rotation: body_rotation,
                    dest_size: Some(size),
                    .. Default::default()
                });
        }
    }
}

/// Draws a shape based on data obtained from a collider
pub fn draw_shape(shape: &dyn Shape, position: &Isometry<f32>, color: Color, stroke: f32) {
    let translation = position.translation;
    let center = vec2(translation.x, translation.y);
    let angle = position.rotation.angle();
    match shape.as_typed_shape() {
        TypedShape::Ball(ball) => {
            draw_circle_lines(center.x, center.y, ball.radius, stroke, color);
        },
        TypedShape::Cuboid(cuboid) => {
            let he = cuboid.half_extents;
            draw_rectangle_lines_ex(
                center.x,
                center.y,
                he.x * 2.0,
                he.y * 2.0, stroke,
                DrawRectangleParams {
                    rotation: angle,
                    color,
                    offset: vec2(0.5, 0.5)
                }
            );
        },
        TypedShape::ConvexPolygon(polygon) => {
            let mut vertices: Vec<Vec2> = Vec::new();
            for point in polygon.points() {
                vertices.push(vec2(point.x, point.y).rotate(vec2(angle.cos(), angle.sin())));
            }
            for i in 0..vertices.len() {
                let point = center + vertices[i];
                let next_point = center + vertices[(i+1) % vertices.len()];
                draw_line(point.x, point.y, next_point.x, next_point.y, stroke, color);
            }
        },
        TypedShape::Compound(compound) => {
            let shapes = compound.shapes();
            for (isometry, shape) in shapes {
                draw_shape(shape.as_ref(), &(isometry * position), color, stroke);
            }
        },
        _ => {
            let aabb = shape.compute_aabb(position);
            let r = Rect::new(
                aabb.mins.x,
                aabb.mins.y,
                aabb.half_extents().x * 2.0,
                aabb.half_extents().y * 2.0);
            draw_rectangle_lines(r.x, r.y, r.w, r.h, stroke, color);
        }
    }
}

/// Makes a polygonal collider from a slice of vertices
///
/// Vertices must be in clockwise order for a downwards y axis (macroquad's
/// default camera), or counter-clockwise order for an upwards y axis
/// The polygon will be approximated with concave shapes, but may not be
/// exact.
pub fn polygon_collider<T: Clone + Into<[f32; 2]>>(polygon: &[T]) -> ColliderBuilder {
    let vertices: Vec<Point<Real>> = polygon.iter()
        .map(|p| {
            let c: [f32; 2] = p.to_owned().into();
            Point::new(c[0], c[1])
        }).collect();
    let indices: Vec<[u32; 2]> = (0..vertices.len() as u32)
        .map(|i| [i, (i + 1) % vertices.len() as u32 ]).collect();
    ColliderBuilder::convex_decomposition_with_params(
        &vertices,
        &indices,
        &VHACDParameters {
            concavity: 0.01,
            ..Default::default()
        }
    )
}