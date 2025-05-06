use macroquad::prelude::*;

#[derive(Clone, Debug)]
pub struct Leg {
    pub hip: Vec3,
    pub foot: Vec3,
}

#[derive(Clone, Debug)]
pub struct QuadrupedJoints {
    pub body_center: Vec3,
    pub body_size: Vec3,
    pub head_offset: Vec3,
    pub head_size: Vec3,
    pub legs: [Leg; 4], // FL, FR, BL, BR
}


fn draw_quadruped(joints: &QuadrupedJoints) {
    let joint_size = vec3(0.15, 0.15, 0.15);

    // Draw body
    draw_cube(joints.body_center, joints.body_size, None, WHITE);

    // Draw head
    draw_cube(
        joints.body_center + joints.head_offset,
        joints.head_size,
        None,
        WHITE,
    );

    let leg_steps = 10;
    let leg_thickness = 0.1;

    for leg in &joints.legs {
        // Draw hip and foot joints
        draw_cube(leg.hip, joint_size, None, WHITE);
        draw_cube(leg.foot, joint_size, None, WHITE);

        // Interpolated leg segments
        for j in 0..leg_steps {
            let t = j as f32 / leg_steps as f32;
            let pos = leg.hip.lerp(leg.foot, t);
            draw_cube(pos, vec3(leg_thickness, leg_thickness, leg_thickness), None, WHITE);
        }
    }
}


#[macroquad::main("Dynamic Quadruped")]
async fn main() {
    loop {
        clear_background(BLACK);

        set_camera(&Camera3D {
            position: vec3(6.0, 6.0, 12.0),
            up: vec3(0.0, 1.0, 0.0),
            target: vec3(0.0, 1.0, 0.0),
            ..Default::default()
        });

        // Example pose
        let quadruped = QuadrupedJoints {
            body_center: vec3(0.0, 1.5, 0.0),
            body_size: vec3(3.0, 1.0, 1.0),
            head_offset: vec3(2.0, 0.0, 0.0),
            head_size: vec3(0.6, 0.6, 0.6),
            legs: [
                Leg { hip: vec3(-1.0, 1.0, -0.3), foot: vec3(-1.0, 0.0, -0.3) }, // FL
                Leg { hip: vec3(-1.0, 1.0,  0.3), foot: vec3(-1.0, 0.0,  0.3) }, // FR
                Leg { hip: vec3( 1.0, 1.0, -0.3), foot: vec3( 1.0, 0.0, -0.3) }, // BL
                Leg { hip: vec3( 1.0, 1.0,  0.3), foot: vec3( 1.0, 0.0,  0.3) }, // BR
            ],
        };

        draw_quadruped(&quadruped);
        draw_grid(10, 1.0, GRAY, DARKGRAY);
        set_default_camera();
        next_frame().await;
    }
}
