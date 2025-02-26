mod kinematics;

use std::f32::consts::PI;
use nalgebra::Vector2;
use kinematics::*;

fn main() {
    println!("\n--- MRUA Simulation ---");
    simulate_mrua(0.0, 10.0, -9.81, 0.1, 10);

    println!("\n--- Pendulum Simulation ---");
    simulate_pendulum(PI / 4.0, 1.0, 0.1, 10);

    println!("\n--- Body Simulation ---");
    let force = Vector2::<f32>::new(10.0, 0.0);
    simulate_body(2.0, force, 0.1, 0.1, 10);

    println!("\n--- Collision Simulation ---");
    let mut pos1 = Vector2::new(0.0, 0.0);
    let mut vel1 = Vector2::new(1.0, 0.0);
    let mut pos2 = Vector2::new(1.0, 0.0);
    let mut vel2 = Vector2::new(-1.0, 0.0);
    simulate_collisions(&mut pos1, &mut vel1, &mut pos2, &mut vel2, 0.1);

    println!("\n--- Optimize Trajectory ---");
    let optimized_position = optimize_trajectory(Vector2::new(0.0, 0.0), Vector2::new(10.0, 10.0), 0.1, 100);
    println!("Optimized Position: ({:.3}, {:.3})", optimized_position.x, optimized_position.y);
}
