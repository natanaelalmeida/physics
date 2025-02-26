extern crate nalgebra as na;
use na::{Vector2};
use std::f32::consts::PI;

pub fn simulate_mrua(x0: f32, v0: f32, a: f32, dt: f32, steps: usize) {
    let mut x = x0;
    let mut v = v0;

    for step in 0..steps {
        x += v * dt + 0.5 * a * dt.powi(2);
        v += a * dt;
        println!("Step {}: Position = {:.3}, Velocity = {:.3}", step, x, v);
    }
}


pub fn simulate_pendulum(theta0: f32, length: f32, dt: f32, steps: usize) {
    let g: f32 = 9.81;
    let mut theta = theta0;
    let mut omega = 0.0;

    for step in 0..steps {
        let alpha = -g / length * theta.sin();
        theta += omega * dt + 0.5 * alpha * dt.powi(2);

        let alpha_new = -g / length * theta.sin();
        omega += 0.5 * (alpha * alpha_new) * dt;

        println!("Step {}: Angle = {:.3} rad, Angle Velocity = {:.3} rad/s", step, theta, omega);
    }
}

pub fn simulate_body(mass: f32, force: Vector2<f32>, drag_coeff: f32, dt: f32, steps: usize) {
    let mut position = Vector2::new(0.0, 0.0);
    let mut velocity = Vector2::new(0.0, 0.0);

    for step in 0..steps {
        let drag = -drag_coeff * velocity;
        let total_force = force + drag;
        let acceleration = total_force / mass;

        velocity += acceleration * dt;
        position += velocity * dt;

        println!("Step {}: Position = ({:.3}, {:.3}), Velocity = ({:.3}, {:.3})", step, position.x, position.y, velocity.x, velocity.y);
    }
}

pub fn simulate_collisions(pos1: &mut Vector2<f64>, vel1: &mut Vector2<f64>, pos2: &mut Vector2<f64>, vel2: &mut Vector2<f64>, dt: f64) {
    let distance = (*pos1 - *pos2).norm();
    if distance < 1.0 {
        let temp = *vel1;
        *vel1 = *vel2;
        *vel2 = temp
    }

    *pos1 += *vel1 * dt;
    *pos2 += *vel2 * dt;
}

pub fn optimize_trajectory(start: Vector2<f64>, target: Vector2<f64>, learning_rate: f64, iterations: usize) -> Vector2<f64> {
    let mut position = start;
    for _ in 0..iterations {
        let gradient = (target - position).normalize();
        position += gradient * learning_rate;
    }

    position
}

pub fn rk4_step(f: fn(f64, f64) -> f64, t: f64, y: f64, dt: f64) -> f64 {
    let k1 = dt * f(t, y);
    let k2 = dt * f(t + dt / 2.0, y + k1 / 2.0);
    let k3 = dt * f(t + dt / 2.0, y + k2 / 2.0);
    let k4 = dt * f(t + dt, y + k3);
    y + (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
}

