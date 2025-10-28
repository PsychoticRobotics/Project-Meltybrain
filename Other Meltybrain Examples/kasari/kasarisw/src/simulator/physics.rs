use kasarisw::shared::rem_euclid_f32;
use libm::{cosf, sinf};
use std::f32::consts::PI;

#[derive(Clone, Copy)]
pub struct Rect {
    pub min_x: f32,
    pub min_y: f32,
    pub max_x: f32,
    pub max_y: f32,
}

pub struct World {
    pub arena: Rect,
    pub objects: Vec<Rect>,
}

impl World {
    pub fn raycast_to_rect(
        &self,
        pos_x: f32,
        pos_y: f32,
        dir_x: f32,
        dir_y: f32,
        rect: &Rect,
    ) -> Option<f32> {
        let mut min_t = f32::INFINITY;

        // Left wall
        if dir_x != 0.0 {
            let t = (rect.min_x - pos_x) / dir_x;
            if t > 0.0 {
                let hit_y = pos_y + t * dir_y;
                if hit_y >= rect.min_y && hit_y <= rect.max_y {
                    min_t = min_t.min(t);
                }
            }
        }

        // Right wall
        if dir_x != 0.0 {
            let t = (rect.max_x - pos_x) / dir_x;
            if t > 0.0 {
                let hit_y = pos_y + t * dir_y;
                if hit_y >= rect.min_y && hit_y <= rect.max_y {
                    min_t = min_t.min(t);
                }
            }
        }

        // Bottom wall
        if dir_y != 0.0 {
            let t = (rect.min_y - pos_y) / dir_y;
            if t > 0.0 {
                let hit_x = pos_x + t * dir_x;
                if hit_x >= rect.min_x && hit_x <= rect.max_x {
                    min_t = min_t.min(t);
                }
            }
        }

        // Top wall
        if dir_y != 0.0 {
            let t = (rect.max_y - pos_y) / dir_y;
            if t > 0.0 {
                let hit_x = pos_x + t * dir_x;
                if hit_x >= rect.min_x && hit_x <= rect.max_x {
                    min_t = min_t.min(t);
                }
            }
        }

        if min_t.is_infinite() {
            None
        } else {
            Some(min_t)
        }
    }

    pub fn raycast(&self, pos_x: f32, pos_y: f32, dir_x: f32, dir_y: f32) -> f32 {
        let mut min_t = f32::INFINITY;

        if let Some(t) = self.raycast_to_rect(pos_x, pos_y, dir_x, dir_y, &self.arena) {
            min_t = min_t.min(t);
        }

        for obj in &self.objects {
            if let Some(t) = self.raycast_to_rect(pos_x, pos_y, dir_x, dir_y, obj) {
                min_t = min_t.min(t);
            }
        }

        if min_t.is_infinite() {
            3000.0
        } else {
            min_t
        }
    }
}

pub struct Robot {
    pub pos_x: f32,
    pub pos_y: f32,
    pub vel_x: f32,
    pub vel_y: f32,
    pub theta: f32,
    pub rpm: f32,
}

impl Robot {
    pub fn update(
        &mut self,
        dt: f32,
        movement_x: f32,
        movement_y: f32,
        world: &World,
        detector_theta: f32,
        angular_correction: f32,
    ) {
        self.theta += (self.rpm / 60.0 * 2.0 * PI) * dt;
        self.theta = rem_euclid_f32(self.theta, 2.0 * PI);

        // Dynamic offset: theta difference + angular correction
        let delta_theta = -(detector_theta - self.theta);
        let target_movement_angle_offset: f32 = PI * 0.0 + delta_theta + angular_correction;

        let cos_off = cosf(target_movement_angle_offset);
        let sin_off = sinf(target_movement_angle_offset);
        let mut rotated_x = movement_x * cos_off - movement_y * sin_off;
        let mut rotated_y = movement_x * sin_off + movement_y * cos_off;
        // Mimick the position control deadzone of the real robot
        if rotated_x.abs() < 0.10 {
            rotated_x = 0.0;
        } else {
            rotated_x -= rotated_x.signum() * 0.10;
        }
        if rotated_y.abs() < 0.10 {
            rotated_y = 0.0;
        } else {
            rotated_y -= rotated_y.signum() * 0.10;
        }

        const ACCEL_CONST: f32 = 500.0; // mm/s² per unit mag
        const DRAG_CONST: f32 = 0.2;
        let accel_x = ACCEL_CONST * rotated_x;
        let accel_y = ACCEL_CONST * rotated_y;
        self.vel_x += (accel_x - self.vel_x * DRAG_CONST) * dt;
        self.vel_y += (accel_y - self.vel_y * DRAG_CONST) * dt;

        // Limit velocity to keep the collision checks working with the fixed
        // time step
        self.vel_x = self.vel_x.max(-20000.0).min(20000.0);
        self.vel_y = self.vel_y.max(-20000.0).min(20000.0);

        self.pos_x += self.vel_x * dt;
        self.pos_y += self.vel_y * dt;

        const HALF_SIZE: f32 = 70.0;
        const ELASTICITY: f32 = 0.5;
        const TANGENTIAL_KICK: f32 = 0.7;
        const KICK_RPM_FACTOR: f32 = 0.3;

        // Handle arena collisions
        let mut robot_min_x = self.pos_x - HALF_SIZE;
        let mut robot_min_y = self.pos_y - HALF_SIZE;
        let mut robot_max_x = self.pos_x + HALF_SIZE;
        let mut robot_max_y = self.pos_y + HALF_SIZE;

        if robot_min_x < world.arena.min_x {
            let pen = world.arena.min_x - robot_min_x;
            self.pos_x += pen;
            self.vel_x = -self.vel_x * ELASTICITY;
            self.vel_y += self.rpm * TANGENTIAL_KICK;
            self.rpm *= KICK_RPM_FACTOR;
        }
        if robot_max_x > world.arena.max_x {
            let pen = robot_max_x - world.arena.max_x;
            self.pos_x -= pen;
            self.vel_x = -self.vel_x * ELASTICITY;
            self.vel_y -= self.rpm * TANGENTIAL_KICK;
            self.rpm *= KICK_RPM_FACTOR;
        }
        if robot_min_y < world.arena.min_y {
            let pen = world.arena.min_y - robot_min_y;
            self.pos_y += pen;
            self.vel_y = -self.vel_y * ELASTICITY;
            self.vel_x -= self.rpm * TANGENTIAL_KICK;
            self.rpm *= KICK_RPM_FACTOR;
        }
        if robot_max_y > world.arena.max_y {
            let pen = robot_max_y - world.arena.max_y;
            self.pos_y -= pen;
            self.vel_y = -self.vel_y * ELASTICITY;
            self.vel_x += self.rpm * TANGENTIAL_KICK;
            self.rpm *= KICK_RPM_FACTOR;
        }

        // Update robot bounds after arena collision
        robot_min_x = self.pos_x - HALF_SIZE;
        robot_min_y = self.pos_y - HALF_SIZE;
        robot_max_x = self.pos_x + HALF_SIZE;
        robot_max_y = self.pos_y + HALF_SIZE;

        // Handle object collisions
        for obj in &world.objects {
            if robot_max_x > obj.min_x
                && robot_min_x < obj.max_x
                && robot_max_y > obj.min_y
                && robot_min_y < obj.max_y
            {
                // Overlapping
                let pen_x_left = robot_max_x - obj.min_x;
                let pen_x_right = obj.max_x - robot_min_x;
                let pen_y_bottom = robot_max_y - obj.min_y;
                let pen_y_top = obj.max_y - robot_min_y;

                let mut min_pen = f32::INFINITY;
                let mut resolve_dx = 0.0;
                let mut resolve_dy = 0.0;
                let mut normal_x = 0.0;
                let mut normal_y = 0.0;

                if pen_x_left > 0.0 && pen_x_left < min_pen {
                    min_pen = pen_x_left;
                    resolve_dx = -pen_x_left;
                    normal_x = -1.0;
                }
                if pen_x_right > 0.0 && pen_x_right < min_pen {
                    min_pen = pen_x_right;
                    resolve_dx = pen_x_right;
                    normal_x = 1.0;
                }
                if pen_y_bottom > 0.0 && pen_y_bottom < min_pen {
                    min_pen = pen_y_bottom;
                    resolve_dy = -pen_y_bottom;
                    normal_y = -1.0;
                }
                if pen_y_top > 0.0 && pen_y_top < min_pen {
                    min_pen = pen_y_top;
                    resolve_dy = pen_y_top;
                    normal_y = 1.0;
                }

                self.pos_x += resolve_dx;
                self.pos_y += resolve_dy;

                if normal_x != 0.0 {
                    self.vel_x = -self.vel_x * ELASTICITY;
                    self.vel_y += normal_x * self.rpm * TANGENTIAL_KICK;
                    self.rpm *= KICK_RPM_FACTOR;
                }
                if normal_y != 0.0 {
                    self.vel_y = -self.vel_y * ELASTICITY;
                    self.vel_x -= normal_y * self.rpm * TANGENTIAL_KICK;
                    self.rpm *= KICK_RPM_FACTOR;
                }

                // Update bounds for potential next objects
                robot_min_x = self.pos_x - HALF_SIZE;
                robot_min_y = self.pos_y - HALF_SIZE;
                robot_max_x = self.pos_x + HALF_SIZE;
                robot_max_y = self.pos_y + HALF_SIZE;
            }
        }
    }
}
