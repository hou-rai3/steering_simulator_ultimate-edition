import math
import pygame
import pygame.locals
import sys
from multiprocessing import Manager, Process

def run_simulation(shared_data):
  pygame.init()
  pygame.display.set_mode((800, 600))
  pygame.display.set_caption("Advanced Swerve Drive Simulation")
  surface = pygame.display.get_surface()
  font = pygame.font.SysFont(None, 24)

  class Robot:
    def __init__(self):
      # --- Chassis State ---
      self.x, self.y = 400.0, 300.0
      self.angle = -90.0
      self.vx, self.vy = 0.0, 0.0
      self.angular_velocity = 0.0

      # --- Control Inputs ---
      self.forward_input = 0.0
      self.strafe_input = 0.0
      self.turn_input = 0.0

      # --- Physics Parameters ---
      self.max_speed = 4.0
      self.max_angular_velocity = 200.0
      self.acceleration = 15.0
      self.angular_acceleration = 400.0
      self.deadband_threshold = 0.1
      self.friction = 0.98

      # --- PID for Rotation ---
      self.integral_angular = 0.0
      self.prev_error_angular = 0.0

      # --- Swerve Wheels ---
      wheel_dist_width = 30
      wheel_dist_length = 30
      self.wheels = [
          {'pos': [-wheel_dist_length, -wheel_dist_width],
              'steer_angle': 0.0, 'velocity_vector': (0, 0)},
          {'pos': [wheel_dist_length, -wheel_dist_width],
              'steer_angle': 0.0, 'velocity_vector': (0, 0)},
          {'pos': [-wheel_dist_length, wheel_dist_width],
              'steer_angle': 0.0, 'velocity_vector': (0, 0)},
          {'pos': [wheel_dist_length, wheel_dist_width],
              'steer_angle': 0.0, 'velocity_vector': (0, 0)},
      ]
      self.max_steer_rate = 480.0

    def normalize_angle(self, angle):
      return (angle + 180) % 360 - 180

    def apply_deadband(self, value):
      return 0.0 if abs(value) < self.deadband_threshold else value

    def update_control_input(self, keys):
      # Left Stick (WASD)
      fwd = 1.0 if keys[pygame.K_w] else -1.0 if keys[pygame.K_s] else 0.0
      strafe = 1.0 if keys[pygame.K_d] else - \
          1.0 if keys[pygame.K_a] else 0.0
      vec = pygame.math.Vector2(strafe, -fwd)
      if vec.length_squared() > 1.0: vec.normalize_ip()
      self.strafe_input = self.apply_deadband(vec.x)
      self.forward_input = self.apply_deadband(-vec.y)

      # Right Stick (Arrows)
      self.turn_input = self.apply_deadband(
          1.0 if keys[pygame.K_RIGHT] else -1.0 if keys[pygame.K_LEFT] else 0.0)

    def update_wheels(self, dt):
      # Convert world velocity to robot's local frame
      angle_rad = math.radians(-self.angle)
      cos_a, sin_a = math.cos(angle_rad), math.sin(angle_rad)
      vx_local = self.vx * cos_a - self.vy * sin_a
      vy_local = self.vx * sin_a + self.vy * cos_a

      chassis_vx_pps = vx_local * 60
      chassis_vy_pps = vy_local * 60
      chassis_omega_rad = math.radians(self.angular_velocity)

      for wheel in self.wheels:
        px, py = wheel['pos']
        vel_x = chassis_vx_pps - chassis_omega_rad * py
        vel_y = chassis_vy_pps + chassis_omega_rad * px

        # Store velocity vector for visualization
        wheel['velocity_vector'] = (vel_x, vel_y)

        if abs(vel_x) < 0.1 and abs(vel_y) < 0.1: continue

        target_angle = math.degrees(math.atan2(vel_y, vel_x))
        current_steer_angle = wheel['steer_angle']
        angle_error = self.normalize_angle(
            target_angle - current_steer_angle)

        if abs(angle_error) > 90:
          target_angle = self.normalize_angle(target_angle + 180)
          angle_error = self.normalize_angle(
              target_angle - current_steer_angle)

        max_turn_this_frame = self.max_steer_rate * dt
        turn_amount = max(-max_turn_this_frame,
                          min(max_turn_this_frame, angle_error))
        wheel['steer_angle'] = self.normalize_angle(
            current_steer_angle + turn_amount)

    def update_state(self, dt):
      # Rotation
      kp, ki, kd = 1.09, 0.0, 0.001
      target_angular_velocity = self.turn_input * self.max_angular_velocity
      error_angular = target_angular_velocity - self.angular_velocity
      self.integral_angular += error_angular * dt
      self.prev_error_angular = error_angular
      angular_control = (kp * error_angular + ki * self.integral_angular +
                         kd * (error_angular - self.prev_error_angular) / dt) if dt > 0 else 0
      self.angular_velocity += max(-self.angular_acceleration *
                                   dt, min(self.angular_acceleration * dt, angular_control))
      self.angular_velocity = max(-self.max_angular_velocity,
                                  min(self.max_angular_velocity, self.angular_velocity))
      self.angle = self.normalize_angle(
          self.angle + self.angular_velocity * dt)

      # Translation
      angle_rad = math.radians(self.angle)
      cos_a, sin_a = math.cos(angle_rad), math.sin(angle_rad)
      target_vx_local = self.forward_input * self.max_speed
      target_vy_local = self.strafe_input * self.max_speed
      target_vx = target_vx_local * cos_a - target_vy_local * sin_a
      target_vy = target_vx_local * sin_a + target_vy_local * cos_a
      self.vx += (target_vx - self.vx) * self.acceleration * dt
      self.vy += (target_vy - self.vy) * self.acceleration * dt
      self.vx *= self.friction
      self.vy *= self.friction
      self.x += self.vx
      self.y += self.vy
      self.x = max(50, min(750, self.x))
      self.y = max(50, min(550, self.y))

      self.update_wheels(dt)

  robot = Robot()
  clock = pygame.time.Clock()

  while True:
    dt = clock.tick(60) / 1000.0
    if dt == 0: continue

    for event in pygame.event.get():
      if event.type == pygame.locals.QUIT: pygame.quit(); sys.exit()

    keys = pygame.key.get_pressed()
    robot.update_control_input(keys)
    robot.update_state(dt)

    # --- Drawing ---
    surface.fill((255, 255, 255))

    # Draw Wheels and Velocity Vectors (Green Lines)
    tire_length, tire_width = 22, 12
    base_tire_points = [(-tire_length / 2, -tire_width / 2), (tire_length / 2, -tire_width / 2),
                        (tire_length / 2, tire_width / 2), (-tire_length / 2, tire_width / 2)]

    for wheel in robot.wheels:
      body_angle_rad = math.radians(robot.angle)
      cos_b, sin_b = math.cos(body_angle_rad), math.sin(body_angle_rad)
      wheel_center_x = robot.x + \
          (wheel['pos'][0] * cos_b - wheel['pos'][1] * sin_b)
      wheel_center_y = robot.y + \
          (wheel['pos'][0] * sin_b + wheel['pos'][1] * cos_b)

      # Draw Velocity Vector (Green Line) from each tire
      vx_local, vy_local = wheel['velocity_vector']
      if abs(vx_local) > 0.1 or abs(vy_local) > 0.1:
        # Rotate local vector to world frame
        vx_world = (vx_local * cos_b - vy_local * sin_b)
        vy_world = (vx_local * sin_b + vy_local * cos_b)
        # Draw line with scaling factor
        scale = 0.15
        line_end_x = wheel_center_x + vx_world * scale
        line_end_y = wheel_center_y + vy_world * scale
        pygame.draw.line(surface, (0, 200, 0), (wheel_center_x,
                         wheel_center_y), (line_end_x, line_end_y), 2)

      # Draw Tire (Rectangle)
      wheel_world_angle_rad = math.radians(
          robot.angle + wheel['steer_angle'])
      cos_w, sin_w = math.cos(wheel_world_angle_rad), math.sin(
          wheel_world_angle_rad)
      rotated_points = [(p_x * cos_w - p_y * sin_w + wheel_center_x, p_x *
                         sin_w + p_y * cos_w + wheel_center_y) for p_x, p_y in base_tire_points]
      pygame.draw.polygon(surface, (0, 0, 0), rotated_points, 2)
      pygame.draw.polygon(surface, (60, 60, 60),
                          rotated_points)  # Fill with dark grey

    # Draw robot body
    robot_size = 60
    body_surf = pygame.Surface((robot_size, robot_size), pygame.SRCALPHA)
    pygame.draw.rect(body_surf, (0, 0, 255, 200),
                     (0, 0, robot_size, robot_size), border_radius=5)
    pygame.draw.line(body_surf, (255, 255, 0), (robot_size / 2,
                     robot_size / 2), (robot_size, robot_size / 2), 4)
    rotated_body = pygame.transform.rotate(body_surf, -robot.angle)
    surface.blit(rotated_body, rotated_body.get_rect(
        center=(robot.x, robot.y)))

    # Display debug info
    debug_info = [
        "Left Stick (WASD): Translate (Move)",
        "Right Stick (←→): Rotate",
        f"Position: ({robot.x:.1f}, {robot.y:.1f})",
        f"Angle: {robot.angle:.1f}°",
    ]
    for i, text in enumerate(debug_info):
      surface.blit(font.render(text, True, (0, 0, 0)), (10, 10 + i * 20))

    pygame.display.update()

if __name__ == "__main__":
  manager = Manager()
  shared_data = manager.dict()
  sim_process = Process(target=run_simulation, args=(shared_data,))
  sim_process.start()
  sim_process.join()
