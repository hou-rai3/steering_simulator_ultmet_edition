import math
import time
import pygame
import pygame.locals
import sys
import random
from multiprocessing import Manager, Process

def run_simulation(shared_data):
  pygame.init()
  pygame.display.set_mode((640, 480))
  pygame.display.set_caption("Independent Steering Simulation")
  surface = pygame.display.get_surface()
  surface.fill((0, 128, 0))
  font = pygame.font.SysFont(None, 24)

  class Vehicle:
    def __init__(self):
      self.x, self.y = 320.0, 240.0
      self.left_wheel_angle = 0.0
      self.left_wheel_speed = 0.0
      self.target_x, self.target_y = self.x, self.y

      # PID制御用内部状態
      self.integral_angle = 0.0
      self.integral_speed = 0.0
      self.pre_error_angle = 0.0
      self.pre_error_speed = 0.0

    def update_target(self, target_x, target_y):
      self.target_x, self.target_y = target_x, target_y

    def calculate_target(self):
      dx, dy = self.target_x - self.x, self.target_y - self.y
      target_angle = math.degrees(math.atan2(dy, dx))
      target_speed = math.sqrt(dx**2 + dy**2) / 30.0
      return target_angle, target_speed

    def update_state(self, dt):
      target_angle, target_speed = self.calculate_target()

      # GUI からリアルタイムに更新された PID ゲインを取得
      kp_angle, ki_angle, kd_angle = shared_data["kp"], shared_data["ki"], shared_data["kd"]

      # 角度 PID 制御
      error = target_angle - self.left_wheel_angle
      self.integral_angle += error * dt
      deriv = (error - self.pre_error_angle) / dt
      pid_output = kp_angle * error + ki_angle * \
          self.integral_angle + kd_angle * deriv
      self.pre_error_angle = error
      self.left_wheel_angle += pid_output

      # 速度 PID 制御
      error = target_speed - self.left_wheel_speed
      self.integral_speed += error * dt
      deriv = (error - self.pre_error_speed) / dt
      pid_output = shared_data["kp_speed"] * error + shared_data["ki_speed"] * \
          self.integral_speed + shared_data["kd_speed"] * deriv
      self.pre_error_speed = error

      # 空転・スリップのシミュレーション
      slip_factor = random.uniform(0.9, 1.1)  # ±10%のランダム誤差
      self.left_wheel_speed += pid_output * slip_factor

      # 外乱要素の追加
      self.left_wheel_angle += random.uniform(-0.5, 0.5)  # ±0.5度のランダム誤差

      # 位置更新
      self.x += math.cos(math.radians(self.left_wheel_angle)
                         ) * self.left_wheel_speed
      self.y += math.sin(math.radians(self.left_wheel_angle)
                         ) * self.left_wheel_speed

  vehicle = Vehicle()
  clock = pygame.time.Clock()

  while True:
    dt = clock.tick(60) / 1000.0
    for event in pygame.event.get():
      if event.type == pygame.locals.QUIT:
        pygame.quit()
        sys.exit()
      if event.type == pygame.locals.MOUSEMOTION:
        vehicle.update_target(*event.pos)

    # 車体の状態を更新
    vehicle.update_state(dt)

    # 画面描画
    surface.fill((0, 128, 0))

    # タイヤの相対座標
    wheel_offsets = [(-30, -30), (-30, 30), (30, -30), (30, 30)]

    # 車体中央とタイヤを描画
    for offset_x, offset_y in wheel_offsets:
      tire_x = vehicle.x + offset_x
      tire_y = vehicle.y + offset_y
      pygame.draw.circle(surface, (255, 255, 255),
                         (int(tire_x), int(tire_y)), 10)  # タイヤの描画

      # 各タイヤから赤い線を描画
      line_end_x = tire_x + \
          math.cos(math.radians(vehicle.left_wheel_angle)) * 30
      line_end_y = tire_y + \
          math.sin(math.radians(vehicle.left_wheel_angle)) * 30
      pygame.draw.line(surface, (255, 0, 0), (int(tire_x), int(
          tire_y)), (int(line_end_x), int(line_end_y)), 3)

    # デバッグ情報を表示
    debug_info = [
        f"Target: ({vehicle.target_x:.2f}, {vehicle.target_y:.2f})",
        f"Position: ({vehicle.x:.2f}, {vehicle.y:.2f})",
        f"Target Angle: {vehicle.calculate_target()[0]:.2f}°",
        f"Wheel Angle: {vehicle.left_wheel_angle:.2f}°",
        f"Target Speed: {vehicle.calculate_target()[1]:.2f}",
        f"Wheel Speed: {vehicle.left_wheel_speed:.2f}",
    ]
    for i, text in enumerate(debug_info):
      debug_surface = font.render(text, True, (255, 255, 255))
      surface.blit(debug_surface, (10, 10 + i * 20))

    pygame.display.update()

if __name__ == "__main__":
  manager = Manager()
  shared_data = manager.dict(
      kp=1.09, ki=0.0, kd=0.001, kp_speed=0.19, ki_speed=0.0, kd_speed=0.0)

  sim_process = Process(target=run_simulation, args=(shared_data,))
  sim_process.start()

  from GUI import run_gui  # GUI の起動
  run_gui(shared_data)

  sim_process.join()
