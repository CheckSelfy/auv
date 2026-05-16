#!/usr/bin/env python3
"""AUV PID Autopilot v34.1 | Anti-Capsize System | Safe Velocity Orbit"""
import rclpy, math, time, sys
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

P_Z0 = 101325.0
RHO_G = 9810.0

ORBIT_RADIUS = 15.0  
PREDICTIVE_ZONE = ORBIT_RADIUS * 1.5  

class AUVController(Node):
    def __init__(self):
        super().__init__('auv_ctrl')
        self.pub_lt = self.create_publisher(Float64, '/model/submarine/joint/left_propeller_joint/cmd_force', 10)
        self.pub_rt = self.create_publisher(Float64, '/model/submarine/joint/right_propeller_joint/cmd_force', 10)
        self.pub_vert = self.create_publisher(Float64, '/model/submarine/joint/vertical_rudder/cmd_position', 10)
        self.pub_hl = self.create_publisher(Float64, '/model/submarine/joint/horizontal_rudder_left/cmd_position', 10)
        self.pub_hr = self.create_publisher(Float64, '/model/submarine/joint/horizontal_rudder_right/cmd_position', 10)
        
        self.create_subscription(Odometry, '/model/submarine/odometry', self.odom_cb, 10)
        
        qos_s = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Float32, '/model/submarine/pressure', self.press_cb, qos_s)

        self.state = 'INIT'
        self.pos = [0.0, 0.0, 0.0]  
        self.baro_z = 0.0
        self.vel = 0.0
        self.rpy = [0.0, 0.0, 0.0]
        self.prev_rpy = [0.0, 0.0, 0.0]
        self.target_global = [0.0, 0.0, 0.0]
        self.bearing = 0.0
        self.dist_2d = 1000.0
        self.prev_baro_z = 0.0
        
        # Настройки маршевой скорости
        self.max_cruise_speed = 2.5  
        self.min_cruise_speed = 0.8  
        self.brake_threshold = 0.2
  
        # ПИД Z (Задемфированный, чтобы не раскачивать нос)
        self.Kp_z = 4.2; self.Kd_z = 1.7
        
        # Базовый курс
        self.Kp_yaw = 1.8; self.Kd_yaw = 0.5
        self.K_diff_base = 3.0  # Умеренный базовый дифференциал

        # 🔥 ЭКСТРЕМАЛЬНАЯ СТАБИЛИЗАЦИЯ КРЕНА (Защита от переворота)
        self.Kp_roll = 16.0; self.Kd_roll = 5.0
        self.roll_bias = 0.04
        
        self.stable_t = 0.0; self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.loop)

    def press_cb(self, msg):
        self.baro_z = (P_Z0 - msg.data) / RHO_G

    def odom_cb(self, msg):
        self.pos[0] = msg.pose.pose.position.x
        self.pos[1] = msg.pose.pose.position.y
        self.pos[2] = self.baro_z 
        
        self.vel = msg.twist.twist.linear.x
        q = msg.pose.pose.orientation
        self.rpy[0] = math.atan2(2*(q.w*q.x + q.y*q.z), 1-2*(q.x**2 + q.y**2))
        self.rpy[1] = math.asin(max(-1.0, min(1.0, 2*(q.w*q.y - q.z*q.x))))
        self.rpy[2] = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y**2 + q.z**2))
        
        if self.state == 'INIT':
            self.target_global = [self.raw_target_x, self.raw_target_y, self.raw_target_z]
            self.prev_rpy = list(self.rpy)
            self.prev_baro_z = self.baro_z
            self.state = 'STAB'
            print(f"\n🎯 Запуск Anti-Capsize системы v34.1:")
            print(f"   Цель: X={self.target_global[0]:.2f} Y={self.target_global[1]:.2f} Z={self.target_global[2]:.2f}")

        dx_rem = self.target_global[0] - self.pos[0]
        dy_rem = self.target_global[1] - self.pos[1]
        self.dist_2d = math.hypot(dx_rem, dy_rem)

        if self.state == 'ORBIT':
            angle_to_sub = math.atan2(self.pos[1] - self.target_global[1], self.pos[0] - self.target_global[0])
            radius_error = self.dist_2d - ORBIT_RADIUS
            correction_angle = max(-0.4, min(0.4, radius_error * 0.12))
            self.bearing = angle_to_sub + math.pi/2 + correction_angle
        else:
            self.bearing = math.atan2(dy_rem, dx_rem)

    def loop(self):
        if self.state not in ['STAB', 'NAV', 'ORBIT', 'FINAL_LOCK']: return
        
        # 🔹 ВЫЧИСЛЕНИЕ ВЫСОТЫ
        z_err = self.pos[2] - self.target_global[2] 
        dz_dt = (self.pos[2] - self.prev_baro_z) / self.dt
        raw_h = -(self.Kp_z * z_err + self.Kd_z * dz_dt)
        rudder_h = max(-0.22, min(0.22, raw_h)) 
        self.prev_baro_z = self.pos[2]

        # 🔹 ВЫЧИСЛЕНИЕ КУРСА
        yaw_err = math.atan2(math.sin(self.bearing - self.rpy[2]), math.cos(self.bearing - self.rpy[2]))
        if abs(math.degrees(yaw_err)) < 1.0: yaw_err = 0.0
        
        d_yaw = (self.rpy[2] - self.prev_rpy[2]) / self.dt
        rudder_v = (self.Kp_yaw * yaw_err + self.Kd_yaw * d_yaw)
        rudder_v = max(-0.45, min(0.45, rudder_v))

        # 🔹 МОЩНЫЙ КОНТУР СТАБИЛИЗАЦИИ КРЕНА
        roll_err = self.rpy[0]
        d_roll = (self.rpy[0] - self.prev_rpy[0]) / self.dt
        roll_pid = self.Kp_roll * roll_err + self.Kd_roll * d_roll
        
        cmd_hl = rudder_h - roll_pid - self.roll_bias
        cmd_hr = rudder_h + roll_pid + self.roll_bias
        cmd_hl = max(-0.6, min(0.6, cmd_hl))
        cmd_hr = max(-0.6, min(0.6, cmd_hr))
        self.prev_rpy = list(self.rpy)
        
        thrust = 0.0; cmd_lt = 0.0; cmd_rt = 0.0

        # ================= АВТОМАТ ТРАЕКТОРИЙ =================
        if self.state == 'STAB':
            if abs(roll_err) < 0.12: self.stable_t += self.dt
            else: self.stable_t = 0.0
            if self.stable_t >= 1.5: self.state = 'NAV'
            cmd_hl = max(-0.15, min(0.15, -roll_pid - self.roll_bias))
            cmd_hr = max(-0.15, min(0.15, roll_pid + self.roll_bias))

        elif self.state == 'NAV':
            # === УЛУЧШЕННАЯ КООРДИНАЦИЯ (скорость выше, замедление мягче) ===
            z_factor = max(0.65, 1.0 - abs(z_err) / 15.0)   # теперь замедляем только при |Z_err| > 15 м
            
            target_speed = max(self.min_cruise_speed, 
                             min(self.max_cruise_speed, self.dist_2d * 0.48))  # ← было 0.35, теперь 0.48
            target_speed *= z_factor

            if self.vel > target_speed + self.brake_threshold:
                thrust = 0.8
            else:
                thrust = -target_speed * 3.3

            # Проверка входа в орбиту (чуть раньше)
            if self.dist_2d < PREDICTIVE_ZONE * 1.3 and abs(z_err) >= 1.3:
                abs_dz = max(abs(dz_dt), 0.05)
                time_to_climb = abs(z_err) / abs_dz
                abs_vel = max(abs(self.vel), 0.12)
                time_to_target = self.dist_2d / abs_vel
                
                if time_to_climb > time_to_target * 0.9:
                    self.state = 'ORBIT'
                    print(f"\n🔮 PREDICT | Большая Z-ошибка → переходим в орбиту (скорость уже выше)")
                    sys.stdout.flush()

            # Динамический дифференциал (чуть агрессивнее на высокой скорости)
            k_diff = self.K_diff_base * (1.0 + abs(self.vel) * 1.2)
            diff = k_diff * yaw_err
            cmd_lt = thrust + diff
            cmd_rt = thrust - diff

        elif self.state == 'ORBIT':
            # Безопасная скорость орбиты
            target_orbit_speed = 0.80
            if self.vel > target_orbit_speed + 0.1:
                thrust = 1.0
            else:
                thrust = -target_orbit_speed * 3.3

            # === УЛУЧШЕННЫЙ КОНТРОЛЬ РАДИУСА ===
            radius_error = self.dist_2d - ORBIT_RADIUS
            # Более агрессивная коррекция + ограничение по углу
            correction_angle = max(-0.85, min(0.85, radius_error * 0.38))   # ← было 0.12!
            
            # Базовый тангенциальный угол (против часовой)
            angle_to_sub = math.atan2(self.pos[1] - self.target_global[1],
                                      self.pos[0] - self.target_global[0])
            self.bearing = angle_to_sub + math.pi/2 + correction_angle

            # Ограниченный дифференциал моторов (чтобы не скручивать на малой скорости)
            k_diff = 2.8
            diff = k_diff * yaw_err
            cmd_lt = thrust + diff
            cmd_rt = thrust - diff

            # Аварийный предохранитель крена
            if abs(math.degrees(roll_err)) > 35.0:
                cmd_lt = thrust
                cmd_rt = thrust
                rudder_v = 0.0

            # Выход из орбиты, когда глубина почти набрана
            if abs(z_err) < 1.2:
                self.state = 'FINAL_LOCK'
                print(f"\n🎯 FINAL | Глубина стабилизирована → выходим в центр")
                sys.stdout.flush()

        elif self.state == 'FINAL_LOCK':
            target_speed = max(self.min_cruise_speed, min(1.0, self.dist_2d * 0.4))
            if self.vel > target_speed + self.brake_threshold: thrust = 0.2  
            else: thrust = -target_speed * 3.3

            diff = self.K_diff_base * yaw_err
            cmd_lt = thrust + diff; cmd_rt = thrust - diff

            if self.dist_2d < 2.0 and abs(z_err) < 1.5:
                self.state = 'FINISH'
                self._pub(0,0,0,0,0)
                print(f"\n\r✅ МИССИЯ ЗАВЕРШЕНА | 1 ЭТАП ПОЛНОСТЬЮ ПОБЕЖДЕН!")
                print(f"Финиш: X={self.pos[0]:.2f} Y={self.pos[1]:.2f} Z={self.pos[2]:.2f}")
                raise SystemExit

        self._pub(cmd_lt, cmd_rt, rudder_v, cmd_hl, cmd_hr)
        print(f"\r[{self.state:10}] Pos:[{self.pos[0]:+.1f}, {self.pos[1]:+.1f}, {self.pos[2]:+.2f}] | "
              f"Dist2D:{self.dist_2d:.1f}m | V:{self.vel:+.2f} | Z_Err:{z_err:+.2f} | "
              f"Roll:{math.degrees(roll_err):+.1f}°", end='', flush=True)

    def _pub(self, lt, rt, rv, hl, hr):
        self.pub_lt.publish(Float64(data=float(lt)))
        self.pub_rt.publish(Float64(data=float(rt)))
        self.pub_vert.publish(Float64(data=float(rv)))
        self.pub_hl.publish(Float64(data=float(hl)))
        self.pub_hr.publish(Float64(data=float(hr)))

    def run(self):
        try:
            print("="*60 + "\n🚢 AUV v34.1 (Anti-Capsize & Safe Orbit Profile)\n" + "="*60)
            self.raw_target_x = float(input("📍 Абсолютный X цели: "))
            self.raw_target_y = float(input("📍 Абсолютный Y цели: "))
            self.raw_target_z = float(input("📍 Абсолютный Z цели: "))
            rclpy.spin(self)
        except (KeyboardInterrupt, SystemExit): 
            self._pub(0,0,0,0,0)

def main():
    rclpy.init(); node = AUVController()
    try: node.run()
    finally: node.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()
