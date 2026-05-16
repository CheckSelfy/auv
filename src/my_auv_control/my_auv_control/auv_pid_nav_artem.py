#!/usr/bin/env python3
"""AUV PID Autopilot v35.4 | LOS Adaptive + ULTRA Anti-Capsize (ROLL FIXED v2)"""
import rclpy, math, time, sys
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

P_Z0 = 101325.0
RHO_G = 9810.0


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
        self.create_subscription(Float32, '/sub_press', self.press_cb, qos_s)

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
        self.min_cruise_speed = 0.6  
        self.brake_threshold = 0.2

        # === LOS GUIDANCE ===
        self.lookahead_base = 25.0
        self.min_lookahead = 8.0

        # ПИД Z (глубина)
        self.Kp_z = 4.2
        self.Kd_z = 1.7
        
        # Курс
        self.Kp_yaw = 1.8
        self.Kd_yaw = 0.5
        self.K_diff_base = 2.5

        # 🔥 ULTRA ANTI-CAPSIZE v2 — ЕЩЁ СИЛЬНЕЕ И С ПРАВИЛЬНЫМ ЗНАКОМ
        self.Kp_roll = 120.0      # сильно увеличено
        self.Kd_roll = 30.0
        self.roll_bias = 0.05
        
        # ←←← ГЛАВНОЕ ИСПРАВЛЕНИЕ: знак дифференциала горизонтальных рулей
        # Если продолжает крениться вправо — поменяй на -1.0
        self.roll_correction_sign = -1.0

        self.stable_t = 0.0
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.loop)
        self.roll_protection_active = False

        self.raw_target_x = 0.0
        self.raw_target_y = 0.0
        self.raw_target_z = 0.0

    def press_cb(self, msg):
        self.baro_z = (P_Z0 - msg.data) / RHO_G
        if not hasattr(self, '_pressure_logged'):
            print(f"✅ Pressure sensor OK! baro_z = {self.baro_z:+.3f} м")
            self._pressure_logged = True

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
            print(f"\n🎯 Запуск LOS Adaptive v35.4 + ULTRA Anti-Capsize (ROLL FIXED v2)")
            print(f"   Цель: X={self.target_global[0]:.1f} Y={self.target_global[1]:.1f} Z={self.target_global[2]:.1f}")
            print(f"   roll_correction_sign = {self.roll_correction_sign} (поменяй на -1.0 / 1.0 если кренится не туда)")

        dx = self.target_global[0] - self.pos[0]
        dy = self.target_global[1] - self.pos[1]
        self.dist_2d = math.hypot(dx, dy)

        path_bearing = math.atan2(dy, dx)
        cross_track = (dx * math.sin(self.rpy[2]) - dy * math.cos(self.rpy[2])) * -1.0
        lookahead = max(self.min_lookahead, min(self.lookahead_base, self.dist_2d * 0.6))
        correction = math.atan2(cross_track, lookahead)
        self.bearing = path_bearing + correction

    def loop(self):
        if self.state not in ['STAB', 'NAV', 'FINAL_LOCK']:
            return
        
        # Глубина
        z_err = self.pos[2] - self.target_global[2]
        dz_dt = (self.pos[2] - self.prev_baro_z) / self.dt
        raw_h = -(self.Kp_z * z_err + self.Kd_z * dz_dt)
        rudder_h = max(-0.22, min(0.22, raw_h))
        self.prev_baro_z = self.pos[2]

        # Курс
        yaw_err = math.atan2(math.sin(self.bearing - self.rpy[2]), math.cos(self.bearing - self.rpy[2]))
        if abs(math.degrees(yaw_err)) < 1.0:
            yaw_err = 0.0
        d_yaw = (self.rpy[2] - self.prev_rpy[2]) / self.dt
        rudder_v = self.Kp_yaw * yaw_err + self.Kd_yaw * d_yaw
        rudder_v = max(-0.45, min(0.45, rudder_v))

        # === ROLL CONTROL (с правильным знаком) ===
        roll_err = self.rpy[0]
        d_roll = (self.rpy[0] - self.prev_rpy[0]) / self.dt
        roll_pid = self.Kp_roll * roll_err + self.Kd_roll * d_roll

        sign = self.roll_correction_sign
        cmd_hl = rudder_h + sign * roll_pid - self.roll_bias
        cmd_hr = rudder_h - sign * roll_pid + self.roll_bias

        cmd_hl = max(-0.78, min(0.78, cmd_hl))
        cmd_hr = max(-0.78, min(0.78, cmd_hr))
        self.prev_rpy = list(self.rpy)
        
        thrust = 0.0
        cmd_lt = 0.0
        cmd_rt = 0.0
        self.roll_protection_active = False
        emergency = False

        # ================= АВТОМАТ ТРАЕКТОРИЙ =================
        if self.state == 'STAB':
            if abs(roll_err) < 0.12:
                self.stable_t += self.dt
            else:
                self.stable_t = 0.0
            if self.stable_t >= 1.5:
                self.state = 'NAV'
            cmd_hl = max(-0.25, min(0.25, sign * roll_pid - self.roll_bias))
            cmd_hr = max(-0.25, min(0.25, -sign * roll_pid + self.roll_bias))

        elif self.state == 'NAV':
            z_factor = max(0.45, 1.0 - abs(z_err) / 18.0)

            if self.dist_2d > 40.0:
                base_speed = self.max_cruise_speed
            elif self.dist_2d > 15.0:
                base_speed = self.dist_2d * 0.12
            else:
                base_speed = self.dist_2d * 0.06

            target_speed = max(self.min_cruise_speed, min(self.max_cruise_speed, base_speed)) * z_factor

            if self.vel > target_speed + self.brake_threshold:
                thrust = 1.4
            else:
                thrust = -target_speed * 4.2

            k_diff = self.K_diff_base * (1.0 + abs(self.vel) * 1.4)
            diff = k_diff * yaw_err
            cmd_lt = thrust + diff
            cmd_rt = thrust - diff

            # === УСИЛЕННАЯ ЗАЩИТА ОТ КАПСУЛИРОВАНИЯ ===
            roll_deg = math.degrees(roll_err)
            if abs(roll_deg) > 8.0:                     # срабатывает ОЧЕНЬ рано
                self.roll_protection_active = True
                emergency = True
                thrust = -5.0                           # ОЧЕНЬ сильный тормоз
                cmd_lt = thrust
                cmd_rt = thrust
                rudder_v = 0.0

                # Максимально агрессивный дифференциал рулей
                emergency_rudder = 0.78 * sign * (-1 if roll_err < 0 else 1)
                cmd_hl = emergency_rudder
                cmd_hr = -emergency_rudder

                print(f"⚠️  ROLL PROTECTION | Roll {roll_deg:+.1f}° | "
                      f"cmd_hl={cmd_hl:+.3f} cmd_hr={cmd_hr:+.3f} | sign={sign}")

            if self.dist_2d < 12.0 and abs(z_err) < 2.5:
                self.state = 'FINAL_LOCK'
                print(f"\n🔄 Переход в FINAL_LOCK")

        elif self.state == 'FINAL_LOCK':
            target_speed = max(0.4, min(1.0, self.dist_2d * 0.08))
            if self.vel > target_speed + 0.1:
                thrust = 1.6
            else:
                thrust = -target_speed * 5.0

            diff = self.K_diff_base * 1.2 * yaw_err
            cmd_lt = thrust + diff
            cmd_rt = thrust - diff

            if self.dist_2d < 2.5 and abs(z_err) < 1.4:
                self.state = 'FINISH'
                self._pub(0, 0, 0, 0, 0)
                print(f"\n✅ МИССИЯ ЗАВЕРШЕНА!")
                print(f"Финиш: X={self.pos[0]:.2f} Y={self.pos[1]:.2f} Z={self.pos[2]:.2f}")
                raise SystemExit

        self._pub(cmd_lt, cmd_rt, rudder_v, cmd_hl, cmd_hr)

        status = "ROLL_PROT" if self.roll_protection_active else self.state
        print(f"\r[{status:10}] Pos:[{self.pos[0]:+.1f},{self.pos[1]:+.1f},{self.pos[2]:+.2f}] | "
              f"Dist:{self.dist_2d:.1f}m | V:{self.vel:+.2f} | Zerr:{z_err:+.2f} | "
              f"Roll:{math.degrees(roll_err):+.1f}°", end='', flush=True)

    def _pub(self, lt, rt, rv, hl, hr):
        self.pub_lt.publish(Float64(data=float(lt)))
        self.pub_rt.publish(Float64(data=float(rt)))
        self.pub_vert.publish(Float64(data=float(rv)))
        self.pub_hl.publish(Float64(data=float(hl)))
        self.pub_hr.publish(Float64(data=float(hr)))

    def run(self):
        try:
            print("="*80)
            print("🚢 AUV v35.4 LOS Adaptive + ULTRA Anti-Capsize (ROLL FIXED v2)")
            print("="*80)
            print("📍 Z = отрицательное значение = глубина (например 15 → Z=-15)")
            print("   roll_correction_sign = -1.0 (поменяй на 1.0 в коде если кренится не туда)")
            print()
            self.raw_target_x = float(input("📍 Абсолютный X цели: "))
            self.raw_target_y = float(input("📍 Абсолютный Y цели: "))
            z_input = float(input("📍 Глубина цели (м, положительное число): "))
            self.raw_target_z = -z_input
            rclpy.spin(self)
        except (KeyboardInterrupt, SystemExit):
            self._pub(0, 0, 0, 0, 0)
            print("\n🛑 AUV остановлен.")


def main():
    rclpy.init()
    node = AUVController()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()