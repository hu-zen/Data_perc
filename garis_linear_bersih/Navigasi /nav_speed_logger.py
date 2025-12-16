#!/usr/bin/env python3
import rospy
import time
import os
import pandas as pd
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry

class SkripsiNavLogger:
    def __init__(self):
        rospy.init_node('skripsi_nav_logger', anonymous=True)
        
        # ==========================================
        # KONFIGURASI ROBOT (SAMA DENGAN SCRIPT ANDA)
        # ==========================================
        self.METER_PER_TICK = 0.031746  # Kalibrasi Encoder
        self.WHEEL_TRACK = 1.75         # Jarak Roda Kiri-Kanan (m)
        self.output_folder = os.path.expanduser('~/data_skripsi')
        # ==========================================

        if not os.path.exists(self.output_folder):
            os.makedirs(self.output_folder)
            
        # --- Variabel Penampung Data Sementara ---
        self.curr_cmd_lin = 0.0   # Target Linear dari DWA
        self.curr_cmd_ang = 0.0   # Target Angular dari DWA
        self.curr_tick_r = 0      # Encoder Kanan
        self.curr_tick_l = 0      # Encoder Kiri
        self.curr_pos_x = 0.0     # Posisi X (Odom)
        self.curr_pos_y = 0.0     # Posisi Y (Odom)
        
        # --- Variabel Logic ---
        self.raw_data = [] 
        self.start_rec_time = 0
        self.is_recording = False
        self.is_moving = False
        self.robot_has_moved = False # Penanda robot pernah bergerak
        self.last_motion_time = 0
        
        # --- Subscribers ---
        # Kita ambil data dari DWA (cmd_vel) dan Sensor (ticks/odom)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_callback)
        rospy.Subscriber('/right_ticks', Int16, self.right_callback)
        rospy.Subscriber('/left_ticks', Int16, self.left_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        self.main_loop()

    # --- CALLBACKS (Hanya update variabel, tidak merekam di sini biar rapi) ---
    def cmd_callback(self, msg):
        self.curr_cmd_lin = msg.linear.x
        self.curr_cmd_ang = msg.angular.z
        
        # Deteksi Gerak (Threshold kecil untuk noise)
        if abs(self.curr_cmd_lin) > 0.01 or abs(self.curr_cmd_ang) > 0.01:
            self.is_moving = True
            self.robot_has_moved = True
            self.last_motion_time = rospy.get_time()
        else:
            self.is_moving = False

    def right_callback(self, msg):
        self.curr_tick_r = msg.data

    def left_callback(self, msg):
        self.curr_tick_l = msg.data

    def odom_callback(self, msg):
        self.curr_pos_x = msg.pose.pose.position.x
        self.curr_pos_y = msg.pose.pose.position.y

    # --- PENGOLAHAN DATA (MIRIP SCRIPT DATA_LINEAR.PY) ---
    def unwrap_ticks(self, raw_array):
        # Fungsi membersihkan overflow encoder (32767 -> -32768)
        clean = []
        offset = 0
        start_val = raw_array[0]
        for i in range(len(raw_array)):
            val = raw_array[i]
            if i > 0:
                diff = val - raw_array[i-1]
                if diff < -30000: offset += 65536
                elif diff > 30000: offset -= 65536
            clean.append(val + offset - start_val)
        return np.array(clean)

    def process_and_save(self, nama_file):
        print(f"\n[PROCESS] Mengolah {len(self.raw_data)} data...")
        if len(self.raw_data) < 10:
            print("[WARN] Data terlalu sedikit/kosong.")
            return

        try:
            # 1. Buat DataFrame
            cols = ['Abs_Time', 'Raw_Tick_L', 'Raw_Tick_R', 'Target_Lin', 'Target_Ang', 'Pos_X', 'Pos_Y']
            df = pd.DataFrame(self.raw_data, columns=cols)
            
            # 2. Normalisasi Waktu (Agar mulai dari 0.00 detik)
            start_t = df['Abs_Time'].iloc[0]
            df['Waktu(s)'] = df['Abs_Time'] - start_t

            # 3. Bersihkan Ticks (Unwrapping)
            df['Clean_L'] = self.unwrap_ticks(df['Raw_Tick_L'].values)
            df['Clean_R'] = self.unwrap_ticks(df['Raw_Tick_R'].values)

            # 4. Hitung Kecepatan Actual (Kinematik Differential Drive)
            WINDOW = 10 # Smoothing window
            
            # Delta Ticks & Time
            dL = df['Clean_L'].diff(WINDOW).fillna(0)
            dR = df['Clean_R'].diff(WINDOW).fillna(0)
            dt = df['Waktu(s)'].diff(WINDOW).fillna(0.1)
            
            # Hindari pembagian nol
            dt = dt.replace(0, 0.1)

            # Kecepatan Roda (m/s)
            vL = (dL / dt) * self.METER_PER_TICK
            vR = (dR / dt) * self.METER_PER_TICK

            # === RUMUS UTAMA ===
            # Linear Velocity (v) = (vR + vL) / 2
            # Angular Velocity (w) = (vR - vL) / Wheel_Track
            df['Actual_Lin_m_s'] = (vR + vL) / 2.0
            df['Actual_Ang_rad_s'] = (vR - vL) / self.WHEEL_TRACK

            # 5. Rapikan Kolom untuk CSV Akhir
            final_df = df[['Waktu(s)', 'Target_Lin', 'Actual_Lin_m_s', 'Target_Ang', 'Actual_Ang_rad_s', 'Pos_X', 'Pos_Y']]
            
            # Simpan
            filename = f"{nama_file}_NAV_FULL.csv"
            path = os.path.join(self.output_folder, filename)
            final_df.to_csv(path, index=False)
            
            print(f"[SUKSES] Data Navigasi Tersimpan: {filename}")
            print(f"         Max Linear: {final_df['Actual_Lin_m_s'].max():.3f} m/s")
            print(f"         Max Angular: {final_df['Actual_Ang_rad_s'].max():.3f} rad/s")

        except Exception as e:
            print(f"[ERROR] Gagal processing: {e}")

    def main_loop(self):
        rate = rospy.Rate(10) # 10Hz sampling rate (Cukup presisi)
        
        while not rospy.is_shutdown():
            os.system('clear')
            print("="*50)
            print("   SKRIPSI LOGGER: NAVIGASI (DWA & ODOM)")
            print("="*50)
            
            nama_file = input("\n[1/2] Masukkan Nama Skenario: ").strip()
            if not nama_file: nama_file = f"nav_{int(time.time())}"

            print("\n[2/2] START RECORDING... (Tekan Enter)")
            print("      Data 0.00 akan terekam segera.")
            
            # --- START LANGSUNG ---
            self.raw_data = []
            self.is_recording = True
            self.robot_has_moved = False
            
            # Tampilan Status Live
            print("\n[REC] Merekam... Silakan set 2D Nav Goal di Rviz sekarang!")
            
            while not rospy.is_shutdown():
                # Ambil snapshot data saat ini
                t_now = rospy.get_time()
                self.raw_data.append([
                    t_now, 
                    self.curr_tick_l, 
                    self.curr_tick_r, 
                    self.curr_cmd_lin, 
                    self.curr_cmd_ang,
                    self.curr_pos_x,
                    self.curr_pos_y
                ])
                
                # Info Live di Terminal
                durasi = t_now - self.raw_data[0][0]
                status = "BERGERAK" if self.is_moving else "DIAM"
                print(f"\r      T: {durasi:.1f}s | Status: {status} | Target: {self.curr_cmd_lin:.2f} m/s | Data: {len(self.raw_data)}", end="")

                # --- LOGIKA AUTO STOP ---
                # Stop jika: Robot sudah pernah bergerak DAN sekarang diam > 2 detik
                if self.robot_has_moved and not self.is_moving:
                    if (rospy.get_time() - self.last_motion_time) > 2.0:
                        print("\n\n[STOP] Robot sampai tujuan / berhenti lama.")
                        break
                
                rate.sleep()

            self.is_recording = False
            if len(self.raw_data) > 0:
                self.process_and_save(nama_file)
            
            input("\nTekan [Enter] untuk percobaan baru...")

if __name__ == '__main__':
    try:
        SkripsiNavLogger()
    except rospy.ROSInterruptException:
        pass
