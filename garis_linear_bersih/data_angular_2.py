#!/usr/bin/env python3
import rospy
import csv
import time
import os
import pandas as pd
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

class AmbilDataAngularFinal:
    def __init__(self):
        rospy.init_node('node_data_angular_final', anonymous=True)
        
        # =========================================================
        # KONFIGURASI TARGET (UBAH DISINI SAJA)
        # =========================================================
        self.TARGET_ANGULAR = 1.0   # Kecepatan yang Anda inginkan (rad/s)
        # =========================================================
        
        # Konfigurasi Fisik
        self.WHEEL_TRACK = 0.48        # Lebar Robot (Meter)
        self.METER_PER_TICK = 0.031746 # Kalibrasi Linear
        
        # Faktor Koreksi (Agar perintah 1.0 tidak jadi 4.16)
        # 1.0 (Target) / 4.16 (Realita Lama) = ~0.24
        self.SCALING_FACTOR = 0.24

        self.output_folder = os.path.expanduser('~/data_skripsi')
        if not os.path.exists(self.output_folder):
            os.makedirs(self.output_folder)
            
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Subscribe Ticks
        self.sub_left = rospy.Subscriber('/left_ticks', Int16, self.left_cb)
        self.sub_right = rospy.Subscriber('/right_ticks', Int16, self.right_cb)
        
        self.raw_data = [] 
        self.left_val = 0
        self.right_val = 0
        
        self.start_rec_time = 0
        self.is_recording = False
        
        self.durasi_putar = 5.0
        self.durasi_diam = 2.0

        print("[INIT] Menunggu sistem siap...")
        rospy.sleep(2)

    def left_cb(self, msg):
        self.left_val = msg.data
        self.record_if_active()

    def right_cb(self, msg):
        self.right_val = msg.data
        self.record_if_active()

    def record_if_active(self):
        if self.is_recording:
            t_stamp = rospy.get_time() - self.start_rec_time
            self.raw_data.append([t_stamp, self.left_val, self.right_val])

    def unwrap_ticks(self, raw_array):
        clean_ticks = []
        offset_overflow = 0
        start_value = raw_array[0]
        for i in range(len(raw_array)):
            val = raw_array[i]
            if i > 0:
                diff = val - raw_array[i-1]
                if diff < -30000: offset_overflow += 65536
                elif diff > 30000: offset_overflow -= 65536
            real_val = val + offset_overflow - start_value
            clean_ticks.append(real_val)
        return np.array(clean_ticks)

    def process_and_save(self, nama_file):
        print(f"\n[PROCESS] Mengolah data {nama_file}...")
        if len(self.raw_data) == 0:
            print("[ERROR] Tidak ada data terekam!")
            return

        # 1. Konversi ke DataFrame
        df = pd.DataFrame(self.raw_data, columns=['Waktu(s)', 'Raw_L', 'Raw_R'])
        df = df.drop_duplicates(subset=['Waktu(s)']).sort_values('Waktu(s)').reset_index(drop=True)

        # 2. Bersihkan Data (Unwrapping)
        df['Clean_L'] = self.unwrap_ticks(df['Raw_L'].values)
        df['Clean_R'] = self.unwrap_ticks(df['Raw_R'].values)

        # 3. Hitung Kecepatan Sudut (Omega)
        WINDOW = 10
        dt = df['Waktu(s)'].diff(WINDOW).fillna(0.1)
        
        dL = df['Clean_L'].diff(WINDOW).fillna(0)
        dR = df['Clean_R'].diff(WINDOW).fillna(0)
        
        # v = delta_ticks * meter_per_tick / waktu
        vL = (dL / dt) * self.METER_PER_TICK
        vR = (dR / dt) * self.METER_PER_TICK
        
        # Omega = (vR - vL) / Lebar_Robot
        df['Kecepatan_Angular_rad_s'] = (vR - vL) / self.WHEEL_TRACK
        
        # Rapikan data awal (biasanya NaN atau 0)
        df = df.fillna(0)

        # 4. Simpan File
        if not nama_file.endswith('.csv'):
            nama_file += '.csv'
            
        full_path = os.path.join(self.output_folder, nama_file)
        df.to_csv(full_path, index=False)
        
        max_omega = df['Kecepatan_Angular_rad_s'].max()
        print(f"[SUKSES] Data Tersimpan: {full_path}")
        print(f"Target Anda: {self.TARGET_ANGULAR} rad/s")
        print(f"Hasil Terukur: {max_omega:.4f} rad/s")
        
        if abs(max_omega - self.TARGET_ANGULAR) < 0.2:
            print(">>> HASIL VALID! Sesuai Target. <<<")
        else:
            print(">>> HASIL MASIH MENYIMPANG. Cek nilai WHEEL_TRACK atau METER_PER_TICK. <<<")

    def run(self):
        # Otomatis membuat nama file berdasarkan target
        nama_file_auto = f"data_angular_{str(self.TARGET_ANGULAR).replace('.', '_')}.csv"
        
        print(f"\n=== PENGAMBILAN DATA ANGULAR ===")
        print(f"Target Kecepatan: {self.TARGET_ANGULAR} rad/s")
        print(f"Perintah ke Robot (Scaled): {self.TARGET_ANGULAR * self.SCALING_FACTOR:.2f} rad/s")
        print("PENTING: Pastikan Anda sudah set Mode (Kasar/Halus) di terminal lain!")
        
        input(f">>> Tekan ENTER untuk mulai (File: {nama_file_auto})...")
        
        print("[GO] Merekam...")
        self.start_rec_time = rospy.get_time()
        self.is_recording = True
        
        # --- KIRIM PERINTAH DENGAN KOREKSI ---
        msg = Twist()
        msg.linear.x = 0.0
        # Di sini kuncinya: Target dikali 0.24 agar robot tidak ngebut
        msg.angular.z = self.TARGET_ANGULAR * self.SCALING_FACTOR 
        
        # FASE PUTAR
        start_run = rospy.get_time()
        while (rospy.get_time() - start_run) < self.durasi_putar:
            self.pub_vel.publish(msg)
            rospy.sleep(0.1)

        # FASE STOP
        print("[STOP] Pengereman...")
        msg.angular.z = 0.0
        start_stop = rospy.get_time()
        while (rospy.get_time() - start_stop) < self.durasi_diam:
            self.pub_vel.publish(msg)
            rospy.sleep(0.1)

        self.is_recording = False
        
        # Diamkan total
        for i in range(5):
            self.pub_vel.publish(msg)
            rospy.sleep(0.1)
            
        self.process_and_save(nama_file_auto)

if __name__ == '__main__':
    try:
        app = AmbilDataAngularFinal()
        app.run()
    except rospy.ROSInterruptException:
        pass
