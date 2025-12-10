#!/usr/bin/env python3
import rospy
import csv
import time
import os
import pandas as pd
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

class AmbilDataPutarManual:
    def __init__(self):
        rospy.init_node('node_data_putar_manual', anonymous=True)
        
        # ==========================================
        # KONFIGURASI MANUAL (EDIT DI SINI)
        # ==========================================
        self.TARGET_ANGULAR = 0.5 # Rad/s (Kecepatan Putar Target)
        self.WHEEL_TRACK = 1.75# Jarak antar roda kiri-kanan (Meter) -> UKUR ROBOT ANDA!
        self.METER_PER_TICK = 0.031746 # Kalibrasi Linear yang sudah didapat
        # ==========================================

        self.output_folder = os.path.expanduser('~/data_skripsi')
        if not os.path.exists(self.output_folder):
            os.makedirs(self.output_folder)
            
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Subscribe ke KEDUA Ticks
        self.sub_left = rospy.Subscriber('/left_ticks', Int16, self.left_cb)
        self.sub_right = rospy.Subscriber('/right_ticks', Int16, self.right_cb)
        
        # Buffer Data
        self.raw_data = [] 
        self.left_val = 0
        self.right_val = 0
        
        self.start_rec_time = 0
        self.is_recording = False
        
        # Durasi Putar
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
        # Merekam data hanya jika flag aktif
        if self.is_recording:
            t_stamp = rospy.get_time() - self.start_rec_time
            # Simpan [Waktu, Tick_Kiri, Tick_Kanan]
            self.raw_data.append([t_stamp, self.left_val, self.right_val])

    def unwrap_ticks(self, raw_array):
        # Fungsi pembantu untuk membersihkan overflow per array
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
        print(f"\n[PROCESS] Mengolah data angular {nama_file}...")
        
        if len(self.raw_data) == 0:
            print("[ERROR] Tidak ada data terekam!")
            return

        # 1. Konversi ke DataFrame
        df = pd.DataFrame(self.raw_data, columns=['Waktu(s)', 'Raw_L', 'Raw_R'])
        
        # Hapus duplikat waktu (karena callback left & right mungkin panggil record 2x)
        df = df.drop_duplicates(subset=['Waktu(s)']).sort_values('Waktu(s)').reset_index(drop=True)

        # 2. Bersihkan Data (Unwrapping)
        df['Clean_L'] = self.unwrap_ticks(df['Raw_L'].values)
        df['Clean_R'] = self.unwrap_ticks(df['Raw_R'].values)

        # 3. Hitung Kecepatan Sudut (Smoothing)
        WINDOW = 10
        
        # Hitung Delta Ticks per sisi
        dL = df['Clean_L'].diff(WINDOW).fillna(0)
        dR = df['Clean_R'].diff(WINDOW).fillna(0)
        dt = df['Waktu(s)'].diff(WINDOW).fillna(0.1) # Hindari div by zero
        
        # Hitung Kecepatan Linear Roda (v = delta_tick * m_per_tick / dt)
        vL = (dL / dt) * self.METER_PER_TICK
        vR = (dR / dt) * self.METER_PER_TICK
        
        # Hitung Kecepatan Angular (Omega)
        # Rumus: Omega = (vR - vL) / Wheel_Track
        # Karena putar di tempat (Pivot), vL akan negatif, vR positif.
        df['Kecepatan_Angular_rad_s'] = (vR - vL) / self.WHEEL_TRACK

        # 4. Simpan File
        if not nama_file.endswith('.csv'):
            nama_file += '.csv'
            
        full_path = os.path.join(self.output_folder, nama_file)
        df.to_csv(full_path, index=False)
        
        max_omega = df['Kecepatan_Angular_rad_s'].max()
        print(f"[SUKSES] Data Tersimpan: {full_path}")
        print(f"Max Angular Terukur: {max_omega:.4f} rad/s (Target: {self.TARGET_ANGULAR})")

    def run(self):
        print(f"\n=== PENGAMBILAN DATA PUTAR (Target: {self.TARGET_ANGULAR} rad/s) ===")
        print("PENTING: Pastikan Mode (Kasar/Halus) sudah diset di terminal lain!")
        
        nama_file = input("Masukkan Nama File Output (contoh: putar_kasar_1): ")
        
        input(">>> Tekan ENTER untuk mulai robot...")
        
        print("[GO] Merekam...")
        self.start_rec_time = rospy.get_time()
        self.is_recording = True
        
        # Perintah Putar (Linear 0, Angular Target)
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = self.TARGET_ANGULAR
        
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
        
        # Pastikan diam total
        for i in range(5):
            self.pub_vel.publish(msg)
            rospy.sleep(0.1)
            
        self.process_and_save(nama_file)

if __name__ == '__main__':
    try:
        app = AmbilDataPutarManual()
        app.run()
    except rospy.ROSInterruptException:
        pass
