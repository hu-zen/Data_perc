#!/usr/bin/env python3
import rospy
import csv
import time
import os
import pandas as pd
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

class AmbilDataInputNama:
    def __init__(self):
        rospy.init_node('node_data_input_nama', anonymous=True)
        
        # ==========================================
        # KONFIGURASI MANUAL (GANTI KECEPATAN DI SINI)
        # ==========================================
        self.TARGET_SPEED = 1.5  # Ganti jadi 0.23 atau angka lain di sini
        # ==========================================

        self.output_folder = os.path.expanduser('~/data_skripsi')
        if not os.path.exists(self.output_folder):
            os.makedirs(self.output_folder)
            
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub_ticks = rospy.Subscriber('/right_ticks', Int16, self.ticks_callback)
        
        self.raw_data = [] 
        self.start_rec_time = 0
        self.is_recording = False
        
        self.durasi_jalan = 5.0
        self.durasi_diam = 2.0

        print("[INIT] Menunggu sistem siap...")
        rospy.sleep(2)

    def ticks_callback(self, msg):
        if self.is_recording:
            t_stamp = rospy.get_time() - self.start_rec_time
            self.raw_data.append([t_stamp, msg.data])

    def process_and_save(self, nama_file):
        print(f"\n[PROCESS] Mengolah data {nama_file}...")
        
        if len(self.raw_data) == 0:
            print("[ERROR] Tidak ada data terekam!")
            return

        df = pd.DataFrame(self.raw_data, columns=['Waktu(s)', 'Raw_Ticks'])
        
        # 1. Bersihkan Data (Unwrapping & Zeroing)
        raw_ticks = df['Raw_Ticks'].values
        clean_ticks = []
        offset_overflow = 0
        start_value = raw_ticks[0]

        for i in range(len(raw_ticks)):
            val = raw_ticks[i]
            if i > 0:
                diff = val - raw_ticks[i-1]
                if diff < -30000: offset_overflow += 65536
                elif diff > 30000: offset_overflow -= 65536
            real_val = val + offset_overflow - start_value
            clean_ticks.append(real_val)

        df['Ticks_Bersih'] = clean_ticks

        # 2. Hitung Kecepatan (Smoothing)
        WINDOW = 10
        df['Speed_Ticks'] = df['Ticks_Bersih'].diff(WINDOW) / df['Waktu(s)'].diff(WINDOW)
        df['Speed_Ticks'] = df['Speed_Ticks'].fillna(0)

        # 3. Konversi ke Meter/Detik (Kalibrasi Tetap)
        FIXED_METER_PER_TICK = 0.031746 
        df['Kecepatan_Final_m_s'] = df['Speed_Ticks'] * FIXED_METER_PER_TICK

        # 4. Simpan File
        if not nama_file.endswith('.csv'):
            nama_file += '.csv'
            
        full_path = os.path.join(self.output_folder, nama_file)
        df.to_csv(full_path, index=False)
        
        max_speed = df['Kecepatan_Final_m_s'].max()
        print(f"[SUKSES] Data Tersimpan: {full_path}")
        print(f"Max Speed Terukur: {max_speed:.4f} m/s (Target Setting: {self.TARGET_SPEED})")

    def run(self):
        print(f"\n=== PENGAMBILAN DATA (Speed: {self.TARGET_SPEED} m/s) ===")
        
        # --- SATU-SATUNYA PERTANYAAN INTERAKTIF ---
        nama_file = input("Masukkan Nama File Output: ")
        
        input(">>> Tekan ENTER untuk mulai robot...")
        
        print("[GO] Merekam...")
        self.start_rec_time = rospy.get_time()
        self.is_recording = True
        
        msg = Twist()
        msg.linear.x = self.TARGET_SPEED
        
        # FASE JALAN
        start_run = rospy.get_time()
        while (rospy.get_time() - start_run) < self.durasi_jalan:
            self.pub_vel.publish(msg)
            rospy.sleep(0.1)

        # FASE STOP
        print("[STOP] Pengereman...")
        msg.linear.x = 0.0
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
        app = AmbilDataInputNama()
        app.run()
    except rospy.ROSInterruptException:
        pass
