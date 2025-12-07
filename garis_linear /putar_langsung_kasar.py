#!/usr/bin/env python3
import rospy
import csv
import time
import os
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, Int16

class PutarLangsungKasar:
    def __init__(self):
        rospy.init_node('node_putar_kasar_clean', anonymous=True)
        
        self.output_folder = os.path.expanduser('~/data_skripsi')
        if not os.path.exists(self.output_folder):
            os.makedirs(self.output_folder)
            
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_mode = rospy.Publisher('/set_mode', Int8, queue_size=10)
        
        self.sub_left = rospy.Subscriber('/left_ticks', Int16, self.left_cb)
        self.sub_right = rospy.Subscriber('/right_ticks', Int16, self.right_cb)
        
        # Variabel untuk Unwrapping (Anti-Overflow)
        self.left_raw = 0
        self.right_raw = 0
        self.left_prev = 0
        self.right_prev = 0
        self.left_total = 0
        self.right_total = 0
        self.left_offset = 0
        self.right_offset = 0
        
        self.first_run = True
        self.csv_writer = None
        self.file_handle = None
        self.start_rec_time = 0
        self.is_recording = False

        print("[INIT] Kalibrasi Encoder (Tunggu 2 detik)...")
        rospy.sleep(2) # Biarkan callback jalan sebentar untuk set offset

    def unwrap(self, current, prev, total):
        # Logika menangani overflow Int16 (-32768 ke 32767)
        diff = current - prev
        if diff < -30000:
            total += 65536 + diff
        elif diff > 30000:
            total += -65536 + diff
        else:
            total += diff
        return total

    def left_cb(self, msg):
        self.left_raw = msg.data
        if self.first_run:
            self.left_prev = self.left_raw
            return
            
        self.left_total = self.unwrap(self.left_raw, self.left_prev, self.left_total)
        self.left_prev = self.left_raw

    def right_cb(self, msg):
        self.right_raw = msg.data
        if self.first_run:
            self.right_prev = self.right_raw
            self.first_run = False # Selesai inisialisasi awal
            
            # Set OFFSET agar data mulai dari 0
            self.right_offset = self.right_total
            self.left_offset = self.left_total
            return

        self.right_total = self.unwrap(self.right_raw, self.right_prev, self.right_total)
        self.right_prev = self.right_raw
        
        # Rekam Data Bersih (Sudah dikurangi offset awal)
        if self.is_recording and self.csv_writer:
            t_stamp = rospy.get_time() - self.start_rec_time
            
            clean_L = self.left_total - self.left_offset
            clean_R = self.right_total - self.right_offset
            
            # Simpan: Waktu, L_Bersih, R_Bersih
            self.csv_writer.writerow([t_stamp, clean_L, clean_R])

    def run(self):
        # 1. SET MODE KASAR (0)
        mode_msg = Int8()
        mode_msg.data = 0 
        for i in range(5):
            self.pub_mode.publish(mode_msg)
            rospy.sleep(0.1)
            
        print("\n=== MODE KASAR (PUTAR LANGSUNG) ===")
        print("Data akan otomatis mulai dari 0.")
        
        # Reset Offset Tepat sebelum mulai
        self.left_offset = self.left_total
        self.right_offset = self.right_total

        filename = os.path.join(self.output_folder, 'putar_kasar_bersih.csv')
        self.file_handle = open(filename, 'w', newline='')
        self.csv_writer = csv.writer(self.file_handle)
        self.csv_writer.writerow(['Waktu(s)', 'Left_Clean', 'Right_Clean'])
        
        print("[READY] Robot PUTAR dalam 3 detik...")
        rospy.sleep(3)
        
        print("[GO] Merekam...")
        self.start_rec_time = rospy.get_time()
        self.is_recording = True
        
        # --- SKENARIO: PUTAR SAJA ---
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 1.0  # Kecepatan Putar (Rad/s)
        
        durasi_putar = 5.0
        end_time = rospy.get_time() + durasi_putar
        
        while rospy.get_time() < end_time:
            self.pub_vel.publish(msg)
            rospy.sleep(0.1)

        print("[STOP] Pengereman...")
        msg.angular.z = 0.0
        
        # Rekam fase berhenti (2 detik)
        end_stop = rospy.get_time() + 2.0
        while rospy.get_time() < end_stop:
            self.pub_vel.publish(msg)
            rospy.sleep(0.1)

        self.is_recording = False
        self.file_handle.close()
        print(f"[SELESAI] Data Bersih Tersimpan: {filename}")

if __name__ == '__main__':
    try:
        app = PutarLangsungKasar()
        app.run()
    except rospy.ROSInterruptException:
        pass
