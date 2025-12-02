#!/usr/bin/env python3
import rospy
import csv
import time
import os
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, Int16

class ManuverBelokHalus:
    def __init__(self):
        # Inisialisasi Node
        rospy.init_node('node_manuver_halus', anonymous=True)
        
        # Konfigurasi Folder
        self.output_folder = os.path.expanduser('~/data_skripsi')
        if not os.path.exists(self.output_folder):
            os.makedirs(self.output_folder)
            
        # Publisher
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_mode = rospy.Publisher('/set_mode', Int8, queue_size=10)
        
        # Subscribe ke KEDUA Ticks
        self.sub_left = rospy.Subscriber('/left_ticks', Int16, self.left_callback)
        self.sub_right = rospy.Subscriber('/right_ticks', Int16, self.right_callback)
        
        # Variabel Data
        self.ticks_L = 0
        self.ticks_R = 0
        self.last_ticks_L = 0
        self.last_ticks_R = 0
        self.last_time = 0
        self.velocity_ticks = 0.0 # Kecepatan Rata-rata Robot
        
        self.csv_writer = None
        self.file_handle = None
        self.start_rec_time = 0
        self.is_recording = False

        print("[INIT] Menunggu sistem siap...")
        rospy.sleep(2)

    def left_callback(self, msg):
        self.ticks_L = msg.data
        self.hitung_kecepatan()

    def right_callback(self, msg):
        self.ticks_R = msg.data
        self.hitung_kecepatan()

    def hitung_kecepatan(self):
        current_time = rospy.get_time()
        
        if self.last_time != 0:
            delta_time = current_time - self.last_time
            if delta_time > 0:
                # 1. Hitung Delta Kiri
                delta_L = self.ticks_L - self.last_ticks_L
                if delta_L < -30000: delta_L += 65536 
                elif delta_L > 30000: delta_L -= 65536 
                
                # 2. Hitung Delta Kanan
                delta_R = self.ticks_R - self.last_ticks_R
                if delta_R < -30000: delta_R += 65536 
                elif delta_R > 30000: delta_R -= 65536 
                
                # 3. Kecepatan Robot = Rata-rata (vL + vR) / 2
                vel_L = delta_L / delta_time
                vel_R = delta_R / delta_time
                self.velocity_ticks = (vel_L + vel_R) / 2.0
                
        self.last_ticks_L = self.ticks_L
        self.last_ticks_R = self.ticks_R
        self.last_time = current_time
        
        # Simpan ke CSV
        if self.is_recording and self.csv_writer:
            t_stamp = rospy.get_time() - self.start_rec_time
            avg_ticks = (self.ticks_L + self.ticks_R) / 2.0
            # FORMAT: Waktu, Total_Ticks_Avg, Kecepatan_Avg
            self.csv_writer.writerow([t_stamp, avg_ticks, self.velocity_ticks])

    def stop_robot(self, durasi):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        end_time = rospy.get_time() + durasi
        while rospy.get_time() < end_time:
            self.pub_vel.publish(msg)
            rospy.sleep(0.1)

    def run(self):
        # 1. SET MODE HALUS (Kirim angka 1)
        mode_msg = Int8()
        mode_msg.data = 1 
        for i in range(5):
            self.pub_mode.publish(mode_msg)
            rospy.sleep(0.1)
            
        print("\n=============================================")
        print("[STATUS] Mode Robot: HALUS (Ramp Rate Control)")
        print("=============================================")
        rospy.sleep(1)

        # 2. BUKA FILE CSV
        filename = os.path.join(self.output_folder, 'data_manuver_halus.csv')
        self.file_handle = open(filename, 'w', newline='')
        self.csv_writer = csv.writer(self.file_handle)
        self.csv_writer.writerow(['Waktu(s)', 'Total_Ticks', 'Kecepatan(Ticks/s)'])
        
        print("[READY] Robot akan bergerak dalam 3 detik...")
        rospy.sleep(3)
        
        self.start_rec_time = rospy.get_time()
        self.is_recording = True
        
        # --- KOREOGRAFI MANUVER ---

        # 1. MAJU LURUS
        print("--> [1/5] Maju Lurus...")
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.0
        end_t = rospy.get_time() + 2.0
        while rospy.get_time() < end_t:
            self.pub_vel.publish(msg)
            rospy.sleep(0.1)

        # 2. JEDA
        print("--> [2/5] Jeda...")
        self.stop_robot(3.0)

        # 3. BELOK KIRI (Linear + Angular)
        print("--> [3/5] Belok Kiri...")
        msg.linear.x = 0.5
        msg.angular.z = 0.5
        end_t = rospy.get_time() + 3.0
        while rospy.get_time() < end_t:
            self.pub_vel.publish(msg)
            rospy.sleep(0.1)

        # 4. JEDA
        print("--> [4/5] Jeda...")
        self.stop_robot(3.0)

        # 5. MAJU LURUS
        print("--> [5/5] Maju Lurus...")
        msg.linear.x = 0.5
        msg.angular.z = 0.0
        end_t = rospy.get_time() + 2.0
        while rospy.get_time() < end_t:
            self.pub_vel.publish(msg)
            rospy.sleep(0.1)

        # SELESAI
        print("--> SELESAI.")
        self.stop_robot(1.0)
        self.is_recording = False
        self.file_handle.close()
        print(f"[SAVED] {filename}")

if __name__ == '__main__':
    try:
        app = ManuverBelokHalus()
        app.run()
    except rospy.ROSInterruptException:
        pass
