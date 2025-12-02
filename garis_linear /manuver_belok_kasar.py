#!/usr/bin/env python3
import rospy
import csv
import time
import os
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, Int16

class ManuverBelokKasar:
    def __init__(self):
        rospy.init_node('node_manuver_kasar', anonymous=True)
        
        self.output_folder = os.path.expanduser('~/data_skripsi')
        if not os.path.exists(self.output_folder):
            os.makedirs(self.output_folder)
            
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_mode = rospy.Publisher('/set_mode', Int8, queue_size=10)
        
        # Subscribe ke KEDUA Encoder (Penting untuk analisis belok)
        self.sub_left = rospy.Subscriber('/left_ticks', Int16, self.left_cb)
        self.sub_right = rospy.Subscriber('/right_ticks', Int16, self.right_cb)
        
        self.left_ticks = 0
        self.right_ticks = 0
        self.csv_writer = None
        self.file_handle = None
        self.start_rec_time = 0
        self.is_recording = False

        print("[INIT] Menunggu sistem siap...")
        rospy.sleep(2)

    def left_cb(self, msg):
        self.left_ticks = msg.data

    def right_cb(self, msg):
        self.right_ticks = msg.data
        # Rekam data setiap kali ada update (Sampling Rate ikut sensor)
        if self.is_recording and self.csv_writer:
            t_stamp = rospy.get_time() - self.start_rec_time
            # Simpan: Waktu, Ticks Kiri, Ticks Kanan
            self.csv_writer.writerow([t_stamp, self.left_ticks, self.right_ticks])

    def stop_robot(self, durasi):
        # Fungsi helper untuk diam
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        
        end_time = rospy.get_time() + durasi
        while rospy.get_time() < end_time:
            self.pub_vel.publish(msg)
            rospy.sleep(0.1)

    def run(self):
        # 1. SET MODE KASAR (Kirim angka 0)
        mode_msg = Int8()
        mode_msg.data = 0 
        for i in range(5):
            self.pub_mode.publish(mode_msg)
            rospy.sleep(0.1)
            
        print("\n=============================================")
        print("[STATUS] Mode Robot: KASAR (Direct Control)")
        print("=============================================")
        rospy.sleep(1)

        # 2. BUKA FILE CSV
        filename = os.path.join(self.output_folder, 'data_manuver_kasar.csv')
        self.file_handle = open(filename, 'w', newline='')
        self.csv_writer = csv.writer(self.file_handle)
        self.csv_writer.writerow(['Waktu(s)', 'Left_Ticks', 'Right_Ticks'])
        
        print("[READY] Robot akan bergerak dalam 3 detik...")
        rospy.sleep(3)
        
        self.start_rec_time = rospy.get_time()
        self.is_recording = True
        
        # --- KOREOGRAFI ---

        # PHASE 1: MAJU LURUS (2 Detik)
        print("--> [1/5] Maju Lurus (0.5 m/s)...")
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.0
        
        end_time = rospy.get_time() + 2.0
        while rospy.get_time() < end_time:
            self.pub_vel.publish(msg)
            rospy.sleep(0.1)

        # PHASE 2: JEDA (3 Detik)
        print("--> [2/5] Jeda (0 m/s)...")
        self.stop_robot(3.0)

        # PHASE 3: BELOK KIRI (3 Detik)
        # Kombinasi Linear + Angular
        print("--> [3/5] Belok Kiri (Maju 0.5 + Putar 0.5)...")
        msg.linear.x = 0.5
        msg.angular.z = 0.5 
        
        end_time = rospy.get_time() + 3.0
        while rospy.get_time() < end_time:
            self.pub_vel.publish(msg)
            rospy.sleep(0.1)

        # PHASE 4: JEDA (3 Detik)
        print("--> [4/5] Jeda (0 m/s)...")
        self.stop_robot(3.0)

        # PHASE 5: MAJU LURUS LAGI (2 Detik)
        print("--> [5/5] Maju Lurus Lagi (0.5 m/s)...")
        msg.linear.x = 0.5
        msg.angular.z = 0.0
        
        end_time = rospy.get_time() + 2.0
        while rospy.get_time() < end_time:
            self.pub_vel.publish(msg)
            rospy.sleep(0.1)

        # FINISH
        print("--> SELESAI.")
        self.stop_robot(1.0) # Pastikan diam

        self.is_recording = False
        self.file_handle.close()
        print(f"\n[SELESAI] Data tersimpan di: {filename}")
        print("Silakan RESET posisi robot ke titik awal.")

if __name__ == '__main__':
    try:
        app = ManuverBelokKasar()
        app.run()
    except rospy.ROSInterruptException:
        pass
