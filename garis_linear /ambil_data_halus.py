#!/usr/bin/env python3
import rospy
import csv
import time
import os
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8

class AmbilDataHalus:
    def __init__(self):
        rospy.init_node('node_ambil_data_halus')
        
        # Lokasi Simpan Data
        self.output_folder = os.path.expanduser('~/data_skripsi')
        if not os.path.exists(self.output_folder):
            os.makedirs(self.output_folder)
            
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_mode = rospy.Publisher('/set_mode', Int8, queue_size=10)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        self.pos_x = 0
        self.linear_vel = 0
        self.csv_writer = None
        self.start_time = 0
        self.is_recording = False

        # Parameter (SAMA PERSIS DENGAN KASAR)
        self.kecepatan_uji = 0.5
        self.durasi_jalan = 5.0
        self.durasi_diam = 2.0

        rospy.sleep(1)

    def odom_callback(self, msg):
        self.pos_x = msg.pose.pose.position.x
        self.linear_vel = msg.twist.twist.linear.x
        
        if self.is_recording:
            t_stamp = rospy.get_time() - self.start_time
            self.csv_writer.writerow([t_stamp, self.pos_x, self.linear_vel])

    def run(self):
        # 1. SET MODE HALUS (1)
        mode_msg = Int8()
        mode_msg.data = 1
        self.pub_mode.publish(mode_msg)
        print("\n[STATUS] Mode Robot: HALUS (Ramp Rate Control)")
        rospy.sleep(1)

        # 2. PERSIAPAN FILE
        filename = os.path.join(self.output_folder, 'data_halus.csv')
        f = open(filename, 'w', newline='')
        self.csv_writer = csv.writer(f)
        self.csv_writer.writerow(['Waktu(s)', 'Posisi_X(m)', 'Kecepatan(m/s)'])
        
        print(f"[ACTION] Robot akan bergerak dalam 3 detik...")
        rospy.sleep(3)
        
        # 3. MULAI JALAN & REKAM
        print("[ACTION] GO! Merekam data...")
        self.start_time = rospy.get_time()
        self.is_recording = True
        
        # Loop Jalan
        msg = Twist()
        msg.linear.x = self.kecepatan_uji
        
        start_run = rospy.get_time()
        while (rospy.get_time() - start_run) < self.durasi_jalan:
            self.pub_vel.publish(msg)
            rospy.sleep(0.1)

        # 4. STOP ROBOT
        print("[ACTION] STOP! Merekam pengereman...")
        msg.linear.x = 0.0
        
        start_stop = rospy.get_time()
        while (rospy.get_time() - start_stop) < self.durasi_diam:
            self.pub_vel.publish(msg)
            rospy.sleep(0.1)

        # 5. SELESAI
        self.is_recording = False
        f.close()
        self.pub_vel.publish(msg)
        print(f"[SELESAI] Data tersimpan di: {filename}")
        print("Pengambilan data selesai.")

if __name__ == '__main__':
    try:
        app = AmbilDataHalus()
        app.run()
    except rospy.ROSInterruptException:
        pass
