#!/usr/bin/env python3
import rospy
import csv
import time
import os
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, Int16

class AmbilDataKasar:
    def __init__(self):
        # Inisialisasi Node
        rospy.init_node('node_ambil_data_kasar', anonymous=True)
        
        # Konfigurasi Folder Penyimpanan
        self.output_folder = os.path.expanduser('~/data_skripsi')
        if not os.path.exists(self.output_folder):
            os.makedirs(self.output_folder)
            
        # Publisher & Subscriber
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_mode = rospy.Publisher('/set_mode', Int8, queue_size=10)
        
        # Subscribe ke Ticks (Karena tidak ada /odom)
        self.sub_ticks = rospy.Subscriber('/right_ticks', Int16, self.ticks_callback)
        
        # Variabel Data
        self.current_ticks = 0
        self.last_ticks = 0
        self.last_time = 0
        self.velocity_ticks = 0.0 # Kecepatan dalam satuan Ticks/detik
        
        self.csv_writer = None
        self.file_handle = None
        self.start_rec_time = 0
        self.is_recording = False

        # Parameter Pengujian
        self.kecepatan_uji = 0.5  # m/s (Target perintah)
        self.durasi_jalan = 5.0   # Detik
        self.durasi_diam = 2.0    # Detik

        # Tunggu koneksi stabil
        print("[INIT] Menunggu sistem siap...")
        rospy.sleep(2)

    def ticks_callback(self, msg):
        # Hitung Kecepatan dari perubahan Ticks
        self.current_ticks = msg.data
        current_time = rospy.get_time()
        
        if self.last_time != 0:
            delta_time = current_time - self.last_time
            if delta_time > 0:
                # Menangani Overflow Int16 (-32768 s/d 32767)
                delta_ticks = self.current_ticks - self.last_ticks
                
                # Koreksi jika angkanya lompat (wrap around)
                if delta_ticks < -30000: 
                    delta_ticks += 65536 
                elif delta_ticks > 30000: 
                    delta_ticks -= 65536 
                
                # Hitung Kecepatan (Ticks per Detik)
                self.velocity_ticks = delta_ticks / delta_time 
                
        self.last_ticks = self.current_ticks
        self.last_time = current_time
        
        # Simpan ke CSV jika sedang merekam
        if self.is_recording and self.csv_writer:
            t_stamp = rospy.get_time() - self.start_rec_time
            # Format: Waktu, Total Ticks, Kecepatan (Ticks/s)
            self.csv_writer.writerow([t_stamp, self.current_ticks, self.velocity_ticks])

    def run(self):
        # 1. SET MODE KASAR (Kirim angka 0)
        mode_msg = Int8()
        mode_msg.data = 0 
        # Kirim beberapa kali untuk memastikan Arduino menerima
        for i in range(5):
            self.pub_mode.publish(mode_msg)
            rospy.sleep(0.1)
            
        print("\n=============================================")
        print("[STATUS] Mode Robot diset ke: KASAR (Direct)")
        print("=============================================")
        rospy.sleep(1)

        # 2. BUKA FILE CSV
        filename = os.path.join(self.output_folder, 'data_kasar.csv')
        self.file_handle = open(filename, 'w', newline='')
        self.csv_writer = csv.writer(self.file_handle)
        self.csv_writer.writerow(['Waktu(s)', 'Total_Ticks', 'Kecepatan(Ticks/s)'])
        
        print(f"[READY] Robot akan bergerak dalam 3 detik...")
        rospy.sleep(3)
        
        # 3. MULAI JALAN & REKAM
        print("[ACTION] GO! Robot Maju & Merekam...")
        self.start_rec_time = rospy.get_time()
        self.is_recording = True
        
        # Kirim perintah jalan
        msg = Twist()
        msg.linear.x = self.kecepatan_uji
        
        start_run = rospy.get_time()
        while (rospy.get_time() - start_run) < self.durasi_jalan:
            self.pub_vel.publish(msg)
            rospy.sleep(0.1) # Loop 10Hz

        # 4. STOP ROBOT
        print("[ACTION] STOP! Merekam pengereman...")
        msg.linear.x = 0.0
        
        start_stop = rospy.get_time()
        while (rospy.get_time() - start_stop) < self.durasi_diam:
            self.pub_vel.publish(msg)
            rospy.sleep(0.1)

        # 5. SELESAI
        self.is_recording = False
        self.file_handle.close()
        
        # Pastikan robot diam total
        for i in range(5):
            self.pub_vel.publish(msg)
            rospy.sleep(0.1)
            
        print(f"\n[SELESAI] Data tersimpan di: {filename}")
        print("Silakan kembalikan robot ke posisi START secara manual.")

if __name__ == '__main__':
    try:
        app = AmbilDataKasar()
        app.run()
    except rospy.ROSInterruptException:
        pass
