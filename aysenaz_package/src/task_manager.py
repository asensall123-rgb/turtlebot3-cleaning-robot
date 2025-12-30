#!/usr/bin/env python3
import rospy
import yaml
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from actionlib_msgs.msg import GoalStatus
import tf.transformations as tf_trans
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pyzbar import pyzbar

class CleaningManager:
    def __init__(self):
        rospy.init_node('task_manager_node')
        rospy.loginfo("Task Manager Başladı!")

        self.bridge = CvBridge()
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.current_qr = None
        self.results = {}

        # Kamera topic'i
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)

        # Mission dosyası
        self.mission_file = "/home/ayse/aysen_ws/src/aysenaz_package/config/mission.yaml"
        with open(self.mission_file, "r") as f:
            self.mission = yaml.safe_load(f)

        # Temizlik raporu dosyası
        self.report_file = "/home/ayse/aysen_ws/src/aysenaz_package/cleaning_report.txt"

    # Kamera callback
    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        decoded = pyzbar.decode(img)
        if decoded:
            self.current_qr = decoded[0].data.decode("utf-8")
        else:
            self.current_qr = None

    # Move_base ile hedefe git
    def send_goal(self, x, y, yaw, timeout=90):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = tf_trans.quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation = Quaternion(*q)
        self.client.send_goal(goal)
        finished = self.client.wait_for_result(rospy.Duration(timeout))
        if not finished:
            self.client.cancel_goal()
            return False
        return self.client.get_state() == GoalStatus.SUCCEEDED

    # QR kodunu oku ve doğrula (retry + pozisyon kaydırma)
    def try_qr(self, expected, retries=3):
        for i in range(retries):
            rospy.sleep(3)  # frame alması için bekle
            if self.current_qr == expected:
                return True
            rospy.loginfo(f"QR okunamadi, {i+1}/{retries} deneme")
            # Retry için pozisyonu biraz değiştir
            if i < retries - 1:
                rospy.loginfo("QR tekrar taranıyor, pozisyon ayarlanıyor...")
                self.send_goal(0.05, 0, 0, timeout=5)  # küçük kaydırma
        return False

    # Temizlik raporunu kaydet
    def save_report(self):
        with open(self.report_file, "w") as f:
            f.write("--- TEMİZLİK RAPORU ---\n")
            for room, status in self.results.items():
                f.write(f"{room}: {status}\n")
        rospy.loginfo(f"Temizlik raporu kaydedildi: {self.report_file}")

    # Görev akışı
    def run(self):
        for room_name in self.mission["rooms"]:
            room = self.mission[room_name]
            rospy.loginfo(f"{room_name} kapısına gidiliyor...")

           
            success = self.send_goal(room['entry_goal']['x'], room['entry_goal']['y'], room['entry_goal']['yaw'])
            if not success:
                rospy.logwarn(f"{room_name} girişine ulaşılamadı, tekrar denenecek...")
                success = self.send_goal(room['entry_goal']['x'], room['entry_goal']['y'], room['entry_goal']['yaw'])
            if not success:
                rospy.logerr(f"{room_name} girişine ulaşılamadı, oda atlandı")
                self.results[room_name] = "FAIL (Ulaşım Hatası)"
                continue

            
            if not self.try_qr(room['qr_expected'], retries=3):
                rospy.logerr(f"{room_name} QR doğrulama başarısız, okunan: {self.current_qr}")
                self.results[room_name] = "SKIPPED (QR Hatası)"
                continue
            rospy.loginfo(f"{room_name} QR doğrulandı: {self.current_qr}")

         
            for i, p in enumerate(room["cleaning_goals"]):
                rospy.loginfo(f"{room_name} temizleme noktası {i+1}")
                self.send_goal(p['x'], p['y'], p['yaw'], timeout=45)

            self.results[room_name] = "SUCCESS"

        rospy.loginfo("Tüm odalar tamamlandı!")
        self.save_report()


if __name__ == "__main__":
    try:
        manager = CleaningManager()
        manager.run()
    except rospy.ROSInterruptException:
        pass

