# turtlebot3-cleaning-robot
Autonomous cleaner robot simulation using ROS and TurtleBot3 
#Oda Bazlı Temizlik Robotu ve QR Kontrolü
**ROS Noetic | TurtleBot3 | Gazebo Simülasyonu**


##  Proje Hakkında
Bu proje, **Gazebo** ortamında çalışan bir TurtleBot3 robotunun ev içinde *oda bazlı temizlik* yapmasını sağlar. Robot:

- Ev haritasını çıkarır ve kaydeder 
- AMCL ile kendi pozisyonunu tespit eder 
- Önceden belirlenmiş odalara gider 
- QR kod tarayarak doğru oda kontrolü yapar 
- Temizlik rotalarını tamamlar 
- Sürecin sonunda rapor oluşturur 
 -Simülasyon ortamı olarak **turtlebot3_house** kullanılmıştır.

## Kullanılan Araçlar
- ROS1 Noetic 
- Gazebo 
- TurtleBot3 (waffle_pi) 
- SLAM (gmapping) 
- Navigation Stack (AMCL + move_base) 
- OpenCV ve pyzbar (QR kod çözme) 
- YAML tabanlı görev tanımı 


## Görev Akışı
1. Gazebo ortamı başlatılır 
2. SLAM ile ev haritası çıkarılır ve kaydedilir 
3. AMCL ile robot konumlandırılır 
4. Görev yöneticisi odalara hedef yollar 
5. Oda girişinde QR kod taranır 
6. QR doğrulama başarılı ise:
   - Odaya ait temizlik waypointleri tamamlanır 
7. QR doğrulama başarısız ise:
   - Oda atlanır 
8. Temizlik raporu oluşturulur 
   
## Hedeflenen Odalar
- LIVINGROOM 
- KITCHEN 
- BEDROOM 
- BATHROOM 

Her oda için
- 1 giriş noktası 
- 3–5 temizlik waypoint’i 
- 1 QR kod 

##  Kurulum ve Çalıştırma

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash

Terminal 1 – Gazebo
source ~/catkin_ws/devel/setup.bash
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/aysen_ws/src/aysenaz_package
roslaunch aysenaz_package gazebo.launch

Terminal 2 – Navigation
source ~/catkin_ws/devel/setup.bash
roslaunch turtlebot3_navigation turtlebot3_navigation.launch \
map_file:=/home/ayse/aysen_ws/src/aysenaz_package/maps/house_map.yaml

Terminal 3 – Görev Yöneticisi
source ~/catkin_ws/devel/setup.bash
rosrun aysenaz_package task_manager.py


Çıktılar

Gazebo üzerinde robotun simülasyonu
SLAM ile oluşturulmuş harita
AMCL ile robot konumu
Oda bazlı navigasyon
QR doğrulama sistemi
Temizlik waypointleri
cleaning_report.txt raporu
