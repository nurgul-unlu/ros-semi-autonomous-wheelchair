# ros-semi-autonomous-wheelchair
Bu proje, Mekatronik Mühendisliği bitirme projesi kapsamında geliştirilmiş, ROS tabanlı yarı otonom bir tekerlekli sandalye sistemidir. Sistem; kullanıcı kontrolünü korurken sensör destekli güvenlik, QR kod tabanlı konum tetikleme ve ROS tabanlı navigasyon altyapısı ile yönlendirme desteği sunmaktadır.

Amaç; özellikle kapalı ve bilinen ortamlarda hareket eden bireyler için daha güvenli, destekleyici ve akıllı bir sürüş altyapısı oluşturmaktır.

Projenin Amaçları
-
-Robotik destekli hareket sistemleri geliştirmek

-Engel algılama ile sürüş güvenliğini artırmak

-ROS altyapısı ile iç mekân navigasyon desteği sağlamak

-Gömülü sistemler ile robotik yazılımı entegre etmek

🔧 Hardware Used / Kullanılan Donanımlar
-
-ESP32 Microcontroller

-BTS7960 Motor Driver

-DC Motors (Differential Drive)

-ToF Distance Sensor (Front)

-USB Camera (QR detection)

-Wheelchair mechanical platform

💻 Software & Tools / Yazılım ve Araçlar
-
-Ubuntu 20.04

-ROS Noetic

-Gazebo Simulator

-RViz Visualization Tool

-Python (ROS Nodes)

-Arduino IDE (ESP32 firmware)

🗺 Simulation to Real Robot / Simülasyondan Gerçek Robota
-
<img width="500" height="722" alt="Ekran görüntüsü 2026-01-13 141455" src="https://github.com/user-attachments/assets/46983b7c-8f27-4115-9325-fa0e814fcdaa" /> <img width="400" height="562" alt="Ekran görüntüsü 2026-01-10 225615" src="https://github.com/user-attachments/assets/5c7c79a8-f0a1-48f7-b237-45b7f796b047" />

Sistem öncelikle Gazebo simülasyon ortamında test edilmiştir. Robot modeli SolidWorks ortamında tasarlanmış ve URDF formatına dönüştürülerek ROS’a entegre edilmiştir.


https://github.com/user-attachments/assets/5a507957-4ce0-4fc0-97d0-840062cc8dbf

Simülasyon aşamasının ardından yazılım gerçek tekerlekli sandalye platformuna aktarılmıştır. ROS üzerinden yayınlanan hız komutları (/cmd_vel) seri haberleşme aracılığıyla ESP32 mikrodenetleyicisine iletilmiş ve motor sürücüleri üzerinden tekerleklerin kontrolü sağlanmıştır. Yapılan testlerde ROS ile ESP32 arasındaki haberleşmenin kararlı ve güvenilir şekilde çalıştığı doğrulanmıştır.


📦 ROS Düğüm (Node) Yapısı /Düğüm	Görevi
--
-cmd_vel_to_esp.py ->	ROS hız komutlarını ESP32’ye iletir.

-esp_sensor_node.py -> Sensör verilerini ROS topic’lerine yayınlar.

-qr_path_planner.py	-> QR kodlara göre hedef konum belirler.

-usb_joystick_teleop.py	-> Manuel joystick kontrolünü sağlar.

Bu proje, robotik yazılım ile gömülü sistemlerin bütünleşik kullanımını gösteren kapsamlı bir mekatronik mühendisliği uygulamasıdır.

